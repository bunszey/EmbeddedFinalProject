#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <filesystem>
#include <sys/stat.h>
#include <unistd.h> 

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

#include "BRAM-uio-driver/src/bram_uio.h"

#define BRAM_SIZW 16384
#define XST_FAILURE 1L

BRAM BRAM1(0,BRAM_SIZW);

#define DATA_SIZE 768

#define BRAMWAITPERIOD 100000 // microseconds 
#define TIMEPERIOD 1000000 // microseconds 
#define GOTOWAITTIMEPERIOD 3000000 // microseconds 

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace cv;
using namespace std;


class MainController : public rclcpp::Node
{
	public:
		MainController() : Node("main_controller") {
			RCLCPP_INFO(this->get_logger(), "Initializing MainController node");

			mReentrantCBGgroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
			mMutualexcCBGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

			rclcpp::SubscriptionOptions optionsSubscriberReentrant;
			optionsSubscriberReentrant.callback_group = mReentrantCBGgroup;
			rclcpp::SubscriptionOptions optionsSubscriberMutualExclusive;
			optionsSubscriberMutualExclusive.callback_group = mMutualexcCBGroup;

			RCLCPP_INFO(this->get_logger(), "Starting starter subscription");
			mStartMainSubscriber = this->create_subscription<std_msgs::msg::Int32>("start", 10, std::bind(&MainController::start_callback, this, _1), optionsSubscriberMutualExclusive);
			RCLCPP_INFO(this->get_logger(), "Starting position subscription");
			mGotoSubscriber = this->create_subscription<std_msgs::msg::Int32>("gotopos", 10, std::bind(&MainController::gotorequest_callback, this, _1), optionsSubscriberReentrant);
			RCLCPP_INFO(this->get_logger(), "Starting position publisher");
			mSetMotorPosPublisher = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
			RCLCPP_INFO(this->get_logger(), "Starting gotopos publisher");
			mGotoPublisher = this->create_publisher<std_msgs::msg::Int32>("gotopos", 10);
			RCLCPP_INFO(this->get_logger(), "Starting client for getting position");
			mGetPositionClient = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position", rmw_qos_profile_services_default, mReentrantCBGgroup);
			RCLCPP_INFO(this->get_logger(), "Starting camera subscriber");
			mCameraSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10,	std::bind(&MainController::onImageMsg, this, std::placeholders::_1), optionsSubscriberReentrant);

			mDirectoryForImages = "gray_images/" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + "/";
			mkdir(mDirectoryForImages.c_str(), 0777);

		}

	private:
		rclcpp::CallbackGroup::SharedPtr mReentrantCBGgroup;
		rclcpp::CallbackGroup::SharedPtr mMutualexcCBGroup;
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mStartMainSubscriber;
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mGotoSubscriber;
		rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr mSetMotorPosPublisher;
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mGotoPublisher;
		rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr mGetPositionClient;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mCameraSubscriber;

		int mPositions[7] = {310, 377, 446, 517, 584, 655, 722};
		int mTurnMotorID = 0;
		int mScrewMotor = 1;
		double mDegPrTick = 0.29;
		int mScrewMiddleTick = 512;
		std::string mDirectoryForImages;
		cv::Mat mGray;
		cv::Mat mGrayToBeUsed;
		cv::Mat mGrayToBeUsedFullSize;

		bool mNewImageFound = false;
		int i = 0;
		
		int mLookingForLabel = -1;
		
		int moveMotor(int motorID, int position){
			int32_t pos = -1;
			auto request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
			request->id = motorID;

			while (pos < position - 3 || pos > position + 3) {

				while (!mGetPositionClient->wait_for_service(1s)) {
					if (!rclcpp::ok()) {
					RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
						continue;
					}
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
				}

				auto result = mGetPositionClient->async_send_request(request);
				std::future_status status = result.wait_for(1s);
				if (status == std::future_status::ready) {
					//RCLCPP_INFO(this->get_logger(), "Received response");
					pos = result.get()->position;
					
				}
				
			}
			RCLCPP_INFO(this->get_logger(), "Motor %d reached pos %d", motorID, pos);
			usleep(TIMEPERIOD);

			return 0;
		}

		int getAngleFromImage(cv::Mat imageInput, int screwType = 0) {
			// define constants
			int cannyDiff = 0;
			int houghThreshold = 50;
			// define variables
			int maskRadius;
			int cannyThreshold;
			int houghMinLineLength;
			int houghMaxLineGap;
			double angle = 0;

			switch (screwType) {
				case 0:
					// Hex
					maskRadius = 100;
					cannyThreshold = 50;
					houghMinLineLength = 10;
					houghMaxLineGap = 10;
					angle += 30;
					break;
				case 1:
					// Slotted hex
					maskRadius = 60;
					cannyThreshold = 30;
					houghMinLineLength = 10;
					houghMaxLineGap = 10;
					break;
				case 3:
					// Screw
					maskRadius = 60;
					cannyThreshold = 30;
					houghThreshold = 50;
					houghMinLineLength = 30;
					houghMaxLineGap = 40;
					break;
				case 4:
					// Square
					maskRadius = 100;
					cannyThreshold = 50;
					houghMinLineLength = 70;
					houghMaxLineGap = 10;
					break;
				default:
					RCLCPP_INFO(this->get_logger(), "Invalid screw type chosen for vision");
					return 0;
			}

			// Mask to only save circle of image
			Mat mask = cv::Mat::zeros(imageInput.size(), imageInput.type());
			Mat masked = cv::Mat::zeros(imageInput.size(), imageInput.type());
			circle(mask, cv::Point(mask.cols/2, mask.rows/1.71), maskRadius, cv::Scalar(255, 0, 0), -1, 8, 0);
			imageInput.copyTo(masked, mask);

			// Canny edge detection
			Mat edges;
			Canny(masked, edges, cannyThreshold, cannyThreshold+cannyDiff); 

			// Mask to remove outer edges from canny edge detecting circle mask from above
			Mat mask2 = cv::Mat::zeros(edges.size(), edges.type());
			Mat masked2 = cv::Mat::zeros(edges.size(), edges.type());
			circle(mask2, cv::Point(mask2.cols/2, mask2.rows/1.71), maskRadius-1, cv::Scalar(255, 0, 0), -1, 8, 0);
			edges.copyTo(masked2, mask2);
			masked2.copyTo(edges);

			// Create a vector to store lines of the image
			vector<Vec4i> lines;
			
			// If no lines are found, decrease the threshold until lines are found
			while (lines.size() == 0 && houghThreshold > 0) {
				// Apply Hough Transform
				HoughLinesP(edges, lines, 1, CV_PI/180, houghThreshold, houghMinLineLength, houghMaxLineGap);
				houghThreshold--;
			}
			// If no lines are found, return 0
			if (lines.size() == 0) {
				RCLCPP_INFO(this->get_logger(), "No lines found from Hough Transform");
				return 0;
			}
			
			// find the line with least difference on the y axis
			int minDiff = 1000;
			int minIndex = 0;
			for (size_t i=0; i<lines.size(); i++) {
				int diff = abs(lines[i][1] - lines[i][3]);
				if (diff < minDiff) {
					minDiff = diff;
					minIndex = i;
				}
			}

			// Find angle of line in degrees
			angle += atan2(lines[minIndex][3] - lines[minIndex][1], lines[minIndex][2] - lines[minIndex][0]) * 180.0 / CV_PI;

			return -angle;
		}


		void start_callback(const std_msgs::msg::Int32::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Starting looking for label: '%d'", msg->data);
			
			mLookingForLabel = msg->data % 5;

			RCLCPP_INFO(this->get_logger(), "Starting main loop");
			int gotopos = 0;
			int oldGotopos = -1;


			while (mLookingForLabel != -1) {
				if (oldGotopos != gotopos){
					std_msgs::msg::Int32 msg;
					msg.data = gotopos;
					mGotoPublisher->publish(msg);
					oldGotopos = gotopos;
				}
				

				if (mNewImageFound) {
					mNewImageFound = false;
					RCLCPP_INFO(this->get_logger(), "NewImgReady and being processed");
					cv::imwrite("img"+to_string(gotopos)+".png", mGrayToBeUsed);
					cv::imwrite("img"+to_string(gotopos)+"fullsize.png", mGrayToBeUsedFullSize);

					int noOfPixels = mGrayToBeUsed.rows * mGrayToBeUsed.cols;

					if (noOfPixels != DATA_SIZE) {
						RCLCPP_INFO(this->get_logger(), "No of pixels is not 768, but %d", noOfPixels);
						continue;
					}


					for(int i = 0; i < noOfPixels; i++) {
						_Float32 scaledData = mGrayToBeUsed.data[i]/255.0;

						int32_t dataAsInt = *((int32_t*)&scaledData);
						//RCLCPP_INFO(this->get_logger(), "Data as float is %f and as int is %d", scaledData, dataAsInt);	
						BRAM1[i] = dataAsInt;
					}
					usleep(BRAMWAITPERIOD);

					int32_t result = BRAM1[128];

					if (result == mLookingForLabel) {
						//Code for rotating motor based on vision
						int angle = getAngleFromImage(mGrayToBeUsedFullSize, mLookingForLabel);
						RCLCPP_INFO(this->get_logger(), "Found label %d, Angle is %d", result, angle);

						auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
						message.id = mScrewMotor;
						message.position = mScrewMiddleTick + int(angle/mDegPrTick);
						RCLCPP_INFO(this->get_logger(), "Publishing: id = '%d' , position = '%d'", message.id, message.position);
						mSetMotorPosPublisher->publish(message);
						
						moveMotor(message.id, message.position);

					} else {
						RCLCPP_INFO(this->get_logger(), "Did not find label %d, but %d", mLookingForLabel, result);
					}
					if (++gotopos >= 7) {
						gotopos = -1;
						mLookingForLabel = -1;
					}

				}
			}
			RCLCPP_INFO(this->get_logger(), "Main loop ended");
		}

        void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) 
		{
			//RCLCPP_INFO(this->get_logger(), "Received image!");

			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
			cv::Mat img = cv_ptr->image;
			std::vector<cv::Mat> channels(2);
			cv::split(img, channels);
			mGray = channels[0];
			

			
		}

		void gotorequest_callback(const std_msgs::msg::Int32::SharedPtr msg) {
			if (msg->data < 0 || msg->data > 6){
				return;
			}

			auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
			message.id = mTurnMotorID;
			message.position = mPositions[msg->data % 7];
			RCLCPP_INFO(this->get_logger(), "Going to pos: '%d', Publishing to motor: id = '%d' , position = '%d'", msg->data, message.id, message.position);
      		mSetMotorPosPublisher->publish(message);
			
			moveMotor(mTurnMotorID, message.position);

			//std::string filename = mDirectoryForImages + "img" + std::to_string(i++) + "_" + std::to_string(pos) +".png";
			cv::resize(mGray, mGrayToBeUsed, cv::Size(32, 24), cv::INTER_LINEAR);
			mGrayToBeUsedFullSize = mGray;
			mNewImageFound = true;
			//cv::imwrite(filename, mGray);


		}
		


};

int main(int argc, char *argv[])
{
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);
	rclcpp::init(argc, argv);
    auto clientNode = std::make_shared<MainController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(clientNode);

    RCLCPP_INFO(clientNode->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();
    RCLCPP_INFO(clientNode->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
	return 0;
}
