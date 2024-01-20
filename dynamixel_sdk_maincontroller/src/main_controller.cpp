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


Mat getCenterImage(Mat& img)
{
    Mat blur;
    GaussianBlur(img, blur, Size(3,3),0);

    //Mat gray;
    //cvtColor(blur, gray, COLOR_BGR2GRAY);

    Mat mask = cv::Mat::zeros(blur.size(), blur.type());
    Mat dstImage = cv::Mat::zeros(blur.size(), blur.type());
    circle(mask, cv::Point(mask.cols/2, mask.rows/1.82), 110, cv::Scalar(255, 0, 0), -1, 8, 0);
    blur.copyTo(dstImage, mask);

    return dstImage;
}


Mat getEdgeDetect(Mat& img, int StartThresh, int EndThresh, int dilationSize)
{
    Mat Edges;
    Canny(img, Edges, StartThresh, EndThresh);

    Mat DialtedImg;

    Mat kernel = getStructuringElement(MORPH_RECT, Size(dilationSize, dilationSize));
    dilate(Edges, DialtedImg, kernel, Point(-1,-1));

    return DialtedImg;
}

float processContours(const Mat& CannyEdge, Mat& image)
{
    float angle;
    vector<vector<Point>> contours;
    findContours(CannyEdge, contours, RETR_LIST, CHAIN_APPROX_NONE);

    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);

        if (area < 8000 || 23000 < area) continue;

        drawContours(image, contours, static_cast<int>(i), Scalar(0, 0, 255), 2);

        RotatedRect RotRec;
        RotRec = minAreaRect(contours[i]);

        // Get the vertices of the rotated rectangle
        Point2f vertices[4];
        RotRec.points(vertices);

        // Draw the rotated rectangle using the vertices
        for (int j = 0; j < 4; j++) {
            line(image, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0), 2);
        }

        // Get the center and angle of the rotated rectangle
        //Point2f center = RotRec.center;
        angle = RotRec.angle;
        break;
    }
    return -angle;
}





class MainController : public rclcpp::Node
{
	public:
		MainController() : Node("main_controller") {
			RCLCPP_INFO(this->get_logger(), "Initializing MainController node");

			mReentrantCBGgroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
			mMutualexcCBGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

			rclcpp::SubscriptionOptions options_sub_reentrant;
			options_sub_reentrant.callback_group = mReentrantCBGgroup;
			rclcpp::SubscriptionOptions options_sub_mutualexc;
			options_sub_mutualexc.callback_group = mMutualexcCBGroup;

			RCLCPP_INFO(this->get_logger(), "Starting starter subscription");
			mStartMainSubscriber = this->create_subscription<std_msgs::msg::Int32>("start", 10, std::bind(&MainController::start_callback, this, _1), options_sub_mutualexc);
			RCLCPP_INFO(this->get_logger(), "Starting position subscription");
			mGotoSubscriber = this->create_subscription<std_msgs::msg::Int32>("gotopos", 10, std::bind(&MainController::gotorequest_callback, this, _1), options_sub_reentrant);
			RCLCPP_INFO(this->get_logger(), "Starting position publisher");
			mSetMotorPosPublisher = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
			RCLCPP_INFO(this->get_logger(), "Starting gotopos publisher");
			mGotoPublisher = this->create_publisher<std_msgs::msg::Int32>("gotopos", 10);
			RCLCPP_INFO(this->get_logger(), "Starting client for getting position");
			mGetPositionClient = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position", rmw_qos_profile_services_default, mReentrantCBGgroup);
			RCLCPP_INFO(this->get_logger(), "Starting camera subscriber");
			mCameraSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10,	std::bind(&MainController::onImageMsg, this, std::placeholders::_1), options_sub_reentrant);

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

		bool mNewImageFound = false;
		int i = 0;
		
		int mLookingForLabel = -1;
		
		int getAngleFromImage(cv::Mat img) {

			Mat centerImg = getCenterImage(img);
			Mat edgeImg = getEdgeDetect(centerImg, 50, 150, 3);
			float angle = processContours(edgeImg, centerImg);

			return angle;
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

					int32_t result = BRAM1[127];

					if (result == mLookingForLabel) {
						RCLCPP_INFO(this->get_logger(), "Found label %d", result);
						//Code for rotating motor based on vision
						int angle = getAngleFromImage(mGrayToBeUsed);
						RCLCPP_INFO(this->get_logger(), "Angle is %d", angle);

						auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
						message.id = mScrewMotor;
						message.position = mScrewMiddleTick + int(angle/mDegPrTick);
						RCLCPP_INFO(this->get_logger(), "Publishing: id = '%d' , position = '%d'", message.id, message.position);
						mSetMotorPosPublisher->publish(message);
						
						usleep(GOTOWAITTIMEPERIOD);

					} else {
						RCLCPP_INFO(this->get_logger(), "Did not find label %d, but %d", mLookingForLabel, result);
					}
					if (++gotopos >= 7) {
						gotopos = -1;
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
			RCLCPP_INFO(this->get_logger(), "Going to pos: '%d'", msg->data);

			auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
			message.id = mTurnMotorID;
			message.position = mPositions[msg->data % 7];
			RCLCPP_INFO(this->get_logger(), "Publishing to motor: id = '%d' , position = '%d'", message.id, message.position);
      		mSetMotorPosPublisher->publish(message);
			
			int32_t pos = -1;
			auto request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
			request->id = mTurnMotorID;

			while (pos < message.position - 3 || pos > message.position + 3) {

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
			RCLCPP_INFO(this->get_logger(), "Reached pos %d", msg->data);
			usleep(TIMEPERIOD);
			//std::string filename = mDirectoryForImages + "img" + std::to_string(i++) + "_" + std::to_string(pos) +".png";
			cv::resize(mGray, mGrayToBeUsed, cv::Size(32, 24), cv::INTER_LINEAR);
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
