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

#define BRAM_SIZW 8000
#define XST_FAILURE 1L

BRAM BRAM1(0,BRAM_SIZW);

#define DATA_SIZE 768

#define BRAMWAITPERIOD 100000 // microseconds 
#define TIMEPERIOD 500000 // microseconds 

using std::placeholders::_1;
using namespace std::chrono_literals;

class MainController : public rclcpp::Node
{
	public:
		MainController() : Node("main_controller") {
			RCLCPP_INFO(this->get_logger(), "Initializing MainController node");

			reentrant_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
			mutualexc_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

			rclcpp::SubscriptionOptions options_sub_reentrant;
			options_sub_reentrant.callback_group = reentrant_cb_group_;
			rclcpp::SubscriptionOptions options_sub_mutualexc;
			options_sub_mutualexc.callback_group = mutualexc_cb_group_;

			RCLCPP_INFO(this->get_logger(), "Starting starter subscription");
			start_main_subscription_ = this->create_subscription<std_msgs::msg::Int32>("start", 10, std::bind(&MainController::start_callback, this, _1), options_sub_mutualexc);
			RCLCPP_INFO(this->get_logger(), "Starting position subscription");
			goto_subscription_ = this->create_subscription<std_msgs::msg::Int32>("gotopos", 10, std::bind(&MainController::gotorequest_callback, this, _1), options_sub_reentrant);
			RCLCPP_INFO(this->get_logger(), "Starting position publisher");
			publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
			RCLCPP_INFO(this->get_logger(), "Starting client for getting position");
			client_ = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position", rmw_qos_profile_services_default, reentrant_cb_group_);
			RCLCPP_INFO(this->get_logger(), "Starting camera subscriber");
			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10,	std::bind(&MainController::onImageMsg, this, std::placeholders::_1), options_sub_reentrant);

			directory = "gray_images/" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + "/";
			mkdir(directory.c_str(), 0777);

			//gray_to_be_used_ = cv::imread("testimg.png", cv::IMREAD_GRAYSCALE);
		}

	private:
		rclcpp::CallbackGroup::SharedPtr reentrant_cb_group_;
		rclcpp::CallbackGroup::SharedPtr mutualexc_cb_group_;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_main_subscription_;
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goto_subscription_;
		rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_;
		rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr client_;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;

		int positions[7] = {310, 377, 446, 517, 584, 655, 722};
		int motorID = 0;
		int screwMotorID = 1;
		double degPrTick = 0.29;
		int screwMiddleTick = 512;
		std::string directory;
		cv::Mat gray_;
		cv::Mat gray_to_be_used_;

		bool newImageReady = false;
		int i = 0;
		
		int looking_for_label = -1;
		
		int getAngleFromImage(cv::Mat img) {
			return 0;
		}

		void start_callback(const std_msgs::msg::Int32::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
			
			looking_for_label = msg->data % 7;

			RCLCPP_INFO(this->get_logger(), "Starting main loop");
			int gotopos = 0;



			while (looking_for_label != -1) {
				const std_msgs::msg::Int32::SharedPtr msg;
				msg->data = gotopos;

				gotorequest_callback(msg);

				if (newImageReady) {
					newImageReady = false;
					RCLCPP_INFO(this->get_logger(), "NewImgReady and being processed");
					
					int noOfPixels = gray_to_be_used_.rows * gray_to_be_used_.cols;

					if (noOfPixels != DATA_SIZE) {
						RCLCPP_INFO(this->get_logger(), "No of pixels is not 768, but %d", noOfPixels);
						continue;
					}


					for(int i = 0; i < noOfPixels; i++) {
						_Float32 scaled_data = gray_to_be_used_.data[i]/255.0;

						int32_t data_as_int = *((int32_t*)&scaled_data);
						RCLCPP_INFO(this->get_logger(), "Data as float is %f and as int is %d", scaled_data, data_as_int);	
						BRAM1[i] = data_as_int;
					}
					usleep(BRAMWAITPERIOD);

					int32_t result = BRAM1[1024];

					if (result == looking_for_label) {
						RCLCPP_INFO(this->get_logger(), "Found label %d", result);
						//Code for rotating motor based on vision
						int angle = getAngleFromImage(gray_to_be_used_);
						RCLCPP_INFO(this->get_logger(), "Angle is %d", angle);

						auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
						message.id = screwMotorID;
						message.position = screwMiddleTick + int(angle/degPrTick);
						RCLCPP_INFO(this->get_logger(), "Publishing: id = '%d' , position = '%d'", message.id, message.position);
						publisher_->publish(message);
						
						usleep(TIMEPERIOD);
						
					} else {
						RCLCPP_INFO(this->get_logger(), "Did not find label %d, but %d", looking_for_label, result);
						if (++gotopos >= 7) {
							gotopos = -1;
						}
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
			gray_ = channels[0];
			

			
		}

		void gotorequest_callback(const std_msgs::msg::Int32::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);

			auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
			message.id = motorID;
			message.position = positions[msg->data % 7];
			RCLCPP_INFO(this->get_logger(), "Publishing: id = '%d' , position = '%d'", message.id, message.position);
      		publisher_->publish(message);
			
			int32_t pos = -1;
			auto request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
			request->id = motorID;

			while (pos < message.position - 3 || pos > message.position + 3) {

				while (!client_->wait_for_service(1s)) {
					if (!rclcpp::ok()) {
					RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
						continue;
					}
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
				}

				auto result = client_->async_send_request(request);
				std::future_status status = result.wait_for(1s);
				if (status == std::future_status::ready) {
					RCLCPP_INFO(this->get_logger(), "Received response");
					pos = result.get()->position;
					
				}
				
			}
			usleep(TIMEPERIOD);
			//std::string filename = directory + "img" + std::to_string(i++) + "_" + std::to_string(pos) +".png";
			cv::resize(gray_, gray_to_be_used_, cv::Size(32, 24), cv::INTER_LINEAR);
			newImageReady = true;
			//cv::imwrite(filename, gray_);


		}
		


};

int main(int argc, char *argv[])
{
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);
	rclcpp::init(argc, argv);
    auto client_node = std::make_shared<MainController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);

    RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();
    RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
	return 0;
}
