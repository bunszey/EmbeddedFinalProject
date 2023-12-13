#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <filesystem>
#include <sys/stat.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

class MainController : public rclcpp::Node
{
	public:
		MainController() : Node("main_controller") {
			RCLCPP_INFO(this->get_logger(), "Initializing MainController node");

			RCLCPP_INFO(this->get_logger(), "Starting position subscription");
			subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      			"gotopos", 10, std::bind(&MainController::gotorequest_callback, this, _1));
			RCLCPP_INFO(this->get_logger(), "Starting position publisher");
			publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
			RCLCPP_INFO(this->get_logger(), "Starting client for getting position");
			client_ = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");
			RCLCPP_INFO(this->get_logger(), "Starting camera subscriber");
			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
					"/image_raw",
					10,
					std::bind(&MainController::onImageMsg, this, std::placeholders::_1)
			);

			directory = std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + "/";
			mkdir(directory.c_str(), 0777);
		}

	private:
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
		rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_;
		rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr client_;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;

		int positions[7] = {307, 375, 442, 512, 582, 650, 717};
		int motorID = 0;
		std::string directory;
		cv::Mat gray_;
		int i = 0;

        void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) 
		{
			RCLCPP_INFO(this->get_logger(), "Received image!");

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

			while (pos < message.position - 5 || pos > message.position + 5) {
			
				while (!client_->wait_for_service(1s)) {
					if (!rclcpp::ok()) {
					RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
						continue;
					}
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
				}

				auto result = client_->async_send_request(request);
				// Wait for the result.
				pos = result.get()->position;
			}

			std::string filename = directory + "img" + std::to_string(i++) + "_" + std::to_string(pos) +".png";
			cv::imwrite(filename, gray_);


		}
		


};

int main(int argc, char *argv[])
{
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<MainController>());

	rclcpp::shutdown();
	return 0;
}
