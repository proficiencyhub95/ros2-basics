#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Subscriber : public rclcpp::Node
{
private:
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

	void sub_callback(const std_msgs::msg::String &msg)
	{
		RCLCPP_INFO(this->get_logger(), "heard: '%s'", msg.data.c_str());
	}

public:
	Subscriber() : Node("cpp_sub")
	{
		sub = this->create_subscription<std_msgs::msg::String>("proficiency", 10, std::bind(&Subscriber::sub_callback, this, std::placeholders::_1));
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Subscriber>());
	rclcpp::shutdown();
	return 0;
}
