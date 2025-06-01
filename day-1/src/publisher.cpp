#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Publisher : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr timer;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  	size_t count;
  	
  	void pubStr()
  	{
  		auto msg = std_msgs::msg::String();
		msg.data = "hello" + std::to_string(this->count++);
		RCLCPP_INFO(this->get_logger(), "publishing: '%s'", msg.data.c_str());
		pub->publish(msg);
  	}

public:
	Publisher() : Node("pub_cpp"), count(0)
	{
		pub = this->create_publisher<std_msgs::msg::String>("/proficiency", 10);
		timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Publisher::pubStr, this));
		RCLCPP_INFO(this->get_logger(), "cpp publisher initiated");
    };
    
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Publisher>());
	rclcpp::shutdown();
	return 0;
}
