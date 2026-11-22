# include <iostream>
# include "rclcpp/rclcpp.hpp" // C++ ROS Library
# include <chrono> // for seconds(1)
# include "smart_vacuum_interfaces/msg/vacuum_status.hpp"
#include <algorithm>   // std::max, std::min
#include <random>      // random numbers

using smart_vacuum_interfaces::msg::VacuumStatus;


using namespace std;

class HardwareDriverNode: public rclcpp :: Node // Class inherits from the ROS node class.. the functions ect
{
    public:
    HardwareDriverNode() 
    : Node("hardware_driver_node") // Calls the constructor of the base class, registers the node in ROS under this name
    {
        //message type, topic name, queue of 10 messages
        status_pub_=create_publisher<VacuumStatus>("/vacuum/status", 10);
        
        timer_=create_wall_timer(chrono::seconds(1),[this](){publish_status();}); // call publish_status every second

        RCLCPP_INFO(get_logger(), "Hardware driver started"); // like printf
    }
    private:
    
    void publish_status() // called  by a timer once per second
    {
        uniform_real_distribution<double> uni(0.0, 1.0);   // random number between 0 and 1

        battery_ = std::max(0.0, battery_ - 0.05);     // battery drops, never below 0
        dust_ = std::min(100.0, dust_ + 0.5 + uni(gen_));       // dustbin fills, never above 100

        // create message
        VacuumStatus msg;                         
        msg.header.stamp = now();                 
        msg.header.frame_id = "base_link";
        msg.battery_level = static_cast<float>(battery_);
        msg.dustbin_status = (dust_ >= 100.0) ? VacuumStatus::DUSTBIN_FULL : VacuumStatus::DUSTBIN_OK;
        msg.cliff = uni(gen_) < 0.05;         // true in 5 % of cases
        msg.bumper = uni(gen_) < 0.10;       // true in 10 % of cases
        msg.proximity = static_cast<float>(0.05 + 1.95 * uni(gen_));  // 0.05 .. 2.0 m

        status_pub_->publish(msg);    // send ie publish

        if (battery_ < 20.0) {
        RCLCPP_WARN(get_logger(), "Battery low: %.1f %%", battery_);
        }
    };
// member variables, must be stored in the class
  rclcpp::Publisher<VacuumStatus>::SharedPtr status_pub_; // 
  rclcpp::TimerBase::SharedPtr timer_;
  double battery_{100.0};  // battery starts at 100%, then drops periodically
  double dust_{0.0};                              // dustbin fill level 0..100 %
  mt19937 gen_{random_device{}()};      // random number generator
};
int main(int argc, char ** argv){

rclcpp::init(argc, argv); // "connect" to ROS
rclcpp::spin(make_shared<HardwareDriverNode>()); // node runs until stopped (Ctrl+c)
rclcpp::shutdown(); // clean up bei ctrl+c

return 0;

}