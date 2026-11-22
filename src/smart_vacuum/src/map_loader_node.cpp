// map_loader_node: service server for /vacuum/set_floor_level
// -> user picks a floor, node loads the map + sets the start pose, answers success/fail

# include "rclcpp/rclcpp.hpp" // C++ ROS Library
# include "smart_vacuum_interfaces/srv/set_floor_level.hpp" // my service (generated from the .srv)
# include "smart_vacuum/floor_maps.h" // floor data (my own header)
# include "std_msgs/msg/string.h" // current_floor topic (added later)
# include <string>
# include <vector>
# include <memory>
# include <cstdio> // snprintf

# include "std_msgs/msg/string.hpp" // for the current_floor topic
using smart_vacuum_interfaces::srv::SetFloorLevel;

using namespace std;

class MapLoaderNode: public rclcpp :: Node
{
    public:
    MapLoaderNode()
    : Node("map_loader_node")
    {
        //1) main job: offer the service
        // service type, service name, callback that runs when a request comes in
        service_=create_service<SetFloorLevel>("/vacuum/set_floor_level",
            [this](const shared_ptr<SetFloorLevel::Request> request,
                   shared_ptr<SetFloorLevel::Response> response){
                on_set_floor(*request, *response); // just forward to my function, easier to read
            });

        // 2) HW says "sets initial pose parameters" -> declare them, 0.0 until a floor is loaded
        declare_parameter("initial_pose_x", 0.0);
        declare_parameter("initial_pose_y", 0.0);
        declare_parameter("initial_pose_theta", 0.0);

        // 3) added later: task manager also needs to know which floor is active -> topic
        //transient_local = late subscribers still get the last msg (like "latched" in ROS1)
        floor_pub_=create_publisher<std_msgs::msg::String>("/vacuum/current_floor", rclcpp::QoS(1).transient_local());

        RCLCPP_INFO(get_logger(), "Map loader ready: /vacuum/set_floor_level");
    }

    private:
    // called every time someone calls the service
    // request = filled by the client, response = empty, I fill it->ROS sends it back after return
    void on_set_floor(const SetFloorLevel::Request & request, SetFloorLevel::Response & response)
    {
        // a) what does the user want
        string floor_id = request.floor_id;
        RCLCPP_INFO(get_logger(), "Request: load map for %s", floor_id.c_str());

        // b) does this floor even exist? if not -> fail + say why (no crash!)
        const auto & maps = smart_vacuum::floor_maps();
        auto it = maps.find(floor_id);
        if (it == maps.end()) {
            response.success = false;
            response.info = "Unknown floor " + floor_id + ", valid: Floor_1, Floor_2";
            RCLCPP_ERROR(get_logger(), "%s", response.info.c_str());
            return; // response is sent anyway
        }
        const smart_vacuum::FloorMap & info = it->second; // it->first = name, it->second = data

        // c) "load the grid map into memory"
        current_map_ = smart_vacuum::build_grid(info.width, info.height);

        // d) set the initial pose params (check with: ros2 param get /map_loader_node initial_pose_x)
        set_parameters({
            rclcpp::Parameter("initial_pose_x", info.init_x),
            rclcpp::Parameter("initial_pose_y", info.init_y),
            rclcpp::Parameter("initial_pose_theta", info.init_theta)});

        // e) tell the others which floor is active now
        std_msgs::msg::String floor_msg;
        floor_msg.data = floor_id;
        floor_pub_->publish(floor_msg);

        // f) fill the response: success + informative msg + the pose
        char text[256]; // snprintf = printf but into a string
        snprintf(text, sizeof(text), "Map %s loaded: %dx%d cells, %zu cells in memory, initial pose = (%.2f, %.2f, %.2f)",
            floor_id.c_str(), info.width, info.height, current_map_.size(), info.init_x, info.init_y, info.init_theta);

        response.success = true;
        response.info = text;
        response.initial_x = static_cast<float>(info.init_x);     // double -> float32
        response.initial_y = static_cast<float>(info.init_y);
        response.initial_phi = static_cast<float>(info.init_theta);

        RCLCPP_INFO(get_logger(), "%s", text);
    }

// member variables
  rclcpp::Service<SetFloorLevel>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr floor_pub_;
  vector<int8_t> current_map_; // the map "in memory"
};

int main(int argc, char ** argv){

rclcpp::init(argc, argv);
rclcpp::spin(make_shared<MapLoaderNode>());
rclcpp::shutdown();

return 0;

}