// task_manager_node: action server for /vacuum/clean_session
// -> user sends a goal (mode + power), robot "cleans" room by room,
//    sends feedback, can be canceled ("Return to Dock"), returns stats at the end

# include "rclcpp/rclcpp.hpp" // C++ ROS Library
# include "rclcpp_action/rclcpp_action.hpp" // actions are an extra library
# include "smart_vacuum_interfaces/action/clean_session.hpp" // my action
# include "smart_vacuum_interfaces/msg/vacuum_status.hpp" // to read battery + dustbin
# include "std_msgs/msg/string.hpp" // current floor from the map loader
# include "smart_vacuum/floor_maps.h" // rooms of each floor (name must match your header file!)
# include <thread> // execute runs in its own thread
# include <atomic> // for variables that 2 threads use
# include <string>
# include <memory>

using smart_vacuum_interfaces::action::CleanSession;
using smart_vacuum_interfaces::msg::VacuumStatus;
using GoalHandle = rclcpp_action::ServerGoalHandle<CleanSession>; // shorter name, the "handle" of one goal

using namespace std;

class TaskManagerNode: public rclcpp :: Node
{
    public:
    TaskManagerNode()
    : Node("task_manager_node")
    {
        // 1) main job: offer the action
        // 3 callbacks: new goal? / cancel? / goal accepted -> start working
        action_server_ = rclcpp_action::create_server<CleanSession>(this, "/vacuum/clean_session",
            [this](const rclcpp_action::GoalUUID & uuid, shared_ptr<const CleanSession::Goal> goal){
                return handle_goal(uuid, goal);
            },
            [this](const shared_ptr<GoalHandle> goal_handle){
                return handle_cancel(goal_handle);
            },
            [this](const shared_ptr<GoalHandle> goal_handle){
                handle_accepted(goal_handle);
            });

        // 2) feedback needs the battery + stop if battery low / dustbin full -> listen to the driver
        status_sub_ = create_subscription<VacuumStatus>("/vacuum/status", 10,
            [this](const VacuumStatus::SharedPtr msg){
                battery_ = msg->battery_level;
                dustbin_full_ = (msg->dustbin_status == VacuumStatus::DUSTBIN_FULL);
            });

        // 3) added later: which floor did the map loader load? -> which rooms to clean
        // transient_local like in the map loader, otherwise they dont match
        floor_sub_ = create_subscription<std_msgs::msg::String>("/vacuum/current_floor", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg){
                floor_ = msg->data;
                RCLCPP_INFO(get_logger(), "Active floor is now %s", floor_.c_str());
            });

        RCLCPP_INFO(get_logger(), "Task manager ready: /vacuum/clean_session");
    }

    private:
    // A) someone sends a goal -> accept or reject (must answer fast!)
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, shared_ptr<const CleanSession::Goal> goal)
    {
        // values outside 0..2 make no sense
        if (goal->mode > 2 || goal->power_mode > 2) {
            RCLCPP_WARN(get_logger(), "Rejected: invalid mode or power_mode");
            return rclcpp_action::GoalResponse::REJECT;
        }
        // only one cleaning at a time
        if (busy_) {
            RCLCPP_WARN(get_logger(), "Rejected: already cleaning");
            return rclcpp_action::GoalResponse::REJECT;
        }
        // CLEAN_EXISTING_MAP needs a map -> call the service first
        if (goal->mode == CleanSession::Goal::MODE_CLEAN_EXISTING_MAP && floor_.empty()) {
            RCLCPP_WARN(get_logger(), "Rejected: no map loaded, call /vacuum/set_floor_level first");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // remember the floor for this session (no map yet -> just take Floor_1)
        if (floor_.empty()) {
            session_floor_ = "Floor_1";
        } else {
            session_floor_ = floor_;
        }
        busy_ = true;
        RCLCPP_INFO(get_logger(), "Goal accepted: mode %d, power %d", goal->mode, goal->power_mode);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // B) someone wants to cancel ("Return to Dock") -> just say ok, execute() checks it
    rclcpp_action::CancelResponse handle_cancel(const shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(get_logger(), "Cancel requested -> returning to dock");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // C) goal accepted -> start the work in a new thread
    // if execute() ran here, the node would be blocked for ~35s (no battery msgs, no cancel!)
    void handle_accepted(const shared_ptr<GoalHandle> goal_handle)
    {
        thread{[this, goal_handle](){ execute(goal_handle); }}.detach();
    }

    // D) the actual cleaning, runs in its own thread
    void execute(const shared_ptr<GoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        // make_shared doesnt work inside the class (Node has its own make_shared) -> SharedPtr + new
        CleanSession::Feedback::SharedPtr feedback(new CleanSession::Feedback());
        CleanSession::Result::SharedPtr result(new CleanSession::Result());

        // speed in m^2 per second, depends on power: ECO / STANDARD / TURBO
        double speed = 0.8;
        if (goal->power_mode == CleanSession::Goal::POWER_STANDARD) speed = 1.2;
        if (goal->power_mode == CleanSession::Goal::POWER_TURBO) speed = 1.8;

        // MAP_ONLY = only drive around, dont clean
        bool cleaning = (goal->mode != CleanSession::Goal::MODE_MAP_ONLY);

        // total area of all rooms -> for the percentage
        const auto & rooms = smart_vacuum::floor_maps().at(session_floor_).rooms;
        double total_area = 0.0;
        for (const auto & room : rooms) {
            total_area += room.area;
        }

        double done_area = 0.0;
        rclcpp::Time start = now();
        rclcpp::Rate rate(2.0); // 2 Hz -> one step every 0.5 s

        // go through the rooms one by one
        for (const auto & room : rooms) {
            double room_done = 0.0;
            while (room_done < room.area) {

                // 1) cancel? -> stop, send partial result
                if (goal_handle->is_canceling()) {
                    result->total_area_cleaned = cleaning ? done_area : 0.0;
                    result->elapsed_time = (now() - start).seconds();
                    result->completion_mode = CleanSession::Result::CANCELED;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Canceled, back to dock");
                    busy_ = false;
                    return;
                }

                // 2) battery low or dustbin full? -> abort
                if (battery_ < 15.0 || (cleaning && dustbin_full_)) {
                    result->total_area_cleaned = cleaning ? done_area : 0.0;
                    result->elapsed_time = (now() - start).seconds();
                    result->completion_mode = CleanSession::Result::ABORTED;
                    goal_handle->abort(result);
                    if (battery_ < 15.0) RCLCPP_ERROR(get_logger(), "Aborted: battery low");
                    else RCLCPP_ERROR(get_logger(), "Aborted: dustbin full");
                    busy_ = false;
                    return;
                }

                // 3) "clean" a bit (0.5 s)
                rate.sleep();
                double step = speed * 0.5;
                if (room_done + step > room.area) step = room.area - room_done; // dont go over the room size
                room_done += step;
                done_area += step;

                // 4) send feedback
                feedback->current_room = room.name;
                feedback->completion_percentage = 100.0 * done_area / total_area;
                feedback->battery_level = battery_;
                goal_handle->publish_feedback(feedback);
            }
            RCLCPP_INFO(get_logger(), "Room done: %s", room.name.c_str());
        }

        // all rooms done -> success
        result->total_area_cleaned = cleaning ? done_area : 0.0;
        result->elapsed_time = (now() - start).seconds();
        result->completion_mode = CleanSession::Result::COMPLETED;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Cleaning finished");
        busy_ = false;
    }

// member variables
  rclcpp_action::Server<CleanSession>::SharedPtr action_server_;
  rclcpp::Subscription<VacuumStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr floor_sub_;

  // atomic = safe when 2 threads use it (callback writes, execute thread reads)
  atomic<float> battery_{100.0};
  atomic<bool> dustbin_full_{false};
  atomic<bool> busy_{false};

  string floor_;          // set by the floor callback
  string session_floor_;  // copy for the running session (only execute() reads it)
};

int main(int argc, char ** argv){

rclcpp::init(argc, argv);
rclcpp::spin(make_shared<TaskManagerNode>()); // here make_shared works (main is NOT inside the class)
rclcpp::shutdown();

return 0;

}
