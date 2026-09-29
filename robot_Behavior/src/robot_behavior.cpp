#include <iostream>
#include <behaviortree_cpp/bt_factory.h>  // BT.CPP v4 on Jazzy (was behaviortree_cpp_v3 on Humble)
#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "explore_lite_msgs/msg/explore_status.hpp"

using namespace BT;

// Mission phase, published on /mission/state as a 1 Hz heartbeat (see main). The web GUI
// uses it to tell whether a mission is running at all, whichever way it was started.
enum class MissionState { kStarting, kExploring, kDetectingRacks, kVisitingRacks, kDocking, kComplete, kFailed };
std::atomic<MissionState> g_mission_state{MissionState::kStarting};

// Set once this mission's map is saved. Rack poses received before that are stale: e.g.
// the latched list of a map_operation_node still running from an earlier mission.
std::atomic<bool> g_map_saved{false};

const char* to_string(MissionState state)
{
    switch (state) {
        case MissionState::kStarting:       return "starting";
        case MissionState::kExploring:      return "exploring";
        case MissionState::kDetectingRacks: return "detecting_racks";
        case MissionState::kVisitingRacks:  return "visiting_racks";
        case MissionState::kDocking:        return "docking";
        case MissionState::kComplete:       return "complete";
        case MissionState::kFailed:         return "failed";
    }
    return "unknown";
}

// Shell prefix for the ROS commands this node spawns (explore_lite, map_saver_cli,
// map_operation_node): they need the ROS and workspace environments.
std::string ros_shell_prefix(const std::string& workspace)
{
    return "source /opt/ros/jazzy/setup.bash && source " + workspace + "/install/setup.bash && ";
}

class SetDockPose : public SyncActionNode
{
public:
    SetDockPose(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}
    
    static PortsList providedPorts()
    {
        return { OutputPort<geometry_msgs::msg::PoseStamped>("output_pose") };
    }
    
    NodeStatus tick() override
    {
        // Dock position is configurable at runtime, e.g.:
        //   ros2 run robot_Behavior robot_behavior --ros-args -p dock_x:=-2.0 -p dock_y:=0.0
        rclcpp::Node::SharedPtr ros_node;
        if (!config().blackboard->get("node", ros_node)) {
            throw std::runtime_error("ROS node not found on the blackboard!");
        }

        geometry_msgs::msg::PoseStamped dock_pose;
        dock_pose.header.frame_id = "map";
        dock_pose.header.stamp = ros_node->now();  // fresh stamp: goals must not look stale

        dock_pose.pose.position.x = ros_node->get_parameter("dock_x").as_double();
        dock_pose.pose.position.y = ros_node->get_parameter("dock_y").as_double();

        dock_pose.pose.orientation.x = 0.0;
        dock_pose.pose.orientation.y = 0.0;
        dock_pose.pose.orientation.z = 0.0;
        dock_pose.pose.orientation.w = 1.0;

        setOutput("output_pose", dock_pose);

        return NodeStatus::SUCCESS;
    }
};

class WaitUntilCharged : public StatefulActionNode
{
public:
    WaitUntilCharged(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config), current_battery_level_(0.0) {
        rclcpp::Node::SharedPtr ros_node;
        
        // This grabs the node from the blackboard
        if (!config.blackboard->get("node", ros_node)) {
            throw std::runtime_error("ROS node not found on the blackboard!");
        }

        sub_ = ros_node->create_subscription<sensor_msgs::msg::BatteryState>("/battery_level", 10,
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                this->current_battery_level_ = msg->percentage;
            });
    }
    
    static PortsList providedPorts(){ return {}; }
    
    NodeStatus onStart() override
    {
        std::cout << "WaitUntilCharged started, waiting for battery..." << std::endl;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        // BatteryState percentage is 0.0 to 1.0. 0.99 means 99% charged!
        if(current_battery_level_ >= 0.99) { 
            std::cout << "Battery is charged" << std::endl;
            return NodeStatus::SUCCESS;
        }
        else {
            std::cout << "Battery is charging... (" << current_battery_level_ << ")" << std::endl;
            return NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        std::cout << "WaitUntilCharged halted" << std::endl;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
    float current_battery_level_;
};

class Explore : public StatefulActionNode
{
public:
    Explore(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config) {
        rclcpp::Node::SharedPtr ros_node;
        
        // This grabs the node from the blackboard
        if (!config.blackboard->get("node", ros_node)) {
            throw std::runtime_error("ROS node not found on the blackboard!");
        }

        sub_ = ros_node->create_subscription<explore_lite_msgs::msg::ExploreStatus>("/explore/status", 10,
            [this](const explore_lite_msgs::msg::ExploreStatus::SharedPtr msg) {
                this->status = msg->status;
            });

        // Latched: tells map_operation_node which map to analyse, and that it belongs to
        // THIS mission (it used to read whatever map file was on disk, i.e. the previous
        // run's map when started during exploration).
        map_saved_pub_ = ros_node->create_publisher<std_msgs::msg::String>(
            "/map_saved", rclcpp::QoS(1).reliable().transient_local());

        // Pull mission parameters declared in main() so the child processes we
        // spawn match how this node was configured.
        sim_time_ = ros_node->get_parameter("use_sim_time").as_bool();
        workspace_prefix_ = ros_node->get_parameter("workspace").as_string();
        map_save_dir_ = ros_node->get_parameter("map_save_dir").as_string();
        logger_ = ros_node->get_logger().get_child("Explore");
    }
    
    static PortsList providedPorts(){ return {}; }
    
    NodeStatus onStart() override
    {
        std::cout << "[Explore] Launching explore_lite natively..." << std::endl;
        g_mission_state = MissionState::kExploring;
        // use_sim_time must MATCH the rest of the stack: 'true' in Gazebo (a /clock
        // publisher exists), 'false' on real hardware. Wrong value = TF timeouts =
        // exploration silently never starts.
        const bool sim = sim_time_.load();
        system(("bash -c '" + ros_shell_prefix(workspace_prefix_) +
                "ros2 launch explore_lite explore.launch.py use_sim_time:=" +
                (sim ? "true" : "false") + " &'").c_str());
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if(status=="returned_to_origin") {
            std::cout << "Mapping done" << std::endl;
            // map_saver_cli only waits 2 s for /map by default, which a freshly started
            // process sometimes needs just for discovery ("Failed to spin map
            // subscription"). Without a saved map map_operation_node waits forever, so
            // give it longer and retry.
            const std::string map_path = map_save_dir_ + "/my_new_map";
            const std::string save_cmd = "bash -c '" + ros_shell_prefix(workspace_prefix_) +
                "ros2 run nav2_map_server map_saver_cli -f " + map_path + " --ros-args -p save_map_timeout:=10.0 -p use_sim_time:=" +
                (sim_time_.load() ? "true" : "false") + "'";
            constexpr int kMaxSaveAttempts = 3;
            bool saved = false;
            for (int attempt = 1; attempt <= kMaxSaveAttempts && !saved; ++attempt) {
                int rc = system(save_cmd.c_str());
                saved = (rc == 0);
                if (!saved) {
                    std::cout << "[Explore] WARNING: map_saver_cli exited with code " << rc
                              << " (attempt " << attempt << "/" << kMaxSaveAttempts << ")" << std::endl;
                }
            }
            system("pkill -SIGINT -f explore.launch.py");
            if (!saved) {
                RCLCPP_ERROR(logger_, "Could not save the map after %d attempts; racks cannot be detected.",
                             kMaxSaveAttempts);
                return NodeStatus::FAILURE;
            }
            g_map_saved = true;
            std_msgs::msg::String saved_msg;
            saved_msg.data = map_path;
            map_saved_pub_->publish(saved_msg);
            return NodeStatus::SUCCESS;
        }
        else {
            return NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        system("pkill -SIGINT -f explore.launch.py");
        std::cout << "[Explore] Halted, killed explore_lite" << std::endl;
    }

private:
    rclcpp::Subscription<explore_lite_msgs::msg::ExploreStatus>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_saved_pub_;
    rclcpp::Logger logger_ = rclcpp::get_logger("Explore");
    std::string status;
    // Runtime-configurable (declared on the ROS node in main()):
    std::atomic<bool> sim_time_{false};   // --ros-args -p use_sim_time:=true  (Gazebo only!)
    std::string workspace_prefix_;        // -p workspace:=~/amr_ws
    std::string map_save_dir_;            // -p map_save_dir:=.../robot_gazebo/maps
};

// Starts the OpenCV rack detection (robot_navigation's map_operation_node) once the map
// is saved, so it no longer has to be started by hand. It picks up the saved map from the
// latched /map_saved and publishes the racks on /rack_poses for GetNextRackPose.
// Stopped again when the mission ends (see main).
class StartRackDetection : public SyncActionNode
{
public:
    StartRackDetection(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return {}; }

    // Set once this node has spawned map_operation_node, so main() only stops its own.
    static std::atomic<bool> launched;

    NodeStatus tick() override
    {
        rclcpp::Node::SharedPtr ros_node;
        if (!config().blackboard->get("node", ros_node)) {
            throw std::runtime_error("ROS node not found on the blackboard!");
        }
        g_mission_state = MissionState::kDetectingRacks;

        // Already started by hand (terminal or the web GUI)? Then it is waiting for
        // /map_saved as well; a second one would only publish a duplicate rack list.
        if (ros_node->count_publishers("/rack_poses") > 0) {
            RCLCPP_INFO(ros_node->get_logger(), "Rack detection already running, not starting another.");
            return NodeStatus::SUCCESS;
        }

        RCLCPP_INFO(ros_node->get_logger(), "Map saved, starting rack detection (map_operation_node).");
        const bool sim = ros_node->get_parameter("use_sim_time").as_bool();
        const std::string workspace = ros_node->get_parameter("workspace").as_string();
        system(("bash -c '" + ros_shell_prefix(workspace) +
                "ros2 run robot_navigation map_operation_node --ros-args -p use_sim_time:=" +
                (sim ? "true" : "false") + " &'").c_str());
        launched = true;
        return NodeStatus::SUCCESS;
    }
};
std::atomic<bool> StartRackDetection::launched{false};

class GetNextRackPose : public StatefulActionNode
{
public:
    GetNextRackPose(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config) {
        rclcpp::Node::SharedPtr ros_node;
        if (!config.blackboard->get("node", ros_node)) {
            throw std::runtime_error("ROS node not found on the blackboard!");
        }
        // A child of the node's logger goes to /rosout (a standalone
        // rclcpp::get_logger() logger does not); the web GUI ticks racks off from there.
        logger_ = ros_node->get_logger().get_child("GetNextRackPose");

        // 1. Subscribe to the official ROS 2 PoseArray message
        // transient_local (map_operation_node latches its one-shot publish): with a
        // volatile subscription the message is lost whenever it is published before
        // DDS discovery has matched us, e.g. map_operation_node started (from the web
        // GUI) after the map was saved publishes within ~1 s of starting.
        sub_ = ros_node->create_subscription<geometry_msgs::msg::PoseArray>("/rack_poses",
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                
                // Only accept the poses ONCE. Ignore subsequent messages
                // so we don't reset the list while the robot is navigating.
                if (this->has_received_poses_) {
                    return;
                }
                if (!g_map_saved) {
                    RCLCPP_INFO(logger_, "Ignoring %zu rack poses published before this mission's map was saved.",
                                msg->poses.size());
                    return;
                }

                // Clear out any old poses
                this->rack_poses_.clear();

                // Loop through the PoseArray and convert each Pose into a PoseStamped
                for (const auto& pose : msg->poses) {
                    geometry_msgs::msg::PoseStamped pose_stamped;
                    pose_stamped.header = msg->header; // Copy the header (timestamp/frame_id)
                    pose_stamped.pose = pose;          // Copy the actual coordinates
                    
                    this->rack_poses_.push_back(pose_stamped);
                }
                
                this->has_received_poses_ = true;
                g_mission_state = MissionState::kVisitingRacks;
            });
    }
    
    static PortsList providedPorts()
    {
        return { OutputPort<geometry_msgs::msg::PoseStamped>("pose_output") };
    }
    
    NodeStatus onStart() override
    {
        if (!has_received_poses_) {
            std::cout << "[GetNextRackPose] Waiting for Python node to publish rack poses..." << std::endl;
            return NodeStatus::RUNNING;
        }

        if (rack_poses_.empty()) {
            std::cout << "[GetNextRackPose] No more racks to visit!" << std::endl;
            // Reset for the next time the mission runs
            has_received_poses_ = false;
            return NodeStatus::FAILURE;
        }

        auto pose = rack_poses_.front();
        rack_poses_.erase(rack_poses_.begin());
        
        RCLCPP_INFO(logger_, "Popped a rack. Remaining: %zu", rack_poses_.size());
        setOutput("pose_output", pose);
        
        return NodeStatus::SUCCESS;
    }

    NodeStatus onRunning() override
    {
        if (has_received_poses_) {
            std::cout << "[GetNextRackPose] Received " << rack_poses_.size() << " racks from Python node!" << std::endl;
            
            if (rack_poses_.empty()) {
                has_received_poses_ = false;
                return NodeStatus::FAILURE;
            }

            auto pose = rack_poses_.front();
            rack_poses_.erase(rack_poses_.begin());
            
            RCLCPP_INFO(logger_, "Popped a rack. Remaining: %zu", rack_poses_.size());
            setOutput("pose_output", pose);
            
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        std::cout << "[GetNextRackPose] Halted" << std::endl;
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
    rclcpp::Logger logger_ = rclcpp::get_logger("GetNextRackPose");
    
    // The C++ Vector we build from the PoseArray
    std::vector<geometry_msgs::msg::PoseStamped> rack_poses_;
    bool has_received_poses_ = false;
};

class LogMissionComplete : public SyncActionNode
{
public:
    LogMissionComplete(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        std::cout << "\n========================================================\n"
                  << "✅ MISSION LOG: All rack waypoints have been completed!\n"
                  << "========================================================\n" << std::endl;
        g_mission_state = MissionState::kDocking;
        return NodeStatus::SUCCESS;
    }
};



int main(int argc, char **argv)
{
    // 1. You MUST initialize ROS 2 first!
    rclcpp::init(argc, argv);

    // 2. Create the ROS 2 node with runtime-tunable mission parameters.
    //    Defaults match the Gazebo setup; on the real robot override them, e.g.:
    //      ros2 run robot_Behavior robot_behavior --ros-args
    //        -p use_sim_time:=false -p dock_x:=-2.0 -p dock_y:=0.0
    //        -p workspace:=~/amr_ws
    //        -p map_save_dir:=~/amr_ws/src/Warehouse_AMR/robot_gazebo/maps
    //    workspace / map_save_dir / tree_file default to paths found from where this
    //    package is installed, so no user-specific paths are baked into the binary.
    auto ros_node = std::make_shared<rclcpp::Node>("bt_mission_controller");

    // <ws>/install/robot_Behavior (isolated install) or <ws>/install (merged install) -> <ws>
    std::filesystem::path install_dir = ament_index_cpp::get_package_prefix("robot_Behavior");
    if (install_dir.filename() != "install") {
        install_dir = install_dir.parent_path();
    }
    const std::string workspace_dir = install_dir.parent_path().string();

    // Declare defaults unless an override was already supplied on the CLI/yaml.
    auto declare_if_needed = [&ros_node](const std::string &name, auto default_value) {
        if (!ros_node->has_parameter(name)) {
            ros_node->declare_parameter(name, default_value);
        }
    };
    declare_if_needed("use_sim_time", false);   // true ONLY in Gazebo/Isaac!
    declare_if_needed("dock_x", -2.0);
    declare_if_needed("dock_y", 0.0);
    declare_if_needed("workspace", workspace_dir);
    // Must match map_operation_node's map_path default (robot_navigation/map_operations.py)
    declare_if_needed("map_save_dir", workspace_dir + "/src/Warehouse_AMR/robot_gazebo/maps");
    declare_if_needed("tree_file",
        ament_index_cpp::get_package_share_directory("robot_Behavior") + "/trees/warehouse_operation.xml");

    const bool sim_time = ros_node->get_parameter("use_sim_time").as_bool();
    std::cout << "[BT] use_sim_time = " << (sim_time ? "true" : "false") << std::endl;
    if (sim_time) {
        RCLCPP_WARN(ros_node->get_logger(),
                    "use_sim_time is TRUE: make sure a /clock publisher (Gazebo/Isaac) is running!");
    }

    BehaviorTreeFactory factory;
    factory.registerNodeType<SetDockPose>("SetDockPose");
    factory.registerNodeType<WaitUntilCharged>("WaitUntilCharged");
    factory.registerNodeType<Explore>("Explore");
    factory.registerNodeType<StartRackDetection>("StartRackDetection");
    factory.registerNodeType<GetNextRackPose>("GetNextRackPose");
    factory.registerNodeType<LogMissionComplete>("LogMissionComplete");
    factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_is_battery_low_condition_bt_node.so");
    factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_navigate_to_pose_action_bt_node.so");
    factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_navigate_through_poses_action_bt_node.so");

    // 3. Create a blackboard and put the ROS 2 node on it!
    auto blackboard = Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", ros_node);
    
    // Nav2 nodes explicitly require these keys to be on the blackboard!
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
    blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));
    blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", std::chrono::milliseconds(1000));

    // 4. Pass the blackboard to the tree when you create it.
    //    Path is runtime-configurable so the same binary works in Docker and on
    //    a native install (defaults match this repo's checkout).
    const std::string tree_file = ros_node->get_parameter("tree_file").as_string();

    // Heartbeat on its own thread, started before waiting for Nav2: the tick loop blocks
    // for seconds at a time (e.g. while map_saver_cli runs), and the GUI treats a silent
    // /mission/state as "no mission running".
    auto state_pub = ros_node->create_publisher<std_msgs::msg::String>("/mission/state", 10);
    auto publish_state = [&state_pub]() {
        std_msgs::msg::String msg;
        msg.data = to_string(g_mission_state.load());
        state_pub->publish(msg);
    };
    std::atomic<bool> heartbeat_running{true};
    std::thread heartbeat([&]() {
        while (heartbeat_running && rclcpp::ok()) {
            publish_state();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // The Nav2 BT nodes only wait wait_for_service_timeout (1 s) for their action server
    // when the tree is built and throw if it isn't there, so block here until Nav2 is up
    // instead of crashing when this node is started before Nav2 finishes activating.
    auto nav2_probe = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        ros_node, "navigate_to_pose");
    while (rclcpp::ok() && !nav2_probe->wait_for_action_server(std::chrono::seconds(2))) {
        if (rclcpp::ok()) {
            RCLCPP_INFO(ros_node->get_logger(), "Waiting for Nav2 (navigate_to_pose action server)...");
        }
    }
    nav2_probe.reset();
    if (!rclcpp::ok()) {
        heartbeat_running = false;
        heartbeat.join();
        return 0;
    }

    auto tree = factory.createTreeFromFile(tree_file, blackboard);

    std::cout << "Tree is starting" << std::endl;
    
    // 5. You need a loop! A tree needs to tick continuously, and ROS needs to spin continuously
    rclcpp::Rate rate(10); // 10 Hz
    while (rclcpp::ok()) {
        auto status = tree.tickOnce();   // Tick the behavior tree (BT.CPP v4: tickRoot() was renamed tickOnce())
        rclcpp::spin_some(ros_node);     // Spin ROS to receive messages!
        
        if (status == NodeStatus::SUCCESS) {
            std::cout << "\n========================================================\n"
                      << "🏁 Mission fully complete. Robot docked. Shutting down.\n"
                      << "========================================================\n" << std::endl;
            // Also on /rosout, where the web GUI picks up the mission phase.
            RCLCPP_INFO(ros_node->get_logger(), "Mission fully complete. Robot docked.");
            g_mission_state = MissionState::kComplete;
            break;
        }
        if (status == NodeStatus::FAILURE) {
            // Ticking again would restart the whole mission, exploration included.
            RCLCPP_ERROR(ros_node->get_logger(), "Mission failed. Stopping.");
            g_mission_state = MissionState::kFailed;
            break;
        }
        
        rate.sleep();
    }

    heartbeat_running = false;
    heartbeat.join();
    if (rclcpp::ok()) {
        publish_state();  // final "complete" / "failed" for the GUI
    }
    if (StartRackDetection::launched) {
        system("pkill -SIGINT -f lib/robot_navigation/map_operation_node");
    }

    std::cout << "Tree is ended" << std::endl;
    rclcpp::shutdown();
    return 0;
}
