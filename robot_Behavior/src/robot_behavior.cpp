#include <iostream>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "explore_lite_msgs/msg/explore_status.hpp"

using namespace BT;

class SetDockPose : public SyncActionNode
{
public:
    SetDockPose(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
    
    static PortsList providedPorts()
    {
        return { OutputPort<geometry_msgs::msg::PoseStamped>("output_pose") };
    }
    
    NodeStatus tick() override
    {
        geometry_msgs::msg::PoseStamped dock_pose;
        dock_pose.header.frame_id = "map"; 
        
        dock_pose.pose.position.x = -2.0;
        dock_pose.pose.position.y = 0.0;
        
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
    WaitUntilCharged(const std::string& name, const NodeConfiguration& config) : StatefulActionNode(name, config), current_battery_level_(0.0) {
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
    Explore(const std::string& name, const NodeConfiguration& config) : StatefulActionNode(name, config) {
        rclcpp::Node::SharedPtr ros_node;
        
        // This grabs the node from the blackboard
        if (!config.blackboard->get("node", ros_node)) {
            throw std::runtime_error("ROS node not found on the blackboard!");
        }

        sub_ = ros_node->create_subscription<explore_lite_msgs::msg::ExploreStatus>("/explore/status", 10,
            [this](const explore_lite_msgs::msg::ExploreStatus::SharedPtr msg) {
                this->status = msg->status;
            });
    }
    
    static PortsList providedPorts(){ return {}; }
    
    NodeStatus onStart() override
    {
        std::cout << "[Explore] Launching explore_lite natively..." << std::endl;
        system("bash -c 'source /opt/ros/humble/setup.bash && source /home/aadil/AMR_ws/install/setup.bash && ros2 launch explore_lite explore.launch.py use_sim_time:=true &'");
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if(status=="returned_to_origin") { 
            std::cout << "Mapping done" << std::endl;
            system("bash -c 'source /opt/ros/humble/setup.bash && source /home/aadil/AMR_ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f /home/aadil/AMR_ws/src/Warehouse_AMR/robot_gazebo/maps/my_new_map'");
            system("pkill -SIGINT -f explore.launch.py");
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
    std::string status;
};

class GetNextRackPose : public StatefulActionNode
{
public:
    GetNextRackPose(const std::string& name, const NodeConfiguration& config) : StatefulActionNode(name, config) {
        rclcpp::Node::SharedPtr ros_node;
        if (!config.blackboard->get("node", ros_node)) {
            throw std::runtime_error("ROS node not found on the blackboard!");
        }

        // 1. Subscribe to the official ROS 2 PoseArray message
        sub_ = ros_node->create_subscription<geometry_msgs::msg::PoseArray>("/rack_poses", 10,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                
                // Only accept the poses ONCE. Ignore subsequent messages
                // so we don't reset the list while the robot is navigating.
                if (this->has_received_poses_) {
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
        
        RCLCPP_INFO(rclcpp::get_logger("GetNextRackPose"), "Popped a rack. Remaining: %zu", rack_poses_.size());
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
            
            RCLCPP_INFO(rclcpp::get_logger("GetNextRackPose"), "Popped a rack. Remaining: %zu", rack_poses_.size());
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
    
    // The C++ Vector we build from the PoseArray
    std::vector<geometry_msgs::msg::PoseStamped> rack_poses_;
    bool has_received_poses_ = false;
};

class LogMissionComplete : public SyncActionNode
{
public:
    LogMissionComplete(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        std::cout << "\n========================================================\n"
                  << "✅ MISSION LOG: All rack waypoints have been completed!\n"
                  << "========================================================\n" << std::endl;
        return NodeStatus::SUCCESS;
    }
};



int main(int argc, char **argv)
{
    // 1. You MUST initialize ROS 2 first!
    rclcpp::init(argc, argv);
    
    // 2. Create the ROS 2 node
    auto ros_node = std::make_shared<rclcpp::Node>("bt_mission_controller");

    BehaviorTreeFactory factory;
    factory.registerNodeType<SetDockPose>("SetDockPose");
    factory.registerNodeType<WaitUntilCharged>("WaitUntilCharged");
    factory.registerNodeType<Explore>("Explore");
    factory.registerNodeType<GetNextRackPose>("GetNextRackPose");
    factory.registerNodeType<LogMissionComplete>("LogMissionComplete");
    factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_is_battery_low_condition_bt_node.so");
    factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_navigate_to_pose_action_bt_node.so");
    factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_navigate_through_poses_action_bt_node.so");

    // 3. Create a blackboard and put the ROS 2 node on it!
    auto blackboard = Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", ros_node);
    
    // Nav2 nodes explicitly require these keys to be on the blackboard!
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
    blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));
    blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", std::chrono::milliseconds(1000));

    // 4. Pass the blackboard to the tree when you create it
    auto tree = factory.createTreeFromFile("/home/aadil/AMR_ws/src/Warehouse_AMR/robot_Behavior/trees/warehouse_operation.xml", blackboard);

    std::cout << "Tree is starting" << std::endl;
    
    // 5. You need a loop! A tree needs to tick continuously, and ROS needs to spin continuously
    rclcpp::Rate rate(10); // 10 Hz
    while (rclcpp::ok()) {
        auto status = tree.tickRoot();   // Tick the behavior tree
        rclcpp::spin_some(ros_node);     // Spin ROS to receive messages!
        
        if (status == NodeStatus::SUCCESS) {
            std::cout << "\n========================================================\n"
                      << "🏁 Mission fully complete. Robot docked. Shutting down.\n"
                      << "========================================================\n" << std::endl;
            break;
        }
        
        rate.sleep();
    }

    std::cout << "Tree is ended" << std::endl;
    rclcpp::shutdown();
    return 0;
}
