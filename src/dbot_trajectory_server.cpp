#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <dbot_interfaces/action/dbot_trajectory.hpp>
#include <dbot_trajectory_player/visibility_control.h>
#include <dbot_trajectory_player/job.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>

namespace dbot_trajectory_player
{

using moveit_msgs::action::MoveGroupSequence;
using GoalHandleMoveGroupSequence = rclcpp_action::ClientGoalHandle<MoveGroupSequence>;

class DbotTrajectoryActionServer : public rclcpp::Node
{
public:
    using DbotTrajectory = dbot_interfaces::action::DbotTrajectory;
    using DbotTrajectoryGoalHandle = rclcpp_action::ServerGoalHandle<DbotTrajectory>;

    DBOT_TRAJECTORY_PLAYER_CPP_PUBLIC
    explicit DbotTrajectoryActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("dbot_trajectory_action_server_node", options)
    {
        // Dbot Trajectory Action Server
        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<DbotTrajectory>(
            this, 
            "dbot_trajectory_action",
            std::bind(&DbotTrajectoryActionServer::handle_goal, this, _1, _2),
            std::bind(&DbotTrajectoryActionServer::handle_cancel, this, _1),
            std::bind(&DbotTrajectoryActionServer::handle_accepted, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Action server running: dbot_trajectory_action");

        // Move Group Sequence Action Client
        move_group_seq_client_ = rclcpp_action::create_client<MoveGroupSequence>(this, "/sequence_move_group");
    }

private:
    /**
     * @brief 
     * 
     */
    rclcpp_action::Server<DbotTrajectory>::SharedPtr action_server_;

    /**
     * @brief 
     * 
     */
    rclcpp_action::Client<MoveGroupSequence>::SharedPtr move_group_seq_client_;    

    /**
     * @brief 
     * 
     * @param uuid 
     * @param goal 
     * @return rclcpp_action::GoalResponse 
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DbotTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with job: \n'%s'", goal->job.c_str());
        (void)uuid;
        
        // Guard
        if(!Job::is_valid(goal->job))
        {
            RCLCPP_INFO(this->get_logger(), "Invalid job format. Rejecting request. . .");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Received goal request accepted and will be executed");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief 
     * 
     * @param goal_handle 
     * @return rclcpp_action::CancelResponse 
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<DbotTrajectoryGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief 
     * 
     * @param goal_handle 
     */
    void handle_accepted(const std::shared_ptr<DbotTrajectoryGoalHandle> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&DbotTrajectoryActionServer::execute, this, _1), goal_handle}.detach();
    }

    /**
     * @brief 
     * 
     * @param goal_handle 
     */
    void execute(const std::shared_ptr<DbotTrajectoryGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<DbotTrajectory::Feedback>();
        auto result = std::make_shared<DbotTrajectory::Result>();
        rclcpp::Rate loop_rate(1);
        RCLCPP_INFO(this->get_logger(), "Goal: %s", goal->job.c_str());
        
        // Parse
        RCLCPP_INFO(this->get_logger(), "Parsing job");
        auto jobstring = goal->job;
        Job job = Job::create(jobstring, this->get_logger());
        RCLCPP_INFO(this->get_logger(), "Parsing succeded");
        RCLCPP_INFO(this->get_logger(), "Dbot running program: %s", job.get_name().c_str());
        //RCLCPP_INFO(this->get_logger(), "Positions: %li", positions.size());
        //RCLCPP_INFO(this->get_logger(), "Proglines: %li", proglines.size());

        // Robot Model
        robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this());
        const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
        RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());

        // Create a MotionSequenceRequest
        moveit_msgs::msg::MotionSequenceRequest sequence_request;
        std::vector<Command> commands = job.get_commands();
        RCLCPP_INFO(this->get_logger(), "Found %li commands\n", commands.size());
        auto len = commands.size();
        for (size_t i = 0; i < len; i++)
        {
            // Check if there is a cancel request
            // if (goal_handle->is_canceling()) {
            //     result->success = false;
            //     goal_handle->canceled(result);
            //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
            //     return;
            // }

            // Data
            Command cmd = commands.at(i);
            // ----- Motion Sequence Item
            // Create a MotionSequenceItem
            moveit_msgs::msg::MotionSequenceItem item;

            // Set pose blend radius
            //double br = (i==0 || i==len-1) ? 0.0 : 0.1*M_PI/180;
            item.blend_radius = 0;
            RCLCPP_INFO(this->get_logger(), "Command:%li Blend Radius: %lf",i,item.blend_radius);

            // PTP | JOINT
            if(cmd.move_type == MoveType::Ptp)
            {
                RCLCPP_INFO(this->get_logger(), "Move Type: PTP");
                // PTP 100.0 [20.0,10.0,00.0,0.0,0.0,0.0]
                double speed = cmd.speed / 100.0;
                std::array<double, 6> target = cmd.position;
                std::vector<double> joint_values;
                for (size_t i = 0; i < target.size(); i++)
                {
                    double rad = target[i] * M_PI / 180.0;
                    joint_values.push_back(rad);
                }
                
                // MotionSequenceItem configuration
                item.req.group_name = "dbot_arm";
                item.req.planner_id = "PTP";
                item.req.allowed_planning_time = 5.0;
                item.req.max_velocity_scaling_factor = speed;
                item.req.max_acceleration_scaling_factor = 1;

                // Using the :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`, we can
                // construct a :moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.h>` that
                // maintains the configuration of the robot. We will set all joints in the state to their default values. We can then
                // get a :moveit_codedir:`JointModelGroup<moveit_core/robot_model/include/moveit/robot_model/joint_model_group.h>`,
                // which represents the robot model for a particular group, e.g. the "panda_arm" of the Panda robot.
                // Robot State
                moveit::core::RobotState robot_state(kinematic_model);
                const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("dbot_arm");
                robot_state.setJointGroupPositions(joint_model_group, joint_values);                
                auto constraints = kinematic_constraints::constructGoalConstraints(robot_state,joint_model_group);
                item.req.goal_constraints.push_back(constraints);

                // Add to Motion Sequence
                RCLCPP_INFO(this->get_logger(), "Adding Cmd: PTP, Speed:%lf, Pos:[%lf,%lf,%lf,%lf,%lf,%lf,]", 
                                                speed,
                                                joint_values[0],joint_values[1],joint_values[2],
                                                joint_values[3],joint_values[4],joint_values[5]);
                sequence_request.items.push_back(item);
            }

            // LINEAR
            else if(cmd.move_type == MoveType::Lin)
            {
                // RCLCPP_INFO(this->get_logger(), "Move Type: PTP");
                // // PTP 0000 100.0
                // double speed = cmd.speed;
                // std::array<double, 6> target = cmd.position;
                // std::vector<double> joint_values;

                // // ----- Motion Sequence Item
                // // Create a MotionSequenceItem
                // moveit_msgs::msg::MotionSequenceItem item1;

                // // Set pose blend radius
                // item1.blend_radius = 0.1;

                // // MotionSequenceItem configuration
                // item1.req.group_name = "dbot_arm";
                // item1.req.planner_id = "LIN";
                // item1.req.allowed_planning_time = 5.0;
                // item1.req.max_velocity_scaling_factor = 0.1;
                // item1.req.max_acceleration_scaling_factor = 0.1;

                // moveit_msgs::msg::Constraints constraints_item1;
                // moveit_msgs::msg::PositionConstraint pos_constraint_item1;
                // pos_constraint_item1.header.frame_id = "world";
                // pos_constraint_item1.link_name = "panda_hand";

                // // Set a constraint pose
                // auto target_pose_item1 = [target] {
                //     geometry_msgs::msg::PoseStamped msg;
                //     msg.header.frame_id = "base_link";
                //     msg.pose.position.x = target[0];
                //     msg.pose.position.y = target[1];
                //     msg.pose.position.z = target[2];
                //     // msg.pose.orientation.x = target[0];
                //     // msg.pose.orientation.y = target[0];
                //     // msg.pose.orientation.z = target[0];
                //     // msg.pose.orientation.w = target[0];
                //     return msg;
                // }();
                // item1.req.goal_constraints.push_back(
                //     kinematic_constraints::constructGoalConstraints("tcp_link", target_pose_item1));
            }

        }

        // Verify that the action server is up and running
        if (!move_group_seq_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Error waiting for action server /sequence_move_group");
            return;
        }

        // Create action goal
        auto goal_msg = MoveGroupSequence::Goal();
        goal_msg.request = sequence_request;

        // Goal response callback
        auto LOGGER = this->get_logger();
        auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
        send_goal_options.goal_response_callback = [LOGGER](std::shared_ptr<GoalHandleMoveGroupSequence> goal_handle) 
        {
            try
            {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
                }
                else
                {
                    RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
                }
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(LOGGER, "Exception while waiting for goal response: %s", e.what());
            }
        };

        // Result callback
        send_goal_options.feedback_callback = [LOGGER](GoalHandleMoveGroupSequence::SharedPtr ptr, const std::shared_ptr<const MoveGroupSequence::Feedback> feedback) 
        {
            (void)ptr;
            RCLCPP_INFO(LOGGER, "Feedback state:%s",(feedback->state).c_str());
        };

        // Result callback
        send_goal_options.result_callback = [LOGGER](const GoalHandleMoveGroupSequence::WrappedResult& result) 
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(LOGGER, "Move Group trajectory plan succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(LOGGER, "Goal was aborted. Status: %d", result.result->response.error_code.val);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(LOGGER, "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(LOGGER, "Unknown result code");
                break;
            }
            RCLCPP_INFO(LOGGER, "Result received");
        };

        // Send the action goal
        auto goal_handle_future = move_group_seq_client_->async_send_goal(goal_msg, send_goal_options);

        // Get result
        auto action_result_future = move_group_seq_client_->async_get_result(goal_handle_future.get());

        // Wait for the result
        std::future_status action_status;
        do
        {
            switch (action_status = action_result_future.wait_for(std::chrono::seconds(1)); action_status)
            {
            case std::future_status::deferred:
                RCLCPP_ERROR(LOGGER, "Deferred");
                break;
            case std::future_status::timeout:
                RCLCPP_INFO(LOGGER, "Trajectory is executing...");
                break;
            case std::future_status::ready:
                RCLCPP_INFO(LOGGER, "Trajectory ready!");
                break;
            }
        } while (action_status != std::future_status::ready);

        if (action_result_future.valid())
        {
            auto result = action_result_future.get();
            RCLCPP_INFO(LOGGER, "Action completed. Result: %d", static_cast<int>(result.code));
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Action couldn't be completed.");
        }

        // Check if goal is done
        auto serv_res = std::make_shared<DbotTrajectory::Result>();
        if (rclcpp::ok()) {
            serv_res->success = true;
            goal_handle->succeed(serv_res);
            RCLCPP_INFO(LOGGER, "Goal succeeded");
        }

    }
    
                
}; // CLASS DbotTrajectoryActionServer

} // NAMESPACE dbot_trajectory_player

RCLCPP_COMPONENTS_REGISTER_NODE(dbot_trajectory_player::DbotTrajectoryActionServer)