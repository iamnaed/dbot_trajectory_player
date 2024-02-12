#include <functional>
#include <memory>
#include <thread>

#include <dbot_interfaces/action/dbot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <dbot_trajectory_player/visibility_control.h>

namespace dbot_trajectory_player
{

class DbotTrajectoryActionServer : public rclcpp::Node
{
public:
    using DbotTrajectory = dbot_interfaces::action::DbotTrajectory;
    using DbotTrajectoryGoalHandle = rclcpp_action::ServerGoalHandle<DbotTrajectory>;

    DBOT_TRAJECTORY_PLAYER_CPP_PUBLIC
    explicit DbotTrajectoryActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("dbot_trajectory_action_server_node", options)
    {
        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<DbotTrajectory>(
            this, 
            "dbot_trajectory_action_server",
            std::bind(&DbotTrajectoryActionServer::handle_goal, this, _1, _2),
            std::bind(&DbotTrajectoryActionServer::handle_cancel, this, _1),
            std::bind(&DbotTrajectoryActionServer::handle_accepted, this, _1)
            );
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
     * @param uuid 
     * @param goal 
     * @return rclcpp_action::GoalResponse 
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DbotTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with job '%s'", goal->job.c_str());
        (void)uuid;
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
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(), "Goal: %s", goal->job.c_str());

        auto feedback = std::make_shared<DbotTrajectory::Feedback>();
        auto & joints = feedback->joints;
        joints = {0,1,2,3,4,5};
        auto result = std::make_shared<DbotTrajectory::Result>();

        for (int i = 1; (i < 10) && rclcpp::ok(); ++i) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Update sequence
            joints = {0.0*i,1.0*i,2.0*i,3.0*i,4.0*i,5.0*i};

            // Publish feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok()) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

}; // CLASS DbotTrajectoryActionServer

} // NAMESPACE dbot_trajectory_player

RCLCPP_COMPONENTS_REGISTER_NODE(dbot_trajectory_player::DbotTrajectoryActionServer)