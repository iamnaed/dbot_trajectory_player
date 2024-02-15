#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>

#include <dbot_interfaces/action/dbot_trajectory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace dbot_trajectory_player
{

class DbotTrajectoryActionClient : public rclcpp::Node
{
public:
    using DbotTrajectory = dbot_interfaces::action::DbotTrajectory;
    using GoalHandleDbotTrajectory = rclcpp_action::ClientGoalHandle<DbotTrajectory>;

    /**
     * @brief Construct a new Dbot Trajectory Action Client object
     * 
     * @param options 
     */
    explicit DbotTrajectoryActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("dbot_trajectory_action_client_node", options)
    {
        this->action_client_ = rclcpp_action::create_client<DbotTrajectory>(this, "dbot_trajectory_action");
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&DbotTrajectoryActionClient::send_goal, this));
    }

    /**
     * @brief 
     * 
     */
    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->action_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        // Send
        auto goal_msg = DbotTrajectory::Goal();
        goal_msg.job = "NAME dbot_job \n\
DATE 2024/02/07 16:04 \n\
POS_START \n\
    0000 = 0,0,0,0,0,0 \n\
    0001 = 0,10,30,0,0,10 \n\
    0002 = 20,-10,-30,45,-10,10 \n\
    0003 = 0,0,0,0,0,0 \n\
POS_END \n\
PROG_START \n\
    PTP 0000 100.0 \n\
    PTP 0001 100.0 \n\
    PTP 0002 100.0 \n\
    PTP 0003 100.0 \n\
PROG_END";
        RCLCPP_INFO(this->get_logger(), "Sending goal");
        auto send_goal_options = rclcpp_action::Client<DbotTrajectory>::SendGoalOptions();
        //send_goal_options.goal_response_callback = std::bind(&DbotTrajectoryActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&DbotTrajectoryActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&DbotTrajectoryActionClient::result_callback, this, _1);
        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    /**
     * @brief 
     * 
     */
    rclcpp_action::Client<DbotTrajectory>::SharedPtr action_client_;

    /**
     * @brief 
     * 
     */
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief 
     * 
     * @param future 
     */
    void goal_response_callback(std::shared_future<GoalHandleDbotTrajectory::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }
    
    /**
     * @brief 
     * 
     * @param ptr 
     * @param feedback 
     */
    void feedback_callback(GoalHandleDbotTrajectory::SharedPtr ptr, const std::shared_ptr<const DbotTrajectory::Feedback> feedback)
    {
        (void)ptr;
        std::stringstream ss;
        ss << "Feedback: \n";
        for (size_t i = 0; i < (feedback->joints).size(); i++)
        {
            ss << "Joint" << i << ": " << feedback->joints[i] << "\n";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    /**
     * @brief 
     * 
     * @param result 
     */
    void result_callback(const GoalHandleDbotTrajectory::WrappedResult& result)
    {
        switch (result.code) 
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        std::stringstream ss;
        ss << "Result received: " << result.result->success << "\n";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }

    /**
     * @brief 
     * 
     * @param s 
     * @return std::string 
     */
    std::string ltrim(const std::string &s)
    {
        const std::string WHITESPACE = " \n\r\t\f\v";
        size_t start = s.find_first_not_of(WHITESPACE);
        return (start == std::string::npos) ? "" : s.substr(start);
    }
    
    /**
     * @brief 
     * 
     * @param s 
     * @return std::string 
     */
    std::string rtrim(const std::string &s)
    {
        const std::string WHITESPACE = " \n\r\t\f\v";
        size_t end = s.find_last_not_of(WHITESPACE);
        return (end == std::string::npos) ? "" : s.substr(0, end + 1);
    }
    
    /**
     * @brief 
     * 
     * @param s 
     * @return std::string 
     */
    std::string trim(const std::string &s) {
        return rtrim(ltrim(s));
    }
        
}; // CLASS DbotTrajectoryActionClient

} // NAMESPACE dbot_trajectory_player

RCLCPP_COMPONENTS_REGISTER_NODE(dbot_trajectory_player::DbotTrajectoryActionClient)