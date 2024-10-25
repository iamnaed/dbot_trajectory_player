#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>

#include <dbot_interfaces/action/dbot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <dbot_trajectory_player/visibility_control.h>
#include <moveit/move_group_interface/move_group_interface.h>



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
            "dbot_trajectory_action",
            std::bind(&DbotTrajectoryActionServer::handle_goal, this, _1, _2),
            std::bind(&DbotTrajectoryActionServer::handle_cancel, this, _1),
            std::bind(&DbotTrajectoryActionServer::handle_accepted, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Action server running: dbot_trajectory_action");
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
        
        // Guard
        if(!is_valid_job(goal->job))
        {
            RCLCPP_INFO(this->get_logger(), "Invalid job format. Rejecting request. . .");
            return rclcpp_action::GoalResponse::REJECT;
        }

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

        //parse(goal->job);
        
        RCLCPP_INFO(this->get_logger(), "Parsing job");
        auto job = goal->job;
        auto name = get_name(job);
        auto positions = get_positions(job);
        auto proglines = get_proglines(job);
        auto len = proglines.size();
        RCLCPP_INFO(this->get_logger(), "Dbot running program: %s", name.c_str());
        RCLCPP_INFO(this->get_logger(), "Positions: %li", positions.size());
        RCLCPP_INFO(this->get_logger(), "Proglines: %li", proglines.size());

        for (size_t i = 0; i < len; i++)
        {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Data
            auto line = proglines.at(i);
            auto line_trimmed = trim(line);
            auto tkns = get_line_tokens(line_trimmed);
            RCLCPP_INFO(this->get_logger(), "Parsing line: %s", line_trimmed.c_str());

            // Command
            auto cmd = tkns.at(0);
            auto cmd_t = trim(cmd);
            
            // PTP | JOINT
            if(cmd_t == "PTP")
            {
                // PTP 0000 100.0
                int pos = std::stoi(tkns.at(1));
                double speed = std::stod(tkns.at(2));
                std::vector<double> target = positions[pos];

                RCLCPP_INFO(this->get_logger(), "Running PTP. Speed:%f",speed);
                RCLCPP_INFO(this->get_logger(), "Joint target");
                for (size_t i = 0; i < target.size(); i++)
                {
                    RCLCPP_INFO(this->get_logger(), "\tj%li : %f", i, target.at(i));
                }
                
                // Publish feedback
                std::array<double, 6> targetarr;
                for (size_t i = 0; i < 6; i++)
                {
                    targetarr.at(i) = target.at(i);
                }
                feedback->joints = targetarr;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Published feedback");

                loop_rate.sleep();
            }

            // LINEAR
            else if(cmd_t == "LIN")
            {

            }
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    /**
     * @brief Get the name object
     * 
     * @param job 
     * @return std::string 
     */
    std::string get_name(const std::string& job)
    {
        std::string name;
        auto joblines = get_joblines(job);
        for (size_t i = 0; i < joblines.size(); i++)
        {
            auto line = joblines.at(i);
            auto line_trimmed = trim(line);
            auto tkns = get_tokens(job, ' ');
            if(line_trimmed.rfind("NAME") == 0)
            {
                name = tkns.at(1);
                break;
            }
        }
        return name;
    }
    
    /**
     * @brief Get the tokens object
     * 
     * @param line 
     * @param split 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> get_tokens(const std::string& line, const char& split)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(line);
        std::string token;
        while(std::getline(ss, token, split))
        {
            tokens.push_back(token);
        }

        return tokens;
    }
    
    /**
     * @brief Get the proglines object
     * 
     * @param job 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> get_proglines(const std::string& job)
    {
        std::vector<std::string> proglines;
        auto joblines = get_joblines(job);
        bool is_prog_line = false;
        for (size_t i = 0; i < joblines.size(); i++)
        {
            auto line = joblines.at(i);
            auto line_trimmed = trim(line);
            auto tkns = get_tokens(job, ' ');

            // Program End
            if(line_trimmed.rfind("PROG_END") == 0)
            {
                is_prog_line = false;
                break;
            }

            // Program Start
            if(line_trimmed.rfind("PROG_START") == 0)
            {
                is_prog_line = true;
                continue; // Skip a line
            }

            // Add line
            if(is_prog_line)
            {
                proglines.push_back(line_trimmed);
            }
        }
        return proglines;
    }

    /**
     * @brief 
     * 
     * @param job 
     */
    std::vector<std::string> get_joblines(const std::string& job)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(job);
        std::string token;
        while(std::getline(ss, token, '\n'))
        {
            tokens.push_back(token);
        }

        return tokens;
    }

    /**
     * @brief Get the line tokens object
     * 
     * @param jobline 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> get_line_tokens(const std::string& jobline)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(jobline);
        std::string token;
        while(std::getline(ss, token, ' '))
        {
            tokens.push_back(token);
        }

        return tokens;
    }

    /**
     * @brief Get the line tokens object
     * 
     * @param job 
     * @return std::vector<std::string> 
     */
    std::map<int, std::vector<double>> get_positions(const std::string& job)
    {
        std::map<int, std::vector<double>> positions;
        auto joblines = get_joblines(job);
        bool is_pos = false;
        for (size_t i = 0; i < joblines.size(); i++)
        {
            auto line = joblines.at(i);
            auto line_trimmed = trim(line);
            auto tkns = get_line_tokens(line_trimmed);
            
            if(line_trimmed == "POS_START")
            {
                is_pos = true;
            }
            else if(line_trimmed == "POS_END")
            {
                is_pos = false;
                break;
            }

            if(is_pos)
            {
                // 0002 = 20,-10,-30,45,-10,10
                if(line_trimmed.find("=") != std::string::npos)
                {
                    int idx = get_index(line_trimmed);
                    std::vector<double> joints = get_joints(line_trimmed);
                    positions[idx] = joints;
                }
            }            
        }

        return positions;
    }

    /**
     * @brief Get the index object
     * 
     * @param jointline 
     * @return int 
     */
    int get_index(const std::string& jointline)
    {
        auto sub_idx = jointline.substr(0,jointline.find("="));
        int idx = std::stoi(sub_idx);
        return idx;
    }

    /**
     * @brief Get the joints object
     * 
     * @param jointline 
     * @return std::vector<double> 
     */
    std::vector<double> get_joints(const std::string& jointline)
    {
        std::vector<double> joints;
        auto sub_values = jointline.substr(jointline.find("=")+1);
        auto tokens = get_tokens(jointline, ',');
        for (size_t i = 0; i < 6; i++)
        {
             double j = std::stod(tokens.at(i));
             joints.push_back(j);
        }
        
        return joints;
    }

    /**
     * @brief 
     * 
     * @param job 
     */
    void parse(const std::string& job)
    {
        // MoveGroupInterface
        // using moveit::planning_interface::MoveGroupInterface;
        // auto mgi = MoveGroupInterface(, "dbot_arm");
        RCLCPP_INFO(this->get_logger(), "Parsing job");
        rclcpp::Rate loop_rate(1);
        auto name = get_name(job);
        auto positions = get_positions(job);
        auto joblines = get_joblines(job);
        auto len = joblines.size();
        RCLCPP_INFO(this->get_logger(), "Dbot running program: %s", name.c_str());
        RCLCPP_INFO(this->get_logger(), "Positions: %li", positions.size());
        RCLCPP_INFO(this->get_logger(), "Joblines: %li", joblines.size());

        for (size_t i = 0; i < len; i++)
        {
            auto line = joblines.at(i);
            auto line_trimmed = trim(line);
            auto tkns = get_line_tokens(line_trimmed);

            // Command
            auto cmd = tkns.at(0);
            auto cmd_t = trim(cmd);
            RCLCPP_INFO(this->get_logger(), "Parsing line: %s", cmd_t.c_str());

            // PTP | JOINT
            if(cmd_t == "PTP")
            {
                // PTP 0000 100.0
                int pos = std::stoi(tkns.at(1));
                double speed = std::stod(tkns.at(2));
                std::vector<double> target = positions[pos];

                RCLCPP_INFO(this->get_logger(), "Running PTP. Speed:%f",speed);
                RCLCPP_INFO(this->get_logger(), "Joint target");
                for (size_t i = 0; i < target.size(); i++)
                {
                    RCLCPP_INFO(this->get_logger(), "\tj%li : %f", i, target.at(i));
                }
                
                loop_rate.sleep();

        //     // Check if there is a cancel request
        //     if (goal_handle->is_canceling()) {
        //         result->success = false;
        //         goal_handle->canceled(result);
        //         RCLCPP_INFO(this->get_logger(), "Goal canceled");
        //         return;
        //     }

        //     // Update sequence
        //     joints = {0.0*i,1.0*i,2.0*i,3.0*i,4.0*i,5.0*i};

        //     // Publish feedback
        //     goal_handle->publish_feedback(feedback);
        //     RCLCPP_INFO(this->get_logger(), "Publish feedback");



                // // Set a target Pose
                // RCLCPP_INFO(this->get_logger(), "Dbot setting joint targets.");
                // mgi.setJointValueTarget(target);

                // // Display planner settings
                // RCLCPP_INFO(this->get_logger(), "Dbot setting planner.");
                // mgi.setPlanningPipelineId("pilz_industrial_motion_planner");
                // mgi.setPlannerId("PTP");
                // mgi.setMaxVelocityScalingFactor(1);
                // mgi.setMaxAccelerationScalingFactor(1);

                // const std::string& planner_id = mgi.getPlannerId();
                // const std::string& planner_frame = mgi.getPlanningFrame();
                // const std::string& planner_pipeline_id = mgi.getPlanningPipelineId();
                // const double& planner_planning_time = mgi.getPlanningTime();
                // RCLCPP_INFO(this->get_logger(), "Dbot planner_id: %s", planner_id.c_str());
                // RCLCPP_INFO(this->get_logger(), "Dbot planner_frame: %s", planner_frame.c_str());
                // RCLCPP_INFO(this->get_logger(), "Dbot planning_pipeline_id: %s", planner_pipeline_id.c_str());
                // RCLCPP_INFO(this->get_logger(), "Dbot planning_time: %lf", planner_planning_time);

                // // Plan
                // RCLCPP_INFO(this->get_logger(), "Dbot attempting to create plan.");
                // auto const [error_code, plan] = [&mgi]{
                //     moveit::planning_interface::MoveGroupInterface::Plan pl;
                //     auto const ok = mgi.plan(pl);
                //     return std::make_pair(ok, pl);
                // }();

                // // Print plan
                // //print_plan(plan);
                
                // // Execute
                // if(error_code == moveit::core::MoveItErrorCode::SUCCESS){
                //     RCLCPP_INFO(this->get_logger(), "Dbot executing plan.");
                //     mgi.execute(plan);
                // } else {
                //     RCLCPP_ERROR(this->get_logger(), "Dbot planning failed.");
                // }
            }

            // LINEAR
            else if(line.rfind("LIN", 0) == 0)
            {

            }
        }
        
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
        
    /**
     * @brief 
     * 
     * @param job 
     * @return true 
     * @return false 
     */
    bool is_valid_job(const std::string& job)
    {
        (void)job;
        return true;
    }



}; // CLASS DbotTrajectoryActionServer

} // NAMESPACE dbot_trajectory_player

RCLCPP_COMPONENTS_REGISTER_NODE(dbot_trajectory_player::DbotTrajectoryActionServer)