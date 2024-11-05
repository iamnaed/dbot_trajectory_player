#ifndef DBOT_TRAJECTORY_PLAYER__JOB_HPP_
#define DBOT_TRAJECTORY_PLAYER__JOB_HPP_

#include <cstring>
#include <memory>
#include <array>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <chrono>
//#include <format> //not supported on gcc 11

/**
 * @brief 
 * 
 */
enum class MoveType {
    Ptp,
    Lin,
    Circ,
    Nop,
    Unk,
};

/**
 * @brief 
 * 
 */
struct Command
{
public:
    /**
     * @brief Construct a new Command object
     * 
     * @param idx 
     * @param mt 
     * @param spd 
     * @param pos 
     */
    Command(int idx, MoveType mt, double spd, const std::array<double,6>& pos)
    {
        index = idx;
        move_type = mt;
        speed = spd;
        position = pos;
    }

    /**
     * @brief 
     * 
     * @return std::string 
     */
    std::string to_string() const 
    {
        std::string idx = std::to_string(index);
        std::string mt = movetype_to_string(move_type);
        std::string spd = std::to_string(speed);
        // std::string pos = std::format("{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}",
        //     position_[0], position_[1],position_[2],position_[3],position_[4],position_[5]);
        std::string pos = std::to_string(position[0]) + "," +
                        std::to_string(position[1]) + "," +
                        std::to_string(position[2]) + "," +
                        std::to_string(position[3]) + "," +
                        std::to_string(position[4]) + "," +  
                        std::to_string(position[5]);
        //std::string res = std::format("{}: {}, {}, [{}],", idx, mt, spd, pos);
        std::string res = idx + ": " + mt + ", " + spd + ", [" + pos + "]";
        return res;
    }

    /**
     * @brief 
     * 
     */
    int index;
    MoveType move_type;
    std::array<double, 6> position;
    double speed;

public:
    /**
     * @brief 
     * 
     * @param move_type 
     * @return std::string 
     */
    static std::string movetype_to_string(const MoveType& move_type)
    {
        switch (move_type) {
            case MoveType::Ptp:     return "PTP";
            case MoveType::Lin:     return "LIN";
            case MoveType::Circ:    return "CIRC";
            case MoveType::Nop:     return "NOP";
            default:                return "UNK";
        }
    }

    /**
     * @brief 
     * 
     * @param str 
     * @return MoveType
     */
    static MoveType string_to_movetype(const std::string& str)
    {
        MoveType mt;
        if (str == "PTP")
        {
            mt = MoveType::Ptp;
        }
        else if(str == "LIN")
        {
            mt = MoveType::Lin;
        }
        else if(str == "CIRC")
        {
            mt = MoveType::Circ;
        }
        else if(str == "NOP")
        {
            mt = MoveType::Nop;
        }
        else
        {
            mt = MoveType::Unk;
        }
        
        return mt;
    }

    /**
     * @brief 
     * 
     * @param str 
     * @return std::array<double,6>
     */
    static std::array<double,6> string_to_pos6(const std::string& str)
    {
        // Guard
        // Must be in the form
        // [20.0,-10.0,-30.0,45.0,-10.0,10.0]
        if(str.at(0)!='[' || str.at(str.size()-1)!=']') return {};
        if(str.size()<=3) return {};

        // Process
        std::string vals = str.substr(1,str.size()-2);  
        auto s = Command::get_tokens(vals,',');
        std::array<double,6> ret;
        ret[0] = std::stod(s[0]);
        ret[1] = std::stod(s[1]);
        ret[2] = std::stod(s[2]);
        ret[3] = std::stod(s[3]);
        ret[4] = std::stod(s[4]);
        ret[5] = std::stod(s[5]);
        return ret;
    }

private:

    /**
     * @brief Get the tokens object
     * 
     * @param line 
     * @param split 
     * @return std::vector<std::string> 
     */
    static std::vector<std::string> get_tokens(const std::string& line, const char& split)
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

};

/**
 * @brief 
 * 
 */
class Job
{
public:
    /**
     * @brief Construct a new Dbot Can object
     * 
     */
    Job()
    {
        // Name
        name_ = "dbot_default_job";
        
        // Get date now
        auto now = std::chrono::system_clock::now();
        auto current_time = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&current_time);
        // date_ = std::format("{}/{:02}/{:02} {:02}:{:02}",
        //                                         local_tm.tm_year + 1900,  // Year
        //                                         local_tm.tm_mon + 1,      // Month
        //                                         local_tm.tm_mday,         // Day
        //                                         local_tm.tm_hour,         // Hour
        //                                         local_tm.tm_min);         // Minute
        // Using std::ostringstream for formatting
        std::ostringstream oss;
        oss << (local_tm.tm_year + 1900) << "/" 
            << (local_tm.tm_mon + 1) << "/" 
            << (local_tm.tm_mday) << " " 
            << (local_tm.tm_hour) << ":" 
            << (local_tm.tm_min < 10 ? "0" : "") << local_tm.tm_min; // Add leading zero if needed

        date_ = oss.str();

        // Commands default to an empty vector
        commands_ = {};
    }
    
    /**
     * @brief 
     * 
     * @param name 
     * @param date 
     * @param cmds 
     * @return Job 
     */
    Job(const std::string& name, const std::string& date, const std::vector<Command>& cmds)
    {
        name_ = name;
        date_ = date;
        commands_ = cmds;
    }

/**
     * @brief 
     * 
     * @param job 
     * @return Job 
     */
    static Job create(const std::string& job, const rclcpp::Logger& logger)
    {
        // Guard
        bool is_job_valid = Job::is_valid(job);
        if(!is_job_valid) return Job();

        RCLCPP_INFO(logger, "[1] Job Starting Parsing of job");
        // Process
        std::istringstream job_stream(job); // Create a string stream from the input string
        std::string line;

        // Loop through each line in the string stream
        std::string name;
        std::string date;
        std::vector<Command> cmds;
        bool is_prog_start = false;
        std::string NAME_C = "NAME";
        std::string DATE_C = "DATE";
        std::string PROGSTART_C = "PROG_START";
        std::string PROGEND_C = "PROG_END";

        RCLCPP_INFO(logger, "[2] Tokenizing");
        while (std::getline(job_stream, line)) 
        {
            std::string l = trim(line);
            if(l.substr(0,NAME_C.size())==NAME_C)
            {
                RCLCPP_INFO(logger, "[3] Found name");
                auto tkns = get_tokens(l, ' ');
                name = trim(tkns[1]);
            }
            
            if(l.substr(0,DATE_C.size())==DATE_C)
            {
                RCLCPP_INFO(logger, "[4] Found date");
                auto tkns = get_tokens(l, ' ');
                date = trim(tkns[1]);
            }

            if(l.substr(0,PROGSTART_C.size())==PROGSTART_C)
            {
                RCLCPP_INFO(logger, "[5] Found PROG START");
                is_prog_start = true;
                continue;
            }

            if(l.substr(0,PROGEND_C.size())==PROGEND_C)
            {
                is_prog_start = false;
            }

            /*
            PROG_START
                0 PTP 100.0 [0.0,0.0,0.0,0.0,0.0,0.0]
                1 PTP 100.0 [0.0,10.0,30.0,0.0,0.0,10.0]
                2 PTP 100.0 [20.0,-10.0,-30.0,45.0,-10.0,10.0]
                3 PTP 100.0 [0.0,0.0,0.0,0.0,0.0,0.0]
            PROG_END
            */
            if(is_prog_start)
            {
                // 0 PTP 100.0 [0.0,0.0,0.0,0.0,0.0,0.0]
                auto lt = trim(l);
                auto pls = get_tokens(lt,' ');
                int cmdidx = std::stoi(trim(pls[0]));
                MoveType cmdmt = Command::string_to_movetype(trim(pls[1]));
                double cmdspd = std::stod(trim(pls[2]));
                std::array<double, 6> cmdpos = Command::string_to_pos6(trim(pls[3]));
                Command cmd(cmdidx,cmdmt,cmdspd,cmdpos);
                cmds.push_back(cmd);
            }

            //std::cout << line << std::endl; // Process the line (e.g., print it)
        }
        RCLCPP_INFO(logger, "[6] Initializing success");

        return Job(name, date, cmds);
    }

    /**
     * @brief 
     * 
     * @param job 
     * @return true 
     * @return false 
     */
    static bool is_valid(const std::string& job)
    {
        (void)job;
        return true;
    }

    /**
     * @brief 
     * 
     * @return std::string 
     */
    std::string get_name() const
    {
        return name_;
    }

    /**
     * @brief 
     * 
     * @return std::string 
     */
    std::string get_date() const
    {
        return date_;
    }

    /**
     * @brief 
     * 
     * @return std::vector<Command> 
     */
    std::vector<Command> get_commands() const
    {
        return commands_;
    }

private:

    /**
     * @brief 
     * 
     * @param line 
     * @return Command 
     */
    Command convert_line_to_command(const std::string& line) const
    {
        int idx;
        MoveType mt;
        double spd;
        std::array<double,6> pos;

        // Process
        // Index
        std::string sub_idx = trim(line.substr(0,line.find(":")));
        idx = std::stoi(sub_idx);
        // Motion Command i.e. PTP,LIN,CIRC
        std::vector<double> joints;
        auto sub_values = trim(line.substr(line.find(":")+1));
        auto tokens = get_tokens(line, ',');
        std::string mt_raw = trim(tokens[0]);
        mt = Command::string_to_movetype(mt_raw);
        // Speed
        spd = std::stod(trim(tokens[1]));
        // Position [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        std::string pos_raw = trim(tokens[0]); // Remove spaces
        pos_raw = pos_raw.substr(1, pos_raw.length() - 2); // Remove Brackets [ and ]
        auto pos_tkns = get_tokens(pos_raw, ',');
        pos[0] = std::stod(pos_tkns[0]);
        pos[1] = std::stod(pos_tkns[1]);
        pos[2] = std::stod(pos_tkns[2]);
        pos[3] = std::stod(pos_tkns[3]);
        pos[4] = std::stod(pos_tkns[4]);
        pos[5] = std::stod(pos_tkns[5]);

        return Command(idx, mt, spd, pos);
    }

    /**
     * @brief 
     * 
     * @param s 
     * @return std::string 
     */
    static std::string ltrim(const std::string &s)
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
    static std::string rtrim(const std::string &s)
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
    static std::string trim(const std::string &s)
    {
        return rtrim(ltrim(s));
    }

    /**
     * @brief Get the tokens object
     * 
     * @param line 
     * @param split 
     * @return std::vector<std::string> 
     */
    static std::vector<std::string> get_tokens(const std::string& line, const char& split)
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

private:
    /**
     * @brief Internal members
     * 
     */
    std::string name_;
    std::string date_;
    std::vector<Command> commands_;
};

#endif // DBOT_TRAJECTORY_PLAYER__JOB_HPP_