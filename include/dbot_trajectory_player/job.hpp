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
#include <format>

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
     * @param pos 
     * @param spd 
     */
    Command(int idx, MoveType mt, const std::array<double,6>& pos, double spd)
    {
        index = idx;
        move_type = mt;
        position = pos;
        speed = spd
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
        std::string pos = std::format("{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}",
            position[0], position[1],position[2],position[3],position[4],position[5]);
        return std::format("{}: {}, {}, [{}],", idx, mt, spd, pos);
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
        switch (str) {
            case "PTP":             return MoveType::Ptp;
            case "LIN":             return MoveType::Lin;
            case "CIRC":            return MoveType::Circ;
            case "NOP":             return MoveType::Nop;
            default:                return MoveType::Unk;
        }
    }
}

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
        date_ = std::format("{}/{:02}/{:02} {:02}:{:02}",
                                                local_tm.tm_year + 1900,  // Year
                                                local_tm.tm_mon + 1,      // Month
                                                local_tm.tm_mday,         // Day
                                                local_tm.tm_hour,         // Hour
                                                local_tm.tm_min);         // Minute

        // Commands default to an empty vector
        commands_ = {};
    }

    /**
     * @brief Construct a new Job object
     * 
     */
    Job(const Job&){};

    /**
     * @brief 
     * 
     * @param job 
     * @return Job 
     */
    static Job create(const std::string& job)
    {
        // Guard
        bool is_job_valid = Job::is_valid(job);
        if(!is_job_valid) return Job();

        // Process
        std::istringstream job_stream(job); // Create a string stream from the input string
        std::string line;

        // Loop through each line in the string stream
        std::string name;
        std::string date;
        std::vector<Command> cmds;
        bool is_prog_start = false;

        while (std::getline(job_stream, line)) {
            std::string l = trim(l);
            if(l.starts_with('NAME'))
            {
                auto tkns = get_tokens(l, ' ');
                name = trim(tkns[1]);
            }
            if(l.starts_with('DATE'))
            {
                auto tkns = get_tokens(l, ' ');
                date = trim(tkns[1]);
            }
            if(l.starts_with('PROG_START'))
            {
                is_prog_start = true;
            }
            if(l.starts_with('PROG_END'))
            {
                is_prog_start = false;
            }

            if(is_prog_start)
            {
                
            }

            std::cout << line << std::endl; // Process the line (e.g., print it)
        }

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
        return true;
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
        auto sub_idx = trim(line.substr(0,line.find(":")));
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
     * @brief Get the index object
     * 
     * @param line 
     * @return int 
     */
    int get_index(const std::string& line) const
    {
        auto sub_idx = line.substr(0,line.find("="));
        int idx = std::stoi(sub_idx);
        return idx;
    }


    /**
     * @brief Convert encoder values to actual joint positions
     * 
     * @param encoder 
     * @return std::array<float, 6> 
     */
    std::array<float, 6> convert_encoder_to_joint(std::array<float, 6> encoder) const
    {
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
    std::string trim(const std::string &s) 
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