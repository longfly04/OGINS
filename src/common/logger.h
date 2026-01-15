/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Your Name
 *    Contact : your.email@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <string>
#include <ctime>
#include <fstream>

class Logger {
public:
    enum LogLevel {
        DEBUG = 0,
        INFO,
        WARN,
        ERROR,
        FATAL
    };

    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void setLevel(LogLevel level) {
        level_ = level;
    }

    void setLogFile(const std::string& filename) {
        log_file_.open(filename, std::ios::app);
        if (log_file_.is_open()) {
            std::cout << "Logger: Log file opened: " << filename << std::endl;
        } else {
            std::cerr << "Logger: Failed to open log file: " << filename << std::endl;
        }
    }

    void closeLogFile() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    template<typename... Args>
    void debug(const char* format, Args&&... args) {
        log(DEBUG, format, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void info(const char* format, Args&&... args) {
        log(INFO, format, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void warn(const char* format, Args&&... args) {
        log(WARN, format, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void error(const char* format, Args&&... args) {
        log(ERROR, format, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void fatal(const char* format, Args&&... args) {
        log(FATAL, format, std::forward<Args>(args)...);
    }

private:
    Logger() : level_(INFO) {
        // 初始化日志系统
    }

    ~Logger() {
        closeLogFile();
    }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::string getCurrentTime() {
        std::time_t now = std::time(nullptr);
        char buffer[20];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        return std::string(buffer);
    }

    std::string levelToString(LogLevel level) {
        switch (level) {
            case DEBUG:
                return "DEBUG";
            case INFO:
                return "INFO";
            case WARN:
                return "WARN";
            case ERROR:
                return "ERROR";
            case FATAL:
                return "FATAL";
            default:
                return "UNKNOWN";
        }
    }

    template<typename... Args>
    void log(LogLevel level, const char* format, Args&&... args) {
        if (level < level_) {
            return;
        }

        std::string time_str = getCurrentTime();
        std::string level_str = levelToString(level);

        // 构建日志消息
        char buffer[1024];
        std::snprintf(buffer, sizeof(buffer), format, std::forward<Args>(args)...);
        std::string message(buffer);

        // 输出到控制台
        std::cout << "[" << time_str << "] [" << level_str << "] " << message << std::endl;

        // 输出到文件
        if (log_file_.is_open()) {
            log_file_ << "[" << time_str << "] [" << level_str << "] " << message << std::endl;
        }
    }

    LogLevel level_;
    std::ofstream log_file_;
};

// 日志宏定义
#define LOG_DEBUG(format, ...) Logger::getInstance().debug(format, ##__VA_ARGS__)
#define LOG_INFO(format, ...) Logger::getInstance().info(format, ##__VA_ARGS__)
#define LOG_WARN(format, ...) Logger::getInstance().warn(format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) Logger::getInstance().error(format, ##__VA_ARGS__)
#define LOG_FATAL(format, ...) Logger::getInstance().fatal(format, ##__VA_ARGS__)

#endif // LOGGER_H
