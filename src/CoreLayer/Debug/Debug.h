#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <mutex>
#include <chrono>

namespace Debug {
    // 调试信息级别
    enum class Level {
        INFO,
        WARNING,
        ERROR
    };

    // 调试信息结构
    struct DebugInfo {
        Level level;
        std::string message;
        std::string file;
        int line;
        double timestamp;
    };

    class DebugManager {
    private:
        static DebugManager* instance;
        std::vector<DebugInfo> debugLog;
        std::mutex logMutex;
        std::ofstream logFile;
        bool isLoggingToFile;

        DebugManager() : isLoggingToFile(false) {}
        ~DebugManager() {
            if (logFile.is_open()) {
                logFile.close();
            }
        }

    public:
        static DebugManager* getInstance() {
            if (!instance) {
                instance = new DebugManager();
            }
            return instance;
        }

        // 启用文件日志记录
        void enableFileLogging(const std::string& filename) {
            std::lock_guard<std::mutex> lock(logMutex);
            if (logFile.is_open()) {
                logFile.close();
            }
            logFile.open(filename);
            isLoggingToFile = logFile.is_open();
        }

        // 添加调试信息
        void log(Level level, const std::string& message, const std::string& file, int line) {
            DebugInfo info{
                level,
                message,
                file,
                line,
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count() / 1000.0
            };

            std::lock_guard<std::mutex> lock(logMutex);
            debugLog.push_back(info);

            if (isLoggingToFile && logFile.is_open()) {
                logFile << "[" << info.timestamp << "] "
                       << "[" << (level == Level::INFO ? "INFO" : 
                                 level == Level::WARNING ? "WARNING" : "ERROR") << "] "
                       << file << ":" << line << " - "
                       << message << std::endl;
            }
        }

        // 获取所有调试信息
        std::vector<DebugInfo> getDebugLog() {
            std::lock_guard<std::mutex> lock(logMutex);
            return debugLog;
        }

        // 清空调试日志
        void clearLog() {
            std::lock_guard<std::mutex> lock(logMutex);
            debugLog.clear();
        }
    };

    // 宏定义，方便使用
    #define DEBUG_LOG(level, message) \
        Debug::DebugManager::getInstance()->log(level, message, __FILE__, __LINE__)

    #define DEBUG_INFO(message) DEBUG_LOG(Debug::Level::INFO, message)
    #define DEBUG_WARNING(message) DEBUG_LOG(Debug::Level::WARNING, message)
    #define DEBUG_ERROR(message) DEBUG_LOG(Debug::Level::ERROR, message)
}