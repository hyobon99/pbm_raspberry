#pragma once

#include <tuple>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>
#include <iomanip>

#include <memory>
#include <queue>
#include <unordered_map>
#include <mutex>
#include <functional>

#include "parameters.h"

namespace PhaseBasedMotionMagification {
// -----------------------------
// [1] ENUM DEFINITIONS
// -----------------------------

enum E_LOG_PROPERTIES {
    // Pyramid CalculateProcess::prepare_variables_to_push
    PREPARE_VARS_ = 100,
    PREPARE_VARS_START = 101,
    PREPARE_VARS_INITIALIZE_PYRAMID = 102,
    PREPARE_VARS_FFT2 = 103,
    PREPARE_VARS_CALC_ELEMENT_WISE_MUL = 104,
    PREPARE_VARS_IFFT2 = 105,
    PREPARE_VARS_CALC_PHASE_ANGLE = 106,
    PREPARE_VARS_END = 107,

    // Pyramid CalculateProcess::run
    RUN_ = 200,
    RUN_START = 201,
    RUN_INTIALIZE_PTR_AND_VARS = 202,
    RUN_FFT2_CURRENT_FRAME = 203,
    RUN_CALC_ELEMENT_WISE_MUL_CSP = 204,
    RUN_IFFT2_LEVEL_WISE = 205,
    RUN_LAZY_PICK_FFT2_FROM_QUEUE = 206,
    RUN_LAZY_CALC_ELEMENT_WISE_MUL = 207,
    RUN_LAZY_CALC_PAHSE = 208,
    RUN_PICK_PHASE_FROM_QUEUE = 209,
    RUN_CALC_PHASE_DIFFERENCE_QUEUE = 210,
    RUN_TEMPORAL_FIR_FILTERING = 211,
    RUN_CALC_ELEMENT_WISE_MUL_ALPHA = 212, // can be merged to exp(1j) process which following after
    RUN_CALC_EXP_OF_PHASE = 213,
    RUN_CALC_ELEMENT_WISE_MUL_OUTPUT = 214,
    RUN_FFT2_OUTPUT = 215,
    RUN_GET_CSP_DOUBLED = 216, // Can be prepared before run
    RUN_CALC_ELEMENT_WISE_MUL_CSP_DOUBLED = 217,
    RUN_ADD_UPON_REGION = 218,
    RUN_LOWPASS_CALC_ELEMENT_WISE_MUL_CSP = 219,
    RUN_LOWPASS_ADD_UPON_REGION = 220,
    RUN_IFFT2_FINAL = 221,
    RUN_END = 230,
};

enum E_LOG_PRINT_CASE {
    HEADER_PREPARING_VARS,
    HEADER_RUN,
    HEADER_SUMMARY,

    TIMESTAMP_PREPARING_VARS,
    TIMESTAMP_RUN,
    ELLAPSED_PREPARING_VARS,
    ELLAPSED_RUN,
    SUMMARY,
};

// -----------------------------
// [2] LogKey STRUCT + hash
// -----------------------------
struct LogKey {
    int TaskId;
    int Level;
    int Orientation;
    E_LOG_PROPERTIES Process;

    // for key usage
    bool operator<(const LogKey& other) const {
        return std::tie(TaskId, Level, Orientation, Process) < 
               std::tie(other.TaskId, other.Level, other.Orientation, other.Process);
    }
    
    bool operator==(const LogKey& other) const {
    return TaskId == other.TaskId &&
           Level == other.Level &&
           Orientation == other.Orientation &&
           Process == other.Process;
    }


};
}; // namespace PhaseBasedMotionMagification 


// -----------------------------
// [3] std::hash<LogKey> specialization
// -----------------------------
namespace std {
    template <>
    struct hash<PhaseBasedMotionMagification::LogKey> {
        std::size_t operator()(const PhaseBasedMotionMagification::LogKey& key) const {
            std::size_t h1 = std::hash<int>()(key.TaskId);
            std::size_t h2 = std::hash<int>()(key.Level);
            std::size_t h3 = std::hash<int>()(key.Orientation);
            std::size_t h4 = std::hash<int>()(key.Process);  // 캐스팅 필요 없음

            // 간단한 해시 결합
            std::size_t combined = h1;
            combined ^= h2 + 0x9e3779b9 + (combined << 6) + (combined >> 2);
            combined ^= h3 + 0x9e3779b9 + (combined << 6) + (combined >> 2);
            combined ^= h4 + 0x9e3779b9 + (combined << 6) + (combined >> 2);

            return combined;
        }
    };
};


// -----------------------------
// [4] LoggingManager CLASS
// -----------------------------
namespace PhaseBasedMotionMagification {

class LoggingManager {
public:
    // singleton
    static LoggingManager& GetInstance() {
        static LoggingManager instance;
        return instance;
    }

    LoggingManager(const LoggingManager&) = delete; 
    LoggingManager& operator=(const LoggingManager&) = delete;

    void LogTimeStamp(LogKey logKey, std::chrono::steady_clock::time_point timeStamp);
    void UpdateFrameInfo(int _width, int _height);
    int SummarizeResults();

private:
    LoggingManager() = default;
    ~LoggingManager() {
        // Destructor
        TimeLogger.clear();
    }

    std::string GetNowTime();
    int PrintBasicInfos(std::ofstream& file);
    int PrintHeaders(std::ofstream& file, E_LOG_PRINT_CASE eHeaderType);
    int PrintData(std::ofstream& file, E_LOG_PRINT_CASE eHeaderType);
    int ConvertChronoClockToDouble();
    int CalculateProcessEllapsedTime();

    // logging map: according to level and property, absolute timestamps are recorded
    std::unordered_map<LogKey, std::chrono::steady_clock::time_point> TimeLogger;
    std::unordered_map<LogKey, double> ProcessTimeStamp;
    std::unordered_map<LogKey, double> ProcessEllapsedTime;
    std::mutex iMtx_Log;
    
    int width;
    int height;

};

}; // namespace PhaseBasedMotionMagification

