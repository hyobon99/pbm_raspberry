#pragma once

#include <memory>
#include <queue>
#include <map>
#include <mutex>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <future>
#include <chrono>
#include <atomic>

#include "parameters.h"

// 이미지를 받아서 queue에다 두고 하나씩 pop해서 보여주는 객체
// 필요시 저장도 함.
namespace PhaseBasedMotionMagification {
class StreamingManager{
public:
    // singleton
    static StreamingManager& GetInstance() {
        static StreamingManager instance; // thread-safe lazy initialization
        return instance;
    }

    StreamingManager(const StreamingManager&) = delete;
    StreamingManager& operator=(const StreamingManager&) = delete;

    // data
    void start();
    void stop();
    int EnqueueResult(const cv::Mat& result);

private:
    StreamingManager() : stopFlag_(false), currentKey_(0) {
        // 생성자에 특정 동작이 필요할 경우 여기 작성
    }
    ~StreamingManager() {
        // 소멸자에 특정 동작이 필요할 경우 여기 작성
    }
    
    // data
    std::mutex iMtx_ResultsMap;
    std::map<int, cv::Mat> ResultsMap = std::map<int, cv::Mat>();

    std::atomic<bool> stopFlag_;
    std::future<void> worker_;

    int currentKey_ = 0; // 모니터링용 인덱스

    // 1ms 이하의 주기마다 map을 보며 결과를 보여주고 해당 map의 쌍을 주기적으로 삭제하는 태스크 메소드
    // 단일 개별 스레드에서 async하게 동작함.
    // 옵션으로 해당 결과를 저장할 수도 있음

    void DequeueResult(const int taskId);

    void ProcessLoop();
};


}; // namespace PhaseBasedMotionMagification