#include "streaming_manager.h"

namespace PhaseBasedMotionMagification {
    void PhaseBasedMotionMagification::StreamingManager::start() {
         worker_ = std::async(std::launch::async, &StreamingManager::ProcessLoop, this);
    }
    
    void PhaseBasedMotionMagification::StreamingManager::stop() {
        stopFlag_ = true;
        if (worker_.valid()) {
            worker_.get();
        }
    }

    int PhaseBasedMotionMagification::StreamingManager::EnqueueResult(const cv::Mat& result) {
        
    }

    void PhaseBasedMotionMagification::StreamingManager::DequeueResult(const int taskId) {
        
    }

    void PhaseBasedMotionMagification::StreamingManager::ProcessLoop()  {
        while (!stopFlag_) {
            bool processed = false;

            {
                std::lock_guard<std::mutex> lock(iMtx_ResultsMap);
                auto it = ResultsMap.find(currentKey_);
                if (it != ResultsMap.end()) {
                    cv::Mat img = it->second;

                    // 1) 결과 표시
                    cv::imshow("Streaming Monitor", img);
                    cv::waitKey(1);

                    // 2) 저장 옵션
                    #if B_SAVING_FRAMES
                        std::string filename = "result_" + std::to_string(currentKey_) + ".png";
                        cv::imwrite(filename, img);
                    #endif

                    // 3) 현재 key 삭제 + key 증가
                    ResultsMap.erase(it);
                    currentKey_++;
                    processed = true;
                }
            }

            // 이미지가 처리되지 않았다면 잠깐 쉬고 재시도
            // if (!processed) {
            //     std::this_thread::sleep_for(std::chrono::microseconds(200));
            // }
        }
    }


}; // namespace PhaseBasedMotionMagification