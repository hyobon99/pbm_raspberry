#pragma once

#include <memory>
#include <queue>
#include <map>
#include <mutex>
#include <iostream>
#include <complex>
#include <opencv2/opencv.hpp>

#include "parameters.h"

// using Pixel = std::complex<float>;
using Real2d = std::vector<float>;
using Complex2d = std::vector<std::complex<float>>;
using Pyramid = std::vector<cv::Mat>;


namespace PhaseBasedMotionMagification {
class StorageManager {
public:
    // singleton
    static StorageManager& GetInstance() {
        static StorageManager instance; // thread-safe lazy initialization
        return instance;
    }

    StorageManager(const StorageManager&) = delete;
    StorageManager& operator=(const StorageManager&) = delete;

    // data
    Pyramid slicedFourier = Pyramid();

    const std::map<int, Pyramid>& getTimeSeries_Pyramid() {
        return TimeSeries_Pyramid;
    }
    const std::vector<cv::Mat>& getFrameYiq(int id) {
        // 현재는 값 변경 없이 참조만 하도록 되어 있지만, run 결과를 참조로 전달해주는 방법 고려해볼 것
        return Map_FrameYiq[id];
    }

    int EnqueuePyramid(const Pyramid& newSlicedFourier);
    void LogPyramidUsage(const int id);
    void DequeuePyramid(const int id);
    
    int EnqueueYiqFrame(const std::vector<cv::Mat>& newFrame);
    void DequeueYiqFrame(const int id);

private:
    StorageManager() = default;
    ~StorageManager() {
        TimeSeries_Pyramid.clear();
        ProcessMap_Pyramid.clear();
    }

    // data
    std::mutex iMtx_Pyramid;
    std::mutex iMtx_Pyramid_Usage;
    std::map<int, Pyramid> TimeSeries_Pyramid = std::map<int, Pyramid>();
    std::map<int, bool> ProcessMap_Pyramid;
    std::map<int, int> UsageCount_Pyramid; // 사용 횟수 카운트용
    int size_Pyramid = 0;
    int index_Pyramid = 0;

    std::mutex iMtx_FrameYiq;
    std::map<int, std::vector<cv::Mat>> Map_FrameYiq = std::map<int, std::vector<cv::Mat>>();
    std::map<int, bool> ProcessMap_FrameYiq;
    int size_FrameYiq = 0;
    int index_FrameYiq = 0;
};

}; // namespace PhaseBasedMotionMagification