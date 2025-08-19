#pragma once

#include "storage_manager.h"
#include "parameters.h"
#include "logging_manager.h"

#include <unordered_map>
#include <chrono>
#include <omp.h>

// #include <opencv4/opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <queue>
#include <fftw3.h>
#include <algorithm>
#include <complex>
#include <unistd.h>
#include <fstream>

namespace PhaseBasedMotionMagification {

class StorageManager;   // load data from storage 

class CalculateProcess {
public:
    CalculateProcess(const Pyramid& inPyr, const std::vector<std::vector<int>>& inPyrIdx)
    : nLevels(N_ORIENTATION * N_PYRAMID_LEVEL + 2),
      LoopRange(N_LOOPRANGE),
      Alpha(10.0f),
      FirFilter FIR_FILTER,
      Pyr(inPyr),
      PyrIdx(inPyrIdx){
        size.clear();
        size.emplace_back(PyrIdx[0][3] - PyrIdx[0][2]); // 0: width
        size.emplace_back(PyrIdx[0][1] - PyrIdx[0][0]); // 1: height
        
        auto& logging_manager = PhaseBasedMotionMagification::LoggingManager::GetInstance();
        logging_manager.UpdateFrameInfo(size[0], size[1]);
        
        omp_set_num_threads(8);
    }

    ~CalculateProcess() {
    
    }

    std::pair<int, cv::Mat> run(int TaskId, cv::Mat frame_vectorized);
    cv::Mat expand_to_complex(const cv::Mat& mat);
    Pyramid prepare_variables_to_push(const cv::Mat& mat, int TaskId);

    
private:
    int nLevels;
    int LoopRange;
    float Alpha;
    std::vector<float> FirFilter;
    const Pyramid& Pyr;
    const std::vector<std::vector<int>>& PyrIdx;
    std::vector<int> size;
        
    inline void LogTimeStamp(int TaskId, int level, int orientation, E_LOG_PROPERTIES prop);
    
    cv::Mat calc_fft_2d_opencv(const cv::Mat& mat);
    cv::Mat calc_ifft_2d_opencv(const cv::Mat& mat);
    

    // inline Complex2d calc_phase_angle(const Complex2d& mat);

    inline std::vector<cv::Mat> calc_phase_difference_queue(const std::map<int, Pyramid>& timeSeries, const cv::Mat& RefFrame, int level, int TaskId);
    inline cv::Mat temporal_fir_filtering(const std::vector<cv::Mat>& XQueue);
    inline cv::Mat calc_exponential_imaginary(const cv::Mat& mat);
    inline cv::Mat cv_complex_multiply(const cv::Mat& mat1, const cv::Mat& mat2, cv::Mat& dst);

    inline float floor_mod_float(float a, float b);
    void saveAsTiff (const cv::Mat& mat, std::string fileName, int rows, int cols); 
    inline float fast_atan2(float y, float x);
    inline float fast_cos(float x);
    inline float fast_sin(float x);
    inline float wrap_angle(float x);

    size_t getCurrentRss();
};

};  // namespace PhaseBasedMotionMagification