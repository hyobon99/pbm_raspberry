#pragma once
#include <vector>
#include <memory>
#include <complex>

#include "storage_manager.h"
#include "parameters.h"
#include <opencv2/opencv.hpp>

namespace PhaseBasedMotionMagification {

class ComplexSteerablePyramid {
public:
    std::vector<int> size;
    int nPyramids;
    int nOrientations;
    float twidth;

    ComplexSteerablePyramid(int width, int height)
    : nPyramids(N_PYRAMID_LEVEL), 
      nOrientations(N_ORIENTATION), 
      twidth(1.0)
    {
        size.clear();
        size.emplace_back(width);
        size.emplace_back(height);

        generate_csp();
    }

    const Pyramid& get_pyr() {
        return Pyr_Cropped;
    }

    const std::vector<std::vector<int>>& get_pyr_idx() {
        return PyrIdx;
    }

    const Pyramid& get_pyr_spatial() {
        return Pyr_Spatial;
    }

private:
    Pyramid Pyr_Cropped;
    Pyramid Pyr_Uncropped;
    std::vector<std::vector<int>> PyrIdx;
    Pyramid Pyr_Spatial;

    void generate_csp();    
    std::vector<cv::Mat> generate_polar_grid();
    std::vector<cv::Mat> calc_radial_mask_pair(float r, cv::Mat rad);
    cv::Mat calc_angle_mask(int b, cv::Mat angle);
    std::vector<cv::Mat> generate_csf(std::vector<float> rVals);
    void crop_csp(const std::vector<cv::Mat>& filters, const std::vector<float>& rVals);
        
    cv::Mat calc_log_rad(cv::Mat mat, float a);
    cv::Mat cut_off_upper_values(cv::Mat mat, float a);
    cv::Mat cut_off_lower_values(cv::Mat mat, float a);

    int factorial(int n);
    float calc_sum_of_elements(const std::vector<float>& arr, int n);
    float floor_mod_float(float a, float b);

    cv::Mat slice_Real2d(const cv::Mat& mat, int lb_y, int ub_y, int lb_x, int ub_x);
    cv::Mat expand_to_complex(const cv::Mat& mat);
    void saveAsTiff (cv::Mat mat, std::string fileName); 
};

};  // namespace PhaseBasedMotionMagification