#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <complex>

#include "complex_steerable_pyramid.h"


namespace PhaseBasedMotionMagification {
    void ComplexSteerablePyramid::generate_csp() {
        std::vector<float> rVals(nPyramids+1);
        for (int i = 0; i < rVals.size(); i++) {
            rVals[i] = pow(0.5, i);
        }
        

        auto filters = generate_csf(rVals);
        for (auto filter : filters) {
            Pyr_Uncropped.emplace_back(filter);
        }
        
        crop_csp(filters, rVals);
        
    }

    std::vector<cv::Mat> ComplexSteerablePyramid::generate_polar_grid() {
        cv::Mat xVals(1, size[0], CV_32F);
        cv::Mat yVals(size[1], 1, CV_32F);
        for (int i = 0; i < size[0]; ++i)
            xVals.at<float>(0, i) = -1.0f + i * (2.0f / size[0]);
        for (int j = 0; j < size[1]; ++j)
            yVals.at<float>(j, 0) = -1.0f + j * (2.0f / size[1]);

        cv::Mat xRamp, yRamp;
        cv::repeat(xVals, size[1], 1, xRamp); // rows, cols, out
        cv::repeat(yVals, 1, size[0], yRamp);

        cv::Mat angle(xRamp.size(), CV_32F);
        cv::Mat rad(xRamp.size(), CV_32F);
        for (int y = 0; y < xRamp.rows; ++y) {
            for (int x = 0; x < xRamp.cols; ++x) {
                float xv = xRamp.at<float>(y, x);
                float yv = yRamp.at<float>(y, x);
                angle.at<float>(y, x) = std::atan2(yv, xv) + static_cast<float>(CV_PI) / 2.0f;
                rad.at<float>(y, x) = std::sqrt(xv * xv + yv * yv);
            }
        }

        return std::vector<cv::Mat> {angle, rad};

    }   
    
    std::vector<cv::Mat> ComplexSteerablePyramid::calc_radial_mask_pair(float r, cv::Mat rad) {
        // cv::Mat logRad = calc_log_rad(rad, r);
        cv::Mat hiMask = calc_log_rad(rad, r);
        
        hiMask = cut_off_upper_values(hiMask, 0);
        hiMask = cut_off_lower_values(hiMask, -twidth);

        // std::stringstream ss;
        // ss.str(""); ss << LogPath << "log_rad_cut.tiff";
        // cv::imwrite(ss.str(), hiMask);

        cv::multiply(hiMask, M_PI_f / (2 * twidth), hiMask);   
        hiMask.forEach<float>([](float& val, const int*) {
            val = std::cos(val);
        });

        cv::Mat loMask;
        cv::multiply(hiMask, hiMask, hiMask);
        cv::subtract(1.0f, hiMask, loMask);
        cv::sqrt(loMask, loMask);
        // cv::Mat loMask = (1.0f - hiMask.square()).sqrt();
        
        return std::vector<cv::Mat> {hiMask, loMask};
    }
    cv::Mat ComplexSteerablePyramid::calc_angle_mask(int b, cv::Mat angle) {
        int order = nOrientations - 1;
        float constant = (pow(2, 2 * order) * pow(factorial(order), 2))
                        / (nOrientations * factorial(2 * order));
        
        cv::Mat adjustedAngle;
        cv::add(angle, (M_PI_f - M_PI_f * (b - 1) / nOrientations), adjustedAngle);

        cv::Mat angleMask = cv::Mat::zeros(adjustedAngle.size(), CV_32F);
        
        angleMask.forEach<float>([&](float& out, const int* pos) {
            float v = adjustedAngle.at<float>(pos[0], pos[1]);
            float val = floor_mod_float(v, (M_PI_f * 2)) - M_PI_f;
            if (std::abs(val) < M_PI_f / 2.0f) {
                out = 2.0f * std::sqrt(constant) * std::pow(std::cos(val), order);
            } else {
                out = 0.0f;
            }
        });
        return angleMask;
    }
    std::vector<cv::Mat> ComplexSteerablePyramid::generate_csf(std::vector<float> rVals){
        std::vector<cv::Mat> polarGrid = generate_polar_grid();
        cv::Mat angle = polarGrid[0];
        cv::Mat rad = polarGrid[1];
        // saveAsTiff(angle, "angle.tiff", 256, 640);
        // saveAsTiff(rad, "rad.tiff", 256, 640);

        std::vector<cv::Mat> masks = calc_radial_mask_pair(1.0, rad);
        cv::Mat hiMask = masks[0];
        cv::Mat loMaskPrev = masks[1];
        // saveAsTiff(hiMask, "himask.tiff", 256, 640);
        // saveAsTiff(loMaskPrev, "lomaskPrev.tiff", 256, 640);

        std::stringstream ss;
        ss.str(""); ss << LogPath << "loMaskPrev.tiff";
        cv::imwrite(ss.str(), loMaskPrev);
        ss.str(""); ss << LogPath << "hiMask.tiff";
        cv::imwrite(ss.str(), hiMask);
        
    
        std::vector<cv::Mat> filters;
        filters.push_back(
            expand_to_complex(hiMask)
        );
    
        for (int k = 1; k < rVals.size(); k++) {
            
            masks = calc_radial_mask_pair(rVals[k], rad);
            cv::Mat currentHiMask = masks[0];
            cv::Mat currentLoMask = masks[1];
    
            cv::Mat radMask;
            cv::multiply(currentHiMask, loMaskPrev, radMask);
            // saveAsTiff(radMask, "radmask.tiff", (int)(256*rVals[k]), (int)(640*rVals[k]));

            
            for (int j = 1; j <= nOrientations; j++) {
                cv::Mat angleMask = calc_angle_mask(j, angle);
                cv::Mat combined;
                cv::multiply(angleMask, radMask, combined);
                cv::multiply(combined, 0.5f, combined);
                
                filters.push_back(
                    expand_to_complex(combined)
                );
            }
            loMaskPrev = currentLoMask;
        }
        filters.push_back(
            expand_to_complex(loMaskPrev)
        );
        return filters;
    }

    void ComplexSteerablePyramid::crop_csp(const std::vector<cv::Mat>& filters, const std::vector<float>& rVals) {
        int ydim = size[1];
        int xdim = size[0];
        int nFilters = filters.size();       
    
        // Initialize indices
        PyrIdx = std::vector<std::vector<int>>(0);
        for (int i = 0; i < nFilters; i++) {
            PyrIdx.push_back(std::vector<int> (4));
        }
        
        // First filter
        PyrIdx[0][0] = 0;
        PyrIdx[0][1] = ydim;
        PyrIdx[0][2] = 0;
        PyrIdx[0][3] = xdim;
        Pyr_Cropped.emplace_back(filters[0]);
    
        // Middle filters
        for (int k = 1; k < nFilters - 1; k += nOrientations) {
            int n = (k / nOrientations) + 1;
            float sumR = calc_sum_of_elements(rVals, n);
            
            int lbY = (int) ((ydim * (sumR - 1)) / 2);
            int ubY = ydim - lbY;
            int lbX = (int) ((xdim * (sumR - 1)) / 2);
            int ubX = xdim - lbX;

            for (int i = 0; i < nOrientations; i++) {
                int idx = k + i;
                PyrIdx[idx][0] = lbY;
                PyrIdx[idx][1] = ubY;
                PyrIdx[idx][2] = lbX;
                PyrIdx[idx][3] = ubX;
            }
    
            for (int i = 0; i < nOrientations; i++) {
                int idx = k + i;
                
                cv::Mat cropped = filters[idx](cv::Rect(lbX, lbY, ubX - lbX, ubY - lbY));
                Pyr_Cropped.emplace_back(cropped);
            }
        }
    
        // Last filter
        float totalSumR = calc_sum_of_elements(rVals, rVals.size());
        int lbY = (int) ((ydim * (totalSumR - 1)) / 2);
        int ubY = ydim - lbY;
        int lbX = (int) ((xdim * (totalSumR - 1)) / 2);
        int ubX = xdim - lbX;
    
        int lastIdx = nFilters - 1;
        PyrIdx[lastIdx][0] = lbY;
        PyrIdx[lastIdx][1] = ubY;
        PyrIdx[lastIdx][2] = lbX;
        PyrIdx[lastIdx][3] = ubX;
        
        Pyr_Cropped.emplace_back(
            filters[lastIdx](cv::Rect(lbX, lbY, ubX - lbX, ubY - lbY))
        );
    }

    cv::Mat ComplexSteerablePyramid::calc_log_rad(cv::Mat mat, float a) {
        // cv::Mat log_val = (mat / a).log() / std::log(2.0f); // log2(x) = log(x)/log(2)
        // log_val = log_val.unaryExpr([](float v) {
        //     return (v == -std::numeric_limits<float>::infinity()) ? -54.0f : v;
        // });
        // return log_val;

        cv::Mat log_val = mat.clone();
        // cv::multiply(mat, 1.0f/a, log_val);
        log_val.forEach<float>([&](float& out, const int* pos) {
            float val = std::log2(out) - std::log2(a);// log2(x) = log(x)/log(2)
            if (val == -std::numeric_limits<float>::infinity()) {
                out = -54.0f;
            } else {
                // std::cout << "v: " << v << ", a: " << a << ", val: " << val << std::endl;
                out = val;
            }
        });
        
        return log_val;
    }

    cv::Mat ComplexSteerablePyramid::cut_off_lower_values(cv::Mat mat, float a) {
        // return mat.unaryExpr([a](float v) { return (v < a) ? a : v; });
        cv::Mat low_cut = cv::max(mat, a);
        return low_cut;
    }

    cv::Mat ComplexSteerablePyramid::cut_off_upper_values(cv::Mat mat, float a) {
        // return mat.unaryExpr([a](float v) { return (v > a) ? a : v; });
        cv::Mat hi_cut = cv::min(mat, a);
        return hi_cut;
    }

    int ComplexSteerablePyramid::factorial(int n) {
        int result = 1;
        int i = 1;
        while (i <= n) {
            result *= i;
            i++;
        }
        return result;
    }

    float ComplexSteerablePyramid::calc_sum_of_elements(const std::vector<float>& arr, int n) {
        float sum = 0;
        for (int i = 0; i < n; i++) {
            sum += arr[i];
        }
        return sum;
    }
    
    float ComplexSteerablePyramid::floor_mod_float(float a, float b) {
        return fmodf(fmodf(a, b) + b, b);
    }

    cv::Mat ComplexSteerablePyramid::expand_to_complex(const cv::Mat& mat) {
        if (mat.channels() == 1) {
            cv::Mat complexMat;
            cv::Mat realPart = mat.clone();
            cv::Mat imagPart = mat.clone();
            cv::merge(std::vector<cv::Mat>{realPart, imagPart}, complexMat);
            return complexMat;
        } else {
            return mat; // Already complex
        }
    }
    void ComplexSteerablePyramid::saveAsTiff (cv::Mat mat, std::string fileName) {
        // int rows = mat.rows();
        // int cols = mat.cols();
        
        // cv::Mat dst(rows, cols, CV_32F);
        // for (int i = 0; i < rows; i++) {
        //     for (int j = 0; j < cols; j++) {
        //         dst.at<float>(i, j) = mat(i, j);
        //     }
        // }
        // cv::imwrite(fileName, dst);

        cv::imwrite(fileName, mat);
    }

};  // PhaseBasedMotionMagification