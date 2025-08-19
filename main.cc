// 표준 라이브러리
#include <iostream>
#include <sstream>
#include <fstream>

//병렬처리 라이브러리
#include <thread>
#include <future>
#include <sstream>
#include <chrono>

#include "storage_manager.h"
#include "calculate_process.h"
#include "complex_steerable_pyramid.h"
#include "parameters.h"
#include "logging_manager.h"
#include "firwin_manager.h"
#include "parameters.h"

#include <opencv2/opencv.hpp>

#include <gpiod.h>
#include <cstdlib>

Real2d convertMatToVector(cv::Mat mat_float) {
    Real2d vec;
    vec.resize(mat_float.rows*mat_float.cols);
    
    #pragma omp parallel for
    for (int i = 0; i < mat_float.rows; ++i) {
        for (int j = 0; j < mat_float.cols; j++) {
            vec[i*mat_float.cols+j] =  mat_float.at<float>(i, j)/255;
        }
    }
    return vec;
}

std::vector<Real2d> convertBgrMatToYiqVector(cv::Mat mat) {
    const int rows = mat.rows;
    const int cols = mat.cols;
    const size_t total_pixels = static_cast<size_t>(rows) * cols;

    Real2d vec_Y, vec_I, vec_Q;
    vec_Y.reserve(total_pixels);
    vec_I.reserve(total_pixels);
    vec_Q.reserve(total_pixels);

    // isContinuous()는 행렬 데이터가 메모리에 한 블록으로 연속적으로 저장되어 있는지 확인합니다.
    // 만약 그렇다면, 전체 데이터를 1차원 배열처럼 순회하여 최고의 효율을 낼 수 있습니다.
    if (mat.isContinuous()) {
        const cv::Vec3b* p = mat.ptr<cv::Vec3b>(0);
        for (size_t i = 0; i < total_pixels; ++i) {
            const float px_b = p[i][0] / 255.0f;
            const float px_g = p[i][1] / 255.0f;
            const float px_r = p[i][2] / 255.0f;
            vec_Y.emplace_back(0.114f * px_b + 0.587f * px_g + 0.299f * px_r);
            vec_I.emplace_back(-0.322f * px_b - 0.274f * px_g + 0.596f * px_r);
            vec_Q.emplace_back(0.312f * px_b - 0.523f * px_g + 0.211f * px_r);
        }
    } else { // 행렬이 연속적이지 않은 경우 (예: 다른 행렬의 일부를 참조하는 ROI)
        for (int i = 0; i < rows; ++i) {
            const cv::Vec3b* p_row = mat.ptr<cv::Vec3b>(i);
            for (int j = 0; j < cols; ++j) {
                const cv::Vec3b& px_bgr = p_row[j];
                const float px_b = px_bgr[0] / 255.0f;
                const float px_g = px_bgr[1] / 255.0f;
                const float px_r = px_bgr[2] / 255.0f;
                vec_Y.emplace_back(0.114f * px_b + 0.587f * px_g + 0.299f * px_r);
                vec_I.emplace_back(-0.322f * px_b - 0.274f * px_g + 0.596f * px_r);
                vec_Q.emplace_back(0.312f * px_b - 0.523f * px_g + 0.211f * px_r);
            }
        }
    }
    return std::vector<Real2d> {std::move(vec_Y), std::move(vec_I), std::move(vec_Q),};
}

std::vector<cv::Mat> converBgrMatToYiqMat(const cv::Mat& mat) {
    std::vector<cv::Mat> mat_YIQ(3);
    mat_YIQ[0] = cv::Mat(mat.rows, mat.cols, CV_32F);
    mat_YIQ[1] = cv::Mat(mat.rows, mat.cols, CV_32F);
    mat_YIQ[2] = cv::Mat(mat.rows, mat.cols, CV_32F);

    #pragma omp parallel for
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; j++) {
            cv::Vec3b px_bgr = mat.at<cv::Vec3b>(i, j);
            float px_b = px_bgr[0] / 255.0f;
            float px_g = px_bgr[1] / 255.0f;
            float px_r = px_bgr[2] / 255.0f;
            mat_YIQ[0].at<float>(i, j) = 0.114f * px_b + 0.587f * px_g + 0.299f * px_r;
            mat_YIQ[1].at<float>(i, j) = -0.322f * px_b - 0.274f * px_g + 0.596f * px_r;
            mat_YIQ[2].at<float>(i, j) = 0.312f * px_b - 0.523f * px_g + 0.211f * px_r;
        }
    }
    return mat_YIQ;
}

cv::Mat convertVectorToMat(const Real2d& mat, int rows, int cols) {
    std::cout<<"mat size: "<<mat.size()<<std::endl;
    cv::Mat cv_mat(rows, cols, CV_8UC1);

    #pragma omp parallel for
    for (int i = 0; i < mat.size(); i++) {
        int pxVal = (int)(mat[i]*255);
        // cv_mat.data[i] = static_cast<uchar>(std::clamp(pxVal, 0, 255));
        cv_mat.at<uchar>(i/cols, i%cols) = static_cast<uchar>(std::clamp(pxVal, 0, 255));
    }

    return cv_mat;
}

cv::Mat convertFloatMatToUint8Mat(const cv::Mat& mat_float) {
    cv::Mat cv_mat(mat_float.rows, mat_float.cols, CV_8UC1);

    #pragma omp parallel for
    for (int i = 0; i < mat_float.rows; ++i) {
        for (int j = 0; j < mat_float.cols; j++) {
            int pxVal = static_cast<int>(mat_float.at<float>(i, j) * 255);
            cv_mat.at<uchar>(i, j) = static_cast<uchar>(std::clamp(pxVal, 0, 255));
        }
    }
    return cv_mat;
}

cv::Mat convertYiqVectorToBgrMat(const std::vector<Real2d>& mat_yiq, int rows, int cols) {
    cv::Mat cv_mat(rows, cols, CV_8UC3);

    #pragma omp parallel for
    for (int i = 0; i < mat_yiq[0].size(); i++) {
        cv::Vec3b pxVal;

        pxVal[0] = static_cast<uchar>(std::clamp((int)(255 * (mat_yiq[0][i] - 1.106f * mat_yiq[1][i] + 1.703f * mat_yiq[2][i])), 0, 255));
        pxVal[1] = static_cast<uchar>(std::clamp((int)(255 * (mat_yiq[0][i] - 0.272f * mat_yiq[1][i] - 0.647f * mat_yiq[2][i])), 0, 255));
        pxVal[2] = static_cast<uchar>(std::clamp((int)(255 * (mat_yiq[0][i] + 0.956f * mat_yiq[1][i] + 0.621f * mat_yiq[2][i])), 0, 255));
        
        cv_mat.at<cv::Vec3b>(i/cols, i%cols) = pxVal;
    }

    return cv_mat;
}

cv::Mat convertYiqMatToBgrMat(const std::vector<cv::Mat>& mat_yiq, int rows, int cols) {
    cv::Mat cv_mat(rows, cols, CV_8UC3);

    #pragma omp parallel for
    for (int i = 0; i < mat_yiq[0].rows * mat_yiq[0].cols; i++) {
        cv::Vec3b pxVal;

        pxVal[0] = static_cast<uchar>(std::clamp((int)(255 * (mat_yiq[0].at<float>(i / cols, i % cols) - 1.106f * mat_yiq[1].at<float>(i / cols, i % cols) + 1.703f * mat_yiq[2].at<float>(i / cols, i % cols))), 0, 255));
        pxVal[1] = static_cast<uchar>(std::clamp((int)(255 * (mat_yiq[0].at<float>(i / cols, i % cols) - 0.272f * mat_yiq[1].at<float>(i / cols, i % cols) - 0.647f * mat_yiq[2].at<float>(i / cols, i % cols))), 0, 255));
        pxVal[2] = static_cast<uchar>(std::clamp((int)(255 * (mat_yiq[0].at<float>(i / cols, i % cols) + 0.956f * mat_yiq[1].at<float>(i / cols, i % cols) + 0.621f * mat_yiq[2].at<float>(i / cols, i % cols))), 0, 255));

        cv_mat.at<cv::Vec3b>(i/cols, i%cols) = pxVal;
    }

    return cv_mat;
}

cv::Mat convertVectorToMat(Complex2d mat, int rows, int cols) {
    
    cv::Mat cv_mat(rows, cols, CV_32F);

    #pragma omp parallel for
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            cv_mat.at<float>(i, j) = (float)mat[i*cols +j].real();
        }
    }
    return cv_mat;
}

Complex2d expandToComplex(Real2d mat) {
    Complex2d result;
    result.resize(mat.size());
    for(int i; i<mat.size(); i++){
        result[i] = {mat[i], 0.0};
    }
    return result;
}

void saveAsTiff (const Complex2d& mat, std::string fileName, int rows, int cols, bool bPickReal) {
    cv::Mat dst(rows, cols, CV_32F);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            dst.at<float>(i, j) = bPickReal ? (float)mat[i * cols + j].real() : (float)mat[i * cols + j].imag();
        }
    }
    cv::imwrite(fileName, dst);
}

void saveAsTiff (const Real2d& mat, std::string fileName, int rows, int cols) {
    cv::Mat dst(rows, cols, CV_32F);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            dst.at<float>(i, j) = (float)mat[i * cols + j];
        }
    }
    
    cv::imwrite(fileName, dst);
}

int main() {
    std::cout << "Starting program..." << std::endl;  // 오류 스트림으로 프로그램 시작 확인
    // std::cout << cv::getBuildInformation() << std::endl;  // OpenCV 빌드 정보 출력

    std::cout.flush();  // 버퍼 비우기

    auto& storage_manager = PhaseBasedMotionMagification::StorageManager::GetInstance();

    # if N_TRIGGERING_TYPE != 0
        // const char* chip_name = _chip_name.c_str(); // inline std::string으로 정의되어있을 때
        const char* chip_name = _chip_name;
        unsigned int line_offset = _line_offset;

        gpiod_chip* chip = gpiod_chip_open_by_name(chip_name);
        gpiod_line* line = gpiod_chip_get_line(chip, line_offset);

        gpiod_line_request_both_edges_events(line, "gpio_event");

        std::cout << "Waiting for trigger..." << std::endl;
        while (true)
        {   
            // Event based Checkout
            struct gpiod_line_event event;
            int ret = gpiod_line_event_wait(line, nullptr);
            // if (ret > 0) {
            //     if (gpiod_line_event_read(line, &event) == 0) {
            //         if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
            //             std::cout << "Pulse high detected" << std::endl;
            //         else if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE)
            //             std::cout << "Pulse low detected" << std::endl;
            //     }
            //     break;
            // }
            if (ret > 0) {
                if (gpiod_line_event_read(line, &event) == 0) {
                    if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
                        std::cout << "Pulse low detected. Starting process..." << std::endl;
                        break;
                    }
                }
            }
        }

        gpiod_line_release(line);
        gpiod_chip_close(chip);

        // pi에서 내용 참고해서 마저 작성할 것
    #endif

    // Video Load
    #if B_MONITORING_FOR_CAM
        // 카메라 초기화
        cv::VideoCapture cap(0);
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open camera device" << std::endl;
            return -1;
        }
        
        // 카메라 설정
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap.set(cv::CAP_PROP_FPS, 30);
        
        std::cout << "Camera opened successfully!" << std::endl;
        std::cout << "Press 'q' to quit, 's' to save frame" << std::endl;
        
        // 윈도우 생성
        cv::namedWindow("Phase-Based Motion Magnification - Real-time", cv::WINDOW_AUTOSIZE);
    #else
        std::stringstream ss;
        ss<<videoFileDirectory<<videoFileName;    
        std::cout << "Video file path: " << ss.str() << std::endl;
        cv::VideoCapture cap(
            ss.str() //, cv::CAP_FFMPEG
        );
        
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open the video file." << std::endl;
            return -1;
        } else {
            std::cout << "Succeeded to open the video file." << std::endl;
        }    
    #endif
    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    // // Fir Filter initialization
    // PhaseBasedMotionMagification::FirWinManager firwin_manager;
    // std::vector<float> b_fir = firwin_manager.bandpass_fir_kernel(N_LOOPRANGE, 2 * (253.0f / 2000), 2 * (257.0f / 2000), PhaseBasedMotionMagification::E_FILTER_TYPE::FIR_BANDPASS);
    // std::cout << b_fir[0] << ", "
    //          << b_fir[1] << ", "
    //         << b_fir[2] << ", "
    //         << b_fir[3] << ", "
    //         << b_fir[4] << std::endl;

    // Complex Steerable Pyramid Initialization
    auto* complexSteerablePyramid = new PhaseBasedMotionMagification::ComplexSteerablePyramid(width, height);
    auto Pyr = complexSteerablePyramid->get_pyr();
    auto PyrIdx = complexSteerablePyramid->get_pyr_idx();
    
    auto PyrSpatial = complexSteerablePyramid->get_pyr_spatial();
    // // (임시) csp 실물을 직접 저장
    // for (int i = 0; i < Pyr.size(); i++) {
    //     auto filter = Pyr[i];
    //     std::vector<cv::Mat> filter_split;
    //     cv::split(filter, filter_split);
        
    //     ss.str("");
    //     ss << LogPath << "csp_lv_" << i << "_real.tiff";
    //     // saveAsTiff(filter, ss.str(), height, width, true);
    //     cv::imwrite(ss.str(), filter_split[0]);

    //     ss.str("");
    //     ss << LogPath << "csp_lv_" << i << "_imag.tiff";
    //     cv::imwrite(ss.str(), filter_split[1]);
    // }
    // std::cout << "CSP spatial domain image saved." << std::endl;

    // Lowpass의 경우 미리 Pyr**2를 해둠.
    int level = N_PYRAMID_LEVEL * N_ORIENTATION + 2 - 1;
    
    cv::multiply(Pyr[level], Pyr[level], Pyr[level]);

    // CalculateProcess initialization
    auto* calculateProcess = new PhaseBasedMotionMagification::CalculateProcess(Pyr, PyrIdx);
    
    cv::Mat frame;
    cv::Mat frame_Real2d;

    cv::Mat pmm_result_mat;

    std::vector<std::future<std::pair<int, cv::Mat>>> futures; // for parallel case
    std::pair<int, cv::Mat> pmm_result; // for serial case

    std::cout << "Loop Ready." << std::endl;
    
    // FPS 계산을 위한 변수들
    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int t = 0; t < N_TESTING_FRAME; t++) {
        
        // 비디오에서 한 프레임을 읽어옴
        cap >> frame;

        // 프레임을 읽을 수 없으면 종료
        if (frame.empty()) {
            std::cout << "End of video or error while reading frame." << std::endl;
            break;
        }
        
        frame_count++;
        
        #if B_MONITORING_FOR_CAM
        // FPS 계산
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        double fps = frame_count / (elapsed_time.count() / 1000.0);
        
        // 프레임 정보 표시
        std::string info_text = "Frame: " + std::to_string(frame_count) + 
                               ", FPS: " + std::to_string(static_cast<int>(fps)) + 
                               ", Size: " + std::to_string(frame.cols) + "x" + std::to_string(frame.rows);
        
        cv::putText(frame, info_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        #endif

        // 프레임 vector화
        std::vector<cv::Mat> frame_yiq = converBgrMatToYiqMat(frame);
        frame_Real2d = frame_yiq[0];

        auto prepared_pyr = calculateProcess->prepare_variables_to_push(frame_Real2d, t);
        
        // 작업: Enqueue
        int TaskId = storage_manager.EnqueuePyramid(prepared_pyr);
        #if B_RUNNING_PARALLEL
            int TaskId_Yiq = storage_manager.EnqueueYiqFrame(frame_yiq);
        #endif

        #if !B_RUNNING_PARALLEL
            pmm_result = calculateProcess->run(TaskId, frame_Real2d);

            //[test]frame_Real2d -> pmm_result.second
            // pmm_result.second = frame_Real2d;
        #else
            auto run_cal_process = [&](int TaskId) -> std::pair<int, cv::Mat> {
                return calculateProcess->run(TaskId, frame_Real2d);
            };
            futures.push_back(std::async(std::launch::async, run_cal_process, TaskId));
        #endif

        #if !B_RUNNING_PARALLEL
        if (t >= N_LOOPRANGE) {     
            frame_yiq[0] = pmm_result.second;
            // pmm_result_mat = convertYiqVectorToBgrMat(frame_yiq, height, width);
            pmm_result_mat = convertYiqMatToBgrMat(frame_yiq, height, width);

            cv::imshow("Result", pmm_result_mat);
            
            // 키 입력 처리
            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') {
                break;
            } else if ((key == 's' || key == 'S') && B_MONITORING_FOR_CAM) {
                // 프레임 저장 (카메라 모드에서만)
                std::string filename = "pmm_frame_" + std::to_string(frame_count) + ".jpg";
                cv::imwrite(filename, pmm_result_mat);
                std::cout << "Saved: " << filename << std::endl;
            }
            // storage_manager.DequeuePyramid(pmm_result.first - N_LOOPRANGE);
        }
        #endif
        
    }

    /// parellel here
    #if B_RUNNING_PARALLEL
    for (auto& future : futures) {
        auto pmm_result = future.get();
        // pmm_result_mat = convertEigenToMat(pmm_result.second, height, width);
            
        std::cout << "Result size: " << pmm_result.second.size() << std::endl;
        auto frame_yiq_orig = storage_manager.getFrameYiq(pmm_result.first);
        // pmm_result_mat = convertFloatMatToUint8Mat(pmm_result.second);
        auto pmm_result_mat_gray = pmm_result.second;
        frame_yiq_orig[0] = pmm_result_mat_gray;
        auto pmm_result_mat = convertYiqMatToBgrMat(frame_yiq_orig, height, width);
        
        cv::imshow("Result", pmm_result_mat);

        // 키 입력 처리
        int key = cv::waitKey(100);
        if (key == 'q' || key == 'Q') {
            break;
        } else if ((key == 's' || key == 'S') && B_MONITORING_FOR_CAM) {
            // 프레임 저장 (카메라 모드에서만)
            std::string filename = "pmm_frame_" + std::to_string(pmm_result.first) + ".jpg";
            cv::imwrite(filename, pmm_result_mat);
            std::cout << "Saved: " << filename << std::endl;
        }

        #if B_SAVING_FRAMES
            std::ostringstream oss;
            oss << "./frames_par/result_" << std::setw(3) << std::setfill('0') << pmm_result.first << ".jpg";
            cv::imwrite(oss.str(), pmm_result_mat);
        #endif

        // storage_manager.DequeuePyramid(pmm_result.first);
        storage_manager.DequeueYiqFrame(pmm_result.first);
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    #endif
    
    cap.release();
    cv::destroyAllWindows();

    #if B_MONITORING_FOR_CAM
    // 카메라 모드에서 통계 정보 출력
    auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start_time);
    double avg_fps = frame_count / (total_time.count() / 1000.0);
    
    std::cout << "Total frames captured: " << frame_count << std::endl;
    std::cout << "Average FPS: " << avg_fps << std::endl;
    #endif

    auto& logging_manager = PhaseBasedMotionMagification::LoggingManager::GetInstance();
    logging_manager.SummarizeResults();

    return 0;
}
