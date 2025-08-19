#include "calculate_process.h"

namespace PhaseBasedMotionMagification {

    inline void CalculateProcess::LogTimeStamp(int taskId, int level, int orientation, E_LOG_PROPERTIES prop) {
        PhaseBasedMotionMagification::LoggingManager::GetInstance()
            .LogTimeStamp({taskId, level, orientation, prop}, std::chrono::steady_clock::now());
    }

    std::pair<int, cv::Mat> CalculateProcess::run(int TaskId, cv::Mat frame_vectorized) {
        // std::stringstream ss;
        
        // TODO: Enqueue pyramid시점에서 frame_vectorized를 미리 넣어줄 수 있는데, 굳이 여기서 다시 넘겨받아야할지? 
        //  storage_manager에 EnqueueYiqFrame 만들어 뒀으니, 이를 이용하는 방식을 생각해둘 것
        // TODO: 각 tensor 연산은 pragma parallel for로 함수 자체를 만들고, 각 함수는 level별 thread로 던지기 해볼 것

        if (TaskId >= LoopRange) {
            LogTimeStamp(TaskId, 0, 0, RUN_START);
            std::cout<<"main task started, ID: "<<TaskId<<std::endl;

            cv::Mat MagnifiedFreq(size[1], size[0], CV_32FC2, cv::Scalar(0.0f, 0.0f));

            auto& storage_manager = StorageManager::GetInstance();
            auto& timeSeries_Pyramid = storage_manager.getTimeSeries_Pyramid();

            LogTimeStamp(TaskId, 0, 0, RUN_INTIALIZE_PTR_AND_VARS);
            cv::Mat VidFft_Last = calc_fft_2d_opencv(frame_vectorized);
            LogTimeStamp(TaskId, 0, 0, RUN_FFT2_CURRENT_FRAME);


            #if B_USING_PRAGMA_OMP_PARALLEL
            #pragma omp parallel for
            #endif
            for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                for (int ori = 0; ori < N_ORIENTATION; ori++) {
                    int level = lv * N_ORIENTATION + ori + 1;
                    int lb_y = PyrIdx[level][0];
                    int ub_y = PyrIdx[level][1];
                    int lb_x = PyrIdx[level][2];
                    int ub_x = PyrIdx[level][3];
                    
                    cv::Mat VidFft_Filtered;
                    cv::multiply(
                        VidFft_Last(cv::Rect(lb_x, lb_y, ub_x - lb_x, ub_y - lb_y)),
                        Pyr[level], 
                        VidFft_Filtered
                    );
                    LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP);
                    cv::Mat OrigFrame = calc_ifft_2d_opencv(VidFft_Filtered);
                    LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_IFFT2_LEVEL_WISE);

                    #if B_PREPARING_LAZY
                        Eigen::ArrayXXcf RefFouriered = timeSeries_Pyramid[TaskId - LoopRange][level];   
                        #if N_FFT_LIB_TYPE == 0
                            // Eigen::ArrayXXcf RefFiltered = calc_ifft_2d_thread_local(RefFouriered, ub_y - lb_y, ub_x - lb_x);
                            Eigen::ArrayXXcf RefFiltered = calc_ifft_2d_using_plan_map_pipelined(
                                RefFouriered, 
                                level, N_PYRAMID_LEVEL + 1 + (level + 1) / 2, TaskId
                            );
                        #elif N_FFT_LIB_TYPE == 1
                            Eigen::ArrayXXcf RefFiltered = calc_ifft_2d_pocketfft(RefFouriered, ub_y - lb_y, ub_x - lb_x);
                        #elif N_FFT_LIB_TYPE == 2
                            Eigen::ArrayXXcf RefFiltered = calc_ifft_2d_using_cufft_plan_map(
                                RefFouriered, 
                                level, N_PYRAMID_LEVEL + 1 + (level + 1) / 2, TaskId
                            );
                        #endif
                        Eigen::ArrayXXcf RefFrame = calc_phase_angle(RefFiltered);
                    
                    #else
                        // auto aa = timeSeries_Pyramid[TaskId - LoopRange];
                        // cv::Mat RefFrame = aa[level];
                        // cv::Mat RefFrame = timeSeries_Pyramid[TaskId - LoopRange][level]; //prepare에 pyramid형태로 queue할 때 방식
                        cv::Mat RefFrame = timeSeries_Pyramid.at(TaskId - LoopRange)[level]; //prepare에 pyramid형태로 queue할 때 방식
                        LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_PICK_PHASE_FROM_QUEUE);
                    #endif

                    auto DeltaPhaseXQueue = calc_phase_difference_queue(timeSeries_Pyramid, RefFrame, level, TaskId);
                    LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_CALC_PHASE_DIFFERENCE_QUEUE);
                    cv::Mat Phase = temporal_fir_filtering(DeltaPhaseXQueue);       
                    LogTimeStamp(TaskId, lv + 1, ori + 1,RUN_TEMPORAL_FIR_FILTERING);
                    cv::Mat Phase_Alpha;
                    cv::multiply(Phase, Alpha, Phase_Alpha);
                    LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_ALPHA);
                    cv::Mat exp_phase = calc_exponential_imaginary(Phase_Alpha);
                    LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_CALC_EXP_OF_PHASE);
                    cv::Mat Output;
                    // cv::multiply(exp_phase, OrigFrame, Output);
                    cv_complex_multiply(exp_phase, OrigFrame, Output);
                    LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_OUTPUT);
                    cv::Mat Output_Fouriered = calc_fft_2d_opencv(Output);
                    LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_FFT2_OUTPUT);
                    cv::Mat ComplexSteerableFilter_Doubled;
                    cv::multiply(Pyr[level], 2.0f, ComplexSteerableFilter_Doubled);
                    LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_GET_CSP_DOUBLED);
                    cv::Mat LocalAppendings;
                    cv::multiply(ComplexSteerableFilter_Doubled, Output_Fouriered, LocalAppendings);
                    LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP_DOUBLED);
                    MagnifiedFreq(cv::Rect(lb_x, lb_y, ub_x - lb_x, ub_y - lb_y)) += LocalAppendings;
                    LogTimeStamp(TaskId, lv + 1, ori + 1, RUN_ADD_UPON_REGION);

                }
            }

            int level = N_ORIENTATION * N_PYRAMID_LEVEL + 1;
            int lb_y = PyrIdx[level][0];
            int ub_y = PyrIdx[level][1];
            int lb_x = PyrIdx[level][2];
            int ub_x = PyrIdx[level][3];
            
            cv::Mat LowpassFrame;
            cv::multiply(
                Pyr[level], VidFft_Last(cv::Rect(lb_x, lb_y, ub_x - lb_x, ub_y - lb_y)), 
                LowpassFrame
            );
            LogTimeStamp(TaskId, N_PYRAMID_LEVEL + 1, 0 , RUN_LOWPASS_CALC_ELEMENT_WISE_MUL_CSP);
            MagnifiedFreq(cv::Rect(lb_x, lb_y, ub_x - lb_x, ub_y - lb_y)) += LowpassFrame;
            LogTimeStamp(TaskId, N_PYRAMID_LEVEL + 1, 0 , RUN_LOWPASS_ADD_UPON_REGION);
            
            
            cv::Mat Output_Final = calc_ifft_2d_opencv(MagnifiedFreq);
            LogTimeStamp(TaskId, N_PYRAMID_LEVEL + 1, 0, RUN_IFFT2_FINAL);
            
            //실제로는 여기서 이미지를 보여주기 위해 event call을 한다거나 하는 걸 구상했음.
            // storage_manager.DequeuePyramid();

            std::vector<cv::Mat> Output_split;
            cv::split(Output_Final, Output_split);
            std::cout << "Output_Final size: " << Output_split[0].size() << std::endl;     

            std::cout << "[Loop " << TaskId << "] done. RSS = "<< getCurrentRss() / 1024.0 / 1024.0 << " MB\n";

            LogTimeStamp(TaskId, N_PYRAMID_LEVEL + 1, 0, RUN_END);
            return std::make_pair(
                TaskId, 
                Output_split[0]
            );

        } else {
            std::cout<<"preparing task"<<std::endl;
            cv::Mat NullData(size[1], size[0], CV_32FC1, cv::Scalar(0.0f));
            return std::make_pair(TaskId, NullData);
        }
    }

    cv::Mat CalculateProcess::expand_to_complex(const cv::Mat& mat) {
        cv::Mat imag_zeros = cv::Mat::zeros(mat.size(), CV_32F);  // same size, zero-filled
        std::vector<cv::Mat> channels = {mat, imag_zeros};
        cv::Mat complex_output;
        cv::merge(channels, complex_output);  // CV_32FC2
        return complex_output;
    }
    
    Pyramid CalculateProcess::prepare_variables_to_push(const cv::Mat& mat, int TaskId) {

        // level 배치 예시
        // Pyramid level:5, Orientations: 2인 경우 총 vetor의 사이즈는 5*2 + 2
        
        LogTimeStamp(TaskId, 0, 0, PREPARE_VARS_START);
        Pyramid result_Pyramid;
        result_Pyramid.reserve(nLevels);
        

        for (int i = 0; i < nLevels; i++) {
            result_Pyramid.emplace_back();
        }
        LogTimeStamp(TaskId, 0, 0, PREPARE_VARS_INITIALIZE_PYRAMID);
        cv::Mat fouriered = calc_fft_2d_opencv(mat);
        LogTimeStamp(TaskId, 0, 0, PREPARE_VARS_FFT2);

        #if B_USING_PRAGMA_OMP_PARALLEL
        #pragma omp parallel for
        #endif
        for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
            for (int ori = 0; ori < N_ORIENTATION; ori++) {
                int level = lv * N_ORIENTATION + ori + 1;
                
                int lb_y = PyrIdx[level][0];
                int ub_y = PyrIdx[level][1];
                int lb_x = PyrIdx[level][2];
                int ub_x = PyrIdx[level][3];
                
                cv::Mat fouriered_local;
                cv::multiply(
                    Pyr[level], 
                    fouriered(cv::Rect(lb_x, lb_y, ub_x - lb_x, ub_y - lb_y)), 
                    fouriered_local
                );
                
                LogTimeStamp(TaskId, lv + 1, ori + 1, PREPARE_VARS_CALC_ELEMENT_WISE_MUL);
                #if B_PREPARING_LAZY
                    result_Pyramid[level] = std::move(fouriered_local);
                #else
                    cv::Mat filtered_local = calc_ifft_2d_opencv(fouriered_local);
                    LogTimeStamp(TaskId, lv + 1, ori + 1, PREPARE_VARS_IFFT2);
                    // cv::Mat Phase_local = filtered_local.arg();
                    std::vector<cv::Mat> filtered_local_split(2);
                    cv::split(filtered_local, filtered_local_split);
                    cv::Mat Phase_local;
                    cv::phase(filtered_local_split[0], filtered_local_split[1], Phase_local, false);  // false: radian
                    LogTimeStamp(TaskId, lv + 1, ori + 1, PREPARE_VARS_CALC_PHASE_ANGLE);

                    result_Pyramid[level] = std::move(Phase_local);
                #endif
            
            }
        }
        LogTimeStamp(TaskId, N_PYRAMID_LEVEL + 1, 0, PREPARE_VARS_END);
        return result_Pyramid;
    }

    cv::Mat CalculateProcess::calc_fft_2d_opencv(const cv::Mat& mat) {
        // Python: np.fft.fftshift(np.fft.fft2(mat))
        cv::Mat mat_inputFixed;
        if (mat.type() == CV_32FC1) {
            cv::Mat planes[] = { mat, cv::Mat::zeros(mat.size(), CV_32F) };
            cv::merge(planes, 2, mat_inputFixed);  // Make it CV_32FC2
        } else {
            mat_inputFixed = mat;
        }

        cv::Mat mat_fft;
        cv::dft(mat_inputFixed, mat_fft, cv::DFT_COMPLEX_OUTPUT);

        cv::Mat output = mat_fft.clone();

        int cx = output.cols / 2;
        int cy = output.rows / 2;

        cv::Mat q0(output, cv::Rect(0, 0, cx, cy));           // Top-Left
        cv::Mat q1(output, cv::Rect(cx, 0, cx, cy));          // Top-Right
        cv::Mat q2(output, cv::Rect(0, cy, cx, cy));          // Bottom-Left
        cv::Mat q3(output, cv::Rect(cx, cy, cx, cy));         // Bottom-Right

        cv::Mat tmp;
        q0.copyTo(tmp);
        q3.copyTo(q0);
        tmp.copyTo(q3);

        q1.copyTo(tmp);
        q2.copyTo(q1);
        tmp.copyTo(q2);
        
        return output;
    }

    cv::Mat CalculateProcess::calc_ifft_2d_opencv(const cv::Mat& mat) {
        // Python: np.fft.ifft2(np.fft.ifftshift(mat))
        cv::Mat mat_inputFixed;
        if (mat.type() == CV_32FC1) {
            cv::Mat planes[] = { mat, cv::Mat::zeros(mat.size(), CV_32F) };
            cv::merge(planes, 2, mat_inputFixed);  // Make it CV_32FC2
        } else {
            mat_inputFixed = mat;
        }

        cv::Mat input = mat_inputFixed.clone();
        int cx = input.cols / 2;
        int cy = input.rows / 2;

        cv::Mat q0(input, cv::Rect(0, 0, cx, cy));           // Top-Left
        cv::Mat q1(input, cv::Rect(cx, 0, cx, cy));          // Top-Right
        cv::Mat q2(input, cv::Rect(0, cy, cx, cy));          // Bottom-Left
        cv::Mat q3(input, cv::Rect(cx, cy, cx, cy));         // Bottom-Right

        cv::Mat tmp;
        q0.copyTo(tmp);
        q3.copyTo(q0);
        tmp.copyTo(q3);

        q1.copyTo(tmp);
        q2.copyTo(q1);
        tmp.copyTo(q2);

        cv::Mat result;
        cv::dft(input, result, cv::DFT_INVERSE | cv::DFT_COMPLEX_OUTPUT | cv::DFT_SCALE);

        return result;
    }

    inline std::vector<cv::Mat> CalculateProcess::calc_phase_difference_queue(const std::map<int, Pyramid>& timeSeries, const cv::Mat& RefFrame, int level, int TaskId) {
        auto& storage_manager = StorageManager::GetInstance();
        
        std::vector<cv::Mat> result;
        
        int lb_y = PyrIdx[level][0];
        int ub_y = PyrIdx[level][1];
        int lb_x = PyrIdx[level][2];
        int ub_x = PyrIdx[level][3];

        result.resize(LoopRange);
        
        for (int t = 0; t < LoopRange; t++) {

            #if B_PREPARING_LAZY
                Eigen::ArrayXXcf fouriered_local = timeSeries[TaskId - LoopRange + t][level];
                #if N_FFT_LIB_TYPE == 0
                    // auto filtered_local = calc_ifft_2d_cache(fouriered_local, ub_y - lb_y, ub_x - lb_x);
                    // auto filtered_local = calc_ifft_2d_thread_local(fouriered_local, ub_y - lb_y, ub_x - lb_x);
                    // auto filtered_local = calc_ifft_2d_using_plan_map_pipelined(
                    //     fouriered_local, 
                    //     level, N_PYRAMID_LEVEL + 1 + (level + 1) / 2, TaskId 
                    // );
                    auto filtered_local = calc_ifft_2d_using_plan_map(fouriered_local, level);
                #elif N_FFT_LIB_TYPE == 1
                    Eigen::ArrayXXcf filtered_local = calc_ifft_2d_pocketfft(fouriered_local, ub_y - lb_y, ub_x - lb_x);
                #elif N_FFT_LIB_TYPE == 2
                    auto filtered_local = calc_ifft_2d_using_cufft_plan_map(fouriered_local, level, N_PYRAMID_LEVEL + 1 + (level + 1) / 2, TaskId);
                #endif
                Eigen::ArrayXXcf LocalPhaseX = calc_phase_angle(filtered_local);
            #else
                // cv::Mat LocalPhaseX = timeSeries[TaskId - LoopRange + t][level];
                cv::Mat LocalPhaseX = timeSeries.at(TaskId - LoopRange + t)[level];
            #endif
            
            cv::Mat Subtracted;
            cv::subtract(LocalPhaseX, RefFrame, Subtracted);
            cv::add(Subtracted, cv::Scalar(M_PI_f), Subtracted); // M_PI_f를 더해서 음수값을 양수로 변환
            Subtracted.forEach<float>([this](float& v, const int*) {
                v = floor_mod_float(v, 2.0f * M_PI_f) - M_PI_f; // 2π로 나눈 나머지에서 π를 빼서 -π~π 범위로 조정
            });
            // cv::Mat Subtracted = (M_PI_f + LocalPhaseX.real() - RefFrame.real())
            //         .unaryExpr([this](float v) { return floor_mod_float(v, 2.0f * M_PI_f) - M_PI_f; });
            result[t] = Subtracted;
            // if (level == 1) // Local Phase X가 마지막 레벨일 때만 사용 횟수를 처리 (시간 축 상 1번 이므로)
            // {
            //     std::cout << "Logging pyramid usage for TargetId: " << TaskId - LoopRange + t << std::endl;
            //     storage_manager.LogPyramidUsage(TaskId - LoopRange + t);
            // }
            // std::cout << "Logging pyramid usage for TargetId: " << TaskId - LoopRange + t << ", level: " << level << std::endl;
            storage_manager.LogPyramidUsage(TaskId - LoopRange + t);
                
        }

         // TODO: 만약 다 사용된 t 시점의 local Phase X일 경우에는 지워줄 것

        return result;
    }

    inline cv::Mat CalculateProcess::temporal_fir_filtering(const std::vector<cv::Mat>& XQueue) {
        cv::Mat result;
        cv::multiply(XQueue[0], FirFilter[0], result);         
        for(int t = 1; t<XQueue.size(); t++) {
            cv::Mat temp;
            cv::multiply(XQueue[t], FirFilter[t], temp);
            cv::add(result, temp, result); // cv::add
        }
        return result;
    }

    inline cv::Mat CalculateProcess::calc_exponential_imaginary(const cv::Mat& mat) {
        cv::Mat cos_mat = mat.clone();
        cv::Mat sin_mat = mat.clone();
        
        cos_mat.forEach<float>([](float& v, const int*) { v = std::cos(v); });
        sin_mat.forEach<float>([](float& v, const int*) { v = std::sin(v); });

        std::vector<cv::Mat> channels = {cos_mat, sin_mat};  // 실수, 허수
        cv::Mat result;
        cv::merge(channels, result);  // CV_32FC2
        return result;
    }

    inline cv::Mat CalculateProcess::cv_complex_multiply(const cv::Mat& mat1, const cv::Mat& mat2, cv::Mat& dst) {
        // mat1, mat2: CV_32FC2
        CV_Assert(mat1.type() == CV_32FC2 && mat2.type() == CV_32FC2);
        CV_Assert(mat1.size() == mat2.size());

        dst.create(mat1.size(), CV_32FC2);
        // A와 B에서 같은 위치의 복소수를 곱하여 dst에 저장
        dst.forEach<cv::Vec2f>([&](cv::Vec2f& out, const int* pos) {
            const cv::Vec2f& a = mat1.at<cv::Vec2f>(pos[0], pos[1]);
            const cv::Vec2f& b = mat2.at<cv::Vec2f>(pos[0], pos[1]);

            float a_re = a[0], a_im = a[1];
            float b_re = b[0], b_im = b[1];

            out[0] = a_re * b_re - a_im * b_im; // Real part
            out[1] = a_re * b_im + a_im * b_re; // Imag part
        });

        return dst;
    }

    inline float CalculateProcess::floor_mod_float(float a, float b) {
        return fmodf(fmodf(a, b) + b, b);
    }

    void CalculateProcess::saveAsTiff (const cv::Mat& mat, std::string fileName, int rows, int cols) {
        // cv::Mat dst(rows, cols, CV_32F);
        // for (int i = 0; i < rows; i++) {
        //     for (int j = 0; j < cols; j++) {
        //         // dst.at<float>(i, j) = (float)mat[i * cols + j].real();
        //         dst.at<float>(i, j) = mat(i, j).real();
        //     }
        // }
        // cv::imwrite(fileName, dst);
        cv::imwrite(fileName, mat);
        
        // 사용 예시
        // ss<<"07_magnified_motion_level"<<level<<"_idx"<<TaskId<<".tiff";
        // saveAsTiff(Output, ss.str(),ub_y - lb_y, ub_x - lb_x);
        // ss.str("");
        // ss.clear();
    }

    inline float CalculateProcess::fast_atan2(float y, float x) {
        const float ONEQTR_PI = M_PI_f * 0.25f;
        const float THRQTR_PI = M_PI_f * 0.75f;
        float abs_y = std::abs(y) + 1e-10f;  // prevent 0/0
        float angle;
        if (x >= 0) {
            float r = (x - abs_y) / (x + abs_y);
            angle = ONEQTR_PI - ONEQTR_PI * r;
        } else {
            float r = (x + abs_y) / (abs_y - x);
            angle = THRQTR_PI - ONEQTR_PI * r;
        }
        return (y < 0) ? -angle : angle;
    }

    inline float CalculateProcess::fast_cos(float x) {
        x = wrap_angle(x);
        
        if (x < -3.14159265f) x += 6.28318531f;
        else if (x > 3.14159265f) x -= 6.28318531f;

        float x2 = x * x;
        return 1.0f - x2 / 2.0f + x2 * x2 / 24.0f;
    }

    inline float CalculateProcess::fast_sin(float x) {
        x = wrap_angle(x);

        // -π~π 범위로 정규화 (더 넓게 하면 정확도 낮아짐)
        if (x < -3.14159265f) x += 6.28318531f;
        else if (x > 3.14159265f) x -= 6.28318531f;

        // 근사식: sin(x) ≈ x * (1 - x² / 6 + x⁴ / 120)
        float x2 = x * x;
        return x * (1.0f - x2 / 6.0f + x2 * x2 / 120.0f);
        
    }

    inline float CalculateProcess::wrap_angle(float x) {
        const float TWO_PI = 6.28318530718f;
        x = std::fmod(x, TWO_PI);
        if (x < 0.0f)
            x += TWO_PI;
        return x;
    }

    size_t CalculateProcess::getCurrentRss() {
        long rss = 0L;
        std::ifstream statm("/proc/self/statm");
        if (statm.good()) {
            long dummy;
            statm >> dummy >> rss;
            rss *= sysconf(_SC_PAGESIZE); // page size 곱하기
        }
        return (size_t)rss;
    }

};
