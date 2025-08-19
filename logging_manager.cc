#include "logging_manager.h"

namespace PhaseBasedMotionMagification  {

void LoggingManager::LogTimeStamp(LogKey logKey, std::chrono::steady_clock::time_point timeStamp) {
    std::unique_lock<std::mutex> lock(iMtx_Log);
    TimeLogger[logKey] = timeStamp;
}

void LoggingManager::UpdateFrameInfo(int _width, int _height) {
    width = _width;
    height = _height;
}

int LoggingManager::SummarizeResults() {
    
    // 로그 파일 생성 시점
    std::string now_str = GetNowTime();
    
    // Csv File Initializing
    std::stringstream ss;
    ss<<LogPath<<"pmm_log_"<<now_str<<".csv";
    std::ofstream file(ss.str());
    file<<std::fixed<<std::setprecision(10);
    std::cout<<"csv log file created."<<std::endl;

    // TODO: 각 메소드별 동작에 완료 출력 넣어주기

    // Data 정리
    ConvertChronoClockToDouble();
    CalculateProcessEllapsedTime();
    
    // === Basic Info ===
    PrintBasicInfos(file);
    file<<"\n\n";

    // === Summary ===
    file<<"\n________ Average Time Span ________\n";
    PrintHeaders(file, HEADER_SUMMARY);
    PrintData(file, SUMMARY);

    // === Time Stamps (prepare) ===
    file<<"\n________ Time Stamps (Prepare) ________\n";
    PrintHeaders(file, HEADER_PREPARING_VARS);
    PrintData(file, TIMESTAMP_PREPARING_VARS);
    
    // === Time Stamps (run) ===
    file<<"\n________ Time Stamps (Run) ________\n";
    PrintHeaders(file, HEADER_RUN);
    PrintData(file, TIMESTAMP_RUN);

    // === Ellpased Time (prepare) ===
    file<<"\n________ Ellapsed Time (Prepare) ________\n";
    PrintHeaders(file, HEADER_PREPARING_VARS);
    PrintData(file, ELLAPSED_PREPARING_VARS);
    
    // === Ellpased Time (run) ===
    file<<"\n________ Ellapsed Run (Prepare) ________\n";
    PrintHeaders(file, HEADER_RUN);
    PrintData(file, ELLAPSED_RUN);
    
    file.close();
    std::cout << "Data Logging finished." << std::endl;
    return 0;
}

std::string LoggingManager::GetNowTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H_%M_%S");
    return oss.str();
}

int LoggingManager::PrintBasicInfos(std::ofstream& file) {
    // run한 환경 종류    
    std::string EnvName;
    std::ifstream file_env_name(EnvNamePath);
    
    if (file_env_name.is_open()) {
        getline(file_env_name, EnvName);
        file_env_name.close();
        std::cout << "Environment name loaded" << std::endl;
    } else {
        std::cerr << "ERROR: Unable to open environment name file" << std::endl;
        return -1;
    }
    file<<"Environment: "<<EnvName<<"\n";

    // 동영상 제목
    file<<"Video: "<<videoFileName<<"\n";

    // 동영상 사이즈
    file<<"Size: "<<width<<" x "<<height<<"\n";

    // === Hyper Parameter Summary ===
    // 총 테스트 프레임 수
    file<<"Calculated Frames: "<<N_TESTING_FRAME<<"\n";

    // Loop Range 수 (Filter Order + 1)
    file<<"Loop Range: "<<N_LOOPRANGE<<" (Filter Order): "<<N_LOOPRANGE - 1<<"\n";

    // Pyramid Level 수준
    file<<"Level of Pyramid (Steerable only): "<<N_PYRAMID_LEVEL<<"\n";

    // Orientation 가짓수
    file<<"Orientations: "<<N_ORIENTATION<<"\n";
    
    // (Pyramid Level * Orientation)
    file<<"Stacked Level: "<<N_ORIENTATION * N_PYRAMID_LEVEL<<"\n";

    // 병렬, 직렬 여부
    #if B_RUNNING_PARALLEL
        file<<"Running Style: Parallel\n";
    #else 
        file<<"Running Style: Serial\n";
    #endif

    // pragma for 사용 여부
    #if B_USING_PRAGMA_OMP_PARALLEL
        file<<"pragma omp parallel: USED\n";
    #else 
        file<<"pragma omp parallel: not used\n";
    #endif


    // fft 모듈의 종류
    #if N_FFT_LIB_TYPE == 0
        file<<"FFT Module: fftw3f\n";
    #elif N_FFT_LIB_TYPE == 1
        file<<"FFT Module: pocketfft\n";
    #elif N_FFT_LIB_TYPE == 3
        file<<"FFT Module: OpenCV\n";
    #endif

    // 삼각함수 관련 cos, sin, exp 등을 poly기반 근사로 사용하는지 여부
    #if B_USING_FAST_TRIGONOMETRIC
        file<<"Trigonometric(cos sin) Calculation: Fast Polynomial\n";
    #else 
        file<<"Trigonometric(cos sin) Calculation: cmath\n";
    #endif

    // 병합을 위한 Post CSP 적용 여부
    #if B_APPLYING_POST_CSP
        file<<"Post Complex Steerable Pyramid Applied: O\n";
    #else 
        file<<"Post Complex Steerable Pyramid Applied: X\n";
    #endif

    std::cout << "Basic informations printed." << std::endl;
    return 0;
}

int LoggingManager::PrintHeaders(std::ofstream& file, E_LOG_PRINT_CASE eHeaderType) {
    if (file.is_open()) {
        switch(eHeaderType) {
            case HEADER_PREPARING_VARS:
            {
                file<<"Start,Initialize Pyramid,FFT2";
                for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                    for (int ori = 0; ori < N_ORIENTATION; ori++) {
                        for (int proc = 0; proc < 3; proc ++) {
                            if (ori == 0 && proc == 0)
                                file<<",Level"<<lv + 1;
                            else
                                file<<",_";
                        }
                    }
                }
                file<<",End\n";

                file<<"_,_,_";
                for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                    for (int ori = 0; ori < N_ORIENTATION; ori++) {
                        for (int proc = 0; proc < 3; proc ++) {
                            if (proc == 0)
                                file<<",Orientation"<<ori + 1;
                            else
                                file<<",_";
                        }
                    }
                }
                file<<"\n";

                file<<"_,_,_";
                for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                    for (int ori = 0; ori < N_ORIENTATION; ori++) {
                        file<<",Apply CSP on Freq Domain,IFFT2,Calculate Phase";
                    }
                }

                file<<"\n";

                std::cout << "Header for prepare_vars printed." << std::endl;
                break;
            }
            case HEADER_RUN:
            {
                file<<"Start,Initialize Vars,FFT2";
                for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                    for (int ori = 0; ori < N_ORIENTATION; ori++) {
                        for (int proc = 0; proc < 12; proc ++) {
                            if (ori == 0 && proc == 0)
                                file<<",Level"<<lv + 1;
                            else
                                file<<",_";
                        }
                    }
                }
                file<<",Lowpass CSP,Lowpass Stack,IFFT2 result,End\n";

                file<<"_,_,_";
                for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                    for (int ori = 0; ori < N_ORIENTATION; ori++) {
                        for (int proc = 0; proc < 12; proc ++) {
                            if (proc == 0)
                                file<<",Orientation"<<ori + 1;
                            else
                                file<<",_";
                        }
                    }
                }
                file<<"\n";

                file<<"_,_,_";
                for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                    for (int ori = 0; ori < N_ORIENTATION; ori++) {
                        file<<",Apply CSP on Freq Domain,IFFT2,Get Local Phase,Subtract Phase,FIR Filtering,Elementwise Multiply(Alpha)";
                        file<<",Exponential Imaginary,Apply Magnified Motion,FFT2 Output, Get x2 CSP, Apply x2 CSP on Freq Domain, Stack upon Region";                        
                    }
                }
                file<<"\n";

                std::cout << "Header for run printed." << std::endl;
                break;
            }

            case HEADER_SUMMARY:
            {
                file<<"Start,Initialize Vars,FFT2";
                file<<",(Level wise average),_,_,_,_,_,_,_,_,_,_,_";
                file<<",Lowpass CSP,Lowpass Stack,IFFT2 result,End\n";

                file<<"_,_,_";
                file<<",Apply CSP on Freq Domain,IFFT2,Get Local Phase,Subtract Phase,FIR Filtering,Elementwise Multiply(Alpha)";
                file<<",Exponential Imaginary,Apply Magnified Motion,FFT2 Output, Get x2 CSP, Apply x2 CSP on Freq Domain, Stack upon Region\n";
                
                std::cout << "Header for summary printed." << std::endl;
                break;
            }
            default:
                break;
        }
    }
    return 0;
}

int LoggingManager::PrintData(std::ofstream& file, E_LOG_PRINT_CASE eHeaderType) {
    if (file.is_open()) {
        LogKey logkey;
        switch(eHeaderType) {
            case TIMESTAMP_PREPARING_VARS:
            {
                for (int t = 0; t < N_LOOPRANGE; t++) {
                    file << ProcessTimeStamp[{t, 0, 0, PREPARE_VARS_START}]
                         << "," << ProcessTimeStamp[{t, 0, 0, PREPARE_VARS_INITIALIZE_PYRAMID}] 
                         << "," << ProcessTimeStamp[{t, 0, 0, PREPARE_VARS_FFT2}];
                    
                    for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                        for (int ori = 0; ori < N_ORIENTATION; ori++) {
                            file << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_ELEMENT_WISE_MUL}] 
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, PREPARE_VARS_IFFT2}] 
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_PHASE_ANGLE}];
                        }
                    }
                    file << "," << ProcessTimeStamp[{t, N_PYRAMID_LEVEL + 1, 0, PREPARE_VARS_END}] << "\n";
                }

                std::cout << "Logging time stamps of prepare_vars finished." << std::endl;
                break;
            }
            
            case TIMESTAMP_RUN:
            {
                for (int t = N_LOOPRANGE; t < N_TESTING_FRAME; t++) {
                    file << ProcessTimeStamp[{t, 0, 0, RUN_START}]
                         << "," << ProcessTimeStamp[{t, 0, 0, RUN_INTIALIZE_PTR_AND_VARS}] 
                         << "," << ProcessTimeStamp[{t, 0, 0, RUN_FFT2_CURRENT_FRAME}];
                    
                    for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                        for (int ori = 0; ori < N_ORIENTATION; ori++) {
                            file << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP}] 
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_IFFT2_LEVEL_WISE}] 
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_PICK_PHASE_FROM_QUEUE}]
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_PHASE_DIFFERENCE_QUEUE}]
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_TEMPORAL_FIR_FILTERING}]
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_ALPHA}]
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_EXP_OF_PHASE}]
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_OUTPUT}]
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_FFT2_OUTPUT}]
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_GET_CSP_DOUBLED}]
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP_DOUBLED}]
                                 << "," << ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_ADD_UPON_REGION}];
                        }
                    }
                    file << "," << ProcessTimeStamp[{t, N_PYRAMID_LEVEL + 1, 0, RUN_LOWPASS_CALC_ELEMENT_WISE_MUL_CSP}] 
                         << "," << ProcessTimeStamp[{t, N_PYRAMID_LEVEL + 1, 0, RUN_LOWPASS_ADD_UPON_REGION}] 
                         << "," << ProcessTimeStamp[{t, N_PYRAMID_LEVEL + 1, 0, RUN_IFFT2_FINAL}] 
                         << "," << ProcessTimeStamp[{t, N_PYRAMID_LEVEL + 1, 0, RUN_END}] 
                         << "\n";
                }
                std::cout << "Logging time stamps of run finished." << std::endl;
                break;
            }
            
            case ELLAPSED_PREPARING_VARS:
            {
                for (int t = 0; t < N_LOOPRANGE; t++) {
                    file << "_," << ProcessEllapsedTime[{t, 0, 0, PREPARE_VARS_INITIALIZE_PYRAMID}] 
                         << "," << ProcessEllapsedTime[{t, 0, 0, PREPARE_VARS_FFT2}];
                    
                    for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                        for (int ori = 0; ori < N_ORIENTATION; ori++) {
                            file << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_ELEMENT_WISE_MUL}] 
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, PREPARE_VARS_IFFT2}] 
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_PHASE_ANGLE}];
                        }
                    }
                    file << "\n";
                }
                std::cout << "Logging ellapsed time of prepare_vars finished." << std::endl;
                break;
            }

            case ELLAPSED_RUN:
            {
                for (int t = N_LOOPRANGE; t < N_TESTING_FRAME; t++) {
                    file << "_," << ProcessEllapsedTime[{t, 0, 0, RUN_INTIALIZE_PTR_AND_VARS}] 
                         << "," << ProcessEllapsedTime[{t, 0, 0, RUN_FFT2_CURRENT_FRAME}];
                    
                    for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                        for (int ori = 0; ori < N_ORIENTATION; ori++) {
                            file << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP}] 
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_IFFT2_LEVEL_WISE}] 
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_PICK_PHASE_FROM_QUEUE}]
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_PHASE_DIFFERENCE_QUEUE}]
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_TEMPORAL_FIR_FILTERING}]
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_ALPHA}]
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_EXP_OF_PHASE}]
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_OUTPUT}]
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_FFT2_OUTPUT}]
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_GET_CSP_DOUBLED}]
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP_DOUBLED}]
                                 << "," << ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_ADD_UPON_REGION}];
                        }
                    }
                    file << "," << ProcessEllapsedTime[{t, N_PYRAMID_LEVEL + 1, 0 , RUN_LOWPASS_CALC_ELEMENT_WISE_MUL_CSP}] 
                         << "," << ProcessEllapsedTime[{t, N_PYRAMID_LEVEL + 1, 0 , RUN_LOWPASS_ADD_UPON_REGION}] 
                         << "," << ProcessEllapsedTime[{t, N_PYRAMID_LEVEL + 1, 0 , RUN_IFFT2_FINAL}] 
                         << "\n";
                }

                std::cout << "Logging time stamps of run finished." << std::endl;
                break;
            }
            
            case SUMMARY:
            {
                // Calcualte Summrized Data
                std::vector<double> TotalEllapsed;
                TotalEllapsed.resize(2 + 12 + 3);
                double TotalEllapsedTime_Frame = 0.0;
                double AverageFps = (double)(N_TESTING_FRAME - N_LOOPRANGE - 1) / 
                                        (ProcessTimeStamp[{N_TESTING_FRAME - 1, N_PYRAMID_LEVEL + 1, 0 , RUN_END}]
                                            - ProcessTimeStamp[{N_LOOPRANGE, N_PYRAMID_LEVEL + 1, 0 , RUN_END}])
                                                * 1000;
                
                double AverageSPrepareStartStampInterval = (ProcessTimeStamp[{N_LOOPRANGE - 1, 0, 0, PREPARE_VARS_START}] - ProcessTimeStamp[{0, 0, 0, PREPARE_VARS_START}]) / (double)N_LOOPRANGE;
                double AveragePrepareEndStampInterval = (ProcessTimeStamp[{N_LOOPRANGE - 1, N_PYRAMID_LEVEL + 1, 0, PREPARE_VARS_END}] - ProcessTimeStamp[{0, N_PYRAMID_LEVEL + 1, 0, PREPARE_VARS_END}]) / (double)N_LOOPRANGE;

                double AverageSRunStartStampInterval = (ProcessTimeStamp[{N_TESTING_FRAME - 1, 0, 0, RUN_START}] - ProcessTimeStamp[{N_LOOPRANGE, 0, 0, RUN_START}]) / (double)(N_TESTING_FRAME - N_LOOPRANGE - 1);
                double AverageRunEndStampInterval = (ProcessTimeStamp[{N_TESTING_FRAME - 1, N_PYRAMID_LEVEL + 1, 0, RUN_END}] - ProcessTimeStamp[{N_LOOPRANGE, N_PYRAMID_LEVEL + 1, 0, RUN_END}]) / (double)(N_TESTING_FRAME - N_LOOPRANGE - 1);
                std::cout << ProcessTimeStamp[{N_TESTING_FRAME - 1, 0, 0, RUN_END}] << std::endl;
                std::cout << ProcessTimeStamp[{N_LOOPRANGE, 0, 0, RUN_END}] << std::endl;
                std::cout << ProcessTimeStamp[{N_TESTING_FRAME - 1, 0, 0, RUN_END}] - ProcessTimeStamp[{N_LOOPRANGE - 1, 0, 0, RUN_END}] << AverageFps << std::endl;

                for (int t = N_LOOPRANGE; t < N_TESTING_FRAME; t++) {
                    TotalEllapsed[0] += ProcessEllapsedTime[{t, 0, 0, RUN_INTIALIZE_PTR_AND_VARS}] ;
                    TotalEllapsed[1] += ProcessEllapsedTime[{t, 0, 0, RUN_FFT2_CURRENT_FRAME}];
                    
                    for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
                        for (int ori = 0; ori < N_ORIENTATION; ori++) {
                            TotalEllapsed[2] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP}];
                            TotalEllapsed[3] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_IFFT2_LEVEL_WISE}];
                            TotalEllapsed[4] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_PICK_PHASE_FROM_QUEUE}];
                            TotalEllapsed[5] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_PHASE_DIFFERENCE_QUEUE}];
                            TotalEllapsed[6] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_TEMPORAL_FIR_FILTERING}];
                            TotalEllapsed[7] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_ALPHA}];
                            TotalEllapsed[8] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_EXP_OF_PHASE}];
                            TotalEllapsed[9] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_OUTPUT}];
                            TotalEllapsed[10] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_FFT2_OUTPUT}];
                            TotalEllapsed[11] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_GET_CSP_DOUBLED}];
                            TotalEllapsed[12] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP_DOUBLED}];
                            TotalEllapsed[13] += ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_ADD_UPON_REGION}];
                        }
                    }
                    TotalEllapsed[14] += ProcessEllapsedTime[{t, N_PYRAMID_LEVEL + 1, 0, RUN_LOWPASS_CALC_ELEMENT_WISE_MUL_CSP}];
                    TotalEllapsed[15] += ProcessEllapsedTime[{t, N_PYRAMID_LEVEL + 1, 0, RUN_LOWPASS_ADD_UPON_REGION}];
                    TotalEllapsed[16] += ProcessEllapsedTime[{t, N_PYRAMID_LEVEL + 1, 0, RUN_IFFT2_FINAL}];
                    
                    TotalEllapsedTime_Frame += (ProcessTimeStamp[{t, N_PYRAMID_LEVEL + 1, 0, RUN_END}] - ProcessTimeStamp[{t, 0, 0, RUN_START}]);
                }
                
                file << "-," << TotalEllapsed[0] / (N_TESTING_FRAME - N_LOOPRANGE)
                     << "," << TotalEllapsed[1] / (N_TESTING_FRAME - N_LOOPRANGE)
                     << "," << TotalEllapsed[2] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[3] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[4] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[5] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[6] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[7] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[8] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[9] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[10] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[11] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[12] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[13] / ((N_TESTING_FRAME - N_LOOPRANGE) * N_PYRAMID_LEVEL * N_ORIENTATION)
                     << "," << TotalEllapsed[14] / (N_TESTING_FRAME - N_LOOPRANGE)
                     << "," << TotalEllapsed[15] / (N_TESTING_FRAME - N_LOOPRANGE)
                     << "," << TotalEllapsed[16] / (N_TESTING_FRAME - N_LOOPRANGE)
                     << "\n";

                file << "Ellapsed Time per Frame (Serial): " << TotalEllapsedTime_Frame / (double)(N_TESTING_FRAME - N_LOOPRANGE) << "\n";
                file << "Average FPS: " << AverageFps << "\n";
                file << "Average Start Stamp Interval (Prepare): " << AverageSPrepareStartStampInterval << "\n";
                file << "Average End Stamp Interval (Prepare): " << AveragePrepareEndStampInterval << "\n";
                file << "Average Start Stamp Interval (Run): " << AverageSRunStartStampInterval << "\n";
                file << "Average End Stamp Interval (Run): " << AverageRunEndStampInterval << "\n";

                std::cout << "Summarizing overall time log finished." << std::endl;
                break;
            }

            default:
                break;
        }
    }
    return 0;
}

int LoggingManager::ConvertChronoClockToDouble() {
    for (const auto& [key, tp] : TimeLogger) {
        double timeStamp = std::chrono::duration<double, std::milli>(tp.time_since_epoch()).count();
        ProcessTimeStamp[key] = timeStamp;
    }
    std::cout << "Time stamp data type conversion finished." << std::endl;
    return 0;
}

int LoggingManager::CalculateProcessEllapsedTime() {
    // Prepareing Vars
    for (int t = 0; t < N_LOOPRANGE; t++) {
        ProcessEllapsedTime[{t, 0, 0, PREPARE_VARS_INITIALIZE_PYRAMID}] = 
            ProcessTimeStamp[{t, 0, 0, PREPARE_VARS_INITIALIZE_PYRAMID}] - ProcessTimeStamp[{t, 0, 0, PREPARE_VARS_START}];
        ProcessEllapsedTime[{t, 0, 0, PREPARE_VARS_FFT2}] = 
            ProcessTimeStamp[{t, 0, 0, PREPARE_VARS_FFT2}] - ProcessTimeStamp[{t, 0, 0, PREPARE_VARS_INITIALIZE_PYRAMID}];
        
        for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
            for (int ori = 0; ori < N_ORIENTATION; ori++) {
                if (lv == 0 && ori == 0) {
                    ProcessEllapsedTime[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_ELEMENT_WISE_MUL}] = 
                        ProcessTimeStamp[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_ELEMENT_WISE_MUL}] - ProcessTimeStamp[{t, 0, 0, PREPARE_VARS_FFT2}];
                } else {
                    if (ori == 0) {
                        ProcessEllapsedTime[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_ELEMENT_WISE_MUL}] = 
                            ProcessTimeStamp[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_ELEMENT_WISE_MUL}] - ProcessTimeStamp[{t, lv, N_ORIENTATION, PREPARE_VARS_CALC_PHASE_ANGLE}];
                    } else {
                        ProcessEllapsedTime[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_ELEMENT_WISE_MUL}] = 
                            ProcessTimeStamp[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_ELEMENT_WISE_MUL}] - ProcessTimeStamp[{t, lv + 1, ori, PREPARE_VARS_CALC_PHASE_ANGLE}];
                    }
                }
                ProcessEllapsedTime[{t, lv + 1, ori + 1, PREPARE_VARS_IFFT2}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, PREPARE_VARS_IFFT2}] - ProcessTimeStamp[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_ELEMENT_WISE_MUL}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_PHASE_ANGLE}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, PREPARE_VARS_CALC_PHASE_ANGLE}] - ProcessTimeStamp[{t, lv + 1, ori + 1, PREPARE_VARS_IFFT2}];
            }
        }
    }
    
    // Run
    for (int t = N_LOOPRANGE; t < N_TESTING_FRAME; t++) {
        ProcessEllapsedTime[{t, 0, 0, RUN_INTIALIZE_PTR_AND_VARS}] = 
            ProcessTimeStamp[{t, 0, 0, RUN_INTIALIZE_PTR_AND_VARS}] - ProcessTimeStamp[{t, 0, 0, RUN_START}];
        ProcessEllapsedTime[{t, 0, 0, RUN_FFT2_CURRENT_FRAME}] = 
            ProcessTimeStamp[{t, 0, 0, RUN_FFT2_CURRENT_FRAME}] - ProcessTimeStamp[{t, 0, 0, RUN_INTIALIZE_PTR_AND_VARS}];
        
        for (int lv = 0; lv < N_PYRAMID_LEVEL; lv++) {
            for (int ori = 0; ori < N_ORIENTATION; ori++) {
                if (lv == 0 && ori == 0) {
                    ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP}] = 
                        ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP}] - ProcessTimeStamp[{t, 0, 0, RUN_FFT2_CURRENT_FRAME}];
                } else {
                    if (ori == 0) {
                        ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP}] = 
                            ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP}] - ProcessTimeStamp[{t, lv, N_ORIENTATION, RUN_ADD_UPON_REGION}];
                    } else {
                        ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP}] = 
                            ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP}] - ProcessTimeStamp[{t, lv + 1, ori, RUN_ADD_UPON_REGION}];
                    }
                }
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_IFFT2_LEVEL_WISE}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_IFFT2_LEVEL_WISE}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_PICK_PHASE_FROM_QUEUE}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_PICK_PHASE_FROM_QUEUE}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_IFFT2_LEVEL_WISE}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_PHASE_DIFFERENCE_QUEUE}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_PHASE_DIFFERENCE_QUEUE}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_PICK_PHASE_FROM_QUEUE}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_TEMPORAL_FIR_FILTERING}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_TEMPORAL_FIR_FILTERING}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_PHASE_DIFFERENCE_QUEUE}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_ALPHA}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_ALPHA}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_TEMPORAL_FIR_FILTERING}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_EXP_OF_PHASE}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_EXP_OF_PHASE}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_ALPHA}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_OUTPUT}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_OUTPUT}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_EXP_OF_PHASE}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_FFT2_OUTPUT}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_FFT2_OUTPUT}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_OUTPUT}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_GET_CSP_DOUBLED}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_GET_CSP_DOUBLED}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_FFT2_OUTPUT}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP_DOUBLED}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP_DOUBLED}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_GET_CSP_DOUBLED}];
                ProcessEllapsedTime[{t, lv + 1, ori + 1, RUN_ADD_UPON_REGION}] = 
                    ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_ADD_UPON_REGION}] - ProcessTimeStamp[{t, lv + 1, ori + 1, RUN_CALC_ELEMENT_WISE_MUL_CSP_DOUBLED}];
            }
        }

        ProcessEllapsedTime[{t,  N_PYRAMID_LEVEL + 1, 0, RUN_LOWPASS_CALC_ELEMENT_WISE_MUL_CSP}] = 
            ProcessTimeStamp[{t,  N_PYRAMID_LEVEL + 1, 0, RUN_LOWPASS_CALC_ELEMENT_WISE_MUL_CSP}] - ProcessTimeStamp[{t, N_PYRAMID_LEVEL, N_ORIENTATION , RUN_ADD_UPON_REGION}];
        ProcessEllapsedTime[{t,  N_PYRAMID_LEVEL + 1, 0, RUN_LOWPASS_ADD_UPON_REGION}] = 
            ProcessTimeStamp[{t,  N_PYRAMID_LEVEL + 1, 0, RUN_LOWPASS_ADD_UPON_REGION}] - ProcessTimeStamp[{t, N_PYRAMID_LEVEL + 1, 0 , RUN_LOWPASS_CALC_ELEMENT_WISE_MUL_CSP}];
        ProcessEllapsedTime[{t,  N_PYRAMID_LEVEL + 1, 0, RUN_IFFT2_FINAL}] = 
            ProcessTimeStamp[{t,  N_PYRAMID_LEVEL + 1, 0, RUN_IFFT2_FINAL}] - ProcessTimeStamp[{t, N_PYRAMID_LEVEL + 1, 0 , RUN_LOWPASS_ADD_UPON_REGION}];
    }

    std::cout << "Ellapsed time calculation finished." << std::endl;
    return 0;
}

}; // namespace PhaseBasedMotionMagification 