#pragma once

#include <iostream>
#include <complex>

#define M_PI_f 3.14159265358979323846f
#define N_TESTING_FRAME 300
#define N_LOOPRANGE 5
#define FIR_FILTER  {-0.00164969f, 0.24671236f, 0.65651579f, 0.24671236f, -0.00164969f}
#define N_PYRAMID_LEVEL 3
#define N_ORIENTATION 2

// 테스트 하는데 쓰이는 하이퍼 파라미터들
#define B_RUNNING_PARALLEL true // 병렬로 돌리느냐, 직렬로 돌리느냐
#define B_USING_PRAGMA_OMP_PARALLEL false // #pragma parallel for 사용 여부
#define B_PREPARING_LAZY false
#define N_TRIGGERING_TYPE 0 // 0: 트리거 안 씀, 1: 트리거 신호에 따라 녹화 게시, 2: 트리거 신호 마다 프레임 그랩

#define B_SAVING_FRAMES false // frame 저장 여부
#define B_LOGGING_DETAILED_TIMESTAMP false // 세부적인 성능 테스트 체크를 위한 timestamp 기록 여부

#define B_MONITORING_FOR_CAM true // Cam으로 모니터링하는지 여부 (아닐 경우 로컬 저장된 비디오)

inline std::string videoFileName = "512x256_20210905_3rd_01(255Hz).mp4";

#if defined(__linux__)
// 비디오 파일 위치
inline std::string videoFileDirectory = "/home/pi/Desktop/pmm_pipelining_cpp/pmm_pipelining_min/PhaseBasedMotionMagification/input/";
// inline std::string videoFileDirectory = "/home/imbedded/Desktop/trigger_pbm/pmm_pipelining_min/PhaseBasedMotionMagification/input/";
// inline std::string videoFileDirectory = "/home/sonic/Desktop/jyan/pmm_pipelining_cpp/pmm_pipelining_min/PhaseBasedMotionMagification/input/";
// inline std::string videoFileDirectory = "/home/jyan/Desktop/pmm_pipelining_min/pmm_pipelining_min/PhaseBasedMotionMagification/input/";

// 가상환경 고정된 위치에 어디서 돌렸는지를 참조하기 위해서
inline std::string EnvNamePath = "/home/pi/Documents/pmm/test_environment_name.txt";
inline std::string LogPath = "/home/pi/Documents/pmm/logs/";
// inline std::string EnvNamePath = "/home/jyan/Documents/pmm/test_environment_name.txt";
// inline std::string LogPath = "/home/jyan/Documents/pmm/logs/";

// 트리거링 관련 gpio 정보
inline const char* _chip_name = "gpiochip0"; // GPIO chip name
inline unsigned int _line_offset = 17;
#endif

#ifdef _WIN32

// 비디오 파일 위치
inline std::string videoFileDirectory = R"(D:\Git\pmm_pipelining_cpp\pmm_pipelining_min\PhaseBasedMotionMagification\input\)";

// 가상환경 고정된 위치에 어디서 돌렸는지를 참조하기 위해서
inline std::string EnvNamePath = R"(C:\Users\jaepa\pmm\test_environment_name.txt)";
inline std::string LogPath = R"(C:\Users\jaepa\pmm\logs\)";
#endif

