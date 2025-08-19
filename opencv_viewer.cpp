#include <iostream>
#include <string>
#include <chrono>
#include <signal.h>
#include <opencv2/opencv.hpp>

static volatile bool keep_running = true;
static void handle_sigint(int sig) { (void)sig; keep_running = false; }

int main(int argc, char** argv) {
    std::string device = "/dev/video0";
    int width = 640;
    int height = 480;
    
    if (argc >= 2) device = argv[1];
    if (argc >= 3) width = std::stoi(argv[2]);
    if (argc >= 4) height = std::stoi(argv[3]);
    
    signal(SIGINT, handle_sigint);
    
    // OpenCV 카메라 열기
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "Error: Could not open camera device " << device << std::endl;
        return -1;
    }
    
    // 카메라 설정
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, 30);
    
    std::cout << "Camera opened successfully!" << std::endl;
    std::cout << "Press 'q' to quit, 's' to save frame" << std::endl;
    
    // 윈도우 생성
    cv::namedWindow("USB Webcam - Real-time Viewer", cv::WINDOW_AUTOSIZE);
    
    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (keep_running) {
        // 프레임 읽기
        cv::Mat frame;
        cap >> frame;
        
        if (frame.empty()) {
            std::cout << "Error: Could not read frame" << std::endl;
            break;
        }
        
        frame_count++;
        
        // FPS 계산
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        double fps = frame_count / (elapsed_time.count() / 1000.0);
        
        // 프레임 정보 표시
        std::string info_text = "Frame: " + std::to_string(frame_count) + 
                               ", FPS: " + std::to_string(static_cast<int>(fps)) + 
                               ", Size: " + std::to_string(frame.cols) + "x" + std::to_string(frame.rows);
        
        cv::putText(frame, info_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        // 화면에 표시
        cv::imshow("USB Webcam - Real-time Viewer", frame);
        
        // 키 입력 처리
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            break;
        } else if (key == 's' || key == 'S') {
            // 프레임 저장
            std::string filename = "opencv_frame_" + std::to_string(frame_count) + ".jpg";
            cv::imwrite(filename, frame);
            std::cout << "Saved: " << filename << std::endl;
        }
    }
    
    // 정리
    cap.release();
    cv::destroyAllWindows();
    
    auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start_time);
    double avg_fps = frame_count / (total_time.count() / 1000.0);
    
    std::cout << "Total frames captured: " << frame_count << std::endl;
    std::cout << "Average FPS: " << avg_fps << std::endl;
    
    return 0;
} 