# OpenCV 웹캠 뷰어

USB 웹캠으로부터 실시간 영상을 표시하는 OpenCV 기반 뷰어입니다.

## 기능

- 실시간 웹캠 영상 표시
- FPS 및 프레임 정보 실시간 표시
- 키보드 단축키로 제어
- 프레임 저장 기능

## 요구사항

- OpenCV 4.x
- C++11 지원 컴파일러
- Linux 시스템

## 설치

### 1. OpenCV 설치
```bash
sudo apt update
sudo apt install -y libopencv-dev
```

### 2. 빌드
```bash
make
```

## 사용법

### 기본 실행
```bash
./webcam_viewer
```

### 특정 해상도로 실행
```bash
./webcam_viewer /dev/video0 640 360
```

### 키보드 단축키
- `q` 또는 `Q`: 프로그램 종료
- `s` 또는 `S`: 현재 프레임을 JPG 파일로 저장

## 파일 구조

```
webcam_viewer/
├── opencv_viewer.cpp    # 소스 코드
├── webcam_viewer        # 실행 파일
├── Makefile            # 빌드 스크립트
└── README.md           # 이 파일
```

## 빌드 옵션

```bash
make          # 빌드
make clean    # 정리
make install  # 시스템에 설치
```

## 예제 출력

```
Camera opened successfully!
Press 'q' to quit, 's' to save frame
Total frames captured: 8874
Average FPS: 247.898
```

## 문제 해결

### 카메라가 열리지 않는 경우
1. 카메라가 연결되어 있는지 확인
2. `/dev/video0` 권한 확인
3. 다른 프로그램이 카메라를 사용 중인지 확인

### OpenCV 관련 오류
1. OpenCV가 제대로 설치되었는지 확인
2. `pkg-config --cflags opencv4` 명령으로 확인 