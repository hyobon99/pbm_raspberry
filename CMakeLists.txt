option(B_USING_GPU "Build with CUDA GPU support" OFF)

cmake_minimum_required(VERSION 3.13)


project(PhaseBasedMotionMagnification LANGUAGES CXX)
message(STATUS "Building without CUDA support.")


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_CXX_FLAGS "-O0 -pg")
# set(CMAKE_EXE_LINKER_FLAGS "-pg")
if(WIN32)
  set(OpenCV_DIR "C:/msys64/mingw64/lib/cmake/opencv4")
  set(FFTW_ROOT "C:/msys64/mingw64/lib/cmake/fftw3")
  set(FFTW_ROOT "C:/msys64/mingw64/lib/cmake/fftw3f")
  set(FFTW_ROOT "C:/msys64/mingw64/lib/cmake")
endif()


find_package(OpenCV REQUIRED) # sudo apt install libopencv-dev
find_package(PkgConfig REQUIRED)
find_package(OpenMP REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
# pkg_search_module(FFTW REQUIRED fftw3)

  

# 헤더 위치 지정
include_directories("${FFTW_ROOT}/include")

set(MAIN_SOURCES
  main.cc
  storage_manager.cc
  calculate_process.cc
  complex_steerable_pyramid.cc
  logging_manager.cc
  firwin_manager.cc
  streaming_manager.cc
  # thread_pool.cc
)
 
set_source_files_properties(
  ${MAIN_SOURCES}
  PROPERTIES LANGUAGE CXX
)

set(MAIN_EXTRA_LIBS
  OpenMP::OpenMP_CXX
  ${OpenCV_LIBS}
  ${FFTW_LIBRARIES}
  ${FFTW3F_LIBRARY}
  ${LIBGPIOD_LIBRARIES}
  # "${FFTW_ROOT}/include"
)

add_executable(PhaseBasedMotionMagnification ${MAIN_SOURCES})

target_link_libraries(PhaseBasedMotionMagnification PRIVATE ${MAIN_EXTRA_LIBS})
link_directories(/usr/local/cuda-11.8/lib64)


# install(TARGETS PhaseBasedMotionMagnification
#         # RUNTIME DESTINATION "C:/Users/jaepa/Desktop/GitRepos/pmm_cpp_min/pmm_pipelining_min/PhaseBasedMotionMagification/build"
#         RUNTIME DESTINATION "D:/Git/pmm_Java_ndk_cpp_trial/PhaseBasedMotionMagification/build"
#         # RUNTIME DESTINATION "./build"
# )

# message(STATUS "OpenCV Include Dir: ${OpenCV_INCLUDE_DIRS}")
# message(STATUS "OpenCV Library Dir: ${OpenCV_LIBS}")

# (선택) CMP0177 워닝 없애기
if(POLICY CMP0177)
  cmake_policy(SET CMP0177 NEW)
endif()
