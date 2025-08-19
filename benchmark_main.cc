#include "complex_steerable_pyramid.h"
#include "calculate_process.h"

#include <benchmark/benchmark.h>
#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

using namespace PhaseBasedMotionMagification;

// 고정 입력 생성 함수
std::vector<std::complex<float>> GenerateFixedInput(std::size_t size) {
    std::vector<std::complex<float>> input(size);
    for (std::size_t i = 0; i < size; ++i) {
        float real = std::sin(i * 0.01f);
        float imag = std::cos(i * 0.01f);
        input[i] = std::complex<float>(real, imag);
    }
    return input;
}

static void BM_calc_element_wise_multiply(benchmark::State& state) {
    std::size_t size = static_cast<std::size_t>(state.range(0));
    const std::vector<std::complex<float>> input1 = GenerateFixedInput(size);
    const std::vector<std::complex<float>> input2 = GenerateFixedInput(size);
    
    auto* complexSteerablePyramid = new PhaseBasedMotionMagification::ComplexSteerablePyramid(size/256, 256);
    auto Pyr = complexSteerablePyramid->get_pyr();
    auto PyrIdx = complexSteerablePyramid->get_pyr_idx();

    auto* calculate_process = new PhaseBasedMotionMagification::CalculateProcess(Pyr, PyrIdx);

    for (auto _ : state) {
        // 복사본을 넘기기 때문에 매 반복마다 동일한 입력
        auto result = calculate_process->FOR_BENCHMARK_calc_element_wise_multiply(input1, input2);

        // 결과를 컴파일러가 제거하지 않게 보장
        benchmark::DoNotOptimize(result);
    }
}

static void BM_conv_fft2(benchmark::State& state) {
    std::size_t size = static_cast<std::size_t>(state.range(0));
    const std::vector<std::complex<float>> input1 = GenerateFixedInput(size);
    const std::vector<std::complex<float>> input2 = GenerateFixedInput(size);
    
    auto* complexSteerablePyramid = new PhaseBasedMotionMagification::ComplexSteerablePyramid(size/256, 256);
    auto Pyr = complexSteerablePyramid->get_pyr();
    auto PyrIdx = complexSteerablePyramid->get_pyr_idx();

    auto* calculate_process = new PhaseBasedMotionMagification::CalculateProcess(Pyr, PyrIdx);

    for (auto _ : state) {
        // 복사본을 넘기기 때문에 매 반복마다 동일한 입력
        auto result = calculate_process->FOR_BENCHMARK_conv_on_freq_domain_with_fft2(input1, input2, 256, size/256);

        // 결과를 컴파일러가 제거하지 않게 보장
        benchmark::DoNotOptimize(result);
    }
}

static void BM_conv_spatial(benchmark::State& state) {
    std::size_t size = static_cast<std::size_t>(state.range(0));
    const std::vector<std::complex<float>> input1 = GenerateFixedInput(size);
    const std::vector<std::complex<float>> input2 = GenerateFixedInput(128*128);
    
    auto* complexSteerablePyramid = new PhaseBasedMotionMagification::ComplexSteerablePyramid(size/256, 256);
    auto Pyr = complexSteerablePyramid->get_pyr();
    auto PyrIdx = complexSteerablePyramid->get_pyr_idx();

    auto* calculate_process = new PhaseBasedMotionMagification::CalculateProcess(Pyr, PyrIdx);

    for (auto _ : state) {
        // 복사본을 넘기기 때문에 매 반복마다 동일한 입력
        auto result = calculate_process->FOR_BENCHMARK_conv_on_sptial_domain(input1, 256, size/256, input2, 128, 128);

        // 결과를 컴파일러가 제거하지 않게 보장
        benchmark::DoNotOptimize(result);
    }
}


// 입력 사이즈를 바꿔가며 테스트 가능
// BENCHMARK(BM_calc_element_wise_multiply)->Arg(163840)->Arg(131072); // 벡터 길이
// 256x640 = 163840
// 256x512 = 131-72

BENCHMARK(BM_conv_fft2)->Arg(163840)->Arg(131072);
BENCHMARK(BM_conv_spatial)->Arg(163840)->Arg(131072);

BENCHMARK_MAIN();