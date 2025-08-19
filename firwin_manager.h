#pragma once

#include <vector>
#include <cmath>
#include <cassert>


namespace PhaseBasedMotionMagification {
enum E_FILTER_TYPE{
    FIR_LOWPASS,
    FIR_HIGHPASS,
    FIR_BANDPASS,
    FIR_BANDSTOP
};

constexpr float pi = 3.14159265358979323846f;
constexpr float epsilon = 1e-6f;

class FirWinManager {
public:
    std::vector<float> bandpass_fir_kernel(int num_taps, float low_cutoff, float high_cutoff, E_FILTER_TYPE type);

private:
    
    inline float sinc(float x);
    inline float hamming(int n, int N);

};

};  // namespace PhaseBasedMotionMagification