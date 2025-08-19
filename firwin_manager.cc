#include "firwin_manager.h"

namespace PhaseBasedMotionMagification {
    std::vector<float> FirWinManager::bandpass_fir_kernel(int num_taps, float low_cutoff, float high_cutoff, E_FILTER_TYPE type) {
        // assert(num_taps % 2 == 1);  // 홀수 tap 수 필수
        // assert(0.0f < low_cutoff && low_cutoff < high_cutoff && high_cutoff < 1.0f);

        // std::vector<float> h(num_taps);
        // int M = num_taps - 1;
        // int mid = M / 2;

        // for (int n = 0; n <= M; ++n) {
        //     float t = n - mid;
        //     float sinc_high = 2 * high_cutoff * sinc(2 * high_cutoff * t);
        //     float sinc_low  = 2 * low_cutoff * sinc(2 * low_cutoff * t);
        //     h[n] = (sinc_high - sinc_low) * hamming(n, num_taps);
        // }

        assert(num_taps % 2 == 1);
        std::vector<float> h(num_taps);
        int M = num_taps - 1;
        int mid = M / 2;

        for (int n = 0; n <= M; ++n) {
            float t = n - mid;
            float value = 0.0f;

            if (type == E_FILTER_TYPE::FIR_LOWPASS) {
                value = 2 * low_cutoff * sinc(2 * low_cutoff * t);
            }
            else if (type == E_FILTER_TYPE::FIR_HIGHPASS) {
                value = sinc(t) - 2 * low_cutoff * sinc(2 * low_cutoff * t);
            }
            else if (type == E_FILTER_TYPE::FIR_BANDPASS) {
                value = 2 * (high_cutoff * sinc(2 * high_cutoff * t) - low_cutoff * sinc(2 * low_cutoff * t));
            }
            else if (type == E_FILTER_TYPE::FIR_BANDSTOP) {
                value = sinc(t) - 2 * (high_cutoff * sinc(2 * high_cutoff * t) - low_cutoff * sinc(2 * low_cutoff * t));
            }

            // Hamming window
            value *= hamming(n, num_taps);
            h[n] = value;
        }
        return h;
    }

    // sinc 함수: sin(πx)/(πx)
    inline float FirWinManager::sinc(float x) {
        // if (x == 0.0f) return 1.0f; // 부동소수점 때문에 실수형 동등 조건식은 위험
        if (std::abs(x) < epsilon)
            return 1.0f;
        else
            return sin(pi * x) / (pi * x);
    }

    // Hamming 윈도우
    inline float FirWinManager::hamming(int n, int N) {
        return 0.54f - 0.46f * cos(2 * pi * n / (N - 1));
    }
    
}; // namespace PhaseBasedMotionMagification 