/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "digital_filter_coefficients.h"
#include <cmath>
#include <vector>
#define PI 3.1415926


/*对于采样时间，采样时间越小，离散时间系统的采样频率就越高，滤波器的截止频率也就越高。因此，较小的采样时间可以实现更高的截止频率，但也会增加计算复杂度和噪声。
对于截止频率，截止频率越高，滤波器的降噪效果就越好，但也会降低信号的带宽。在实际应用中，需要根据具体的信号特性和应用要求来选择合适的截止频率。*/
namespace common {

void LpfCoefficients(const double ts, const double cutoff_freq,
                     std::vector<double> *denominators,
                     std::vector<double> *numerators) {
  denominators->clear();
  numerators->clear();
  denominators->reserve(3);
  numerators->reserve(3);

  double wa = 2.0 * M_PI * cutoff_freq;  // Analog frequency in rad/s
  double alpha = wa * ts / 2.0;          // tan(Wd/2), Wd is discrete frequency
  double alpha_sqr = alpha * alpha;
  double tmp_term = std::sqrt(2.0) * alpha + alpha_sqr;
  double gain = alpha_sqr / (1.0 + tmp_term);

  denominators->push_back(1.0);
  denominators->push_back(2.0 * (alpha_sqr - 1.0) / (1.0 + tmp_term));
  denominators->push_back((1.0 - std::sqrt(2.0) * alpha + alpha_sqr) /
                          (1.0 + tmp_term));

  numerators->push_back(gain);
  numerators->push_back(2.0 * gain);
  numerators->push_back(gain);
}

void LpFirstOrderCoefficients(const double ts, const double settling_time,
                              const double dead_time,
                              std::vector<double> *denominators,
                              std::vector<double> *numerators) {
  // sanity check
  if (ts <= 0.0 || settling_time < 0.0 || dead_time < 0.0) {
    //AERROR << "time cannot be negative";
    return;
  }

  const size_t k_d = static_cast<size_t>(dead_time / ts);
  double a_term;

  denominators->clear();
  numerators->clear();
  denominators->reserve(2);
  numerators->reserve(k_d + 1);  // size depends on dead-time

  if (settling_time == 0.0) {
    a_term = 0.0;
  } else {
    a_term = exp(-1 * ts / settling_time);
  }

  denominators->push_back(1.0);
  denominators->push_back(-a_term);
  numerators->insert(numerators->end(), k_d, 0.0);
  numerators->push_back(1 - a_term);
}

void applyFilter(const double input, const std::vector<double>& denominators, const std::vector<double>& numerators, double& output) {
    std::vector<double> x;
    x.resize(denominators.size(), 0.0);
    std::vector<double> y;
    y.resize(numerators.size(), 0.0);

    // 更新输入历史
    for (size_t j = x.size() - 1; j > 0; --j) {
        x[j] = x[j - 1];
    }
    x[0] = input;

    // 计算滤波器输出
    y[0] = numerators[0] * x[0];
    for (size_t j = 1; j < numerators.size(); ++j) {
        y[0] += numerators[j] * x[j] - denominators[j] * y[j];
    }

    // 更新输出历史
    for (size_t j = y.size() - 1; j > 0; --j) {
        y[j] = y[j - 1];
    }
    output = y[0];
}
//二阶低通滤波器
float LPF2_T2(float xin,float sample_freq,float cutoff_freq) {
   static float lpf2_yout[3] = {0};
   static float lpf2_xin[3] = {0};
 
  //  float sample_freq = 10000;
  //  float cutoff_freq = 100;
  // float fr = sample_freq / cutoff_freq;
 
   float ohm = tan(PI * cutoff_freq *sample_freq);
   float c = 1 + 1.414 * ohm  + ohm * ohm;
   
   float b0 = ohm * ohm /c;
   float b1 = 2.0f * b0;
   float b2 = b0;
 
   float a1 = 2.0f * (ohm * ohm - 1.0f) /c;
   float a2 = (1.0f - 1.414 * ohm + ohm * ohm) / c;
   
   lpf2_xin[2] = xin;
   
   lpf2_yout[2] = b0 * lpf2_xin[2] + b1 * lpf2_xin[1] + b2 * lpf2_xin[0] - a1 * lpf2_yout[1] - a2 * lpf2_yout[0];
 
   lpf2_xin[0] = lpf2_xin[1];
   lpf2_xin[1] = lpf2_xin[2];
   lpf2_yout[0] = lpf2_yout[1];
   lpf2_yout[1] = lpf2_yout[2];
 
   return lpf2_yout[2];
}
// 添加相位补偿的函
}  // namespace common
