/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "piecewise_jerk_trajectory1d.h"
#include <algorithm>
//初始化 
PiecewiseJerkTrajectory1d::PiecewiseJerkTrajectory1d(const double p,
                                                     const double v,
                                                     const double a) {
  last_p_ = p;
  last_v_ = v;
  last_a_ = a;
  param_.push_back(0.0);
}

// 依据恒定jerk曲线(即3次多项式),根据 Δs 把各段曲线的起点、终点信息恢复出来
void PiecewiseJerkTrajectory1d::AppendSegment(const double jerk,
                                              const double param) {

  // param_（即delta_s）有个累加的行为
  // 这样下面Evaluate的时候就方便知道具体是哪个segment
  param_.push_back(param_.back() + param);//delta_s 

  segments_.emplace_back(ConstantJerkTrajectory1d(last_p_, last_v_, last_a_, jerk, param));

  last_p_ = segments_.back().end_position();

  last_v_ = segments_.back().end_velocity();

  last_a_ = segments_.back().end_acceleration();
}

// 调用 constant_jerk_trajectory1d.cc 的 Evaluate()，输入 s 求出 l、l'、l''
// 此时会判断下该s 属于哪个segment，借助lower_bound()函数
double PiecewiseJerkTrajectory1d::Evaluate(const std::uint32_t order,
                                           const double param) const {
  //中第一个范围[param_.begin(), param_.end())不小于param的元素
  auto it_lower = std::lower_bound(param_.begin(), param_.end(), param);
  // 两头越界情况的处理
  if (it_lower == param_.begin()) {
    return segments_[0].Evaluate(order, param);
  }
  if (it_lower == param_.end()) {
    auto index = std::max(0, static_cast<int>(param_.size() - 2));
    return segments_.back().Evaluate(order, param - param_[index]);
  }

  auto index = std::distance(param_.begin(), it_lower);
  return segments_[index - 1].Evaluate(order, param - param_[index - 1]);
}
// 返回曲线总长度
double PiecewiseJerkTrajectory1d::ParamLength() const { return param_.back(); }
std::string PiecewiseJerkTrajectory1d::ToString() const { return ""; }


ConstantJerkTrajectory1d::ConstantJerkTrajectory1d(const double p0,
                                                   const double v0,
                                                   const double a0,
                                                   const double j,
                                                   const double param)
    : p0_(p0), v0_(v0), a0_(a0), param_(param), jerk_(j) {
  //CHECK_GT(param, FLAGS_numerical_epsilon);
  p1_ = Evaluate(0, param_);
  v1_ = Evaluate(1, param_);
  a1_ = Evaluate(2, param_);
}
//知道起点去计算终点 
double ConstantJerkTrajectory1d::Evaluate(const std::uint32_t order,
                                          const double param) const {
  switch (order) {
    case 0: {
      return p0_ + v0_ * param + 0.5 * a0_ * param * param +
             jerk_ * param * param * param / 6.0;
    }
    case 1: {
      return v0_ + a0_ * param + 0.5 * jerk_ * param * param;
    }
    case 2: {
      return a0_ + jerk_ * param;
    }
    case 3: {
      return jerk_;
    }
    default:
      return 0.0;
  }
}

double ConstantJerkTrajectory1d::start_position() const { return p0_; }

double ConstantJerkTrajectory1d::start_velocity() const { return v0_; }

double ConstantJerkTrajectory1d::start_acceleration() const { return a0_; }

double ConstantJerkTrajectory1d::end_position() const { return p1_; }

double ConstantJerkTrajectory1d::end_velocity() const { return v1_; }

double ConstantJerkTrajectory1d::end_acceleration() const { return a1_; }

double ConstantJerkTrajectory1d::ParamLength() const { return param_; }

std::string ConstantJerkTrajectory1d::ToString() const { return ""; }

double ConstantJerkTrajectory1d::jerk() const { return jerk_; }




