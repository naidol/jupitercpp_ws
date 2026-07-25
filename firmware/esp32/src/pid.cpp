// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Arduino.h"
#include "pid.h"

PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd)
{
}

double PID::compute(float setpoint, float measured_value, float dt)
{
    double error = setpoint - measured_value;
    derivative_ = (error - prev_error_) / dt;

    // Tentative integral step (only committed below if it won't wind up).
    double integral_candidate = integral_ + error * dt;

    if(setpoint == 0 && error == 0)
    {
        integral_candidate = 0;
        derivative_ = 0;
    }

    double pid = (kp_ * error) + (ki_ * integral_candidate) + (kd_ * derivative_);

    // ANTI-WINDUP (conditional integration): when the output is saturated, only commit
    // the integral if the current error would pull the output BACK toward the linear
    // range (i.e. let it unwind, never grow). This stops the integral accumulating while
    // a wheel is stalled/blocked — the cause of the multi-second reverse-breakaway lurch
    // (integral wound up for ~13 s, then dumped on breakaway). (2026-07-25)
    if (pid > max_val_) {
        pid = max_val_;
        if (error < 0) integral_ = integral_candidate;   // unwinding only
    } else if (pid < min_val_) {
        pid = min_val_;
        if (error > 0) integral_ = integral_candidate;   // unwinding only
    } else {
        integral_ = integral_candidate;                  // in range: accumulate normally
    }

    prev_error_ = error;
    return pid;
}

void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
