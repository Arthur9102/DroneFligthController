/**
 * @file    pid.c
 * @brief   PID control implementation for STM32
 * @author  Ho Quang Dung
 * @date    2025-05-07
 * @version v2.0
 * @par Copyright
 *          (C) 2025 Ho Quang Dung. All rights reserved.
 * @par History
 *          1: Created
 * @par Reference
 *          PID Control Theory, STM32 HAL Library
 */
#include "pid.h"

#define  NULL ((void*)0)
// #define ALPHA 0.95 //  filtered

/**
 * @brief Initialize PID parameters
 * @param pid Pointer to PID_PARA structure
 */
void PID_INIT(PID_PARA *pid, float *target, float *output, float *measure,
               float sample_time, float outmax, float outmin, float *kp, float *ki, float *kd) {
    if (!pid || !target || !output || !measure || !kp || !ki || !kd) {
        return;
    }

    pid->target = target;
    pid->T = (sample_time > 0) ? sample_time/1000.0 : DEFAULT_SAMPLE_TIME;
    pid->MaxOutput = outmax;
    pid->MinOutput = outmin;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output = output;
    pid->output_last = 0.0;
    pid->last_error1 = 0.0;
    pid->last_error2 = 0.0;
    pid->error = 0.0;
    pid->measure = measure;
    pid->last_filtered_derivative = 0.0;
}

void PID_SetTarget(PID_PARA *pid, float *new_target) {
    if (!pid || !new_target) {
        return;
    }
    pid->target = new_target;
}

void PID_UpdateSampleTime(PID_PARA *pid, float new_sample_time) {
    if (new_sample_time > 0) {
        pid->T = new_sample_time / 1000.0; // ms -> s
    }
}

/**
 * @brief PID control algorithm
 * @param pid Pointer to PID_PARA structure
 * @note  This function calculates the PID output based on the target and measured values.
 *        It uses the PID parameters (Kp, Ki, Kd) to compute the control output.
 */
void PID_CONTROLLER(PID_PARA *pid) {
    if (!pid || !pid->target || !pid->kp || !pid->ki || !pid->kd || !pid->output || !pid->measure) {
        return;
    }

    pid->error = *(pid->target) - *(pid->measure);

    // Apply deadband
    if (ABS(pid->error) < DEFAULT_DEADBAND) {
        pid->error = 0;
    }
    // PID calculations

    float kp = *(pid->kp);
    float ki = *(pid->ki);
    float kd = *(pid->kd);

    float filtered_derivative = ALPHA * (pid->error - 2 * pid->last_error1 + pid->last_error2)/ pid->T + (1-ALPHA) * pid->last_filtered_derivative;
    pid->last_filtered_derivative = filtered_derivative;

    float incKp = kp * (pid->error - pid->last_error1);
    float incKi = ki * pid->T * (pid->error + pid->last_error1) / 2.0; // Trapezoidal approximation
    float incKd = kd * filtered_derivative;

    // Update output
    *pid->output = pid->output_last + incKp + incKi + incKd;

    // Clamping the output
    if (*pid->output > pid->MaxOutput) {
        *pid->output = pid->MaxOutput;
    } else if (*pid->output < pid->MinOutput) {
        *pid->output = pid->MinOutput;
    }

    // Update error history
    pid->last_error2 = pid->last_error1;
    pid->last_error1 = pid->error;
    pid->output_last = *pid->output;
}
void PID_CONTROLER_2(PID_PARA *pid) {
    // This function is similar to PID_CONTROLLER but uses a different approach for derivative calculation.
    if (!pid || !pid->target || !pid->kp || !pid->ki || !pid->kd || !pid->output || !pid->measure) {
        return;
    }

    pid->error = *(pid->target) - *(pid->measure);

    // Apply deadband
    if (ABS(pid->error) < DEFAULT_DEADBAND) {
        pid->error = 0;
    }

    // PID calculations
    float kp = *(pid->kp);
    float ki = *(pid->ki);
    float kd = *(pid->kd);
    float derivative = (1-ALPHA) * pid->last_filtered_derivative + ALPHA * (pid->error - pid->last_error1); // Derivative term
    pid->last_filtered_derivative = derivative;

    float incKp = kp * pid->error;
    float incKi = pid->pre_integral + ki * pid->T * (pid->error + pid->last_error1) / 2.0; // Trapezoidal approximation
    float incKd = kd * derivative / pid->T; // Derivative term

    // Update output
    *pid->output = incKp + incKi + incKd;

    // Clamping the output
    if (*pid->output > pid->MaxOutput) {
        *pid->output = pid->MaxOutput;
    } else if (*pid->output < pid->MinOutput) {
        *pid->output = pid->MinOutput;
    }

    // Update last error for next derivative calculation
    pid->last_error1 = pid->error;
    pid->pre_integral = incKi; // Store integral term for next iteration
}

void PD_CONTROLLER(PID_PARA *pid) {
    if (!pid || !pid->target || !pid->kp || !pid->output || !pid->measure) {
        return;
    }

    pid->error = *(pid->target) - *(pid->measure);

    // Apply deadband
    if (ABS(pid->error) < DEFAULT_DEADBAND) {
        pid->error = 0;
    }

    // Proportional-Derivative control
    float kp = *(pid->kp);
    float kd = *(pid->kd);

    float derivative = (1-ALPHA) *(pid->error - pid->last_error1) + ALPHA * pid->last_filtered_derivative; // Derivative term
    pid->last_filtered_derivative = derivative;
    *pid->output = kp * pid->error + kd * derivative / pid->T; // Proportional term + Derivative term

    // Clamping the output
    if (*pid->output > pid->MaxOutput) {
        *pid->output = pid->MaxOutput;
    } else if (*pid->output < pid->MinOutput) {
        *pid->output = pid->MinOutput;
    }

    // Update last error for next derivative calculation
    pid->last_error1 = pid->error;
}

void PI_CONTROLLER(PID_PARA *pid) {
    if (!pid || !pid->target || !pid->kp || !pid->ki || !pid->output || !pid->measure) {
        return;
    }

    pid->error = *(pid->target) - *(pid->measure);

    // Apply deadband
    if (ABS(pid->error) < DEFAULT_DEADBAND) {
        pid->error = 0;
    }

    // Proportional-Integral control
    float kp = *(pid->kp);
    float ki = *(pid->ki);
    
    *pid->output += kp * pid->error + ki * pid->T * pid->error; // Integral term

    // Clamping the output
    if (*pid->output > pid->MaxOutput) {
        *pid->output = pid->MaxOutput;
    } else if (*pid->output < pid->MinOutput) {
        *pid->output = pid->MinOutput;
    }
}
void P_CONTROLLER(PID_PARA *pid) {
    if (!pid || !pid->target || !pid->kp || !pid->output || !pid->measure) {
        return;
    }

    pid->error = *(pid->target) - *(pid->measure);

    // Apply deadband
    if (ABS(pid->error) < DEFAULT_DEADBAND) {
        pid->error = 0;
    }

    // Proportional control
    *pid->output = *(pid->kp) * pid->error;

    // Clamping the output
    if (*pid->output > pid->MaxOutput) {
        *pid->output = pid->MaxOutput;
    } else if (*pid->output < pid->MinOutput) {
        *pid->output = pid->MinOutput;
    }
}

/**
 * @brief Initialize Cascaded PID controller
 * @param cpid Pointer to CascadedPID_t structure
 * @param angle_target Pointer to target angle
 * @param angle_measure Pointer to measured angle
 * @param rate_measure Pointer to measured rate
 * @param output Pointer to output value
 * @param sample_time Sample time in seconds
 * @param angle_kp, angle_ki, angle_kd PID parameters for angle loop
 * @param rate_kp, rate_ki, rate_kd PID parameters for rate loop
 */
void CascadedPID_Init(CascadedPID_t *cpid,
                      float *angle_target, float *angle_measure,
                      float *rate_measure, float *output,
                      float sample_time,
                      float *angle_kp, float *angle_ki, float *angle_kd,
                      float *rate_kp, float *rate_ki, float *rate_kd) {
    // Initialize angle (outer) loop - 50Hz
    PID_INIT(&cpid->angle, angle_target, &cpid->rate_setpoint, angle_measure,
             20.0f,  // 20ms (50Hz) for outer loop
             150.0f, -150.0f,
             angle_kp, angle_ki, angle_kd);

    // Initialize rate (inner) loop - 250Hz  
    PID_INIT(&cpid->rate, &cpid->rate_setpoint, output, rate_measure,
             4.0f, // 4ms (250Hz) for inner loop
             500.0f, -500.0f, 
             rate_kp, rate_ki, rate_kd);
             
    cpid->last_outer_update = 0;
}

/**
 * @brief Set rate setpoint trực tiếp cho vòng trong của cascaded PID
 * @param cpid Pointer đến cấu trúc CascadedPID_t
 * @param new_rate_setpoint Giá trị rate setpoint mới (đơn vị độ/s hoặc rad/s tùy hệ)
 */
void CascadedPID_SetRateSetpoint(CascadedPID_t *cpid, float new_rate_setpoint) {
    if (!cpid) return;
    cpid->rate_setpoint = new_rate_setpoint;
}

/**
 * @brief Update Cascaded PID controller
 * @param cpid Pointer to CascadedPID_t structure
 * @note  This function updates the outer loop (angle) and inner loop (rate) of the cascaded PID controller.
 */
void CascadedPID_Update(CascadedPID_t *cpid) {
    uint32_t current_time = HAL_GetTick();
    
    // Update outer loop at 50Hz
    // if (current_time - cpid->last_outer_update >= 20) {  // 20ms = 50Hz
    //     P_CONTROLLER(&cpid->angle);
    //     cpid->last_outer_update = current_time;
    // }
    
    // Update inner loop (rate) - 250Hz (always)
    PD_CONTROLLER(&cpid->rate);
}
/**
 * @brief Reset PID parameters
 * @param pid Pointer to PID_PARA structure
 * @note  This function resets the PID output, last output, errors, and other parameters to their initial state.
 */
void reset_pid(PID_PARA *pid) {
    if (pid == NULL) return;

    *pid->output = 0.0f;
    pid->output_last = 0.0f;

    pid->error = 0.0f;
    pid->last_error1 = 0.0f;
    pid->last_error2 = 0.0f;

    // pid->last_filtered_derivative = 0.0f;
    // *pid->rate_setpoint = 0.0f;
    // *pid->measure = 0.0f;
}
/**
 * @brief Set target for Cascaded PID controller
 * @param cpid Pointer to CascadedPID_t structure
 * @param angle_target Pointer to target angle
 * @param rate_target Pointer to target rate
 */
void CascadedPID_SetTarget(CascadedPID_t *cpid, float *angle_target, float *rate_target) {
    if (!cpid || !angle_target || !rate_target) {
        return;
    }
    
    cpid->angle.target = angle_target;
    cpid->rate.target = rate_target;
}
void PID_SetOutputLimits(PID_PARA *pid, float min, float max) {
    if (!pid) {
        return;
    }
    
    pid->MinOutput = min;
    pid->MaxOutput = max;
}
void PID_SetKp(PID_PARA *pid, float kp) {
    if (!pid || !pid->kp) {
        return;
    }
    
    *(pid->kp) = kp;
}
void PID_SetKi(PID_PARA *pid, float ki) {
    if (!pid || !pid->ki) {
        return;
    }
    
    *(pid->ki) = ki;
}
void PID_SetKd(PID_PARA *pid, float kd) {
    if (!pid || !pid->kd) {
        return;
    }
    
    *(pid->kd) = kd;
}
void PID_SetOutput(PID_PARA *pid, float output) {
    if (!pid || !pid->output) {
        return;
    }
    
    *(pid->output) = output;
}

