/*
 * pid.c
 *
 *  Created on: Nov 16, 2024
 *      Author: Ho Quang Dung
 *
 */

#include "pid.h"

#define  NULL ((void*)0)
#define ALPHA 0.95 // not fil

void PID_INIT(PID_PARA *pid, double *target, double *output, double *measure,
               double sample_time, double outmax, double outmin, double *kp, double *ki, double *kd) {
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

void PID_SetTarget(PID_PARA *pid, double *new_target) {
    if (!pid || !new_target) {
        return;
    }
    pid->target = new_target;
}

void PID_UpdateSampleTime(PID_PARA *pid, double new_sample_time) {
    if (new_sample_time > 0) {
        pid->T = new_sample_time / 1000.0; // ms -> s
    }
}

void PID_CONTROLLER(PID_PARA *pid) {
    if (!pid || !pid->target || !pid->kp || !pid->ki || !pid->kd || !pid->output || !pid->measure) {
        return;
    }

    pid->error = *(pid->target) - *(pid->measure);

    // Apply deadband
    // if (ABS(pid->error) < DEFAULT_DEADBAND) {
    //     pid->error = 0;
    // }
    // PID calculations

    double kp = *(pid->kp);
    double ki = *(pid->ki);
    double kd = *(pid->kd);

    double filtered_derivative = ALPHA * (pid->error - 2 * pid->last_error1 + pid->last_error2)/ pid->T + (1-ALPHA) * pid->last_filtered_derivative;
    pid->last_filtered_derivative = filtered_derivative;

    double incKp = kp * (pid->error - pid->last_error1);
    double incKi = ki * pid->T * (pid->error + pid->last_error1) / 2.0; // Trapezoidal approximation
    double incKd = kd * filtered_derivative;

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

void CascadedPID_Init(CascadedPID_t *cpid,
                      double *angle_target, double *angle_measure,
                      double *rate_measure, double *output,
                      double sample_time,
                      double angle_kp, double angle_ki, double angle_kd,
                      double rate_kp, double rate_ki, double rate_kd) {
    // Initialize angle (outer) loop
    PID_INIT(&cpid->angle, angle_target, &cpid->rate_setpoint, angle_measure,
             sample_time, 250.0, -250.0, &angle_kp, &angle_ki, &angle_kd);
             
    // Initialize rate (inner) loop                 
    PID_INIT(&cpid->rate, &cpid->rate_setpoint, output, rate_measure,
             sample_time, 500.0, -500.0, &rate_kp, &rate_ki, &rate_kd);
}

void CascadedPID_Update(CascadedPID_t *cpid) {
    // Update outer loop (angle)
    PID_CONTROLLER(&cpid->angle);
    
    // Rate setpoint is automatically updated through pointer
    
    // Update inner loop (rate)
    PID_CONTROLLER(&cpid->rate);
}

