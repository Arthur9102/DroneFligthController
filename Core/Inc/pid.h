/*
 * pid.h
 *
 *  Created on: May 6, 2025
 *     @author: Ho Quang Dung
 */
#ifndef INC_PID_H_
#define INC_PID_H_


/* ------------------------------ Include files ------------------------------ */

#include <stdint.h>

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Macro ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#define ABS(x)		((x>=0)? x: -x)
#define DEFAULT_SAMPLE_TIME 0.01 //s
#define DEFAULT_DEADBAND 1.0 // Deadband for PID control
#define ALPHA 0 // Filter coefficient for derivative calculation
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  PID_PARA ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

typedef struct {
    float *target;
    float *output;
    float T;
    float MaxOutput;
    float MinOutput;
    float *kp;
    float *ki;
    float *kd;
    float output_last;
    float last_error1;
    float last_error2;
    float error;
    float *measure;
    float last_filtered_derivative;
} PID_PARA;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  CascadedPID_t ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

typedef struct {
    PID_PARA angle;    // Outer loop for angle control
    PID_PARA rate;     // Inner loop for rate control
    float rate_setpoint; // Intermediate setpoint from angle to rate
} CascadedPID_t;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Declare function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* PID initial */
void PID_INIT(PID_PARA *pid, float *target, float *output, float *measure,
               float sample_time, float outmax, float outmin, float *kp, float *ki, float *kd);

void CascadedPID_Init(CascadedPID_t *cpid,
                      float *angle_target, float *angle_measure,
                      float *rate_measure, float *output,
                      float sample_time,
                      float *angle_kp, float *angle_ki, float *angle_kd,
                      float *rate_kp, float *rate_ki, float *rate_kd);

/* PID controller*/
void PID_CONTROLLER(PID_PARA *pid);
void PID_UpdateSampleTime(PID_PARA *pid, float new_sample_time);
void CascadedPID_Update(CascadedPID_t *cpid);
void CascadedPID_SetTarget(CascadedPID_t *cpid, float *angle_target, float *rate_target);

#endif /* __PID_H_ */





