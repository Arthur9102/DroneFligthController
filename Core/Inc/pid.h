/*
 * pid.h
 *
 *  Created on: May 6, 2025
 *      Author: Ho Quang Dung
 */
#ifndef INC_PID_H_
#define INC_PID_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* ------------------------------------------------------------------ */

/* Check compiler */
#ifdef __CODEVISIONAVR__
    /* Đoạn mã đặc biệt cho CodeVisionAVR (nếu cần) */
#endif
#define ABS(x)		((x>=0)? x: -x)
#define DEFAULT_SAMPLE_TIME 0.01 //s
#define DEFAULT_DEADBAND 0.01
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  PID_PARA ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

typedef struct {
    double *target;
    double *output;
    double T;
    double MaxOutput;
    double MinOutput;
    double *kp;
    double *ki;
    double *kd;
    double output_last;
    double last_error1;
    double last_error2;
    double error;
    double *measure;
    double last_filtered_derivative;
} PID_PARA;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  CascadedPID_t ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

typedef struct {
    PID_PARA angle;    // Outer loop for angle control
    PID_PARA rate;     // Inner loop for rate control
    double rate_setpoint; // Intermediate setpoint from angle to rate
} CascadedPID_t;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Declare function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* PID initial */
void PID_INIT(PID_PARA *pid, double *target, double *output, double *measure,
               double sample_time, double outmax, double outmin, double *kp, double *ki, double *kd);

void CascadedPID_Init(CascadedPID_t *cpid,
                      double *angle_target, double *angle_measure,
                      double *rate_measure, double *output,
                      double sample_time,
                      double angle_kp, double angle_ki, double angle_kd,
                      double rate_kp, double rate_ki, double rate_kd);

/* PID controller*/
void PID_CONTROLLER(PID_PARA *pid);
void PID_UpdateSampleTime(PID_PARA *pid, double new_sample_time);
void CascadedPID_Update(CascadedPID_t *cpid);
void CascadedPID_SetTarget(CascadedPID_t *cpid, double *angle_target, double *rate_target);

#endif /* __PID_H_ */





