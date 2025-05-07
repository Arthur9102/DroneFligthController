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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Declare function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* PID initial */
void PID_INIT(PID_PARA *pid, double *target, double *output, double *measure,
               double sample_time, double outmax, double outmin, double *kp, double *ki, double *kd);

/* PID controller*/
void PID_CONTROLLER(PID_PARA *pid);
void PID_UpdateSampleTime(PID_PARA *pid, double new_sample_time);

#endif /* __PID_H_ */





