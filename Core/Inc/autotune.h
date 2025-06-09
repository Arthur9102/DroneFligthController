#ifndef AUTOTUNE_H_
#define AUTOTUNE_H_

#include "pid.h"
#include "config.h"
// #include "main.h"
typedef struct {
    float Kp;
    float Kd;
    float fitness;
} Individual;

// Cấu trúc lưu dữ liệu thử nghiệm
typedef struct {
    float setpoint;    // Giá trị đặt
    float output;      // Giá trị đầu ra
    float error;       // Sai số
    float control;     // Tín hiệu điều khiển
    uint32_t time;     // Thời điểm (ms)
} TestData;
typedef enum{
    TUNE,
    NO_TUNE
}State_Tune;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POP_SIZE 20       // Population size for GA
#define MAX_GENERATIONS 50 // Number of generations
#define MUTATION_RATE 0.1 // Mutation probability
#define CROSSOVER_RATE 0.8 // Crossover probability
#define KP_MIN 0.0f       // Kp range [0, 10]
#define KP_MAX 10.0f
#define KD_MIN 0.0f       // Kd range [0, 10]
#define KD_MAX 10.0f
#define KI_MIN 0.0f       // Ki range [0, 10]
#define KI_MAX 10.0f
#define DT 0.01f          // Discrete time step (10ms)
#define SIM_TIME 5.0f     // Simulation time (5s)
#define SETPOINT 0.0f     // Step response setpoint (0 radian)


/* USER CODE BEGIN PFP */
float rand_float(float min, float max);
void init_population(void);
float simulate_pitch(float Kp, float Kd);
float calculate_fitness(float *output, int steps);
void select_parents(int *parent1, int *parent2);
void crossover(Individual *parent1, Individual *parent2, Individual *child1, Individual *child2);
void mutate(Individual *ind);
void autotune_pid(float *Kp, float *Kd);
float evaluate_controller(float Kp, float Kd, TestData* data, int max_samples);
float calculate_fitness_real(TestData* data, int samples);
#endif /* __AUTOTUNE_H */
//void calculate_motor_outputs(float roll, float pitch, float yaw, float throttle,
//                              float *motor1_output, float *motor2_output,
//                              float *motor3_output, float *motor4_output);
