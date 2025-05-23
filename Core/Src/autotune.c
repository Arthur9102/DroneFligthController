/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : autotune.c
  * @brief          : PID Autotuning using Genetic Algorithm for STM32
  ******************************************************************************
  * @attention
  * Implementation of PID autotuning based on Tsounis' thesis.
  * Uses Genetic Algorithm to tune Kp and Ki for a PI controller.
  * Simulates pitch dynamics and outputs results via UART.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "autotune.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <main.h>
/* External variables */
extern UART_HandleTypeDef huart6;  // UART handle for debug output
extern float _roll_setpoint;       // Roll setpoint in radians 
extern float _pitch_setpoint;      // Pitch setpoint in radians
extern float _roll_measured;       // Measured roll angle
extern float _output_roll;         // Roll control output

// PID controller structures
extern CascadedPID_t _pid_roll_cascade;   // Roll axis cascaded PID
extern CascadedPID_t _pid_pitch_cascade;  // Pitch axis cascaded PID

Individual population[POP_SIZE];
Individual new_population[POP_SIZE];
char uart_buf[100];

/* Generate random float between min and max */
float rand_float(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}

/* Initialize population with random Kp, Ki */
void init_population(void) {
    for (int i = 0; i < POP_SIZE; i++) {
        population[i].Kp = rand_float(KP_MIN, KP_MAX);
        population[i].Kd = rand_float(KD_MIN, KD_MAX);
        population[i].fitness = 0.0f;
    }
}

/* Simulate pitch dynamics with PI controller */
float simulate_pitch(float Kp, float Kd) {
    // Discretized second-order system (approximated from thesis)
    // Transfer function: G(s) = 1.5/(s^2 + 2s + 1.5)
    // Discretized using Tustin method (simplified coefficients)
    float a1 = -1.98f, a2 = 0.98f, b0 = 0.015f;
    float y[3] = {0.0f, 0.0f, 0.0f}; // Output history
    float u[2] = {0.0f, 0.0f};       // Input history
    float error_integral = 0.0f;
    float error, u_new;
    int steps = (int)(SIM_TIME / DT);
    float output[steps];

    for (int t = 0; t < steps; t++) {
        error = SETPOINT - y[0];
        error_integral += error * DT;
        u_new = Kp * error + Kd * error_integral;

        // Update system dynamics
        y[0] = -a1 * y[1] - a2 * y[2] + b0 * u_new;
        output[t] = y[0];

        // Shift history
        y[2] = y[1];
        y[1] = y[0];
        u[1] = u[0];
        u[0] = u_new;
    }

    return calculate_fitness(output, steps);
}

/* Calculate fitness based on performance criteria */
float calculate_fitness(float *output, int steps) {
    float rise_time = 0.0f, overshoot = 0.0f, settling_time = 0.0f;
    float threshold = 0.9f * SETPOINT;
    int rise_idx = -1, settle_idx = -1;
    float max_output = SETPOINT;

    for (int i = 0; i < steps; i++) {
        if (output[i] >= threshold && rise_idx == -1) {
            rise_idx = i;
            rise_time = i * DT;
        }
        if (output[i] > max_output) {
            max_output = output[i];
        }
        if (fabs(output[i] - SETPOINT) <= 0.02f * SETPOINT) {
            settle_idx = i;
            settling_time = i * DT;
        }
    }

    overshoot = (max_output - SETPOINT) / SETPOINT * 100.0f;

    // Fitness: Minimize rise time, overshoot, settling time
    // Weighted sum (tuned empirically)
    return 1.0f / (0.5f * rise_time + 0.3f * overshoot + 0.2f * settling_time + 1e-6f);
}

/* Select parents using tournament selection */
void select_parents(int *parent1, int *parent2) {
    int best1 = rand() % POP_SIZE, best2 = rand() % POP_SIZE;
    for (int i = 0; i < 3; i++) {
        int idx = rand() % POP_SIZE;
        if (population[idx].fitness > population[best1].fitness) best1 = idx;
        idx = rand() % POP_SIZE;
        if (population[idx].fitness > population[best2].fitness) best2 = idx;
    }
    *parent1 = best1;
    *parent2 = best2;
}

/* Perform crossover between parents */
void crossover(Individual *parent1, Individual *parent2, Individual *child1, Individual *child2) {
    if (rand_float(0.0f, 1.0f) < CROSSOVER_RATE) {
        float alpha = rand_float(0.0f, 1.0f);
        child1->Kp = alpha * parent1->Kp + (1.0f - alpha) * parent2->Kp;
        child1->Kd = alpha * parent1->Kd + (1.0f - alpha) * parent2->Kd;
        child2->Kp = alpha * parent2->Kp + (1.0f - alpha) * parent1->Kp;
        child2->Kd = alpha * parent2->Kd + (1.0f - alpha) * parent1->Kd;
    } else {
        child1->Kp = parent1->Kp;
        child1->Kd = parent1->Kd;
        child2->Kp = parent2->Kp;
        child2->Kd = parent2->Kd;
    }
}

/* Mutate an individual */
void mutate(Individual *ind) {
    if (rand_float(0.0f, 1.0f) < MUTATION_RATE) {
        ind->Kp += rand_float(-0.5f, 0.5f);
        if (ind->Kp < KP_MIN) ind->Kp = KP_MIN;
        if (ind->Kp > KP_MAX) ind->Kp = KP_MAX;
    }
    if (rand_float(0.0f, 1.0f) < MUTATION_RATE) {
        ind->Kd += rand_float(-0.5f, 0.5f);
        if (ind->Kd < KD_MIN) ind->Kd = KD_MIN;
        if (ind->Kd > KD_MAX) ind->Kd = KD_MAX;
    }
}

/* Main autotuning function */
void autotune_pid(float *Kp, float *Kd) {
    const int MAX_SAMPLES = 500;  // 5 giây @ 10ms/mẫu
    const int STAGNATION_LIMIT = 5;  // Số thế hệ tối đa không cải thiện
    TestData *test_data = malloc(sizeof(TestData) * MAX_SAMPLES);
    
    if (test_data == NULL) {
        return;
    }

    init_population();
    
    float best_fitness_ever = 0.0f;
    int stagnation_counter = 0;
    int best_generation = 0;
    
    char buf[50];
    sprintf(buf, "Starting autotuning...\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)buf, strlen(buf), 100);

    for (int gen = 0; gen < MAX_GENERATIONS; gen++) {
        // Đánh giá từng cá thể
        float generation_best_fitness = 0.0f;
        
        for (int i = 0; i < POP_SIZE; i++) {
            population[i].fitness = evaluate_controller(
                population[i].Kp, 
                population[i].Kd,
                test_data,
                MAX_SAMPLES
            );
            
            // Cập nhật fitness tốt nhất của thế hệ hiện tại
            if (population[i].fitness > generation_best_fitness) {
                generation_best_fitness = population[i].fitness;
            }
        }

        // Kiểm tra cải thiện
        if (generation_best_fitness > best_fitness_ever) {
            best_fitness_ever = generation_best_fitness;
            best_generation = gen;
            stagnation_counter = 0;  // Reset counter
            
            // Thông báo cải thiện
            sprintf(buf, "Gen %d: New best fitness: %.4f\r\n", 
                    gen + 1, best_fitness_ever);
            HAL_UART_Transmit(&huart6, (uint8_t*)buf, strlen(buf), 100);
        } else {
            stagnation_counter++;
            
            // Thông báo không cải thiện
            sprintf(buf, "Gen %d: No improvement, count: %d\r\n", 
                    gen + 1, stagnation_counter);
            HAL_UART_Transmit(&huart6, (uint8_t*)buf, strlen(buf), 100);
        }

        // Kiểm tra điều kiện dừng sớm
        if (stagnation_counter >= STAGNATION_LIMIT) {
            sprintf(buf, "Early stopping at generation %d\r\n", gen + 1);
            HAL_UART_Transmit(&huart6, (uint8_t*)buf, strlen(buf), 100);
            break;
        }

        // Tạo thế hệ mới
        for (int i = 0; i < POP_SIZE; i += 2) {
            int parent1, parent2;
            select_parents(&parent1, &parent2);
            crossover(&population[parent1], &population[parent2],
                     &new_population[i], &new_population[i+1]);
            mutate(&new_population[i]);
            mutate(&new_population[i+1]);
        }

        memcpy(population, new_population, sizeof(population));
    }

    // Tìm cá thể tốt nhất
    float best_fitness = 0.0f;
    int best_idx = 0;
    for (int i = 0; i < POP_SIZE; i++) {
        if (population[i].fitness > best_fitness) {
            best_fitness = population[i].fitness;
            best_idx = i;
        }
    }

    *Kp = population[best_idx].Kp;
    *Kd = population[best_idx].Kd;

    // Gửi kết quả cuối cùng
    sprintf(buf, "Best PID: Kp=%.3f Kd=%.3f (Gen %d)\r\n", 
            *Kp, *Kd, best_generation + 1);
    HAL_UART_Transmit(&huart6, (uint8_t*)buf, strlen(buf), 100);

    free(test_data);
}

/* Đánh giá bộ điều khiển với dữ liệu thực nghiệm */
float evaluate_controller(float Kp, float Kd, TestData* data, int max_samples) {
    float error_integral = 0.0f;
//    float prev_output = 0.0f;
    int sample_count = 0;
    uint32_t start_time = HAL_GetTick();
    
    // Reset các biến trạng thái
    _roll_setpoint = 0.0f;
    _pitch_setpoint = 0.0f;
    error_integral = 0.0f;

    // Create local variables to store the PID parameters
    float roll_rate_kp = Kp;
    float roll_rate_kd = Kd;
    float pitch_rate_kp = Kp;
    float pitch_rate_kd = Kd;

    // Đặt các hệ số PID mới
    _pid_roll_cascade.rate.kp = &roll_rate_kp;
    _pid_roll_cascade.rate.kd = &roll_rate_kd;
    _pid_pitch_cascade.rate.kp = &pitch_rate_kp;
    _pid_pitch_cascade.rate.kd = &pitch_rate_kd;

    // Chờ hệ thống ổn định
    HAL_Delay(1000);

    // Thử nghiệm với bước nhảy
    _roll_setpoint = SETPOINT;  // Đặt giá trị đặt mới

    // Thu thập dữ liệu trong 5 giây
    while (sample_count < max_samples) {
        uint32_t current_time = HAL_GetTick();
        
        // Lưu dữ liệu mỗi 10ms
        if (current_time - start_time >= sample_count * 10) {
            data[sample_count].time = current_time - start_time;
            data[sample_count].setpoint = _roll_setpoint;
            data[sample_count].output = _roll_measured;
            data[sample_count].error = _roll_setpoint - _roll_measured;
            data[sample_count].control = _output_roll;
            
            sample_count++;
        }
        
        osDelay(1); // Nhường CPU
    }

    // Reset lại setpoint
    _roll_setpoint = 0.0f;
    HAL_Delay(1000);

    return calculate_fitness_real(data, max_samples);
}

/* Tính toán fitness dựa trên dữ liệu thực nghiệm */
float calculate_fitness_real(TestData* data, int samples) {
    float rise_time = 0.0f;
    float overshoot = 0.0f;
    float settling_time = 0.0f;
    float steady_state_error = 0.0f;
    float threshold = 0.9f * SETPOINT;
    int rise_idx = -1;
    
    float max_output = data[0].output;
    float final_value = 0.0f;

    float ise = 0.0f; // Integral Square Error
    
    // Tính các chỉ số
    for (int i = 0; i < samples; i++) {
        // Tính ISE
        float error = data[i].error;
        ise += error * error * (data[i].time / 1000.0f); // Nhân với dt (s)
        
        // Rise time
        if (data[i].output >= threshold && rise_idx == -1) {
            rise_idx = i;
            rise_time = data[i].time / 1000.0f;
        }

        // Maximum overshoot
        if (data[i].output > max_output) {
            max_output = data[i].output;
        }
        
        // Settling time (±2%)
        if (fabs(data[i].output - SETPOINT) <= 0.02f * SETPOINT) {
            settling_time = data[i].time / 1000.0f;
        }
        
        // Tính giá trị trung bình cuối
        if (i >= samples - 10) {
            final_value += data[i].output;
        }
    }
    
    final_value /= 10.0f;
    overshoot = (max_output - SETPOINT) / SETPOINT * 100.0f;
    steady_state_error = fabs(SETPOINT - final_value);
    
    // Tính điểm fitness (càng nhỏ càng tốt)
    float fitness = 0.0f;
    fitness += 0.45f * (ise / samples);        // 45% cho ISE
    fitness += 0.25f * (overshoot / 100.0f);   // 25% cho overshoot
    fitness += 0.15f * rise_time;              // 15% cho rise time
    fitness += 0.10f * settling_time;       // 10% cho thời gian ổn định
    fitness += 0.05f * steady_state_error;  // 5% cho sai số ổn định

    return 1.0f / (fitness + 1e-6f);  // Đảo ngược để có fitness càng lớn càng tốt
}

