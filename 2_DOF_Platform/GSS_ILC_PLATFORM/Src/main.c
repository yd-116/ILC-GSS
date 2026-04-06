#include "stm32f10x.h"
#include "delay.h"
#include "oled.h"
#include "sys.h"
#include <stdlib.h>

#define TRAJ_LENGTH 20
#define LEARNING_GAIN 4.7
#define MAX_ITERATIONS 5

#define HORIZ_AMPLITUDE 300
#define HORIZ_CENTER 600

#define VERT_AMPLITUDE 150
#define VERT_CENTER 750

#define PWM_0_DEG 500
#define PWM_180_DEG 1000

#define SERVO_GAIN 60
#define SERVO_OFFSET 20

static int iteration = 0;
static int sample_count = 0;

static int horiz_pwm_k[TRAJ_LENGTH] = {0};
static int horiz_e_k[TRAJ_LENGTH] = {0};
static int horiz_pwm_next[TRAJ_LENGTH] = {0};
static int horiz_actual_history[30][20] = {0};
static int horiz_desired_history[30][20] = {0};
static int horiz_pwm_history[30][20] = {0};

static int vert_pwm_k[TRAJ_LENGTH] = {0};
static int vert_e_k[TRAJ_LENGTH] = {0};
static int vert_pwm_next[TRAJ_LENGTH] = {0};
static int vert_actual_history[30][20] = {0};
static int vert_desired_history[30][20] = {0};
static int vert_pwm_history[30][20] = {0};

int angle_to_pwm(int angle_deci) {
    if(angle_deci < 0) angle_deci = 0;
    if(angle_deci > 1800) angle_deci = 1800;
    int pwm = PWM_0_DEG + angle_deci * (PWM_180_DEG - PWM_0_DEG) / 1800;
    return pwm;
}

int pwm_to_angle(int pwm) {
    if(pwm < PWM_0_DEG) pwm = PWM_0_DEG;
    if(pwm > PWM_180_DEG) pwm = PWM_180_DEG;
    int angle = (pwm - PWM_0_DEG) * 1800 / (PWM_180_DEG - PWM_0_DEG);
    return angle;
}

int servo_model(int pwm) {
    if(pwm < PWM_0_DEG) pwm = PWM_0_DEG;
    if(pwm > PWM_180_DEG) pwm = PWM_180_DEG;
    int ideal_angle = pwm_to_angle(pwm);
    int actual_angle = (ideal_angle * SERVO_GAIN) / 100 + SERVO_OFFSET;
    return actual_angle;
}

void display_servo_data(int servo_num) {
    OLED_Clear();

    OLED_ShowString(0, 0, (uint8_t*)"Iter:");
    OLED_ShowNumber(30, 0, iteration+1, 2, 12);

    if(servo_num == 1) {
        OLED_ShowString(48, 0, (uint8_t*)" Servo1");
    } else {
        OLED_ShowString(48, 0, (uint8_t*)" Servo2");
    }

    for(int row = 0; row < 5; row++) {
        int y_pos = 12 + row * 10;

        for(int col = 0; col < 4; col++) {
            int idx = row * 4 + col;
            int x_pos = col * 32;

            if(idx < TRAJ_LENGTH) {
                int actual_angle;
                if(servo_num == 1) {
                    actual_angle = vert_actual_history[iteration][idx];
                } else {
                    actual_angle = horiz_actual_history[iteration][idx];
                }

                OLED_ShowNumber(x_pos, y_pos, actual_angle/10, 2, 12);
                OLED_ShowString(x_pos + 12, y_pos, (uint8_t*)".");
                OLED_ShowNumber(x_pos + 18, y_pos, actual_angle%10, 1, 12);
            }
        }
    }

    OLED_Refresh_Gram();
}

int main(void) {
    delay_init();
    OLED_Init();
    OLED_Clear();

    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    GPIOB->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);
    GPIOB->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1;

    GPIOB->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOB->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1;

    TIM4->PSC = 143;
    TIM4->ARR = 9999;

    TIM4->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);
    TIM4->CCMR2 |= (0x6 << 4) | (0x6 << 12);
    TIM4->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    TIM4->CCER = 0x0000;
    TIM4->CCER |= TIM_CCER_CC3E;
    TIM4->CCER &= ~TIM_CCER_CC3P;
    TIM4->CCER |= TIM_CCER_CC4E;
    TIM4->CCER &= ~TIM_CCER_CC4P;

    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_CEN;

    static const int sin_table[20] = {
        0, 31, 59, 81, 95, 100, 95, 81, 59, 31,
        0, -31, -59, -81, -95, -100, -95, -81, -59, -31
    };

    static const int cos_table[20] = {
        100, 95, 81, 59, 31, 0, -31, -59, -81, -95,
        -100, -95, -81, -59, -31, 0, 31, 59, 81, 95
    };

    for(int i = 0; i < TRAJ_LENGTH; i++) {
        int horiz_yd = HORIZ_CENTER + (HORIZ_AMPLITUDE * sin_table[i]) / 100;
        horiz_pwm_k[i] = angle_to_pwm(horiz_yd);
        horiz_pwm_next[i] = 0;
        horiz_desired_history[0][i] = horiz_yd;

        int vert_yd = VERT_CENTER + (VERT_AMPLITUDE * cos_table[i]) / 100;
        vert_pwm_k[i] = angle_to_pwm(vert_yd);
        vert_pwm_next[i] = 0;
        vert_desired_history[0][i] = vert_yd;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"Start ILC");
    OLED_ShowString(0, 16, (uint8_t*)"Servo1: Cosine");
    OLED_ShowString(0, 32, (uint8_t*)"Servo2: Sine");
    OLED_Refresh_Gram();
    delay_ms(3000);

    display_servo_data(1);
    delay_ms(5000);

    display_servo_data(2);
    delay_ms(5000);

    while(1) {
        int horiz_yd = HORIZ_CENTER + (HORIZ_AMPLITUDE * sin_table[sample_count]) / 100;
        int vert_yd = VERT_CENTER + (VERT_AMPLITUDE * cos_table[sample_count]) / 100;

        horiz_desired_history[iteration][sample_count] = horiz_yd;
        vert_desired_history[iteration][sample_count] = vert_yd;

        int horiz_current_pwm = horiz_pwm_k[sample_count];
        int vert_current_pwm = vert_pwm_k[sample_count];

        horiz_pwm_history[iteration][sample_count] = horiz_current_pwm;
        vert_pwm_history[iteration][sample_count] = vert_current_pwm;

        TIM4->CCR3 = horiz_current_pwm;
        TIM4->CCR4 = vert_current_pwm;

        int horiz_actual_angle = servo_model(horiz_current_pwm);
        int vert_actual_angle = servo_model(vert_current_pwm);

        horiz_actual_history[iteration][sample_count] = horiz_actual_angle;
        vert_actual_history[iteration][sample_count] = vert_actual_angle;

        horiz_e_k[sample_count] = horiz_yd - horiz_actual_angle;
        vert_e_k[sample_count] = vert_yd - vert_actual_angle;

        delay_ms(200);

        sample_count++;

        if(sample_count >= TRAJ_LENGTH) {
            display_servo_data(1);
            delay_ms(5000);

            display_servo_data(2);
            delay_ms(5000);

            for(int i = 0; i < TRAJ_LENGTH; i++) {
                horiz_pwm_next[i] = horiz_pwm_k[i] + (LEARNING_GAIN * horiz_e_k[i]) / 10;

                if(horiz_pwm_next[i] < PWM_0_DEG) horiz_pwm_next[i] = PWM_0_DEG;
                if(horiz_pwm_next[i] > PWM_180_DEG) horiz_pwm_next[i] = PWM_180_DEG;

                vert_pwm_next[i] = vert_pwm_k[i] + (LEARNING_GAIN * vert_e_k[i]) / 10;

                if(vert_pwm_next[i] < PWM_0_DEG) vert_pwm_next[i] = PWM_0_DEG;
                if(vert_pwm_next[i] > PWM_180_DEG) vert_pwm_next[i] = PWM_180_DEG;
            }

            iteration++;

            if(iteration >= MAX_ITERATIONS) {
                OLED_Clear();
                OLED_ShowString(0, 0, (uint8_t*)"ILC COMPLETED");
                while(1) delay_ms(1000);
            }

            for(int i = 0; i < TRAJ_LENGTH; i++) {
                horiz_pwm_k[i] = horiz_pwm_next[i];
                vert_pwm_k[i] = vert_pwm_next[i];
            }

            sample_count = 0;

            TIM4->CCR3 = angle_to_pwm(HORIZ_CENTER);
            TIM4->CCR4 = angle_to_pwm(VERT_CENTER);

            OLED_Clear();
            OLED_ShowString(0, 0, (uint8_t*)"Next Iter");
            OLED_ShowString(0, 16, (uint8_t*)"Iter:");
            OLED_ShowNumber(30, 16, iteration+1, 2, 12);
            OLED_Refresh_Gram();
            delay_ms(5000);
        }
    }
}
