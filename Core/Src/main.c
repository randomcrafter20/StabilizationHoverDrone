/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>

#include "bno08x_hal.h"
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "bmp280.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t bno_irq_flag = 0;
static sh2_SensorValue_t sensorValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
static void I2C_BusRecovery(void);
static void I2C_Scan(void);
static void BMP280_CheckID(void);

static int BNO08x_EnableGameRV(void);
static int BNO08x_EnableGyro(void);

static void update_attitude_filter_from_quaternion(void);
static void update_rc_link_status(void);
static void update_targets_from_rc(void);
static void apply_rc_failsafe(void);
static void run_stabilization_loop(void);
static void run_output_loop(void);
static void run_hover_altitude_loop(void);

static bool BMP280_InitSensor(void);
static bool BMP280_ReadData(float *temp_c, float *press_pa);
static float BMP280_ComputeAltitude(float pressure, float pressure0);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RAD_TO_DEG               57.2957795f
#define CALIBRATION_SAMPLES      100
#define CONTROL_DT_S             0.005f

#define ESC_MIN_US               1050.0f
#define ESC_ARM_US               1050.0f
#define ESC_IDLE_US              1100.0f
#define ESC_MAX_US               1950.0f

#define MOTOR_OUTPUT_MIN         ESC_IDLE_US
#define MOTOR_OUTPUT_MAX         ESC_MAX_US

#define MOTOR1_CHANNEL           TIM_CHANNEL_1
#define MOTOR2_CHANNEL           TIM_CHANNEL_2
#define MOTOR3_CHANNEL           TIM_CHANNEL_3
#define MOTOR4_CHANNEL           TIM_CHANNEL_4

#define RC_NUM_CHANNELS          6
#define RC_MIN_US                1000
#define RC_MID_US                1500
#define RC_MAX_US                2000
#define RC_TIMEOUT_MS            100
#define RC_DEADBAND_US           20

static bool hover_mode_latched = false;
static float hover_target_altitude_m = 0.0f;
static float hover_throttle_us = 1350.0f;   // initial guess, tune this
static float altitude_hold_output_us = 0.0f;
typedef enum
{
    MODE_STABILIZE = 0,
    MODE_HOVER     = 1
} FlightMode_t;

typedef struct PID_s
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} PID_t;

typedef struct
{
    float x;
    float y;
    float z;
} Vec3f_t;

typedef struct
{
    float y;
    float alpha;
    bool initialized;
} LPF1_t;

static BMP280_HandleTypedef bmp280;
static bmp280_params_t bmp280_params;
static bool bmp280_ok = false;

static float pressure_pa = 0.0f;
static float temperature_c = 0.0f;
static float altitude_m = 0.0f;
static float pressure_sum = 0.0f;
static float pressure_baseline_pa = 101325.0f;

/* ---- raw quaternion from BNO ---- */
static float quat_r = 1.0f;
static float quat_i = 0.0f;
static float quat_j = 0.0f;
static float quat_k = 0.0f;

/* ---- filtered attitude ---- */
static float roll_filt_deg = 0.0f;
static float pitch_filt_deg = 0.0f;
static float yaw_dbg_deg = 0.0f;
static float altitude_filt_m = 0.0f;

static LPF1_t roll_lpf    = {.y = 0.0f, .alpha = 0.20f, .initialized = false};
static LPF1_t pitch_lpf   = {.y = 0.0f, .alpha = 0.20f, .initialized = false};
static LPF1_t yaw_dbg_lpf = {.y = 0.0f, .alpha = 0.20f, .initialized = false};
static LPF1_t altitude_lpf = {.y = 0.0f, .alpha = 0.10f, .initialized = false};

static volatile uint32_t bno_event_count = 0;
static volatile uint32_t bno_grv_count = 0;
static volatile uint32_t bno_gyro_count = 0;
static uint32_t bmp_read_count = 0;


/* ---- gyro ---- */
static float gyro_x_dps = 0.0f;
static float gyro_y_dps = 0.0f;
static float gyro_z_dps = 0.0f;

/* ---- offsets ---- */
static float roll_offset_deg = 0.0f;
static float pitch_offset_deg = 0.0f;
static float yaw_offset_deg = 0.0f;

static float gyro_x_bias = 0.0f;
static float gyro_y_bias = 0.0f;
static float gyro_z_bias = 0.0f;

static float roll_sum = 0.0f;
static float pitch_sum = 0.0f;
static float yaw_sum = 0.0f;

static float gyro_x_sum = 0.0f;
static float gyro_y_sum = 0.0f;
static float gyro_z_sum = 0.0f;

static int cal_count = 0;
static bool calibrated = false;

/* ---- targets ---- */
static float target_roll_deg = 0.0f;
static float target_pitch_deg = 0.0f;
static float target_roll_rate_dps = 0.0f;
static float target_pitch_rate_dps = 0.0f;
static float target_yaw_rate_dps = 0.0f;

/* ---- controller outputs ---- */
static float roll_rate_target_dps = 0.0f;
static float pitch_rate_target_dps = 0.0f;
static float roll_control = 0.0f;
static float pitch_control = 0.0f;
static float yaw_control = 0.0f;
static float throttle_control = 0.0f;

/* ---- motor state ---- */
static float base_throttle_us = ESC_ARM_US;
static float motor1_us = ESC_ARM_US;
static float motor2_us = ESC_ARM_US;
static float motor3_us = ESC_ARM_US;
static float motor4_us = ESC_ARM_US;
static bool motors_armed = false;

/* ---- RC input state ---- */
/* Update these comments to match your actual pins in CubeMX/main.h */
static volatile uint16_t rc_rise_time_us[RC_NUM_CHANNELS] = {0};
static volatile uint16_t rc_pulse_us[RC_NUM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000};

static uint32_t rc_last_update_ms = 0;
static bool rc_connected = false;
static FlightMode_t flight_mode = MODE_STABILIZE;

#define RC_CH_ROLL_IDX      0
#define RC_CH_PITCH_IDX     1
#define RC_CH_THROTTLE_IDX  2
#define RC_CH_YAW_IDX       3
#define RC_CH_ARM_IDX       4
#define RC_CH_MODE_IDX      5

static PID_t pid_roll_angle = {
    .kp = 1.2f, .ki = 0.0f, .kd = 0.0f,
    .integral = 0.0f, .prev_error = 0.0f,
    .out_min = -80.0f, .out_max = 80.0f
};

static PID_t pid_pitch_angle = {
    .kp = 1.2f, .ki = 0.0f, .kd = 0.0f,
    .integral = 0.0f, .prev_error = 0.0f,
    .out_min = -80.0f, .out_max = 80.0f
};

static PID_t pid_roll_rate = {
    .kp = 0.015f, .ki = 0.0f, .kd = 0.0f,
    .integral = 0.0f, .prev_error = 0.0f,
    .out_min = -120.0f, .out_max = 120.0f
};

static PID_t pid_pitch_rate = {
    .kp = 0.015f, .ki = 0.0f, .kd = 0.0f,
    .integral = 0.0f, .prev_error = 0.0f,
    .out_min = -120.0f, .out_max = 120.0f
};

static PID_t pid_yaw_rate = {
    .kp = 0.020f, .ki = 0.0f, .kd = 0.0f,
    .integral = 0.0f, .prev_error = 0.0f,
    .out_min = -80.0f, .out_max = 80.0f
};
static PID_t pid_altitude = {
    .kp = 180.0f, .ki = 20.0f, .kd = 0.0f,
    .integral = 0.0f, .prev_error = 0.0f,
    .out_min = -200.0f, .out_max = 200.0f
};

static float clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static int16_t clamp_i16(int16_t x, int16_t min_val, int16_t max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int16_t apply_deadband(int16_t value, int16_t center, int16_t deadband)
{
    if (value > (center - deadband) && value < (center + deadband))
    {
        return center;
    }
    return value;
}

static float pid_update(PID_t *pid, float error, float dt)
{
    float derivative;
    float out;

    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    pid->integral += error * dt;

    if (pid->ki > 0.0f)
    {
        float integral_limit = fmaxf(fabsf(pid->out_max / pid->ki), fabsf(pid->out_min / pid->ki));
        pid->integral = clampf(pid->integral, -integral_limit, integral_limit);
    }

    derivative = (error - pid->prev_error) / dt;

    out = pid->kp * error
        + pid->ki * pid->integral
        + pid->kd * derivative;

    out = clampf(out, pid->out_min, pid->out_max);
    pid->prev_error = error;

    return out;
}

static void pid_reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static float lpf1_update(LPF1_t *f, float x)
{
    if (!f->initialized)
    {
        f->y = x;
        f->initialized = true;
        return f->y;
    }

    f->y += f->alpha * (x - f->y);
    return f->y;
}

static Vec3f_t remap_vec3(float x, float y, float z)
{
    Vec3f_t v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

static uint32_t pwm_us_to_counts(float us)
{
    us = clampf(us, ESC_MIN_US, ESC_MAX_US);
    return (uint32_t)(us + 0.5f);
}

static void motor_write_us(float m1, float m2, float m3, float m4)
{
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_1, pwm_us_to_counts(m1)); // PA0  M1
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_2, pwm_us_to_counts(m2)); // PB3  M2
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pwm_us_to_counts(m3)); // PA7  M3
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_2, pwm_us_to_counts(m4)); // PA4  M4
}

static void motors_stop(void)
{
    motor1_us = ESC_ARM_US;
    motor2_us = ESC_ARM_US;
    motor3_us = ESC_ARM_US;
    motor4_us = ESC_ARM_US;
    motor_write_us(motor1_us, motor2_us, motor3_us, motor4_us);
}

static void motors_pwm_start(void)
{
    if (HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    motors_stop();
}

static void motor_mix_and_output(void)
{
    float throttle = base_throttle_us + throttle_control;

    // 🔴 SAFETY: if throttle low, kill corrections
    if (base_throttle_us < 1150.0f)
    {
        roll_control = 0.0f;
        pitch_control = 0.0f;
        yaw_control = 0.0f;
        throttle_control = 0.0f;
    }

    motor1_us = throttle + pitch_control + roll_control + yaw_control;
    motor2_us = throttle + pitch_control - roll_control - yaw_control;
    motor3_us = throttle - pitch_control - roll_control + yaw_control;
    motor4_us = throttle - pitch_control + roll_control - yaw_control;

    motor1_us = clampf(motor1_us, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    motor2_us = clampf(motor2_us, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    motor3_us = clampf(motor3_us, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    motor4_us = clampf(motor4_us, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);

    // 🔴 SAFETY: disarmed
    if (!motors_armed)
    {
        motors_stop();
        return;
    }

    motor_write_us(motor1_us, motor2_us, motor3_us, motor4_us);
}

static uint16_t rc_timer_us(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

static void rc_handle_edge(uint8_t ch_idx, GPIO_TypeDef *port, uint16_t pin)
{
    uint16_t now_us = rc_timer_us();

    if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
    {
        rc_rise_time_us[ch_idx] = now_us;
    }
    else
    {
        uint16_t width = (uint16_t)(now_us - rc_rise_time_us[ch_idx]);

        if (width >= 900U && width <= 2200U)
        {
            rc_pulse_us[ch_idx] = width;
            rc_last_update_ms = HAL_GetTick();
        }
    }
}

static void update_rc_link_status(void)
{
    uint32_t now = HAL_GetTick();
    rc_connected = ((now - rc_last_update_ms) < RC_TIMEOUT_MS);
}

static void update_targets_from_rc(void)
{
    int16_t ch1 = clamp_i16((int16_t)rc_pulse_us[RC_CH_ROLL_IDX], RC_MIN_US, RC_MAX_US);
    int16_t ch2 = clamp_i16((int16_t)rc_pulse_us[RC_CH_PITCH_IDX], RC_MIN_US, RC_MAX_US);
    int16_t ch3 = clamp_i16((int16_t)rc_pulse_us[RC_CH_THROTTLE_IDX], RC_MIN_US, RC_MAX_US);
    int16_t ch4 = clamp_i16((int16_t)rc_pulse_us[RC_CH_YAW_IDX], RC_MIN_US, RC_MAX_US);
    int16_t ch5 = clamp_i16((int16_t)rc_pulse_us[RC_CH_ARM_IDX], RC_MIN_US, RC_MAX_US);
    int16_t ch6 = clamp_i16((int16_t)rc_pulse_us[RC_CH_MODE_IDX], RC_MIN_US, RC_MAX_US);

    ch1 = apply_deadband(ch1, RC_MID_US, RC_DEADBAND_US);
    ch2 = apply_deadband(ch2, RC_MID_US, RC_DEADBAND_US);
    ch4 = apply_deadband(ch4, RC_MID_US, RC_DEADBAND_US);

    motors_armed = (ch5 > 1500);
    static FlightMode_t last_mode = MODE_STABILIZE;

    if (ch6 > 1700)
    {
        last_mode = MODE_HOVER;
    }
    else if (ch6 < 1300)
    {
        last_mode = MODE_STABILIZE;
    }

    flight_mode = last_mode;

    target_roll_deg  = mapf((float)ch1, 1000.0f, 2000.0f, -20.0f, 20.0f);
    target_pitch_deg = mapf((float)ch2, 1000.0f, 2000.0f, -20.0f, 20.0f);
    target_roll_rate_dps = 0.0f;
    target_pitch_rate_dps = 0.0f;
    target_yaw_rate_dps = mapf((float)ch4, 1000.0f, 2000.0f, -120.0f, 120.0f);

    if (flight_mode == MODE_HOVER)
    {
        if (!hover_mode_latched)
        {
            hover_target_altitude_m = altitude_m;
            hover_mode_latched = true;

            pid_reset(&pid_altitude);
            altitude_hold_output_us = 0.0f;
        }

        {
            float climb_cmd = mapf((float)ch3, 1000.0f, 2000.0f, -0.5f, 0.5f);

            if (fabsf(climb_cmd) < 0.05f)
            {
                climb_cmd = 0.0f;
            }

            hover_target_altitude_m += climb_cmd * 0.02f;
            hover_target_altitude_m = clampf(hover_target_altitude_m, -5.0f, 50.0f);
        }

        base_throttle_us = hover_throttle_us;
    }
    else
    {
        hover_mode_latched = false;
        base_throttle_us = mapf((float)ch3, 1000.0f, 2000.0f, ESC_IDLE_US, 1700.0f);
    }
}

static void run_hover_altitude_loop(void)
{
    float altitude_error;

    altitude_error = hover_target_altitude_m - altitude_filt_m;

    altitude_hold_output_us = pid_update(&pid_altitude, altitude_error, CONTROL_DT_S);

    throttle_control = altitude_hold_output_us;
}

static void apply_rc_failsafe(void)
{
    target_roll_deg = 0.0f;
    target_pitch_deg = 0.0f;
    target_roll_rate_dps = 0.0f;
    target_pitch_rate_dps = 0.0f;
    target_yaw_rate_dps = 0.0f;
    base_throttle_us = ESC_ARM_US;
    motors_armed = false;
    flight_mode = MODE_STABILIZE;
}

static void update_attitude_filter_from_quaternion(void)
{
    float qr = quat_r;
    float qi = quat_i;
    float qj = quat_j;
    float qk = quat_k;
    float gx, gy, gz;
    float roll_meas_deg;
    float pitch_meas_deg;
    float yaw_dbg_meas_deg;
    float norm;

    norm = sqrtf(qr*qr + qi*qi + qj*qj + qk*qk);
    if (norm > 0.0f)
    {
        qr /= norm;
        qi /= norm;
        qj /= norm;
        qk /= norm;
    }

    gx = 2.0f * (qi * qk - qr * qj);
    gy = 2.0f * (qr * qi + qj * qk);
    gz = qr*qr - qi*qi - qj*qj + qk*qk;

    roll_meas_deg  = atan2f(gy, gz) * RAD_TO_DEG;
    pitch_meas_deg = atan2f(-gx, sqrtf(gy*gy + gz*gz)) * RAD_TO_DEG;
    yaw_dbg_meas_deg = atan2f(2.0f * (qr * qk + qi * qj),
                              1.0f - 2.0f * (qj * qj + qk * qk)) * RAD_TO_DEG;

    roll_filt_deg  = lpf1_update(&roll_lpf, roll_meas_deg);
    pitch_filt_deg = lpf1_update(&pitch_lpf, pitch_meas_deg);
    yaw_dbg_deg    = lpf1_update(&yaw_dbg_lpf, yaw_dbg_meas_deg);
}

static bool BMP280_InitSensor(void)
{
    bmp280.i2c = &hi2c1;

    bmp280_init_default_params(&bmp280_params);

    bmp280_params.mode = BMP280_MODE_NORMAL;
    bmp280_params.filter = BMP280_FILTER_4;
    bmp280_params.oversampling_pressure = BMP280_ULTRA_HIGH_RES;
    bmp280_params.oversampling_temperature = BMP280_STANDARD;
    bmp280_params.standby = BMP280_STANDBY_250;

    bmp280.addr = BMP280_I2C_ADDRESS_0; // 0x76
    if (bmp280_init(&bmp280, &bmp280_params))
    {
        return true;
    }

    bmp280.addr = BMP280_I2C_ADDRESS_1; // 0x77
    if (bmp280_init(&bmp280, &bmp280_params))
    {
        return true;
    }

    return false;
}

static bool BMP280_ReadData(float *temp_c, float *press_pa)
{
    if (!bmp280_read_float(&bmp280, temp_c, press_pa, NULL))
    {
        return false;
    }

    return true;
}

static float BMP280_ComputeAltitude(float pressure, float pressure0)
{
    if (pressure <= 0.0f || pressure0 <= 0.0f)
    {
        return 0.0f;
    }

    return 44330.0f * (1.0f - powf(pressure / pressure0, 0.19029495f));
}

static void sensorHandler(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;

    bno_event_count++;

    if (sh2_decodeSensorEvent(&sensorValue, event) != SH2_OK)
    {
        return;
    }

    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR)
    {
        bno_grv_count++;

        quat_i = sensorValue.un.gameRotationVector.i;
        quat_j = sensorValue.un.gameRotationVector.j;
        quat_k = sensorValue.un.gameRotationVector.k;
        quat_r = sensorValue.un.gameRotationVector.real;
    }
    else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED)
    {
        bno_gyro_count++;

        Vec3f_t g = remap_vec3(
            sensorValue.un.gyroscope.x,
            sensorValue.un.gyroscope.y,
            sensorValue.un.gyroscope.z
        );

        gyro_x_dps = g.x * RAD_TO_DEG;
        gyro_y_dps = g.y * RAD_TO_DEG;
        gyro_z_dps = g.z * RAD_TO_DEG;
    }
}

static int BNO08x_EnableGameRV(void)
{
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));

    config.reportInterval_us = 10000;
    config.batchInterval_us  = 0;
    config.sensorSpecific    = 0;
    config.changeSensitivity = 0;
    config.wakeupEnabled     = false;
    config.alwaysOnEnabled   = false;

    return sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &config);
}

static int BNO08x_EnableGyro(void)
{
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));

    config.reportInterval_us = 5000;
    config.batchInterval_us  = 0;
    config.sensorSpecific    = 0;
    config.changeSensitivity = 0;
    config.wakeupEnabled     = false;
    config.alwaysOnEnabled   = false;

    return sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BNO_INT_Pin)
    {
        bno_irq_flag = 1;
        return;
    }

    if (GPIO_Pin == CH1_Pin)
    {
        rc_handle_edge(RC_CH_ROLL_IDX, CH1_GPIO_Port, CH1_Pin);
    }
    else if (GPIO_Pin == CH2_Pin)
    {
        rc_handle_edge(RC_CH_PITCH_IDX, CH2_GPIO_Port, CH2_Pin);
    }
    else if (GPIO_Pin == CH3_Pin)
    {
        rc_handle_edge(RC_CH_THROTTLE_IDX, CH3_GPIO_Port, CH3_Pin);
    }
    else if (GPIO_Pin == CH4_Pin)
    {
        rc_handle_edge(RC_CH_YAW_IDX, CH4_GPIO_Port, CH4_Pin);
    }
    else if (GPIO_Pin == CH5_Pin)
    {
        rc_handle_edge(RC_CH_ARM_IDX, CH5_GPIO_Port, CH5_Pin);
    }
    else if (GPIO_Pin == CH6_Pin)
    {
        rc_handle_edge(RC_CH_MODE_IDX, CH6_GPIO_Port, CH6_Pin);
    }
}

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

static void eventHandler(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    (void)cookie;
    (void)pEvent;
}

static void I2C_BusRecovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    for (int i = 0; i < 10; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
    }
}

static void I2C_Scan(void)
{
    printf("I2C scan start\r\n");
    for (uint16_t addr = 1; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 50) == HAL_OK)
        {
            printf("Found device at 0x%02X\r\n", addr);
        }
    }
    printf("I2C scan done\r\n");
}

static void BMP280_CheckID(void)
{
    HAL_StatusTypeDef rc;
    uint8_t id = 0;

    rc = HAL_I2C_Mem_Read(&hi2c1, (0x76 << 1), 0xD0, I2C_MEMADD_SIZE_8BIT, &id, 1, 100);
    printf("BMP ID rc=%d id=0x%02X\r\n", rc, id);
}

static void run_startup_calibration(void)
{
    if (calibrated) return;

    update_attitude_filter_from_quaternion();

    roll_sum += roll_filt_deg;
    pitch_sum += pitch_filt_deg;
    yaw_sum += yaw_dbg_deg;

    gyro_x_sum += gyro_x_dps;
    gyro_y_sum += gyro_y_dps;
    gyro_z_sum += gyro_z_dps;

    if (bmp280_ok)
    {
        float temp_tmp, press_tmp;
        if (BMP280_ReadData(&temp_tmp, &press_tmp))
        {
            temperature_c = temp_tmp;
            pressure_pa = press_tmp;
            pressure_sum += press_tmp;
        }
    }
    printf("CAL=%d/%d\r\n", cal_count, CALIBRATION_SAMPLES);
    cal_count++;

    if (cal_count >= CALIBRATION_SAMPLES)
    {
        roll_offset_deg = roll_sum / CALIBRATION_SAMPLES;
        pitch_offset_deg = pitch_sum / CALIBRATION_SAMPLES;
        yaw_offset_deg = yaw_sum / CALIBRATION_SAMPLES;

        gyro_x_bias = gyro_x_sum / CALIBRATION_SAMPLES;
        gyro_y_bias = gyro_y_sum / CALIBRATION_SAMPLES;
        gyro_z_bias = gyro_z_sum / CALIBRATION_SAMPLES;

        if (bmp280_ok)
        {
            pressure_baseline_pa = pressure_sum / CALIBRATION_SAMPLES;
            altitude_m = 0.0f;
        }

        pid_reset(&pid_roll_angle);
        pid_reset(&pid_pitch_angle);
        pid_reset(&pid_roll_rate);
        pid_reset(&pid_pitch_rate);
        pid_reset(&pid_yaw_rate);
        pid_reset(&pid_altitude);

        calibrated = true;
        printf("CALIBRATION DONE\r\n");
    }
}

static void run_stabilization_loop(void)
{
    float roll_meas_deg;
    float pitch_meas_deg;
    float roll_rate_meas_dps;
    float pitch_rate_meas_dps;
    float yaw_rate_meas_dps;
    float roll_angle_error;
    float pitch_angle_error;
    float roll_rate_error;
    float pitch_rate_error;
    float yaw_rate_error;

    roll_meas_deg  = roll_filt_deg - roll_offset_deg;
    pitch_meas_deg = pitch_filt_deg - pitch_offset_deg;

    roll_rate_meas_dps  = gyro_x_dps - gyro_x_bias;
    pitch_rate_meas_dps = gyro_y_dps - gyro_y_bias;
    yaw_rate_meas_dps   = gyro_z_dps - gyro_z_bias;

    roll_angle_error  = target_roll_deg  - roll_meas_deg;
    pitch_angle_error = target_pitch_deg - pitch_meas_deg;

    roll_rate_target_dps  = pid_update(&pid_roll_angle, roll_angle_error, CONTROL_DT_S);
    pitch_rate_target_dps = pid_update(&pid_pitch_angle, pitch_angle_error, CONTROL_DT_S);

    roll_rate_error  = roll_rate_target_dps  - roll_rate_meas_dps;
    pitch_rate_error = pitch_rate_target_dps - pitch_rate_meas_dps;
    yaw_rate_error   = target_yaw_rate_dps   - yaw_rate_meas_dps;

    roll_control  = pid_update(&pid_roll_rate, roll_rate_error, CONTROL_DT_S);
    pitch_control = pid_update(&pid_pitch_rate, pitch_rate_error, CONTROL_DT_S);
    yaw_control   = pid_update(&pid_yaw_rate, yaw_rate_error, CONTROL_DT_S);

    if (flight_mode == MODE_HOVER)
    {
        run_hover_altitude_loop();
    }
    else
    {
        throttle_control = 0.0f;
    }
}
static void run_output_loop(void)
{
    const char *mode_str;

    if (flight_mode == MODE_STABILIZE)
        mode_str = "STAB";
    else
        mode_str = "HOVER";

    printf("MODE=%s ARM=%d RC=%d ALT=%.2f ALT_TGT=%.2f THR_BASE=%.1f THR_CTL=%.1f "
           "R=%.2f P=%.2f Y=%.2f "
           "GX_RAW=%.2f GY_RAW=%.2f GZ_RAW=%.2f "
           "GX=%.2f GY=%.2f GZ=%.2f "
           "CH1=%u CH2=%u CH3=%u CH4=%u CH5=%u CH6=%u "
           "M1=%.1f M2=%.1f M3=%.1f M4=%.1f\r\n",
           mode_str,
           motors_armed ? 1 : 0,
           rc_connected ? 1 : 0,
           altitude_filt_m,
           hover_target_altitude_m,
           base_throttle_us,
           throttle_control,
           roll_filt_deg - roll_offset_deg,
           pitch_filt_deg - pitch_offset_deg,
           yaw_dbg_deg - yaw_offset_deg,

           gyro_x_dps,
           gyro_y_dps,
           gyro_z_dps,

           gyro_x_dps - gyro_x_bias,
           gyro_y_dps - gyro_y_bias,
           gyro_z_dps - gyro_z_bias,

           rc_pulse_us[0], rc_pulse_us[1], rc_pulse_us[2],
           rc_pulse_us[3], rc_pulse_us[4], rc_pulse_us[5],
           motor1_us, motor2_us, motor3_us, motor4_us);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    int status;
    HAL_StatusTypeDef rc76, rc4a;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();

    I2C_BusRecovery();
    printf("Bus recovery done\r\n");

    MX_I2C1_Init();

    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM17_Init();

    HAL_TIM_Base_Start(&htim1);
    motors_pwm_start();
    motors_stop();

    HAL_Delay(1000);
    printf("BOOT OK\r\n");
    I2C_Scan();

    rc76 = HAL_I2C_IsDeviceReady(&hi2c1, (0x76 << 1), 2, 100);
    rc4a = HAL_I2C_IsDeviceReady(&hi2c1, (0x4A << 1), 2, 100);

    printf("BMP 0x76 ready rc=%d\r\n", rc76);
    printf("BNO 0x4A ready rc=%d\r\n", rc4a);

    if (rc76 == HAL_OK)
    {
        BMP280_CheckID();
        bmp280_ok = BMP280_InitSensor();
        printf("BMP init ok=%d id=0x%02X\r\n", bmp280_ok ? 1 : 0, bmp280.id);
    }

    if (rc4a == HAL_OK)
    {
        printf("Opening BNO...\r\n");

        status = sh2_open(&bno08x_hal, eventHandler, NULL);
        printf("sh2_open status = %d\r\n", status);

        if (status == SH2_OK)
        {
            status = sh2_setSensorCallback(sensorHandler, NULL);
            printf("callback status = %d\r\n", status);

            status = BNO08x_EnableGameRV();
            printf("GameRV status = %d\r\n", status);

            status = BNO08x_EnableGyro();
            printf("Gyro status = %d\r\n", status);
        }
    }

    printf("Starting control loop\r\n");

    while (1)
    {
        for(int i = 0; i< 45; i++){
        	sh2_service();
        }

        update_attitude_filter_from_quaternion();
        if (bmp280_ok)
                {
                    float temp_tmp, press_tmp;

                    if (BMP280_ReadData(&temp_tmp, &press_tmp))
                    {
                        bmp_read_count++;
                        temperature_c = temp_tmp;
                        pressure_pa = press_tmp;

                        if (calibrated)
                        {
                            altitude_m = BMP280_ComputeAltitude(pressure_pa, pressure_baseline_pa);
                            altitude_filt_m = lpf1_update(&altitude_lpf, altitude_m);
                        }
                    }
                }
        update_rc_link_status();



        if (!calibrated)
        {
            run_startup_calibration();
            motors_stop();
        }
        else
        {
            if (rc_connected)
            {
                update_targets_from_rc();
            }
            else
            {
                apply_rc_failsafe();
            }

            run_stabilization_loop();
            motor_mix_and_output();
        }

        run_output_loop();
        HAL_Delay(20);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1050;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1050;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 7;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 19999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1050;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : CH2_Pin CH6_Pin */
  GPIO_InitStruct.Pin = CH2_Pin|CH6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CH3_Pin */
  GPIO_InitStruct.Pin = CH3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CH3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CH1_Pin CH4_Pin */
  GPIO_InitStruct.Pin = CH1_Pin | CH4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CH5_Pin */
  GPIO_InitStruct.Pin = CH5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CH5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_INT_Pin */
  GPIO_InitStruct.Pin = BNO_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNO_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
