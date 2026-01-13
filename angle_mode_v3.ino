#include <Wire.h>
#include "driver/mcpwm.h"

#define x 0
#define y 1
#define z 2

#define INTERNAL_LED_PIN 2

#define RC_CH_COUNT 6

// Addresses
#define IMU_ADDR 0x68
#define MAG_ADDR 0x2C

// Times
#define LOOP_CYCLE 0.002f  // Flying fine at P rate = 0.7 and others set to 0 so if not flying good increase loop time
#define I2C_TIMEOUT_US 500
#define PID_MAX_OP 400

#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3
#define CH5 4
#define CH6 5

#define PPM_PIN 32

volatile uint16_t radio_channels[RC_CH_COUNT] = { 1500, 1500, 1000, 1500, 1000, 1500 };
float radio_filtered_channels[RC_CH_COUNT] = { 1500.0f, 1500.0f, 1000.0f, 1500.0f, 1000.0f, 1500.0f };
volatile uint8_t channel_index;
volatile int64_t ppm_last_rise_time;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define ESC1_PIN 25
#define ESC2_PIN 26
#define ESC3_PIN 27
#define ESC4_PIN 14

int esc1, esc2, esc3, esc4;

#define SCL_PIN 19
#define SDA_PIN 21

#define ACCEL_SCALE 0.000244f  // 1 / 4096
#define GYRO_SCALE 0.015267f   // 1 / 65.6

int16_t rgx, rgy, rgz, rax, ray, raz, rmx, rmy, rmz;
float gx, gy, gz, ax, ay, az, mx, my, mz;

float prev_gx, prev_gy, prev_gz;
float prev_ax, prev_ay, prev_az;
float prev_mx, prev_my, prev_mz;

float gyro_offsets[3], accel_offsets[3];
float mag_offsets[3], mag_scales[3];

float p_angle = 0.0f;

float p_roll_rate = 0.7f, i_roll_rate = 0.0f, d_roll_rate = 0.0f;
float p_pitch_rate = p_roll_rate, i_pitch_rate = i_roll_rate, d_pitch_rate = d_roll_rate;

float p_yaw_rate = 3.0f, i_yaw_rate = 0.02f, d_yaw_rate = 0.0f;

float roll_angle, pitch_angle;
float roll_angle_uncertainity = 4.0f;
float pitch_angle_uncertainity = 4.0f;

float desired_roll_rate, desired_pitch_rate, desired_yaw_rate, desired_throttle;
float desired_roll_angle, desired_pitch_angle;

float adjusted_roll_rate, adjusted_pitch_rate, adjusted_yaw_rate, adjusted_throttle;

float error_roll_rate, error_pitch_rate, error_yaw_rate;
float error_roll_angle, error_pitch_angle;

float prev_error_roll_rate, prev_error_pitch_rate, prev_error_yaw_rate;
float prev_error_roll_angle, prev_error_pitch_angle;

float prev_iterm_roll_rate, prev_iterm_pitch_rate, prev_iterm_yaw_rate;
float prev_iterm_roll_angle, prev_iterm_pitch_angle;

int64_t loop_time;

// States
enum STATES { UNARMED,
              SAFETY_TRIP,
              ARMED };

STATES state = UNARMED;

// Modes
enum FLIGHT_MODES { ANGLE_MODE,
                    ACRO_MODE,
                    POS_HOLD_MODE };

FLIGHT_MODES mode = ACRO_MODE;

uint8_t i2c_freeze_flag = 0;

void IRAM_ATTR radio_ppm_isr() {
  int64_t current_time = esp_timer_get_time();
  uint16_t us = (uint16_t)(current_time - ppm_last_rise_time);
  ppm_last_rise_time = current_time;

  if (us > 3000) {
    channel_index = 0;
  } else {
    if (channel_index < RC_CH_COUNT) {
      radio_channels[channel_index] = us;
      channel_index++;
    }
  }
}

void read_radio() {
  portENTER_CRITICAL(&mux);
  float pulses[RC_CH_COUNT];
  for (int i = 0; i < RC_CH_COUNT; i++) pulses[i] = radio_channels[i];
  portEXIT_CRITICAL(&mux);

  for (int i = 0; i < RC_CH_COUNT; i++) {
    float pulse = pulses[i];

    if (pulse > 2000.0f) pulse = 2000.0f;
    if (pulse < 1000.0f) pulse = 1000.0f;

    if (i != CH3 && pulse > 1490.0f && pulse < 1510.0f) pulse = 1500.0f;
    radio_filtered_channels[i] = pulse;
  }
}

void initialize_mcpwm() {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, ESC1_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, ESC2_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, ESC3_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, ESC4_PIN);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = (uint16_t)(1 / LOOP_CYCLE);
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  // Timer 0 handles ESC1 & 2
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);  // Timer 1 handles ESC3 & 4
}

void kalman_1d(float* state, float* uncertainty, float rate, float angle) {
  *state += rate * LOOP_CYCLE;
  *uncertainty += LOOP_CYCLE * LOOP_CYCLE * 16.0f;  // gyro variance (4 deg/s)^2

  float gain = *uncertainty / (*uncertainty + 9.0f);  // accel variance (3 deg)^2
  *state += gain * (angle - *state);
  *uncertainty *= (1.0f - gain);
}

void read_orientation() {
  ax = ((float)rax * ACCEL_SCALE) - accel_offsets[x];
  ay = ((float)ray * ACCEL_SCALE) - accel_offsets[y];
  az = ((float)raz * ACCEL_SCALE) - accel_offsets[z];

  gx = ((float)rgx * GYRO_SCALE) - gyro_offsets[x];
  gy = ((float)rgy * GYRO_SCALE) - gyro_offsets[y];
  gz = ((float)rgz * GYRO_SCALE) - gyro_offsets[z];

  // mx = ((float)rmx - mag_offsets[x]) * mag_scales[x];
  // my = ((float)rmy - mag_offsets[y]) * mag_scales[y];
  // mz = ((float)rmz - mag_offsets[z]) * mag_scales[z];

  kalman_1d(&roll_angle, &roll_angle_uncertainity, gx, atan2(ay, az) * RAD_TO_DEG);
  kalman_1d(&pitch_angle, &pitch_angle_uncertainity,
            gy,
            atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG);


  // // --- Tilt-compensated yaw (rotation matrix) ---
  // float roll_r = roll_angle * DEG_TO_RAD;
  // float pitch_r = pitch_angle * DEG_TO_RAD;

  // float cr = cos(roll_r);
  // float sr = sin(roll_r);
  // float cp = cos(pitch_r);
  // float sp = sin(pitch_r);

  // // Rotate magnetometer into horizontal plane
  // float mx_h = mx * cp + mz * sp;
  // float my_h = mx * sr * sp + my * cr - mz * sr * cp;

  // kalman_1d(&yaw_angle, &yaw_angle_uncertainity, gz, atan2(-my_h, mx_h) * RAD_TO_DEG);
}

void toggle_mode() {
  if (radio_filtered_channels[CH3] > 1200) return;

  if (radio_filtered_channels[CH6] > 1700) mode = POS_HOLD_MODE;
  else if (radio_filtered_channels[CH6] < 1300) mode = ACRO_MODE;
  else if (radio_filtered_channels[CH6] > 1300 && radio_filtered_channels[CH6] < 1700) mode = ANGLE_MODE;
}

void update_arm_state() {

  // HARD DISARM (always allowed)
  if (radio_filtered_channels[CH5] < 1300) {
    state = UNARMED;
    return;
  }

  switch (state) {
    case UNARMED:
      // Only allow arming if not safety-tripped
      if (radio_filtered_channels[CH5] > 1700 && radio_filtered_channels[CH3] < 1100) state = ARMED;
      break;

    case ARMED:
      // Safety kill
      if (fabs(roll_angle) > 80 || fabs(pitch_angle) > 80 || i2c_freeze_flag) state = SAFETY_TRIP;
      break;

    case SAFETY_TRIP:
      // Do nothing until disarmed
      // (forces pilot to reset switch)
      break;
  }
}

void translate_flight_mode() {
  switch (mode) {
    case ANGLE_MODE:

      desired_roll_angle =
        30.0f * ((radio_filtered_channels[CH1] - 1500.0f) / 500.0f);  // 30°
      desired_pitch_angle =
        30.0f * ((radio_filtered_channels[CH2] - 1500.0f) / 500.0f);  // 30°
      break;

    case ACRO_MODE:
    
      desired_roll_rate =
        75.0f * ((radio_filtered_channels[CH1] - 1500.0f) / 500.0f);  // 75°/s
      desired_pitch_rate =
        75.0f * ((radio_filtered_channels[CH2] - 1500.0f) / 500.0f);  // 75°/s
      break;

    default:
      break;
  }
  desired_yaw_rate =
    100.0f * ((radio_filtered_channels[CH4] - 1500.0f) / 500.0f);  // 100°/s
  desired_throttle =
    radio_filtered_channels[CH3];
}

void setup() {
  // Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  pinMode(INTERNAL_LED_PIN, OUTPUT);
  pinMode(PPM_PIN, INPUT_PULLUP);

  digitalWrite(INTERNAL_LED_PIN, HIGH);

  initialize_mcpwm();
  delay(100);

  for (int i = 0; i < 100; i++) {
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 1000);
    delay(20);
  }

  delay(2000);

  initialize_imu();

  attachInterrupt(digitalPinToInterrupt(PPM_PIN), radio_ppm_isr, RISING);

  loop_time = esp_timer_get_time();
  digitalWrite(INTERNAL_LED_PIN, LOW);
}

void loop() {
  while (esp_timer_get_time() - loop_time <= LOOP_CYCLE * 1e6)
    ;
  loop_time = esp_timer_get_time();

  imu_signal();

  read_radio();

  update_arm_state();
  toggle_mode();

  read_orientation();
  translate_flight_mode();
  apply_corrections();

  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, esc1);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, esc2);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, esc3);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, esc4);

  digitalWrite(INTERNAL_LED_PIN, (esp_timer_get_time() - loop_time > LOOP_CYCLE * 1e6));
}
