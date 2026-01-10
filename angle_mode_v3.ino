#include <Wire.h>
#include "driver/mcpwm.h"

#define x 0
#define y 1
#define z 2

#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3
#define CH5 4
#define CH6 5

// States
#define UNARMED 0
#define READY_TO_ARM 1
#define ARMED 2

// Modes
#define ANGLE_MODE 0
#define ACRO_MODE 1

// Pins
#define ESC1_PIN 25
#define ESC2_PIN 26
#define ESC3_PIN 27
#define ESC4_PIN 14
#define PPM_PIN 32
#define SCL_PIN 19
#define SDA_PIN 21
#define INTERNAL_LED_PIN 2

#define RC_CH_COUNT 6

// Addresses
#define IMU_ADDR 0x68
#define BARO_ADDR 0x77
#define LOX_ADDR 0x29
#define MAG_ADDR 0x2C

// Times
#define LOOP_CYCLE 0.004f
#define I2C_TIMEOUT_US 800
#define PID_MAX_OP 400

// Scales
#define ACCEL_SCALE 0.000244f  // 1 / 4096
#define GYRO_SCALE 0.015267f   // 1 / 65.6

volatile uint16_t radio_channels[RC_CH_COUNT] = { 1500, 1500, 1000, 1500, 1000, 1500 };
float radio_filtered_channels[RC_CH_COUNT] = { 1500.0f, 1500.0f, 1000.0f, 1500.0f, 1000.0f, 1500.0f };
volatile uint8_t channel_index;
volatile int64_t ppm_last_rise_time;

int esc1, esc2, esc3, esc4;

int16_t rgx, rgy, rgz, rax, ray, raz;
float gx, gy, gz, ax, ay, az;

float prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az;

float gyro_offsets[3], accel_offsets[3];
float mag_offsets[3], mag_scales[3];

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

float p_angle = 0.0f;

float p_roll_rate = 0.7f, i_roll_rate = 0.0f, d_roll_rate = 0.0f;
float p_pitch_rate = p_roll_rate, i_pitch_rate = i_roll_rate, d_pitch_rate = d_roll_rate;

float p_yaw_rate = 3.0f, i_yaw_rate = 0.02f, d_yaw_rate = 0.0f;

/*
------------------------------------------------
STEP 0 — BEFORE YOU START
------------------------------------------------
- Props ON
- Angle mode OFF (use ACRO / rate mode)
- Set ALL angle PIDs to 0
- Set Rate I = 0
- Set Rate D = 0

Only Rate P should be non-zero.

------------------------------------------------
STEP 1 — RATE P (how strong the correction is)
------------------------------------------------
Goal: The quad follows stick commands without wobbling.

Start value:
  p_roll_rate = 0.30
  p_pitch_rate = 0.30

Procedure:
1. Hover at low throttle.
2. Increase P by SMALL steps:
     +0.05 each time

Stop increasing when:
- You see FAST shaking / buzzing (high frequency)

Then:
- Reduce P by 20%

Example:
  Oscillates at 0.50
  Final P = 0.40

------------------------------------------------
STEP 2 — RATE D (stops overshoot)
------------------------------------------------
Goal: Stop the quad from bouncing back after a quick stick move.

Keep P FIXED from Step 1.

Start value:
  d_roll_rate = 0.000
  d_pitch_rate = 0.000

Increase D in SMALL steps:
  +0.002 each time

Example:
  0.000 → 0.002 → 0.004 → 0.006

Test:
- Quickly move stick and release.
- If quad overshoots and comes back → increase D.

Stop increasing when:
- Motors get warm quickly
- Or buzzing noise appears

Then:
- Reduce D by ONE step.

------------------------------------------------
STEP 3 — RATE I (stops slow drift)
------------------------------------------------
Goal: Quad holds attitude without slowly drifting.

Keep P and D FIXED.

Start value:
  i_roll_rate = 0.00
  i_pitch_rate = 0.00

Increase I in SMALL steps:
  +0.01 each time

Example:
  0.00 → 0.01 → 0.02 → 0.03

Stop increasing when:
- Drift stops

If you see:
- Slow left-right wobble → I is TOO HIGH

Then:
- Reduce I by 0.01

------------------------------------------------
STEP 4 — VERIFY RATE LOOP
------------------------------------------------
At this point:
- ACRO mode should feel stable
- No fast shaking
- No slow wobble
- No motor overheating

If NOT stable:
- LOWER gains
- Do NOT raise multiple values

------------------------------------------------
STEP 5 — ANGLE MODE (only after RATE is perfect)
------------------------------------------------
Start values:
  p_angle = 0.05

Increase p_angle by:
  +0.02 steps

Example:
  0.05 → 0.07 → 0.09 → 0.11

Stop when:
- Quad levels quickly
- No oscillation when releasing stick

------------------------------------------------
IMPORTANT RULES
------------------------------------------------
- Change ONLY ONE value at a time
- Test hover for 10–15 seconds
- If unstable → LOWER values
- Large props = lower I and D
- Motor heat = D too high
*/

int64_t loop_time;

uint8_t state = UNARMED;
bool safety_trip = false;
uint8_t mode = ACRO_MODE;

uint8_t i2c_freeze_flag = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

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
  pwm_config.frequency = 500;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  // Timer 0 handles ESC1 & 2
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);  // Timer 1 handles ESC3 & 4
}

void initialize_imu() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);  // Power on
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);  // DLF
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  int samples = 2000;

  for (int i = 0; i < 3; i++) {
    gyro_offsets[i] = 0;
    accel_offsets[i] = 0;
  }

  for (int i = 0; i < samples; i++) {
    imu_signal();

    ax = (float)rax * ACCEL_SCALE;
    ay = (float)ray * ACCEL_SCALE;
    az = (float)raz * ACCEL_SCALE;

    gx = (float)rgx * GYRO_SCALE;
    gy = (float)rgy * GYRO_SCALE;
    gz = (float)rgz * GYRO_SCALE;

    gyro_offsets[x] += gx;
    gyro_offsets[y] += gy;
    gyro_offsets[z] += gz;

    accel_offsets[x] += ax;
    accel_offsets[y] += ay;
    accel_offsets[z] += (az - 1);
    delay(1);
  }

  for (int i = 0; i < 3; i++) {
    gyro_offsets[i] /= samples;
    accel_offsets[i] /= samples;
  }

  roll_angle = pitch_angle = 0;
}

void imu_signal() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDR, 14);

  int64_t t = esp_timer_get_time();

  while (Wire.available() < 14) {
    if (esp_timer_get_time() - t > I2C_TIMEOUT_US) {
      i2c_freeze_flag = 1;
      return;
    }
  }

  rax = Wire.read() << 8 | Wire.read();
  ray = Wire.read() << 8 | Wire.read();
  raz = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read();

  rgx = Wire.read() << 8 | Wire.read();
  rgy = Wire.read() << 8 | Wire.read();
  rgz = Wire.read() << 8 | Wire.read();
}

void reset_controller() {
  adjusted_roll_rate = adjusted_pitch_rate = adjusted_yaw_rate = adjusted_throttle = 0;
  prev_error_roll_rate = prev_error_pitch_rate = prev_error_yaw_rate = 0;
  prev_iterm_roll_rate = prev_iterm_pitch_rate = prev_iterm_yaw_rate = 0;

  prev_error_roll_angle = prev_error_pitch_angle = 0;
  prev_iterm_roll_angle = prev_iterm_pitch_angle = 0;

  desired_roll_angle = desired_pitch_angle = 0;
  desired_roll_rate = desired_pitch_rate = desired_yaw_rate = 0;

  prev_gx = prev_gy = prev_gz = 0;
}

void kalman_1d(float* state, float* uncertainty, float rate, float angle_meas) {
  *state += rate * LOOP_CYCLE;
  *uncertainty += LOOP_CYCLE * LOOP_CYCLE * 16.0f;  // gyro variance (4 deg/s)^2

  float gain = *uncertainty / (*uncertainty + 9.0f);  // accel variance (3 deg)^2
  *state += gain * (angle_meas - *state);
  *uncertainty *= (1.0f - gain);
}

void read_orientation() {
  ax = ((float)rax * ACCEL_SCALE) - accel_offsets[x];
  ay = ((float)ray * ACCEL_SCALE) - accel_offsets[y];
  az = ((float)raz * ACCEL_SCALE) - accel_offsets[z];

  gx = ((float)rgx * GYRO_SCALE) - gyro_offsets[x];
  gy = ((float)rgy * GYRO_SCALE) - gyro_offsets[y];
  gz = ((float)rgz * GYRO_SCALE) - gyro_offsets[z];

  // float d_theta = gz * LOOP_CYCLE * DEG_TO_RAD;
  // float s = sin(d_theta);
  // float c = cos(d_theta);

  // float prev_roll_angle = roll_angle;
  // float prev_pitch_angle = pitch_angle;

  // roll_angle = c * prev_roll_angle - s * prev_pitch_angle;
  // pitch_angle = s * prev_roll_angle + c * prev_pitch_angle;

  kalman_1d(&roll_angle, &roll_angle_uncertainity, gx, (atan2(ay, az) * RAD_TO_DEG));
  kalman_1d(&pitch_angle, &pitch_angle_uncertainity,
            gy,
            atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG);
}

void compute_angle_correction(float error, float p, float* output) {
  *output = p * error;

  if (*output > PID_MAX_OP) *output = PID_MAX_OP;
  if (*output < -PID_MAX_OP) *output = -PID_MAX_OP;
}

void compute_rate_correction(float error, float p, float i, float d, float* prev_error, float* prev_i_term, float rate, float* prev_rate, float* output) {
  float p_term = p * error;

  float i_term = *prev_i_term + i * (error + *prev_error) * LOOP_CYCLE / 2.0f;

  if (i_term > PID_MAX_OP) i_term = PID_MAX_OP;
  if (i_term < -PID_MAX_OP) i_term = -PID_MAX_OP;

  rate = 0.7 * rate + 0.3 * (*prev_rate);
  float d_term = -d * (rate - *prev_rate) / LOOP_CYCLE;

  *output = p_term + i_term + d_term;

  *prev_error = error;
  *prev_i_term = i_term;
  *prev_rate = rate;

  if (*output > PID_MAX_OP) *output = PID_MAX_OP;
  if (*output < -PID_MAX_OP) *output = -PID_MAX_OP;
}

void apply_corrections() {
  if (state != ARMED) {
    reset_controller();
    esc1 = esc2 = esc3 = esc4 = 1000;
    return;
  }

  if (mode == ANGLE_MODE) {
    desired_roll_angle = 30.0f * ((radio_filtered_channels[CH1] - 1500.0f) / 500.0f);
    desired_pitch_angle = 30.0f * ((radio_filtered_channels[CH2] - 1500.0f) / 500.0f);

    error_roll_angle = desired_roll_angle - roll_angle;
    error_pitch_angle = desired_pitch_angle - pitch_angle;

    compute_angle_correction(error_roll_angle, p_angle, &desired_roll_rate);
    compute_angle_correction(error_pitch_angle, p_angle, &desired_pitch_rate);
  } else {
    desired_roll_rate = 75.0f * ((radio_filtered_channels[CH1] - 1500.0f) / 500.0f);
    desired_pitch_rate = 75.0f * ((radio_filtered_channels[CH2] - 1500.0f) / 500.0f);
  }

  desired_yaw_rate = 100.0f * ((radio_filtered_channels[CH4] - 1500.0f) / 500.0f);
  desired_throttle = radio_filtered_channels[CH3];

  adjusted_throttle = desired_throttle;

  if (adjusted_throttle > 1800) adjusted_throttle = 1800;

  if (adjusted_throttle < 1050) {
    reset_controller();
  }

  error_roll_rate = desired_roll_rate - gx;
  error_pitch_rate = desired_pitch_rate - gy;
  error_yaw_rate = -(desired_yaw_rate - gz);

  compute_rate_correction(error_roll_rate, p_roll_rate, i_roll_rate, d_roll_rate, &prev_error_roll_rate, &prev_iterm_roll_rate, gx, &prev_gx, &adjusted_roll_rate);
  compute_rate_correction(error_pitch_rate, p_pitch_rate, i_pitch_rate, d_pitch_rate, &prev_error_pitch_rate, &prev_iterm_pitch_rate, gy, &prev_gy, &adjusted_pitch_rate);
  compute_rate_correction(error_yaw_rate, p_yaw_rate, i_yaw_rate, d_yaw_rate, &prev_error_yaw_rate, &prev_iterm_yaw_rate, gz, &prev_gz, &adjusted_yaw_rate);

  esc1 = adjusted_throttle - adjusted_roll_rate - adjusted_pitch_rate - adjusted_yaw_rate;
  esc2 = adjusted_throttle - adjusted_roll_rate + adjusted_pitch_rate + adjusted_yaw_rate;
  esc3 = adjusted_throttle + adjusted_roll_rate + adjusted_pitch_rate - adjusted_yaw_rate;
  esc4 = adjusted_throttle + adjusted_roll_rate - adjusted_pitch_rate + adjusted_yaw_rate;

  if (esc1 > 2000) esc1 = 2000;
  if (esc2 > 2000) esc2 = 2000;
  if (esc3 > 2000) esc3 = 2000;
  if (esc4 > 2000) esc4 = 2000;

  if (esc1 < 1100) esc1 = 1100;
  if (esc2 < 1100) esc2 = 1100;
  if (esc3 < 1100) esc3 = 1100;
  if (esc4 < 1100) esc4 = 1100;
}

void update_arm_state() {
  if (radio_filtered_channels[CH5] < 1300) {
    state = UNARMED;
    safety_trip = false;
    i2c_freeze_flag = 0;
  } else if (radio_filtered_channels[CH5] > 1700) {
    if (state == UNARMED && !safety_trip) {
      if (radio_filtered_channels[CH3] < 1100) state = ARMED;
    }
  }
  if (state == ARMED) {
    if (fabs(roll_angle) > 80 || fabs(pitch_angle) > 80 || i2c_freeze_flag == 1) {
      state = UNARMED;
      safety_trip = true;
    }
  }
}

void toggle_mode() {
  mode = radio_filtered_channels[CH6] < 1500 ? ACRO_MODE : ANGLE_MODE;
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
  apply_corrections();

  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, esc1);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, esc2);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, esc3);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, esc4);
}
