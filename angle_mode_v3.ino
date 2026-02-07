#include <Wire.h>
#include <Preferences.h>
#include "driver/mcpwm.h"

#define x 0
#define y 1
#define z 2

#define INTERNAL_LED_PIN 2
#define RC_CH_COUNT 6

// Addresses
#define IMU_ADDR 0x68
#define MAG_ADDR 0x2C

constexpr float loop_cycle = 0.002f;
constexpr uint16_t loop_cycle_us = 2000;
constexpr uint8_t pid_max_op = 400;

#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3
#define CH5 4
#define CH6 5

#define PPM_PIN 23

volatile uint16_t radio[RC_CH_COUNT] = { 1500, 1500, 1000, 1500, 1000, 1000 };
float radio_filtered[RC_CH_COUNT] = { 1500.0f, 1500.0f, 1000.0f, 1500.0f, 1000.0f, 1000.0f };
volatile uint8_t channel_index;
volatile int64_t ppm_last_rise_time;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define ESC1_PIN 19
#define ESC2_PIN 18
#define ESC3_PIN 26
#define ESC4_PIN 25

int esc1, esc2, esc3, esc4;

constexpr float accel_scale = 0.0002441;  // 1 / 4096 -> 7 digits
constexpr float gyro_scale = 0.0152671;   // 1 / 65.5 -> 7 digits

constexpr float p_roll_angle = 1.6f, p_pitch_angle = p_roll_angle, p_yaw_angle = 0.0f;

// P rate is stable at 0.65 - Do not change
// D rate is stable at 0.008
// I rate is stable at 0.08
constexpr float p_roll_rate = 0.65f, i_roll_rate = 0.08f, d_roll_rate = 0.008f;
constexpr float p_pitch_rate = p_roll_rate, i_pitch_rate = i_roll_rate, d_pitch_rate = d_roll_rate;

constexpr float p_yaw_rate = 1.5f, i_yaw_rate = 3.0f, d_yaw_rate = 0.0f;

int16_t rgx, rgy, rgz, rax, ray, raz, rmx, rmy, rmz, rvx, rvy, rvz;
float gx, gy, gz, ax, ay, az, mx, my, mz, vx, vy, vz;

// Tracking rates are necessary
float prev_gx, prev_gy, prev_gz, prev_vx, prev_vy, prev_vz;

float gyro_offsets[3], accel_offsets[3];
float mag_offsets[3], mag_scales[3];

float roll_angle, pitch_angle, yaw_angle;
float roll_angle_uncertainity = 4.0f, pitch_angle_uncertainity = 4.0f, yaw_angle_uncertainity = 4.0f;

float desired_roll_rate, desired_pitch_rate, desired_yaw_rate, desired_throttle;
float desired_roll_angle, desired_pitch_angle, desired_yaw_angle;

float desired_vx, desired_vy, desired_vz;

float adjusted_roll_rate, adjusted_pitch_rate, adjusted_yaw_rate, adjusted_throttle;

float error_roll_rate, error_pitch_rate, error_yaw_rate;
float error_roll_angle, error_pitch_angle, error_yaw_angle;

float prev_error_roll_rate, prev_error_pitch_rate, prev_error_yaw_rate;
float prev_error_roll_angle, prev_error_pitch_angle, prev_error_yaw_angle;

float prev_i_term_roll_rate, prev_i_term_pitch_rate, prev_i_term_yaw_rate;
float prev_i_term_roll_angle, prev_i_term_pitch_angle, prev_i_term_yaw_angle;

constexpr float mag_declination = -0.25f;

int64_t loop_time, mag_read_time, baro_read_time;

Preferences prefs;
// States
enum STATES { UNARMED,
              SAFETY_TRIP,
              ARMED };

STATES state = UNARMED;

enum FLIGHT_MODES { ANGLE_MODE,
                    ACRO_MODE };

FLIGHT_MODES mode = ACRO_MODE;

void IRAM_ATTR radio_ppm_isr() {
  int64_t current_time = esp_timer_get_time();
  uint16_t us = (uint16_t)(current_time - ppm_last_rise_time);
  ppm_last_rise_time = current_time;

  if (us > 3000) channel_index = 0;
  else if (channel_index < RC_CH_COUNT) radio[channel_index++] = us;
}

void read_radio() {
  portENTER_CRITICAL(&mux);
  float pulses[RC_CH_COUNT];
  for (int i = 0; i < RC_CH_COUNT; i++) pulses[i] = radio[i];
  portEXIT_CRITICAL(&mux);

  for (int i = 0; i < RC_CH_COUNT; i++) {
    float pulse = pulses[i];

    if (pulse > 2000.0f) pulse = 2000.0f;
    if (pulse < 1000.0f) pulse = 1000.0f;

    if (i != CH3 && pulse > 1490.0f && pulse < 1510.0f) pulse = 1500.0f;
    radio_filtered[i] = pulse;
  }
}

void initialize_mcpwm() {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, ESC1_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, ESC2_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, ESC3_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, ESC4_PIN);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = (uint16_t)(1 / loop_cycle);
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  // Timer 0 handles ESC1 & 2
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);  // Timer 1 handles ESC3 & 4
}

void kalman_1d(float* state, float* uncertainty, float rate, float angle) {
  *state += rate * loop_cycle;
  *uncertainty += loop_cycle * loop_cycle * 16.0f;  // gyro variance (4 deg/s)^2

  float gain = *uncertainty / (*uncertainty + 9.0f);  // accel variance (3 deg)^2
  *state += gain * (angle - *state);
  *uncertainty *= (1.0f - gain);
}

void read_orientation() {
  kalman_1d(&roll_angle, &roll_angle_uncertainity, gx, atan2f(ay, az) * RAD_TO_DEG);
  kalman_1d(&pitch_angle, &pitch_angle_uncertainity,
            gy,
            atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG);

  yaw_angle += gz * loop_cycle;

  float r_rad = roll_angle * DEG_TO_RAD;
  float p_rad = pitch_angle * DEG_TO_RAD;

  float cr = cosf(r_rad);
  float cp = cosf(p_rad);

  float sr = sinf(r_rad);
  float sp = sinf(p_rad);

  float mx_horizontal = mx * cp + my * sr * sp + mz * cr * sp;

  float my_horizontal = my * cr - mz * sr;

  float heading = atan2f(my_horizontal, mx_horizontal) * RAD_TO_DEG;

  if (heading > 360.0f) heading -= 360.0f;
  if (heading < 0.0f) heading += 360.0f;

  float error = heading - yaw_angle;

  if (error > 180.0f) error -= 360.0f;
  if (error < -180.0f) error += 360.0f;

  yaw_angle += 0.3 * error;
  yaw_angle += mag_declination;

  if (yaw_angle > 360.0f) yaw_angle -= 360.0f;
  if (yaw_angle < 0.0f) yaw_angle += 360.0f;
}

void toggle_mode() {
  if (roll_angle > 10.0f || roll_angle < -10.0f || pitch_angle > 10.0f || pitch_angle < -10.0f) return;

  else if (radio_filtered[CH6] < 1300) mode = ACRO_MODE;
  else if (radio_filtered[CH6] > 1700) mode = ANGLE_MODE;
}

void update_arm_state() {
  if (radio_filtered[CH5] < 1300) {
    state = UNARMED;
    return;
  }

  switch (state) {
    case UNARMED:
      // Check if sensors are actually healthy before allowing arm
      if ((radio_filtered[CH5] > 1700) && (radio_filtered[CH3] < 1200)) state = ARMED;
      break;

    case ARMED:
      // Safety kill triggers
      // If it immediately jumps to SAFETY_TRIP, one of these is true:
      if (roll_angle > 80.0f || roll_angle < -80.0f || pitch_angle > 80.0f || pitch_angle < -80.0f) state = SAFETY_TRIP;
      break;

    case SAFETY_TRIP:
      // Recovery: Switch must be toggled back to LOW to exit this state
      if (radio_filtered[CH5] < 1300) state = UNARMED;
      break;
  }
}

void translate_flight_mode() {
  switch (mode) {
    case ANGLE_MODE:

      desired_roll_angle =
        30.0f * ((radio_filtered[CH1] - 1500.0f) / 500.0f);  // 30°
      desired_pitch_angle =
        30.0f * ((radio_filtered[CH2] - 1500.0f) / 500.0f);  // 30°
      break;

    case ACRO_MODE:

      desired_roll_rate =
        75.0f * ((radio_filtered[CH1] - 1500.0f) / 500.0f);  // 75°/s
      desired_pitch_rate =
        75.0f * ((radio_filtered[CH2] - 1500.0f) / 500.0f);  // 75°/s
      break;
  }
  desired_yaw_rate =
    100.0f * ((radio_filtered[CH4] - 1500.0f) / 500.0f);  // 100°/s
  desired_throttle =
    radio_filtered[CH3];
}

void setup() {
  // Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(INTERNAL_LED_PIN, OUTPUT);
  pinMode(PPM_PIN, INPUT_PULLUP);

  digitalWrite(INTERNAL_LED_PIN, HIGH);

  initialize_mcpwm();
  delay(100);
  // Arm the ESCs
  for (int i = 0; i < 100; i++) {
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 1000);
    delay(20);
  }

  delay(2000);

  initialize_imu();
  initialize_mag();

  attachInterrupt(digitalPinToInterrupt(PPM_PIN), radio_ppm_isr, RISING);
  loop_time = esp_timer_get_time();
  digitalWrite(INTERNAL_LED_PIN, LOW);
}

void loop() {
  // Wait until exactly loop_time have passed since the start of the last loop
  while (esp_timer_get_time() - loop_time < loop_cycle_us)
    ;

  // Reset loop time
  loop_time = esp_timer_get_time();

  imu_signal();  // Read IMU

  read_radio();        // Get latest RC pulses
  update_arm_state();  // Safety and arming logic
  toggle_mode();       // Check flight mode switch

  translate_flight_mode();  // Convert RC to setpoints
  read_orientation();       // Kalman filter / AHRS

  if (state == ARMED) command_corrections();  // Run PID controllers
  else reset_controller();

  // Write to motors
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, esc1);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, esc2);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, esc3);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, esc4);

  if (esp_timer_get_time() - mag_read_time >= 5 * 1e3) {  // 200Hz
    mag_signal();
    mag_read_time += 5 * 1e3;
  }
  // Visual warning: If execution takes more than 90% of our budget (1800us)
  // we turn on the LED to indicate a CPU bottleneck.
  digitalWrite(INTERNAL_LED_PIN, esp_timer_get_time() - loop_time > loop_cycle_us * 0.9);
}