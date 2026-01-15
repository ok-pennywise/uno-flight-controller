/*
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
*/

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

  float rate_f = 0.7 * rate + 0.3 * (*prev_rate);
  float d_term = -d * (rate_f - *prev_rate) / LOOP_CYCLE;

  *output = p_term + i_term + d_term;

  *prev_error = error;
  *prev_i_term = i_term;
  *prev_rate = rate_f;

  if (*output > PID_MAX_OP) *output = PID_MAX_OP;
  if (*output < -PID_MAX_OP) *output = -PID_MAX_OP;
}

void command_corrections() {
  if (state != ARMED) {
    reset_controller();
    esc1 = esc2 = esc3 = esc4 = 1000;
    return;
  }

  adjusted_throttle = desired_throttle;

  if (adjusted_throttle > 1800) adjusted_throttle = 1800;

  if (adjusted_throttle < 1050) reset_controller();

  if (mode == ANGLE_MODE) {
    error_roll_angle = desired_roll_angle - roll_angle;
    error_pitch_angle = desired_pitch_angle - pitch_angle;

    compute_angle_correction(error_roll_angle, p_roll_angle, &desired_roll_rate);
    compute_angle_correction(error_pitch_angle, p_pitch_angle, &desired_pitch_rate);
  }

  error_roll_rate = desired_roll_rate - gx;
  error_pitch_rate = desired_pitch_rate - gy;
  error_yaw_rate = desired_yaw_rate - gz;

  compute_rate_correction(error_roll_rate, p_roll_rate, i_roll_rate, d_roll_rate, &prev_error_roll_rate, &prev_iterm_roll_rate, gx, &prev_gx, &adjusted_roll_rate);
  compute_rate_correction(error_pitch_rate, p_pitch_rate, i_pitch_rate, d_pitch_rate, &prev_error_pitch_rate, &prev_iterm_pitch_rate, gy, &prev_gy, &adjusted_pitch_rate);
  compute_rate_correction(error_yaw_rate, p_yaw_rate, i_yaw_rate, d_yaw_rate, &prev_error_yaw_rate, &prev_iterm_yaw_rate, gz, &prev_gz, &adjusted_yaw_rate);

  esc1 = adjusted_throttle - adjusted_roll_rate - adjusted_pitch_rate - adjusted_yaw_rate;
  esc2 = adjusted_throttle - adjusted_roll_rate + adjusted_pitch_rate + adjusted_yaw_rate;
  esc3 = adjusted_throttle + adjusted_roll_rate + adjusted_pitch_rate - adjusted_yaw_rate;
  esc4 = adjusted_throttle + adjusted_roll_rate - adjusted_pitch_rate + adjusted_yaw_rate;

  if (esc1 < 1100) esc1 = 1100;
  if (esc2 < 1100) esc2 = 1100;
  if (esc3 < 1100) esc3 = 1100;
  if (esc4 < 1100) esc4 = 1100;

  if (esc1 > 2000) esc1 = 2000;
  if (esc2 > 2000) esc2 = 2000;
  if (esc3 > 2000) esc3 = 2000;
  if (esc4 > 2000) esc4 = 2000;
}


void reset_controller() {
  adjusted_roll_rate = adjusted_pitch_rate = adjusted_yaw_rate = adjusted_throttle = 0;
  
  prev_error_roll_rate = prev_error_pitch_rate = prev_error_yaw_rate = 0;
  prev_iterm_roll_rate = prev_iterm_pitch_rate = prev_iterm_yaw_rate = 0;

  prev_error_roll_angle = prev_error_pitch_angle = prev_error_yaw_angle = 0;
  prev_iterm_roll_angle = prev_iterm_pitch_angle = prev_iterm_yaw_angle = 0;

  desired_roll_angle = desired_pitch_angle = desired_yaw_angle = 0;
  desired_roll_rate = desired_pitch_rate = desired_yaw_rate = 0;

  prev_gx = prev_gy = prev_gz = 0;
  prev_ax = prev_ay = prev_az = 0;
  prev_mx = prev_my = prev_mz = 0;
  prev_vx = prev_vy = prev_vz = 0;
}