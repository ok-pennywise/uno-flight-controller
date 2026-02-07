void initialize_imu() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
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

    gyro_offsets[x] += (float)rgx * gyro_scale;
    gyro_offsets[y] += (float)rgy * gyro_scale;
    gyro_offsets[z] += (float)rgz * gyro_scale;

    accel_offsets[x] += (float)rax * accel_scale;
    accel_offsets[y] += (float)ray * accel_scale;
    accel_offsets[z] += ((float)raz * accel_scale - 1);

    delay(1);
  }

  for (int i = 0; i < 3; i++) {
    gyro_offsets[i] /= samples;
    accel_offsets[i] /= samples;
  }
}

void imu_signal() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(IMU_ADDR, 14);

  while (Wire.available() < 14)
    ;

  rax = (int16_t)(Wire.read() << 8 | Wire.read());
  ray = (int16_t)(Wire.read() << 8 | Wire.read());
  raz = (int16_t)(Wire.read() << 8 | Wire.read());

  Wire.read();
  Wire.read();

  rgx = (int16_t)(Wire.read() << 8 | Wire.read());
  rgy = (int16_t)(Wire.read() << 8 | Wire.read());
  rgz = (int16_t)(Wire.read() << 8 | Wire.read());

  ax = ((float)rax * accel_scale) - accel_offsets[x];
  ay = ((float)ray * accel_scale) - accel_offsets[y];
  az = ((float)raz * accel_scale) - accel_offsets[z];

  gx = ((float)rgx * gyro_scale) - gyro_offsets[x];
  gy = ((float)rgy * gyro_scale) - gyro_offsets[y];
  gz = ((float)rgz * gyro_scale) - gyro_offsets[z];
}