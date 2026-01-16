void initialize_imu() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);  // Power on
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);  // DLF 44Hz
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);  // Accel config ±8g
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1B);  // Gyro config ±500°/s
  Wire.write(0x08);
  Wire.endTransmission();

  int samples = 2000;

  for (int i = 0; i < 3; i++) {
    gyro_offsets[i] = 0;
    accel_offsets[i] = 0;
  }

  for (int i = 0; i < samples; i++) {
    imu_signal();

    ax = (float)rax / 4096.0f;
    ay = (float)ray / 4096.0f;
    az = (float)raz / 4096.0f;

    gx = (float)rgx / 65.5f;
    gy = (float)rgy / 65.5f;
    gz = (float)rgz / 65.5f;

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
}

void imu_signal() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 14);

  int64_t t = esp_timer_get_time();

  while (Wire.available() < 14) {
    if (esp_timer_get_time() - t > I2C_TIMEOUT_US) {
      i2c_freeze_flag = 1;
      return;
    }
  }

  rax = (int16_t)(Wire.read() << 8 | Wire.read());
  ray = (int16_t)(Wire.read() << 8 | Wire.read());
  raz = (int16_t)(Wire.read() << 8 | Wire.read());

  Wire.read();
  Wire.read();

  rgx = (int16_t)(Wire.read() << 8 | Wire.read());
  rgy = (int16_t)(Wire.read() << 8 | Wire.read());
  rgz = (int16_t)(Wire.read() << 8 | Wire.read());
}
