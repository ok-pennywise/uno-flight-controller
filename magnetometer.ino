void initialize_mag() {
  // Wake up MPU6050
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // Wake up
  Wire.endTransmission();

  // Disable MPU I2C master
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6A);  // USER_CTRL
  Wire.write(0x00);  // I2C master OFF
  Wire.endTransmission();

  // Enable I2C bypass
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x37);  // INT_PIN_CFG
  Wire.write(0x02);  // BYPASS_EN
  Wire.endTransmission();

  // QMC6310 SIGN register (datasheet required)
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x29);
  Wire.write(0x06);
  Wire.endTransmission();

  // CTRL2: RNG=8G, Set/Reset ON
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0B);
  Wire.write(0x08);
  Wire.endTransmission();

  // CTRL1: Continuous mode, 200Hz
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.write(0xCF);
  Wire.endTransmission();
  delay(50);  // allow mag to start conversions

  prefs.begin("calibration", true);
  if (prefs.getBool("valid", false)) {
    prefs.getBytes("mag_offsets", mag_offsets, sizeof(mag_offsets));
    prefs.getBytes("mag_scales", mag_scales, sizeof(mag_scales));
  }
  prefs.end();
}


void mag_signal() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x01);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 6);

  int64_t t = esp_timer_get_time();

  while (Wire.available() < 6) {
    if (esp_timer_get_time() - t > I2C_TIMEOUT_US) {
      i2c_freeze_flag = 1;
      return;
    }
  }
  rmx = (int16_t)(Wire.read() | (Wire.read() << 8));
  rmy = (int16_t)(Wire.read() | (Wire.read() << 8));
  rmz = (int16_t)(Wire.read() | (Wire.read() << 8));
}