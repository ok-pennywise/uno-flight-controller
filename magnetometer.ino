void initialize_mag() {
  // Enable I2C Bypass on MPU-series IMUs
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x37);  // INT Pin / Bypass Enable Configuration register
  Wire.write(0x02);  // Bit 1 high enables bypass
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  // B11001111
  Wire.write(0xCF);  // OSR2 = OSR1 = 8, ODR = 200Hz, Mode = Continuous
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0B);
  // B00001000
  Wire.write(0x08);  // SOFT_RST = Off, SELF_TEST = Off, RNG = 8Guass, SET/RESET MODE = On
  Wire.endTransmission();
}

void mag_signals() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x09);
  Wire.endTransmission();

  Wire.requestFrom(MAG_ADDR, 1);
  if (!(Wire.read() & 0x01)) return;  // Exit if DRDY bit is not 1

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x01);
  Wire.endTransmission();

  Wire.requestFrom(MAG_ADDR, 6);

  rmx = Wire.read() | Wire.read() << 8;
  rmy = Wire.read() | Wire.read() << 8;
  rmz = Wire.read() | Wire.read() << 8;
}