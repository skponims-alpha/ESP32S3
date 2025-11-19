# Sensor Calibration

- **MPU6050** —
- bool initMPU() {
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found!");
    return false;
  }
  Serial.println("✅ MPU6050 initialized.");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(1);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(false);
  mpu.setMotionInterrupt(true);
  return true;
}
void readMPU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float Ax = a.acceleration.x;
  float Ay = a.acceleration.y;
  float Az = a.acceleration.z;

  float Wx = g.gyro.x * (180.0 / PI);
  float Wy = g.gyro.y * (180.0 / PI);
  float Wz = g.gyro.z * (180.0 / PI);

  // Calculate Roll, Pitch, Yaw
  float roll  = atan2(Ay, Az) * 180.0 / PI;
  float pitch = atan(-Ax / sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;
  float yaw   = atan2(Wy, Wx) * 180.0 / PI;  // Approx yaw

  Serial.println("--------- MPU DATA ---------");
  Serial.printf("Ax: %.2f  Ay: %.2f  Az: %.2f (m/s²)\n", Ax, Ay, Az);
  Serial.printf("Wx: %.2f  Wy: %.2f  Wz: %.2f (°/s)\n", Wx, Wy, Wz);
  Serial.printf("Roll:  %.2f°\nPitch: %.2f°\nYaw:   %.2f°\n", roll, pitch, yaw);
  Serial.println("----------------------------");
}
- **BH1750** — Lux verification steps
- void initBH1750() {
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23)) {
    Serial.println("❌ BH1750 not found!");
    while (1);
  }
}
- **BMP180** — Pressure/altitude alignment
- 
- **Soil Moisture** — Raw ADC to percentage mapping
  
-**Soil Temerature**- Raw ADC to percentage mapping
  -**LoRa**- Packet sending at every Int from MPU
  
