#include <Wire.h>
#include <Adafruit_BMP280.h>   // Install via Library Manager

// ===== Existing MPU9250 direct-register code (I2C0: SDA=21, SCL=22) =====

// MPU9250 (MPU6500) I2C address (change to 0x69 if AD0 pin is HIGH)
#define MPU_ADDR        0x68

// AK8963 (magnetometer) I2C address (behind MPU's master)
#define AK8963_ADDR     0x0C

// MPU-6500 register map
#define PWR_MGMT_1      0x6B
#define SMPLRT_DIV      0x19
#define CONFIG_REG      0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D
#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define ACCEL_XOUT_H    0x3B
#define TEMP_OUT_H      0x41
#define GYRO_XOUT_H     0x43
#define USER_CTRL       0x6A
#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27
#define I2C_MST_STATUS  0x36
#define I2C_SLV0_DO     0x63
#define WHO_AM_I_MPU    0x75

// AK8963 registers
#define AK8963_WHO_AM_I 0x00
#define AK8963_ST1      0x02
#define AK8963_HXL      0x03
#define AK8963_CNTL1    0x0A
#define AK8963_ASAX     0x10

// Scale factors
float accelScale = 2.0f / 32768.0f;     // g/LSB for ±2g
float gyroScale  = 250.0f / 32768.0f;   // dps/LSB for ±250 dps
float magAdj[3]  = {1.0f, 1.0f, 1.0f};  // ASA adjustment from AK8963 fuse ROM
float magScale   = 4912.0f / 32760.0f;  // µT/LSB for 16-bit, 0x16 mode (approx)

// ESP32 I2C0 pins for MPU9250
#define I2C_SDA 21
#define I2C_SCL 22

// Helpers
void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, (int)count, (int)true);
  for (int i = 0; i < count && Wire.available(); i++) {
    dest[i] = Wire.read();
  }
}

int16_t read16(uint8_t addr, uint8_t regHigh) {
  uint8_t buf[2];
  i2cReadBytes(addr, regHigh, 2, buf);
  return (int16_t)((buf[0] << 8) | buf[1]);
}

void ak8963WriteReg(uint8_t reg, uint8_t val) {
  i2cWriteByte(MPU_ADDR, I2C_SLV0_ADDR, AK8963_ADDR); // write (bit7=0)
  i2cWriteByte(MPU_ADDR, I2C_SLV0_REG, reg);
  i2cWriteByte(MPU_ADDR, I2C_SLV0_DO, val);
  i2cWriteByte(MPU_ADDR, I2C_SLV0_CTRL, 0x81); // EN=1, LEN=1
  delay(10);
}

void mpuSetupI2CMaster() {
  i2cWriteByte(MPU_ADDR, USER_CTRL, 0x20); // I2C_MST_EN
  i2cWriteByte(MPU_ADDR, I2C_MST_CTRL, 0x0D); // ~400kHz
  delay(10);
}

void ak8963Init() {
  ak8963WriteReg(AK8963_CNTL1, 0x00);
  delay(20);
  ak8963WriteReg(AK8963_CNTL1, 0x0F);
  delay(20);

  // Read ASA (3 bytes) via SLV0 -> EXT_SENS_DATA_00 (0x49)
  i2cWriteByte(MPU_ADDR, I2C_SLV0_ADDR, 0x80 | AK8963_ADDR); // read
  i2cWriteByte(MPU_ADDR, I2C_SLV0_REG, AK8963_ASAX);
  i2cWriteByte(MPU_ADDR, I2C_SLV0_CTRL, 0x83); // LEN=3
  delay(20);

  uint8_t ext[3] = {0};
  i2cReadBytes(MPU_ADDR, 0x49, 3, ext);
  for (int i = 0; i < 3; i++) {
    magAdj[i] = ((float)ext[i] - 128.0f) / 256.0f + 1.0f;
  }

  ak8963WriteReg(AK8963_CNTL1, 0x00);
  delay(20);
  ak8963WriteReg(AK8963_CNTL1, 0x16); // 16-bit, 100Hz, cont. 2
  delay(20);

  // Continuous read ST1+XYZ+ST2 (7 bytes)
  i2cWriteByte(MPU_ADDR, I2C_SLV0_ADDR, 0x80 | AK8963_ADDR);
  i2cWriteByte(MPU_ADDR, I2C_SLV0_REG, AK8963_HXL);
  i2cWriteByte(MPU_ADDR, I2C_SLV0_CTRL, 0x87);
}

bool mpuInit() {
  i2cWriteByte(MPU_ADDR, PWR_MGMT_1, 0x01); // PLL
  delay(100);
  i2cWriteByte(MPU_ADDR, CONFIG_REG, 0x03);    // DLPF=3
  i2cWriteByte(MPU_ADDR, SMPLRT_DIV, 9);       // 100Hz
  i2cWriteByte(MPU_ADDR, GYRO_CONFIG, 0x00);   // ±250dps
  i2cWriteByte(MPU_ADDR, ACCEL_CONFIG, 0x00);  // ±2g
  i2cWriteByte(MPU_ADDR, ACCEL_CONFIG2, 0x03);
  mpuSetupI2CMaster();
  ak8963Init();

  uint8_t who = 0;
  i2cReadBytes(MPU_ADDR, WHO_AM_I_MPU, 1, &who);
  Serial.print("MPU WHO_AM_I: 0x"); Serial.println(who, HEX);
  return true;
}

void readAccelGyro(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& tempC) {
  uint8_t buf[14];
  i2cReadBytes(MPU_ADDR, ACCEL_XOUT_H, 14, buf);
  int16_t axraw = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ayraw = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t azraw = (int16_t)((buf[4] << 8) | buf[5]);
  int16_t traw  = (int16_t)((buf[6] << 8) | buf[7]);
  int16_t gxraw = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t gyraw = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t gzraw = (int16_t)((buf[12] << 8) | buf[13]);

  ax = axraw * accelScale;
  ay = ayraw * accelScale;
  az = azraw * accelScale;

  gx = gxraw * gyroScale;
  gy = gyraw * gyroScale;
  gz = gzraw * gyroScale;

  // Temp in C: (raw/333.87)+21
  tempC = ((float)traw) / 333.87f + 21.0f;
}

bool readMag(float& mx, float& my, float& mz) {
  uint8_t ext[8] = {0};
  i2cReadBytes(MPU_ADDR, 0x49, 8, ext);
  if (!(ext[0] & 0x01)) return false; // ST1 DRDY

  int16_t mxraw = (int16_t)((ext[2] << 8) | ext[1]);
  int16_t myraw = (int16_t)((ext[4] << 8) | ext[3]);
  int16_t mzraw = (int16_t)((ext[6] << 8) | ext[5]);

  mx = (float)mxraw * magAdj[0] * magScale;
  my = (float)myraw * magAdj[1] * magScale;
  mz = (float)mzraw * magAdj[2] * magScale;
  return true;
}

// ===== Add BMP280 on second I2C bus (I2C1: SDA=32, SCL=33) =====

// Wire1 pins
static const int I2C1_SDA = 32;
static const int I2C1_SCL = 33;

// Create second I2C bus
TwoWire I2C_1 = TwoWire(1);

// Bind BMP280 to Wire1
Adafruit_BMP280 bmp(&I2C_1);

// Addresses
#define BMP280_I2C_ADDR 0x76  // fallback 0x77

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("ESP32: MPU9250@Wire(21/22) + BMP280@Wire1(32/33)"));

  // Init I2C0 for MPU9250
  Wire.begin(I2C_SDA, I2C_SCL, 400000); // stable high-speed
  delay(100);

  // Quick MPU address probe
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU9250 not found at 0x68. If AD0=HIGH, set MPU_ADDR=0x69.");
  }

  // Init MPU9250
  if (!mpuInit()) {
    Serial.println("MPU init failed.");
    while (1) { delay(1000); }
  }
  Serial.println("MPU9250 initialized.");

  // Init I2C1 for BMP280
  I2C_1.begin(I2C1_SDA, I2C1_SCL, 400000);
  delay(50);

  // Initialize BMP280
  if (!bmp.begin(BMP280_I2C_ADDR)) {
    Serial.println(F("BMP280 not found at 0x76 on Wire1, trying 0x77..."));
    if (!bmp.begin(0x77)) {
      Serial.println(F("ERROR: BMP280 not found on Wire1. Check wiring/address."));
      while (1) delay(100);
    }
  }

  // Configure BMP280 sampling
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,    // Temp oversampling
    Adafruit_BMP280::SAMPLING_X16,   // Pressure oversampling
    Adafruit_BMP280::FILTER_X16,     // IIR filter
    Adafruit_BMP280::STANDBY_MS_500
  );
  Serial.println(F("BMP280 initialized on Wire1."));
}

void loop() {
  // Read MPU9250 accel/gyro/temp
  float ax, ay, az, gx, gy, gz, tempC;
  readAccelGyro(ax, ay, az, gx, gy, gz, tempC);

  // Read MPU9250 mag
  float mx, my, mz;
  bool magOk = readMag(mx, my, mz);

  // Read BMP280
  float temperatureC = bmp.readTemperature();
  float pressurePa   = bmp.readPressure();
  float pressurehPa  = pressurePa / 100.0f;
  float altitudeM    = bmp.readAltitude(1013.25); // adjust for local sea-level pressure

  // Print BMP280
  Serial.println(F("----- BMP280 -----"));
  Serial.print(F("Temperature: ")); Serial.print(temperatureC, 2); Serial.println(F(" °C"));
  Serial.print(F("Pressure:    ")); Serial.print(pressurehPa, 2); Serial.println(F(" hPa"));
  Serial.print(F("Altitude:    ")); Serial.print(altitudeM, 2);   Serial.println(F(" m"));

  // Print MPU9250
  Serial.println(F("----- MPU9250 (Accel / Gyro / Mag) -----"));
  Serial.print(F("Accel (g): "));
  Serial.print(ax, 3); Serial.print(F(", "));
  Serial.print(ay, 3); Serial.print(F(", "));
  Serial.println(az, 3);

  Serial.print(F("Gyro (deg/s): "));
  Serial.print(gx, 1); Serial.print(F(", "));
  Serial.print(gy, 1); Serial.print(F(", "));
  Serial.println(gz, 1);

  Serial.print(F("Temp (IMU, C): "));
  Serial.println(tempC, 2);

  if (magOk) {
    Serial.print(F("Mag (uT): "));
    Serial.print(mx, 1); Serial.print(F(", "));
    Serial.print(my, 1); Serial.print(F(", "));
    Serial.println(mz, 1);
  } else {
    Serial.println(F("Mag: not ready"));
  }

  Serial.println();
  delay(100); // ~10Hz print
}
