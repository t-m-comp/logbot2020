// based on https://github.com/kriswiner/BMX-055/blob/master/BMX055_MS5637_BasicAHRS_t3.ino
// BMX055 data sheet http://ae-bst.resource.bosch.com/media/products/dokumente/bmx055/BST-BMX055-DS000-01v2.pdf
// The BMX055 is a conglomeration of three separate motion sensors packaged together but
// addressed and communicated with separately by design
// Accelerometer registers

#define BMX055_ACC_WHOAMI        0x00   // should return 0xFA
//#define BMX055_ACC_Reserved    0x01
#define BMX055_ACC_D_X_LSB       0x02
#define BMX055_ACC_D_X_MSB       0x03
#define BMX055_ACC_D_Y_LSB       0x04
#define BMX055_ACC_D_Y_MSB       0x05
#define BMX055_ACC_D_Z_LSB       0x06
#define BMX055_ACC_D_Z_MSB       0x07
#define BMX055_ACC_D_TEMP        0x08
#define BMX055_ACC_INT_STATUS_0  0x09
#define BMX055_ACC_INT_STATUS_1  0x0A
#define BMX055_ACC_INT_STATUS_2  0x0B
#define BMX055_ACC_INT_STATUS_3  0x0C
//#define BMX055_ACC_Reserved    0x0D
#define BMX055_ACC_FIFO_STATUS   0x0E
#define BMX055_ACC_PMU_RANGE     0x0F
#define BMX055_ACC_PMU_BW        0x10
#define BMX055_ACC_PMU_LPW       0x11
#define BMX055_ACC_PMU_LOW_POWER 0x12
#define BMX055_ACC_D_HBW         0x13
#define BMX055_ACC_BGW_SOFTRESET 0x14
//#define BMX055_ACC_Reserved    0x15
#define BMX055_ACC_INT_EN_0      0x16
#define BMX055_ACC_INT_EN_1      0x17
#define BMX055_ACC_INT_EN_2      0x18
#define BMX055_ACC_INT_MAP_0     0x19
#define BMX055_ACC_INT_MAP_1     0x1A
#define BMX055_ACC_INT_MAP_2     0x1B
//#define BMX055_ACC_Reserved    0x1C
//#define BMX055_ACC_Reserved    0x1D
#define BMX055_ACC_INT_SRC       0x1E
//#define BMX055_ACC_Reserved    0x1F
#define BMX055_ACC_INT_OUT_CTRL  0x20
#define BMX055_ACC_INT_RST_LATCH 0x21
#define BMX055_ACC_INT_0         0x22
#define BMX055_ACC_INT_1         0x23
#define BMX055_ACC_INT_2         0x24
#define BMX055_ACC_INT_3         0x25
#define BMX055_ACC_INT_4         0x26
#define BMX055_ACC_INT_5         0x27
#define BMX055_ACC_INT_6         0x28
#define BMX055_ACC_INT_7         0x29
#define BMX055_ACC_INT_8         0x2A
#define BMX055_ACC_INT_9         0x2B
#define BMX055_ACC_INT_A         0x2C
#define BMX055_ACC_INT_B         0x2D
#define BMX055_ACC_INT_C         0x2E
#define BMX055_ACC_INT_D         0x2F
#define BMX055_ACC_FIFO_CONFIG_0 0x30
//#define BMX055_ACC_Reserved    0x31
#define BMX055_ACC_PMU_SELF_TEST 0x32
#define BMX055_ACC_TRIM_NVM_CTRL 0x33
#define BMX055_ACC_BGW_SPI3_WDT  0x34
//#define BMX055_ACC_Reserved    0x35
#define BMX055_ACC_OFC_CTRL      0x36
#define BMX055_ACC_OFC_SETTING   0x37
#define BMX055_ACC_OFC_OFFSET_X  0x38
#define BMX055_ACC_OFC_OFFSET_Y  0x39
#define BMX055_ACC_OFC_OFFSET_Z  0x3A
#define BMX055_ACC_TRIM_GPO      0x3B
#define BMX055_ACC_TRIM_GP1      0x3C
//#define BMX055_ACC_Reserved    0x3D
#define BMX055_ACC_FIFO_CONFIG_1 0x3E
#define BMX055_ACC_FIFO_DATA     0x3F

// BMX055 Gyroscope Registers
#define BMX055_GYRO_WHOAMI           0x00  // should return 0x0F
//#define BMX055_GYRO_Reserved       0x01
#define BMX055_GYRO_RATE_X_LSB       0x02
#define BMX055_GYRO_RATE_X_MSB       0x03
#define BMX055_GYRO_RATE_Y_LSB       0x04
#define BMX055_GYRO_RATE_Y_MSB       0x05
#define BMX055_GYRO_RATE_Z_LSB       0x06
#define BMX055_GYRO_RATE_Z_MSB       0x07
//#define BMX055_GYRO_Reserved       0x08
#define BMX055_GYRO_INT_STATUS_0  0x09
#define BMX055_GYRO_INT_STATUS_1  0x0A
#define BMX055_GYRO_INT_STATUS_2  0x0B
#define BMX055_GYRO_INT_STATUS_3  0x0C
//#define BMX055_GYRO_Reserved    0x0D
#define BMX055_GYRO_FIFO_STATUS   0x0E
#define BMX055_GYRO_RANGE         0x0F
#define BMX055_GYRO_BW            0x10
#define BMX055_GYRO_LPM1          0x11
#define BMX055_GYRO_LPM2          0x12
#define BMX055_GYRO_RATE_HBW      0x13
#define BMX055_GYRO_BGW_SOFTRESET 0x14
#define BMX055_GYRO_INT_EN_0      0x15
#define BMX055_GYRO_INT_EN_1      0x16
#define BMX055_GYRO_INT_MAP_0     0x17
#define BMX055_GYRO_INT_MAP_1     0x18
#define BMX055_GYRO_INT_MAP_2     0x19
#define BMX055_GYRO_INT_SRC_1     0x1A
#define BMX055_GYRO_INT_SRC_2     0x1B
#define BMX055_GYRO_INT_SRC_3     0x1C
//#define BMX055_GYRO_Reserved    0x1D
#define BMX055_GYRO_FIFO_EN       0x1E
//#define BMX055_GYRO_Reserved    0x1F
//#define BMX055_GYRO_Reserved    0x20
#define BMX055_GYRO_INT_RST_LATCH 0x21
#define BMX055_GYRO_HIGH_TH_X     0x22
#define BMX055_GYRO_HIGH_DUR_X    0x23
#define BMX055_GYRO_HIGH_TH_Y     0x24
#define BMX055_GYRO_HIGH_DUR_Y    0x25
#define BMX055_GYRO_HIGH_TH_Z     0x26
#define BMX055_GYRO_HIGH_DUR_Z    0x27
//#define BMX055_GYRO_Reserved    0x28
//#define BMX055_GYRO_Reserved    0x29
//#define BMX055_GYRO_Reserved    0x2A
#define BMX055_GYRO_SOC           0x31
#define BMX055_GYRO_A_FOC         0x32
#define BMX055_GYRO_TRIM_NVM_CTRL 0x33
#define BMX055_GYRO_BGW_SPI3_WDT  0x34
//#define BMX055_GYRO_Reserved    0x35
#define BMX055_GYRO_OFC1          0x36
#define BMX055_GYRO_OFC2          0x37
#define BMX055_GYRO_OFC3          0x38
#define BMX055_GYRO_OFC4          0x39
#define BMX055_GYRO_TRIM_GP0      0x3A
#define BMX055_GYRO_TRIM_GP1      0x3B
#define BMX055_GYRO_BIST          0x3C
#define BMX055_GYRO_FIFO_CONFIG_0 0x3D
#define BMX055_GYRO_FIFO_CONFIG_1 0x3E
#define BMX055_GYRO_FIFO_DATA     0x3F

// BMX055 magnetometer registers
#define BMX055_MAG_WHOAMI         0x40  // should return 0x32
#define BMX055_MAG_Reserved       0x41
#define BMX055_MAG_XOUT_LSB       0x42
#define BMX055_MAG_XOUT_MSB       0x43
#define BMX055_MAG_YOUT_LSB       0x44
#define BMX055_MAG_YOUT_MSB       0x45
#define BMX055_MAG_ZOUT_LSB       0x46
#define BMX055_MAG_ZOUT_MSB       0x47
#define BMX055_MAG_ROUT_LSB       0x48
#define BMX055_MAG_ROUT_MSB       0x49
#define BMX055_MAG_INT_STATUS     0x4A
#define BMX055_MAG_PWR_CNTL1      0x4B
#define BMX055_MAG_PWR_CNTL2      0x4C
#define BMX055_MAG_INT_EN_1       0x4D
#define BMX055_MAG_INT_EN_2       0x4E
#define BMX055_MAG_LOW_THS        0x4F
#define BMX055_MAG_HIGH_THS       0x50
#define BMX055_MAG_REP_XY         0x51
#define BMX055_MAG_REP_Z          0x52
/* Trim Extended Registers */
#define BMM050_DIG_X1             0x5D // needed for magnetic field calculation
#define BMM050_DIG_Y1             0x5E
#define BMM050_DIG_Z4_LSB         0x62
#define BMM050_DIG_Z4_MSB         0x63
#define BMM050_DIG_X2             0x64
#define BMM050_DIG_Y2             0x65
#define BMM050_DIG_Z2_LSB         0x68
#define BMM050_DIG_Z2_MSB         0x69
#define BMM050_DIG_Z1_LSB         0x6A
#define BMM050_DIG_Z1_MSB         0x6B
#define BMM050_DIG_XYZ1_LSB       0x6C
#define BMM050_DIG_XYZ1_MSB       0x6D
#define BMM050_DIG_Z3_LSB         0x6E
#define BMM050_DIG_Z3_MSB         0x6F
#define BMM050_DIG_XY2            0x70
#define BMM050_DIG_XY1            0x71

// Set initial input parameters
// define X055 ACC full scale options
//#define AFS_2G  0x03
//#define AFS_4G  0x05
#define AFS_8G  0x08
//#define AFS_16G 0x0C

//#define ACC_BW_8Hz 0      // 7.81 Hz,  64 ms update time
//#define ACC_BW_16Hz 1     // 15.63 Hz, 32 ms update time
#define ACC_BW_31Hz 2     // 31.25 Hz, 16 ms update time
//#define ACC_BW_63Hz 3     // 62.5  Hz,  8 ms update time
//#define ACC_BW_125Hz 4    // 125   Hz,  4 ms update time
//#define ACC_BW_250Hz 5    // 250   Hz,  2 ms update time
//#define ACC_BW_500Hz 6    // 500   Hz,  1 ms update time
//#define ACC_BW_100Hz 7    // 1000  Hz,  0.5 ms update time

//enum ACC_BW_SETTING {    // define BMX055 accelerometer bandwidths
//  ACC_BW_8Hz = 0,      // 7.81 Hz,  64 ms update time
//  ACC_BW_16Hz,     // 15.63 Hz, 32 ms update time
//  ACC_BW_31Hz,     // 31.25 Hz, 16 ms update time
//  ACC_BW_63Hz,     // 62.5  Hz,  8 ms update time
//  ACC_BW_125Hz,    // 125   Hz,  4 ms update time
//  ACC_BW_250Hz,    // 250   Hz,  2 ms update time
//  ACC_BW_500Hz,    // 500   Hz,  1 ms update time
//  ACC_BW_100Hz     // 1000  Hz,  0.5 ms update time
//};

//#define GFS_2000DPS 0
//#define GFS_1000DPS 1
#define GFS_500DPS 2
//#define GFS_250DPS 3
//#define GFS_125DPS 4

//enum GYRO_SCALE_OPTIONS {
//  GFS_2000DPS = 0,
//  GFS_1000DPS,
//  GFS_500DPS,
//  GFS_250DPS,
//  GFS_125DPS
//};

//#define G_2000Hz523Hz 0 // 2000 Hz ODR and unfiltered (bandwidth 523Hz)
//#define G_2000Hz230Hz 1
//#define G_1000Hz116Hz 2
#define G_400Hz47Hz 3
//#define G_200Hz23Hz 4
//#define G_100Hz12Hz 5
//#define G_200Hz64Hz 6
//#define G_100Hz32Hz 7 // 100 Hz ODR and 32 Hz bandwidth


//enum GYRO_ODR_BW_OPTIONS {
//  G_2000Hz523Hz = 0, // 2000 Hz ODR and unfiltered (bandwidth 523Hz)
//  G_2000Hz230Hz,
//  G_1000Hz116Hz,
//  G_400Hz47Hz,
//  G_200Hz23Hz,
//  G_100Hz12Hz,
//  G_200Hz64Hz,
//  G_100Hz32Hz  // 100 Hz ODR and 32 Hz bandwidth
//};


//#define MAG_ODR_10Hz 0   // 10 Hz ODR
#define MAG_ODR_2Hz 1   // 2 Hz ODR
//#define MAG_ODR_6Hz 2   // 6 Hz ODR
//#define MAG_ODR_8Hz 3   // 8 Hz ODR
//#define MAG_ODR_15Hz 4   // 15 Hz ODR
//#define MAG_ODR_20Hz 5   // 20 Hz ODR
//#define MAG_ODR_25Hz 6   // 25 Hz ODR
//#define MAG_ODR_30Hz 7   // 30 Hz ODR


//enum MAG_ODR_OPTIONS {
//  MAG_ODR_10Hz = 0,   // 10 Hz ODR
//  MAG_ODR_2Hz     ,   // 2 Hz ODR
//  MAG_ODR_6Hz     ,   // 6 Hz ODR
//  MAG_ODR_8Hz     ,   // 8 Hz ODR
//  MAG_ODR_15Hz    ,   // 15 Hz ODR
//  MAG_ODR_20Hz    ,   // 20 Hz ODR
//  MAG_ODR_25Hz    ,   // 25 Hz ODR
//  MAG_ODR_30Hz        // 30 Hz ODR
//};

#define LOW_POWER 0   // rms noise ~1.0 microTesla, 0.17 mA power
//#define REGULAR 1   // rms noise ~0.6 microTesla, 0.5 mA power
//#define ENHANCED_REGULAR 2   // rms noise ~0.5 microTesla, 0.8 mA power
//#define HIGH_ACCURACY 3       // rms noise ~0.3 microTesla, 4.9 mA power

//enum MAG_MODE_OPTIONS {
//  LOW_POWER         = 0,   // rms noise ~1.0 microTesla, 0.17 mA power
//  REGULAR             ,   // rms noise ~0.6 microTesla, 0.5 mA power
//  ENHANCED_REGULAR     ,   // rms noise ~0.5 microTesla, 0.8 mA power
//  HIGH_ACCURACY            // rms noise ~0.3 microTesla, 4.9 mA power
//};

// Specify sensor full scale
static const uint8_t GYRO_SCALE = GFS_500DPS;       // set gyro full scale
static const uint8_t GYRO_ODR_BW = G_400Hz47Hz;      // set gyro ODR and bandwidth
static const uint8_t ACC_SCALE = AFS_8G;           // set accel full scale
static const uint8_t ACC_BW  = 0x08 | ACC_BW_31Hz;  // Choose bandwidth for accelerometer, bit 3 always set to '1'
static const uint8_t MAG_MODE  = LOW_POWER;          // Choose magnetometer operation mode
static const uint8_t MAG_ODR   = MAG_ODR_2Hz;        // set magnetometer data rate

// Parameters to hold BMX055 trim values
signed char   dig_x1;
signed char   dig_y1;
signed char   dig_x2;
signed char   dig_y2;
uint16_t      dig_z1;
int16_t       dig_z2;
int16_t       dig_z3;
int16_t       dig_z4;
unsigned char dig_xy1;
signed char   dig_xy2;
uint16_t      dig_xyz1;

static const float MAG_RES = 1./1.6;

#ifdef GFS_125DPS
static const float GYRO_RES = 124.87/32768.0; // per data sheet, not exactly 125!?
#endif
#ifdef GFS_250DPS
static const float GYRO_RES = 249.75/32768.0;
#endif
#ifdef GFS_500DPS
static const float GYRO_RES = 499.5/32768.0;
#endif
#ifdef GFS_1000DPS
static const float GYRO_RES = 999.0/32768.0;
#endif
#ifdef GFS_2000DPS
static const float GYRO_RES = 1998.0/32768.0;
#endif

#ifdef AFS_2G
static const float ACC_RES = 2.0/2048.0;
#endif
#ifdef AFS_4G
static const float ACC_RES = 4.0/2048.0;
#endif
#ifdef AFS_8G
static const float ACC_RES = 8.0/2048.0;
#endif
#ifdef AFS_16G
static const float ACC_RES = 16.0/2048.0;
#endif

void sleep_BMX055()
{
  IMU_pwr_on = false;

  I2C_write_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_PMU_LPW, 0x20); //set to deep suspend (init required to restart)
  I2C_write_byte(IMU_BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM1, 0x20); //set to deep suspend (init required to restart)
  I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x00);  // set to suspend mode
}

void wake_BMX055()
{
  IMU_pwr_on = true;

  set_up_BMX055();
}

void set_up_ACC()
{
  I2C_write_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_PMU_RANGE, ACC_SCALE & 0x0F); // Set accelerometer full range
  I2C_write_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_PMU_BW, ACC_BW & 0x0F);     // Set accelerometer bandwidth
  I2C_write_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_D_HBW, 0x00);              // Use filtered data

  //set low power mode 2: (ACC 0x11) lowpower_en bit set to 1 with (ACC 0x12) lowpower_mode bit set to 1
  //set EST: (ACC 0x12) sleeptimer_en bit set to 1
  I2C_write_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_PMU_LPW, 0x50); //set low power mode 2 and sleep duration 4 ms #this value is valid for BW of 31 or 62
  I2C_write_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_PMU_LOW_POWER, 0x60);

  //FIFO stream mode 31 frames acc, overwrites when full
  //set (ACC 0x3E) bits 7:6 to '10'
  I2C_write_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_FIFO_CONFIG_1, 0x80); //stream mode
//  I2C_write_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_FIFO_CONFIG_1, 0x40); //blocking mode
//  I2C_write_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_FIFO_CONFIG_1, 0x20 | 0x1F); //set FIFO watermark
}

void set_up_GYRO()
{
  //set sleep duration (GYR 0x11) sleep_dur bits 3:1 to '111' (20 ms, which is the maximum)
  //this setup gives us 40 Hz sampled into FIFO
  I2C_write_byte(IMU_BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM1, 0x0E);

  I2C_write_byte(IMU_BMX055_GYRO_ADDRESS, BMX055_GYRO_RANGE, GYRO_SCALE);  // set GYRO FS range
  I2C_write_byte(IMU_BMX055_GYRO_ADDRESS, BMX055_GYRO_BW, GYRO_ODR_BW);     // set GYRO ODR and Bandwidth

  //set power save mode (GYR 0x12) power_save_mode bit to 1
  //I2C_write_byte(IMU_BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM2, 0x42); //set wake duration (GYR 0x12) auto_sleep_dur bits 2:0 to '010' (5 ms, which is the minimum for 47Hz)
  //I2C_write_byte(IMU_BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM2, 0x43); // 8 ms
  I2C_write_byte(IMU_BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM2, 0x44); // 10 ms

  //FIFO stream mode 40 frames GYRO (99 max), overwrites when full
  //TODO: need to fully read out GYRO FIFO each second (may be larger than 40 some times
  //set (GYR 0x3E) 7:6 bits to '10'
  I2C_write_byte(IMU_BMX055_GYRO_ADDRESS, BMX055_GYRO_FIFO_CONFIG_1, 0x80);
}

void set_up_MAG()
{
  I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // Wake up magnetometer

  // forced mode allows us to sample mag on demand at 1 Hz
  // use OpMode = '01' to switch to forced mode, will switch back to sleep after a measurement
  I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MAG_ODR << 3 | 0x02); // forced mode - set before sampling each second

#ifdef LOW_POWER
   // Low-power
   I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x01);  // 3 repetitions (oversampling)
   I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x02);  // 3 repetitions (oversampling)
#endif
#ifdef REGULAR
    // Regular
    I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x04);  //  9 repetitions (oversampling)
    I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x16);  // 15 repetitions (oversampling)
#endif
#ifdef ENHANCED_REGULAR
    // Enhanced Regular
    I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x07);  // 15 repetitions (oversampling)
    I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x22);  // 27 repetitions (oversampling)
#endif
#ifdef HIGH_ACCURACY
    // High Accuracy
    I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x17);  // 47 repetitions (oversampling)
    I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x51);  // 83 repetitions (oversampling)
#endif

  trim_BMX055();
  I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MAG_ODR << 3 | 0x06); //sleep mode
  //should switch to forced mode before sampling each second
  //I2C_write_byte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MAG_ODR << 3 | 0x02); // forced mode - set before sampling each second
}

void trim_BMX055()  // get trim values for magnetometer sensitivity
{
  uint8_t raw_data[2];  //placeholder for 2-byte trim data
  dig_x1 = I2C_read_byte(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_X1);
  dig_x2 = I2C_read_byte(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_X2);
  dig_y1 = I2C_read_byte(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_Y1);
  dig_y2 = I2C_read_byte(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_Y2);
  dig_xy1 = I2C_read_byte(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_XY1);
  dig_xy2 = I2C_read_byte(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_XY2);
  I2C_read(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_Z1_LSB, 2, raw_data);
  dig_z1 = (uint16_t) (((uint16_t)raw_data[1] << 8) | raw_data[0]);
  I2C_read(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_Z2_LSB, 2, raw_data);
  dig_z2 = (int16_t) (((int16_t)raw_data[1] << 8) | raw_data[0]);
  I2C_read(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_Z3_LSB, 2, raw_data);
  dig_z3 = (int16_t) (((int16_t)raw_data[1] << 8) | raw_data[0]);
  I2C_read(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_Z4_LSB, 2, raw_data);
  dig_z4 = (int16_t) (((int16_t)raw_data[1] << 8) | raw_data[0]);
  I2C_read(IMU_BMX055_MAG_ADDRESS, BMM050_DIG_XYZ1_LSB, 2, raw_data);
  dig_xyz1 = (uint16_t) (((uint16_t)raw_data[1] << 8) | raw_data[0]);
}


void set_up_BMX055()
{
  IMU_pwr_on = true;

  // start with all sensors in default mode with all registers reset
  I2C_write_byte(IMU_BMX055_ACC_ADDRESS,  BMX055_ACC_BGW_SOFTRESET, 0xB6);  // reset accelerometer
  I2C_write_byte(IMU_BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SOFTRESET, 0xB6); // reset gyro
  I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x82);  // Softreset magnetometer, ends up in sleep mode
  delay(10); // Wait for all registers to reset

  // Configure accelerometer
  set_up_ACC();

  set_up_GYRO();

  set_up_MAG();
}

void read_BMX055_other(float mag_data[], float &temperature)
{
  int16_t temp_buffer = read_bmx055_acc_temperature();  // Read the gyro adc values
  temperature = ((float) temp_buffer) / 2.0 + 23.0; // Gyro chip temperature in degrees Centigrade

  read_mag_data(mag_data);  // Read the x/y/z adc values
}

void write_BMX055_FIFO_to_SD()
{
  I2C_write_byte(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MAG_ODR << 3 | 0x02); // forced mode - set before sampling each second

  write_ACC_FIFO_to_SD();

  write_gyro_FIFO_to_SD();
}

void clear_acc_buffer()
{
  I2C_write_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_FIFO_CONFIG_1, 0x80);
}

void clear_gyro_buffer()
{
  I2C_write_byte(IMU_BMX055_GYRO_ADDRESS, BMX055_GYRO_FIFO_CONFIG_1, 0x80);
}

void write_gyro_FIFO_to_SD()
{
  float gyro_data[3] = {0, 0, 0};
  if (SD_pwr_on)
  {
    if(logFile)
    {
      logFile.write((uint8_t *)gyro_data, sizeof(gyro_data));
    }
  }
}

void write_ACC_FIFO_to_SD()
{
  float acc_data[3] = {0, 0, 0};
  uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};
  int16_t acc_temp[3] = {0, 0, 0};
  uint8_t samples = 0;

  samples = I2C_read_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_FIFO_STATUS) & 0x7F; // Read number of stored samples

  ACC_FIFO_FILLED = samples >= ACC_BUFFER_SIZE ? true : false;

  uint8_t mag_buffer_index = 0;

  for (uint8_t i = 0; i < ACC_BUFFER_SIZE; i++) // Read the acc data stored in the FIFO
  {
    acc_data[0] = 0;
    acc_data[1] = 0;
    acc_data[2] = 0;

    float x = 0;
    float y = 0;
    float z = 0;

    if (i < samples)
    {
      I2C_read(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_FIFO_DATA, 6, raw_data);

      acc_temp[0] = (int16_t) (((int16_t)raw_data[1] << 8) | raw_data[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value
      acc_temp[1] = (int16_t) (((int16_t)raw_data[3] << 8) | raw_data[2]) >> 4;
      acc_temp[2] = (int16_t) (((int16_t)raw_data[5] << 8) | raw_data[4]) >> 4;

      acc_data[0] = (float)acc_temp[0]*ACC_RES; // + accelBias[0];  // get actual g value, this depends on scale being set
      acc_data[1] = (float)acc_temp[1]*ACC_RES; // + accelBias[1];
      acc_data[2] = (float)acc_temp[2]*ACC_RES; // + accelBias[2];


      x = acc_data[0];
      y = acc_data[1];
      z = acc_data[2];
    }

    mag_buffer[mag_buffer_index++] = sqrt(x*x + y*y + z*z);

    if (SD_pwr_on)
    {
      if(logFile)
      {
        logFile.write((uint8_t *)acc_data, sizeof(acc_data));
      }
    }
  }
}

void read_mag_data(float mag_data[])
{
  int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
  uint16_t data_r = 0;
  uint8_t raw_data[8];  // x/y/z hall magnetic field data, and Hall resistance data
  I2C_read(IMU_BMX055_MAG_ADDRESS, BMX055_MAG_XOUT_LSB, 8, raw_data);  // Read the eight raw data registers sequentially into data array

  if(raw_data[6] & 0x01) // Check if data ready status bit is set
  {
    mdata_x = (int16_t) (((int16_t)raw_data[1] << 8) | raw_data[0]) >> 3;  // 13-bit signed integer for x-axis field
    mdata_y = (int16_t) (((int16_t)raw_data[3] << 8) | raw_data[2]) >> 3;  // 13-bit signed integer for y-axis field
    mdata_z = (int16_t) (((int16_t)raw_data[5] << 8) | raw_data[4]) >> 1;  // 15-bit signed integer for z-axis field
    data_r = (uint16_t) (((uint16_t)raw_data[7] << 8) | raw_data[6]) >> 2;  // 14-bit unsigned integer for Hall resistance
  }

  // Calculate the magnetometer values in milliGauss
  // Temperature-compensated magnetic field is in 16 LSB/microTesla
  mag_data[0] = (float)mdata_x*MAG_RES;  // get actual magnetometer value, this depends on scale being set
  mag_data[1] = (float)mdata_y*MAG_RES;
  mag_data[2] = (float)mdata_z*MAG_RES;
}

int8_t read_bmx055_acc_temperature()
{
  uint8_t c =  I2C_read_byte(IMU_BMX055_ACC_ADDRESS, BMX055_ACC_D_TEMP);  // Read the raw data register
  return ((int16_t)((int16_t)c << 8)) >> 8 ;  // Turn the byte into a signed 8-bit integer
}
