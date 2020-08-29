#define BH1721FVC_ADRESS     0x23  // light sensor address
#define BH1721FVC_DVI_pin    5  //light sensor enable pin: DVL pin

#define CONT_AUTO_RES 0x10 //up to 1 lx resolution - up to 180 ms - seems to work fine without any delay statement, only needed <60 msec in tests

void start_ILLUM()
{
  //照度センサの電源ON
  I2C_write_byte(BH1721FVC_ADRESS, 0x01);
  delay(16);

  //Auto-resolutionモード
  I2C_write_byte(BH1721FVC_ADRESS, CONT_AUTO_RES);
  delay(100);

  ILLUM_pwr_on = true;
}

void stop_ILLUM()
{
  I2C_write_byte(BH1721FVC_ADRESS, 0x00);

  ILLUM_pwr_on = false;
}

void read_ILLUM(uint16_t &illuminance)
{
  uint8_t buf[2];

  I2C_read(BH1721FVC_ADRESS, 2, buf);

  illuminance = ((uint16_t)buf[0] << 8) | buf[1];  // reconstruct 16 bit data

  stop_ILLUM();
}
