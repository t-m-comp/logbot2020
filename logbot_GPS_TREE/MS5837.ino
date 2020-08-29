/* Blue Robotics MS5837 Library Example
-----------------------------------------------------

Title: Blue Robotics MS5837 Library Example
Description: This example demonstrates the MS5837 Library with a connected
sensor. The example reads the sensor and prints the resulting values
to the serial terminal.
The code is designed for the Arduino Uno board and can be compiled and
uploaded via the Arduino 1.0+ software.
-------------------------------
The MIT License (MIT)
Copyright (c) 2015 Blue Robotics Inc.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/

//#include<SD.h>
#define MS5837_ADDR               0x76
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A  //(according to datasheet, this requires 20msec for conversion)
#define MS5837_CONVERT_D2_8192    0x5A
#define MS5837_CONVERT_D1_256    0x40  //10ms
#define MS5837_CONVERT_D2_256    0x50


void setup_wp()
{

  uint16_t C[8];

  // Reset the MS5837, per datasheet
  reset_water();

  // Read calibration values and CRC
  for ( uint8_t i = 0 ; i < 7 ; i++ )
  {
    Wire.beginTransmission(MS5837_ADDR);
    Wire.write(MS5837_PROM_READ+i*2);
    Wire.endTransmission();

    Wire.requestFrom(MS5837_ADDR,2);
    C[i] = ((uint16_t)Wire.read() << 8) | Wire.read();
  }

  write_C(C, sizeof(C));
}


void read_water_D1D2(uint32_t &D1, uint32_t &D2)
{

  // Request D1 conversion
  read_water(D1, 1);

  // Request D2 conversion
  read_water(D2, 2);

}

void read_water(uint32_t &D, const uint8_t &type)
{
  // Request D1 conversion
  Wire.beginTransmission(MS5837_ADDR);
  if (type == 1)
  {
    Wire.write(MS5837_CONVERT_D1_8192);
  }
  else
  {
    Wire.write(MS5837_CONVERT_D2_8192);
  }
  Wire.endTransmission();

  delay(20); // Max conversion time per datasheet

  Wire.beginTransmission(MS5837_ADDR);
  Wire.write(MS5837_ADC_READ);
  Wire.endTransmission();

  Wire.requestFrom(MS5837_ADDR, 3);
  D = 0;
  D = Wire.read();
  D = (D << 8) | Wire.read();
  D = (D << 8) | Wire.read();
}

//Based on datasheet
bool crc_check(uint16_t n_prom[])
{
  uint8_t crc = (uint8_t)((n_prom[0] & 0xF000) >> 12);
  uint16_t n_rem = 0; // crc remainder

  uint16_t temp1 = n_prom[0];
  uint16_t temp2 = n_prom[7];

  n_prom[0]=((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
  n_prom[7]=0; // Subsidiary value, set to 0

  for (uint8_t cnt=0; cnt < 16; cnt++) // operation is performed on bytes
  { // choose LSB or MSB
    if (cnt%2==1)
    {
      n_rem ^= (uint16_t) ((n_prom[cnt>>1]) & 0x00FF);
    }
    else
    {
      n_rem ^= (uint16_t) (n_prom[cnt>>1]>>8);
    }

    for (uint8_t n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & (0x8000))
      {
        n_rem = (n_rem << 1) ^ 0x3000;
      }
      else
      {
        n_rem = (n_rem << 1);
      }
    }
  }

  n_rem = ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
  n_prom[0] = temp1;
  n_prom[7] = temp2;

  return (n_rem == crc);
}

void reset_water()
{
  Wire.beginTransmission(MS5837_ADDR);
  Wire.write(MS5837_RESET);
  Wire.endTransmission();
  delay(10);
}


