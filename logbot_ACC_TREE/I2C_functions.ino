void I2Cread(const uint8_t &Address, const uint8_t &Register, const uint8_t &Nbytes, uint8_t Data[])
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

uint8_t I2C_read_byte(const uint8_t &address, const uint8_t &subaddress)
{
  uint8_t data[1] = {0};
  I2C_read(address, subaddress, 1, data);
  return data[0];
}

uint8_t I2C_read(const uint8_t &address, const uint8_t &n_bytes, uint8_t data[])
{
  uint8_t bytes_read = 0;

  // Read Nbytes
  Wire.requestFrom(address, n_bytes);
  for (uint8_t i = 0; i < n_bytes; i++)
  {
    if (!Wire.available())
    {
      delay(5);
    }

    if (Wire.available())
    {
      data[i]=Wire.read();
      bytes_read++;
    }
    else
    {
      break;
    }
  }

  return bytes_read;
}

uint8_t I2C_read(const uint8_t &address, const uint8_t &subaddress, const uint8_t &n_bytes, uint8_t data[])
{
  uint8_t bytes_read = 0;

  I2C_write_byte(address, subaddress);

  return I2C_read(address, n_bytes, data);
}

// Write a byte (data) in device (address) at register (subaddress)
void I2C_write_byte(const uint8_t &address, const uint8_t &subaddress, const uint8_t &data)
{
  // Set register address
  Wire.beginTransmission(address);
  Wire.write(subaddress);
  Wire.write(data);
  Wire.endTransmission();
}

void I2C_write_byte(const uint8_t &address, const uint8_t &data)
{
  // Set register address
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}
