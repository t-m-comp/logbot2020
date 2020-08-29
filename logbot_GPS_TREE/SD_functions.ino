void start_SD()
{
  digitalWrite(sdPwrPin, HIGH);
  while (!SD.begin(sdCDPin))
  {

  }

  SD_pwr_on = true;
}

void stop_SD()
{
  if (SD_pwr_on)
  {
    if (logFile)
    {
      logFile.close();
    }

    SD.end();
    digitalWrite(sdPwrPin, LOW);
    digitalWrite(sdCmdPin, LOW);
    digitalWrite(sdClkPin, LOW);
    digitalWrite(sdCDPin, LOW);
    SD_pwr_on = false;
  }
}

void write_time(const char directory[], const char filename[], const rtc1_t &time, uint8_t &prev_dir_day, uint8_t &prev_dir_hour)
{
  if (SD_pwr_on)
  {
    if (prev_dir_day != time.day || prev_dir_hour != time.hour)
    {
      SD.mkdir(directory);
      prev_dir_day = time.day;
      prev_dir_hour = time.hour;
    }

    logFile = SD.open(filename, FILE_WRITE);

    if(logFile)
    {
      logFile.write((uint8_t *)&time, sizeof(time));
    }
  }
}

//NOTE: arrays decay to pointers when passed to a function, so sizeof(array) must be called before the function call, not within the function
void write_C(const uint16_t C[], const uint8_t &array_size)
{
  if (SD_pwr_on)
  {
    File calibFile = SD.open(F("calib.bin"), (O_READ | O_WRITE | O_CREAT)); //Turns off append function, this does not delete the existing file, it just sets the write head to the beginning of the file and starts overwriting from there
    if(calibFile)
    {
      calibFile.write((uint8_t *)C, array_size);
      calibFile.close();
    }
  }
}

void write_gps(const gps_t &gpsdata)
{
  if (SD_pwr_on)
  {
    if(logFile)
    {
      logFile.write((uint8_t *)&gpsdata, sizeof(gpsdata));
      logFile.close();
    }
  }
}

void write_other_data(const data_t &data)
{
  if (SD_pwr_on)
  {
    if(logFile)
    {
      logFile.write((uint8_t *)&data, sizeof(data));
    }
  }
}
