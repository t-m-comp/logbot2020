static uint8_t runTree()
{
  float acc_mag_mean = Mean(mag_buffer, ACC_BUFFER_SIZE);
  uint8_t acc_mag_mc = RollingMeanCross(mag_buffer, acc_mag_mean);
  if (acc_mag_mc <= 7.5)
  {
    float acc_mag_var = Variance(mag_buffer, ACC_BUFFER_SIZE);
    if (acc_mag_var <= 0.0045491240452975035)
    {
      if (acc_mag_var <= 0.0034175069304183125)
      {
        float acc_mag_kurtosis = Kurtosis(mag_buffer);
        if (acc_mag_kurtosis <= 2.101611614227295)
        {
          return 2; //stationary
        }
        else
        {
          return 2; //stationary
        }
      }
      else
      {
        if (acc_mag_mc <= 6.5)
        {
          return 2; //stationary
        }
        else
        {
          return 1; //biking
        }
      }
    }
    else
    {
      if (acc_mag_mc <= 6.5)
      {
        if (acc_mag_var <= 0.034942399710416794)
        {
          return 0; //walking
        }
        else
        {
          return 0; //walking
        }
      }
      else
      {
        float acc_mag_RMS = RMS(mag_buffer);
        float acc_mag_crest = Crest(mag_buffer, acc_mag_RMS);
        if (acc_mag_crest <= 0.6191019713878632)
        {
          return 0; //walking
        }
        else
        {
          return 1; //biking
        }
      }
    }
  }
  else
  {
    float acc_mag_RMS = RMS(mag_buffer);
    float acc_mag_crest = Crest(mag_buffer, acc_mag_RMS);
    if (acc_mag_crest <= 0.09328557550907135)
    {
      if (acc_mag_crest <= 0.07116721197962761)
      {
        return 2; //stationary
      }
      else
      {
        if (acc_mag_crest <= 0.07265827804803848)
        {
          return 1; //biking
        }
        else
        {
          return 2; //stationary
        }
      }
    }
    else
    {
      if (acc_mag_mc <= 8.5)
      {
        float acc_mag_kurtosis = Kurtosis(mag_buffer);
        if (acc_mag_kurtosis <= 2.4068626165390015)
        {
          return 0; //walking
        }
        else
        {
          return 1; //biking
        }
      }
      else
      {
        float acc_mag_var = Variance(mag_buffer, ACC_BUFFER_SIZE);
        if (acc_mag_var <= 0.00182577152736485)
        {
          return 2; //stationary
        }
        else
        {
          return 1; //biking
        }
      }
    }
  }
}
