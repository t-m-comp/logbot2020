static uint8_t runTree()
{
  float acc_mag_var = Variance(mag_buffer, ACC_BUFFER_SIZE);
  if (acc_mag_var <= 0.0332538527743699)
  {
    if (acc_mag_var <= 0.011276327050105436)
    {
      if (acc_mag_var <= 0.0037011223862355096)
      {
        if (acc_mag_var <= 0.0026676088001101415)
        {
          return 2; //stationary
        }
        else
        {
          return 1; //biking
        }
      }
      else
      {
        float acc_mag_energy = Energy(mag_buffer);
        if (acc_mag_energy <= 1.3675155110846724)
        {
          return 1; //biking
        }
        else
        {
          return 1; //biking
        }
      }
    }
    else
    {
      float acc_mag_mean = Mean(mag_buffer, ACC_BUFFER_SIZE);
      uint8_t acc_mag_mc = RollingMeanCross(mag_buffer, acc_mag_mean);
      if (acc_mag_mc <= 3.9001180165913505)
      {
        if (acc_mag_var <= 0.017506434363149477)
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
        if (acc_mag_mc <= 5.184851209253469)
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
    float acc_mag_mean = Mean(mag_buffer, ACC_BUFFER_SIZE);
    uint8_t acc_mag_mc = RollingMeanCross(mag_buffer, acc_mag_mean);
    if (acc_mag_mc <= 10.28761016218346)
    {
      if (acc_mag_var <= 1.020556659449329)
      {
        if (acc_mag_var <= 0.6679017245615443)
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
        return 1; //biking
      }
    }
    else
    {
      if (acc_mag_mc <= 12.079741275440782)
      {
        if (acc_mag_mc <= 11.463635320059785)
        {
          return 1; //biking
        }
        else
        {
          return 1; //biking
        }
      }
      else
      {
        return 1; //biking
      }
    }
  }
}
