static uint8_t runTree()
{
  float gps_avg_manh_speed = MeanApproximateSpeed();
  if (gps_avg_manh_speed <= 2.0008248686790466)
  {
    float gps_var_manh_speed = VarianceApproximateSpeed();
    if (gps_var_manh_speed <= 0.31658706068992615)
    {
      return 0; //walking
    }
    else
    {
      float gps_var_angle = VarianceGPSAngle();
      if (gps_var_angle <= 898.3047790527344)
      {
        float gps_avg_angle = MeanGPSAngle();
        if (gps_avg_angle <= 151.125)
        {
          return 1; //running
        }
        else
        {
          return 0; //walking
        }
      }
      else
      {
        float lat_lon_var[2];
        float lat_lon_mean[2];
        RotateLatLonMatrix(lat_lon_mean, lat_lon_var);
        float gps_lon_var = lat_lon_var[1];
        if (gps_lon_var <= 4.402566332828428e-06)
        {
          return 0; //walking
        }
        else
        {
          return 1; //running
        }
      }
    }
  }
  else
  {
    return 1; //running
  }
}
