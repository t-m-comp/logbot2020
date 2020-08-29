static uint8_t runTree()
{
  float gps_manh_distance = Distance_Manhattan();
  if (gps_manh_distance <= 1.3329076651280594)
  {
    if (gps_manh_distance <= 0.663887073174009)
    {
      float gps_manh_displacement = Displacement_Manhattan();
      if (gps_manh_displacement <= 0.624568221969582)
      {
        if (gps_manh_displacement <= 0.07389812754446892)
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
        return 0; //walking
      }
    }
    else
    {
      float gps_manh_displacement = Displacement_Manhattan();
      if (gps_manh_displacement <= 1.0213879373361423)
      {
        float gps_var_manh_displacement = VarianceDisplacement_Manhattan();
        if (gps_var_manh_displacement <= 0.08493880620628096)
        {
          return 0; //walking
        }
        else
        {
          return 1; //running
        }
      }
      else
      {
        return 1; //running
      }
    }
  }
  else
  {
    return 1; //running
  }
}
