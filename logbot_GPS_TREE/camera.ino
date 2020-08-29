void start_CAM()
{
  digitalWrite(camPwrPin, HIGH);

  if (!CAM_pwr_on)
  {
    CAM_pwr_on = true;
    video_count++;
  }
}


void stop_CAM()
{
  digitalWrite(camPwrPin, LOW);
  CAM_pwr_on = false;
}
