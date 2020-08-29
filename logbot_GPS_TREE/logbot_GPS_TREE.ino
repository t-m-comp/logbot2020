#define SDA_PORT PORTC
#define SDA_PIN_SOFT 4
#define SCL_PORT PORTC
#define SCL_PIN_SOFT 5
#define I2C_TIMEOUT 10

#define    IMU_BMX055_ACC_ADDRESS         0x19
#define    IMU_BMX055_GYRO_ADDRESS         0x69
#define    IMU_BMX055_MAG_ADDRESS         0x13
#define    GPS_ADDRESS                0x42
#define    WATER_ADDRESS              0xEC
#define    LIGHT_ADDRESS              0x46
#define    BALO_ADDRESS               0xB8

#define rxPin  0
#define txPin  1
#define camRxPin 2
#define rtc1Pin 3
#define sdCDPin 4
#define ledPin 5
#define gpsPwrPin 6
#define camPwrPin 7
#define gpsTxPin 8
#define gpsRxPin 9
#define tstBtnPin 10
#define sdCmdPin 11
#define sdDatPin 12
#define sdClkPin 13
#define camTxPin A0
#define batVolPin A1
#define sdPwrPin A2
#define extBoardConnectPin A3

#include <Wire.h>
#include <SoftI2CMaster.h>
#include<avr/sleep.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>

SoftwareSerial softSerial(gpsRxPin, gpsTxPin);

#define array_length(x) (sizeof(x) / sizeof(x[0]))

#define YEAR ((__DATE__[9] - '0') * 10 + (__DATE__[10] - '0'))

#define UTC_OFFSET 9

#define SLEEP_CTRL_BY_TIME
#define DAWN 313
#define DUSK 1094

#define ILLUM_THRESHOLD 1

//Example values for MAX_VIDEO_COUNT
//7.2 GB SD: 1 min vids: 400x; 5 min vids: 80x
//28.8 GB SD: 1 min vids: 1800x; 5 min vids: 400x
//only needed when using large capacity batteries that can record enough video to fill camera SD
#define MAX_VIDEO_COUNT 400

#define GPS_PER_MINUTE

//Delay logging for STARTUP_DELAY number of minutes
#define STARTUP_DELAY 0

#define REST_TIME_SEC 300
#define RECORD_TIME_SEC 60

//pressure in millibars
//NOTE: pressure reading have at least +-5 millibar noise,
//if possible don't use threshold less than 20 to avoid false positives
#define PRESSURE_BUFFER_SIZE 10
#define PRESSURE_THRESHOLD 30


//minutes of GPS data to collect when running decision tree
#define GPS_BUFFER_SIZE 10

#define ACC_BUFFER_SIZE 31

#define DEG_LEN 110.25

#define FPT_KM 2

//largest gap in GPS data before the buffer should be restarted
#define MAX_GAP 2

//number of consecutive windows that should be id'd as target behavior before starting camera with GPS data
#define CONSEC 2
#define FLIGHT_MIN 5
#define FLIGHT_COUNTDOWN 5

#define TARGET_CLASS_NUMBER 0

const float SIN_VALUES[] = { 0, 0.7071068f, 1, 0.7071068f };  //left to right is SIN(0)->SIN(45)->SIN(90)->SIN(135)
const float COS_VALUES[] = { 1, 0.7071068f, 0, -0.7071068f };  //left to right is COS(0)->COS(45)->COS(90)->COS(135)

bool SD_pwr_on = false;
bool CAM_pwr_on = false;
bool GPS_pwr_on = false;
bool IMU_pwr_on = false;
bool ILLUM_pwr_on = false;

volatile bool state_1Hz = false;
bool time_initialized = false;
uint16_t sleep_remaining = STARTUP_DELAY;

int8_t activity_class = -1;
int16_t video_count = -1;
bool camera_ctrl_on = false;
uint32_t camera_ctrl_count = 0;

uint8_t GPS_index = 0;
uint16_t GPS_gap = 0; //incremented each time a GPS is not collected, reset when GPS is collected, reset when device first wakes up
uint16_t GPS_collected = 0; //incremented each time a new GPS location is counted, reset when GPS_gap > MAX_GAP, reset when device first wakes up
uint8_t consec = 0;
float latitude_buffer[GPS_BUFFER_SIZE];
float longitude_buffer[GPS_BUFFER_SIZE];
float rotated_latitude_buffer[GPS_BUFFER_SIZE];
float rotated_longitude_buffer[GPS_BUFFER_SIZE];
uint16_t minute_buffer[GPS_BUFFER_SIZE];



uint16_t last_min = 0;
uint16_t gps_timeout = 0;

int16_t battery_level = 0;
uint16_t pressure = 0;
uint16_t current_illuminance = 100;
uint16_t min_counter = 0;
uint8_t current_sec = 100;

uint8_t prev_dir_day = 0;
uint8_t prev_dir_hour = 0;

#include "user_data_type.h"

File logFile;

gps_t gpsdata;
rtc1_t timedata;

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(gpsPwrPin, OUTPUT);

  //リファレンスモード用スイッチを入力に
  pinMode(tstBtnPin, INPUT);
  //外部ボード接続確認用ポート
  pinMode(extBoardConnectPin, INPUT);
  pinMode(rtc1Pin, INPUT_PULLUP);

  pinMode(camPwrPin, OUTPUT);
  pinMode(camTxPin, OUTPUT);

  pinMode(sdPwrPin, OUTPUT);
  pinMode(sdCmdPin, OUTPUT);
  pinMode(sdClkPin, OUTPUT);
  pinMode(sdCDPin, OUTPUT);
  pinMode(sdDatPin, INPUT);

  digitalWrite(sdPwrPin, LOW);
  digitalWrite(sdCmdPin, LOW);
  digitalWrite(sdClkPin, LOW);
  digitalWrite(sdCDPin, LOW);

  digitalWrite(gpsPwrPin, LOW);

  digitalWrite(camPwrPin, LOW);
  digitalWrite(camTxPin, LOW);

  strobe(500, 0);

  /******************************************
     I2C接続開始．高速化400Khz
   ******************************************/
  Wire.begin();
  Wire.setClock(100000L);

  /*****************************************************************
     IFボードに接続していていたらSD読み込みモードに移行するか選択する
   *****************************************************************/
  if (digitalRead(extBoardConnectPin) == 1)
  {
    Serial.begin(9600);
    Serial.println(F(__DATE__));
    Serial.println(F(__TIME__));
    Serial.flush();
    Serial.end();

    while (digitalRead(extBoardConnectPin) == 1)
    {
      if (digitalRead(tstBtnPin))
      {
        break;
      }
    }
  }

  /*********************************************
     ここから各センサのSetup
   *********************************************/

  //加速度センサ
  set_up_BMX055();

  //気圧センサ
  setup_barometer();

  setCLKOUT_RTC1();
  setTimer_RTC1();

  //割り込み処理
  attachInterrupt(digitalPinToInterrupt(rtc1Pin), Timer1Hz, FALLING);

  if (digitalRead(extBoardConnectPin) == 0)
  {
    start_SD();
  }

  //水圧センサのセットアップ
  setup_wp();
  start_GPS();

  initGPSdata(gpsdata);

  strobe(250, 250);
  strobe(250, 0);
}

void Timer1Hz()
{
  state_1Hz = true;
}

void strobe(const uint16_t &on, const uint16_t &off)
{
  digitalWrite(ledPin, HIGH);
  delay(on);
  digitalWrite(ledPin, LOW);
  delay(off);
}

void loop()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sei();
  sleep_mode();

  if (state_1Hz)
  {
    battery_level = analogRead(batVolPin);
    if (battery_level < 300)
    {
      stop_CAM();
      camera_ctrl_on = false;
      camera_ctrl_count = 0;
    }

    rtc1_t time;
    read_time(time);
    min_counter = convert_gmt_to_local_hour(time.hour) * 60 + time.min;
    current_sec = time.sec;

    if (time_initialized && !CAM_pwr_on && (sleep_remaining > 0 || sleepCtrlByTime(min_counter)))
    {
      initGPSdata(gpsdata);

      last_min = 0;
      gps_timeout = 0;

      consec = 0;
      GPS_collected = 0;
      GPS_gap = 0;

      if (current_sec == 0 && sleep_remaining > 0)
      {
        sleep_remaining -= 1;
      }

      if (GPS_pwr_on)
      {
        stop_GPS();
      }

      if (IMU_pwr_on)
      {
        sleep_BMX055();
      }

      stop_CAM();
      camera_ctrl_on = false;
      camera_ctrl_count = 0;
    }
    else //light turned on in this scope
    {
      //あらかじめ照度センサをONにする(要200msec)
      start_ILLUM();

      /*************************************************************
         センサデータの読み込み
       *************************************************************/
      //RTCからの読み込みと保存
      char directory[5] = {'\0'};
      char filename[12] = {'\0'};

      if (!GPS_pwr_on)
      {
        start_GPS();

        read_time(time);
        min_counter = convert_gmt_to_local_hour(time.hour) * 60 + time.min;
        current_sec = time.sec;
      }

      if (!IMU_pwr_on)
      {
        wake_BMX055();
      }

      if (!time_initialized) //using GPS for DATETIME data
      {
        strobe(25, 50);
        strobe(25, 0);

        if (read_gps_date(timedata))
        {
          time_initialized = adjust_RTC1(timedata);
        }
        else
        {
          send_gps_date_request();
        }
      }
      else
      {
        initGPSdata(gpsdata);

        if (GPS_pwr_on)
        {
          if (current_sec == 58 || current_sec == 59)
          {
            send_gps_loc_request();
          }
          else if (current_sec == 0)
          {
            read_gps_loc(gpsdata);
          }
        }

        save_time(directory, filename, time, prev_dir_day, prev_dir_hour);

        save_acc_data();

        //その他（気圧とか）のデータの読み込みと保存
        //関数内部で照度によるカメラON/OFFも行っている
        //pressure data is read here, so any use of pressure variable is stale until after this call
        save_other_data();

        write_gps(gpsdata);

        activity_class = -1;

        if (current_sec == 0)
        {
          if (gpsdata.gps_status < 2 || gpsdata.lat == 0 || gpsdata.lon == 0)
          {
            GPS_gap += 1;
            if (GPS_gap > MAX_GAP)
            {
              GPS_collected = 0;
              consec = 0;
            }
          }
          else
          {
            GPS_gap = 0;
            GPS_collected += 1;

            latitude_buffer[GPS_index] = gpsdata.lat;
            longitude_buffer[GPS_index] = gpsdata.lon;
            minute_buffer[GPS_index] = min_counter;

            GPS_index = RollingIndex(GPS_index + 1, GPS_BUFFER_SIZE);
          }
        }

        camera_ctrl_count++;

        if (camera_ctrl_on && camera_ctrl_count >= RECORD_TIME_SEC)
        {
          camera_ctrl_on = false;
          camera_ctrl_count = 0;
        }

        if (video_count < MAX_VIDEO_COUNT && battery_level > 299)
        {
          if (current_sec == 0 && camCtrlByGps(gpsdata) && !camera_ctrl_on && camera_ctrl_count > REST_TIME_SEC && current_illuminance >= ILLUM_THRESHOLD)
          {
            camera_ctrl_on = true;
            camera_ctrl_count = 0;
          }
        }

        if (camera_ctrl_on)
        {
          start_CAM();
        }
        else
        {
          stop_CAM();
        }
      } //end data recording
      if (ILLUM_pwr_on)
      {
        stop_ILLUM();
      }
    }

    state_1Hz = false;
  }
}

static uint8_t convert_gmt_to_local_hour(const uint8_t &gmt_hour)
{
  return (24 + gmt_hour + UTC_OFFSET) % 24;
}

//Should only be used with values representing minutes since midnight (0-1440)
static uint16_t minute_of_day_subtraction(const uint16_t &end_minute, const uint16_t &start_minute)
{
  return start_minute > end_minute ? (end_minute+1440) - start_minute : end_minute-start_minute;
}

void setFilename(char directory[], char filename[], const rtc1_t &time)
{
  char dd[3] = {'\0'};
  char hh[3] = {'\0'};
  char mm[3] = {'\0'};
  sprintf(dd, "%02d", time.day);
  sprintf(hh, "%02d", time.hour);
  sprintf(mm, "%02d", time.min);
  directory[0] = dd[0];
  directory[1] = dd[1];
  directory[2] = hh[0];
  directory[3] = hh[1];
  filename[0] = dd[0];
  filename[1] = dd[1];
  filename[2] = hh[0];
  filename[3] = hh[1];
  //filename[4]='0'; filename[5]='0';
  filename[4]='/';
  filename[5]=mm[0];
  filename[6]=mm[1];
  filename[7] = '.';
  filename[8] = 'b';
  filename[9] = 'i';
  filename[10] = 'n';
}

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

static bool camCtrlByGps(const gps_t &gpsdata)
{
  if (GPS_collected >= GPS_BUFFER_SIZE)
  {
    activity_class = runTree();

    if (activity_class == TARGET_CLASS_NUMBER)
    {
      consec += 1;
    }
    else
    {
      consec = 0;
    }

    if (consec >= CONSEC)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    consec = 0;
    return false;
  }
}


//returns True if the logger should sleep
static bool sleepCtrlByTime(const uint16_t &minute_time)
{
  if (minute_time < DAWN || minute_time > DUSK)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void save_time(char directory[], char filename[], const rtc1_t &time, uint8_t &prev_dir_day, uint8_t &prev_dir_hour)
{
  setFilename(directory, filename, time);
  write_time(directory, filename, time, prev_dir_day, prev_dir_hour);
}

static void save_acc_data()
{
  write_BMX055_FIFO_to_SD();
}

void save_other_data()
{
  data_t d;
  read_BMX055_other(d.mag, d.temp);

  read_barometer(d.atmopress);

  read_water_D1D2(d.D1, d.D2);

  d.camera_status = CAM_pwr_on;
  if (CAM_pwr_on)
  {
    d.video_count = video_count;
  }
  else
  {
    d.video_count = -1;
  }

  d.activity_class = activity_class;

  read_ILLUM(d.illuminance);

  d.battery_level = battery_level;

  write_other_data(d);

  current_illuminance = d.illuminance;
}

void initGPSdata(gps_t &gpsdata)
{
  gpsdata.gps_hour = 0;
  gpsdata.gps_min = 0;
  gpsdata.gps_sec = 0;
  gpsdata.lat = 0;
  gpsdata.lon = 0;
  gpsdata.alt = 0;
  gpsdata.gps_status = 0;
}

void copyGPSdata(const gps_t &gpsfrom, gps_t &gpsto)
{
  gpsto.gps_hour = gpsfrom.gps_hour;
  gpsto.gps_min = gpsfrom.gps_min;
  gpsto.gps_sec = gpsfrom.gps_sec;
  gpsto.lat = gpsfrom.lat;
  gpsto.lon = gpsfrom.lon;
  gpsto.alt = gpsfrom.alt;
  gpsto.gps_status = gpsfrom.gps_status;
}

static uint16_t MaxDiff(const uint16_t data[], const uint8_t &buffer_size)
{
    uint16_t max_val = data[0];
    uint16_t min_val = data[0];

    for (byte i = 1; i < buffer_size; i++)
    {
      max_val = data[i] > max_val ? data[i] : max_val;
      min_val = data[i] < min_val ? data[i] : min_val;
    }

    return max_val - min_val;
}

static bool MeanCross(const float &x, const float &y, const float &mean)
{
    return (x > mean) != (y > mean);
}

static bool OneCross(const float &x, const float &y)
{
    return (x > 1) != (y > 1);
}

static float Mean(const float data[], const uint8_t &buffer_size)
{
    float mean = 0;

    for (uint8_t i = 0; i < buffer_size; i++)
    {
        mean += data[i];
    }

    return mean / buffer_size;
}

static float Variance(const float data[], const uint8_t &buffer_size)
{
    float M = 0;
    float oldM = 0;
    float S = 0;

    for (uint8_t i = 0; i < buffer_size; i++)
    {
        oldM = M;
        M = (M + (data[i] - M) / (i + 1));
        S = (S + (data[i] - M) * (data[i] - oldM));
    }

    return S / (buffer_size - 1);
}

static uint8_t RollingIndex(const uint8_t &index, const uint8_t &buffer_size)
{
    return index % buffer_size;
}

static float MeanDiff(const float data[], const uint8_t &buffer_size, const uint8_t &start_index)
{
    float mean = 0;

    for (byte i = 1; i < buffer_size; i++)
    {
        mean += data[RollingIndex(start_index+i, buffer_size)] - data[RollingIndex(start_index+i-1, buffer_size)];
    }

    return mean / (buffer_size-1);
}

static float ApproximateSpeed(const uint8_t &index1, const uint8_t &index2)
{
  float d = minute_of_day_subtraction(minute_buffer[index2], minute_buffer[index1]);
  return d > 0 ? ManhattanDistance(index1, index2) * 16.66667f / d : 0;
}

//used to rotate the matrix 22.5, 45, 67.5, and 90 degrees to find a rotation that maximizes the variance
static void RotateLatLonMatrix(float lat_lon_mean[], float lat_lon_var[])
{
    lat_lon_mean[0] = Mean(latitude_buffer, GPS_BUFFER_SIZE);
    lat_lon_mean[1] = Mean(longitude_buffer, GPS_BUFFER_SIZE);

    float maxVariance = Variance(longitude_buffer, GPS_BUFFER_SIZE);
    uint8_t maxIndex = 0;

    for (uint8_t j = 1; j < 4; j++)
    {
        Rotate(j, lat_lon_mean);

        lat_lon_var[1] = Variance(rotated_longitude_buffer, GPS_BUFFER_SIZE);

        if (lat_lon_var[1] > maxVariance)
        {
            maxVariance = lat_lon_var[1];
            maxIndex = j;
        }
    }

    if (maxIndex < 4)
    {
        Rotate(maxIndex, lat_lon_mean);
        lat_lon_var[1] = maxVariance;
    }

    lat_lon_var[0] = Variance(rotated_latitude_buffer, GPS_BUFFER_SIZE);
    lat_lon_mean[0] = Mean(rotated_latitude_buffer, GPS_BUFFER_SIZE);
    lat_lon_mean[1] = Mean(rotated_longitude_buffer, GPS_BUFFER_SIZE);
}

static int16_t ThreePointAngle(const uint8_t &index1, const uint8_t &index2, const uint8_t &index3)
{
    int16_t a1 = Bearing(index2, index1);
    int16_t a2 = Bearing(index2, index3);

    return (int16_t) min((a1 - a2) < 0 ? a1 - a2 + 360 : a1 - a2, (a2 - a1) < 0 ? a2 - a1 + 360 : a2 - a1);
}

static float ManhattanDistance(const uint8_t &index1, const uint8_t &index2)
{
    float dy = latitude_buffer[index2] - latitude_buffer[index1];
    float dx = longitude_buffer[index2] - longitude_buffer[index1];

    return DEG_LEN * (abs(dx) + abs(dy));
}

static uint16_t FirstPassTime_Manhattan()
{
    uint16_t startTime = minute_buffer[GPS_index];
    uint16_t endTime = minute_buffer[GPS_index];

    for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
    {
        endTime = minute_buffer[RollingIndex(GPS_index+i, GPS_BUFFER_SIZE)];

        if (ManhattanDistance(RollingIndex(GPS_index+i, GPS_BUFFER_SIZE), GPS_index) > FPT_KM)
        {
            break;
        }
    }

    return minute_of_day_subtraction(endTime, startTime);
}

static void MeanCrossLatLon(uint8_t lat_lon_zc[], const bool &run_rotate, float lat_lon_mean[], float lat_lon_var[])
{
    if (run_rotate)
    {
      RotateLatLonMatrix(lat_lon_mean, lat_lon_var);
    }
    lat_lon_zc[0] = RollingMeanCross(rotated_latitude_buffer, lat_lon_mean[0]);
    lat_lon_zc[1] = RollingMeanCross(rotated_longitude_buffer, lat_lon_mean[1]);
}

static float MeanApproximateSpeed()
{
    float speed_acum = 0;
    for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
    {
        speed_acum += ApproximateSpeed(RollingIndex(GPS_index+i-1, GPS_BUFFER_SIZE), RollingIndex(GPS_index+i, GPS_BUFFER_SIZE));
    }

    return speed_acum / (GPS_BUFFER_SIZE - 1);
}

static float VarianceApproximateSpeed()
{
    float M = 0;
    float oldM = 0;
    float S = 0;
    float sp = 0;

    for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
    {
        sp = ApproximateSpeed(RollingIndex(GPS_index+i-1, GPS_BUFFER_SIZE), RollingIndex(GPS_index+i, GPS_BUFFER_SIZE));
        oldM = M;
        M = M + (sp - M) / i;
        S = S + (sp - M) * (sp - oldM);
    }

    return S / (GPS_BUFFER_SIZE - 2);
}

static float Displacement_Manhattan()
{
    return ManhattanDistance(RollingIndex(GPS_index+GPS_BUFFER_SIZE-1, GPS_BUFFER_SIZE), GPS_index);
}

static float Distance_Manhattan()
{
    float distance = 0;

    for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
    {
        distance += ManhattanDistance(RollingIndex(GPS_index+i, GPS_BUFFER_SIZE), RollingIndex(GPS_index+i-1, GPS_BUFFER_SIZE));
    }

    return distance;
}

static float MaxDisplacement_Manhattan()
{
    float max_displacement = 0;

    for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
    {
        float displacement = ManhattanDistance(RollingIndex(GPS_index+i, GPS_BUFFER_SIZE), GPS_index);

        if (displacement > max_displacement)
        {
            max_displacement = displacement;
        }
    }

    return max_displacement;
}

static float MeanDisplacement_Manhattan()
{
    float mean_displacement = 0;

    for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
    {
        mean_displacement += ManhattanDistance(RollingIndex(GPS_index+i, GPS_BUFFER_SIZE), GPS_index);
    }

    return mean_displacement / (GPS_BUFFER_SIZE - 1);
}

static float VarianceDisplacement_Manhattan()
{
    float M = 0;
    float oldM = 0;
    float S = 0;
    float d = 0;

    for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
    {
        d = ManhattanDistance(RollingIndex(GPS_index+i, GPS_BUFFER_SIZE), GPS_index);
        oldM = M;
        M = M + (d - M) / i;
        S = S + (d - M) * (d - oldM);
    }

    return S / (GPS_BUFFER_SIZE - 2);
}

static float MeanGPSAngle()
{
    float mean_angle = 0;

    for (uint8_t i = 2; i < GPS_BUFFER_SIZE; i++)
    {
        mean_angle += ThreePointAngle(RollingIndex(GPS_index+i, GPS_BUFFER_SIZE), RollingIndex(GPS_index+i-1, GPS_BUFFER_SIZE), RollingIndex(GPS_index+i-2, GPS_BUFFER_SIZE));
    }

    return mean_angle / (GPS_BUFFER_SIZE - 2);
}

static float VarianceGPSAngle()
{
    float M = 0;
    float oldM = 0;
    float S = 0;
    float a = 0;

    for (uint8_t i = 2; i < GPS_BUFFER_SIZE; i++)
    {
        a = ThreePointAngle(RollingIndex(GPS_index+i, GPS_BUFFER_SIZE), RollingIndex(GPS_index+i-1, GPS_BUFFER_SIZE), RollingIndex(GPS_index+i-2, GPS_BUFFER_SIZE));
        oldM = M;
        M = M + (a - M) / (i - 1);
        S = S + (a - M) * (a - oldM);
    }

    return S / (GPS_BUFFER_SIZE - 2);
}

static float MeanDisplacementAngle()
{
    float mean_displacement_angle = 0;

    for (uint8_t i = 1; i < GPS_BUFFER_SIZE - 1; i++)
    {
        mean_displacement_angle += ThreePointAngle(RollingIndex(GPS_index+GPS_BUFFER_SIZE-1, GPS_BUFFER_SIZE), RollingIndex(GPS_index+i, GPS_BUFFER_SIZE), GPS_index);
    }

    return mean_displacement_angle / (GPS_BUFFER_SIZE - 2);
}

static void Rotate(const int &j, const float lat_lon_mean[])
{
    float x;
    float y;

    float c = cos(DEG_TO_RAD * lat_lon_mean[0]);

    for (uint8_t i = 0; i < GPS_BUFFER_SIZE; i++)
    {
        //Source: http://www.movable-type.co.uk/scripts/latlong.html
        //Equirectangular approximation
        //Formula x = Δλ ⋅ cos φm
        x = (longitude_buffer[i] - lat_lon_mean[1]) * c;
        //y = Δφ
        y = latitude_buffer[i] - lat_lon_mean[0];

        //y′= y cosθ + x sinθ
        rotated_latitude_buffer[i] = y * COS_VALUES[j] + x * SIN_VALUES[j];

        //x′= x cosθ − y sinθ
        rotated_longitude_buffer[i] = x * COS_VALUES[j] - y * SIN_VALUES[j];
    }
}

static int16_t Bearing(const uint8_t &index1, const uint8_t &index2)
{
    float dLon = longitude_buffer[index2] - longitude_buffer[index1];
    float y = sin(DEG_TO_RAD * dLon) * cos(DEG_TO_RAD * latitude_buffer[index2]);
    float x = cos(DEG_TO_RAD * latitude_buffer[index1]) * sin(DEG_TO_RAD * latitude_buffer[index2]) - sin(DEG_TO_RAD * latitude_buffer[index1]) * cos(DEG_TO_RAD * latitude_buffer[index2]) * cos(DEG_TO_RAD * dLon);

    return ((int16_t)(atan2(y, x) * RAD_TO_DEG + 360) % 360);
}

static uint8_t RollingMeanCross(const float data[], const float &mean)
{
    uint8_t mc = 0;

    for (uint8_t i = 1; i < GPS_BUFFER_SIZE; i++)
    {
        mc += MeanCross(data[RollingIndex(GPS_index+i, GPS_BUFFER_SIZE)], data[RollingIndex(GPS_index+i-1, GPS_BUFFER_SIZE)], mean) ? 1 : 0;
    }

    return mc;
}
