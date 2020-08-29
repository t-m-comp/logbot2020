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

#define CONSEC 2
#define FLIGHT_MIN 5
#define FLIGHT_COUNTDOWN 5

#define TARGET_CLASS_NUMBER 0


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


uint8_t flight_count = 0;
uint8_t flight_countdown = 0;
float mag_buffer[ACC_BUFFER_SIZE];
bool ACC_FIFO_FILLED = false;


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

      flight_count = 0;
      flight_countdown = 0;

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


        camera_ctrl_count++;

        if (camera_ctrl_on && camera_ctrl_count >= RECORD_TIME_SEC)
        {
          camera_ctrl_on = false;
          camera_ctrl_count = 0;
        }

        if (video_count < MAX_VIDEO_COUNT && battery_level > 299)
        {
          if (ACC_FIFO_FILLED && camCtrlByAcc() && !camera_ctrl_on && camera_ctrl_count > REST_TIME_SEC && current_illuminance >= ILLUM_THRESHOLD)
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


static bool camCtrlByAcc()
{
  activity_class = runTree();

  if (activity_class == 1)
  {
    if (flight_count < FLIGHT_MIN)
    {
      flight_count++;
    }

    if (flight_count >= FLIGHT_MIN || flight_countdown > 0)
    {
      flight_count = 0;
      flight_countdown = FLIGHT_COUNTDOWN;
    }
    return false;
  }
  else
  {
    flight_count = 0;
    if (flight_countdown > 0)
    {
      flight_countdown--;
    }

    if (activity_class == TARGET_CLASS_NUMBER && flight_countdown > 0)
    {
      flight_count = 0;
      flight_countdown = 0;

      return true;
    }
    else
    {
      return false;
    }
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

static uint8_t RollingMeanCross(const float data[], const float &mean)
{
    uint8_t mc = 0;

    for (uint8_t i = ACC_BUFFER_SIZE - 1; i > 0; i--)
    {
        mc += MeanCross(data[i - 1], data[i], mean) ? 1 : 0;
    }

    return mc;
}

static uint8_t RollingOneCross(const float data[])
{
    uint8_t oc = 0;

    for (uint8_t i = ACC_BUFFER_SIZE - 1; i > 0; i--)
    {
        oc += OneCross(data[i - 1], data[i]) ? 1 : 0;
    }

    return oc;
}

static float RMS(const float buffer1[])
{
    float sumX2 = 0;

    for (uint8_t i = 0; i < ACC_BUFFER_SIZE; i++)
    {
        sumX2 += buffer1[i] * buffer1[i];
    }
    return sqrt(sumX2 / ACC_BUFFER_SIZE);
}

static float Kurtosis(const float buffer1[])
{
    uint8_t n = 0;
    uint8_t n1 = 0;
    float mean = 0;
    float m2 = 0;
    float m3 = 0;
    float m4 = 0;
    float delta = 0;
    float delta_n = 0;
    float delta_n2 = 0;
    float term1 = 0;

    for (uint8_t i = 0; i < ACC_BUFFER_SIZE; i++)
    {
        n1 = i;
        n = n + 1;
        delta = buffer1[i] - mean;
        delta_n = delta / n;
        delta_n2 = delta_n * delta_n;
        term1 = delta * delta_n * n1;
        mean = mean + delta_n;
        m4 = m4 + term1 * delta_n2 * (n * n - 3 * n + 3) + 6 * delta_n2 * m2 - 4 * delta_n * m3;
        m3 = m3 + term1 * delta_n * (n - 2) - 3 * delta_n * m2;
        m2 = m2 + term1;

    }

    if ((m2 * m3) == 0)
    {
        return 0;
    }
    else
    {
        return (n * m4) / (m2 * m2);
    }
}

static float Crest(const float buffer1[], const float &rms)
{
    if (rms == 0)
    {
        return 0;
    }
    else
    {
      float minVal = buffer1[0];
      float maxVal = buffer1[0];

      for (uint8_t i = 1; i < ACC_BUFFER_SIZE; i++)
      {
          if (buffer1[i] > maxVal)
          {
              maxVal = buffer1[i];
          }

          if (buffer1[i] < minVal)
          {
              minVal = buffer1[i];
          }
      }

        return (0.5f * (maxVal - minVal)) / rms;
    }
}

static float Energy(const float data[])
{
    float magEnergy = 0;

    for (uint8_t i = 0; i < ACC_BUFFER_SIZE; i++)
    {
        magEnergy += (data[i] * data[i]);
    }

    return magEnergy / ACC_BUFFER_SIZE;
}

