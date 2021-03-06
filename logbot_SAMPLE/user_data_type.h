/************************************************
 * RTC1から読み出す時間（曜日は不要）
 ************************************************/
struct rtc1_t
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
};

struct gps_t
{
  uint8_t gps_hour;
  uint8_t gps_min;
  uint8_t gps_sec;

  //緯度
  double lat;

  //経度
  double lon;

  //高度
  double alt;

  uint8_t gps_status;
};

struct RTC8564_TIME
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t weekday;
};


/************************************************
 * 毎秒SDに保存するためのデータの構造体
 ***********************************************/
struct data_t
{
  //素子温度
  float temp;

  //磁気センサデータ(x,y,z)
  float mag[3];

  uint32_t atmopress;

  //照度データ
  uint16_t illuminance;

  //水圧データ
  uint32_t D1, D2;

  //カメラが起動しているか否か
  bool camera_status;

  int16_t video_count;

  int8_t activity_class;

  int16_t battery_level;
};
