//CFG-PRT UART All IN UBX OUT 9600 Extended TX timeout (should be used by both 328 and 2560)
static const byte CFG_PRT_UART[] PROGMEM = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0xA2, 0xB1 };
static const uint8_t CFG_PRT_UART_LEN = sizeof(CFG_PRT_UART)/sizeof(CFG_PRT_UART[0]);

//CFG-PRT I2C none
static const byte CFG_PRT_I2C[] PROGMEM = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9E, 0x88 };
static const uint8_t CFG_PRT_I2C_LEN = sizeof(CFG_PRT_I2C)/sizeof(CFG_PRT_I2C[0]);

//CFG-GNSS turn off GLONASS, enable GPS, SBAS, QZSS
static const byte CFG_GNSS[] PROGMEM = { 0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x05, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x32, 0xAD };
static const uint8_t CFG_GNSS_LEN = sizeof(CFG_GNSS)/sizeof(CFG_GNSS[0]);

//CFG-NAV5 set as airborne <2g 2D/3D auto
static const byte CFG_NAV5[] PROGMEM = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x2A };
static const uint8_t CFG_NAV5_LEN = sizeof(CFG_NAV5)/sizeof(CFG_NAV5[0]);

//CFG-CFG save to flash and BBR
static const byte CFG_CFG[] PROGMEM = { 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB };
static const uint8_t CFG_CFG_LEN = sizeof(CFG_CFG)/sizeof(CFG_CFG[0]);

//CFG-RST Full hardware reset, does not clear flash but will cause system to mostly start from scratch, may be useful when GPS is acting up
//static const byte CFG_RST[] PROGMEM = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xB9, 0x00, 0x00, 0xC6, 0x8B };
//static const uint8_t CFG_RST_LEN = sizeof(CFG_RST)/sizeof(CFG_RST[0]);

void start_GPS()
{
  if (!GPS_pwr_on)
  {
    digitalWrite(gpsPwrPin, HIGH);
    delay(1000);
    softSerial.begin(9600);
    GPS_pwr_on = true;
  }

  uint8_t gpsSetSuccess =0;

  gpsSetSuccess =0;
  while(gpsSetSuccess <6)
  {
    gpsSetSuccess = sendUBXConfig(CFG_PRT_UART, CFG_PRT_UART_LEN);
  }

  gpsSetSuccess =0;
  while(gpsSetSuccess <6)
  {
    gpsSetSuccess = sendUBXConfig(CFG_PRT_I2C, CFG_PRT_I2C_LEN);
  }

  gpsSetSuccess =0;
  while(gpsSetSuccess <6)
  {
    gpsSetSuccess = sendUBXConfig(CFG_GNSS, CFG_GNSS_LEN);
  }

  gpsSetSuccess =0;
  while(gpsSetSuccess <6)
  {
    gpsSetSuccess = sendUBXConfig(CFG_NAV5, CFG_NAV5_LEN);
  }

  gpsSetSuccess =0;
  while(gpsSetSuccess <6)
  {
    gpsSetSuccess = sendUBXConfig(CFG_CFG, CFG_CFG_LEN);
  }

  while (clearBuffer())
  {
    //clear any remaining text in the buffer
  }
}


bool clearBuffer()
{
  while (softSerial.available() > 0)
  {
    if (softSerial.read() == '$')
    {
      return true;
    }
  }
  return false;
}

void stop_GPS()
{
  if (GPS_pwr_on)
  {
    softSerial.flush();
    softSerial.end();
  }
  digitalWrite(gpsPwrPin, LOW);
  GPS_pwr_on = false;
}

void send_gps_loc_request()
{
  softSerial.println(F("$PUBX,00*33"));
}

void send_gps_date_request()
{
  softSerial.println(F("$PUBX,04*37"));
}

void send_gps_status_request()
{
  softSerial.println(F("$PUBX,03*30"));
}

uint8_t read_gps_status()
{
  //send_gps_status_request();

  uint8_t i= 0;
  uint8_t j = 0;
  char raw_count[3] ={'\0'};
  uint8_t count = 0;
  bool valid_flag = false;

  while (softSerial.available() > 0){
    char c= softSerial.read();

    if((i==1 && c != 'P')||(i == 6 && c != '0')||(i == 7 && c != '3')){ //$G...で始まるものがきたら無視
       while (softSerial.available() > 0){
        softSerial.read();
        }
        break;
    }else if(i>=9 && i <=10){
      valid_flag = true;
      raw_count[j] = c;
      j++;
      if(j==2) j=0;
    }
    i++;
  }

  if(valid_flag){ //処理されたエントリが存在
    char *endp;
    count = strtol(raw_count, &endp, 10);
    return count;
  }else{
    return 0;
  }
}

//$PUBX,04,hhmmss.ss,ddmmyy,...
bool read_gps_date(rtc1_t &timedata)
{
  uint8_t field_nbr = 0;
  uint8_t field_index = 0;

  char raw_hour[3] ={'\0'};
  char raw_min[3] ={'\0'};
  char raw_sec[3] ={'\0'};
  char raw_year[3] ={'\0'};
  char raw_mon[3] ={'\0'};
  char raw_day[3] ={'\0'};
  bool valid_flag = false;

  if (clearBuffer())
  {
    while (softSerial.available() > 0)
    {
      char c= softSerial.read();

      if (c == '$')
      {
        field_nbr = 0;
        field_index = 0;
      }
      else if (c == ',')
      {
        field_nbr++;
        field_index = 0;
      }
      else if (field_nbr == 0 || field_nbr == 1) //message type fields
      {
        if ( (field_nbr == 0 && ((field_index == 1 && c != 'U') || (field_index == 2 && c != 'B'))) ||
             (field_nbr == 1 && ((field_index == 0 && c != '0') || (field_index == 1 && c != '4'))) ) //wrong message type
        {
          if (clearBuffer())
          {
            field_nbr = 0;
            field_index = 0;
          }
          else
          {
            break; //break main loop
          }
        }
        else
        {
          field_index++;
        }
      }
      else if (field_nbr == 2) //time field
      {
        if (field_index < 2)
        {
          raw_hour[field_index] = c;
        }
        else if (field_index < 4)
        {
          raw_min[field_index%2] = c;
        }
        else if (field_index < 6)
        {
          raw_sec[field_index%2] = c;
        }

        field_index++;
      }
      else if (field_nbr == 3) //date field
      {
        if (field_index < 2)
        {
          raw_day[field_index] = c;
        }
        else if (field_index < 4)
        {
          raw_mon[field_index%2] = c;
        }
        else
        {
          raw_year[field_index%2] = c;
          if (field_index%2 == 1)
          {
            valid_flag = true;
            break;
          }
        }

        field_index++;
      }
    }
  }

  if(valid_flag)
  { //処理されたエントリが存在
    char *endp;
    timedata.hour = strtol(raw_hour, &endp, 10);
    timedata.min = strtol(raw_min, &endp, 10);
    timedata.sec = strtol(raw_sec, &endp, 10);
    timedata.year = strtol(raw_year, &endp, 10);
    timedata.month = strtol(raw_mon, &endp, 10);
    timedata.day = strtol(raw_day, &endp, 10);

    if (timedata.year >= YEAR) //GPS outputs a valid looking but incorrect date when it first turns on
    {
      return true;
    }
  }
  timedata.hour = 0;
  timedata.min = 0;
  timedata.sec = 0;
  timedata.year = 0;
  timedata.month = 0;
  timedata.day = 0;
  return false;
}

bool read_gps_loc(gps_t &gpsdata)
{
  uint8_t field_nbr = 0;
  uint8_t field_index = 0;

  char raw_hour[3] ={'\0'};
  char raw_min[3] ={'\0'};
  char raw_sec[3] ={'\0'};
  char lat_term[3]={'\0'};
  char lat_deg[3]={'\0'};
  char lat_billi[6]={'\0'};
  int8_t lat_sign = 1;
  char lon_term[4]={'\0'};
  char lon_deg[3]={'\0'};
  char lon_billi[6]={'\0'};
  int8_t lon_sign = 1;
  char altref[6]={'\0'};
  bool valid_flag = false;
  char raw_status[3] = {'\0'};
  gpsdata.gps_status = 8;

  if (clearBuffer())
  {
    while (softSerial.available() > 0)
    {
      char c= softSerial.read();

      if (c == '$')
      {
        field_nbr = 0;
        field_index = 0;
      }
      else if (c == ',')
      {
        field_nbr++;
        field_index = 0;
      }
      else if (field_nbr == 0 || field_nbr == 1) //message type fields
      {
        if ( (field_nbr == 0 && ((field_index == 1 && c != 'U') || (field_index == 2 && c != 'B'))) ||
             (field_nbr == 1 && ((field_index == 0 && c != '0') || (field_index == 1 && c != '0'))) ) //wrong message type
        {
          if (clearBuffer())
          {
            field_nbr = 0;
            field_index = 0;
          }
          else
          {
            break; //break main loop
          }
        }
        else
        {
          field_index++;
        }
      }
      else if (field_nbr == 2) //time field
      {
        if (field_index < 2)
        {
          raw_hour[field_index] = c;
        }
        else if (field_index < 4)
        {
          raw_min[field_index%2] = c;
        }
        else if (field_index < 6)
        {
          raw_sec[field_index%2] = c;
        }

        field_index++;
      }
      else if (field_nbr == 3) //lat field
      {
        if (field_index < 2)
        {
          lat_term[field_index] = c;
        }
        else if (field_index < 4)
        {
          lat_deg[field_index - 2] = c;
        }
        else if (field_index > 4)
        {
          lat_billi[field_index - 5] = c;
        }

        field_index++;
      }
      else if (field_nbr == 4) //lat sign field
      {
        if (c == 'S')
        {
          lat_sign = -1;
        }
      }
      else if (field_nbr == 5) //lon field
      {
        if (field_index < 3)
        {
          lon_term[field_index] = c;
        }
        else if (field_index < 5)
        {
          lon_deg[field_index - 3] = c;
        }
        else if (field_index > 5)
        {
          lon_billi[field_index - 6] = c;
          valid_flag = true;
        }

        field_index++;
      }
      else if (field_nbr == 6) //lon sign field
      {
        if (c == 'W')
        {
          lon_sign = -1;
        }
      }
      else if (field_nbr == 7) //alt field
      {
        if (field_index < 5) //alt field varies in size but is min 5 chars
        {
          altref[field_index] = c;
        }

        field_index++;
      }
      //Only using NF and G3 since our GPS unit appears to only return those two values, kept status codes as 0 and 4 to leave ParseLog file compatible for all codes in the future
      //NF  0 No fix
      //G3  1 Standalone 3D
      else if (field_nbr == 8)
      {
        if (field_index == 0)
        {
          if (c == 'N')
          {
            gpsdata.gps_status = 0;
          }
          else if (c == 'G')
          {
            gpsdata.gps_status = 4;
          }
        }
      }
    }
  }

  if(valid_flag){ //処理されたエントリが存在
    char *endp;
    gpsdata.gps_hour = strtol(raw_hour, &endp, 10);
    gpsdata.gps_min = strtol(raw_min, &endp, 10);
    gpsdata.gps_sec = strtol(raw_sec, &endp, 10);
    gpsdata.lat = getLat(lat_term, lat_deg, lat_billi, lat_sign);
    gpsdata.lon = getLon(lon_term, lon_deg, lon_billi, lon_sign);
    gpsdata.alt = atof(altref);

    return true;
  }
  else
  {
    gpsdata.gps_hour = 0;
    gpsdata.gps_min = 0;
    gpsdata.gps_sec = 0;
    gpsdata.lat = 0;
    gpsdata.lon = 0;
    gpsdata.alt = 0;
    gpsdata.gps_status = 0;
    return false;
  }
}

/**********************************************************
 * Lat/Lonの変換
 **********************************************************/
float getLat(const char lat_term[], const char lat_deg[], const char lat_billi[], const int8_t &lat_sign){
  char *endp;
  return (float)lat_sign*(strtol(lat_term, &endp, 10) + (float)(strtol(lat_deg, &endp, 10)+(float)(strtol(lat_billi, &endp, 10)/100000.0f))/60);
}
float getLon(const char lon_term[], const char lon_deg[], const char lon_billi[], const int8_t &lon_sign){
  char *endp;
  return (float)lon_sign*(strtol(lon_term, &endp, 10) + (float)(strtol(lon_deg, &endp, 10)+(float)(strtol(lon_billi, &endp, 10)/100000.0f))/60);
}

uint8_t sendUBXConfig(const uint8_t *UBXmsg, const uint8_t &msgLength)
{
  for(uint8_t i = 0; i < msgLength; i++)
  {
    softSerial.write(pgm_read_byte(UBXmsg+i));
  }
  softSerial.println();
  softSerial.flush();

  return getUBX_ACK(pgm_read_byte(UBXmsg+2), pgm_read_byte(UBXmsg+3));
}


uint8_t getUBX_ACK(const uint8_t &msgID1, const uint8_t &msgID2)
{
  uint8_t CK_A = 0, CK_B = 0;
  uint8_t incoming_char;
  boolean headerReceived = false;
  unsigned long ackWait = millis();
  uint8_t ackPacket[10] = { 0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  int i = 0;

  while (!softSerial.available())
  {
    //
  }

  while (true)
  {
    if (softSerial.available())
    {
      incoming_char = softSerial.read();

      if (incoming_char == ackPacket[i])
      {
        i++;
      }
      else if (i > 2) //first three bytes are the same for both ack and nak
      {
        ackPacket[i] = incoming_char;
        i++;
      }
    }

    if (i > 9)
    {
      break;
    }
    else if (i == 4 && ackPacket[3] == 0x00)
    {
      return 1; //nak
    }
    else if ((millis() - ackWait) > 1500)
    {
      return 5;
    }
  }

  for (i = 2; i < 8 ;i++)
  {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  if (msgID1 == ackPacket[6] && msgID2 == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9])
  {
    return 10;
  }
  else
  {
    return 1;
  }
}
