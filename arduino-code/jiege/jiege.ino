#include <Wire.h>
#include <ArduinoJson.h>
#include <Servo.h>            //舵机的库文件
#include <SPI.h>
#include <Adafruit_BMP280.h>  //BMP280的库文件

//User Modified Part
#define wifi_ssid     "gakki"    
#define wifi_psw      "wangjun08"     
#define clientIDstr   "test001"
#define timestamp     "998"
#define ProductKey    "a1cmPoy6mUu"
#define DeviceName    "test001"
#define DeviceSecret  "59062028d208010dfe52692518ee564a"
#define password      "FFE074EC7099536F8393A52017BEDFBE6B967437"



//Logic Preset
#define OFF           0
#define ON            1
#define MUTE          2
#define KEEP_OFF      2
#define KEEP_ON       3

#define AC_ON   digitalWrite(ACPin,HIGH)
#define AC_OFF  digitalWrite(ACPin,LOW)

#define Fan_ON      digitalWrite(FanPin,HIGH)
#define Fan_OFF     digitalWrite(FanPin,LOW)

#define Buzzer_ON   digitalWrite(BuzzerPin,HIGH)
#define Buzzer_OFF  digitalWrite(BuzzerPin,LOW)

#define Pump_ON     digitalWrite(PumpPin,HIGH)
#define Pump_OFF    digitalWrite(PumpPin,LOW)


//ATcmd Format
#define AT                    "AT\r"
#define AT_OK                 "OK"
#define AT_REBOOT             "AT+REBOOT\r"
#define AT_ECHO_OFF           "AT+UARTE=OFF\r"
#define AT_MSG_ON             "AT+WEVENT=ON\r"

#define AT_WIFI_START         "AT+WJAP=%s,%s\r"
#define AT_WIFI_START_SUCC    "+WEVENT:STATION_UP"

#define AT_MQTT_AUTH          "AT+MQTTAUTH=%s&%s,%s\r"
#define AT_MQTT_CID           "AT+MQTTCID=%s|securemode=3\\,signmethod=hmacsha1\\,timestamp=%s|\r"
#define AT_MQTT_SOCK          "AT+MQTTSOCK=%s.iot-as-mqtt.cn-shanghai.aliyuncs.com,1883\r"

#define AT_MQTT_AUTOSTART_OFF "AT+MQTTAUTOSTART=OFF\r"
#define AT_MQTT_ALIVE         "AT+MQTTKEEPALIVE=500\r"
#define AT_MQTT_START         "AT+MQTTSTART\r"
#define AT_MQTT_START_SUCC    "+MQTTEVENT:CONNECT,SUCCESS"
#define AT_MQTT_PUB_SET       "AT+MQTTPUB=/sys/%s/%s/thing/event/property/post,1\r"
#define AT_MQTT_PUB_ALARM_SET "AT+MQTTPUB=/sys/%s/%s/thing/event/GasAlarm/post,1\r"
#define AT_MQTT_PUB_DATA      "AT+MQTTSEND=%d\r"
#define JSON_DATA_PACK3       "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"RoomTemp\":%d.%02d,\"AC\":%d,\"Fan\":%d,\"Buzzer\":%d,\"GasDetector\":%d}}\r"
#define JSON_DATA_PACK        "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"ColorGreen\":%d,\"ColorBlue\":%d,\"ColorRed\":%d,\"Switch\":%d}}\r"
#define JSON_DATA_PACK_T     "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"RoomTemp\":%d.%02d}}\r"
#define JSON_DATA_PACK_P     "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"Atmosphere\":%d.%02d}}\r"
#define JSON_DATA_PACK_A     "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"Altitude\":%d.%02d}}\r"
#define JSON_DATA_PACK_I     "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"PhotoResistors\":%d}}\r"
#define JSON_DATA_PACK_2      "{\"id\":\"110\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"LightDetector\":%d,\"Curtain\":%d,\"Light\":%d,\"SoilHumi\":%d,\"Pump\":%d,\"eCO2\":%d,\"TVOC\":%d}}\r"
#define JSON_DATA_PACK_ALARM  "{\"id\":\"110\",\"version\":\"1.0\",\"method\":\"thing.event.GasAlarm.post\",\"params\":{\"GasDetector\":%d}}\r"
#define AT_MQTT_PUB_DATA_SUCC "+MQTTEVENT:PUBLISH,SUCCESS"

#define AT_MQTT_UNSUB2         "AT+MQTTUNSUB=3\r"
#define AT_MQTT_SUB2           "AT+MQTTSUB=3,/%s/%s/user/color,1\r"
#define AT_MQTT_SUB_SUCC2      "+MQTTEVENT:3,SUBSCRIBE,SUCCESS"

#define AT_MQTT_UNSUB        "AT+MQTTUNSUB=1\r"
#define AT_MQTT_SUB          "AT+MQTTSUB=1,/sys/%s/%s/thing/service/property/set,1\r"
#define AT_MQTT_SUB_SUCC    "+MQTTEVENT:1,SUBSCRIBE,SUCCESS"

#define AT_BUZZER_MUTE           "\"Buzzer\":2"


#define DEFAULT_TIMEOUT       10   //seconds
#define BUF_LEN               100
#define BUF_LEN_DATA          190

char      ATcmd[BUF_LEN];
char      ATbuffer[BUF_LEN];
char      ATdata[BUF_LEN_DATA];
#define BuzzerPin             3
int   Buzzer = OFF;

String data;
int frequency;
int ColorBlue;  //灯的亮度
int ColorGreen;
int ColorRed;
int IsTemp;     //传感器的打开状况
int IsPressure;
int IsAltitude;
int IsIntensity;
int IsOpen;     //门的打开状况
double RoomTemp;  //温度数据
double Pressure;  //气压
double Altitude;  //海拔
int Intensity;    //光照强度

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

Servo myservo; // create servo object to control a serv
int pos;       //舵机的转动的角度


void setup() {
  //Serial Initial
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(115200);
  
  String inString="";
  pinMode(7,OUTPUT);  //板载LED
  ColorBlue=0;
  ColorGreen=0;
  ColorRed=0;
  IsTemp=0;
  IsPressure=0;
  IsAltitude=0;
  IsIntensity = 0;
  IsOpen=0;
  pos = 90; //每次运行前，调整舵机为90度，即关门状态
  
   if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check the wiring!"));
    while (1);
  }
    /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    
    myservo.attach(A4); // attaches the servo on pin 3 to the servo object
  //Pin Initial
  // Pin_init();
   BEEP(1);
  
  //Cloud Initial
  while(1)
  {
    if(!WiFi_init())continue;
    BEEP(2);
    if(!Ali_connect())continue;
    break;
  }
  BEEP(3);

 
}

void loop() {
  if(Serial2.available()){    //检测串口是否有信息
      int inByte = Serial2.read();    //读取信息
      Serial.println(inByte);
      switch(inByte)                  //根据信息判断不同的运行程序
      {
        case 'a':
          ColorRed = 50; ColorGreen = 0; ColorBlue = 0;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'b':
          ColorRed = 0; ColorGreen = 0; ColorBlue = 0;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'c':
          ColorRed = 0; ColorGreen = 50; ColorBlue = 0;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'd':
          ColorRed = 0; ColorGreen = 0; ColorBlue = 0;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'e':
          ColorRed = 0; ColorGreen = 0; ColorBlue = 50;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'f':
          ColorRed = 0; ColorGreen = 0; ColorBlue = 0;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'g':
          ColorRed *= 2; ColorGreen *= 2; ColorBlue *= 2;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'h':
          ColorRed /= 2; ColorGreen /= 2; ColorBlue /= 2;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'i':
          ColorRed = 50; ColorGreen = 50; ColorBlue = 0;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'j':
          ColorRed = 50; ColorGreen = 0; ColorBlue = 50;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'k':
          ColorRed = 0; ColorGreen = 50; ColorBlue = 50;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'l':
          ColorRed = 50; ColorGreen = 50; ColorBlue = 50;
          analogWrite(9, ColorBlue);
          analogWrite(8, ColorGreen);
          analogWrite(7, ColorRed);
          break;
        case 'm':
          if(IsTemp == 1) Serial.println("已在测量！");
          else {
            IsTemp = 1;
          }
          RoomTemp = bmp.readTemperature();
          Serial.print(F("RoomTemp = "));
          Serial.print(RoomTemp);
          Serial.println(" *C");
          break;
        case 'n':
          if(IsPressure == 1) Serial.println("已在测量！");
          else {
            IsPressure = 1;
          }
          Pressure = bmp.readPressure()/1000;
          Serial.print(F("Pressure = "));
          Serial.print(Pressure);
          Serial.println(" kPa");
          break;
        case 'o':
          if(IsAltitude == 1) Serial.println("已在测量！");
          else {
            IsAltitude = 1;
          }
          Altitude = bmp.readAltitude(1013.25); 
          Serial.print(F("Approx altitude = "));
          Serial.print(Altitude); 
          Serial.println(" m");
          break;
        case 'r':
          if(IsIntensity == 1) Serial.println("已在测量！");
          else {
            IsIntensity = 1;
          }
          Intensity = 1024 - analogRead(A2); 
          Serial.print(F("Light Intensity = "));
          Serial.print(Intensity); 
          Serial.println(" m");
          break;
        case 'p':
           if(IsOpen == 1) Serial.println("门已打开！");
           else{
             IsOpen = 1;
             for (; pos >= 0; pos -= 1) { // goes from 0 degrees to 90 degrees
                // in steps of 1 degree
                myservo.write(pos); // tell servo to go to position in variable 'pos'
                delay(15);  // waits 15ms for the servo to reach the position
             }
           }
            break;
        case 'q':
           if(IsOpen == 0) Serial.println("门已关闭！");
           else{
             IsOpen = 0;
             for (; pos <= 90; pos += 1) { // goes from 90 degrees to 0 degrees
                myservo.write(pos); // tell servo to go to position in variable 'pos'
                delay(15); // waits 15ms for the servo to reach the position
             }
           }
           break;
        case 's':
           IsTemp = IsPressure = IsAltitude = IsIntensity = 0;
           break;
           
      }
      
      Serial.println(inByte);    //串口显示读取到的信息，以便观察数据变化
    }
    Upload();
  //MsgReceive
 // if(check_send_cmd(AT,AT_BUZZER_MUTE,DEFAULT_TIMEOUT))Buzzer_mute();
}

bool Upload()
{
  bool flag;
  int inte1,frac1;
  int inte12,frac12;
  int inte13,frac13;
  int len;
 
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
   
  
  cleanBuffer(ATdata,BUF_LEN_DATA);

  if(IsTemp) {
       inte1 = (int)(RoomTemp);
       frac1 = (RoomTemp - inte1) * 100;
 
       cleanBuffer(ATdata,BUF_LEN_DATA);
       len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_T,inte1,frac1);
       cleanBuffer(ATcmd,BUF_LEN);
       snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
       flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
       if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
   }else {
       cleanBuffer(ATdata,BUF_LEN_DATA);
       len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_T,0,0);
       cleanBuffer(ATcmd,BUF_LEN);
       snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
       flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
       if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
   }
  if(IsPressure) {
        inte12 = (int)(Pressure);
        frac12 = (Pressure - inte12) * 100;
     
        cleanBuffer(ATdata,BUF_LEN_DATA);
        len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_P,inte12,frac12);
        cleanBuffer(ATcmd,BUF_LEN);
        snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
        flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
        if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
    }else {
       cleanBuffer(ATdata,BUF_LEN_DATA);
       len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_P,0,0);
       cleanBuffer(ATcmd,BUF_LEN);
       snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
       flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
       if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
   }
  if(IsAltitude) {
        inte13 = (int)(Altitude);
        frac13 = (Altitude - inte13) * 100;
        
        cleanBuffer(ATdata,BUF_LEN_DATA);
        len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_A,inte13,frac13);
        cleanBuffer(ATcmd,BUF_LEN);
        snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
        flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
        if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
    }else {
       cleanBuffer(ATdata,BUF_LEN_DATA);
       len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_A,0,0);
       cleanBuffer(ATcmd,BUF_LEN);
       snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
       flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
       if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
   }
  if(IsIntensity) {
        cleanBuffer(ATdata,BUF_LEN_DATA);
        len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_I,Intensity);
        cleanBuffer(ATcmd,BUF_LEN);
        snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
        flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
        if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
    }else {
       cleanBuffer(ATdata,BUF_LEN_DATA);
       len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_I,0);
       cleanBuffer(ATcmd,BUF_LEN);
       snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
       flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
       if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
   }
  
  cleanBuffer(ATdata,BUF_LEN_DATA);
  len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK,ColorGreen,ColorBlue,ColorRed,IsOpen);
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
 
 
  return flag;
}

bool Ali_connect()
{
  bool flag;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_AUTH,DeviceName,ProductKey,password);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_CID,clientIDstr,timestamp);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_SOCK,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_AUTOSTART_OFF,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_ALIVE,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_START,AT_MQTT_START_SUCC,20);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  //flag = check_send_cmd(AT_MQTT_UNSUB,AT_OK,DEFAULT_TIMEOUT);
  //if(!flag)return false;
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_SUB,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_MQTT_SUB_SUCC,DEFAULT_TIMEOUT);
  if(!flag)BEEP(4);
  return flag;
}

bool WiFi_init()
{
  bool flag;

  flag = check_send_cmd(AT,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;
  
  flag = check_send_cmd(AT_REBOOT,AT_OK,20);
  if(!flag)return false;
  delay(5000);

  flag = check_send_cmd(AT_ECHO_OFF,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MSG_ON,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_WIFI_START,wifi_ssid,wifi_psw);
  flag = check_send_cmd(ATcmd,AT_WIFI_START_SUCC,20);
  return flag;
}

bool check_send_cmd(const char* cmd,const char* resp,unsigned int timeout)
{
  int i = 0;
  unsigned long timeStart;
  timeStart = millis();
  cleanBuffer(ATbuffer,BUF_LEN);
  Serial3.print(cmd);
  Serial3.flush();
  while(1)
  {
    while(Serial3.available())
    {
      ATbuffer[i++] = Serial3.read();
      if(i >= BUF_LEN)i = 0;
    }
    if(NULL != strstr(ATbuffer,resp))break;
    if((unsigned long)(millis() - timeStart > timeout * 1000)) break;
  }
  
  if(NULL != strstr(ATbuffer,resp))return true;
  return false;
}

void cleanBuffer(char *buf,int len)
{
  for(int i = 0;i < len;i++)
  {
    buf[i] = '\0';
  } 
}

/*
void Pin_init()
{
  pinMode(ACPin,OUTPUT);
  digitalWrite(ACPin,LOW);
  pinMode(BuzzerPin,OUTPUT);
  digitalWrite(BuzzerPin,LOW);
  pinMode(PumpPin,OUTPUT);
  digitalWrite(PumpPin,LOW);
  pinMode(CurtainOpenPin,OUTPUT);
  digitalWrite(CurtainOpenPin,LOW);
  pinMode(CurtainClosePin,OUTPUT);
  digitalWrite(CurtainClosePin,LOW);
  pinMode(Light1Pin,OUTPUT);
  digitalWrite(Light1Pin,LOW);
  pinMode(Light2Pin,OUTPUT);
  digitalWrite(Light2Pin,LOW);
  pinMode(Light3Pin,OUTPUT);
  digitalWrite(Light3Pin,LOW);
  pinMode(FanPin,OUTPUT);
  digitalWrite(FanPin,LOW);
  Curtain_ON();
}
*/

void BEEP(int b_time)
{
  for(int i = 1;i <= b_time;i++)
  { 
    digitalWrite(BuzzerPin,HIGH);
    delay(100);
    digitalWrite(BuzzerPin,LOW);
    delay(100);
  }
}
void Buzzer_mute()
{
  Buzzer_OFF;
  Buzzer = MUTE;
}
