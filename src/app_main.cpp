/*************************************************************************************************************************************************
Very loosly based on this: https://www.bitsnblobs.com/time-lapse-camera-using-the-esp32-cam

Author name: Radu Privantu
License: Apache License 2.0

*************************************************************************************************************************************************/


  /********************************************************************************************************************
 *  Board Settings:
 *  Board: "ESP32 Cam AI Thinker"
 *  Upload Speed: "921600"
 *  Flash Frequency: "80MHz"
 *  Flash Mode: "QIO"
 *  Partition Scheme: "Hue APP (3MB No OTA/1MB SPIFFS)"
 *  Core Debug Level: "None"
 *  COM Port: Depends *On Your System*
 *********************************************************************************************************************/

#include "esp_camera.h"
#include "FS.h"
#include "SPI.h"
#include "driver/i2c.h"
#include <SD.h>
#include "SD_MMC.h"
#include "driver/rtc_io.h"
#include <WiFi.h>
#include <ESP32_FTPClient.h>
#include <WiFiManager.h>
#include <time.h>
#include "esp_sntp.h"

//!!DONT FORGET TO UPDATE Secrets.h with your WIFI Credentials!!
char ftp_server[] = "192.168.1.199";
char ftp_user[]   = "amit";
char ftp_pass[]   = "amit";
const char* sdcardPhotoDir = "/Photos";
const char* ftpDir = "files";

///RTC Time Setting
const char* ntpServer = "192.168.1.199";
const long gmtOffset_sec = -28800;
const int daylightOffset_sec = 3600;

// you can pass a FTP timeout and debbug mode on the last 2 arguments
ESP32_FTPClient ftp (ftp_server,ftp_user,ftp_pass, 10000, 0); // Disable Debug to increase Tx Speed

RTC_DATA_ATTR int cur_pic = 0;


int debug = 0;
bool debugSdcard = false;

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"



#define NIGHT_TIME_TO_SLEEP  30            //night time ESP32 will go to sleep (in seconds)
#define DAY_TIME_TO_SLEEP  900            //day time ESP32 will go to sleep (in seconds)
#define uS_TO_S_FACTOR 1000000ULL   //conversion factor for micro seconds to seconds */

#define OV2640_MAXLEVEL_SHARPNESS 6

typedef unsigned char u8;
uint16_t nextImageNumber = 0;
int max_retry_count=10;
int light=0;
int day_switch_value=45;
bool enable_ftp=true;

void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void deleteFile(fs::FS &fs, const char * path);
void start_ftp_process();
void readAndSendBigBinFile(fs::FS& fs,const char* filename, String fullpath, ESP32_FTPClient ftpClient);
void createDir(fs::FS &fs, const char * path);
void logSDCard(const char * dataMessage);
void setupLogfile();
void appendFile(fs::FS &fs, const char * path, const char * message);
void writeFile(fs::FS &fs, const char * path, const char * message);
bool initConfigFile();
bool updateConfigFile();
void printLocalTime();
String gettime();
int change_sharpness( int sharpness );


const static u8 OV2640_SHARPNESS_AUTO[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0x20, 0x20,
0x00, 0x00, 0x00
};

const static u8 OV2640_SHARPNESS_MANUAL[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0x00, 0x20,
0x00, 0x00, 0x00
};

const static u8 OV2640_SHARPNESS_LEVEL0[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xc0, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL1[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xc1, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL2[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xc2, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL3[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xc4, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL4[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xc8, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL5[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xd0, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL6[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xdf, 0x1f,
0x00, 0x00, 0x00
};

const static u8 *OV_SETTING_SHARPNESS[]=
{
OV2640_SHARPNESS_LEVEL0,
OV2640_SHARPNESS_LEVEL1,
OV2640_SHARPNESS_LEVEL2,
OV2640_SHARPNESS_LEVEL3,
OV2640_SHARPNESS_LEVEL4,
OV2640_SHARPNESS_LEVEL5,
OV2640_SHARPNESS_LEVEL6
};

static int table_mask_write(const u8* ptab)
{
u8 address;
u8 value,orgval;
u8 mask;
const u8 *pdata=ptab;

if ( NULL==pdata )   
    return -1;

sensor_t * s = esp_camera_sensor_get();

while(1)   
{   
    address =*pdata++;   
    value = *pdata++;   
    mask = *pdata++;           
    if ( (0==address) && (0==value) &&(0==mask) )   
    {   
        break;   
    }   
    
    s->set_reg(s,address,mask,value);
}   

return 0;   

}

int change_sharpness( int sharpness )
{
if ( sharpness > OV2640_MAXLEVEL_SHARPNESS)
{
return -1;
}

if( sharpness <0 )   
{   
    table_mask_write(OV2640_SHARPNESS_AUTO);       
}   
else   
{   
    table_mask_write(OV2640_SHARPNESS_MANUAL);     
    table_mask_write(OV_SETTING_SHARPNESS[sharpness]);   
}      

return 0;   

}


String gettime(){
  struct tm timeinfo;
  

  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return "";
  }
  char currentTime[15];
  strftime(currentTime,15, "%d%m%Y%H%M%S", &timeinfo);

  Serial.println("currentTime "+String(currentTime));

  return String(currentTime);
}


void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
      Serial.print("Connecting to ");
      Serial.println("Cudy-24G");
      WiFi.begin("Cudy-24G", "");
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.println(WiFi.localIP());
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer,ntpServer,ntpServer);
      sntp_sync_status_t syncStatus = sntp_get_sync_status();
      if (syncStatus != SNTP_SYNC_STATUS_COMPLETED) {
          delay(5000); // Adjust the delay time as per your requirements
          syncStatus = sntp_get_sync_status();
          Serial.print("syncStatus ");
          Serial.println(syncStatus);
      }
      sntp_stop();  //resets syncStatus to SNTP_SYNC_STATUS_RESET for the next time I initiate 
    
    
  } else {
    
  Serial.println(&timeinfo, "%A, %B %d %m %Y %H:%M:%S");
  }
    //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();
  
}

// Write the sensor readings on the SD card
void logSDCard(const char * dataMessage) {
  //Serial.print("Save data: ");
  //Serial.println(dataMessage);
  if (debugSdcard) appendFile(SD_MMC, "/esplog.txt", dataMessage);
}

void setupLogfile(){
    // Create a file on the SD card and write the data labels
  File logfile = SD_MMC.open("/esplog.txt");
  if(!logfile) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD_MMC, "/esplog.txt", "Log generated \r\n");
    debugSdcard=true;
  }
  else {
    Serial.println("File already exists");  
    debugSdcard=true;
  }
  logfile.close();
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File logfile = fs.open(path, FILE_WRITE);
  if(!logfile) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(logfile.println(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  logfile.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  //Serial.printf("Appending to file: %s\n", path);

  File logfile = fs.open(path, FILE_APPEND);
  if(!logfile) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(logfile.println(message)) {
    //Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
    debugSdcard=false;
  }
  logfile.close();
}


void setup() 
{
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  //setCpuFrequencyMhz(240);
  //# Set to the new timezone we want & convert it
  setenv("TZ","PST8PDT",1);
  tzset();
  
  Serial.begin(115200);
  delay(1200);
  printLocalTime();
  
  
  

  #if defined(CAMERA_MODEL_AI_THINKER)
    pinMode(LED_OUT_PIN, OUTPUT);
    digitalWrite(LED_OUT_PIN, LOW);
  #endif

  

  //initialize & mount SD card
  if(!SD_MMC.begin("/sdcard",true))
  {
    Serial.println("Card Mount Failed");
    delay(10000);
    ESP.restart();
    
  } else {
    setupLogfile();
    if (! initConfigFile()) Serial.println("Unable to initialise config file");
    createDir(SD_MMC,sdcardPhotoDir); //Create directory for saving images
  }

uint8_t cardType = SD_MMC.cardType();

  if(cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    logSDCard("No SD card attached"); 
    return;
  }

  Serial.println();
  Serial.print("Booting... at ");
  Serial.println(millis());
  logSDCard("Booting... at ");
  logSDCard(String(millis()).c_str());

  //pinMode(4, INPUT);              //GPIO for LED flash
  //digitalWrite(4, LOW);
  //rtc_gpio_hold_dis(GPIO_NUM_4);  //diable pin hold if it was enabled before sleeping
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz =   10000000;
  
  config.pixel_format = PIXFORMAT_JPEG;
  
  //init with high specs to pre-allocate larger buffers
  if(psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 16;
    config.fb_count = 2;
  } else 
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }




  
  //initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) 
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    logSDCard("Camera init failed with error "+err);
    delay(10000);
    
    esp_sleep_enable_timer_wakeup(NIGHT_TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
    
    
  } else {
    digitalWrite(LED_OUT_PIN, HIGH);   
    delay(1200);
    
  }

camera_fb_t * fb = NULL;
sensor_t * s = esp_camera_sensor_get();


  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 2);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  //s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
 // s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  //s->set_ae_level(s, 2);       // -2 to 2
  //s->set_aec_value(s, 1200);    // 0 to 1200
  s->set_gain_ctrl(s, 0);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)6);  // 0 to 6
  s->set_bpc(s, 1);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 0);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 0);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable    
 
    
   s->set_reg(s,0xff,0xff,0x01);//banksel   
   light=s->get_reg(s,0x2f,0xff);
   Serial.print("First light is ");
   Serial.println(light);
   logSDCard("First light is ");
   logSDCard(String(light).c_str());



    if(light<day_switch_value)
    {
      //here we are in night mode
      if(light<45)s->set_reg(s,0x11,0xff,1);//frame rate (1 means longer exposure)
      s->set_reg(s,0x13,0xff,0);//manual everything
      s->set_reg(s,0x0c,0x6,0x8);//manual banding
           
      s->set_reg(s,0x45,0x3f,0x3f);//really long exposure (but it doesn't really work)

         
        if(light==0)
          {
            s->set_reg(s,0x47,0xff,0x40);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0xf0);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust
          }
          else if(light==1)
          {
            s->set_reg(s,0x47,0xff,0x40);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0xd0);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust
          }
          else if(light==2)
          {
            s->set_reg(s,0x47,0xff,0x40);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0xb0);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust  
          }
          else if(light==3)
          {
            s->set_reg(s,0x47,0xff,0x40);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x70);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust      
          }    
          else if(light==4)
          {
            s->set_reg(s,0x47,0xff,0x40);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x40);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust
          }
          else if(light==5)
          {
            s->set_reg(s,0x47,0xff,0x20);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x80);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }
          else if(light==6)
          {
            s->set_reg(s,0x47,0xff,0x20);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x40);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }         
          else if(light==7)
          {
            s->set_reg(s,0x47,0xff,0x20);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x30);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }
          else if(light==8)
          {
            s->set_reg(s,0x47,0xff,0x20);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x20);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }     
          else if(light==9)
          {
            s->set_reg(s,0x47,0xff,0x20);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x10);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }    
          else if(light==10)
          {
            s->set_reg(s,0x47,0xff,0x10);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x70);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }
          else if(light<=12)
          {
            s->set_reg(s,0x47,0xff,0x10);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x60);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }
          else if(light<=14)
          {
            s->set_reg(s,0x47,0xff,0x10);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x40);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }          
          else if(light<=18)
          {
            s->set_reg(s,0x47,0xff,0x08);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0xb0);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }
          else if(light<=20)
          {
            s->set_reg(s,0x47,0xff,0x08);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x80);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }      
          else if(light<=23)
          {
            s->set_reg(s,0x47,0xff,0x08);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x60);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }           
          else if(light<=27)
          {
            s->set_reg(s,0x47,0xff,0x04);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0xd0);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }
          else if(light<=31)
          {
            s->set_reg(s,0x47,0xff,0x04);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x80);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }     
          else if(light<=35)
          {
            s->set_reg(s,0x47,0xff,0x04);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x60);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }    
          else if(light<=40)
          {
            s->set_reg(s,0x47,0xff,0x02);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x70);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }
          else if(light<45)
          {
            s->set_reg(s,0x47,0xff,0x02);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0x40);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }
          //after this the frame rate is higher, so we need to compensate
          else if(light<50)
          {
            s->set_reg(s,0x47,0xff,0x04);//Frame Length Adjustment MSBs
            s->set_reg(s,0x2a,0xf0,0xa0);//line adjust MSB
            s->set_reg(s,0x2b,0xff,0xff);//line adjust        
          }  
          if(light<day_switch_value)s->set_reg(s,0x43,0xff,0x40);//magic value to give us the frame faster (bit 6 must be 1)
          change_sharpness(2); 
            
       

    }
    else
    {
      esp_camera_deinit();
      //initialize camera
      esp_err_t err = esp_camera_init(&config);
      if (err != ESP_OK) 
      {
        Serial.printf(" 2nd time Camera init failed with error 0x%x", err);
        logSDCard(" 2nd time Camera init failed with error "+err);        
        delay(10000);
        esp_sleep_enable_timer_wakeup(NIGHT_TIME_TO_SLEEP * uS_TO_S_FACTOR);
        esp_deep_sleep_start();
        
        
      } else {
        digitalWrite(LED_OUT_PIN, HIGH);     
        s = esp_camera_sensor_get();
        s->set_wb_mode(s, 0); 
      }
         
    }
 
                  
    //skipping initial 2 frames
        delay(1200);
        Serial.println("Getting first frame at");
        Serial.println(millis());    
        logSDCard("Getting first frame at ");
        logSDCard(String(millis()).c_str());
        if(fb)esp_camera_fb_return(fb);
        fb = esp_camera_fb_get();
        if(fb)esp_camera_fb_return(fb);
        fb = esp_camera_fb_get();
        Serial.println("Got first frame at");
        Serial.println(millis());   
        logSDCard("Got first frame at "); 
        logSDCard(String(millis()).c_str());
        Serial.println("Frame size "+String(fb->len));
        
        if(fb)esp_camera_fb_return(fb);

  
  

  //delay(1200); //wait before reading the frame
 
  // 1st Frame
  fb = nullptr; //clearing up memory
  
  fb = esp_camera_fb_get();
  int retries=0;
  if(!fb)
  while(1)
  {
    Serial.println("Not having image yet, waiting a bit");
    logSDCard("Not having image yet, waiting a bit"); 
    delay(500);
    fb = esp_camera_fb_get();
    if(fb)break;

    retries++;
    if(retries>max_retry_count)break;
  }
   
    if (light < day_switch_value) {
//since we got the frame buffer, we reset the sensor and put it to sleep while saving the file
    s->set_reg(s,0xff,0xff,0x01);//banksel
    s->set_reg(s,0x12,0xff,0x80);//reset (we do this to clear the sensor registries, it seems to get more consistent images this way)
    delay(1000);
    s->set_reg(s,0x09,0x10,0x10);//stand by
    delay(1000);
    }

  char path[128];
  String fullPath = sdcardPhotoDir;
  
  if(debug)sprintf(path,"/%05i_%i_%i.jpg",cur_pic,millis(),cur_pic);
  else sprintf(path,"/%05i_%i_%s.jpg",cur_pic,light,gettime().c_str());

  fullPath.concat(path);

  fs::FS &fs = SD_MMC;

  
  if(fb) {

  //create new file
  File file = fs.open(fullPath, FILE_WRITE);
  if(!file)
  {
    Serial.println("Failed to create file");
    Serial.println("Exiting now"); 
    logSDCard("Failed to create file Exiting now"); 
    //while(1);   //wait here as something is not right    
  } else {
  
  file.write(fb->buf, fb->len);  
  file.close();
  Serial.print("Saving first file as ");
  Serial.println(fullPath);
  logSDCard("Saving first file as "); 
  logSDCard(fullPath.c_str());
  Serial.print("File size is ");
  Serial.println(String(fb->len));
  logSDCard("File size is "); 
  logSDCard(String(fb->len).c_str());

  cur_pic++;
  if (! updateConfigFile()) Serial.println("Unable to update config file");
  }
  } else {

    Serial.println("Unable to get the image from camera.");
    logSDCard("Unable to get the image from camera.");

  }

  //pinMode(4, OUTPUT);              //GPIO for LED flash
  //digitalWrite(4, LOW);            //turn OFF flash LED
  //rtc_gpio_hold_en(GPIO_NUM_4);    //make sure flash is held LOW in sleep  
  


  if(enable_ftp && light>day_switch_value && (cur_pic%10) == 0)
    {
      //here we are in daylight mode
      //Not use for taking night sky picture, so switching off camera and switching on Wifi and transfer all photo to raspberry
      start_ftp_process();
      
      Serial.println("Entering deep sleep mode at after ftp ");
      Serial.println(millis());
      logSDCard("Entering deep sleep mode at after ftp  ");
      logSDCard(String(millis()).c_str()); 
      Serial.flush(); 
      esp_sleep_enable_timer_wakeup(NIGHT_TIME_TO_SLEEP * uS_TO_S_FACTOR);
      esp_deep_sleep_start();     
    
    } else {
        Serial.println("Entering deep sleep mode at ");
        Serial.println(millis());
        logSDCard("Entering deep sleep mode at "); 
        logSDCard(String(millis()).c_str());
        Serial.flush(); 
        esp_sleep_enable_timer_wakeup(NIGHT_TIME_TO_SLEEP * uS_TO_S_FACTOR);
        esp_deep_sleep_start();
    }
    
}

void loop() 
{


}

void start_ftp_process(){
  
  WiFi.mode(WIFI_MODE_STA);
  delay(500);
  WiFi.begin( "Cudy-24G", "" );
  
  Serial.println("Connecting Wifi...");
  logSDCard("Connecting Wifi..."); 
  int retries=0;
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      //logSDCard("."); 
      retries++;
      if(retries>20) ESP.restart();
  }
  Serial.println("");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  logSDCard("IP address: "); 
  logSDCard(WiFi.localIP().toString().c_str());

  fs::FS &fs = SD_MMC;
 
  // Display Card Size
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);


  ftp.OpenConnection();

  if (ftp.isConnected()) {

    logSDCard("FTP Connection Successful!");

  // Using File Upload Types:
  // ftp.InitFile("Type A") [ASCII]    and      ftp.InitFile("Type I")    [IMAGE/BINARY]
  // If you can read the whole file in notepad, then type A can be used....
  // Other wise use Type I
  ftp.ChangeWorkDir(ftpDir);
  
  // Transfer a file from our SD Card in Binary Mode, which is too big for memory

  Serial.printf("Listing directory: %s\n", sdcardPhotoDir);
  
  logSDCard("Listing directory: "); 
  logSDCard(sdcardPhotoDir); 

  File root = fs.open(sdcardPhotoDir);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());      
    } else if (String(file.name()).endsWith("jpg") ){
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
      String fullPath = sdcardPhotoDir;
      fullPath.concat("/"+String(file.name()));
      //Serial.printf("Reading file: %s\n", fullPath);
      readAndSendBigBinFile(SD_MMC, file.name(),fullPath, ftp);                 // 272,546     
      //fs.remove(fullPath);
    } else {
      logSDCard("File not valid for ftp.");
      logSDCard(String(file.name()).c_str()); 
    }
    if(!ftp.isConnected()){
     break;
    }
     file = root.openNextFile();
  }

if(ftp.isConnected()){
  //transfer log file also
  File logfile = fs.open("/esplog.txt");
  readAndSendBigBinFile(SD_MMC, logfile.name(),"/esplog.txt", ftp);
  ftp.CloseConnection();
}

  } else {
          Serial.println("FTP Connection Failure!");
          logSDCard("FTP Connection Failure!");
         }

 
}


// ReadFile Example from ESP32 SD_MMC Library within Core\Libraries
// Changed to also write the output to an FTP Stream
void readAndSendBigBinFile(fs::FS& fs,const char* filename, String fullpath, ESP32_FTPClient ftpClient) {
    ftpClient.InitFile("Type I");
    ftpClient.NewFile(filename);
    
    

    File file = fs.open(fullpath);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.println("Read from file: "+fullpath);

    logSDCard("Read from file: "); 
    logSDCard(fullpath.c_str());

    while (file.available()) {
        // Create and fill a buffer
        unsigned char buf[512];
        int readVal = file.read(buf, sizeof(buf));
        ftpClient.WriteData(buf,sizeof(buf));
    }
    ftpClient.CloseFile();
    file.close();
    if(ftp.isConnected()){
    //ftp.OpenConnection();
    fs.rename(fullpath,fullpath+".txd"); //rename the file which were successfully transferred
     }
    
    
}

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
     file = root.openNextFile();
  }
}

bool initConfigFile(){
   File myFile;
   if (!SD_MMC.exists("/timelapse.cfg")) {
    Serial.println(F("/timelapse.cfg doesn't exist. Creating /timelapse.cfg file..."));
    // create a new file by opening a new file and immediately close it
    myFile = SD_MMC.open("/timelapse.cfg", FILE_WRITE);
    if (myFile) {
                  myFile.println("filecnt:"+String(cur_pic));
                  myFile.close();
                } else {
                  Serial.print(F("SD Card: Issue encountered while attempting to open the file timelapse.cfg"));
                  return false;
                }
   
  } else {
    myFile = SD_MMC.open("/timelapse.cfg", FILE_READ);
    Serial.println(F("Reading timelapse.cfg ..."));
     if (myFile) {
                    while (myFile.available()) {
                              String line = myFile.readStringUntil('\n');
                              Serial.println(line);
                              //if line contains config parameter, then use to initialise filecnt, else initialise with 1
                              if(line.startsWith("filecnt")){
                                //get last filecnt value
                                line.replace("filecnt:","");
                                cur_pic = line.toInt();
                              }
                    }
                    myFile.close();
                 } else {
                    Serial.print(F("SD Card: Issue encountered while attempting to open the file /timelapse.cfg"));
                    return false;
                    }

  }

  return true;

}

bool updateConfigFile(){
   File myFile;
   if (!SD_MMC.exists("/timelapse.cfg")) {
    Serial.println(F("/timelapse.cfg doesn't exist. Can't update timelapse.cfg file..."));
    return false;
   
  } else {
    myFile = SD_MMC.open("/timelapse.cfg", FILE_WRITE);

     if (myFile) {
                  myFile.seek(0);
                  myFile.println("filecnt:"+String(cur_pic));
                  myFile.close();
                 } 
    else {
                 Serial.print(F("SD Card: Issue encountered while attempting to open the file /timelapse.cfg"));
                 return false;
          }

  }

  return true;

}