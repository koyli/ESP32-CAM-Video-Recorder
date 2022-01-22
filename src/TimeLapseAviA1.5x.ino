#include <dummy.h>

/*

  TimeLapseAvi

  ESP32-CAM Video Recorder

  This program records an AVI video on the SD Card of an ESP32-CAM.

  by James Zahary July 20, 2019  TimeLapseAvi23x.ino
     jamzah.plc@gmail.com

  https://github.com/jameszah/ESP32-CAM-Video-Recorder

    jameszah/ESP32-CAM-Video-Recorder is licensed under the
    GNU General Public License v3.0


*/

static const char vernum[] = "vA1";

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  edit parameters for wifi name, startup parameters in the local file settings.h
#include "settings.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int count_avi = 0;
int count_cam = 0;
int count_loop = 0;
int  new_config = 5;         // this system abandoned !
int  xlength = total_frames_config * capture_interval / 1000;
int repeat = repeat_config;  // repeat_config declared in settings
int total_frames = total_frames_config;
int recording = 0;
int PIRstatus = 0;
int PIRrecording = 0;
int ready = 0;

// eprom stuff v87

#include <EEPROM.h>

struct eprom_data {
  int eprom_good;
  int Internet_Enabled;
  int DeepSleepPir;
  int record_on_reboot;
  int PIRpin;
  int PIRenabled;
  int  framesize;
  int  repeat;
  int  xspeed;
  int  gray;
  int  quality;
  int  capture_interval;
  int  total_frames;
  int  xlength;
  int EnableBOT;

};



//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "esp_camera.h"

// v98x-WiFiMan


#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
bool InternetFailed = false;

int diskspeed = 0;
char fname[130];
int Wait_for_bot = 0;

#include <ESPmDNS.h>




// Time
#include "time.h"

// MicroSD
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include <SD_MMC.h>

long current_millis;
long last_capture_millis = 0;
static esp_err_t cam_err;
static esp_err_t card_err;
char strftime_buf[64];
char strftime_buf2[12];

int file_number = 0;
bool internet_connected = false;
struct tm timeinfo;
time_t now;

char *filename ;
char *stream ;
int newfile = 0;
int frames_so_far = 0;
FILE *myfile;
long bp;
long ap;
long bw;
long aw;
long totalp;
long totalw;
float avgp;
float avgw;
int overtime_count = 0;
unsigned long nothing_cam = 0;
unsigned long nothing_avi = 0;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// GLOBALS
#define BUFFSIZE 512

// global variable used by these pieces

char str[20];
uint16_t n;
uint8_t buf[BUFFSIZE];

static int i = 0;
uint8_t temp = 0, temp_last = 0;
unsigned long fileposition = 0;
uint16_t frame_cnt = 0;
uint16_t remnant = 0;
uint32_t length = 0;
uint32_t startms;
uint32_t elapsedms;
uint32_t uVideoLen = 0;
bool is_header = false;
long bigdelta = 0;
int other_cpu_active = 0;
int skipping = 0;
int skipped = 0;
int bad_jpg = 0;
int extend_jpg = 0;
int normal_jpg = 0;

int fb_max = 12;

camera_fb_t * fb_q[30];
int fb_in = 0;
int fb_out = 0;

camera_fb_t * fb = NULL;

FILE *avifile = NULL;
FILE *idxfile = NULL;


#define AVIOFFSET 240 // AVI main header length

unsigned long movi_size = 0;
unsigned long jpeg_size = 0;
unsigned long idx_offset = 0;

uint8_t zero_buf[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t   dc_buf[4] = {0x30, 0x30, 0x64, 0x63};    // "00dc"
uint8_t avi1_buf[4] = {0x41, 0x56, 0x49, 0x31};    // "AVI1"
uint8_t idx1_buf[4] = {0x69, 0x64, 0x78, 0x31};    // "idx1"

uint8_t  vga_w[2] = {0x80, 0x02}; // 640
uint8_t  vga_h[2] = {0xE0, 0x01}; // 480
uint8_t  cif_w[2] = {0x90, 0x01}; // 400
uint8_t  cif_h[2] = {0x28, 0x01}; // 296
uint8_t svga_w[2] = {0x20, 0x03}; // 800
uint8_t svga_h[2] = {0x58, 0x02}; // 600
uint8_t   hd_w[2] = {0x00, 0x05}; // 1280
uint8_t   hd_h[2] = {0xD0, 0x02}; // 720
uint8_t uxga_w[2] = {0x40, 0x06}; // 1600
uint8_t uxga_h[2] = {0xB0, 0x04}; // 1200


const int avi_header[AVIOFFSET] PROGMEM = {
  0x52, 0x49, 0x46, 0x46, 0xD8, 0x01, 0x0E, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
  0xD0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x4E, 0x46, 0x4F,
  0x10, 0x00, 0x00, 0x00, 0x6A, 0x61, 0x6D, 0x65, 0x73, 0x7A, 0x61, 0x68, 0x61, 0x72, 0x79, 0x20,
  0x76, 0x41, 0x31, 0x20, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// AviWriterTask runs on cpu 1 to write the avi file
//

TaskHandle_t CameraTask, AviWriterTask;
SemaphoreHandle_t baton;
int counter = 0;

void codeForAviWriterTask( void * parameter )
{
  uint32_t ulNotifiedValue;
  Serial.print("aviwriter, core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));

  for (;;) {
    ulNotifiedValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (ulNotifiedValue-- > 0)  {
      make_avi();
      count_avi++;
      delay(1);
    }
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// CameraTask runs on cpu 1 to take pictures and drop them in a queue
//

void codeForCameraTask( void * parameter )
{
  int pic_delay = 0;
  int next = 0;
  long next_run_time = 0;
  Serial.print("camera, core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));

  for (;;) {

    if (other_cpu_active == 1 ) {
      current_millis = millis();
      count_cam++;
      xSemaphoreTake( baton, portMAX_DELAY );

      int q_size = (fb_in + fb_max - fb_out) % fb_max ;

      if ( q_size + 1 == fb_max) {
        xSemaphoreGive( baton );

        Serial.print(" Queue Full, Skipping ... ");  // the queue is full
        skipped++; skipped++;
        skipping = 1;
        next = 3 * capture_interval;

      } else {
        frames_so_far++;
        frame_cnt++;

        fb_in = (fb_in + 1) % fb_max;
        bp = millis();

        //fb_q[fb_in] = esp_camera_fb_get();
        //Serial.print (fb_q[fb_out]->buf[fblen-2],HEX );  Serial.print(":");
        //Serial.print (fb_q[fb_out]->buf[fblen-1],HEX );  //Serial.print(":");

        do {
          fb_q[fb_in] = esp_camera_fb_get();
          int x = fb_q[fb_in]->len;
          int foundffd9 = 0;
          //if (fb_q[fb_in]->buf[x - 1] != 0xD9) {

          for (int j = 1; j <= 1025; j++) {
            if (fb_q[fb_in]->buf[x - j] != 0xD9) {
              // no d9, try next for
            } else {

              //Serial.println("Found a D9");
              if (fb_q[fb_in]->buf[x - j - 1] == 0xFF ) {
                //Serial.print("Found the FFD9, junk is "); Serial.println(j);
                if (j == 1) {
                  normal_jpg++;
                } else {
                  extend_jpg++;
                }
                if (j > 1000) { //  never happens. but > 1 does, usually 400-500
                  Serial.print("Frame "); Serial.print(frames_so_far);
                  Serial.print(", Len = "); Serial.print(x);
                  Serial.print(", Corrent Len = "); Serial.print(x - j + 1);
                  Serial.print(", Extra Bytes = "); Serial.println( j - 1);
                }
                foundffd9 = 1;
                break;
              }
            }
          }

          if (!foundffd9) {
            bad_jpg++;
            Serial.print("Bad jpeg, Len = "); Serial.println(x);
            esp_camera_fb_return(fb_q[fb_in]);

          } else {
            break;
            // count up the useless bytes
          }

        } while (1);

        totalp = totalp - bp + millis();
        pic_delay = millis() - current_millis;
        xSemaphoreGive( baton );
        last_capture_millis = millis();

        if (q_size == 0) {
          if (skipping == 1) {
            Serial.println(" Queue cleared. ");
            skipping = 0;
          }
          next = capture_interval - pic_delay;
          if (next < 2) next = 2;
        } else if (q_size < 2 ) {
          next = capture_interval - pic_delay;
          if (next < 2) next = 2;
        } else if (q_size < 4 ) {
          next =  capture_interval ;
        } else {
          next = 2 * capture_interval;
          skipped++;
          //Serial.print(((fb_in + fb_max - fb_out) % fb_max));
        }
      }

      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(AviWriterTask, &xHigherPriorityTaskWoken);

      delay(next);
      next_run_time = millis() + next;
    } else {
      next_run_time = millis() + capture_interval;
      delay(capture_interval);
    }
  }
  //delay(1);
}


//
// Writes an uint32_t in Big Endian at current file position
//
static void inline print_quartet(unsigned long i, FILE * fd)
{
  uint8_t y[4];
  //uint8_t x[1];

  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;

  size_t i1_err = fwrite(y , 1, 4, fd);

  //x[0] = i % 0x100;
  //size_t i1_err = fwrite(x , 1, 1, fd);
  //i = i >> 8;  x[0] = i % 0x100;
  //size_t i2_err = fwrite(x , 1, 1, fd);
  //i = i >> 8;  x[0] = i % 0x100;
  //size_t i3_err = fwrite(x , 1, 1, fd);
  //i = i >> 8;  x[0] = i % 0x100;
  //size_t i4_err = fwrite(x , 1, 1, fd);
}



char the_page[4000];

char localip[20];
WiFiEventId_t eventID;

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  PIR_ISR - interupt handler for PIR  - starts or extends a video
//
static void IRAM_ATTR PIR_ISR(void* arg) {

  PIRstatus = digitalRead(PIRpin) + digitalRead(PIRpin) + digitalRead(PIRpin) ;
  //Serial.print("PIR Interupt>> "); Serial.println(PIRstatus);

  //do_blink_short();
  if (PIRenabled == 1) {
    if (PIRstatus == 0) {
      if (PIRrecording == 1) {
        // keep recording for 15 more seconds

        if ( (millis() - startms) > (total_frames * capture_interval - 5000)  ) {

          total_frames = total_frames + 10000 / capture_interval ;
          //Serial.print("PIR frames = "); Serial.println(total_frames);
          Serial.print("#");
          //Serial.println("Add another 10 seconds");
        }

      } else {

        if ( recording == 0 && newfile == 0) {

          //start a pir recording with current parameters, except no repeat and 15 seconds
          Serial.println("Start a PIR");
          PIRrecording = 1;
          repeat = 0;
          total_frames = 15000 / capture_interval;
          startms = millis();
          Serial.print("PIR frames = "); Serial.println(total_frames);
          xlength = total_frames * capture_interval / 1000;
          recording = 1;
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
          vTaskNotifyGiveFromISR(AviWriterTask, &xHigherPriorityTaskWoken);
          do_blink();
        }
      }
    }
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// setup some interupts during reboot
//

static void setupinterrupts() {

  pinMode(PIRpin, INPUT_PULLDOWN);

  Serial.print("PIRpin = ");
  for (int i = 0; i < 5; i++) {
    Serial.print( digitalRead(PIRpin) ); Serial.print(", ");
  }
  Serial.println(" ");

  esp_err_t err = gpio_isr_handler_add((gpio_num_t)PIRpin, &PIR_ISR, NULL);

  if (err != ESP_OK) Serial.printf("gpio_isr_handler_add failed (%x)", err);
  gpio_set_intr_type((gpio_num_t)PIRpin, GPIO_INTR_ANYEDGE);

  Serial.println(" ");
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// blink functions - which turn on/off Blinding Disk-Active Light ... gently
//

hw_timer_t * timer = NULL;

// shut off the Blinding Disk-Active Light
void IRAM_ATTR onTimer() {
  ledcWrite( 5, 0);
}

// blink on the Blinding Disk-Active Light for 100 ms, 1/256th intensity
void do_blink() {
  //Serial.println("<<<*** BLINK ***>>>");
  // timer 3, 80 million / 80000 = 1 millisecond, 100 ms
  timer = timerBegin(3, 8000, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100, false);
  timerAlarmEnable(timer);

  // pwm channel 5, 5000 freq, 8 bit resolution, dutycycle 7, gpio 4

  ledcSetup(5, 5000, 8 );
  ledcAttachPin(4, 5);
  ledcWrite( 5, 7);
}

void do_blink_short() {
  //Serial.println("<<<*** blink ***>>>");
  // timer 3, 80 million / 80000 = 1 millisecond, 20 ms
  timer = timerBegin(3, 8000, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 20, false);
  timerAlarmEnable(timer);

  // pwm channel 5, 5000 freq, 8 bit resolution, dutycycle 1, gpio 4

  ledcSetup(5, 5000, 8 );
  ledcAttachPin(4, 5);
  ledcWrite( 5, 1);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// print_ram - debugging function for show heap total and in tasks, loops through priority tasks
//

void print_ram() {
  Serial.println("cam / avi / loop ");
  Serial.print(count_cam); Serial.print(" / ");
  Serial.print(count_avi); Serial.print(" / ");
  Serial.print(count_loop); Serial.println("  ");

  Serial.printf("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("SPIRam Total heap   %d, SPIRam Free Heap   %d\n", ESP.getPsramSize(), ESP.getFreePsram());

  Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
  //Serial.printf(" Flash Size %d, Flash Speed %d\n",ESP.getFlashChipSize(), ESP.getFlashChipSpeed());

  if (ready) {
    Serial.println("Avi Writer / Camera ");
    Serial.print  (uxTaskGetStackHighWaterMark(AviWriterTask));
    Serial.print  (" / "); Serial.print  (uxTaskGetStackHighWaterMark(CameraTask));
  }


  //Serial.printf( "Task Name\tStatus\tPrio\tHWM\tTask\tAffinity\n");
  // char stats_buffer[1024];
  //vTaskList(stats_buffer);
  // vTaskGetRunTimeStats(stats_buffer);
  // Serial.printf("%s\n\n", stats_buffer);
  Serial.println("----");
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// save_photo_dated - just save one picture as a jpg and optioning send to telegram
//

static void save_photo_dated()
{

  Serial.println("Taking a picture for file ...");
  camera_fb_t *fb = esp_camera_fb_get();

  time(&now);
  localtime_r(&now, &timeinfo);

  //delay(2000);

  strftime(strftime_buf2, sizeof(strftime_buf2), "/%Y%m%d", &timeinfo);
  SD_MMC.mkdir(strftime_buf2);

  strftime(strftime_buf, sizeof(strftime_buf), "%F_%H.%M.%S", &timeinfo);

  char jfname[130];

  //v99 - uxga 13, hd 11, svga 9, vga 8, cif 6
  if (framesize == 8) {
    sprintf(jfname, "/sdcard%s/%s_%s_vga_Q%d_I%d_L%d_S%d.jpg", strftime_buf2, devname, strftime_buf, quality, capture_interval, xlength, xspeed);
  } else if (framesize == 9) {
    sprintf(jfname, "/sdcard%s/%s_%s_svga_Q%d_I%d_L%d_S%d.jpg", strftime_buf2, devname,  strftime_buf, quality, capture_interval, xlength, xspeed);
  } else if (framesize == 11) {
    sprintf(jfname, "/sdcard%s/%s_%s_hd_Q%d_I%d_L%d_S%d.jpg", strftime_buf2, devname,  strftime_buf, quality, capture_interval, xlength, xspeed);
  } else if (framesize == 13) {
    sprintf(jfname, "/sdcard%s/%s_%s_uxga_Q%d_I%d_L%d_S%d.jpg", strftime_buf2, devname, strftime_buf, quality, capture_interval, xlength, xspeed);
  } else  if (framesize == 6) {
    sprintf(jfname, "/sdcard%s/%s_%s_cif_Q%d_I%d_L%d_S%d.jpg", strftime_buf2, devname, strftime_buf, quality, capture_interval, xlength, xspeed);
  } else {
    Serial.println("Wrong framesize");
  }

  FILE *file = fopen(jfname, "w");
  //file = fopen(fname, "w");
  if (file != NULL)  {
    size_t err = fwrite(fb->buf, 1, fb->len, file);
    Serial.printf("File saved: %s\n", jfname);
  }  else  {
    Serial.printf("Could not open file: %s\n\n ", jfname);
  }
  fclose(file);


  esp_camera_fb_return(fb);
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  delete_old_stuff() - before every avi, delete oldest day if freespace less than 10%
//
//  code copied from user @gemi254

void delete_old_stuff() {

  Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));

  float full = 1.0 * SD_MMC.usedBytes() / SD_MMC.totalBytes();;
  if (full  <  0.9) {
    Serial.printf("Nothing deleted, %.1f%% disk full\n", 100.0 * full);
  } else {
    Serial.printf("Disk is %.1f%% full ... deleting oldest day\n", 100.0 * full);
    int oldest = 22222222;
    char oldestname[50];

    File f = SD_MMC.open("/");
    if (f.isDirectory()) {

      File file = f.openNextFile();
      while (file) {
        if (file.isDirectory()) {

          char foldname[50];
          strcpy(foldname, file.name());
          //foldname[0] = 0x20;
          int i = atoi(foldname);
          if (i > 20200000 && i < oldest) {
            strcpy (oldestname, file.name());
            oldest = i;
          }
          //Serial.printf("Name is %s, number is %d\n", foldname, i);
        }
        file = f.openNextFile();
      }
      //Serial.printf("Oldest is Name is %s, number is %d\n", oldestname, oldest);
      deleteFolderOrFile(oldestname);
      f.close();
    }
  }
}

void deleteFolderOrFile(const char * val) {
  // Function provided by user @gemi254
  char val_w_slash[60];  //ver A1 for 2.0.2
  sprintf(val_w_slash, "/%s", val);
  Serial.printf("Deleting : %s\n", val_w_slash);
  File f = SD_MMC.open(val_w_slash);
  if (!f) {
    Serial.printf("Failed to open %s\n", val_w_slash);
    return;
  }

  if (f.isDirectory()) {
    File file = f.openNextFile();
    while (file) {
      if (file.isDirectory()) {
        Serial.print("  DIR : ");
        Serial.println(file.name());
      } else {
        char file_w_slash[100];
        sprintf(file_w_slash, "%s/%s", val_w_slash,file.name());
        Serial.print("  FILE: ");
        Serial.print(file_w_slash);
        Serial.print("  SIZE: ");
        Serial.print(file.size());

        if (SD_MMC.remove(file_w_slash)) {
          Serial.println(" deleted.");
        } else {
          Serial.println(" FAILED.");
        }
      }
      file = f.openNextFile();
    }
    f.close();
    //Remove the dir
    if (SD_MMC.rmdir(val_w_slash)) {
      Serial.printf("Dir %s removed\n", val_w_slash);
    } else {
      Serial.println("Remove dir failed");
    }

  } else {
    //Remove the file
    if (SD_MMC.remove(val_w_slash)) {
      Serial.printf("File %s deleted\n", val_w_slash);
    } else {
      Serial.println("Delete failed");
    }
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// setup() runs on cpu 1
//

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "soc/soc.h"
#include "soc/cpu.h"
#include "soc/rtc_cntl_reg.h"

#include "esp_task_wdt.h"

#ifdef CONFIG_BROWNOUT_DET_LVL
#define BROWNOUT_DET_LVL CONFIG_BROWNOUT_DET_LVL
#else
#define BROWNOUT_DET_LVL 5
#endif //CONFIG_BROWNOUT_DET_LVL

#define CONFIG_BROWNOUT_DET_LVL_SEL_5 1

#ifdef include_pir_and_touch

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// print_wakeup_reason - display message after deepsleep wakeup
//

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}
#endif

void do_eprom_read() {

  eprom_data ed;

  long x = millis();
  EEPROM.begin(200);
  EEPROM.get(0, ed);
  Serial.println("Get took " + String(millis() - x));

  if (ed.eprom_good == MagicNumber) {
    Serial.print("Good settings in the EPROM ");
    Serial.println(ed.eprom_good);
    Internet_Enabled = ed.Internet_Enabled; Serial.print("Internet_Enabled "); Serial.println(Internet_Enabled );
    DeepSleepPir  = ed.DeepSleepPir; Serial.print("DeepSleepPir "); Serial.println(DeepSleepPir );
    record_on_reboot = ed.record_on_reboot; Serial.print("record_on_reboot "); Serial.println(record_on_reboot );
    PIRpin = ed.PIRpin; Serial.print("PIRpin "); Serial.println(PIRpin );
    PIRenabled = ed.PIRenabled; Serial.print("PIRenabled "); Serial.println(PIRenabled );
    framesize = ed.framesize; Serial.print("framesize "); Serial.println(framesize );
    repeat_config = ed.repeat; Serial.print("repeat_config "); Serial.println(repeat_config );
    repeat = ed.repeat;
    xspeed = ed.xspeed; Serial.print("xspeed "); Serial.println(xspeed );
    gray = ed.gray; Serial.print("gray "); Serial.println(gray );
    quality = ed.quality; Serial.print("quality "); Serial.println(quality );
    capture_interval = ed.capture_interval; Serial.print("capture_interval "); Serial.println(capture_interval );
    total_frames = ed.total_frames;
    total_frames_config = ed.total_frames; Serial.print("total_frames_config "); Serial.println(total_frames_config );
    xlength = ed.xlength; Serial.print("xlength "); Serial.println(xlength );
    EnableBOT = ed.EnableBOT; Serial.print("EnableBOT "); Serial.println(EnableBOT );
  } else {
    Serial.println("No settings in EPROM - putting in hardcoded settings ");
    do_eprom_write();
  }
}


void do_eprom_write() {

  eprom_data ed;

  Serial.println("Write settings in the EPROM ");
  ed.eprom_good = MagicNumber;
  ed.Internet_Enabled = Internet_Enabled;
  ed.DeepSleepPir  = DeepSleepPir;
  ed.record_on_reboot = record_on_reboot;
  ed.PIRpin = PIRpin;
  ed.PIRenabled = PIRenabled;
  ed.framesize = framesize;
  ed.repeat = repeat_config;
  ed.xspeed = xspeed;
  ed.gray = gray;
  ed.quality = quality;
  ed.capture_interval = capture_interval;
  ed.total_frames = total_frames_config;
  ed.xlength = xlength;
  ed.EnableBOT = EnableBOT;

  Serial.println("Writing to EPROM ...");

  long x = millis();
  EEPROM.begin(200);
  EEPROM.put(0, ed);
  EEPROM.commit();
  EEPROM.end();

  Serial.println("Put took " + String(millis() - x) + " ms, bytes = " + String(sizeof(ed)));
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  setup()  - the Arduino setup
//

void setup() {

  Serial.begin(115200);
  Serial.println("\n\n---");

  rtc_gpio_hold_dis(GPIO_NUM_33);
  pinMode(33, OUTPUT);             // little red led on back of chip
  digitalWrite(33, LOW);           // turn on the red LED on the back of chip

  rtc_gpio_hold_dis(GPIO_NUM_4);
  pinMode(4, OUTPUT);               // Blinding Disk-Avtive Light
  digitalWrite(4, LOW);             // turn off

  Serial.setDebugOutput(true);
  Serial.print("setup, core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));

  // zzz
  Serial.println("                                    ");
  Serial.println("-------------------------------------");
  Serial.printf("ESP-CAM Video Recorder %s\n", vernum);
  Serial.printf(" http://%s.local - to access the camera\n", devname);
  Serial.println("-------------------------------------");

#ifdef include_pir_and_touch
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
#endif

  do_eprom_read();
  repeat = repeat_config;
  total_frames = total_frames_config;

  if (!psramFound()) {
    Serial.println("paraFound wrong - major fail");
    major_fail();
  }

  if (Internet_Enabled) {
    Serial.println("Starting wifi ...");
    if (init_wifi()) { // Connected to WiFi
      internet_connected = true;
    } else {
      Serial.println("Internet skipped");
      internet_connected = false;
    }
  }
  //plm print_ram();  delay(1000);
  Serial.println("Starting sd card ...");

  // SD camera init
  card_err = init_sdcard();
  if (card_err != ESP_OK) {
    Serial.printf("SD Card init failed with error 0x%x", card_err);
    major_fail();
    return;
  }

  Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));

  //plm print_ram();  delay(2000);
  Serial.println("Starting tasks ...");

  baton = xSemaphoreCreateMutex();  // baton controls access to camera and frame queue

  xTaskCreatePinnedToCore(
    codeForCameraTask,
    "CameraTask",
    1024,        // heap available for this task
    NULL,
    2,           // prio 2 - higher number is higher priio
    &CameraTask,
    1);          // core 1

  delay(20);

  xTaskCreatePinnedToCore(
    codeForAviWriterTask,
    "AviWriterTask",
    3072,       // heap
    NULL,
    3,          // prio 3
    &AviWriterTask,
    1);         // on cpu 1 - same as ftp http

  delay(20);

  //plm print_ram();  delay(2000);
  Serial.println("Starting camera ...");

  recording = 0;  // we are NOT recording
  config_camera();

#ifdef include_pir_and_touch
  setupinterrupts();
#endif

  newfile = 0;    // no file is open  // don't fiddle with this!

  recording = record_on_reboot;

  //plm print_ram();  delay(2000);

  ready = 1;
  digitalWrite(33, HIGH);         // red light turns off when setup is complete

  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  xTaskNotifyGive(AviWriterTask);

  delay(1000);

  print_ram();

  if (delete_old_files) delete_old_stuff();
}


//
// if we have no camera, or sd card, then flash rear led on and off to warn the human SOS - SOS
//
void major_fail() {

  Serial.println(" ");

  for  (int i = 0;  i < 10; i++) {                 // 10 loops or about 100 seconds then reboot
    for (int j = 0; j < 3; j++) {
      digitalWrite(33, LOW);   delay(150);
      digitalWrite(33, HIGH);  delay(150);
    }

    delay(1000);

    for (int j = 0; j < 3; j++) {
      digitalWrite(33, LOW);  delay(500);
      digitalWrite(33, HIGH); delay(500);
    }

    delay(1000);
    Serial.print("Major Fail  "); Serial.print(i); Serial.print(" / "); Serial.println(10);
  }

  ESP.restart();
}


bool init_wifi()
{
  int connAttempts = 0;

  Serial.println(" Disable brownout");
  uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG); //save WatchDog register
  Serial.print("\nBrownOut Regsiter was (in hex) "); Serial.println(brown_reg_temp, HEX);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  if (strlen(ssid) == 1) {
  //if (1) {
    WiFiManager wm;
    bool res;
    //wm.resetSettings();  // for debugging

    wm.setConnectTimeout(20); // how long to try to connect for before continuing
    wm.setConfigPortalTimeout(30); // auto close configportal after n seconds

    // res = wm.autoConnect(); // auto generated AP name from chipid

    res = wm.autoConnect(devname); // use the devname defined above, with no password
    //res = wm.autoConnect("AutoConnectAP","password"); // password protected ap

    if (res) {
      Serial.println("Succesful Connection using WiFiManager");

    } else {

      InternetFailed = true;
      Serial.println("Internet failed using WiFiManager - not starting Web services");
    }
  } else {

    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(devname);
    //WiFi.printDiag(Serial);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED ) {
      delay(1000);
      Serial.print(".");
      if (connAttempts == 20 ) {
        Serial.println("Cannot connect - try again");
        WiFi.begin(ssid, password);
      }
      if (connAttempts == 30) {
        Serial.println("Cannot connect - fail");

        WiFi.printDiag(Serial);
        return false;
      }
      connAttempts++;
    }

    Serial.println("\nInternet connected");
  }

  if (!InternetFailed) {
    if (!MDNS.begin(devname)) {
      Serial.println("Error setting up MDNS responder!");
    } else {
      Serial.printf("mDNS responder started '%s'\n", devname);
    }

    configTime(0, 0, "pool.ntp.org");

    setenv("TZ", TIMEZONE, 1);  // mountain time zone from #define at top
    tzset();

    time_t now ;
    timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 15;
    delay(1000);
    time(&now);
    localtime_r(&now, &timeinfo);

    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
      Serial.printf("Waiting for system time to be set... (%d/%d) -- %d\n", retry, retry_count, timeinfo.tm_year);
      delay(1000);
      time(&now);
      localtime_r(&now, &timeinfo);
    }

    Serial.print("Local time: "); Serial.println(ctime(&now));
    sprintf(localip, "%s", WiFi.localIP().toString().c_str());
  }

  //typedef enum {
  //    WIFI_PS_NONE,        /**< No power save */
  //    WIFI_PS_MIN_MODEM,   /**< Minimum modem power saving. In this mode, station wakes up to receive beacon every DTIM period */
  //    WIFI_PS_MAX_MODEM,   /**< Maximum modem power saving. In this mode, interval to receive beacons is determined by the listen_interval
  //                              parameter in wifi_sta_config_t.
  //                              Attention: Using this option may cause ping failures. Not recommended */
  //} wifi_ps_type_t;

  wifi_ps_type_t the_type;

  esp_err_t get_ps = esp_wifi_get_ps(&the_type);
  Serial.printf("The power save was: %d\n", the_type);

  Serial.printf("Set power save to %d\n", WIFI_PS_NONE);
  esp_err_t set_ps = esp_wifi_set_ps(WIFI_PS_NONE);

  esp_err_t new_ps = esp_wifi_get_ps(&the_type);
  Serial.printf("The power save is : %d\n", the_type);

  Serial.println(" Enable brownout");
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector




  return true;
}


static esp_err_t init_sdcard()
{

  //pinMode(12, PULLUP);
  pinMode(13, PULLUP);
  //pinMode(4, OUTPUT);

  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.flags = SDMMC_HOST_FLAG_1BIT;                       // using 1 bit mode
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
  diskspeed = host.max_freq_khz;
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.width = 1;                                   // using 1 bit mode
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 8,
  };

  sdmmc_card_t *card;

  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret == ESP_OK) {
    Serial.println("SD card mount successfully!");
  }  else  {
    Serial.printf("Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
    Serial.println("Try again...");
    delay(5000);
    diskspeed = 400;
    host.max_freq_khz = SDMMC_FREQ_PROBING;
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret == ESP_OK) {
      Serial.println("SD card mount successfully SLOW SLOW SLOW");
    } else {
      Serial.printf("Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
      major_fail();
    }
  }
  sdmmc_card_print_info(stdout, card);
  Serial.print("SD_MMC Begin: "); Serial.println(SD_MMC.begin());   // required by ftp system ??
  return ret;
  //pinMode(13, PULLDOWN);
  //pinMode(13, INPUT_PULLDOWN);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Make the avi move in 4 pieces
//
// make_avi() called in every loop, which calls below, depending on conditions
//   start_avi() - open the file and write headers
//   another_pic_avi() - write one more frame of movie
//   end_avi() - write the final parameters and close the file

void make_avi( ) {

#ifdef include_pir_and_touch

  if (PIRenabled == 1) {

      PIRstatus = digitalRead(PIRpin) + digitalRead(PIRpin) + digitalRead(PIRpin) ;
      if (DeepSleepPir == 1 && millis() < 15000 ) {
          //DeepSleepPir = 0;
          PIRstatus  = 0;
      }
      //Serial.print("Mak>> "); Serial.println(PIRstatus);
      if (PIRstatus == 0) {

      if (PIRrecording == 1) {
        // keep recording for 15 more seconds
        if ( (millis() - startms) > (total_frames * capture_interval - 5000)  ) {

          total_frames = total_frames + 10000 / capture_interval ;
          //Serial.print("Make PIR frames = "); Serial.println(total_frames);
          Serial.print("@");
          //Serial.println("Add another 10 seconds");
        }

      } else {

        if ( recording == 0 && newfile == 0) {

          //start a pir recording with current parameters, except no repeat and 15 seconds
          Serial.println("Start a PIR");
          PIRrecording = 1;
          repeat = 0;
          total_frames = 15000 / capture_interval;
          xlength = total_frames * capture_interval / 1000;
          recording = 1;
        }
      }
    }
  }

#endif

  // we are recording, but no file is open

  if (newfile == 0 && recording == 1) {                                     // open the file

    digitalWrite(33, HIGH);
    newfile = 1;

    Serial.println(" ");

    if (delete_old_files) delete_old_stuff();

    start_avi();                                 // now start the avi

  } else {

    // we have a file open, but not recording

    if (newfile == 1 && recording == 0) {                                  // got command to close file

      digitalWrite(33, LOW);
      end_avi();

      Serial.println("Done capture due to command");

      frames_so_far = total_frames;

      newfile = 0;    // file is closed
      recording = 0;  // DO NOT start another recording
      PIRrecording = 0;

    } else {

      if (newfile == 1 && recording == 1) {                            // regular recording

        if ((millis() - startms) > (total_frames * capture_interval)) {  // time is up, even though we have not done all the frames

          Serial.println (" "); Serial.println("Done capture for time");
          Serial.print("Time Elapsed: "); Serial.print(millis() - startms); Serial.print(" Frames: "); Serial.println(frame_cnt);
          Serial.print("Config:       "); Serial.print(total_frames * capture_interval ) ; Serial.print(" (");
          Serial.print(total_frames); Serial.print(" x "); Serial.print(capture_interval);  Serial.println(")");

          digitalWrite(33, LOW);                                                       // close the file

          end_avi();

          frames_so_far = 0;
          newfile = 0;          // file is closed
          if (repeat > 0) {
            recording = 1;        // start another recording
            repeat = repeat - 1;
            xTaskNotifyGive(AviWriterTask);
          } else {
            recording = 0;
            PIRrecording = 0;
          }

        } else  {                                                            // regular

          another_save_avi();

        }
      }
    }
  }
}

static void config_camera() {

  camera_config_t config;

  //Serial.println("config camera");

  if (new_config == 5) {

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
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    config.frame_size = FRAMESIZE_UXGA;

    //  v99 - lets get rid of this queuing system --- not just yet
    //    fb_max = 6;           //74.5 from 7                      // for vga and uxga
    //    config.jpeg_quality = 6;  //74.5 from 7

    fb_max = 6;           //74.5 from 7                      // for vga and uxga
    config.jpeg_quality = 6;  //74.5 from 7

    config.fb_count = fb_max + 1;

    // camera init
    cam_err = esp_camera_init(&config);
    if (cam_err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", cam_err);
      major_fail();
    }

    new_config = 2;
  }

  delay(100);

  sensor_t * ss = esp_camera_sensor_get();
  ss->set_quality(ss, quality);
  ss->set_framesize(ss, (framesize_t)framesize);
  if (gray == 1) {
    ss->set_special_effect(ss, 2);  // 0 regular, 2 grayscale
  } else {
    ss->set_special_effect(ss, 0);  // 0 regular, 2 grayscale
  }
  ss->set_brightness(ss, 1);  //up the blightness just a bit
  ss->set_saturation(ss, -2); //lower the saturation


  for (int j = 0; j < 5; j++) {
    do_fb();  // start the camera ... warm it up
    delay(50);
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// start_avi - open the files and write in headers
//


static void start_avi() {

  Serial.println("Starting an avi ");

  //plm print_ram();

  //89 config_camera();

  time(&now);
  localtime_r(&now, &timeinfo);

  strftime(strftime_buf2, sizeof(strftime_buf2), "/%Y%m%d", &timeinfo);
  SD_MMC.mkdir(strftime_buf2);

  strftime(strftime_buf, sizeof(strftime_buf), "%F_%H.%M.%S", &timeinfo);

  //v99 - uxga 13, hd 11, svga 9, vga 8, cif 6
  if (framesize == 8) {
    sprintf(fname, "/sdcard%s/%s_%s_vga_Q%d_I%d_L%d_S%d.avi", strftime_buf2, devname, strftime_buf, quality, capture_interval, xlength, xspeed);
  } else if (framesize == 9) {
    sprintf(fname, "/sdcard%s/%s_%s_svga_Q%d_I%d_L%d_S%d.avi", strftime_buf2, devname,  strftime_buf, quality, capture_interval, xlength, xspeed);
  } else if (framesize == 11) {
    sprintf(fname, "/sdcard%s/%s_%s_hd_Q%d_I%d_L%d_S%d.avi", strftime_buf2, devname,  strftime_buf, quality, capture_interval, xlength, xspeed);
  } else if (framesize == 13) {
    sprintf(fname, "/sdcard%s/%s_%s_uxga_Q%d_I%d_L%d_S%d.avi", strftime_buf2, devname, strftime_buf, quality, capture_interval, xlength, xspeed);
  } else  if (framesize == 6) {
    sprintf(fname, "/sdcard%s/%s_%s_cif_Q%d_I%d_L%d_S%d.avi", strftime_buf2, devname, strftime_buf, quality, capture_interval, xlength, xspeed);
  } else {
    Serial.println("Wrong framesize");
  }

  Serial.print("\nFile name will be >"); Serial.print(fname); Serial.println("<");

  avifile = fopen(fname, "w");
  idxfile = fopen("/sdcard/idx.tmp", "w");

  if (avifile != NULL)  {
    //Serial.printf("File open: %s\n", fname);
  }  else  {
    Serial.println("Could not open file");
    major_fail();
  }

  if (idxfile != NULL)  {
    //Serial.printf("File open: %s\n", "/sdcard/idx.tmp");
  }  else  {
    Serial.println("Could not open file");
    major_fail();
  }

  for ( i = 0; i < AVIOFFSET; i++)
  {
    char ch = pgm_read_byte(&avi_header[i]);
    buf[i] = ch;
  }

  size_t err = fwrite(buf, 1, AVIOFFSET, avifile);
  //v99 - uxga 13, hd 11, svga 9, vga 8, cif 6
  if (framesize == 8) {

    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(vga_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(vga_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(vga_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(vga_h, 1, 2, avifile);

  } else if (framesize == 13) {

    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(uxga_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(uxga_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(uxga_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(uxga_h, 1, 2, avifile);

  } else if (framesize == 11) {

    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(hd_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(hd_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(hd_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(hd_h, 1, 2, avifile);


  } else if (framesize == 9) {

    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(svga_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(svga_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(svga_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(svga_h, 1, 2, avifile);

  }  else if (framesize == 6) {

    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(cif_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(cif_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(cif_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(cif_h, 1, 2, avifile);
  }

  fseek(avifile, AVIOFFSET, SEEK_SET);

  Serial.print(F("\nRecording "));
  Serial.print(total_frames);
  Serial.println(F(" video frames ...\n"));

  startms = millis();
  bigdelta = millis();
  totalp = 0;
  totalw = 0;
  overtime_count = 0;
  jpeg_size = 0;
  movi_size = 0;
  uVideoLen = 0;
  idx_offset = 4;


  frame_cnt = 0;
  frames_so_far = 0;

  skipping = 0;
  skipped = 0;
  bad_jpg = 0;
  extend_jpg = 0;
  normal_jpg = 0;

  newfile = 1;
  other_cpu_active = 1;


} // end of start avi

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  another_save_avi runs on cpu 1, saves another frame to the avi file
//
//  the "baton" semaphore makes sure that only one cpu is using the camera subsystem at a time
//

static void another_save_avi() {

  xSemaphoreTake( baton, portMAX_DELAY );

  if (fb_in == fb_out) {        // nothing to do

    xSemaphoreGive( baton );
    nothing_avi++;

  } else {

    fb_out = (fb_out + 1) % fb_max;

    int fblen;
    fblen = fb_q[fb_out]->len;

    //xSemaphoreGive( baton );

    if (BlinkWithWrite) {
      digitalWrite(33, LOW);
    }

    jpeg_size = fblen;
    movi_size += jpeg_size;
    uVideoLen += jpeg_size;

    bw = millis();
    size_t dc_err = fwrite(dc_buf, 1, 4, avifile);
    size_t ze_err = fwrite(zero_buf, 1, 4, avifile);


    int time_to_give_up = 0;
    while (ESP.getFreeHeap() < 35000) {
      Serial.print(time_to_give_up); Serial.print(" Low on heap "); Serial.print(ESP.getFreeHeap());
      Serial.print(" frame q = "); Serial.println((fb_in + fb_max - fb_out) % fb_max);
      if (time_to_give_up++ == 50) break;
      delay(100 + 5 * time_to_give_up);
    }

    size_t err = fwrite(fb_q[fb_out]->buf, 1, fb_q[fb_out]->len, avifile);

    time_to_give_up = 0;
    while (err != fb_q[fb_out]->len) {
      Serial.print("Error on avi write: err = "); Serial.print(err);
      Serial.print(" len = "); Serial.println(fb_q[fb_out]->len);
      time_to_give_up++;
      if (time_to_give_up == 10) major_fail();
      Serial.print(time_to_give_up); Serial.print(" Low on heap !!! "); Serial.println(ESP.getFreeHeap());

      delay(1000);
      size_t err = fwrite(fb_q[fb_out]->buf, 1, fb_q[fb_out]->len, avifile);

    }

    //totalw = totalw + millis() - bw;

    //xSemaphoreTake( baton, portMAX_DELAY );
    esp_camera_fb_return(fb_q[fb_out]);     // release that buffer back to the camera system
    xSemaphoreGive( baton );

    remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;

    print_quartet(idx_offset, idxfile);
    print_quartet(jpeg_size, idxfile);

    idx_offset = idx_offset + jpeg_size + remnant + 8;

    jpeg_size = jpeg_size + remnant;
    movi_size = movi_size + remnant;
    if (remnant > 0) {
      size_t rem_err = fwrite(zero_buf, 1, remnant, avifile);
    }

    fileposition = ftell (avifile);       // Here, we are at end of chunk (after padding)
    fseek(avifile, fileposition - jpeg_size - 4, SEEK_SET);    // Here we are the the 4-bytes blank placeholder

    print_quartet(jpeg_size, avifile);    // Overwrite placeholder with actual frame size (without padding)

    fileposition = ftell (avifile);


    fseek(avifile, fileposition + jpeg_size  , SEEK_SET);

    totalw = totalw + millis() - bw;

    digitalWrite(33, HIGH);

  }
} // end of another_pic_avi

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  end_avi runs on cpu 1, empties the queue of frames, writes the index, and closes the files
//

static void end_avi() {

  unsigned long current_end = 0;

  other_cpu_active = 0 ;  // shuts down the picture taking program

  //Serial.print(" Write Q: "); Serial.print((fb_in + fb_max - fb_out) % fb_max); Serial.print(" in/out  "); Serial.print(fb_in); Serial.print(" / "); Serial.println(fb_out);

  for (int i = 0; i < fb_max; i++) {           // clear the queue
    another_save_avi();
  }

  //Serial.print(" Write Q: "); Serial.print((fb_in + fb_max - fb_out) % fb_max); Serial.print(" in/out  "); Serial.print(fb_in); Serial.print(" / "); Serial.println(fb_out);

  current_end = ftell (avifile);

  Serial.println("End of avi - closing the files");

  elapsedms = millis() - startms;
  float fRealFPS = (1000.0f * (float)frame_cnt) / ((float)elapsedms) * xspeed;
  float fmicroseconds_per_frame = 1000000.0f / fRealFPS;
  uint8_t iAttainedFPS = round(fRealFPS);
  uint32_t us_per_frame = round(fmicroseconds_per_frame);

  //Modify the MJPEG header from the beginning of the file, overwriting various placeholders

  fseek(avifile, 4 , SEEK_SET);
  print_quartet(movi_size + 240 + 16 * frame_cnt + 8 * frame_cnt, avifile);

  fseek(avifile, 0x20 , SEEK_SET);
  print_quartet(us_per_frame, avifile);

  unsigned long max_bytes_per_sec = movi_size * iAttainedFPS / frame_cnt;

  fseek(avifile, 0x24 , SEEK_SET);
  print_quartet(max_bytes_per_sec, avifile);

  fseek(avifile, 0x30 , SEEK_SET);
  print_quartet(frame_cnt, avifile);

  fseek(avifile, 0x8c , SEEK_SET);
  print_quartet(frame_cnt, avifile);

  fseek(avifile, 0x84 , SEEK_SET);
  print_quartet((int)iAttainedFPS, avifile);

  fseek(avifile, 0xe8 , SEEK_SET);
  print_quartet(movi_size + frame_cnt * 8 + 4, avifile);

  Serial.println(F("\n*** Video recorded and saved ***\n"));
  Serial.print(F("Recorded "));
  Serial.print(elapsedms / 1000);
  Serial.print(F("s in "));
  Serial.print(frame_cnt);
  Serial.print(F(" frames\nFile size is "));
  Serial.print(movi_size + 12 * frame_cnt + 4);
  Serial.print(F(" bytes\nActual FPS is "));
  Serial.print(fRealFPS, 2);
  Serial.print(F("\nMax data rate is "));
  Serial.print(max_bytes_per_sec);
  Serial.print(F(" byte/s\nFrame duration is "));  Serial.print(us_per_frame);  Serial.println(F(" us"));
  Serial.print(F("Average frame length is "));  Serial.print(uVideoLen / frame_cnt);  Serial.println(F(" bytes"));
  Serial.print("Average picture time (ms) "); Serial.println( 1.0 * totalp / frame_cnt);
  Serial.print("Average write time (ms)   "); Serial.println( totalw / frame_cnt );
  Serial.print("Frames Skipped % ");  Serial.println( 100.0 * skipped / frame_cnt, 2 );
  Serial.print("Normal jpg % ");  Serial.println( 100.0 * normal_jpg / frame_cnt, 1 );
  Serial.print("Extend jpg % ");  Serial.println( 100.0 * extend_jpg / frame_cnt, 1 );
  Serial.print("Bad    jpg % ");  Serial.println( 100.0 * bad_jpg / total_frames, 1 );

  Serial.println("Writing the index");

  fseek(avifile, current_end, SEEK_SET);

  fclose(idxfile);

  size_t i1_err = fwrite(idx1_buf, 1, 4, avifile);

  print_quartet(frame_cnt * 16, avifile);

  idxfile = fopen("/sdcard/idx.tmp", "r");

  if (idxfile != NULL)  {
    //Serial.printf("File open: %s\n", "/sdcard/idx.tmp");
  }  else  {
    Serial.println("Could not open file");
    //major_fail();
  }

  char * AteBytes;
  AteBytes = (char*) malloc (8);

  for (int i = 0; i < frame_cnt; i++) {
    size_t res = fread ( AteBytes, 1, 8, idxfile);
    size_t i1_err = fwrite(dc_buf, 1, 4, avifile);
    size_t i2_err = fwrite(zero_buf, 1, 4, avifile);
    size_t i3_err = fwrite(AteBytes, 1, 8, avifile);
  }

  free(AteBytes);
  fclose(idxfile);
  fclose(avifile);
  int xx = remove("/sdcard/idx.tmp");

  Serial.println("---");

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  do_fb - just takes a picture and discards it
//

static void do_fb() {
  xSemaphoreTake( baton, portMAX_DELAY );
  camera_fb_t * fb = esp_camera_fb_get();

  //Serial.print("Pic, len="); Serial.println(fb->len);

  esp_camera_fb_return(fb);
  xSemaphoreGive( baton );
}

void do_time() {

  if (WiFi.status() != WL_CONNECTED) {

    Serial.println("***** WiFi reconnect *****");
    WiFi.reconnect();
    delay(5000);

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("***** WiFi rerestart *****");
      init_wifi();
    }

    MDNS.begin(devname);
    sprintf(localip, "%s", WiFi.localIP().toString().c_str());
  }

}

////////////////////////////////////////////////////////////////////////////////////
//
// some globals for the loop()
//

long wakeup;
long last_wakeup = 0;
int first = 1;

void loop()
{
  if (first) {
    Serial.print("the loop, core ");  Serial.print(xPortGetCoreID());
    Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));
    //vTaskPrioritySet( NULL, 2 );
    //print_ram();
    first = 0;
  }


  if (DeepSleepPir) {

        if (recording == 0 && PIRenabled == 1) {
        

        delay(10000);                                     //   wait 10 seoonds for another event before sleep

        Serial.println("Trying again ..  to sleep now?" );
        Serial.print("recording: "); Serial.println(recording);
        Serial.print("PIRenabled: "); Serial.println(PIRenabled);

      if (recording == 0 && PIRenabled == 1) {

        Serial.println("Going to sleep now");

        pinMode(4, OUTPUT);
        digitalWrite(4, LOW);
        rtc_gpio_hold_en(GPIO_NUM_4);
        gpio_deep_sleep_hold_en();
        digitalWrite(33, HIGH);
        //rtc_gpio_hold_en(GPIO_NUM_33);

        esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0);
        delay(500);
        esp_deep_sleep_start();
      }
    }
  }


  count_loop++;
  wakeup = millis();
  if (wakeup - last_wakeup > (60  * 60 * 1000) ) {       // 60 minutes
    last_wakeup = millis();
    do_time();


  }


  delay(1000);
}



