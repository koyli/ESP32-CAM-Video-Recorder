#include <dummy.h>

#include "credentials.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <esp32-aws-s3.h>
#include <esp32-cam.h>
#include <TimeLib.h>
#define digitalWrite(a,b) {Serial.print("Writing: ");Serial.print(a);Serial.print(",");Serial.println(b);digitalWrite(a,b);}

// MicroSD
#include <SD_MMC.h>

#include "driver/rtc_io.h"


#include "esp_task_wdt.h"


static const char devname[] = "desklens";         // name of your camera for mDNS, Router, and filenames


// https://sites.google.com/a/usapiens.com/opnode/time-zones  -- find your timezone here
#define TIMEZONE "GMT0BST,M3.5.0/01,M10.5.0/02"             // your timezone  -  this is GMT

// reboot startup parameters here

#define DEEP_SLEEP_PIR 1            // set to 1 to deepsleep between pir videos
#define RECORD_ON_REBOOT 1          // set to 1 to record, or 0 to NOT record on reboot

#define PIR_ENABLED 1

// here are 2 sets of startup parameters -- more down in the stop and restart webpage

// VGA 10 fps for 30 minutes, and repeat, play at real time

#define GRAY 0                     //  not gray
#define QUALITY 12                //  quality on the 10..50 subscale - 10 is good, 20 is grainy and smaller files, 12 is better in bright sunshine due to clipping
#define CAPTURE_INTERVAL 100       //  milli-seconds between frames
#define TOTAL_FRAMES_CONFIG 30;  //  how many frames - length of movie in ms is total_frames x capture_interval



#define RED_LIGHT_PIN GPIO_NUM_33
#define WHITE_LIGHT_PIN GPIO_NUM_4

#define PIR_PIN GPIO_NUM_12                   // for active high pir or microwave etc
#define IR_LED_PIN GPIO_NUM_13

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  edit parameters for wifi name, startup parameters in the local file settings.h

RTC_DATA_ATTR unsigned int epoch_time = 0;

framesize_t  framesize = (framesize_t) 8;                //  13 UXGA, 11 HD, 9 SVGA, 8 VGA, 6 CIF
int xspeed = 1;                                          /* speed up of record */

int quality = QUALITY;
#define MAX_VIDEOS 3
int videos;

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

int total_frames_config = TOTAL_FRAMES_CONFIG;
int capture_interval = CAPTURE_INTERVAL;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int count_avi = 0;
int count_cam = 0;

int  xlength = total_frames_config * capture_interval / 1000;
int total_frames = total_frames_config;

int recording = 0;
int uploading = 0;

int PIRrecording = 0;


WiFiMulti wifiMulti;

char fname[130];


long current_millis;
long last_capture_millis = 0;
static esp_err_t card_err;
char strftime_buf[64];
char strftime_buf2[12];

struct tm timeinfo;


char *filename ;
char *stream ;
int newfile = 0;
int frames_so_far = 0;
long bp;
long ap;
long bw;
long aw;
long totalp;
long totalw;
float avgp;
float avgw;


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


File avifile;
File idxfile;


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

TaskHandle_t CameraTask, AviWriterTask, uploadTask;
SemaphoreHandle_t baton;

int counter = 0;



#define SEND_BUF 8192
byte send_buffer[SEND_BUF];

int uploadFile(File todo)
{
    String name = todo.name();
    Serial.print("Uploading: ");
    String fullpath = String(name);
    Serial.println(fullpath);

    String cofile = todo.readStringUntil('\n');
    /* strip /sdcard/ from front */

    //    int bytes_sent = todo.readStringUntil('\n').toInt();

    
    File avi = SD_MMC.open(cofile.c_str());

    if (!avi) {
        Serial.print("Could not open AVI: ");
        Serial.println(cofile);
    }

    
    if (!AWS_S3::put(cofile, avi)) {
            /* fail */
        Serial.println ("failed put");
        return 0;
    }

    avi.close();
    return -1;
    /* return -1 if completed */
    /* return 0 if problem */
}


void traverse(File d) {
    File f;
    Serial.print("traversing: ");
    Serial.println(d.name());
    if (d.isDirectory()) {
        while (f = d.openNextFile())  {
            if (!f.isDirectory()) {
                String name = f.name();
                if (name.endsWith(".todo")) {
                    Serial.print("reading..: ");
                    Serial.println(f.name());
                    uploading = 1;
                    if (!uploadFile(f)) {
                        uploading = 0;
                        Serial.print("Error uploading ");
                        Serial.println(name);
                        return;
                    }

                    f.close();
                    SD_MMC.remove(name.c_str());

                }
                else {
                    f.close();
                }
            }
            else {
                traverse(f);
                f.close();
            }
        }
    }
    else {

    }
}

void upload() {
    
    /* look for files with " */
    Serial.println("Upload task ran");
    File d = SD_MMC.open("/");
    traverse(d);
    uploading = 0;
}


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

                do {

                    digitalWrite(IR_LED_PIN, HIGH);


                    fb_q[fb_in] = Camera::getFrame();
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
                        Camera::returnFrame(fb_q[fb_in]);

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
        } else {

            delay(capture_interval);
        }
    }

}


//
// Writes an uint32_t in Big Endian at current file position
//
static void inline print_quartet(unsigned long i, File fd)
{
    uint8_t y[4];
//uint8_t x[1];

  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;

  fd.write(y, 4);

}


char localip[20];

#define MAX_FRAMES (10 * 30)





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





void codeForUploadTask(void *parameter) {

    Serial.print("UploadTask running on core ");
    Serial.println(xPortGetCoreID());

    delay(5000);               /* wait 10  */
        
    Serial.println("Initializing Wifi on uploadTask");

        
    for (;;) {
        do_time();

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Wifi connected - will attempt upload");
            upload();
        }
        else {
            Serial.println("No wifi - skipping upload");
        }
        delay(15 * 1000);
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  setup()  - the Arduino setup
//

void setup() {

    Serial.begin(115200);
    Serial.println("\n\n---");

    rtc_gpio_hold_dis(RED_LIGHT_PIN);

    pinMode(RED_LIGHT_PIN, OUTPUT);             // little red led on back of chip
    digitalWrite(RED_LIGHT_PIN, LOW);           // turn on the red LED on the back of chip

    rtc_gpio_hold_dis(WHITE_LIGHT_PIN);
    pinMode(WHITE_LIGHT_PIN, OUTPUT);               // Blinding Disk-Avtive Light
    digitalWrite(WHITE_LIGHT_PIN, LOW);             // turn off


    AWS_S3::setup(ACCESS_KEY, SECRET_KEY, BUCKET);
    videos = 0;

    init_sdcard();
    
    Serial.setDebugOutput(true);
    Serial.print("setup, core ");  Serial.print(xPortGetCoreID());
    Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));

    // zzz
    Serial.println("                                    ");
    Serial.println("-------------------------------------");
    Serial.printf("ESP-CAM Video Recorder\n");
    Serial.println("-------------------------------------");


    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));
    print_wakeup_reason();

    if (epoch_time > 0) {
        /* will be lower bound - better than 1970 */
        struct timeval tv;
        tv.tv_sec = epoch_time;
        tv.tv_usec = 0;
        settimeofday(&tv, nullptr);
    }

    total_frames = total_frames_config;

    if (!psramFound()) {
        Serial.println("paraFound wrong - major fail");
        major_fail();
    }

    Serial.println("Starting sd card ...");
    // SD camera init

    
    pinMode(IR_LED_PIN, OUTPUT);
    pinMode(PIR_PIN, INPUT);
    digitalWrite(IR_LED_PIN, HIGH);


    if (card_err != ESP_OK) {
        Serial.printf("SD Card init failed with error 0x%x", card_err);
        major_fail();
        return;
    }

    Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));


    Serial.println("Starting tasks ...");

    if (!Camera::configure(fb_max, QUALITY, GRAY, framesize)) {
        major_fail();
    }


    
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


    xTaskCreatePinnedToCore(codeForUploadTask,
                            "uploadTask",
                            8192,
                            NULL,
                            1,
                            &uploadTask,
                            0);
    delay(20);

    Serial.println("Starting camera ...");


    newfile = 0;    // no file is open  // don't fiddle with this!

    recording = RECORD_ON_REBOOT;
    digitalWrite(IR_LED_PIN, recording ? HIGH : LOW);


    digitalWrite(RED_LIGHT_PIN, HIGH);         // red light turns off when setup is complete

    xTaskNotifyGive(AviWriterTask);

    delay(1000);


    delete_old_stuff();
}


//
// if we have no camera, or sd card, then flash rear led on and off to warn the human SOS - SOS
//
void major_fail() {

    Serial.println(" ");

    for  (int i = 0;  i < 5; i++) {                 // 5 loops or about 100 seconds then reboot
        for (int j = 0; j < 3; j++) {
            digitalWrite(RED_LIGHT_PIN, LOW);   delay(150);
            digitalWrite(RED_LIGHT_PIN, HIGH);  delay(150);
        }

        delay(1000);

        for (int j = 0; j < 3; j++) {
            digitalWrite(RED_LIGHT_PIN, LOW);  delay(500);
            digitalWrite(RED_LIGHT_PIN, HIGH); delay(500);
        }
        delay(1000);
        Serial.print("Major Fail  "); Serial.print(i); Serial.print(" / "); Serial.println(10);
    }

    ESP.restart();
}

void printLocalTime()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        Serial.println("Failed to obtain time");
        return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

bool init_wifi()
{
    Serial.println("Initializing Wifi");

    Serial.println(" Disable brownout");
    uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG); //save WatchDog register
    Serial.print("\nBrownOut Regsiter was (in hex) "); Serial.println(brown_reg_temp, HEX);
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

    wifiMulti.addAP(WIFI_SSID_1, WIFI_ACCESSCODE_1);
    wifiMulti.addAP(WIFI_SSID_2, WIFI_ACCESSCODE_2);

    unsigned long startTime = millis();
    
    while (wifiMulti.run() != WL_CONNECTED && millis() - startTime < 60000)
        {
            Serial.print(".");
            delay(500);
        }

    if (!MDNS.begin(devname)) {
        Serial.println("Error setting up MDNS responder!");
    } else {
        Serial.printf("mDNS responder started '%s'\n", devname);
    }
    
    Serial.print("Setting up NTP.. existing time is ");
    printLocalTime();

    configTime(0, 0, "pool.ntp.org");

    setenv("TZ", TIMEZONE, 1);  // mountain time zone from #define at top
    tzset();
    
    timeinfo = { 0 };
    delay(1000);

    sprintf(localip, "%s", WiFi.localIP().toString().c_str());
    
    while(time(nullptr) < 100000) {
        Serial.print("t");
        delay(100);
    }
    setTime(time(nullptr));
    unsigned long t = now();
    
    char buf1[20];
    
    Serial.println(buf1);
    printLocalTime();
 

    Serial.println(" Enable brownout");
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector

    return true;
}




static void init_sdcard()
{
    Serial.println("Initializing SD Card");
    pinMode(IR_LED_PIN, INPUT_PULLUP);

    Serial.print("SD_MMC Begin: ");
    if (!SD_MMC.begin("/sdcard", true))
        major_fail();   // required by ftp system ??
    pinMode(IR_LED_PIN, INPUT_PULLDOWN);
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

    if (PIR_ENABLED) {

        int PIRstatus = digitalRead(PIR_PIN);
        if (DEEP_SLEEP_PIR && millis() < 15000 ) {
            PIRstatus  = 1;
        }

        if (PIRstatus == 1) {
            if (PIRrecording == 1) {
                // keep recording for 15 more seconds
                if ( (millis() - startms) > (total_frames * capture_interval - 5000)
                     &&
                     total_frames < MAX_FRAMES) {

                    total_frames = total_frames + 10000 / capture_interval ;
                    //Serial.print("Make PIR frames = "); Serial.println(total_frames);
                    Serial.print("f");
                    //Serial.println("Add another 10 seconds");
                }
                else {
                    Serial.print("u");
                }
            } else {

                if ( recording == 0 && newfile == 0 && videos++ < MAX_VIDEOS) {

                    //start a pir recording with current parameters, except no repeat and 15 seconds
                    Serial.println("Start a PIR");
                    PIRrecording = 1;
                    total_frames = 15000 / capture_interval;
                    xlength = total_frames * capture_interval / 1000;
                    recording = 1;
                    digitalWrite(IR_LED_PIN, HIGH);
                  
                }
            }
        }
    }

    // we are recording, but no file is open

    if (newfile == 0 && recording == 1) {                                     // open the file
        digitalWrite(IR_LED_PIN, HIGH);
        digitalWrite(RED_LIGHT_PIN, HIGH);
        newfile = 1;

        Serial.println(" ");

        delete_old_stuff();

        start_avi();                                 // now start the avi

    } else {

         // we have a file open, but not recording

        if (newfile == 1 && recording == 0) {                                  // got command to close file

            end_avi();

            Serial.println("Done capture due to command");

            frames_so_far = total_frames;

            newfile = 0;    // file is closed
           recording = 0;  // DO NOT start another recording
            digitalWrite(IR_LED_PIN, LOW);
            PIRrecording = 0;

        } else {

            if (newfile == 1 && recording == 1) {                            // regular recording

                if ((millis() - startms) > (total_frames * capture_interval)) {  // time is up, even though we have not done all the frames

                    Serial.println (" "); Serial.println("Done capture for time");
                    Serial.print("Time Elapsed: "); Serial.print(millis() - startms); Serial.print(" Frames: "); Serial.println(frame_cnt);
                    Serial.print("Config:       "); Serial.print(total_frames * capture_interval ) ; Serial.print(" (");
                    Serial.print(total_frames); Serial.print(" x "); Serial.print(capture_interval);  Serial.println(")");

                    digitalWrite(RED_LIGHT_PIN, LOW);                                                       // close the file

                    end_avi();
                    frames_so_far = 0;
                    newfile = 0;          // file is closed
                    recording = 0;
                    PIRrecording = 0;
                    digitalWrite(IR_LED_PIN, LOW);;
                    

                } else  {                                                            // regular

                    another_save_avi();

                }
            }
        }
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// start_avi - open the files and write in headers
//


static void start_avi() {

    Serial.println("Starting an avi ");


    time_t n = time(nullptr);
    bool fixTime = false; 
    if (time(nullptr) < 100000) {
        /* still 1970.. */
        fixTime = true;
    }

    Serial.print("Fixtime: ");
    Serial.println(fixTime);
    Serial.println(time(nullptr));
    Serial.println(now());
    
    localtime_r(&n, &timeinfo);
    if (!fixTime) {
        strftime(strftime_buf2, sizeof(strftime_buf2), "/%Y%m%d", &timeinfo);
    }
    else {
        snprintf(strftime_buf2, sizeof(strftime_buf2), "/%08d", bootCount);
    }
    SD_MMC.mkdir(strftime_buf2);

    strftime(strftime_buf, sizeof(strftime_buf), "%F_%H.%M.%S", &timeinfo);

    //v99 - uxga 13, hd 11, svga 9, vga 8, cif 6
    if (framesize == 8) {
        sprintf(fname, "%s/%s_%s_vga_Q%d_I%d_L%d_S%d.avi", strftime_buf2, devname, strftime_buf, quality, capture_interval, xlength, xspeed);
    } else if (framesize == 9) {
        sprintf(fname, "%s/%s_%s_svga_Q%d_I%d_L%d_S%d.avi", strftime_buf2, devname,  strftime_buf, quality, capture_interval, xlength, xspeed);
    } else if (framesize == 11) {
        sprintf(fname, "%s/%s_%s_hd_Q%d_I%d_L%d_S%d.avi", strftime_buf2, devname,  strftime_buf, quality, capture_interval, xlength, xspeed);
    } else if (framesize == 13) {
        sprintf(fname, "%s/%s_%s_uxga_Q%d_I%d_L%d_S%d.avi", strftime_buf2, devname, strftime_buf, quality, capture_interval, xlength, xspeed);
    } else  if (framesize == 6) {
        sprintf(fname, "%s/%s_%s_cif_Q%d_I%d_L%d_S%d.avi", strftime_buf2, devname, strftime_buf, quality, capture_interval, xlength, xspeed);
    } else {
        Serial.println("Wrong framesize");
    }

    Serial.print("\nFile name will be >"); Serial.print(fname); Serial.println("<");

    avifile = SD_MMC.open(fname, FILE_WRITE);
    idxfile = SD_MMC.open("/idx.tmp", FILE_WRITE);

    if (!avifile) {
        Serial.println("Could not open file");
        major_fail();
    }

    if (!idxfile) {
        Serial.println("Could not open file");
        major_fail();
    }

    for ( i = 0; i < AVIOFFSET; i++)
        {
            char ch = pgm_read_byte(&avi_header[i]);
            buf[i] = ch;
        }

    avifile.write(buf, AVIOFFSET);
    //v99 - uxga 13, hd 11, svga 9, vga 8, cif 6
    if (framesize == 8) {

        avifile.seek(0x40);
        avifile.write(vga_w, 2);
        avifile.seek(0xA8);
        avifile.write(vga_w, 2);
        avifile.seek(0x44);
        avifile.write(vga_h, 2);
        avifile.seek(0xAC);
        avifile.write(vga_h, 2);

    } else if (framesize == 13) {

        avifile.seek(0x40);
        avifile.write(uxga_w, 2);
        avifile.seek(0xA8);
        avifile.write(uxga_w, 2);
        avifile.seek(0x44);
        avifile.write(uxga_h, 2);
        avifile.seek(0xAC);
        avifile.write(uxga_h, 2);

    } else if (framesize == 11) {

        avifile.seek(0x40);
        avifile.write(hd_w, 2);
        avifile.seek(0xA8);
        avifile.write(hd_w, 2);
        avifile.seek(0x44);
        avifile.write(hd_h, 2);
        avifile.seek(0xAC);
        avifile.write(hd_h, 2);


    } else if (framesize == 9) {

        avifile.seek(0x40);
        avifile.write(svga_w, 2);
        avifile.seek(0xA8);
        avifile.write(svga_w, 2);
        avifile.seek(0x44);
        avifile.write(svga_h, 2);
        avifile.seek(0xAC);
        avifile.write(svga_h, 2);

    }  else if (framesize == 6) {

        avifile.seek(0x40);
        avifile.write(cif_w, 2);
        avifile.seek(0xA8);
        avifile.write(cif_w, 2);
        avifile.seek(0x44);
        avifile.write(cif_h, 2);
        avifile.seek(0xAC);
        avifile.write(cif_h, 2);
    }

    avifile.seek(AVIOFFSET);

    Serial.print(F("\nRecording "));
    Serial.print(total_frames);
    Serial.println(F(" video frames ...\n"));

    startms = millis();
    bigdelta = millis();
    totalp = 0;
    totalw = 0;
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

    } else {

        fb_out = (fb_out + 1) % fb_max;

        int fblen;
        fblen = fb_q[fb_out]->len;

        //xSemaphoreGive( baton );


        jpeg_size = fblen;
        movi_size += jpeg_size;
        uVideoLen += jpeg_size;

        bw = millis();
        avifile.write(dc_buf, 4);
        avifile.write(zero_buf, 4);


        int time_to_give_up = 0;
        while (ESP.getFreeHeap() < 35000) {
            Serial.print(time_to_give_up); Serial.print(" Low on heap "); Serial.print(ESP.getFreeHeap());
            Serial.print(" frame q = "); Serial.println((fb_in + fb_max - fb_out) % fb_max);
            if (time_to_give_up++ == 50) break;
            delay(100 + 5 * time_to_give_up);
        }

        size_t err = avifile.write(fb_q[fb_out]->buf, fb_q[fb_out]->len);
        time_to_give_up = 0;
        if (err != fb_q[fb_out]->len) {
            Serial.print("Error on avi write: bytes_written = "); Serial.print(err);
            Serial.print(" len = "); Serial.println(fb_q[fb_out]->len);
            major_fail();

        }

        //totalw = totalw + millis() - bw;

        //xSemaphoreTake( baton, portMAX_DELAY );
        Camera::returnFrame(fb_q[fb_out]);     // release that buffer back to the camera system
        xSemaphoreGive( baton );

        remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;

        print_quartet(idx_offset, idxfile);
        print_quartet(jpeg_size, idxfile);

        idx_offset = idx_offset + jpeg_size + remnant + 8;

        jpeg_size = jpeg_size + remnant;
        movi_size = movi_size + remnant;
        if (remnant > 0) {
            avifile.write(zero_buf, remnant);
        }

        fileposition = avifile.position();       // Here, we are at end of chunk (after padding)
        avifile.seek(fileposition - jpeg_size - 4);    // Here we are the the 4-bytes blank placeholder

        print_quartet(jpeg_size, avifile);    // Overwrite placeholder with actual frame size (without padding)

        fileposition = avifile.position();


        avifile.seek(fileposition + jpeg_size);

        totalw = totalw + millis() - bw;



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

    current_end = avifile.position();

    Serial.println("End of avi - closing the files");

    elapsedms = millis() - startms;
    float fRealFPS = (1000.0f * (float)frame_cnt) / ((float)elapsedms) * xspeed;
    float fmicroseconds_per_frame = 1000000.0f / fRealFPS;
    uint8_t iAttainedFPS = round(fRealFPS);
    uint32_t us_per_frame = round(fmicroseconds_per_frame);

    //Modify the MJPEG header from the beginning of the file, overwriting various placeholders

    avifile.seek(4);
    print_quartet(movi_size + 240 + 16 * frame_cnt + 8 * frame_cnt, avifile);

    avifile.seek(0x20);
    print_quartet(us_per_frame, avifile);

    unsigned long max_bytes_per_sec = movi_size * iAttainedFPS / frame_cnt;

    avifile.seek(0x24);
    print_quartet(max_bytes_per_sec, avifile);

    avifile.seek(0x30);
    print_quartet(frame_cnt, avifile);

    avifile.seek(0x8c);
    print_quartet(frame_cnt, avifile);

    avifile.seek(0x84);
    print_quartet((int)iAttainedFPS, avifile);

    avifile.seek(0xe8);
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

    avifile.seek(current_end);

    idxfile.close();

    avifile.write(idx1_buf, 4);

    print_quartet(frame_cnt * 16, avifile);

    idxfile = SD_MMC.open("/idx.tmp", FILE_READ);

    if (!idxfile) {
        Serial.println("Could not open file");
        //major_fail();
    }

    uint8_t * AteBytes;
    AteBytes = (uint8_t*) malloc (8);

    for (int i = 0; i < frame_cnt; i++) {
        idxfile.read(AteBytes, 8);
        avifile.write(dc_buf, 4);
        avifile.write(zero_buf, 4);
        avifile.write(AteBytes, 8);
    }

    free(AteBytes);
    idxfile.close();
    avifile.close();
    SD_MMC.remove("/idx.tmp");

    String fname_status = fname;

    fname_status = fname_status + String(".todo");
    
    File f = SD_MMC.open(fname_status.c_str(), FILE_WRITE);
    f.write((const uint8_t*)fname, strlen(fname));
    f.write((const uint8_t*)"\n0\n", 3);
    f.close();

    
    Serial.println("---");

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  do_fb - just takes a picture and discards it
//


void do_time() {

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("***** WiFi rerestart *****");
        init_wifi();
    }

}

////////////////////////////////////////////////////////////////////////////////////
//
// some globals for the loop()
//


void loop()
{


  if (DEEP_SLEEP_PIR) {
      if (recording == 0) {
          Serial.println("Not recording");

          if (PIR_ENABLED && uploading == 0) {

              delay(30000);                                     //   wait 10 seoonds for another event before sleep
              
              Serial.println("Trying again ..  to sleep now?" );


              if (uploading == 0) {

                  Serial.println("Going to sleep now");
                  digitalWrite(IR_LED_PIN, LOW);
                  gpio_hold_en(IR_LED_PIN);
                  
                  pinMode(WHITE_LIGHT_PIN, OUTPUT);
                  digitalWrite(WHITE_LIGHT_PIN, LOW);
                  rtc_gpio_hold_en(WHITE_LIGHT_PIN);
                  gpio_deep_sleep_hold_en();
                  digitalWrite(RED_LIGHT_PIN, HIGH);

                  epoch_time = now();

                  if (videos >= MAX_VIDEOS) {
                      Serial.println("Too many videos - sleeping for 5 mins");
                      esp_sleep_enable_timer_wakeup(10e6 * 300); /* 5 mins */
                      delay(500);
                  }
                  else {
                      esp_sleep_enable_ext0_wakeup(PIR_PIN, 1); /* wake on high */
                      delay(500);
                  }
                  esp_deep_sleep_start();
              }
              else {
                  Serial.println("Uploading started - not sleeping");
              }
          }
          else {
              
          }
      }
      else {

      }
  }
  Serial.printf("recording: %d, uploading: %d, AVI duration: %lu\n", recording, uploading, (bw - startms)/1000);

  delay(1000);
}



