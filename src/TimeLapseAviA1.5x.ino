#include <dummy.h>

#include "HTTPClient.h"
#include "mbedtls/md.h"
#include "credentials.h"

#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoOTA.h>

#include <time.h>
#include <TimeLib.h>
//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "esp_camera.h"

#define RED_LIGHT_PIN GPIO_NUM_33
#define WHITE_LIGHT_PIN GPIO_NUM_4

#define PIR_PIN GPIO_NUM_12                   // for active high pir or microwave etc

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  edit parameters for wifi name, startup parameters in the local file settings.h
#include "settings.h"


RTC_DATA_ATTR unsigned int epoch_time = 0;


framesize_t  framesize = (framesize_t) 8;                //  13 UXGA, 11 HD, 9 SVGA, 8 VGA, 6 CIF
int xspeed = XSPEED;

int doUpload = 0;
int quality = QUALITY;
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
int repeat = REPEAT_CONFIG;
int total_frames = total_frames_config;

int recording = 0;
int uploading = 0;

int PIRrecording = 0;
int ready = 0;










#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
bool InternetFailed = false;

int diskspeed = 0;
char fname[130];

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

TaskHandle_t CameraTask, AviWriterTask, uploadTask;
SemaphoreHandle_t baton;
SemaphoreHandle_t aviBaton;

int counter = 0;



String sha256(const byte* payload, unsigned int len)
{
    byte shaResult[32];
    String r;
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
    mbedtls_md_starts(&ctx);
    mbedtls_md_update(&ctx, (const unsigned char *) payload, len);
    mbedtls_md_finish(&ctx, shaResult);
    mbedtls_md_free(&ctx);
  
    for(int i= 0; i< sizeof(shaResult); i++){
        char str[3];

        sprintf(str, "%02x", (int)shaResult[i]);
        Serial.print(str);
        r += String(str);
    }
    return r;
}


String hmac256(const byte *key, int keylen, const byte* payload, int len)
{
    byte shaResult[32];
    String r;
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
    mbedtls_md_hmac_starts(&ctx, key, keylen);
    mbedtls_md_hmac_update(&ctx, (const unsigned char *) payload, len);
    mbedtls_md_hmac_finish(&ctx, shaResult);
    mbedtls_md_free(&ctx);
  
    for(int i= 0; i< sizeof(shaResult); i++){
        char str[3];

        sprintf(str, "%02x", (int)shaResult[i]);
        r += String(str);
    }

    return r;

}

void fromHex(String s, byte* b)
{
    for (int i = 0; i < 32; i++) {
        byte c = s.c_str()[2*i];
        c = c > '9' ? (c -'a' + 10) : (c-'0');
        byte d = s.c_str()[2*i + 1];
        d = d > '9' ? (d -'a' + 10) : (d-'0');
      
        b[i] = (byte) (c * 16 + d);
    }
}

String dateHeader;

String canonicalRequest(String req, String path, const byte payload[], int length) {

    unsigned long t = now();

    char buf1[20];

    sprintf(buf1, "%04d%02d%02dT%02d%02d%02dZ",  year(t),month(t), day(t), hour(t), minute(t), second(t));

    String filename = path;
  
    String hash = sha256(payload, length);
    Serial.println(hash); 
    dateHeader = buf1;
  
    String r = req +"\n"; 
    r += filename + "\n";		/* CanonicalURI */
    r += String("\n");		/* CanonicalQueryString */
    r += String("host:" BUCKET "\n");		/* CanonicalHeaders */
    r += String("x-amz-content-sha256:") + hash + "\n";
    r += String( "x-amz-date:") + dateHeader + "\n";
    r += "\n";
    r += String("host;x-amz-content-sha256;x-amz-date\n");		/* SignedHeaders */
    r += sha256(payload, length);

    return r;
}

String canonicalUnsignedRequest(String req, String path, int length) {

    unsigned long t = now();

    char buf1[20];

    sprintf(buf1, "%04d%02d%02dT%02d%02d%02dZ",  year(t),month(t), day(t), hour(t), minute(t), second(t));

    String filename = path;
  
    dateHeader = buf1;
  
    String r = req +"\n"; 
    r += filename + "\n";		/* CanonicalURI */
    r += String("\n");		/* CanonicalQueryString */
    r += String("host:" BUCKET "\n");		/* CanonicalHeaders */
    r += String("x-amz-content-sha256:") + "UNSIGNED-PAYLOAD" + "\n";
    r += String( "x-amz-date:") + dateHeader + "\n";
    r += "\n";
    r += String("host;x-amz-content-sha256;x-amz-date\n");		/* SignedHeaders */
    r += "UNSIGNED-PAYLOAD";

    return r;
}



String toSign(String msg)
{
    unsigned long t = now();
    char buf1[20];
    char buf2[20];
    sprintf(buf1, "%04d%02d%02dT%02d%02d%02dZ",  year(t),month(t), day(t), hour(t), minute(t), second(t));
    sprintf(buf2, "%04d%02d%02d",  year(t),month(t), day(t));

    String sts = "AWS4-HMAC-SHA256\n";

    sts += buf1;
    sts += "\n";
  
    sts += buf2;
    sts += "/eu-west-2/s3/aws4_request\n"  ;
    
    sts += sha256((const byte*)msg.c_str(), strlen(msg.c_str()));
    return sts;
  
}

String signKey()
{
    unsigned long t = now();
    char buf1[20];
    char buf2[20];
    sprintf(buf1, "%04d%02d%02dT%02d%02d%02dZ",  year(t),month(t), day(t), hour(t), minute(t), second(t));
    sprintf(buf2, "%04d%02d%02d",  year(t),month(t), day(t));

    byte h1[32];
    byte h2[32];
    byte h3[32];
    const char* region = "eu-west-2";
    const char* service = "s3";
    const char* req = "aws4_request";
    const char * secret_key = "AWS4" SECRET_KEY;
    Serial.println(secret_key);
    String hex1 = hmac256((const byte*)(secret_key), strlen(secret_key), (const byte*)buf2, strlen(buf2));
    fromHex(hex1, h1);
    String hex2 = hmac256(h1, 32, (byte*)region, strlen(region));
    Serial.println(hex2);
    fromHex(hex2, h2);
    String hex3 = hmac256(h2, 32, (byte*)service, strlen(service));
    Serial.println(hex3);
    fromHex(hex3, h3);
    return hmac256(h3, 32, (byte*)req, strlen(req));
}

String sign(String key, String msg) {
    byte b[32];
    fromHex(key, b);
    return hmac256(b, 32, (const byte*) msg.c_str(), msg.length());
}

String auth(String sig) {
  
    unsigned long t = now();

    char buf2[20];
    sprintf(buf2, "%04d%02d%02d",  year(t),month(t), day(t));
  
    String ah = "AWS4-HMAC-SHA256 ";
    ah += "Credential=" ACCESS_KEY;
    ah += "/";
    ah += buf2;
    ah += "/eu-west-2/s3/aws4_request, ";
    ah += "SignedHeaders=host;x-amz-content-sha256;x-amz-date, ";
    ah += "Signature=";
    ah += sig;
    return ah;
}

int put(String path, const byte payload[], int length)
{

    String can = canonicalRequest(String("PUT"), path, payload, length);
    String sts = toSign(can);
    String skey = signKey();
  

    Serial.print("Can: ");
    Serial.println(can);
    Serial.print("String to Sign: ");
    Serial.println(sts);
    Serial.print("Signing Key: ");
    Serial.println(skey);
    Serial.print("Signed:");
    Serial.println(sign(skey, sts));
    HTTPClient http;
    http.begin(String("http://net.lecomber.cctv.s3.eu-west-2.amazonaws.com") + path);
    http.addHeader("Authorization", auth(sign(skey, sts)));
    http.addHeader("x-amz-content-sha256", sha256(payload, length));
    http.addHeader("x-amz-date", dateHeader);
    http.sendRequest("PUT", (unsigned char*)payload, length);

    Serial.println(http.getString());
    return -1;
    
}

#define PAYLOAD_MAX 8192
byte transferBuff[PAYLOAD_MAX];

int put(String path, File payload)
{
    
    
    String can = canonicalUnsignedRequest(String("PUT"), path, payload.size());
    String sts = toSign(can);
    String skey = signKey();

    Serial.print("Can: ");
    Serial.println(can);
    Serial.print("String to Sign: ");
    Serial.println(sts);
    Serial.print("Signing Key: ");
    Serial.println(skey);
    Serial.print("Signed:");
    Serial.println(sign(skey, sts));

    Serial.print("Length: ");
    Serial.println(payload.size());
    
    HTTPClient http;
    http.begin(String("http://net.lecomber.cctv.s3.eu-west-2.amazonaws.com") + path);
    http.addHeader("Authorization", auth(sign(skey, sts)));
    http.addHeader("x-amz-content-sha256", "UNSIGNED-PAYLOAD");
    http.addHeader("x-amz-date", dateHeader);
    Serial.println("Sending.. " );

    if (payload.size() > 0) {
        http.sendRequest("PUT", &payload, payload.size());
    }
    Serial.print("File sent: ");
    Serial.println(path);

    return -1;
    
}




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

    cofile = cofile.substring(strlen("/sdcard"));

    //    int bytes_sent = todo.readStringUntil('\n').toInt();

    
    File avi = SD_MMC.open(cofile.c_str());

    if (!avi) {
        Serial.print("Could not open AVI: ");
        Serial.println(cofile);
    }

    
    if (!put(fname, avi)) {
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
                    uploading = 0;
                    return;
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

                    digitalWrite(GPIO_NUM_13, HIGH);

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
        } else {

            delay(capture_interval);
        }
    }

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

  fwrite(y , 1, 4, fd);

}



char localip[20];
WiFiEventId_t eventID;

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  PIR_ISR - interupt handler for PIR  - starts or extends a video
//
#define MAX_FRAMES (10 * 30)


static void IRAM_ATTR PIR_ISR(void* arg) {

    int PIRstatus = digitalRead(PIR_PIN);

  if (PIR_ENABLED) {
    if (PIRstatus == 0) {
      if (PIRrecording == 1) {
        // keep recording for 15 more seconds

        if ( (millis() - startms) > (total_frames * capture_interval - 5000) &&
             total_frames < MAX_FRAMES) {

          total_frames = total_frames + 10000 / capture_interval ;
          //Serial.print("PIR frames = "); Serial.println(total_frames);
          Serial.print("*");
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

  pinMode(PIR_PIN, INPUT_PULLDOWN);

  Serial.print("PIR_PIN = ");
  for (int i = 0; i < 5; i++) {
    Serial.print( digitalRead(PIR_PIN) ); Serial.print(", ");
  }
  Serial.println(" ");

  esp_err_t err = gpio_isr_handler_add((gpio_num_t)PIR_PIN, &PIR_ISR, NULL);

  if (err != ESP_OK) Serial.printf("gpio_isr_handler_add failed (%x)", err);
  gpio_set_intr_type((gpio_num_t)PIR_PIN, GPIO_INTR_ANYEDGE);

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
  ledcAttachPin(WHITE_LIGHT_PIN, 5);
  ledcWrite( 5, 7);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// print_ram - debugging function for show heap total and in tasks, loops through priority tasks
//

void print_ram() {
  Serial.println("cam / avi / loop ");
  Serial.print(count_cam); Serial.print(" / ");
  Serial.print(count_avi); Serial.print(" / ");

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
//  delete_old_stuff() - before every avi, delete oldest day if freespace less than 10%
//
//  code copied from user @gemi254

void delete_old_stuff() {
    if (!DELETE_OLD_FILES) {
        Serial.println("Not deleting - configured not to delete old files");
        return;
    }
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

    for (;;) {
        
        do_time();
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Wifi connected - will attempt upload");
            upload();
        }
        else {
            Serial.println("No wifi - skipping upload");
        }
        delay(60 * 1000);
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

    rtc_gpio_hold_dis(GPIO_NUM_13);
    //pinMode(GPIO_NUM_13, OUTPUT);
    //digitalWrite(GPIO_NUM_13, HIGH);

    
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

    
    card_err = init_sdcard();
    pinMode(GPIO_NUM_13, OUTPUT);
    digitalWrite(GPIO_NUM_13, HIGH);


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


    xTaskCreatePinnedToCore(codeForUploadTask,
                            "uploadTask",
                            3072,
                            NULL,
                            1,
                            &uploadTask,
                            0);
                          
    delay(20);

    Serial.println("Starting camera ...");

    recording = 0;  // we are NOT recording
    config_camera();

    setupinterrupts();

    newfile = 0;    // no file is open  // don't fiddle with this!

    recording = RECORD_ON_REBOOT;

    //plm print_ram();  delay(2000);

    ready = 1;
    digitalWrite(RED_LIGHT_PIN, HIGH);         // red light turns off when setup is complete

    Serial.print(WiFi.localIP());
    Serial.println("' to connect");

    xTaskNotifyGive(AviWriterTask);

    delay(1000);

    print_ram();

    delete_old_stuff();
}


//
// if we have no camera, or sd card, then flash rear led on and off to warn the human SOS - SOS
//
void major_fail() {

    Serial.println(" ");

    for  (int i = 0;  i < 10; i++) {                 // 10 loops or about 100 seconds then reboot
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

    Serial.println("Using WM - go configure - if you haven't yet!");
    WiFiManager wm;
    bool res;
    //wm.resetSettings();  // for debugging

    wm.setConnectTimeout(20); // how long to try to connect for before continuing
    wm.setConfigPortalTimeout(30); // auto close configportal after n seconds
    
    res = wm.autoConnect(devname); // use the devname defined above, with no password
    
    if (res) {
        Serial.println("Connecting using WiFiManager");
        
    } else {
        InternetFailed = true;
        Serial.println("Internet failed using WiFiManager");
    }
    
    setTime(time(nullptr));

    if (!InternetFailed) {
        if (!MDNS.begin(devname)) {
            Serial.println("Error setting up MDNS responder!");
        } else {
            Serial.printf("mDNS responder started '%s'\n", devname);
        }

        configTime(0, 0, "pool.ntp.org");

        setenv("TZ", TIMEZONE, 1);  // mountain time zone from #define at top
        tzset();

        timeinfo = { 0 };
        delay(1000);

        time_t n = now();
        localtime_r(&n, &timeinfo);
        Serial.print("Local time: "); Serial.println(ctime(&n));
        sprintf(localip, "%s", WiFi.localIP().toString().c_str());

        while(time(nullptr) < 100000) {
            Serial.print("t");
            delay(100);
        }
        printLocalTime();
        setTime(time(nullptr));
        unsigned long t = now();
        
        char buf1[20];

        sprintf(buf1, "%04d%02d%02dT%02d%02d%02dZ",  year(t),month(t), day(t), hour(t), minute(t), second(t));
        Serial.println(buf1);
    }

    wifi_ps_type_t the_type;

    esp_wifi_get_ps(&the_type);
    Serial.printf("The power save was: %d\n", the_type);

    Serial.printf("Set power save to %d\n", WIFI_PS_NONE);
    esp_wifi_set_ps(WIFI_PS_NONE);

    esp_wifi_get_ps(&the_type);
    Serial.printf("The power save is : %d\n", the_type);

    Serial.println(" Enable brownout");
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector


    return true;
}




static esp_err_t init_sdcard()
{
    Serial.println("Initializing SD Card");
    pinMode(GPIO_NUM_13, INPUT_PULLUP);

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
    Serial.print("SD_MMC Begin: ");
    Serial.println(SD_MMC.begin());   // required by ftp system ??
    pinMode(GPIO_NUM_13, INPUT_PULLDOWN);
    return ret;
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
            PIRstatus  = 0;
        }

        if (PIRstatus == 0) {
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

    // we are recording, but no file is open

    if (newfile == 0 && recording == 1) {                                     // open the file
        digitalWrite(GPIO_NUM_13, HIGH);
        digitalWrite(RED_LIGHT_PIN, HIGH);
        newfile = 1;

        Serial.println(" ");

        delete_old_stuff();

        start_avi();                                 // now start the avi

    } else {

         // we have a file open, but not recording

        if (newfile == 1 && recording == 0) {                                  // got command to close file

            digitalWrite(RED_LIGHT_PIN, LOW);
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

                    digitalWrite(RED_LIGHT_PIN, LOW);                                                       // close the file

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

    fb_max = 6;           //74.5 from 7                      // for vga and uxga
    config.jpeg_quality = 6;  //74.5 from 7

    config.fb_count = fb_max + 1;

    // camera init
    cam_err = esp_camera_init(&config);
    if (cam_err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", cam_err);
        major_fail();
    }

    delay(100);

    sensor_t * ss = esp_camera_sensor_get();
    ss->set_quality(ss, quality);
    ss->set_framesize(ss, (framesize_t)framesize);
    if (GRAY) {
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

    fwrite(buf, 1, AVIOFFSET, avifile);
    //v99 - uxga 13, hd 11, svga 9, vga 8, cif 6
    if (framesize == 8) {

        fseek(avifile, 0x40, SEEK_SET);
        fwrite(vga_w, 1, 2, avifile);
        fseek(avifile, 0xA8, SEEK_SET);
        fwrite(vga_w, 1, 2, avifile);
        fseek(avifile, 0x44, SEEK_SET);
        fwrite(vga_h, 1, 2, avifile);
        fseek(avifile, 0xAC, SEEK_SET);
        fwrite(vga_h, 1, 2, avifile);

    } else if (framesize == 13) {

        fseek(avifile, 0x40, SEEK_SET);
        fwrite(uxga_w, 1, 2, avifile);
        fseek(avifile, 0xA8, SEEK_SET);
        fwrite(uxga_w, 1, 2, avifile);
        fseek(avifile, 0x44, SEEK_SET);
        fwrite(uxga_h, 1, 2, avifile);
        fseek(avifile, 0xAC, SEEK_SET);
        fwrite(uxga_h, 1, 2, avifile);

    } else if (framesize == 11) {

        fseek(avifile, 0x40, SEEK_SET);
        fwrite(hd_w, 1, 2, avifile);
        fseek(avifile, 0xA8, SEEK_SET);
        fwrite(hd_w, 1, 2, avifile);
        fseek(avifile, 0x44, SEEK_SET);
        fwrite(hd_h, 1, 2, avifile);
        fseek(avifile, 0xAC, SEEK_SET);
        fwrite(hd_h, 1, 2, avifile);


    } else if (framesize == 9) {

        fseek(avifile, 0x40, SEEK_SET);
        fwrite(svga_w, 1, 2, avifile);
        fseek(avifile, 0xA8, SEEK_SET);
        fwrite(svga_w, 1, 2, avifile);
        fseek(avifile, 0x44, SEEK_SET);
        fwrite(svga_h, 1, 2, avifile);
        fseek(avifile, 0xAC, SEEK_SET);
        fwrite(svga_h, 1, 2, avifile);

    }  else if (framesize == 6) {

        fseek(avifile, 0x40, SEEK_SET);
        fwrite(cif_w, 1, 2, avifile);
        fseek(avifile, 0xA8, SEEK_SET);
        fwrite(cif_w, 1, 2, avifile);
        fseek(avifile, 0x44, SEEK_SET);
        fwrite(cif_h, 1, 2, avifile);
        fseek(avifile, 0xAC, SEEK_SET);
        fwrite(cif_h, 1, 2, avifile);
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


        jpeg_size = fblen;
        movi_size += jpeg_size;
        uVideoLen += jpeg_size;

        bw = millis();
        fwrite(dc_buf, 1, 4, avifile);
        fwrite(zero_buf, 1, 4, avifile);


        int time_to_give_up = 0;
        while (ESP.getFreeHeap() < 35000) {
            Serial.print(time_to_give_up); Serial.print(" Low on heap "); Serial.print(ESP.getFreeHeap());
            Serial.print(" frame q = "); Serial.println((fb_in + fb_max - fb_out) % fb_max);
            if (time_to_give_up++ == 50) break;
            delay(100 + 5 * time_to_give_up);
        }

        size_t err = fwrite(fb_q[fb_out]->buf, 1, fb_q[fb_out]->len, avifile);

        time_to_give_up = 0;
        if (err != fb_q[fb_out]->len) {
            Serial.print("Error on avi write: bytes_written = "); Serial.print(err);
            Serial.print(" len = "); Serial.println(fb_q[fb_out]->len);
            major_fail();

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
            fwrite(zero_buf, 1, remnant, avifile);
        }

        fileposition = ftell (avifile);       // Here, we are at end of chunk (after padding)
        fseek(avifile, fileposition - jpeg_size - 4, SEEK_SET);    // Here we are the the 4-bytes blank placeholder

        print_quartet(jpeg_size, avifile);    // Overwrite placeholder with actual frame size (without padding)

        fileposition = ftell (avifile);


        fseek(avifile, fileposition + jpeg_size  , SEEK_SET);

        totalw = totalw + millis() - bw;

        digitalWrite(RED_LIGHT_PIN, HIGH);

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

    fwrite(idx1_buf, 1, 4, avifile);

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
        fread ( AteBytes, 1, 8, idxfile);
        fwrite(dc_buf, 1, 4, avifile);
        fwrite(zero_buf, 1, 4, avifile);
        fwrite(AteBytes, 1, 8, avifile);
    }

    free(AteBytes);
    fclose(idxfile);
    fclose(avifile);
    remove("/sdcard/idx.tmp");

    String fname_status = fname;

    fname_status = fname_status + String(".todo");
    
    FILE* f = fopen(fname_status.c_str(), "w");
    fwrite(fname, 1, strlen(fname), f);
    fwrite("\n0\n", 1, 3, f);
    fclose(f);

    
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

void loop()
{


  if (DEEP_SLEEP_PIR) {
      if (recording == 0) {
          Serial.println("Not recording");

          if (PIR_ENABLED && uploading == 0) {

              delay(10000);                                     //   wait 10 seoonds for another event before sleep
              
              Serial.println("Trying again ..  to sleep now?" );
              Serial.print("recording: "); Serial.println(recording);

              if (uploading == 0) {

                  Serial.println("Going to sleep now");
                  digitalWrite(GPIO_NUM_13, LOW);
                  gpio_hold_en(GPIO_NUM_13);
                  
                  pinMode(WHITE_LIGHT_PIN, OUTPUT);
                  digitalWrite(WHITE_LIGHT_PIN, LOW);
                  rtc_gpio_hold_en(WHITE_LIGHT_PIN);
                  gpio_deep_sleep_hold_en();
                  digitalWrite(RED_LIGHT_PIN, HIGH);
                  //rtc_gpio_hold_en(RED_LIGHT_PIN);

                  epoch_time = now();
                  esp_sleep_enable_ext0_wakeup(PIR_PIN, 0);
                  delay(500);
                  esp_deep_sleep_start();
              }
          }
          else {
              Serial.println("Uploading");
          }
      }
      else {
          Serial.println("Still recording");
      }
  }


  delay(1000);
}



