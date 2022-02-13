static const char devname[] = "desklens";         // name of your camera for mDNS, Router, and filenames

#define DELETE_OLD_FILES 1       // set to 1 and it will delete your oldest day of files so you SD is always 10% empty

// https://sites.google.com/a/usapiens.com/opnode/time-zones  -- find your timezone here
#define TIMEZONE "GMT0BST,M3.5.0/01,M10.5.0/02"             // your timezone  -  this is GMT


// reboot startup parameters here

#define DEEP_SLEEP_PIR 1            // set to 1 to deepsleep between pir videos
#define RECORD_ON_REBOOT 1          // set to 1 to record, or 0 to NOT record on reboot

#define PIR_ENABLED 1
#define DEEP_SLEEP_PIR 1

// here are 2 sets of startup parameters -- more down in the stop and restart webpage

// VGA 10 fps for 30 minutes, and repeat, play at real time

#define REPEAT_CONFIG 0                 //  repaeat same movie this many times
#define XSPEED 1                   //  playback speed - realtime is 1, or 300 means playpack 30 fps of frames at 10 second per frame ( 30 fps / 0.1 fps ) 
#define GRAY 0                     //  not gray
#define QUALITY 12                //  quality on the 10..50 subscale - 10 is good, 20 is grainy and smaller files, 12 is better in bright sunshine due to clipping
#define CAPTURE_INTERVAL 100       //  milli-seconds between frames
#define TOTAL_FRAMES_CONFIG 100;  //  how many frames - length of movie in ms is total_frames x capture_interval

