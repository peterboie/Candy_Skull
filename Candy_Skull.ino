//for the distance sensor
#include "Adafruit_VL53L1X.h"
//for the servo board
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//for the dfminiplayer
#include <DFMiniMp3.h>

#define SERVO_FREQ 50  // Analog servos run at ~50 Hz

//hardware serial for dfplayer mini connection
#define RX 16
#define TX 17

//calls default address of 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//variables for input on servo board
const int jaw_servo_num = 0;
const int brain_servo_num = 1;

//the min and max that the jaw can go to talk
const int jaw_max = 565;
const int jaw_min = 520;

//the min and max for the brain cap servo arm to open cap
const int brain_max = 400;
const int brain_min = 150;

//increase these delays for slower movements, decrease for faster
const int jaw_delay = 15;
const int brain_delay = 30;

//activate_distance the longest distance that the candy skull will activate at. The boo will activate between activate_distance and boo_max_distance
const int activate_distance = 460;  //in mm
const int boo_max_distance = 620;   //in mm

//distance sensor
#define IRQ_PIN 2
#define XSHUT_PIN 3
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

//the number slot on the servo board the LEDs are connected to
const int left_eye_num = 2;
const int right_eye_num = 3;

//volume for df mini player
const int volume = 30;  //0-30

class Mp3Notify;
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3;
DfMp3 dfmp3(Serial2);

//not sure exactly what this does but is needed for dfmini player:)
class Mp3Notify {
public:
  static void PrintlnSourceAction(DfMp3_PlaySources source, const char* action) {
    if (source & DfMp3_PlaySources_Sd) {
      Serial.print("SD Card, ");
    }
    if (source & DfMp3_PlaySources_Usb) {
      Serial.print("USB Disk, ");
    }
    if (source & DfMp3_PlaySources_Flash) {
      Serial.print("Flash, ");
    }
    Serial.println(action);
  }
  static void OnError([[maybe_unused]] DfMp3& mp3, uint16_t errorCode) {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    Serial.println(errorCode);
  }
  static void OnPlayFinished([[maybe_unused]] DfMp3& mp3, [[maybe_unused]] DfMp3_PlaySources source, uint16_t track) {
    Serial.print("Play finished for #");
    Serial.println(track);

    // start next track
    track += 1;
    // this example will just start back over with 1 after track 3
    if (track > 3) {
      track = 1;
    }
    dfmp3.playMp3FolderTrack(track);  // sd:/mp3/0001.mp3, sd:/mp3/0002.mp3, sd:/mp3/0003.mp3
  }
  static void OnPlaySourceOnline([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "online");
  }
  static void OnPlaySourceInserted([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "inserted");
  }
  static void OnPlaySourceRemoved([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "removed");
  }
};

void eat_my_brain() {
  dfmp3.playFolderTrack(1, 1);
  for (int cycle = 0; cycle < 3; cycle++) {
    for (uint16_t pulselen = jaw_max; pulselen > jaw_min; pulselen--) {
      pwm.setPWM(jaw_servo_num, 0, pulselen);
      delay(jaw_delay);
      Serial.println(pulselen);
    }
    for (uint16_t pulselen = jaw_min; pulselen < jaw_max; pulselen++) {
      pwm.setPWM(jaw_servo_num, 0, pulselen);
      delay(jaw_delay);
      //Serial.println(pulselen);
    }
  }
}

void brain_cap_open() {
  for (uint16_t pulselen = brain_min; pulselen < brain_max; pulselen++) {
    pwm.setPWM(brain_servo_num, 0, pulselen);
    delay(brain_delay);
    //Serial.println(pulselen);
  }
}


void brain_cap_closed() {
  for (uint16_t pulselen = brain_max; pulselen > brain_min; pulselen--) {
    pwm.setPWM(brain_servo_num, 0, pulselen);
    delay(brain_delay);
    //Serial.println(pulselen);
  }
  Serial.println("cap closed");
}

void boo() {
  pwm.setPWM(left_eye_num, 0, 4095);
  pwm.setPWM(right_eye_num, 0, 4095);
  dfmp3.playFolderTrack(1, 2);
  for (uint16_t pulselen = jaw_max; pulselen > jaw_min; pulselen--) {
    pwm.setPWM(jaw_servo_num, 0, pulselen);
    delay(jaw_delay);
    Serial.println(pulselen);
  }
  for (uint16_t pulselen = jaw_min; pulselen < jaw_max; pulselen++) {
    pwm.setPWM(jaw_servo_num, 0, pulselen);
    delay(brain_delay);
    //Serial.println(pulselen);
  }
  pwm.setPWM(left_eye_num, 0, 0);
  pwm.setPWM(right_eye_num, 0, 0);
}

void setup() {
  Serial2.begin(9600, SERIAL_8N1, RX, TX);
  Serial.begin(115200);
  while (!Serial) delay(10);
  //begin sensor
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  //servo board setup
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //df mini player setup
  dfmp3.begin(RX, TX, 9600);
  dfmp3.setVolume(volume);  //0-30

  delay(10);
}

void loop() {
  int16_t distance;
  distance = vl53.distance();
  if (distance < activate_distance && distance > 1) {
    Serial.println("Activated. Distance is: ");
    Serial.println(distance);
    pwm.setPWM(left_eye_num, 0, 4095);
    pwm.setPWM(right_eye_num, 0, 4095);

    eat_my_brain();

    brain_cap_open();
    delay(6000);
    brain_cap_closed();
    pwm.setPWM(left_eye_num, 0, 0);
    pwm.setPWM(right_eye_num, 0, 0);
  } else if (distance > activate_distance && distance < boo_max_distance) {
    Serial.println("Activated. Distance is: ");
    Serial.println(distance);
    boo();
    delay(2000);
  }
}