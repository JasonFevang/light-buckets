#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN    14
#define LED_COUNT 60

// Pixel controller
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Accelerometer interface class
MPU6050 mpu;

bool bucket = false;
bool on_target = false;

// Thresholds for triggering motion in each direction
const int xThresh = 2;
const int yThresh = 2;
const int zThresh = 2;

// Pins for HC-SR04 ultrasonic sensor
const int trigPin = 33;
const int echoPin = 25;
const int accel_scl = 19; // These are unused, they are the defaults in arduino Wire library
const int accel_sda = 21;

const int post_action_delay = 900;
const int accelerometer_period = 250;

float duration;
int count = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)
  
  xTaskCreate(bucket_detect_tsk, "bucket", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  xTaskCreate(on_target_detect_tsk, "ontarget", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);

  // Give time for sensors to send through inital data before executing control logic
  delay(500);
  bucket = false;
  on_target = false;
}

int64_t miss_start = 0;
int64_t miss_timeout = 600*1000;
bool potential_miss = false;

void loop() {
  if(bucket){
    bucket = false;
    potential_miss = false;
    Serial.println("Flash green!");
    for(int i = 0; i < 3; i++){
      strip.fill(0x00ff00, 0, strip.numPixels());
      strip.show();
      delay(200);
      strip.fill(0, 0, strip.numPixels());
      strip.show();
      delay(200);
    }
    delay(1000);
    on_target = false;
    potential_miss = false;
  }

  // on_target is controlled by the accelerometer task
  if(!potential_miss && on_target){
    potential_miss = true;
    miss_start = esp_timer_get_time();
  }

  if(potential_miss && esp_timer_get_time() - miss_start > miss_timeout){
    Serial.println("Flash red!");
    strip.fill(0xff0000, 0, strip.numPixels());
    strip.show();
    delay(300);
    strip.fill(0, 0, strip.numPixels());
    strip.show();
    potential_miss = false;
    on_target = false;
  }

  delay(10);
}

// Detects buckets using ultrasonic sensor
void bucket_detect_tsk(void *arg){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW);
  for(;;){
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    //Serial.println(duration);
    //distance = (duration*.0343)/2;

    if(duration > 0 && duration < 400){
      Serial.print("bucket detect");
      Serial.println(duration);
      bucket = true;
      delay(post_action_delay);
    }
    
    delay(1);
  }
  vTaskDelete(nullptr);
}

// Detects shots on target using accelerometer(MPU5060)
void on_target_detect_tsk(void *arg){
  Vector prevAccel;
  if(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    vTaskDelete(nullptr);
  }
  
  for(;;){
    Vector accel = mpu.readNormalizeAccel();

    bool xTrig = abs(accel.XAxis - prevAccel.XAxis) > xThresh;
    bool yTrig = abs(accel.YAxis - prevAccel.YAxis) > yThresh;
    bool zTrig = abs(accel.ZAxis - prevAccel.ZAxis) > zThresh;

    if(!on_target){
      if(xTrig){
        Serial.print("x");
      }
      if(yTrig){
        Serial.print("y");
      }
      if(zTrig){
        Serial.print("z");
      }
  
      if(xTrig || yTrig || zTrig){
        Serial.println(" -trig");
        on_target = true;
      }
    }

    prevAccel.XAxis = accel.XAxis;
    prevAccel.YAxis = accel.YAxis;
    prevAccel.ZAxis = accel.ZAxis;

    delay(accelerometer_period);
  }
  vTaskDelete(nullptr);
}
