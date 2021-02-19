#include <Wire.h>
#include <MPU6050.h>

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
const int echoPin = 32;
const int accel_scl = 19; // These are unused, they are the defaults in arduino Wire library
const int accel_sda = 21;

const int post_action_delay = 200;
const int accelerometer_period = 250;

float duration;
int count = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  xTaskCreate(bucket_detect_tsk, "bucket", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  xTaskCreate(on_target_detect_tsk, "ontarget", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
}

int64_t miss_start = 0;
int64_t miss_timeout = 600*1000;
bool potential_miss = false;

void loop() {
  if(bucket){
    Serial.println("Flash green!");
    bucket = false;
    potential_miss = false;
  }

  if(on_target){
    potential_miss = true;
    miss_start = esp_timer_get_time();
    on_target = false;
  }

  if(potential_miss && esp_timer_get_time() - miss_start > miss_timeout){
    Serial.println("Flash red!");
    on_target = false;
  }
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
    //distance = (duration*.0343)/2;

    if(duration > 0 && duration < 1000){
      Serial.print("BUCKET! ");
      Serial.println(duration);
      delay(post_action_delay);
    }
  }
  vTaskDelete(nullptr);
}

// Detects shots on target using accelerometer(MPU5060)
void on_target_detect_tsk(void *arg){
  Vector prevAccel;

  for(;;){
    Vector accel = mpu.readNormalizeAccel();

    bool xTrig = abs(accel.XAxis - prevAccel.XAxis) > xThresh;
    bool yTrig = abs(accel.YAxis - prevAccel.YAxis) > yThresh;
    bool zTrig = abs(accel.ZAxis - prevAccel.ZAxis) > zThresh;

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
      delay(post_action_delay);
    }

    prevAccel.XAxis = accel.XAxis;
    prevAccel.YAxis = accel.YAxis;
    prevAccel.ZAxis = accel.ZAxis;

    delay(accelerometer_period);
  }
  vTaskDelete(nullptr);
}
