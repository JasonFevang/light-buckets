const int trigPin = 33;
const int echoPin = 32;

float duration, distance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("Hello World!");
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 1500);
  distance = (duration*.0343)/2;
  
  if(distance > 0){
    Serial.print("Distance: ");
    Serial.println(distance);
    delay(99);
  }
  delay(1);
}
