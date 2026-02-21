#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Servo.h>
#include <MPU9250_WE.h>
#include <PubSubClient.h>

const char* ssid = "";       
const char* password = "";

const char* mqtt_server = "[...].cloud.shiftr.io";
const char* mqtt_user = "";
const char* mqtt_pass = "";
const char* mqtt_topic = ""; 

WiFiClient espClient;
PubSubClient client(espClient);

#define MPU9250_ADDR 0x68 
const int pinSDA = D1;
const int pinSCL = D2;
const int pinServoYaw = D5;
const int pinServoPitch = D6;


float alpha = 0.5; 
float Kp_pitch = 0.1501;
float Ki_pitch = 0.3776;
float Kd_pitch = 0.0080;
float Kp_yaw = 0.1501;
float Ki_yaw = 0.3776;
float Kd_yaw = 0.0080;
float MAX_INTEGRAL = 238.0;

MPU9250_WE myMPU = MPU9250_WE(MPU9250_ADDR);
Servo servoYaw;
Servo servoPitch;

float initialPitch = 0.0;
float initialYaw = 0.0;
float rawPitch, rawYaw;      
float filteredPitch, filteredYaw; 

float errorPitch, integralPitch, previousErrorPitch;
float errorYaw, integralYaw, previousErrorYaw;
float dt = 0.0;

xyzFloat angles; 
unsigned long previousMpuMillis = 0;
const long mpuInterval = 10;

unsigned long previousMqttMillis = 0;
const long mqttInterval = 100;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Lucky";
    clientId += String(random(0xffff), HEX);
    
    // Connect dengan User & Password baru
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  setup_wifi(); 
  client.setServer(mqtt_server, 1883);

  delay(2000);

  servoYaw.attach(pinServoYaw, 500, 2500);
  servoPitch.attach(pinServoPitch, 500, 2500);
  servoYaw.write(90);
  servoPitch.write(90);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  
  Wire.begin(pinSDA, pinSCL);
  Wire.setClock(400000);
  
  if(!myMPU.init()){
    Serial.println("MPU9250 error!");
    while(1);
  }
  Serial.println("MPU9250 connected.");

  if(!myMPU.initMagnetometer()){
    Serial.println("Magnetometer error!");
    while(1);
  }
  Serial.println("Magnetometer connected.");

  Serial.println("Calibrating...");
  delay(2000);
  myMPU.autoOffsets(); 
  Serial.println("Done.");

  myMPU.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(500);

  angles = myMPU.getAngles();
  initialPitch = angles.x;

  xyzFloat mag = myMPU.getMagValues();
  float calculatedYaw = atan2(mag.y, mag.x) * 180.0 / PI;
  initialYaw = calculatedYaw;

  filteredPitch = initialPitch;
  filteredYaw = initialYaw;
  
  previousMpuMillis = millis(); 
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMpuMillis >= mpuInterval){
    dt = (currentMillis - previousMpuMillis) / 1000.0;
    previousMpuMillis = currentMillis;

    angles = myMPU.getAngles();
    rawPitch = angles.x;
    xyzFloat mag = myMPU.getMagValues();
    rawYaw = atan2(mag.y, mag.x) * 180.0 / PI;

    filteredPitch = (alpha * rawPitch) + ((1.0 - alpha) * filteredPitch);
    filteredYaw   = (alpha * rawYaw)   + ((1.0 - alpha) * filteredYaw);
    
    errorPitch = 0 - (filteredPitch - initialPitch);
    integralPitch += errorPitch * dt;
    integralPitch = constrain(integralPitch, -MAX_INTEGRAL, MAX_INTEGRAL);
    float derivativePitch = (errorPitch - previousErrorPitch) / dt;
    float outputPitch = (Kp_pitch * errorPitch) + (Ki_pitch * integralPitch) + (Kd_pitch * derivativePitch);
    previousErrorPitch = errorPitch;

    float currentYawError = filteredYaw - initialYaw;
    while (currentYawError > 180) currentYawError -= 360;
    while (currentYawError < -180) currentYawError += 360;
    errorYaw = 0 - currentYawError;
    integralYaw += errorYaw * dt;
    integralYaw = constrain(integralYaw, -MAX_INTEGRAL, MAX_INTEGRAL);
    float derivativeYaw = (errorYaw - previousErrorYaw) / dt;
    float outputYaw = (Kp_yaw * errorYaw) + (Ki_yaw * integralYaw) + (Kd_yaw * derivativeYaw);
    previousErrorYaw = errorYaw;

    int servoValPitch = constrain(90 - outputPitch, 0, 180);
    int servoValYaw   = constrain(90 - outputYaw,   0, 180);

    servoPitch.write(servoValPitch);
    servoYaw.write(servoValYaw);
  }

  if (currentMillis - previousMqttMillis >= mqttInterval) {
    previousMqttMillis = currentMillis;

    String payload = "{";
    payload += "\"pitch\":"; payload += String(filteredPitch - initialPitch, 2); payload += ",";
    payload += "\"yaw\":"; payload += String(filteredYaw - initialYaw, 2);
    payload += "}";

    client.publish(mqtt_topic, payload.c_str());
  }
}
