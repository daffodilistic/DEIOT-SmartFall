#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>

#define LED_PIN_RED 10
#define FALL_LED_FLASH_INTERVAL_MS 500
#define FALL_ALARM_DURATION_MS 10000
#define FALL_DETECTION_MS 1000
#define FALL_ACCELERATION_G 14.715
#define MQTT_ENDPOINT "broker.emqx.io"
#define MQTT_PORTNUM 1883

unsigned long previousMs = 0;
unsigned long ledPreviousMs = 0;
bool ledRedOn = false;
bool hasFallen = false;
char* WIFI_SSID = "Wokwi-GUEST";
char* WIFI_PASSWORD = "";
char* MQTT_DEFAULT_TOPIC = "DEIOT/SmartFall";
char MQTT_CLIENT_NAME[32] = "SmartFall_";

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MPU6050 mpu;
sensors_event_t acc, gcc, temp;

void led_flash(unsigned long currentMs) {
  unsigned long duration = currentMs - ledPreviousMs;
  // Serial.println(duration);
  if (duration >= FALL_LED_FLASH_INTERVAL_MS) {
    ledRedOn = !ledRedOn;
    ledPreviousMs = currentMs;
  }

  if (ledRedOn) {
    digitalWrite(LED_PIN_RED, HIGH);
  } else {
    digitalWrite(LED_PIN_RED, LOW);
  }
}

void mqtt_generateClientName() {
  char *buffer = &MQTT_CLIENT_NAME[10];
  randomSeed(analogRead(0));
  for (int i = 0; i < 12; i++) {
    bool isNumber = random(0, 2);
    if (isNumber) {
      // Use ASCII table trickery because of laziness
      buffer[i] = (char) random(48, 58);
    } else {
      // Use ASCII table trickery because of laziness
      buffer[i] = (char) random(65, 91);
    }
  }
  //(MQTT_CLIENT_NAME+1) = buffer[0];
  Serial.print("Generated client name is ");
  Serial.println(MQTT_CLIENT_NAME);
}

void mqtt_connect()
{
  while (!client.connected() )
  {
    Serial.println("Connecting to MQTT ...");
    if (client.connect(MQTT_CLIENT_NAME) ) //, mqttUser, mqttPassword) )
    {
      Serial.println("Connected");
      client.subscribe(MQTT_DEFAULT_TOPIC);
    }
    else
    {
      Serial.print("Failed with state ");
      Serial.println(client.state() );
      delay(2000);
    }
  }
}

void mqtt_publish(char* topic, char* payload)
{
  char buffer[256];
  sprintf(buffer, "%s", payload);
  client.publish(topic, buffer);
}

void mqtt_setup() {
  mqtt_generateClientName();
  client.setServer(MQTT_ENDPOINT, MQTT_PORTNUM);
}

void wifi_setup()
{
  Serial.print("Connecting to Wi-Fi ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED )
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("Given IP by the router to ESP32 is ");
  Serial.println(WiFi.localIP());
}

void setup() {
  unsigned long startTime = millis();
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, ESP32!");

  pinMode(LED_PIN_RED, OUTPUT);

  if (!mpu.begin()) {
    while (1) {
      delay(20);
    }
  }

  wifi_setup();
  mqtt_setup();

  unsigned long initTime = millis() - startTime;
  Serial.print("Finished setup in ");
  Serial.print(initTime);
  Serial.println("ms");
}

void loop() {
  if ( !client.connected() )
  {
    mqtt_connect();
  }
  client.loop();

  unsigned long currentMs = millis();
  if (hasFallen == false) {
    if (currentMs - previousMs >= FALL_DETECTION_MS) {
      // TODO should calibrate and use big data/ML to detect a fall!
      mpu.getEvent(&acc, &gcc, &temp);

      Serial.print("Z-axis acceleration: ");
      Serial.println(acc.acceleration.z);
      float z_acc = acc.acceleration.z;
      if (z_acc >= FALL_ACCELERATION_G) {
        char buffer[256];
        sprintf(buffer, "%s,EVENT_FALL,%f", MQTT_CLIENT_NAME, z_acc);
        Serial.println(buffer);
        mqtt_publish(MQTT_DEFAULT_TOPIC, buffer);

        hasFallen = true;
      }

      previousMs = currentMs;
      ledPreviousMs = currentMs;
    }
  } else {
    if (currentMs - previousMs <= FALL_ALARM_DURATION_MS) {
      led_flash(currentMs);
    } else {
      digitalWrite(LED_PIN_RED, LOW);
      previousMs = currentMs;
      hasFallen = false;
    }
  }

  delay(10); // this speeds up the simulation
}
