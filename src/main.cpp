#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "M5Unified.h"
// #include <Adafruit_MPU6050.h>

#define LED_PIN_RED 10
#define ANALOG_PIN 34
#define FALL_LED_FLASH_INTERVAL_MS 500
#define FALL_ALARM_DURATION_MS 10000
#define FALL_DETECTION_MS 500
#define FALL_ACCELERATION_G 2
#define MQTT_ENDPOINT "broker.emqx.io"
#define MQTT_PORTNUM 1883
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"
#define BLE_RESET_MS 500
#define BLE_DATA_INTERVAL_MS 3000
#define FALL_LED_FLASH_INTERVAL_MS 500

unsigned long previousMs = 0;
unsigned long ledPreviousMs = 0;
unsigned long blePreviousResetMs = 0;
unsigned long blePreviousDataIntervalMs = 0;
float z_acc = 0.0f;
bool ledRedOn = false;
bool hasFallen = false;
char* WIFI_SSID = "Wokwi-GUEST";
char* WIFI_PASSWORD = "";
char MQTT_DEFAULT_TOPIC[] = "DEIOT/SmartFall";
char MQTT_CLIENT_NAME[32] = "SmartFall_";

WiFiClient espClient;
PubSubClient client(espClient);

BLEServer *pServer = NULL;
BLECharacteristic *pSensorCharacteristic = NULL;
BLECharacteristic *pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Adafruit_MPU6050 mpu;
// sensors_event_t acc, gcc, temp;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pLedCharacteristic)
  {
    std::string value = pLedCharacteristic->getValue();
    if (value.length() > 0)
    {
      Serial.print("Characteristic event, written: ");
      Serial.println(static_cast<int>(value[0])); // Print the integer value

      int receivedValue = static_cast<int>(value[0]);
      if (receivedValue == 1)
      {
        digitalWrite(LED_PIN_RED, LOW);
      }
      else
      {
        digitalWrite(LED_PIN_RED, HIGH);
      }
    }
  }
};

void ble_setup()
{
  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
      SENSOR_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
      LED_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE);

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void ble_loop()
{
  unsigned long currentMs = millis();
  if (deviceConnected)
  {
    // connecting (set device state)
    if (!oldDeviceConnected)
    {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
      Serial.println("Device Connected");
    }
    else
    {
      // bluetooth stack will go into congestion if too many packets are sent,
      // in 6 hours test i was able to go as low as 3ms
      if (currentMs - blePreviousDataIntervalMs >= BLE_DATA_INTERVAL_MS)
      {
        // notify changed value
        pSensorCharacteristic->setValue(String(z_acc).c_str());
        pSensorCharacteristic->notify();
        Serial.print("New value notified: ");
        Serial.println(z_acc);
        blePreviousDataIntervalMs = currentMs;
      }
    }
  }
  else
  {
    // disconnecting (unset device state)
    if (oldDeviceConnected)
    {
      // give the bluetooth stack the chance to get things ready
      if (currentMs - blePreviousResetMs >= BLE_RESET_MS)
      {
        Serial.println("Device disconnected.");
        pServer->startAdvertising(); // restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
        blePreviousResetMs = currentMs;
      }
    }
  }
}

void led_flash(unsigned long currentMs)
{
  unsigned long duration = currentMs - ledPreviousMs;
  // Serial.println(duration);
  if (duration >= FALL_LED_FLASH_INTERVAL_MS)
  {
    ledRedOn = !ledRedOn;
    ledPreviousMs = currentMs;
  }

  if (ledRedOn)
  {
    digitalWrite(LED_PIN_RED, LOW);
  }
  else
  {
    digitalWrite(LED_PIN_RED, HIGH);
  }
}

void mqtt_generateClientName()
{
  char *buffer = &MQTT_CLIENT_NAME[10];
  randomSeed(analogRead(ANALOG_PIN));
  for (int i = 0; i < 12; i++)
  {
    bool isNumber = random(0, 2);
    if (isNumber)
    {
      // Use ASCII table trickery because of laziness
      buffer[i] = (char)random(48, 58);
    }
    else
    {
      // Use ASCII table trickery because of laziness
      buffer[i] = (char)random(65, 91);
    }
  }
  //(MQTT_CLIENT_NAME+1) = buffer[0];
  Serial.print("Generated client name is ");
  Serial.println(MQTT_CLIENT_NAME);
}

void mqtt_connect()
{
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT ...");
    if (client.connect(MQTT_CLIENT_NAME)) //, mqttUser, mqttPassword) )
    {
      Serial.println("Connected");
      client.subscribe(MQTT_DEFAULT_TOPIC);
    }
    else
    {
      Serial.print("Failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void mqtt_publish(char *topic, char *payload)
{
  char buffer[256];
  sprintf(buffer, "%s", payload);
  client.publish(topic, buffer);
}

void mqtt_setup()
{
  mqtt_generateClientName();
  client.setServer(MQTT_ENDPOINT, MQTT_PORTNUM);
}

float mpu_get_acceleration_z()
{
  float z_accel = 0.0f;
  // TODO should calibrate and use big data/ML to detect a fall!
  // mpu.getEvent(&acc, &gcc, &temp);

  // Serial.print("Z-axis acceleration: ");
  // Serial.println(acc.acceleration.z);

  auto imu_update = M5.Imu.update();
  if (imu_update)
  {
    auto data = M5.Imu.getImuData();
    z_accel = data.accel.z;
    // char buffer[64];
    // sprintf(buffer, "ax:%f  ay:%f  az:%f", data.accel.x, data.accel.y, data.accel.z);
    // Serial.println(buffer);
  }
  else
  {
    M5.update();
  }

  return z_accel;
}

void mpu_setup()
{
  auto cfg = M5.config();
  cfg.serial_baudrate = 9600;
  M5.begin(cfg);

  const char *name;
  auto imu_type = M5.Imu.getType();
  // switch (imu_type)
  // {
  // case m5::imu_none:
  //   name = "not found";
  //   break;
  // case m5::imu_sh200q:
  //   name = "sh200q";
  //   break;
  // case m5::imu_mpu6050:
  //   name = "mpu6050";
  //   break;
  // case m5::imu_mpu6886:
  //   name = "mpu6886";
  //   break;
  // case m5::imu_mpu9250:
  //   name = "mpu9250";
  //   break;
  // case m5::imu_bmi270:
  //   name = "bmi270";
  //   break;
  // default:
  //   name = "unknown";
  //   break;
  // };
  // M5_LOGI("imu:%s", name);

  if (imu_type == m5::imu_none)
  {
    for (;;)
    {
      delay(1);
    }
  }

  // if (!mpu.begin()) {
  //   while (1) {
  //     delay(20);
  //   }
  // }
}

void display_ok()
{
  lgfx::v1::TextStyle ts = lgfx::v1::TextStyle();
  ts.back_rgb888 = 0x00FF00;
  ts.fore_rgb888 = 0x000000;
  // 0 = 270 deg / 1 = normal / 2 = 90 deg / 3 = 180 deg / 4~7 = upside down
  M5.Display.setRotation(3);
  M5.Display.setColor(0, 255, 0);
  M5.Display.fillRect(0, 0, 160, 80);
  M5.Display.setCursor(80 - (26 / 2), 40 - (26 / 2));
  // From https://m5stack.lang-ship.com/howto/m5gfx/font/
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextStyle(ts);
  M5.Display.print("OK");
}

void display_fall()
{
  lgfx::v1::TextStyle ts = lgfx::v1::TextStyle();
  ts.back_rgb888 = 0xFF0000;
  ts.fore_rgb888 = 0xFFFFFF;
  // 0 = 270 deg / 1 = normal / 2 = 90 deg / 3 = 180 deg / 4~7 = upside down
  M5.Display.setRotation(3);
  M5.Display.setColor(255, 0, 0);
  M5.Display.fillRect(0, 0, 160, 80);
  M5.Display.setCursor(80 - (26 / 2 * 2), 40 - (26 / 2));
  // From https://m5stack.lang-ship.com/howto/m5gfx/font/
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextStyle(ts);
  M5.Display.print("FALL");
}

void wifi_setup()
{
  Serial.print("Connecting to Wi-Fi ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
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

void setup()
{
  unsigned long startTime = millis();
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Hello, ESP32!");

  pinMode(LED_PIN_RED, OUTPUT);
  digitalWrite(LED_PIN_RED, HIGH);

  mpu_setup();
  wifi_setup();
  mqtt_setup();
  ble_setup();

  unsigned long initTime = millis() - startTime;
  Serial.print("Finished setup in ");
  Serial.print(initTime);
  Serial.println("ms");

  display_ok();
}

void loop()
{
  if (!client.connected())
  {
    mqtt_connect();
  }
  client.loop();

  ble_loop();

  unsigned long currentMs = millis();
  if (hasFallen == false)
  {
    if (currentMs - previousMs >= FALL_DETECTION_MS)
    {
      z_acc = mpu_get_acceleration_z();
      if (z_acc >= FALL_ACCELERATION_G)
      {
        char buffer[256];
        sprintf(buffer, "%s,EVENT_FALL,%f", MQTT_CLIENT_NAME, z_acc);
        Serial.println(buffer);
        mqtt_publish(MQTT_DEFAULT_TOPIC, buffer);

        hasFallen = true;
        display_fall();
      }

      previousMs = currentMs;
      ledPreviousMs = currentMs;
    }
  }
  else
  {
    if (currentMs - previousMs <= FALL_ALARM_DURATION_MS)
    {
      led_flash(currentMs);
    }
    else
    {
      digitalWrite(LED_PIN_RED, HIGH);
      previousMs = currentMs;
      hasFallen = false;
      display_ok();
    }
  }

  delay(10); // this speeds up the simulation
}
