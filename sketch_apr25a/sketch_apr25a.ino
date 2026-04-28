#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <HX711_ADC.h>
#include <DHT.h>
#include <Wire.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

// Wi-Fi параметри
const char* ssid = "YourSSID";
const char* password = "YourPassword";

// MQTT параметри
const char* mqtt_server = "mqtt_server_address";
const char* mqtt_username = "your_mqtt_username";
const char* mqtt_password = "your_mqtt_password";
const char* mqtt_topic = "hive_data";

// Піни для датчиків
const int HX711_DOUT_PIN = D5; // Пін DOUT датчика ваги HX711
const int HX711_SCK_PIN = D6; // Пін SCK датчика ваги HX711
const int DHT_PIN = D2; // Пін датчика температури та вологості DHT22
const int HALL_SENSOR_PIN = D1; // Пін датчика відкриття вулика

// Клас для підключення до Wi-Fi
class WiFiConnection {
public:
    void connect() {
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.print(".");
        }
        Serial.println("Connected to WiFi");
    }
};

// Клас для підключення до MQTT
class MQTTConnection {
private:
    WiFiClient& client;
    PubSubClient mqttClient;
public:
    MQTTConnection(WiFiClient& client) : client(client), mqttClient(client) {}

    void connect() {
        mqttClient.setServer(mqtt_server, 1883);
        while (!mqttClient.connected()) {
            if (mqttClient.connect("ESP8266Client", mqtt_username, mqtt_password)) {
                Serial.println("Connected to MQTT server");
            } else {
                Serial.print("Failed to connect to MQTT server, rc=");
                Serial.print(mqttClient.state());
                Serial.println(" Retrying...");
                delay(5000);
            }
        }
    }

    void publish(const String& message) {
        mqttClient.publish(mqtt_topic, message.c_str());
    }

    bool isConnected() {
        return mqttClient.connected();
    }
};

// Клас для отримання даних ваги
class StrainGauge {
private:
    HX711_ADC hx711;
public:
    float getData() {
        return hx711.readScale();
    }
};

// Клас для вимірювання температури та вологості
class DHTSensor {
private:
    DHT dht;
public:
    DHTSensor(int pin) : dht(pin, DHT22) {
        dht.begin();
    }

    float readTemperature() {
        return dht.readTemperature();
    }

    float readHumidity() {
        return dht.readHumidity();
    }
};

// Клас для отримання часу з Інтернету
class TimeClient {
private:
    WiFiUDP ntpUDP;
    NTPClient timeClient;
public:
    TimeClient() : timeClient(ntpUDP, "pool.ntp.org") {
        timeClient.begin();
    }

    String getTime() {
        timeClient.update();
        return timeClient.getFormattedTime();
    }
};

// Клас для збереження даних в EEPROM
class DataStorage {
public:
    void saveDataToEEPROM(int address, const void* data, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            EEPROM.write(address + i, *((uint8_t*)data + i));
        }
        EEPROM.commit();
    }
};

// Головний клас для моніторингу вулика
class HiveMonitor {
private:
    WiFiConnection wifiConnection;
    MQTTConnection mqttConnection;
    StrainGauge strainGauge;
    DHTSensor dhtSensor;
    TimeClient timeClient;
    DataStorage dataStorage;
    float lastWeight = 0;
    bool hiveOpened = false;
    unsigned long lastMeasurementTime = 0;
public:
    HiveMonitor() : mqttConnection(wifiConnection), strainGauge(), dhtSensor(DHT_PIN), timeClient() {}

    // Ініціалізація
    void setup() {
        Serial.begin(9600);
        wifiConnection.connect();
        mqttConnection.connect();
        lastWeight = strainGauge.getData();
        lastMeasurementTime = millis();
    }

    // Головний цикл роботи
    void loop() {
        if (millis() - lastMeasurementTime >= 1800000) { // Кожні 30 хвилин
            wakeUp();
            float currentWeight = getWeight();
            float weightDifference = abs(currentWeight - lastWeight);
            float temperature = dhtSensor.readTemperature();
            float humidity = dhtSensor.readHumidity();
            bool error = false;

            if (!hiveOpened && weightDifference > 0.5) {
                // Перевірка ваги
                float weight1 = strainGauge.getData();
                delay(100);
                float weight2 = strainGauge.getData();
                delay(100);
                float weight3 = strainGauge.getData();

                if (abs(weight1 - weight2) < 0.1 && abs(weight2 - weight3) < 0.1) {
                    currentWeight = (weight1 + weight2 + weight3) / 3.0;
                } else {
                    error = true;
                }
            }

            if (!error) {
                String message = buildMessage(currentWeight, temperature, humidity, hiveOpened);
                mqttConnection.publish(message);
                lastWeight = currentWeight;
                lastMeasurementTime = millis();
            }

            hiveOpened = false;
            if (error) {
                // Збереження даних в EEPROM
                saveDataToEEPROM();
            }

            // Засинання
            deepSleep();
        }
    }

private:
    // Побудова повідомлення для відправки на сервер MQTT
    String buildMessage(float weight, float temperature, float humidity, bool hiveOpened) {
        String message = String(weight) + "," + String(temperature) + "," + String(humidity);
        if (hiveOpened) {
            message += ",1";
        } else {
            message += ",0";
        }
        return message;
    }

    // Отримання ваги
    float getWeight() {
        return strainGauge.getData();
    }

    // Прокинути пристрій
    void wakeUp() {
        // Прокинутися з глибокого сну
    }

    // Збереження даних в EEPROM
    void saveDataToEEPROM() {
        // Збереження даних
    }

    // Перевірка відкриття вулика
    bool checkHiveOpening() {
        return digitalRead(HALL_SENSOR_PIN) == HIGH;
    }

    // Засинання
    void deepSleep() {
        ESP.deepSleep(0, WAKE_RF_DEFAULT);
    }
};

// Глобальний об'єкт моніторингу вулика
HiveMonitor monitor;

// Функція setup() Arduino
void setup() {
    monitor.setup();
}

// Функція loop() Arduino
void loop() {
    monitor.loop();
}
