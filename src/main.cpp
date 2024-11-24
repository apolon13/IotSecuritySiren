#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define ALARM_PIN 19
#define MAX_ALARM_DURATION_IN_MSEC 60000

typedef struct Message {
    bool alarm;
} Message;

SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
bool alarmState;
int alarmStartedAt;

void setAlarmState(bool state) {
    alarmState = state;
    if (alarmState) {
        alarmStartedAt = millis();
    } else {
        alarmStartedAt = 0;
    }
}


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        auto data = (Message *) incomingData;
        if (alarmState == data->alarm) {
            xSemaphoreGive(mutex);
            return;
        }
        setAlarmState(data->alarm);
        if (alarmState) {
            digitalWrite(ALARM_PIN, LOW);
        } else {
            digitalWrite(ALARM_PIN, HIGH);
        }
        xSemaphoreGive(mutex);
    }
}

void setup() {
    pinMode(ALARM_PIN, OUTPUT);
    digitalWrite(ALARM_PIN, HIGH);
    Serial.begin(9600);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        if (alarmState && (millis() - alarmStartedAt) > MAX_ALARM_DURATION_IN_MSEC) {
            alarmState = false;
            digitalWrite(ALARM_PIN, HIGH);
        }
        xSemaphoreGive(mutex);
    }
}
