#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define ALARM_PIN 19
#define MAX_ALARM_DURATION_IN_MSEC 60000

typedef struct Message {
    bool alarm;
} Message;

bool currentAlarmState;
int alarmStartedAt;
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    auto data = (Message *) incomingData;
    if (currentAlarmState == data->alarm) {
        return;
    }
    currentAlarmState = data->alarm;
    if (data->alarm) {
        alarmStartedAt = millis();
        digitalWrite(ALARM_PIN, LOW);
    } else {
        digitalWrite(ALARM_PIN, HIGH);
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
    if (currentAlarmState && (millis() - alarmStartedAt) > MAX_ALARM_DURATION_IN_MSEC) {
        currentAlarmState = false;
        digitalWrite(ALARM_PIN, HIGH);
    }
}
