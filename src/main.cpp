#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Ticker.h"

#define ALARM_PIN 19
#define MAX_ALARM_DURATION_IN_MSEC 60000

SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

typedef struct Message {
    bool alarm;
    bool ping;
} Message;

class Semaphore {
public:
    static bool take() {
        return xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE;
    }

    static bool give() {
        return xSemaphoreGive(mutex);
    }
};

Ticker ticker;
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
    auto data = (Message *) incomingData;
    if (data->ping) {
        ticker.once_ms(1000, []() {
            if (Semaphore::take()) {
                for (auto i = 0; i < 3; i++) {
                    digitalWrite(ALARM_PIN, LOW);
                    delay(50);
                    digitalWrite(ALARM_PIN, HIGH);
                    delay(1000);
                }
                Semaphore::give();
            }
        });
    }

    if (Semaphore::take()) {
        if (alarmState == data->alarm) {
            Semaphore::give();
            return;
        }

        setAlarmState(data->alarm);
        if (alarmState) {
            digitalWrite(ALARM_PIN, LOW);
        } else {
            digitalWrite(ALARM_PIN, HIGH);
        }

        Semaphore::give();
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
    if (Semaphore::take()) {
        if (alarmState && (millis() - alarmStartedAt) > MAX_ALARM_DURATION_IN_MSEC) {
            alarmState = false;
            digitalWrite(ALARM_PIN, HIGH);
        }
        Semaphore::give();
    }
}
