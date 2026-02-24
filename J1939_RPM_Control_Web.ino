#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "driver/twai.h"

// 핀 설정
#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_21
const int STATUS_LED = 27;  
const int RESUME_PIN = 32;
const int UP_PIN     = 33;
const int DOWN_PIN   = 25;
const int STOP_PIN    = 26;

// J1939 설정
byte mySA = 0xE5; 
bool addressConfirmed = false;
unsigned long claimTimer = 0;
unsigned char myNAME[8] = {0x01, 0x00, 0x00, 0xE8, 0x00, 0x21, 0x00, 0x80};

// BLE 및 제어 변수
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
volatile int targetRPM = 0;   
volatile int currentRPM = 0;
enum ControlState { IDLE, RUNNING, STOPPING };
volatile ControlState currentState = IDLE;
byte rollingCount = 0;
unsigned long lastBtnTime = 0;

// [추가] 태스크 핸들 선언 (에러 해결용)
TaskHandle_t CAN_Task;

// 주소 청구 메시지 송신 함수 (PGN 60928)
void sendAddressClaim() {
    addressConfirmed = false;
    twai_message_t msg;
    msg.identifier = 0x18EEFF00 | mySA; 
    msg.extd = 1; 
    msg.data_length_code = 8;
    memcpy(msg.data, myNAME, 8);
    twai_transmit(&msg, pdMS_TO_TICKS(10));
    claimTimer = millis();
    Serial.printf(">>> J1939 Address Claim Sent: %d\n", mySA);
}

// CAN 드라이버 초기화
void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 20; 
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
}

// [Core 0] CAN 수신 및 네트워크 관리 테스크
void CAN_Loop(void * pvParameters) {
    twai_message_t rx_msg;
    for(;;) {
        while (twai_receive(&rx_msg, 0) == ESP_OK) {
            uint32_t id = rx_msg.identifier;
            byte pf = (id >> 16) & 0xFF;
            byte ps = (id >> 8) & 0xFF;
            byte senderSA = id & 0xFF;

            if (pf == 0xF0 && ps == 0x04) {
                currentRPM = (int)((rx_msg.data[4] * 256 + rx_msg.data[3]) * 0.125);
                digitalWrite(STATUS_LED, HIGH);
            }
            else if (pf == 0xEA && (ps == 0xFF || ps == mySA)) {
                if (rx_msg.data[0] == 0x00 && rx_msg.data[1] == 0xEE && rx_msg.data[2] == 0x00) {
                    byte targetDA = (ps == 0xFF) ? 0xFF : senderSA;
                    twai_message_t tx_msg;
                    tx_msg.identifier = 0x18EE0000 | (targetDA << 8) | mySA;
                    tx_msg.extd = 1; tx_msg.data_length_code = 8;
                    memcpy(tx_msg.data, myNAME, 8);
                    twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
                }
            }
            else if (pf == 0xEE && ps == 0xFF) {
                if (senderSA == mySA) {
                    if (memcmp(rx_msg.data, myNAME, 8) > 0) {
                        mySA = 0xFE; 
                        sendAddressClaim();
                    }
                }
            }
        }

        if (!addressConfirmed && (millis() - claimTimer >= 250)) {
            addressConfirmed = true;
            Serial.printf(">>> J1939 SA Confirmed: %d\n", mySA);
        }

        twai_status_info_t status;
        twai_get_status_info(&status);
        if (status.state == TWAI_STATE_BUS_OFF) {
            twai_stop(); twai_driver_uninstall();
            vTaskDelay(pdMS_TO_TICKS(50));
            initCAN();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; }
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; pServer->getAdvertising()->start(); }
};

class MyCharCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String v = pChar->getValue();
        if (v.length() >= 2) {
            uint16_t val = (uint8_t)v[0] | ((uint8_t)v[1] << 8);
            if (val == 1000 && currentState == IDLE) { currentState = RUNNING; targetRPM = 1000; }
            else if (val == 600 && currentState == RUNNING) { currentState = STOPPING; targetRPM = currentRPM; }
            else if (currentState == RUNNING) targetRPM = constrain(val, 1000, 2500);
        }
    }
};

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    pinMode(RESUME_PIN, INPUT_PULLUP); pinMode(UP_PIN, INPUT_PULLUP);
    pinMode(DOWN_PIN, INPUT_PULLUP); pinMode(STOP_PIN, INPUT_PULLUP);

    initCAN();
    sendAddressClaim();
    
    xTaskCreatePinnedToCore(CAN_Loop, "CAN_Task", 4096, NULL, 1, &CAN_Task, 0);

    BLEDevice::init("Truck_RPM_Control");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
    pCharacteristic = pService->createCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8", 
                BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->setCallbacks(new MyCharCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    pServer->getAdvertising()->start();
}

void loop() {
    unsigned long now = millis();

    if (now - lastBtnTime > 150) {
        if (digitalRead(RESUME_PIN) == LOW && currentState == IDLE) { currentState = RUNNING; targetRPM = 1000; lastBtnTime = now; }
        if (digitalRead(STOP_PIN) == LOW && currentState == RUNNING) { currentState = STOPPING; targetRPM = currentRPM; lastBtnTime = now; }
        if (currentState == RUNNING) {
            if (digitalRead(UP_PIN) == LOW) { targetRPM = min(targetRPM + 25, 2500); lastBtnTime = now; }
            if (digitalRead(DOWN_PIN) == LOW) { targetRPM = max(targetRPM - 25, 1000); lastBtnTime = now; }
        }
    }

    static unsigned long last100 = 0;
    if (now - last100 >= 100) {
        if (currentState == STOPPING) {
            targetRPM -= 25;
            if (targetRPM <= 700) { currentState = IDLE; targetRPM = 0; }
        }
        if (deviceConnected) {
            uint8_t tx[4] = { (uint8_t)(currentRPM & 0xFF), (uint8_t)(currentRPM >> 8), 
                             (uint8_t)(targetRPM & 0xFF), (uint8_t)(targetRPM >> 8) };
            pCharacteristic->setValue(tx, 4);
            pCharacteristic->notify();
        }
        Serial.printf("CUR:%4d | TAR:%4d | SA:%d\n", currentRPM, targetRPM, mySA);
        digitalWrite(STATUS_LED, LOW);
        last100 = now;
    }

    static unsigned long last10 = 0;
    if (now - last10 >= 10 && currentState != IDLE && addressConfirmed) {
        unsigned int val = (unsigned int)(targetRPM / 0.125);
        twai_message_t tx_msg;
        tx_msg.identifier = 0x0C000000 | (0x00 << 8) | mySA;
        tx_msg.extd = 1; tx_msg.data_length_code = 8;
        tx_msg.data[0] = 0x01; tx_msg.data[1] = (byte)(val & 0xFF); tx_msg.data[2] = (byte)(val >> 8);
        memset(&tx_msg.data[3], 0xFF, 4);
        byte ck = 0; for(int i=0; i<7; i++) ck ^= tx_msg.data[i];
        ck ^= (rollingCount & 0x0F); ck ^= mySA;
        tx_msg.data[7] = (ck << 4) | (rollingCount & 0x0F);
        twai_transmit(&tx_msg, pdMS_TO_TICKS(2));
        rollingCount = (rollingCount + 1) % 16;
        last10 = now;
    }
}