#include <Arduino.h>
#include "esp_log.h"
#include "config.h"
#include "BatteryMonitor.h"
#include "Buzzer.h"
////#include "SoundController.h"
#include "ILedController.h"
#include "Ws28xxController.h"
#include "BleServer.h"
#include "CanBus.h"
#include "AppConfiguration.h"
#include "LightBarController.h"
#include <ble_ota_dfu.hpp>

const int mainBufSize = 128;
char mainBuf[mainBufSize];

#if defined(CANBUS_ENABLED) && defined(BMS_TX_PIN) && defined(BMS_ON_PIN)
  #include "BMSController.h"
#endif

unsigned long mainLoop = 0;
unsigned long loopTime = 0;
unsigned long maxLoopTime = 0;
int new_forward = LOW;
int new_backward = LOW;
int new_brake = LOW;
int idle = LOW;
int mall_grab = LOW;
double idle_erpm = 10.0;
boolean updateInProgress = false;

BLE_OTA_DFU ota_dfu_ble;

VescData vescData;

#ifndef CANBUS_ENABLED
  HardwareSerial vesc(2);
#endif

ILedController *ledController;

#if defined(CANBUS_ENABLED)
CanBus *canbus = new CanBus(&vescData);
#endif //CANBUS_ENABLED
BatteryMonitor *batMonitor = new BatteryMonitor(&vescData);

BleServer *bleServer = new BleServer();
LightBarController *lightbar = new LightBarController();

#if defined(CANBUS_ENABLED) && defined(BMS_TX_PIN) && defined(BMS_ON_PIN)
  BMSController *bmsController = new BMSController(&vescData);
#endif

// --- SPEED SENSOR & EN06 UART DEFINITIONS ---
#ifndef SPEED_SENSOR_PIN
 #define SPEED_SENSOR_PIN 5  // digital pin for SPEED_SENSOR
#endif //SPEED_SENSOR_PIN

#define PULSES_PER_REVOLUTION 6 // Adjust based on sensor (e.g., 6 magnets)
#define WHEEL_DIAMETER_M 0.66   // Adjust for your wheel (e.g., 26")

#include <HardwareSerial.h>
HardwareSerial EN06(1);

volatile unsigned long pulse_count = 0;
unsigned long last_pulse_time = 0;
float speed_kmh = 0;

void IRAM_ATTR speed_sensor_isr() {
    pulse_count++;
}

void calculate_speed() {
    static unsigned long last_calc = 0;
    if (millis() - last_calc < 500) return;
    last_calc = millis();
    unsigned long pulses = pulse_count;
    pulse_count = 0;
    unsigned long delta_time = millis() - last_pulse_time;
    last_pulse_time = millis();
    if (delta_time > 0) {
        float circumference = 3.14159f * WHEEL_DIAMETER_M;
        float revolutions_per_sec = (pulses / (float)PULSES_PER_REVOLUTION) / (delta_time / 1000.0f);
        speed_kmh = revolutions_per_sec * circumference * 3.6f;
    } else {
        speed_kmh = 0;
    }
}

void send_en06_packet() {
    static unsigned long last_send = 0;
    if (millis() - last_send < 500) return;
    last_send = millis();
    calculate_speed();
    uint8_t speed = (uint8_t)speed_kmh;
    uint8_t voltage = (uint8_t)vescData.inputVoltage; // Use vescData for voltage
    uint8_t current = (uint8_t)vescData.current;      // Use vescData for current
    uint8_t pas_level = 1; // Placeholder
    uint8_t packet[] = {0x46, speed, voltage, current, pas_level, 0x00};
    uint8_t checksum = 0;
    for (int i = 1; i < sizeof(packet) - 1; i++) {
        checksum += packet[i];
    }
    packet[sizeof(packet) - 1] = checksum;
    EN06.write(packet, sizeof(packet));
}
// --- END SPEED SENSOR & EN06 UART DEFINITIONS ---

void setup() {

  // give some time to avoid brownouts
  delay(500);

//Debug LED on board
#ifdef PIN_BOARD_LED
    pinMode(PIN_BOARD_LED,OUTPUT);
    digitalWrite(PIN_BOARD_LED,LOW);
#endif

    AppConfiguration::getInstance()->readPreferences();
 //   AppConfiguration::getInstance()->readMelodies();
    delay(10);
    AppConfiguration::getInstance()->config.sendConfig = false;

    if (esp_log_level_get("*") != ESP_LOG_NONE) {
#ifdef ESP32S3
        Serial.setRxBufferSize(2048);
#endif
        Serial.begin(VESC_BAUD_RATE);
    }

    if (AppConfiguration::getInstance()->config.otaUpdateActive) {
        return;
    }

    ledController = LedControllerFactory::getInstance()->createLedController(&vescData);

    #if defined(PIN_FORWARD) && defined(PIN_BACKWARD) && defined(PIN_BRAKE)
    pinMode(PIN_FORWARD, INPUT);
    pinMode(PIN_BACKWARD, INPUT);
    pinMode(PIN_BRAKE, INPUT);
    #endif

    delay(50);
#ifndef CANBUS_ENABLED
    vesc.begin(VESC_BAUD_RATE, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN, false);
#else
    // initializes the CANBUS
    canbus->init();
#endif //CANBUS_ENABLED

#if defined(CANBUS_ENABLED) && defined(BMS_TX_PIN) && defined(BMS_ON_PIN)
    bmsController->init(canbus);
#endif

    // initializes the battery monitor
    batMonitor->init();
    // initialize the UART bridge from VESC to BLE and the BLE support for Blynk (https://blynk.io)
#ifdef CANBUS_ONLY
    bleServer->init(canbus->stream, canbus);
#else
    bleServer->init(&vesc);
#endif
    // initialize the LED (either COB or Neopixel)
    ledController->init();
    lightbar->init();

    Buzzer::startSequence();
    ledController->startSequence();

    snprintf(mainBuf, mainBufSize, " sw-version %d.%d.%d is happily running on hw-version %d.%d",
             SOFTWARE_VERSION_MAJOR, SOFTWARE_VERSION_MINOR, SOFTWARE_VERSION_PATCH,
             HARDWARE_VERSION_MAJOR, HARDWARE_VERSION_MINOR);
    ESP_LOGI("rESCue", "%s", mainBuf);

#ifdef PIN_BOARD_LED
    digitalWrite(PIN_BOARD_LED,HIGH);
#endif
#ifndef DISPLAY_RX_PIN
 #define DISPLAY_RX_PIN 9  // digital pin for DISPLAY_RX_PIN
#endif //DISPLAY_RX_PIN

#ifndef DISPLAY_TX_PIN
 #define DISPLAY_TX_PIN 10  // digital pin for DISPLAY_RX_PIN
#endif //DISPLAY_TX_PIN
    // --- SPEED SENSOR & EN06 UART INIT ---
    EN06.begin(9600, SERIAL_8N1, DISPLAY_RX_PIN, DISPLAY_TX_PIN); // UART1 for EN06, adjust pins as needed
    pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), speed_sensor_isr, FALLING);
    // --- END SPEED SENSOR & EN06 UART INIT ---
}

void loop() {
    loopTime = millis() - mainLoop;
    mainLoop = millis();
    if (loopTime > maxLoopTime) {
        maxLoopTime = loopTime;
    }

    if (AppConfiguration::getInstance()->config.otaUpdateActive) {
        if(!updateInProgress) {
            Buzzer::startUpdateSequence();
            bleServer->stop();
            ota_dfu_ble.begin(AppConfiguration::getInstance()->config.deviceName.c_str()); 
            updateInProgress = true;
        }
        return;
    }

    if (AppConfiguration::getInstance()->config.sendConfig) {
        BleServer::sendConfig();
        AppConfiguration::getInstance()->config.sendConfig = false;
    }
    if (AppConfiguration::getInstance()->config.saveConfig) {
        AppConfiguration::getInstance()->savePreferences();
        //AppConfiguration::getInstance()->saveMelodies();
        AppConfiguration::getInstance()->config.saveConfig = false;

        // reboot to take over config, specially for the vesc can id
        ESP_LOGI("rESCue", "Rebooting to take over new saved config...");
        ESP.restart();
    }

#ifdef CANBUS_ENABLED
    new_forward = vescData.erpm > idle_erpm ? HIGH : LOW;
    new_backward = vescData.erpm < -idle_erpm ? HIGH : LOW;
    idle = (abs(vescData.erpm) < idle_erpm && vescData.switchState == 0) ? HIGH : LOW;
    new_brake = (abs(vescData.erpm) > idle_erpm && vescData.current < -4.0) ? HIGH : LOW;
    mall_grab = (vescData.pitch > 70.0) ? HIGH : LOW;
#else
    new_forward  = digitalRead(PIN_FORWARD);
    new_backward = digitalRead(PIN_BACKWARD);
    new_brake    = digitalRead(PIN_BRAKE);
    idle         = new_forward == LOW && new_backward == LOW;
    mall_grab    = LOW;
#endif

#ifdef CANBUS_ENABLED
    canbus->loop();
#endif

#if defined(CANBUS_ENABLED) && defined(BMS_TX_PIN) && defined(BMS_ON_PIN)
    bmsController->loop();
#endif

    // call the led controller loop
    ledController->loop(&new_forward, &new_backward, &idle, &new_brake, &mall_grab);

    // measure and check voltage
    batMonitor->checkValues();

    lightbar->updateLightBar(vescData.inputVoltage, vescData.switchState, vescData.adc1, vescData.adc2, vescData.erpm);  // update the WS28xx battery bar

    // call the VESC UART-to-Bluetooth bridge
    bleServer->loop(&vescData, loopTime, maxLoopTime);

    // --- SPEED SENSOR & EN06 UART LOOP ---
    send_en06_packet();
    // --- END SPEED SENSOR & EN06 UART LOOP ---
}