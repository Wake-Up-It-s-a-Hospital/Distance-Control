#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

const uint8_t PIN_RST = 15;
const uint8_t PIN_IRQ = 17; 
const uint8_t PIN_SS = 4;

#define led 2  // task2용 추가 
 
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

volatile byte expectedMsgId = POLL;
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
boolean protocolFailed = false;

uint64_t timePollSent, timePollReceived, timePollAckSent, timePollAckReceived;
uint64_t timeRangeSent, timeRangeReceived;
uint64_t timeComputedRange;

#define LEN_DATA 16
byte data[LEN_DATA];
uint32_t lastActivity;
uint32_t resetPeriod = 250;
uint16_t replyDelayTimeUS = 3000;
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

device_configuration_t DEFAULT_CONFIG = {
    false, true, true, true, false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
    true, true, true, false, true
};

void setupUWB();
void loopUWB();
void noteActivity();
void resetInactive();
void handleSent();
void handleReceived();
void transmitPollAck();
void transmitRangeReport(float curRange);
void transmitRangeFailed();
void receiver();

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(led,OUTPUT);
  xTaskCreatePinnedToCore(task1code, "UWB Anchor Task", 10000, NULL, 1, NULL, 0);
  delay(100);
  xTaskCreatePinnedToCore(task2code, "task2", 4000, NULL, 1, NULL, 1);
}

void task1code(void *parameter) {
  setupUWB();
  UBaseType_t remainingStack = uxTaskGetStackHighWaterMark(NULL);
  Serial.print("remain:");
  Serial.println(remainingStack);
  for (;;) {
    loopUWB();
    vTaskDelay(10);
  }
}

void task2code(void *parameter) {
  Serial.println("task2 ing");
  for (;;) {
    Serial.println(xPortGetCoreID());
    Serial.println("task2 3s delay");
    digitalWrite(led,1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(led,0);
    Serial.println("task2 4s delay");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setupUWB() {
  Serial.println(F("### DW1000Ng-arduino-ranging-anchor ###"));
  DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
  Serial.println(F("DW1000Ng initialized ..."));
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
  DW1000Ng::setDeviceAddress(1);
  DW1000Ng::setAntennaDelay(16436);
  Serial.println(F("Committed configuration ..."));

  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg); Serial.print("Device ID: "); Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg); Serial.print("Unique ID: "); Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg); Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg); Serial.print("Device mode: "); Serial.println(msg);

  DW1000Ng::attachSentHandler(handleSent);
  DW1000Ng::attachReceivedHandler(handleReceived);
  receiver();
  noteActivity();
  rangingCountPeriod = millis();
}

void loopUWB() {
  //Serial.printf("receivedAck: %d, sentAck: %d, protocolFailed: %d\n", receivedAck, sentAck, protocolFailed);
  //Serial.print(", expectedMsgId: "); Serial.println(expectedMsgId);
  int32_t curMillis = millis();
  if (!sentAck && !receivedAck) {
    if (curMillis - lastActivity > resetPeriod) {
      resetInactive();
    }
    return;
  }
  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];
    if (msgId == POLL_ACK) {
      timePollAckSent = DW1000Ng::getTransmitTimestamp();
      noteActivity();
    }
    DW1000Ng::startReceive();
  }
  if (receivedAck) {
    receivedAck = false;
    DW1000Ng::getReceivedData(data, LEN_DATA);
    byte msgId = data[0];
    if (msgId != expectedMsgId) {
      Serial.printf("Wrong msgId: %d (expected: %d)\n", msgId, expectedMsgId); // 추가 
      protocolFailed = true;
    }
    if (msgId == POLL) {
      protocolFailed = false;
      timePollReceived = DW1000Ng::getReceiveTimestamp();
      expectedMsgId = RANGE;
      transmitPollAck();
      noteActivity();
    }
    else if (msgId == RANGE) {
      timeRangeReceived = DW1000Ng::getReceiveTimestamp();
      expectedMsgId = POLL;
      static int failCount = 0;
      if (!protocolFailed) {
        
        timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
        timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
        timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
        double distance = DW1000NgRanging::computeRangeAsymmetric(
            timePollSent, timePollReceived,
            timePollAckSent, timePollAckReceived,
            timeRangeSent, timeRangeReceived
        );
        distance = DW1000NgRanging::correctRange(distance);
        Serial.printf("Range: %.2f m\t RX power: %.2f dBm\t Sampling: %.2f Hz\n", 
                      distance, DW1000Ng::getReceivePower(), samplingRate);
        transmitRangeReport(distance * DISTANCE_OF_RADIO_INV);
        successRangingCount++;
        if (curMillis - rangingCountPeriod > 1000) {
          samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
          rangingCountPeriod = curMillis;
          successRangingCount = 0;
        }
        failCount = 0;  
      }
      else {
        transmitRangeFailed();
         failCount++;
         if (failCount > 5) {
           Serial.println("Too many failures, resetting DW1000...");
           setupUWB();
           failCount = 0;
           return;  
         }
      }
      noteActivity();
    }
  }
}


void noteActivity() { lastActivity = millis(); }
void resetInactive() { expectedMsgId = POLL; receiver(); noteActivity(); }
void handleSent() { sentAck = true; }
void handleReceived() { receivedAck = true; }
void transmitPollAck() {
  data[0] = POLL_ACK;
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}
void transmitRangeReport(float curRange) {
  data[0] = RANGE_REPORT;
  memcpy(data + 1, &curRange, 4);
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}
void transmitRangeFailed() {
  data[0] = RANGE_FAILED;
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}
void receiver() {
  DW1000Ng::forceTRxOff();
  DW1000Ng::startReceive();   
}
void loop() {
  
}
