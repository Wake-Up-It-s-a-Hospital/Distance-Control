#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <SimpleKalmanFilter.h>
// connection pins
const uint8_t PIN_RST = 15; // reset pin
const uint8_t PIN_IRQ = 13; // irq pin
const uint8_t PIN_SS = 14; // spi select pin

#include "esp_system.h"  
// 리셋 추가 코드

SimpleKalmanFilter kalmanFilter(1.5, 2, 0.2);

#define PIN_MOSI  23
#define PIN_MISO  19
#define PIN_SCK   12  // 원래 18번  

HardwareSerial hwSer(2); // UART2 인스턴스 선언

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

double currentFiltered = -1; // 칼만 필터 거친 거리값

// ===== 추가된 코드 시작 =====
double previousDistance = -1;           // 이전 distance 값 저장
uint32_t lastDistanceChangeTime = 0;    // 마지막으로 distance가 변경된 시간
const uint32_t DISTANCE_TIMEOUT = 1000; // 1초 (1000ms)
// ===== 추가된 코드 끝 =====

// message flow state
volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;
// timestamps to remember
uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;

uint64_t timeComputedRange;
// last computed range/time
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
    true,
    true,
    true,
    false,
    true
};

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    delay(1000);
    hwSer.begin(9600, SERIAL_8N1, 21, 17); // RX=21, TX=18
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS); // spi 바꿔보기 
   
    delay(1000);
    Serial.println(F("### DW1000Ng-arduino-ranging-anchor ###"));
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
	DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

    DW1000Ng::setDeviceAddress(1);
	
    DW1000Ng::setAntennaDelay(16436);
    
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    // attach callback for (successfully) sent and received messages
    DW1000Ng::attachSentHandler(handleSent);
    DW1000Ng::attachReceivedHandler(handleReceived);
    // anchor starts in receiving mode, awaiting a ranging poll message
   
    receiver();
    noteActivity();
    // for first time ranging frequency computation
    rangingCountPeriod = millis();
    
    // ===== 추가된 코드 시작 =====
    lastDistanceChangeTime = millis(); // 초기화
    // ===== 추가된 코드 끝 =====
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // anchor listens for POLL
    expectedMsgId = POLL;
    receiver();
    noteActivity();
}

// ===== 추가된 함수 시작 =====
void checkDistanceTimeout() {
    uint32_t currentTime = millis();
    
    // 1초 동안 distance가 변경되지 않았으면 리셋
    if (currentTime - lastDistanceChangeTime > DISTANCE_TIMEOUT) {
        Serial.println("Distance timeout detected! Resetting system...");
        
        // 방법 1: 소프트웨어 리셋
        //resetInactive();
        
        // 방법 2: 하드웨어 리셋 (필요시 주석 해제)
         esp_restart();
        
        // 변수 초기화
        previousDistance = -1;
        currentFiltered = -1;
        lastDistanceChangeTime = currentTime;
    }
}
// ===== 추가된 함수 끝 =====

void handleSent() {
    // status change on sent success
    sentAck = true;
    //Serial.println("s");
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
    //Serial.println("r");
}

void transmitPollAck() {
    data[0] = POLL_ACK;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitRangeReport(float curRange) {
    data[0] = RANGE_REPORT;
    // write final ranging result
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
    // so we don't need to restart the receiver manually
    DW1000Ng::startReceive();
}

void loop() {
    int32_t curMillis = millis();
    
    // ===== 추가된 코드 시작 =====
    // distance 타임아웃 체크
    checkDistanceTimeout();
    // ===== 추가된 코드 끝 =====
    
    if (!sentAck && !receivedAck) {
        // check if inactive
        if (curMillis - lastActivity > resetPeriod) {
            resetInactive();
        }
        return;
    }
    // continue on any success confirmation
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
        // get message and parse
        DW1000Ng::getReceivedData(data, LEN_DATA);
        byte msgId = data[0];
        if (msgId != expectedMsgId) {
            // unexpected message, start over again (except if already POLL)
            protocolFailed = true;
        }
        if (msgId == POLL) {
            // on POLL we (re-)start, so no protocol failure
            protocolFailed = false;
            timePollReceived = DW1000Ng::getReceiveTimestamp();
            expectedMsgId = RANGE;
            transmitPollAck();
            noteActivity();
        }
        else if (msgId == RANGE) {
            timeRangeReceived = DW1000Ng::getReceiveTimestamp();
            expectedMsgId = POLL;
            if (!protocolFailed) {
                timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
                timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
                timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
                // (re-)compute range as two-way ranging is done
                double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                            timePollReceived, 
                                                            timePollAckSent, 
                                                            timePollAckReceived, 
                                                            timeRangeSent, 
                                                            timeRangeReceived);
                /* Apply simple bias correction */
                distance = DW1000NgRanging::correctRange(distance);
                
                // ===== 추가된 코드 시작 =====
                // distance 값 변경 체크
                if (abs(distance - previousDistance) > 0.01) { // 1cm 이상 변경시
                    previousDistance = distance;
                    lastDistanceChangeTime = curMillis;
                }
                // ===== 추가된 코드 끝 =====
                
                double filtered = kalmanFilter.updateEstimate(distance);  // 칼만 필터 적용
                currentFiltered = filtered;  // 전역 변수에 저장! // 추가 

                String uartMessage = String(filtered, 2); // 소수점 둘째 자리까지 문자열 변환

                Serial.println(uartMessage); // 추가 
                hwSer.print(uartMessage);          // STM32로 전송 (\n 포함)              // UART로 전송*/

                
                String rangeString = "Range: "; rangeString += distance; rangeString += " m";
                rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
                rangeString += "\t Sampling: "; rangeString += samplingRate; rangeString += " Hz";
                Serial.println(rangeString);
                //Serial.print("FP power is [dBm]: "); Serial.print(DW1000Ng::getFirstPathPower());
                //Serial.print("RX power is [dBm]: "); Serial.println(DW1000Ng::getReceivePower());
                //Serial.print("Receive quality: "); Serial.println(DW1000Ng::getReceiveQuality());
                // update sampling rate (each second)
                transmitRangeReport(distance * DISTANCE_OF_RADIO_INV);
                successRangingCount++;
                if (curMillis - rangingCountPeriod > 1000) {
                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                    rangingCountPeriod = curMillis;
                    successRangingCount = 0;
                }
            }
            else {
                transmitRangeFailed();
            }

            noteActivity();
        }
    }
}