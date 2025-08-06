#include <Arduino.h>
#include <IO7F32.h>

String user_html = "";
char* ssid_pfix = (char*)"Watch_tag";
unsigned long lastPublishMillis = 0;

#define BUTTON_PIN 19
#define LED_PIN 21
#define VIBRATION_PIN 32
#define BUZZER_PIN 23

bool isAutoMode = false;
String currentModeText = "manual";

unsigned long lastLostAlertTime = 0;
unsigned long lastBatteryAlertTime = 0;
const unsigned long ALERT_COOLDOWN = 5000;

int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

volatile bool modeChanged = false;

void IRAM_ATTR buttonInterrupt() {
  modeChanged = true;
}

void handleUserCommand(char* topic, JsonDocument* root) {
    JsonObject d = (*root)["d"];
    Serial.println("\n=== 명령 수신 ===");
    Serial.println("토픽: " + String(topic));
    String msgStr;
    serializeJson(*root, msgStr);
    Serial.println("수신 메시지: " + msgStr);

    if (d.containsKey("mode_sync") && d.containsKey("device_type")) {
        String deviceType = d["device_type"];
        if (deviceType == "husky_controller") {
            String newMode = d["mode_sync"];
            String sourceDevice = d["source_device"];
            Serial.println("허스키부 컨트롤러에서 모드 동기화 명령 수신");
            Serial.println("새로운 모드: " + newMode);
            String oldMode = currentModeText;
            currentModeText = newMode;
            isAutoMode = (newMode == "auto");
            digitalWrite(LED_PIN, isAutoMode ? HIGH : LOW);
            Serial.println("모드 동기화 완료: " + currentModeText);
        }
    }
    else if (d.containsKey("alert_type")) {
        String alertType = d["alert_type"];
        unsigned long currentTime = millis();
        if (alertType == "lost") {
          if (currentTime - lastLostAlertTime >= ALERT_COOLDOWN) {
              Serial.println("LOST 알림 - 부저 1초 작동");
              digitalWrite(BUZZER_PIN, HIGH);
              delay(1000);
              digitalWrite(BUZZER_PIN, LOW);
              lastLostAlertTime = currentTime;
          }
        }
        else if (alertType == "battery") { 
          if (currentTime - lastBatteryAlertTime >= ALERT_COOLDOWN) {
              Serial.println("BATTERY 알림 - 진동모터 1초 작동");
              digitalWrite(VIBRATION_PIN, HIGH);
              delay(1000);
              digitalWrite(VIBRATION_PIN, LOW);
              lastBatteryAlertTime = currentTime;
          }
        }
    }
}

void publishMode() {
  StaticJsonDocument<256> doc;
  JsonObject data = doc.createNestedObject("d");

  data["mode"] = currentModeText;
  data["device_id"] = WiFi.macAddress();
  data["device_type"] = "uwb_tag";

  serializeJson(doc, msgBuffer);
  
  String modeTopicStr = "iot3/watch1/evt/mode/fmt/json";
  strcpy(evtTopic, modeTopicStr.c_str());

  if (client.publish(evtTopic, msgBuffer)) {
    Serial.println("Mode published to AWS IoT:");
    Serial.println("토픽: " + String(evtTopic));
    Serial.println("메시지: " + String(msgBuffer));
  } else {
    Serial.println("Failed to publish mode");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, FALLING);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(VIBRATION_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("시계부 ESP32 시작");

  initDevice();
  strcpy(evtTopic, "iot3/watch1/evt/mode/fmt/json");

  JsonObject meta = cfg["meta"];
  pubInterval = meta.containsKey("pubInterval") ? meta["pubInterval"] : 0;
  lastPublishMillis = -pubInterval;

  WiFi.mode(WIFI_STA);
  WiFi.begin((const char*)cfg["ssid"], (const char*)cfg["w_pw"]);
  
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }

  Serial.println("\nConnected to WiFi");
  Serial.println("IP: " + WiFi.localIP().toString());
  Serial.println("MAC: " + WiFi.macAddress());

  userCommand = handleUserCommand;
  
  set_iot_server();
  iot_connect();

  Serial.println("Ready. Default mode: manual");
  Serial.println("허스키부와 양방향 동기화 준비 완료");
  Serial.println("진동모터(32번): Lost 알림");
  Serial.println("부저(23번): Battery 알림");
  Serial.println("=======================================");
}

void loop() {
  if (!client.connected()) {
    Serial.println("Reconnecting to AWS IoT...");
    iot_connect();
  }
  client.loop();

  int currentState = digitalRead(BUTTON_PIN);
  if (currentState == LOW && lastButtonState == HIGH) {
    unsigned long now = millis();
    if (now - lastDebounceTime > debounceDelay) {
      isAutoMode = !isAutoMode;
      currentModeText = isAutoMode ? "auto" : "manual";
      modeChanged = true;
      lastDebounceTime = now;
    }
  }
  lastButtonState = currentState;

  if (modeChanged) {
    Serial.println("Mode toggled → " + currentModeText);
    publishMode();
    modeChanged = false;
    digitalWrite(LED_PIN, isAutoMode ? HIGH : LOW);
  }

  delay(20);
}
