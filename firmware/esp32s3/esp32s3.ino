//FaBo JetRacer_XIAO_ESP32S3 togikaidrive hack
//Board Rev3.0.4
//2024/06/19 -> RPM割り込みカウント版に改変
//             RPM計算を実経過時間ベースに修正（pulseInLongブロック対策）
//2026/03/01 -> pinMode(INPUT)追加（pulseInLongがピン設定なしでは動作しない問題の修正）
//ESP32S3
//ブラシレスモーターRPMセンサー（割り込みカウント方式）

//#define DEBUG
//#define DEBUG_RCV
//#define DEBUG_MILLIS

#include "Wire.h"
#include "SPI.h"

//ボード情報
#define FIRMWARE_NUMBER    8     //Firmware Version ID 8 (pinMode修正版)
#define BOARDMAJOR         3
#define BOARDMINOR         0
#define BOARDPATCH         4

//ピン設定
#define ST_SIGNAL_INPUT_PIN       D0  //受信機1ch
#define TH_SIGNAL_INPUT_PIN       D1  //受信機2ch
#define FSW_SIGNAL_INPUT_PIN      D2  //受信機3ch
#define SELECT_OUTPUT_PIN         D3  //マルチプレクサ信号切り替え信号入力ピン
#define BULSHLESSMOTOR_PIN1       D6  //ブラレスモーターセンサーピン１（未使用）
#define BULSHLESSMOTOR_PIN2       D7  //ブラレスモーターセンサーピン２（RPMカウント）

//I2Cスレーブデバイスアドレス設定
#define I2C_DEV_ADDR 0x08

// RPM計算パラメータ
#define RPM_SAMPLE_INTERVAL_MS  20   // RPM計算周期 (ms) = 50Hz更新
#define MOTOR_POLE_PAIRS         1   // モーターのポールペア数（2極=1）
#define RPM_DEBOUNCE_US        100   // デバウンス時間 (µs) ※6万RPM=1kHz → 周期1ms なので100µsで十分

//I2Cデバイスレジスタ
uint8_t registerIndex = 0x01;

//LED切り替えモード
int sw_led = 0;

//色の選択 BGR
int blue = 0;
int green = 0;
int red = 0;

//チャッタリング対策
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 800;
bool noSignalStatus = false;

//数値をバイト列への型
typedef union
{
    uint32_t    before;
    struct
    {
        uint8_t d;
        uint8_t c;
        uint8_t b;
        uint8_t a;
    };
} Transfer;

Transfer transfer1;  // ステアリング
Transfer transfer2;  // スロットル
Transfer transfer3;  // 切り替え信号
Transfer transfer4;  // バージョン情報
Transfer transfer5;  // RPMセンサー生データ（上位16bit=パルス数, 下位16bit=経過ms）

// ---- RPM計測用変数 ----
volatile uint32_t pulseCount = 0;         // 割り込みでインクリメント
volatile uint32_t lastPulseTime_us = 0;   // デバウンス用タイムスタンプ
uint16_t lastPulseSnapshot = 0;           // 前回送信したパルス数
uint16_t lastElapsedSnapshot = 0;         // 前回送信した経過時間
unsigned long lastRpmCalcTime = 0;        // 前回計測時刻

// portMUX: ESP32のクリティカルセクション用
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ---- RPM割り込みハンドラ ----
void IRAM_ATTR rpmPulseISR() {
  uint32_t now_us = (uint32_t)(esp_timer_get_time());  // µs単位タイムスタンプ

  // デバウンス: 前回パルスから RPM_DEBOUNCE_US µs 未満なら無視
  if ((now_us - lastPulseTime_us) < RPM_DEBOUNCE_US) {
    return;
  }
  lastPulseTime_us = now_us;

  portENTER_CRITICAL_ISR(&mux);
  pulseCount++;
  portEXIT_CRITICAL_ISR(&mux);
}

// ---- パルスカウント取得関数 ----
// パルス数を安全に取得してリセットし、経過時間とともに返す
// RPM計算はPython側（pwm_controller.py / rpm_sensor.py）で行う
void snapshotPulseCount(unsigned long elapsed_ms) {
  uint32_t count;

  portENTER_CRITICAL(&mux);
  count = pulseCount;
  pulseCount = 0;
  portEXIT_CRITICAL(&mux);

  // uint16_tに収まるようクリップ（最大65535）
  lastPulseSnapshot = (count > 65535) ? 65535 : (uint16_t)count;
  lastElapsedSnapshot = (elapsed_ms > 65535) ? 65535 : (uint16_t)elapsed_ms;

  // transfer5: 上位16bit=パルス数, 下位16bit=経過ms
  transfer5.before = ((uint32_t)lastPulseSnapshot << 16) | (uint32_t)lastElapsedSnapshot;
}

// ---- I2Cコールバック ----
void onRequest(){
  if(registerIndex == 0x01)
  {
    Wire.write(transfer1.a);
    Wire.write(transfer1.b);
    Wire.write(transfer1.c);
    Wire.write(transfer1.d);
    Wire.write(transfer2.a);
    Wire.write(transfer2.b);
    Wire.write(transfer2.c);
    Wire.write(transfer2.d);
    Wire.write(transfer3.a);
    Wire.write(transfer3.b);
    Wire.write(transfer3.c);
    Wire.write(transfer3.d);
    // RPM値（計算済み）を送信
    Wire.write(transfer5.a);
    Wire.write(transfer5.b);
    Wire.write(transfer5.c);
    Wire.write(transfer5.d);
  }
  else if(registerIndex == 0x00){
    Wire.write(transfer4.a);
    Wire.write(transfer4.b);
    Wire.write(transfer4.c);
    Wire.write(transfer4.d);
  }
}

void onReceive(int len){
  while(Wire.available()){
    registerIndex = Wire.read();
    switch(registerIndex) {
      case 0x10: sw_led = 0; break;
      case 0x1a: blue=0;   green=0;   red=255; sw_led=1; break;
      case 0x1b: blue=255; green=0;   red=0;   sw_led=1; break;
      case 0x1c: blue=0;   green=255; red=255; sw_led=1; break;
      case 0x1d: blue=0;   green=255; red=0;   sw_led=1; break;
      case 0x1e: blue=255; green=255; red=255; sw_led=1; break;
      case 0x1f: blue=0;   green=60;  red=228; sw_led=1; break;
      case 0x20: blue=90;  green=0;   red=64;  sw_led=1; break;
      case 0x21: blue=59;  green=204; red=170; sw_led=1; break;
      case 0x22: blue=159; green=110; red=235; sw_led=1; break;
      case 0x30: blue=0;   green=0;   red=0;   sw_led=1; break;
    }
  }
}

//LED-SPI信号関数
void startBit() {
  byte start = 0x00;
  SPI.transfer(start); SPI.transfer(start);
  SPI.transfer(start); SPI.transfer(start);
}

void endBit() {
  byte end = 0x00;
  SPI.transfer(end); SPI.transfer(end);
  SPI.transfer(end); SPI.transfer(end);
}

void setRGB(short r, short g, short b) {
  SPI.transfer(0xEF);
  SPI.transfer(r);
  SPI.transfer(g);
  SPI.transfer(b);
}

// ---- Setup ----
void setup() {
  #if defined(DEBUG) || defined(DEBUG_RCV) || defined(DEBUG_MILLIS)
    Serial.begin(115200);
    Serial.setDebugOutput(true);
  #endif
  delay(500);

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  SPI.begin();

  // RC受信機信号ピン設定（pulseInLongはpinMode設定が必須）
  pinMode(ST_SIGNAL_INPUT_PIN, INPUT);
  pinMode(TH_SIGNAL_INPUT_PIN, INPUT);
  pinMode(FSW_SIGNAL_INPUT_PIN, INPUT);

  pinMode(SELECT_OUTPUT_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // RPMセンサーピン設定 + 割り込み登録（立ち上がりエッジ検出）
  pinMode(BULSHLESSMOTOR_PIN2, INPUT);
  attachInterrupt(digitalPinToInterrupt(BULSHLESSMOTOR_PIN2), rpmPulseISR, RISING);

  // バージョン情報
  transfer4.a = BOARDMAJOR;
  transfer4.b = BOARDMINOR;
  transfer4.c = BOARDPATCH;
  transfer4.d = FIRMWARE_NUMBER;

  lastRpmCalcTime = millis();

  #ifdef DEBUG
    Serial.println("RPM Sensor initialized (Interrupt mode)");
    Serial.print("Sample interval: "); Serial.print(RPM_SAMPLE_INTERVAL_MS); Serial.println(" ms");
    Serial.print("Max measurable RPM: "); Serial.println(60000UL / RPM_SAMPLE_INTERVAL_MS * 255);
  #endif
}

// ---- Loop ----
void loop() {
  static uint8_t counta;
  static uint16_t countled;

  // RC受信機を信号計測（pulseInLong はRPMセンサー以外のみ）
  uint32_t duration = pulseInLong(FSW_SIGNAL_INPUT_PIN, HIGH, 25000);
  uint32_t pwm0     = pulseInLong(ST_SIGNAL_INPUT_PIN,  HIGH, 25000);
  uint32_t pwm1     = pulseInLong(TH_SIGNAL_INPUT_PIN,  HIGH, 25000);

  // パルスカウント取得（一定周期ごと）
  unsigned long now = millis();
  unsigned long elapsed = now - lastRpmCalcTime;

  if (elapsed >= RPM_SAMPLE_INTERVAL_MS) {
    lastRpmCalcTime = now;
    snapshotPulseCount(elapsed);

    #ifdef DEBUG_RCV
      char buf[64];
      sprintf(buf, "Pulse: %u (elapsed: %ums)", lastPulseSnapshot, lastElapsedSnapshot);
      Serial.println(buf);
    #endif
  }

  // 整数からバイト列へ変換
  transfer1.before = pwm0;
  transfer2.before = pwm1;
  transfer3.before = duration;
  // transfer5 は上記RPM計算ブロックで更新済み

  #ifdef DEBUG_RCV
    char buf[32];
    sprintf(buf, "Steering %d", pwm0);     Serial.println(buf);
    sprintf(buf, "Throttle %d", pwm1);     Serial.println(buf);
    sprintf(buf, "Duration %d", duration); Serial.println(buf);
    sprintf(buf, "Pulse    %u (%ums)", lastPulseSnapshot, lastElapsedSnapshot); Serial.println(buf);
  #endif

  // 信号切り替え + LED制御（元のロジックをそのまま維持）
  if (duration > 1500){
    digitalWrite(SELECT_OUTPUT_PIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    startBit();
    setRGB(80, 0, 45);
    if (sw_led == 0){
      for(int i=0; i<15; i++) setRGB(80, 0, 255);
    } else {
      for(int i=0; i<15; i++) setRGB(blue, green, red);
    }
    endBit();
    noSignalStatus = false;

  } else if ((duration <= 1500) && (duration >= 100)){
    digitalWrite(SELECT_OUTPUT_PIN, LOW);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    startBit();
    if (sw_led == 0){
      for(int i=0; i<16; i++) setRGB(0, 255, 0);
    } else {
      for(int i=0; i<16; i++) setRGB(blue, green, red);
    }
    endBit();
    noSignalStatus = false;

  } else {
    if (noSignalStatus == false){
      lastDebounceTime = millis();
      noSignalStatus = true;
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
      digitalWrite(SELECT_OUTPUT_PIN, LOW);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      startBit();
      if (sw_led == 0){
        for(int i=0; i<16; i++) setRGB(255, 0, 0);
      } else {
        for(int i=0; i<16; i++) setRGB(blue, green, red);
      }
      endBit();
      noSignalStatus = false;
      delay(1000);
    }
  }
}