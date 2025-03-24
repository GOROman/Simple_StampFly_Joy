/*
* MIT License
* 
* Copyright (c) 2024 Kouhei Ito
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/


//Controller for M5Fly
//#define DEBUG

#include <Arduino.h>
#include <M5AtomS3.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MPU6886.h>
#include <MadgwickAHRS.h>
#include <atoms3joy.h>
#include <FS.h>
#include <SPIFFS.h>
#include "buzzer.h"

#define ANGLECONTROL 0
#define RATECONTROL 1
#define ANGLECONTROL_W_LOG 2
#define RATECONTROL_W_LOG 3
#define ALT_CONTROL_MODE 4
#define NOT_ALT_CONTROL_MODE 5
#define RESO10BIT (4096)

// BLE Service and Characteristic UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CONTROL_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TELEMETRY_CHAR_UUID "8b7c9c6a-c2dc-41e9-a087-7f4c2f9a75d0"

// コントロール値
float Throttle;
float Phi, Theta, Psi;
uint16_t Phi_bias = 2048;
uint16_t Theta_bias = 2048;
uint16_t Psi_bias = 2048;
uint16_t Throttle_bias = 2048;

// モード設定
uint8_t Mode = ANGLECONTROL;
uint8_t AltMode = NOT_ALT_CONTROL_MODE;
uint8_t StickMode = 2;

// タイマー関連
volatile uint8_t Loop_flag = 0;
float Timer = 0.0;
float dTime = 0.01;
uint8_t Timer_state = 0;
unsigned long stime, etime, dtime;

// BLE関連
BLEServer* pServer = nullptr;
BLECharacteristic* pControlCharacteristic = nullptr;
BLECharacteristic* pTelemetryCharacteristic = nullptr;
volatile bool BleConnected = false;
static uint32_t connectionCounter = 0;

void rc_init(void);
void data_send(void);
void show_battery_info();
void voltage_print(void);



#define BUF_SIZE 128
// EEPROMにデータを保存する


void rc_init(void)
{
    // BLEデバイスの初期化
    BLEDevice::init("StampFly");

    // BLEサーバーの作成
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // BLEサービスの作成
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // コントロールキャラクタリスティックの作成
    pControlCharacteristic = pService->createCharacteristic(
        CONTROL_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pControlCharacteristic->setCallbacks(new ControlCallbacks());

    // テレメトリーキャラクタリスティックの作成
    pTelemetryCharacteristic = pService->createCharacteristic(
        TELEMETRY_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTelemetryCharacteristic->addDescriptor(new BLE2902());

    // サービスの開始
    pService->start();

    // アドバタイジングの開始
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // iPhoneの接続問題を解決するための設定
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    USBSerial.println("BLE Server Ready");
}

//周期カウンタ割り込み関数
hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer() 
{
  Loop_flag = 1;
}

void setup() {
  M5.begin();
  Wire1.begin(38, 39, 400*1000);
  M5.update();
  setup_pwm_buzzer();
  M5.Lcd.setRotation( 2 );
  M5.Lcd.setTextFont(2);
  M5.Lcd.setCursor(4, 2);
  
  // BLEの初期化
  rc_init();
  M5.Lcd.println("BLE Ready!");
  M5.Lcd.fillScreen(BLACK);
  joy_update();

  StickMode = 2;
  if(getOptionButton())
  {
    StickMode = 3;
    M5.Lcd.println("Please release button.");
    while(getOptionButton())joy_update();
  }
  AltMode =NOT_ALT_CONTROL_MODE;
  delay(500);

  if (StickMode == 3)
  {
    THROTTLE = RIGHTY;
    AILERON = LEFTX;
    ELEVATOR = LEFTY;
    RUDDER = RIGHTX;
    ARM_BUTTON = RIGHT_STICK_BUTTON;
    FLIP_BUTTON = LEFT_STICK_BUTTON;
    MODE_BUTTON = RIGHT_BUTTON;
    OPTION_BUTTON = LEFT_BUTTON;
  }
  else
  {
    THROTTLE = LEFTY;
    AILERON = RIGHTX;
    ELEVATOR = RIGHTY;
    RUDDER = LEFTX;
    ARM_BUTTON = LEFT_STICK_BUTTON;
    FLIP_BUTTON = RIGHT_STICK_BUTTON;
    MODE_BUTTON = RIGHT_BUTTON;
    OPTION_BUTTON = LEFT_BUTTON;
  }

  byte error, address;
  int nDevices;

////////////////////////////////////////////////////////
  USBSerial.println("Scanning... Wire1");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      USBSerial.print("I2C device found at address 0x");
      if (address < 16)
        USBSerial.print("0");
      USBSerial.print(address, HEX);
      USBSerial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      USBSerial.print("Unknown error at address 0x");
      if (address < 16)
        USBSerial.print("0");
      USBSerial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    USBSerial.println("No I2C devices found\n");
  else
    USBSerial.println("done\n");

  esp_now_get_version(&espnow_version);
  USBSerial.printf("ESP-NOW Version %d\n", espnow_version);


  //割り込み設定
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);
  delay(100);
}

uint8_t check_control_mode_change(void)
{
  uint8_t state;
  static uint8_t flag =0;
  state = 0;
  if (flag==0)
  {
    if (getModeButton() == 1)
    {
      flag = 1;
    }
  }
  else
  {
    if (getModeButton() == 0)
    {
      flag = 0;
      state = 1;
    }
  }
  //USBSerial.printf("%d %d\n\r", state, flag);
  return state;
}

uint8_t check_alt_mode_change(void)
{
  uint8_t state;
  static uint8_t flag =0;
  state = 0;
  if (flag==0)
  {
    if (getOptionButton() == 1)
    {
      flag = 1;
    }
  }
  else
  {
    if (getOptionButton() == 0)
    {
      flag = 0;
      state = 1;
    }
  }
  //USBSerial.printf("%d %d\n\r", state, flag);
  return state;
}



void loop() {
  uint16_t _throttle;
  uint16_t _phi;
  uint16_t _theta;
  uint16_t _psi;

  while(Loop_flag==0);
  Loop_flag = 0;
  etime = stime;
  stime = micros();
  dtime = stime - etime;  
  M5.update();
  joy_update();

  //Stop Watch Start&Stop&Reset  
  if(M5.Btn.wasPressed()==true)
  {
    if (Timer_state == 0)Timer_state = 1;
    else if (Timer_state == 1)Timer_state = 0;
  }

  if(M5.Btn.pressedFor(400)==true)
  {
    Timer_state = 2;
  }

  if (Timer_state == 1)
  {
    //カウントアップ
    Timer = Timer + dTime;
  }
  else if (Timer_state == 2)
  {
    //タイマリセット
    Timer = 0.0;
    Timer_state = 0;
  }

  if (check_control_mode_change() == 1)
  {
    if (Mode==ANGLECONTROL)Mode=RATECONTROL;
    else Mode = ANGLECONTROL;
  }

  if (check_alt_mode_change() == 1)
  {
    if (AltMode==ALT_CONTROL_MODE)AltMode=NOT_ALT_CONTROL_MODE;
    else AltMode = ALT_CONTROL_MODE;
  }

  _throttle = getThrottle();
  _phi = getAileron();
  _theta = getElevator();
  _psi = getRudder();

  if(getArmButton()==1)
  {
    //Throttle_bias = _throttle;
    Phi_bias = _phi;
    Theta_bias = _theta;
    Psi_bias = _psi;
  }

  //量産版
  Throttle = -(float)(_throttle - Throttle_bias)/(float)(RESO10BIT*0.5);
  Phi =       (float)(_phi - Phi_bias)/(float)(RESO10BIT*0.5); 
  Theta =     (float)(_theta - Theta_bias)/(float)(RESO10BIT*0.5);
  Psi =       (float)(_psi - Psi_bias)/(float)(RESO10BIT*0.5);

  // テレメトリーデータの送信
  if (BleConnected && pTelemetryCharacteristic != nullptr) {
    float telemetryData[16] = {0};
    telemetryData[0] = Psi;
    telemetryData[1] = Throttle;
    telemetryData[2] = Phi;
    telemetryData[3] = Theta;
    telemetryData[4] = (float)getArmButton();
    telemetryData[5] = (float)getFlipButton();
    telemetryData[6] = (float)Mode;
    telemetryData[7] = (float)AltMode;
    telemetryData[8] = (float)proactive_flag;

    pTelemetryCharacteristic->setValue((uint8_t*)telemetryData, sizeof(float) * 16);
    pTelemetryCharacteristic->notify();
  }

  // 接続タイムアウトの検出
  connectionCounter++;
  if (connectionCounter > 1000) { // 約10秒のタイムアウト
    BleConnected = false;
    connectionCounter = 0;
  }
  #ifdef DEBUG
  USBSerial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
    peerInfo.peer_addr[0],
    peerInfo.peer_addr[1],
    peerInfo.peer_addr[2],
    peerInfo.peer_addr[3],
    peerInfo.peer_addr[4],
    peerInfo.peer_addr[5]);
  #endif
  //Display information
  //float vbat =0.0;// M5.Axp.GetBatVoltage();
  //int8_t bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  
  M5.Lcd.setCursor(4, 2+disp_counter*17);
  switch (disp_counter)
  {
    case 0:
      M5.Lcd.printf("MAC ADR %02X:%02X    ", peerInfo.peer_addr[4],peerInfo.peer_addr[5]);
      break;
    case 1:
      M5.Lcd.printf("BAT 1:%4.1f 2:%4.1f", Battery_voltage[0],Battery_voltage[1]);
      //M5.Lcd.printf("X:%4d",xstick);
      break;
    case 2:
      #ifdef NEW_ATOM_JOY
      M5.Lcd.printf("MODE: %d", StickMode);
      //M5.Lcd.printf("X:%4d",xstick);
      #endif
      break;
    case 3:
      M5.Lcd.printf("CHL: %02d",peerInfo.channel);
      break;
    case 4:
      if( AltMode == ALT_CONTROL_MODE ) M5.Lcd.printf("-Auto ALT-  ");
      else if ( AltMode == NOT_ALT_CONTROL_MODE )   M5.Lcd.printf("-Mnual ALT- ");
      break;
    case 5:
      if( Mode == ANGLECONTROL )      M5.Lcd.printf("-STABILIZE-");
      else if ( Mode == RATECONTROL ) M5.Lcd.printf("-ACRO-     ");
      break;
    case 6:
      M5.Lcd.printf("Time:%7.2f",Timer);
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
  }
  disp_counter++;
  if(disp_counter==11)disp_counter=0;

  //Reset
  if( /*M5.Axp.GetBtnPress() == 2*/ 0 ){
    // 電源ボタンクリック
    //M5.Lcd.println("AtomFly2.0"); 
    esp_restart();
  } 

}

void show_battery_info(){
  #if 0
  // バッテリー電圧表示
  double vbat = 0.0;
  int8_t bat_charge_p = 0;

  vbat = M5.Axp.GetBatVoltage();
  M5.Lcd.setCursor(5, 100);
  //M5.Lcd.setTextSize(1);
  M5.Lcd.printf("Volt:\n %8.2fV", vbat);

  // バッテリー残量表示
  bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  M5.Lcd.setCursor(5, 140);
  M5.Lcd.printf("Charge:\n %8d%%", bat_charge_p);
#endif
}

void voltage_print(void)
{

  M5.Lcd.setCursor(0, 17, 2);
  M5.Lcd.printf("%3.1fV", Battery_voltage);
}
