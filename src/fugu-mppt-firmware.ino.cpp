# 1 "/var/folders/nl/jtl1vjrd30b9y_r9gbv3sjsc0000gn/T/tmpb3wv88ez"
#include <Arduino.h>
# 1 "/Users/fabianschlieper/dev/pv/fugu-mppt-fw/src/fugu-mppt-firmware.ino"

#if 0
String
  firmwareInfo = "V1.11   ",
  firmwareDate = "21/03/23",
  firmwareContactR1 = "www.youtube.com/",
  firmwareContactR2 = "TechBuilder     ";
# 16 "/Users/fabianschlieper/dev/pv/fugu-mppt-fw/src/fugu-mppt-firmware.ino"
bool
  MPPT_Mode = 1,
  output_Mode = 1,
  disableFlashAutoLoad = 0,
  enablePPWM = 1,
  enableWiFi = 1,
  enableFan = 1,
  enableBluetooth = 1,
  enableLCD = 1,
  enableLCDBacklight = 1,
  overrideFan = 0,
  enableDynamicCooling = 0;
int
  serialTelemMode = 1,

  temperatureFan = 60,
  temperatureMax = 90,
  telemCounterReset = 0,
  errorTimeLimit = 1000,
  errorCountLimit = 5,
  millisRoutineInterval = 250,
  millisSerialInterval = 500,
  millisLCDInterval = 500,
  millisWiFiInterval = 2000,
  millisLCDBackLInterval = 2000,
  backlightSleepMode = 0,
  baudRate = 115200;

float
  voltageBatteryMax = 27.3000,
  voltageBatteryMin = 22.4000,
  currentCharging = 30.0000,
  electricalPrice = 9.5000;







constexpr bool ADS1015_Mode = std::is_same<Adafruit_ADS1015, decltype(ads)>::value;

int
  ADC_GainSelect = 2,
  avgCountVS = 3,
  avgCountCS = 4,
  avgCountTS = 500;
float
  inVoltageDivRatio = 43.5532,
  outVoltageDivRatio = 24.5000,
  vOutSystemMax = 50.0000,
  cOutSystemMax = 50.0000,
  ntcResistance = 10000.00,
  voltageDropout = 1.0000,
  voltageBatteryThresh = 0.0000,
  currentInAbsolute = 31.0000,
  currentOutAbsolute = 50.0000,
  efficiencyRate = 1.0000,
  currentMidPoint = 2.5250,
  currentSens = 0.0000,
  currentSensV = 0.0660,
  vInSystemMin = 10.000;






bool
  useInternalADC = 0,
  buckEnable = 0,
  fanStatus = 0,
  bypassEnable = 0,
  chargingPause = 0,
  lowPowerMode = 0,
  buttonRightStatus = 0,
  buttonLeftStatus = 0,
  buttonBackStatus = 0,
  buttonSelectStatus = 0,
  buttonRightCommand = 0,
  buttonLeftCommand = 0,
  buttonBackCommand = 0,
  buttonSelectCommand = 0,
  settingMode = 0,
  setMenuPage = 0,
  boolTemp = 0,
  flashMemLoad = 0,
  confirmationMenu = 0,
  WIFI = 0,
  BNC = 0,
  REC = 0,
  FLV = 0,
  IUV = 0,
  IOV = 0,
  IOC = 0,
  OUV = 0,
  OOV = 0,
  OOC = 0,
  OTE = 0;
int
inputSource = 0,
avgStoreTS = 0,
temperature = 0,
sampleStoreTS = 0,
PPWM = 0,

batteryPercent = 0,
errorCount = 0,
menuPage = 0,
subMenuPage = 0,
ERR = 0,
conv1 = 0,
conv2 = 0,
intTemp = 0;
float
  VSI = 0.0000,
  VSO = 0.0000,
  CSI = 0.0000,
  CSI_converted = 0.0000,
  TS = 0.0000,
  powerInput = 0.0000,
  powerInputPrev = 0.0000,
  powerOutput = 0.0000,
  energySavings = 0.0000,
  voltageInput = 0.0000,
  voltageInputPrev = 0.0000,
  voltageOutput = 0.0000,
  currentInput = 0.0000,
  currentOutput = 0.0000,
  TSlog = 0.0000,
  ADC_BitReso = 0.0000,
  daysRunning = 0.0000,
  Wh = 0.0000,
  kWh = 0.0000,
  MWh = 0.0000,
  loopTime = 0.0000,
  outputDeviation = 0.0000,
  buckEfficiency = 0.0000,
  floatTemp = 0.0000,
  vOutSystemMin = 0.0000;
unsigned long
  currentErrorMillis = 0,
  currentButtonMillis = 0,
  currentSerialMillis = 0,
  currentRoutineMillis = 0,
  currentLCDMillis = 0,
  currentLCDBackLMillis = 0,
  currentWiFiMillis = 0,
  currentMenuSetMillis = 0,
  prevButtonMillis = 0,
  prevSerialMillis = 0,
  prevRoutineMillis = 0,
  prevErrorMillis = 0,
  prevWiFiMillis = 0,
  prevLCDMillis = 0,
  prevLCDBackLMillis = 0,
  timeOn = 0,
  loopTimeStart = 0,
  loopTimeEnd = 0,
  secondsElapsed = 0;

uint8_t ADC_internal_ch_map[4] = { 0, 39, 36, 34 };
# 188 "/Users/fabianschlieper/dev/pv/fugu-mppt-fw/src/fugu-mppt-firmware.ino"
void coreTwo(void* pvParameters);
void loop();
#line 188 "/Users/fabianschlieper/dev/pv/fugu-mppt-fw/src/fugu-mppt-firmware.ino"
void coreTwo(void* pvParameters) {
  setupWiFi();

  while (1) {
    Wireless_Telemetry();
  }
}

[[noreturn]] void setup() {


  Serial.begin(baudRate);
  Serial.println("> Serial Initialized");


  pinMode(backflow_MOSFET,OUTPUT);

  pinMode(LED,OUTPUT);
  pinMode(FAN,OUTPUT);
  pinMode(TempSensor,INPUT);
# 219 "/Users/fabianschlieper/dev/pv/fugu-mppt-fw/src/fugu-mppt-firmware.ino"
  ADC_SetGain();
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS. Using internal ADC");
    useInternalADC = true;
    for (auto pin : ADC_internal_ch_map)
      if (pin != 0) {
        if (!adcAttachPin(pin)) Serial.println("Failed to attach pin");
        analogSetPinAttenuation(pin, ADC_6db);
      }
  } else {
    Serial.println("ADC initialized");
      pinMode(ADC_ALERT, INPUT_PULLDOWN);
  }


  buck_Disable();


  xTaskCreatePinnedToCore(coreTwo, "coreTwo", 10000, NULL, 0, &Core2, 0);


  EEPROM.begin(512);
  Serial.println("> FLASH MEMORY: STORAGE INITIALIZED");
  initializeFlashAutoload();
  Serial.println("> FLASH MEMORY: SAVED DATA LOADED");


  if (enableLCD == 1) {
    lcd.begin();
    lcd.setBacklight(HIGH);
    lcd.setCursor(0, 0);
    lcd.print("MPPT INITIALIZED");
    lcd.setCursor(0, 1);
    lcd.print("FIRMWARE ");
    lcd.print(firmwareInfo);
    delay(1500);
    lcd.clear();
  }


  Serial.println("> MPPT HAS INITIALIZED");
}

void loop() {
  Read_Sensors();
  Device_Protection();
  System_Processes();
  Charging_Algorithm();
  Onboard_Telemetry();
  LCD_Menu();
}

#endif