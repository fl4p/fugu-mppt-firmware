
String
  firmwareInfo = "V1.11   ",
  firmwareDate = "21/03/23",
  firmwareContactR1 = "www.youtube.com/",
  firmwareContactR2 = "TechBuilder     ";



//====================================== USER PARAMETERS ==========================================//
// The parameters below are the default parameters used when the MPPT charger settings have not    //
// been set or saved through the LCD menu interface or mobile phone WiFi app. Some parameters here //
// would allow you to override or unlock features for advanced users (settings not on the LCD menu)//
//=================================================================================================//
bool
  MPPT_Mode = 1,             //   USER PARAMETER - Enable MPPT algorithm, when disabled charger uses CC-CV algorithm
  output_Mode = 1,           //   USER PARAMETER - 0 = PSU MODE, 1 = Charger Mode
  disableFlashAutoLoad = 0,  //   USER PARAMETER - Forces the MPPT to not use flash saved settings, enabling this "1" defaults to programmed firmware settings.
  enablePPWM = 1,            //   USER PARAMETER - Enables Predictive PWM, this accelerates regulation speed (only applicable for battery charging application)
  enableWiFi = 1,            //   USER PARAMETER - Enable WiFi Connection
  enableFan = 1,             //   USER PARAMETER - Enable Cooling Fan
  enableBluetooth = 1,       //   USER PARAMETER - Enable Bluetooth Connection
  enableLCD = 1,             //   USER PARAMETER - Enable LCD display
  enableLCDBacklight = 1,    //   USER PARAMETER - Enable LCD display's backlight
  overrideFan = 0,           //   USER PARAMETER - Fan always on
  enableDynamicCooling = 0;  //   USER PARAMETER - Enable for PWM cooling control
int
  serialTelemMode = 1,            //  USER PARAMETER - Selects serial telemetry data feed (0 - Disable Serial, 1 - Display All Data, 2 - Display Essential, 3 - Number only)

  temperatureFan = 60,            //  USER PARAMETER - Temperature threshold for fan to turn on
  temperatureMax = 90,            //  USER PARAMETER - Overtemperature, System Shudown When Exceeded (deg C)
  telemCounterReset = 0,          //  USER PARAMETER - Reset Telem Data Every (0 = Never, 1 = Day, 2 = Week, 3 = Month, 4 = Year)
  errorTimeLimit = 1000,          //  USER PARAMETER - Time interval for reseting error counter (milliseconds)
  errorCountLimit = 5,            //  USER PARAMETER - Maximum number of errors
  millisRoutineInterval = 250,    //  USER PARAMETER - Time Interval Refresh Rate For Routine Functions (ms)
  millisSerialInterval = 500,     //  USER PARAMETER - Time Interval Refresh Rate For USB Serial Datafeed (ms)
  millisLCDInterval = 500,        //  USER PARAMETER - Time Interval Refresh Rate For LCD Display (ms)
  millisWiFiInterval = 2000,      //  USER PARAMETER - Time Interval Refresh Rate For WiFi Telemetry (ms)
  millisLCDBackLInterval = 2000,  //  USER PARAMETER - Time Interval Refresh Rate For WiFi Telemetry (ms)
  backlightSleepMode = 0,         //  USER PARAMETER - 0 = Never, 1 = 10secs, 2 = 5mins, 3 = 1hr, 4 = 6 hrs, 5 = 12hrs, 6 = 1 day, 7 = 3 days, 8 = 1wk, 9 = 1month
  baudRate = 115200;              //  USER PARAMETER - USB Serial Baud Rate (bps)

float
  voltageBatteryMax = 27.3000,  //   USER PARAMETER - Maximum Battery Charging Voltage (Output V)
  voltageBatteryMin = 22.4000,  //   USER PARAMETER - Minimum Battery Charging Voltage (Output V)
  currentCharging = 30.0000,    //   USER PARAMETER - Maximum Charging Current (A - Output)
  electricalPrice = 9.5000;     //   USER PARAMETER - Input electrical price per kWh (Dollar/kWh,Euro/kWh,Peso/kWh)


//================================== CALIBRATION PARAMETERS =======================================//
// The parameters below can be tweaked for designing your own MPPT charge controllers. Only modify //
// the values below if you know what you are doing. The values below have been pre-calibrated for  //
// MPPT charge controllers designed by TechBuilder (Angelo S. Casimiro)                            //
//=================================================================================================//
constexpr bool ADS1015_Mode = std::is_same<Adafruit_ADS1015, decltype(ads)>::value; 

int
  ADC_GainSelect = 2,  //  CALIB PARAMETER - ADC Gain Selection (0→±6.144V 3mV/bit, 1→±4.096V 2mV/bit, 2→±2.048V 1mV/bit)
  avgCountVS = 3,      //  CALIB PARAMETER - Voltage Sensor Average Sampling Count (Recommended: 3)
  avgCountCS = 4,      //  CALIB PARAMETER - Current Sensor Average Sampling Count (Recommended: 4)
  avgCountTS = 500;    //  CALIB PARAMETER - Temperature Sensor Average Sampling Count
float
  inVoltageDivRatio = 43.5532,    //  CALIB PARAMETER - Input voltage divider sensor ratio (change this value to calibrate voltage sensor)
  outVoltageDivRatio = 24.5000,   //  CALIB PARAMETER - Output voltage divider sensor ratio (change this value to calibrate voltage sensor)
  vOutSystemMax = 50.0000,        //  CALIB PARAMETER -
  cOutSystemMax = 50.0000,        //  CALIB PARAMETER -
  ntcResistance = 10000.00,       //  CALIB PARAMETER - NTC temp sensor's resistance. Change to 10000.00 if you are using a 10k NTC
  voltageDropout = 1.0000,        //  CALIB PARAMETER - Buck regulator's dropout voltage (DOV is present due to Max Duty Cycle Limit)
  voltageBatteryThresh = 0.0000,  //  CALIB PARAMETER - Power cuts-off when this voltage is reached (Output V)
  currentInAbsolute = 31.0000,    //  CALIB PARAMETER - Maximum Input Current The System Can Handle (A - Input)
  currentOutAbsolute = 50.0000,   //  CALIB PARAMETER - Maximum Output Current The System Can Handle (A - Input)
  efficiencyRate = 1.0000,        //  CALIB PARAMETER - Theroretical Buck Efficiency (% decimal)
  currentMidPoint = 2.5250,       //  CALIB PARAMETER - Current Sensor Midpoint (V)
  currentSens = 0.0000,           //  CALIB PARAMETER - Current Sensor Sensitivity (V/A)
  currentSensV = 0.0660,          //  CALIB PARAMETER - Current Sensor Sensitivity (mV/A)
  vInSystemMin = 10.000;          //  CALIB PARAMETER -

//===================================== SYSTEM PARAMETERS =========================================//
// Do not change parameter values in this section. The values below are variables used by system   //
// processes. Changing the values can damage the MPPT hardware. Kindly leave it as is! However,    //
// you can access these variables to acquire data needed for your mods.                            //
//=================================================================================================//
bool
  useInternalADC = 0,
  buckEnable = 0,           // SYSTEM PARAMETER - Buck Enable Status
  fanStatus = 0,            // SYSTEM PARAMETER - Fan activity status (1 = On, 0 = Off)
  bypassEnable = 0,         // SYSTEM PARAMETER -
  chargingPause = 0,        // SYSTEM PARAMETER -
  lowPowerMode = 0,         // SYSTEM PARAMETER -
  buttonRightStatus = 0,    // SYSTEM PARAMETER -
  buttonLeftStatus = 0,     // SYSTEM PARAMETER -
  buttonBackStatus = 0,     // SYSTEM PARAMETER -
  buttonSelectStatus = 0,   // SYSTEM PARAMETER -
  buttonRightCommand = 0,   // SYSTEM PARAMETER -
  buttonLeftCommand = 0,    // SYSTEM PARAMETER -
  buttonBackCommand = 0,    // SYSTEM PARAMETER -
  buttonSelectCommand = 0,  // SYSTEM PARAMETER -
  settingMode = 0,          // SYSTEM PARAMETER -
  setMenuPage = 0,          // SYSTEM PARAMETER -
  boolTemp = 0,             // SYSTEM PARAMETER -
  flashMemLoad = 0,         // SYSTEM PARAMETER -
  confirmationMenu = 0,     // SYSTEM PARAMETER -
  WIFI = 0,                 // SYSTEM PARAMETER -
  BNC = 0,                  // SYSTEM PARAMETER -
  REC = 0,                  // SYSTEM PARAMETER -
  FLV = 0,                  // SYSTEM PARAMETER -
  IUV = 0,                  // SYSTEM PARAMETER -
  IOV = 0,                  // SYSTEM PARAMETER -
  IOC = 0,                  // SYSTEM PARAMETER -
  OUV = 0,                  // SYSTEM PARAMETER -
  OOV = 0,                  // SYSTEM PARAMETER -
  OOC = 0,                  // SYSTEM PARAMETER -
  OTE = 0;                  // SYSTEM PARAMETER -
int
inputSource           = 0,           // SYSTEM PARAMETER - 0 = MPPT has no power source, 1 = MPPT is using solar as source, 2 = MPPTis using battery as power source
avgStoreTS            = 0,           // SYSTEM PARAMETER - Temperature Sensor uses non invasive averaging, this is used an accumulator for mean averaging
temperature           = 0,           // SYSTEM PARAMETER -
sampleStoreTS         = 0,           // SYSTEM PARAMETER - TS AVG nth Sample
PPWM                  = 0,           // SYSTEM PARAMETER -

batteryPercent        = 0,           // SYSTEM PARAMETER -
errorCount            = 0,           // SYSTEM PARAMETER -
menuPage              = 0,           // SYSTEM PARAMETER -
subMenuPage           = 0,           // SYSTEM PARAMETER -
ERR                   = 0,           // SYSTEM PARAMETER - 
conv1                 = 0,           // SYSTEM PARAMETER -
conv2                 = 0,           // SYSTEM PARAMETER -
intTemp               = 0;           // SYSTEM PARAMETER -
float
  VSI = 0.0000,               // SYSTEM PARAMETER - Raw input voltage sensor ADC voltage
  VSO = 0.0000,               // SYSTEM PARAMETER - Raw output voltage sensor ADC voltage
  CSI = 0.0000,               // SYSTEM PARAMETER - Raw current sensor ADC voltage
  CSI_converted = 0.0000,     // SYSTEM PARAMETER - Actual current sensor ADC voltage
  TS = 0.0000,                // SYSTEM PARAMETER - Raw temperature sensor ADC value
  powerInput = 0.0000,        // SYSTEM PARAMETER - Input power (solar power) in Watts
  powerInputPrev = 0.0000,    // SYSTEM PARAMETER - Previously stored input power variable for MPPT algorithm (Watts)
  powerOutput = 0.0000,       // SYSTEM PARAMETER - Output power (battery or charing power in Watts)
  energySavings = 0.0000,     // SYSTEM PARAMETER - Energy savings in fiat currency (Peso, USD, Euros or etc...)
  voltageInput = 0.0000,      // SYSTEM PARAMETER - Input voltage (solar voltage)
  voltageInputPrev = 0.0000,  // SYSTEM PARAMETER - Previously stored input voltage variable for MPPT algorithm
  voltageOutput = 0.0000,     // SYSTEM PARAMETER - Input voltage (battery voltage)
  currentInput = 0.0000,      // SYSTEM PARAMETER - Output power (battery or charing voltage)
  currentOutput = 0.0000,     // SYSTEM PARAMETER - Output current (battery or charing current in Amperes)
  TSlog = 0.0000,             // SYSTEM PARAMETER - Part of NTC thermistor thermal sensing code
  ADC_BitReso = 0.0000,       // SYSTEM PARAMETER - System detects the approriate bit resolution factor for ADS1015/ADS1115 ADC
  daysRunning = 0.0000,       // SYSTEM PARAMETER - Stores the total number of days the MPPT device has been running since last powered
  Wh = 0.0000,                // SYSTEM PARAMETER - Stores the accumulated energy harvested (Watt-Hours)
  kWh = 0.0000,               // SYSTEM PARAMETER - Stores the accumulated energy harvested (Kiliowatt-Hours)
  MWh = 0.0000,               // SYSTEM PARAMETER - Stores the accumulated energy harvested (Megawatt-Hours)
  loopTime = 0.0000,          // SYSTEM PARAMETER -
  outputDeviation = 0.0000,   // SYSTEM PARAMETER - Output Voltage Deviation (%)
  buckEfficiency = 0.0000,    // SYSTEM PARAMETER - Measure buck converter power conversion efficiency (only applicable to my dual current sensor version)
  floatTemp = 0.0000,
  vOutSystemMin = 0.0000;  //  CALIB PARAMETER -
unsigned long
  currentErrorMillis = 0,     //SYSTEM PARAMETER -
  currentButtonMillis = 0,    //SYSTEM PARAMETER -
  currentSerialMillis = 0,    //SYSTEM PARAMETER -
  currentRoutineMillis = 0,   //SYSTEM PARAMETER -
  currentLCDMillis = 0,       //SYSTEM PARAMETER -
  currentLCDBackLMillis = 0,  //SYSTEM PARAMETER -
  currentWiFiMillis = 0,      //SYSTEM PARAMETER -
  currentMenuSetMillis = 0,   //SYSTEM PARAMETER -
  prevButtonMillis = 0,       //SYSTEM PARAMETER -
  prevSerialMillis = 0,       //SYSTEM PARAMETER -
  prevRoutineMillis = 0,      //SYSTEM PARAMETER -
  prevErrorMillis = 0,        //SYSTEM PARAMETER -
  prevWiFiMillis = 0,         //SYSTEM PARAMETER -
  prevLCDMillis = 0,          //SYSTEM PARAMETER -
  prevLCDBackLMillis = 0,     //SYSTEM PARAMETER -
  timeOn = 0,                 //SYSTEM PARAMETER -
  loopTimeStart = 0,          //SYSTEM PARAMETER - Used for the loop cycle stop watch, records the loop start time
  loopTimeEnd = 0,            //SYSTEM PARAMETER - Used for the loop cycle stop watch, records the loop end time
  secondsElapsed = 0;         //SYSTEM PARAMETER -

uint8_t ADC_internal_ch_map[4] = { 0, 39, 36, 34 };

//====================================== MAIN PROGRAM =============================================//
// The codes below contain all the system processes for the MPPT firmware. Most of them are called //
// from the 8 .ino tabs. The codes are too long, Arduino tabs helped me a lot in organizing them.  //
// The firmware runs on two cores of the Arduino ESP32 as seen on the two separate pairs of void   //
// setups and loops. The xTaskCreatePinnedToCore() freeRTOS function allows you to access the      //
// unused ESP32 core through Arduino. Yes it does multicore processes simultaneously!              //
//=================================================================================================//

//================= CORE0: SETUP (DUAL CORE MODE) =====================//
void coreTwo(void* pvParameters) {
  setupWiFi();  //TAB#7 - WiFi Initialization
                //================= CORE0: LOOP (DUAL CORE MODE) ======================//
  while (1) {
    Wireless_Telemetry();  //TAB#7 - Wireless telemetry (WiFi & Bluetooth)
  }
}
//================== CORE1: SETUP (DUAL CORE MODE) ====================//
void setup() {

  //SERIAL INITIALIZATION
  Serial.begin(baudRate);                  //Set serial baud rate
  Serial.println("> Serial Initialized");  //Startup message

  //GPIO PIN INITIALIZATION
  pinMode(backflow_MOSFET,OUTPUT);                          
  //pinMode(buck_EN,OUTPUT);
  pinMode(LED,OUTPUT); 
  pinMode(FAN,OUTPUT);
  pinMode(TempSensor,INPUT);

  pinMode(buttonLeft, INPUT_PULLDOWN);
  pinMode(buttonRight, INPUT_PULLDOWN);
  pinMode(buttonBack, INPUT_PULLDOWN);
  pinMode(buttonSelect, INPUT_PULLDOWN);



  // TODO p
  
  //Wire.setClock(400000UL);
  //ads.setDataRate(RATE_ADS1115_860SPS);

  //ADC INITIALIZATION
  ADC_SetGain();       //Sets ADC Gain & Range
  if (!ads.begin()) {  //Initialize ADC
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

  //GPIO INITIALIZATION
  buck_Disable();

  //ENABLE DUAL CORE MULTITASKING
  xTaskCreatePinnedToCore(coreTwo, "coreTwo", 10000, NULL, 0, &Core2, 0);

  //INITIALIZE AND LIOAD FLASH MEMORY DATA
  EEPROM.begin(512);
  Serial.println("> FLASH MEMORY: STORAGE INITIALIZED");  //Startup message
  initializeFlashAutoload();                              //Load stored settings from flash memory
  Serial.println("> FLASH MEMORY: SAVED DATA LOADED");    //Startup message

  //LCD INITIALIZATION
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

  //SETUP FINISHED
  Serial.println("> MPPT HAS INITIALIZED");  //Startup message
}
//================== CORE1: LOOP (DUAL CORE MODE) ======================//
void loop() {
  Read_Sensors();        //TAB#2 - Sensor data measurement and computation
  Device_Protection();   //TAB#3 - Fault detection algorithm
  System_Processes();    //TAB#4 - Routine system processes
  Charging_Algorithm();  //TAB#5 - Battery Charging Algorithm
  Onboard_Telemetry();   //TAB#6 - Onboard telemetry (USB & Serial Telemetry)
  LCD_Menu();            //TAB#8 - Low Power Algorithm
}
