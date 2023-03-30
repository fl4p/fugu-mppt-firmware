
/*
float readADCVoltage(int ch) {
  if(useInternalADC) {
    auto raw = analogRead(ADC_internal_ch_map[ch]);
    constexpr float Vmax = 1.750f; // 6db

    if (raw>4094) return Vmax;
    // poly curve fit from https://github.com/espressif/esp-idf/issues/164#issuecomment-318861287
    else return(-0.000000000023926 * pow(raw,3) + 0.000000094746 * pow(raw,2) + 0.00074539 * raw + 0.14925) * ((1/3.275f) * Vmax * intADC_gainCorr[ch]);
    //return raw * Vmax / 4095.0f * intADC_gainCorr[num];
  } else {
      return  ads.computeVolts(ads.readADC_SingleEnded(ch));    
  }
}


void Read_Sensors(){

  /////////// TEMPERATURE SENSOR /////////////
  if(sampleStoreTS<=avgCountTS){                               //TEMPERATURE SENSOR - Lite Averaging
    TS = TS + analogRead(TempSensor); // ~150us
    sampleStoreTS++;   
  }
  else{
    TS = TS/sampleStoreTS;
    TSlog = log(ntcResistance*(4095.00/TS-1.00));
    temperature = (1.0/(1.009249522e-03+2.378405444e-04*TSlog+2.019202697e-07*TSlog*TSlog*TSlog))-273.15;
    sampleStoreTS = 0;
    TS = 0;
  }
  /////////// VOLTAGE & CURRENT SENSORS /////////////
  VSI = 0.0000;      //Clear Previous Input Voltage 
  VSO = 0.0000;      //Clear Previous Output Voltage  
  CSI = 0.0000;      //Clear Previous Current  

  //VOLTAGE SENSOR - Instantenous Averaging   
  for(int i = 0; i<avgCountVS; i++){
    VSI = VSI + readADCVoltage(3);
    VSO = VSO + readADCVoltage(1);
  }
  voltageInput  = (VSI/avgCountVS)*inVoltageDivRatio; 
  voltageOutput = (VSO/avgCountVS)*outVoltageDivRatio; 

  
  //CURRENT SENSOR - Instantenous Averaging   
  for(int i = 0; i<avgCountCS; i++){
    CSI = CSI + readADCVoltage(2);
  }
  CSI_converted = (CSI/avgCountCS)*1.3300;
  currentInput  = ((CSI_converted-currentMidPoint)*-1)/currentSensV;  
  if(currentInput<0){currentInput=0.0000;} // todo fix
  if(voltageOutput<=0){currentOutput = 0.0000;}
  else{currentOutput = (voltageInput*currentInput)/voltageOutput;}

  //POWER SOURCE DETECTION
  if(voltageInput<=3 && voltageOutput<=3){inputSource=0;}  //System is only powered by USB port
  else if(voltageInput>voltageOutput)    {inputSource=1;}  //System is running on solar as power source
  else if(voltageInput<voltageOutput)    {inputSource=2;}  //System is running on batteries as power source
  
  //////// AUTOMATIC CURRENT SENSOR CALIBRATION ////////
  if(buckEnable==0 && FLV==0 && OOV == 0){                
    currentMidPoint = ((CSI/avgCountCS)*1.3300)-0.003;
  }
  
  //POWER COMPUTATION - Through computation
  powerInput      = voltageInput*currentInput;
  powerOutput     = voltageInput*currentInput*efficiencyRate;
  outputDeviation = (voltageOutput/voltageBatteryMax)*100.000;

  //STATE OF CHARGE - Battery Percentage
  batteryPercent  = ((voltageOutput-voltageBatteryMin)/(voltageBatteryMax-voltageBatteryMin))*101;
  batteryPercent  = constrain(batteryPercent,0,100);

  //TIME DEPENDENT SENSOR DATA COMPUTATION
  currentRoutineMillis = millis();
  if(currentRoutineMillis-prevRoutineMillis>=millisRoutineInterval){   //Run routine every millisRoutineInterval (ms)
    prevRoutineMillis = currentRoutineMillis;                          //Store previous time
    Wh = Wh+(powerInput/(3600.000*(1000.000/millisRoutineInterval)));  //Accumulate and compute energy harvested (3600s*(1000/interval))
    kWh = Wh/1000.000;
    MWh = Wh/1000000.000;
    daysRunning = timeOn/(86400.000*(1000.000/millisRoutineInterval)); //Compute for days running (86400s*(1000/interval))
    timeOn++;                                                          //Increment time counter
  } 

  //OTHER DATA
  secondsElapsed = millis()/1000;                                      //Gets the time in seconds since the was turned on  and active
  energySavings  = electricalPrice*(Wh/1000.0000);                     //Computes the solar energy saving in terms of money (electricity flag rate)   
}
*/