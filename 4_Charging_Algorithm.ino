
void buck_Disable() {  //Disable MPPT Buck Converter
  buckEnable = 0;
  //digitalWrite(buck_EN,LOW);
  ledcWrite(pwmCh_EN, 0);
  digitalWrite(LED, LOW);
  PWM = 0;
}
void predictivePWM() {                  //PREDICTIVE PWM ALGORITHM
  if (voltageInput <= 0) { PPWM = 0; }  //Prevents Indefinite Answer when voltageInput is zero
  else {
    PPWM = (PPWM_margin * pwmMax * min(voltageOutput, voltageBatteryMin)) / (100.00 * voltageInput);
  }  //Compute for predictive PWM Floor and store in variable

  PPWM = constrain(PPWM, 0, pwmMaxLimited); 
}

void PWM_Modulation() {
  if (output_Mode == 0) { PWM = constrain(PWM, 0, pwmMaxLimited); }  //PSU MODE PWM = PWM OVERFLOW PROTECTION (limit floor to 0% and ceiling to maximim allowable duty cycle)
  else {
    predictivePWM();                            //Runs and computes for predictive pwm floor
    PWM = constrain(PWM, PPWM, pwmMaxLimited);  //CHARGER MODE PWM - limit floor to PPWM and ceiling to maximim allowable duty cycle)
    // setting PWM >= PPWM might violate other contraints (OOV, OOI)
    //PWM = constrain(PWM,100,pwmMaxLimited);
  }
  ledcWrite(pwmCh_IN, PWM);  //Set PWM duty cycle and write to GPIO when buck is enabled

  ledcWrite(pwmCh_EN, constrain(PWM + (2e-6 * pwmMax * pwmFrequency), 0, pwmMax)); // EN 2us longer
  buckEnable = 1;

  digitalWrite(LED, HIGH);  //Turn on MPPT buck (IR2104)
}

void Charging_Algorithm() {
  if (ERR > 0 || chargingPause == 1) { buck_Disable(); }  //ERROR PRESENT  - Turn off MPPT buck when error is present or when chargingPause is used for a pause override
  else {
    if (REC == 1) {    //IUV RECOVERY - (Only active for charging mode)
      REC = 0;         //Reset IUV recovery boolean identifier
      buck_Disable();  //Disable buck before PPWM initialization
      lcd.setCursor(0, 0);
      lcd.print("POWER SOURCE    ");  //Display LCD message
      lcd.setCursor(0, 1);
      lcd.print("DETECTED        ");                    //Display LCD message
      Serial.println("> Solar Panel Detected");         //Display serial message
      Serial.print("> Computing For Predictive PWM ");  //Display serial message
      for (int i = 0; i < 40; i++) {
        Serial.print(".");
        delay(30);
      }                    //For loop "loading... effect
      Serial.println("");  //Display a line break on serial for next lines
      Read_Sensors();
      predictivePWM();
      PWM = PPWM;
      lcd.clear();
    } else {

      if (currentOutput > currentCharging) {
        if (currentOutput / currentCharging > 1.2f) {
          PWM /= 2;
          Serial.println("Current is 20% above limit, halving PWM");
        }  // current is 20% too high
        else if (currentOutput / currentCharging > 1.05f)
          PWM -= 2;      // current is 5% too high
        else { PWM--; }  //Current Is Above → Decrease Duty Cycle
      }
      //NO ERROR PRESENT  - Continue power conversion
      /////////////////////// CC-CV BUCK PSU ALGORITHM //////////////////////////////
      else if (MPPT_Mode == 0) {                           //CC-CV PSU MODE
        if (voltageOutput > voltageBatteryMax) { PWM--; }  //Voltage Is Above → Decrease Duty Cycle
        else if (voltageOutput < voltageBatteryMax) {
          PWM++;
        }        //Increase duty cycle when output is below charging voltage (for CC-CV only mode)
        else {}  //Do nothing when set output voltage is reached
                 //Set PWM signal to Buck PWM GPIO
      }
      /////////////////////// MPPT & CC-CV CHARGING ALGORITHM ///////////////////////
      else {

        if (voltageOutput > voltageBatteryMax) { PWM-=2; }                                 //Voltage Is Above → Decrease Duty Cycle
        else {                                                                            //MPPT ALGORITHM
          if (powerInput > powerInputPrev && voltageInput > voltageInputPrev) { PWM--; }  //  ↑P ↑V ; →MPP  //D--
          else if (powerInput > powerInputPrev && voltageInput < voltageInputPrev) {
            PWM++;
          }                                                                                    //  ↑P ↓V ; MPP←  //D++
          else if (powerInput < powerInputPrev && voltageInput > voltageInputPrev) { PWM++; }  //  ↓P ↑V ; MPP→  //D++
          else if (powerInput < powerInputPrev && voltageInput < voltageInputPrev) {
            PWM--;
          }                                                       //  ↓P ↓V ; ←MPP  //D--
          else if (voltageOutput < voltageBatteryMax) { PWM++; }  //  MP MV ; MPP Reached -
          powerInputPrev = powerInput;                            //Store Previous Recorded Power
          voltageInputPrev = voltageInput;                        //Store Previous Recorded Voltage
        }                                                         //Set PWM signal to Buck PWM GPIO
      }
      PWM_Modulation();
    }
  }
}
