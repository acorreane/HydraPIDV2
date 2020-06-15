#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <PID_v1.h>
#include <Modbus.h>
#include <ModbusSerial.h>
#include <Thread.h>
#include <ThreadController.h>
#include <TimerOne.h>

ModbusSerial mb;
Thread sampler;
Thread calc;
Thread regs;

const uint8_t pin_triac = 13;
const uint8_t pin_lcdbl =  3;

const uint8_t raw_reads_1 = 10;  // sampling reads

uint8_t  raw_index_1;                 // the raw_index_1 of the current reading
uint16_t raw_sample_1[raw_reads_1];   // the raw_sample_1 from the analog input
uint16_t raw_average_1;               // the raw_average_1
uint32_t raw_total_1;                 // the running raw_total_1

double prev_wtemp, wtemp, set_wtemp, pwm_wtemp;
double wKp=0.0, wKi=0.0, wKd=0.0;

const float T1=273;      // [K]        Temperatura de calibração 1
const float T2=373;      // [K]        Temperatura de calibração 2
const float RT1=30000;   // [ohms]     Resistência medida na temperatura de calibração 1
const float RT2=650;     // [ohms]     Resistência medida na temperatura de calibração 2
const float Raux=11200;  // [ohm]      Resistor auxiliar
const float R0=10000;    // [ohm]      valor nominal do NTC a 25ºC
const float T0=298.15;   // [K] (25ºC)
const float Vin=5.0;     // [V]     

float Vout_1=0.0;    // [V]        Tensão de saída do divisor
float Rout_1=0.0;    // [ohm]      Valor de resistência do NTC
float beta_1=0.0;    // [K]        parâmetro beta_1
float Rinf_1=0.0;    // [ohm]      parâmetro Rinf_1
float TempK_1=0.0;   // [K]        Temperatura de saída em Kelvin
float TempC_1=0.0;   // [ºC]       Temperatura de saída em °C   

const uint16_t MB_REG_DO0 = 0; //power sw
const uint16_t MB_REG_AI0 = 0; //water temp
const uint16_t MB_REG_AI1 = 1; //power out
const uint16_t MB_REG_AO0 = 0; //setpoint
const uint16_t MB_REG_AO1 = 1; //wKp
const uint16_t MB_REG_AO2 = 2; //wKi
const uint16_t MB_REG_AO3 = 3; //wKd

uint8_t prev_pwm_power, pwm_power, pwr_toggle, rem_power;
uint8_t pot_value, pot_last, pot_up, pot_down;

Adafruit_PCD8544 display = Adafruit_PCD8544(8, 7, 6, 5, 4);
PID waterPID(&wtemp, &pwm_wtemp, &set_wtemp, wKp, wKi, wKd, P_ON_E, DIRECT);

void timerIsr() {
  softPWM();
}

void softPWM() {
 digitalWrite(pin_triac, (millis()%100 < pwm_power) ? (HIGH) : (LOW)  )  ; // 10 Hz
}

void setup() 
{
 pinMode(pin_triac, OUTPUT); // heater 1 pwm
 pinMode(pin_lcdbl, OUTPUT); analogWrite(pin_lcdbl, 127); // lcd backlight pwm
 
 display.begin(); 
 display.clearDisplay();
 display.display();
 
 if (EEPROM.read(200) != 123)
 {
  EEPinfo('r');
  EEPinfo('w');
  EEPROM.write(200, 123);
  EEPROM.write(0, 13); //wp 
  EEPROM.write(1, 12); //wi
  EEPROM.write(2, 11); //wd
 }
 
  EEPinfo('r');
  wKp=EEPROM.read(0)/10, wKi=EEPROM.read(1)/10, wKd=EEPROM.read(2)/10; // fixed to floating point (*10 = 1.0) default values
  
  for (int i = 0; i < raw_reads_1; i++) raw_sample_1[i] = 0;
  
  waterPID.SetMode(AUTOMATIC);
  waterPID.SetOutputLimits(0, 100);
  waterPID.SetSampleTime(500);

  waterPID.SetTunings(wKp, wKi, wKd, P_ON_M);
  
  mb.config(&Serial, 19200, SERIAL_8N2);
  mb.setSlaveId(1);  

  mb.addCoil(MB_REG_DO0); // power switch
  mb.addIreg(MB_REG_AI0); // water temp
  mb.addIreg(MB_REG_AI1); // power out
  mb.addHreg(MB_REG_AO0); // set water temp
  mb.addHreg(MB_REG_AO1, wKp*10);
  mb.addHreg(MB_REG_AO2, wKi*10); 
  mb.addHreg(MB_REG_AO3, wKd*10);

  beta_1=(log(RT1/RT2))/((1/T1)-(1/T2));
  Rinf_1=R0*exp(-beta_1/T0);

  sampler.setInterval(50);
  sampler.onRun(smoothRaw);
  calc.setInterval(100);
  calc.onRun(deltaC);
  regs.setInterval(200);
  regs.onRun(regsUpdate);
  
  Timer1.initialize(2500);
  Timer1.attachInterrupt(timerIsr); 
}

void loop()
{
 if (sampler.shouldRun())
 {
  sampler.run();
  mb.task();
 }
 if (calc.shouldRun()){ calc.run(); }

 if (regs.shouldRun()){ regs.run(); }
  
 if (pwr_toggle == true) { pidCalc(); }
}

void smoothRaw()
{
 readPot();
 
 raw_total_1 = raw_total_1 - raw_sample_1[raw_index_1];         
 raw_sample_1[raw_index_1] = analogRead(7); delay(1);
 raw_total_1 = raw_total_1 + raw_sample_1[raw_index_1];       
 raw_index_1 = raw_index_1 + 1;
 if (raw_index_1 >= raw_reads_1){ raw_index_1 = 0; }
 raw_average_1 = raw_total_1 / raw_reads_1;
}
  
void deltaC()
{
 Vout_1 = Vin*((float)(raw_average_1)/1024.0);
 Rout_1 = (Raux*Vout_1/(Vin-Vout_1));
 TempK_1 = (beta_1/log(Rout_1/Rinf_1));
 TempC_1 = TempK_1-273.15;
 TempC_1 = constrain(TempC_1, 0, 99.0);
 mb.Ireg(MB_REG_AI0, TempC_1*10);
 wtemp = TempC_1;
 pscan();
 ioscan();
}
  

void pscan()
{
 if (prev_wtemp != wtemp) { prev_wtemp = wtemp; UpdateTFT(); }
 if (prev_pwm_power != pwm_power) { prev_pwm_power = pwm_power; UpdateTFT(); }
}

 void ioscan()
{
 if (!rem_power) {
  if (pot_value <1 && pwr_toggle == true) 
  {
   pwm_power = 0;
   pwr_toggle = false; 
   mb.Coil(MB_REG_DO0, pwr_toggle);
  }
 } else if(pot_value >1) { rem_power = false; }
 if (pot_value >1 && pwr_toggle == false) 
 { 
  pwr_toggle = true;
  mb.Coil(MB_REG_DO0, pwr_toggle); 
 }
}

void regsUpdate()
{
 if (mb.Hreg(MB_REG_AO1) != wKp*10)
 {
  wKp = mb.Hreg(MB_REG_AO1)/10;
  waterPID.SetTunings(wKp, wKi, wKd, P_ON_M);
  EEPROM.write(0, wKp*10); 
  EEPinfo('w');
 }
  
 if (mb.Hreg(MB_REG_AO2) != wKi*10)
 {
  wKi = mb.Hreg(MB_REG_AO2)/10;
  waterPID.SetTunings(wKp, wKi, wKd, P_ON_M);
  EEPROM.write(1, wKi*10);
  EEPinfo('w');
 }
  
 if (mb.Hreg(MB_REG_AO3) != wKd*10)
 {
  wKd = mb.Hreg(MB_REG_AO3)/10;
  waterPID.SetTunings(wKp, wKi, wKd, P_ON_M);
  EEPROM.write(2, wKd*10);
  EEPinfo('w');
 }

 if (mb.Hreg(MB_REG_AO0) != set_wtemp) { set_wtemp = mb.Hreg(MB_REG_AO0); UpdateTFT(); } //hreg
 
 if (mb.Ireg(MB_REG_AI1) != pwm_power) { mb.Ireg(MB_REG_AI1, pwm_power); UpdateTFT(); } //ireg

 if ((mb.Coil(MB_REG_DO0) != pwr_toggle) && !rem_power)
 {
  pwr_toggle = mb.Coil(MB_REG_DO0);
  if (pwr_toggle == false)
  {
   pwm_power = 0;
  }
  UpdateTFT();
  rem_power = true;
 }
}

void pidCalc()
{
  waterPID.Compute();
  pwm_power = pwm_wtemp;
  //pwm_power = 50; //debug trick
}

void readPot()
{
  MAIN:
  pot_value = analogRead(0)/20;

  if (pot_value/2 > pot_last) {
    pot_last = pot_value/2;
    pot_down = true;
  }else   if (pot_value/2 < pot_last) {
    pot_last = pot_value/2;
    pot_up = true;
  }

  if (pot_down || pot_up)
 {
  set_wtemp = map(pot_value, 0, 50, 20, 40);
  mb.Hreg(MB_REG_AO0, set_wtemp);
  UpdateTFT(); delay(100);
  pot_down = false; pot_up = false;
  goto MAIN;
 }
}


void UpdateTFT()
{
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(BLACK, WHITE);
 display.setCursor(2, 0);
 
 if (pwr_toggle == 0) { display.print("   STANDBY   "); }
 if (pwr_toggle == 1) { display.print("TEMP: "); display.print(wtemp, 1); display.print("'C"); }
 
 display.drawFastHLine(0,10,83,BLACK);
 display.setCursor(5, 15);
 display.print("Set       Pwr");
 display.setTextSize(2);
 display.setCursor(5, 25);

 if (pwr_toggle == 0) { display.print("OFF"); }
 if (pwr_toggle == 1) { display.print(set_wtemp, 0); display.print("'C"); }
 
 display.setCursor(65, 25);
 display.setTextSize(1);
 display.print((float)map(pwm_power, 0, 100, 0, 68)/10, 1);
 display.setCursor(65, 35);
 display.print("kW");
 display.display(); 
}

void EEPinfo(char c)
{
 display.setTextSize(1);
 display.setTextColor(BLACK, WHITE);
 display.setCursor(2, 0);
 if (c=='w') { display.print("writting  eep"); }
 if (c=='r') { display.print("reading  eep "); }
 display.display(); 
 delay(250);
 UpdateTFT();
}

