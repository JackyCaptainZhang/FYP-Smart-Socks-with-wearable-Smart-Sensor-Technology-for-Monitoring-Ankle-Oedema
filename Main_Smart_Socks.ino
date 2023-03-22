#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdlib.h>
#include <math.h>
#include <AltSoftSerial.h>
AltSoftSerial BTserial; // Bluetooth objects

// pin defination
const int flexPin = A3; 
#define RSTPIN 7
bool connected = false;

//GERO System varables
Adafruit_BNO055 bno = Adafruit_BNO055(55);
double cal_value_Acc;  //save the calibration value for GERO
double cal_value_Ang;
double cal_value_Gforce;
double ACCERLATION;
double ANGLE;
double thres_Acc = 0;
double user_Acc = 0;
double current_Acc = 0;
double previous_Acc = 0;
int num_Moving = 0;
bool moving = false;
unsigned long current_Time_Gero;  //GERO timer
unsigned long previous_Time_Gero;

//Flex System Variables
double value1; //save analog value from flex1
double cal_value1;  //save the calibration value for flex1
bool swelling = false;
double ini_Flex1;  // initial reading
bool firstRun = true;
int swelling_Times = 0;
unsigned long current_Time_Flex; //Flex Timers 
unsigned long previous_Time_Flex;

// Bluetooth System Variables
char datarcv;
String content = "";
int mode;

//*****************************   Manual Set Variables  ********************************************************************************************

//Bluetooth Manual Set Variables
int CMD_Num = 8;  // Number of CMD system supported
String sys_CMD[] = {"Gero","Flex", "Full", "Shutdown", "Restart", "FlexValue", "GeroValue", "Start"};  // CMD List
bool sys_Mode_Flags[] = {false, false, false, false, false, false, false, false};

//Flex Manual Set Variables
double sampleNum_Flex = 30.0;  //the sample rate for avarage filtering
double damping = 0.015;
double thres_Swelling_Detected = 0.08;
int thres_Swelling_Times = 6;  // Must <= 8
int time_interval_Flex = 5000;  // ms

//GERO Manual Set Variables
int time_interval_GERO = 5000;  // ms
double thres_Moving_Detected = 1.0;
int thres_Moving_Times = 8;
double sampleNum_GERO = 10.0;  //the sample rate for avarage filtering

//***************************************************************************************************************************************************

struct filterResult {
    double filter_Acc;
    double filter_Ang;
    double filter_Gforce;
};


double flex_AvarageFiltering(int pinNum, double sampleRate){ //Flex avarage filter
    int i;
  double filter_sum = 0;
  for(i = 0; i < sampleRate; i++) {
      filter_sum += analogRead(pinNum);
      delay(10);
    }
  return filter_sum;
}


filterResult GERO_Filter(){ //Gero avarage filter
  double dataSum_Acc = 0;
  double dataSum_Ang = 0;
  double dataSum_Gforce = 0;
  double filter_Acc = 0;
  double filter_Ang = 0;
  double filter_Gforce = 0;
  imu::Vector<3> ACCELEROMETER = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  for(int j = 0; j < sampleNum_GERO; j++){
    sensors_event_t event; 
    bno.getEvent(&event);
    imu::Vector<3> ACCELEROMETER = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    if(ACCELEROMETER.z() != 0.00){
      dataSum_Acc += ACCELEROMETER.z();
      dataSum_Ang += event.orientation.y;
      dataSum_Gforce += ACCELEROMETER.x();
      delay(10);
    } else {
      j -= 1;
    }
  }
  filter_Acc = dataSum_Acc / sampleNum_GERO;
  filter_Ang = dataSum_Ang / sampleNum_GERO;
  filter_Gforce = dataSum_Gforce /sampleNum_GERO;
return {filter_Acc, filter_Ang, filter_Gforce};
}

void flex_Calibration(){
  Serial.println("Start flex_Calibration.....");
  BTserial.println("Start flex_Calibration.....");  // Return message
  double flex_Calibration_sum = 0;
  for(int j = 0; j < sampleNum_Flex; j++){
    double filter_Result = flex_AvarageFiltering(flexPin, sampleNum_Flex);
    value1 = (((filter_Result / sampleNum_Flex) / 1024.0) * 5.0);
    flex_Calibration_sum += value1;
  }
  cal_value1 = flex_Calibration_sum / sampleNum_Flex;
  Serial.print("Calibration Flex: ");  Serial.println(cal_value1);
  BTserial.println("Flex Calibration successfully");  // Return message
}


void GERO_Calibration(){
  Serial.println("Start GERO Calibrating.....");
  BTserial.println("Start GERO Calibrating.....");  // Return message
  cal_value_Acc = 0;
  cal_value_Ang = 0;
  cal_value_Gforce = 0;
  for(int j = 0; j < sampleNum_GERO; j++){
    
    cal_value_Acc += GERO_Filter().filter_Acc;
    cal_value_Ang += GERO_Filter().filter_Ang;
    cal_value_Gforce += GERO_Filter().filter_Gforce;
  }
  cal_value_Acc = cal_value_Acc / sampleNum_GERO;
  cal_value_Ang = cal_value_Ang / sampleNum_GERO;
  cal_value_Gforce = cal_value_Gforce / sampleNum_GERO;
  Serial.print("Calibration accerlation: "); Serial.println(cal_value_Acc);
  Serial.print("Calibration orrintation: "); Serial.println(cal_value_Ang);
  Serial.print("Calibration Gforce: "); Serial.println(cal_value_Gforce);
  BTserial.println("GERO Calibration successfully");  // Return message
  BTserial.println("*******************************************");
}


void BT_Setup(){
    Serial.print("Sketch:   ");   Serial.println(__FILE__); 
    Serial.print("Uploaded: ");   Serial.println(__DATE__); 
    delay(3000);
    Serial.println("System initializing....."); 
    Serial.println("***************************************************************************************************");
    Serial.println("BTserial started at 9600 successfully"); // Local message
    BTserial.println("BTserial started at 9600 successfully");  // Return message
    delay(1500);
    BTserial.println("System initializing.....");
    BTserial.println("*******************************************");
}


void get_Flex() {
  double filter_Result = flex_AvarageFiltering(flexPin, sampleNum_Flex);
  value1 = (((filter_Result / sampleNum_Flex) / 1024.0) * 5.0) - cal_value1;
  if(abs(value1 - ini_Flex1) < damping){
    value1 = ini_Flex1;
  }
  ini_Flex1 = value1;
}

void get_Acc() {
    ACCERLATION = GERO_Filter().filter_Acc - cal_value_Acc;

}

void calculate_Acc() { // 
    user_Acc = 0;
    double ANGLE = 0;
    ANGLE = (GERO_Filter().filter_Ang - cal_value_Ang + 2) / 57.3;
    if(ACCERLATION <= 0){
      user_Acc = cos(ANGLE) * (ACCERLATION - cal_value_Gforce * sin(ANGLE));
    }
    if(ACCERLATION > 0){
      user_Acc = cos(ANGLE) * (ACCERLATION + cal_value_Gforce * sin(ANGLE));
    }
    if(abs(user_Acc - thres_Acc) < 0.5){
      user_Acc = thres_Acc;
    }
}

void GERO_Timer() {
  current_Time_Gero = millis();
  current_Acc = user_Acc;
  if(abs(current_Acc - previous_Acc) >= thres_Moving_Detected){
      num_Moving += 1;
      previous_Acc = current_Acc;
    }
  if(current_Time_Gero - previous_Time_Gero >= time_interval_GERO){
    previous_Time_Gero = current_Time_Gero;
    moving = false;
    if(num_Moving >= thres_Moving_Times){
      moving = true;
      Serial.println("Moving....");
      BTserial.println("Moving....");
      num_Moving = 0;
    }
    if(num_Moving < thres_Moving_Times && moving == false){
      Serial.println("Still....");
      BTserial.println("Still....");
    }
}
}


void alarm_Swelling() {
    current_Time_Flex = millis();
  if (abs(value1) >= thres_Swelling_Detected) {
    swelling_Times += 1;
  }
  if(current_Time_Flex - previous_Time_Flex >= time_interval_Flex){
    previous_Time_Flex = current_Time_Flex;
    swelling = false;
  if(swelling_Times >= thres_Swelling_Times){
    Serial.println("Swelling detected!!! ");
    BTserial.println("Swelling detected!!! ");
    swelling = true;
    swelling_Times = 0;      
    }
  if(swelling_Times < thres_Swelling_Times && swelling == false){
    Serial.println("Normal.... ");
    BTserial.println("Normal.... ");
    swelling_Times = 0;
  }
  }  
  }


void BT_CMDContral(String CMD[]){
   if (BTserial.available())
    {
      delay(20);
      while (BTserial.available()) {
        datarcv = BTserial.read();
        if(datarcv != '\n'){
          content.concat(datarcv);
        }
      }
      content.toLowerCase();
      content.trim();
      if(content == "start" && !connected){
        sys_Mode_Flags[4] = true;
        Serial.println("Connecting to the system...");
        BTserial.println("Connecting to the system...");
      }
      if(connected){
        Serial.print("You receive a CMD: "); Serial.println(content); // Local message
        BTserial.print("You sent a CMD: ");  BTserial.println(content); // Return message
        if(content == "start"){
          Serial.println("System has already started!"); // Local message
          BTserial.println("System has already started!"); // Return message
        }
        for(int i = 0; i < CMD_Num - 1; i++){
        CMD[i].toLowerCase();
        CMD[i].trim();
        if(content.equals(CMD[i]) && content != "start"){
          if(mode >= 1 && mode != i + 1){ // Clear the former different mode
            sys_Mode_Flags[mode - 1] = false;
          }
          if(mode == i + 1){ // Check if in the same mode
            Serial.print("You already in: ");  Serial.print(sys_CMD[mode - 1]);  Serial.println(" mode");
            BTserial.print("You already in: ");  BTserial.print(sys_CMD[mode - 1]);  BTserial.println(" mode");
          }else{ // Change to the new mode
            mode = i + 1; // To make a difference between the no input and mode zero
            sys_Mode_Flags[mode - 1] = true;
            Serial.print("Set: "); Serial.print(sys_CMD[mode - 1]);  Serial.println(" mode"); 
            BTserial.print("Set: "); BTserial.print(sys_CMD[mode - 1]);  BTserial.println(" mode"); 
          }
      }
      }
      }if(!connected && content != "start"){
        Serial.println("System offline! Please use Start cmd to start the system!");
        BTserial.println("System offline! Please use Start cmd to start the system!");
      }
      content = "";
    }
}


void setup(void) 
{
  Serial.begin(9600);
  BTserial.begin(9600);  
}


void loop(void) 
{

BT_CMDContral(sys_CMD); // BT CMD Unit

if(sys_Mode_Flags[0] == true) { // Gero mode
    get_Acc();
    calculate_Acc();
    GERO_Timer();
}

if(sys_Mode_Flags[1] == true) { // Flex mode
    get_Flex();
    alarm_Swelling();
}

if(sys_Mode_Flags[2] == true){ // Full mode
    get_Acc();
    calculate_Acc();
    GERO_Timer();
    if(moving == false){
    get_Flex();
    alarm_Swelling();
    }
}

if(sys_Mode_Flags[3] == true){ // Shutdown mode
  Serial.println("Shutting down....");
  BTserial.println("Shutting down....");
  Serial.println("System offline! Please use Start cmd to start the system!");
  BTserial.println("System offline! Please use Start cmd to start the system!");
  digitalWrite(RSTPIN, HIGH);
  pinMode( RSTPIN, OUTPUT);
  delay(4000);
  digitalWrite( RSTPIN , LOW);
  sys_Mode_Flags[3] == false;
}

if(sys_Mode_Flags[4] == true){ // Restart Mode
  BT_Setup();
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(800);
  bno.setExtCrystalUse(true);
  flex_Calibration();
  GERO_Calibration();
  Serial.println("***************************************************************************************************");
  BTserial.println("Connected! System is ready");
  BTserial.println("*******************************************");
  BTserial.println("Select Modes: Gero; Flex; Full; Shutdown ; Restart; FlexValue; GeroValue");
  BTserial.println("*******************************************");
  sys_Mode_Flags[4] = false;
  connected = true;
}

if(sys_Mode_Flags[5] == true){ // Value mode for Flex
  get_Flex();
  Serial.println(abs(value1));
  BTserial.println(abs(value1));
  delay(100);
}

if(sys_Mode_Flags[6] == true){ // Value mode for Gero
  get_Acc();
  calculate_Acc();
  Serial.println(user_Acc);
  BTserial.println(user_Acc);
  delay(100);
}

if(sys_Mode_Flags[7] == true && !connected){ // Start Mode
  BT_Setup();
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(800);
  bno.setExtCrystalUse(true);
  flex_Calibration();
  GERO_Calibration();
  Serial.println("***************************************************************************************************");
  BTserial.println("Connected! System is ready");
  BTserial.println("*******************************************");
  BTserial.println("Select Modes: Gero; Flex; Full; Shutdown ; Restart; FlexValue; GeroValue");
  BTserial.println("*******************************************");
  sys_Mode_Flags[4] = false;
  connected = true;
}

}