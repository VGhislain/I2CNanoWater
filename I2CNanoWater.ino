/*   
 *    Animhouse - Water component
 *    Version 2.1
 *    Last modification date : 29/07/2024
 *    
 *    Change history
 *    Version 1.0 - initial version
 *    Version 1.2 - replace the VL53L0X by IR detection to detect the cat
 *                - read water level only when the pump is not active
 *    Version 2.0 - replace the VL53L0X by an HX711 mesuring the weight of the water container
 *    Version 2.1 - add the recycle of the water and check if cat is present 
 *    
 */


#include <Wire.h>
#include <HX711.h>
#include <EEPROM.h>
#include <VGSCommon.h>
#include "TimeLib.h"

#define pinWaterFilling        6 //D6
#define pinIRCat               3 //D3

#define pinHX711DT             A0 //A0
#define pinHX711SCK            A1 //A1


#define pinLedRed     8   //D7
#define pinLedGreen   7   //D8
#define pinLedBlue    13  //D13 - LED_BUILTIN

#define LAST_COMMAND_DELAY  60000
#define BLINK_LED_DELAY 1000

#define deviceId VGS_DEVICE_I2C_ADDR_WATER

// Force EEPROM config re-initialization only when you intentionally want to erase and reset the stored settings.
// Set this to 1 temporarily for a reset, then put it back to 0 for normal operation.
#define FORCE_EEPROM_REINITIALIZATION 0


template< typename T > void Serial_printf(char * text, T data ){
    Serial.print(text);
    Serial.println(data);
  }

struct VGSDeviceConfigStruct{
  long iCounter = 0;
  long lWaterContainerEmptyWeight = 0;
  long lWaterContainerFullWeight = 0;
  int iWaterMode = 0;        //action device mode 0:OFF, 1:ON, 2:Auto in auto mode, water pump running when cat is detected, ON : water pump running, OFF : water pump off
  int iWaterFillLowPerc = 0; //percentage below wich an alarm should be sent
  int iWaterRecyclePerc = 0; //percentage of time the pump should run
  int iWaterCheckCatPresence = 0; //flag to check the cat presence or not before activating the recycle
  int iWaterDesiableAutoFill = 0; //flag to disable the auto fill if the cat doesn't like when pump is running
  int refData = VGS_CONFIG_REFDATA; //ref data to assure EEPROM is well saved - correct value is 12345
};

struct VGSDeviceSensorStruct{
  int waterFillPercentage;
  int waterFilling;
  int isCatDrinking;
  unsigned long catDrinkingDelay;
  unsigned long pumpStoppedDelay;
  int waterFillLowAlarm;
  int isCatPresent=0;  //value provided by the presense sensor -> disable if cat present in house
  unsigned long pumpLastOnTime;  //last time the pump was on
  unsigned long pumpLastOffTime; //last time the pump was off
  unsigned long lastCatDetectionTime;
  unsigned long systemTime;
};


char i2cBuffer[VGS_I2C_BUFFER_LEN+1];

VGSDeviceConfigStruct deviceConfig;
VGSDeviceSensorStruct deviceSensors;
unsigned long clockTime = 0;          //used to store the epoch time received from master

int iIndex=0;
int lastIRValueCatDrinking=0;

HX711 waterContainerScale;

unsigned long lastCommandTime;
unsigned long loggingDelay;
boolean connectedToMaster;
boolean redLedOn = false;
unsigned long redLedBlinkTime;
unsigned long redLedBlinkDelay;
unsigned long lastCommandMaxDelay;

//--- device config ---
void resetConfigToDefaults(){
  deviceConfig.iCounter = 0;
  deviceConfig.lWaterContainerEmptyWeight = 0;
  deviceConfig.lWaterContainerFullWeight = 0;
  deviceConfig.iWaterMode = 0;
  deviceConfig.iWaterFillLowPerc = 0;
  deviceConfig.iWaterRecyclePerc = 0;
  deviceConfig.iWaterCheckCatPresence = 0;
  deviceConfig.iWaterDesiableAutoFill = 0;
  deviceConfig.refData = VGS_CONFIG_REFDATA;
}

void logConfig(){
  Serial.println("Dump of deviceConfig");
  Serial.println("[");
  Serial_printf("iCounter=",deviceConfig.iCounter);
  Serial_printf("lWaterContainerEmptyWeight=",deviceConfig.lWaterContainerEmptyWeight);
  Serial_printf("lWaterContainerFullWeight=",deviceConfig.lWaterContainerFullWeight);
  Serial_printf("iWaterMode=",deviceConfig.iWaterMode);
  Serial_printf("iWaterFillLowPerc=",deviceConfig.iWaterFillLowPerc);
  Serial_printf("iWaterRecyclePerc=",deviceConfig.iWaterRecyclePerc);
  Serial_printf("iWaterCheckCatPresence=",deviceConfig.iWaterCheckCatPresence);
   Serial_printf("iWaterDesiableAutoFill=",deviceConfig.iWaterDesiableAutoFill);
  Serial_printf("refData=",deviceConfig.refData);
  Serial.println("]");
}

//
// -------------------------- Receive Event -----------------------------
//

void receiveEvent() {
  //Action received from master
  int iIndex=0;
  lastCommandTime = millis();
  connectedToMaster = true;
  
  memset(i2cBuffer, 0, VGS_I2C_BUFFER_LEN+1);
  int actionId = Wire.read();
  while(Wire.available()){
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);
    i2cBuffer[iIndex++]=c;        
  }
  Serial.println("New action to execute received");
  
  Serial_printf("Action id : ", actionId);
  Serial_printf("Action value : ",i2cBuffer);

  //handle action
  switch(actionId){
    case -1 :
      Serial.println("Scan command received from master");
    break;

    case VGS_ACTION_SET_AD_WATER_MODE :
      deviceConfig.iWaterMode = atoi(i2cBuffer);
      Serial.print("Water mode set to : ");
      Serial.println(deviceConfig.iWaterMode );
      saveConfig();
      if(deviceConfig.iWaterMode==VGS_WATER_MODE_OFF) {
        deviceSensors.pumpStoppedDelay = millis();
        deviceSensors.waterFilling = 0;
      }
      if(deviceConfig.iWaterMode==VGS_WATER_MODE_MANUAL){
        deviceSensors.waterFilling = 1;
      } 
    

    case VGS_ACTION_SET_WATER_PERC_LOW :
      deviceConfig.iWaterFillLowPerc = atoi(i2cBuffer);
      Serial.print("Water perc low set to : ");
      Serial.println(deviceConfig.iWaterFillLowPerc);
      saveConfig();
    break;

    case VGS_ACTION_SET_WATER_RECYCLE_PERC :
      deviceConfig.iWaterRecyclePerc = atoi(i2cBuffer);
      Serial.print("Water recycle percentage set to : ");
      Serial.println(deviceConfig.iWaterRecyclePerc);
      saveConfig();
    break;
      
    case VGS_ACTION_SET_WATER_RECYCLE_CHECK_CAT_PRESENCE :
      deviceConfig.iWaterCheckCatPresence = atoi(i2cBuffer);
      Serial.print("Check cat presense set to : ");
      Serial.println(deviceConfig.iWaterCheckCatPresence);
      saveConfig();
    break;

    case VGS_ACTION_SET_WATER_DISABLE_AUTO_FILL :
      deviceConfig.iWaterDesiableAutoFill = atoi(i2cBuffer);
      Serial.print("Disable auto fill set to : ");
      Serial.println(deviceConfig.iWaterDesiableAutoFill);
      saveConfig();
    break;

    case VGS_ACTION_SET_WATER_CONTAINER_EMPTY :
      deviceConfig.lWaterContainerEmptyWeight = waterContainerScale.read_average(10);
      Serial.println("Action : set water container empty");
      Serial.print("Value lWaterContainerEmptyWeight : ");
      Serial.println(deviceConfig.lWaterContainerEmptyWeight);
      saveConfig();
    break;

    case VGS_ACTION_SET_WATER_CONTAINER_FULL :
      deviceConfig.lWaterContainerFullWeight = waterContainerScale.read_average(10);
      Serial.println("Action : set water container full");
      Serial.print("Value lWaterContainerEmptyWeight : ");
      Serial.println(deviceConfig.lWaterContainerEmptyWeight);
      saveConfig();
    break;
    
    case VGS_ACTION_SET_CAT_PRESENT_WATER :
      deviceSensors.isCatPresent = atoi(i2cBuffer); //used by device presence to inform this device if the cat is in house
      Serial.print("isCatPresent (cat in house):");
      Serial.println(deviceSensors.isCatPresent);
    break;

    case VGS_ACTION_SET_SYSTEM_TIME :
      Serial_printf("Set system time to : ",atol(i2cBuffer));
      setDeviceTime(atol(i2cBuffer));
    break;
    
  }
}

//
// ----------------------------- Request Event -----------------------
//
void requestEvent(int nbr) {

  memset(i2cBuffer,0,VGS_I2C_BUFFER_LEN+1);
  int curItem=0;
  
  if(iIndex==curItem++) {sprintf(i2cBuffer,"S %d",deviceSensors.waterFilling); i2cBuffer[1]=VGS_SENSOR_WATER_FILLING;};
  if(iIndex==curItem++) {sprintf(i2cBuffer,"C %d",deviceConfig.iWaterMode); i2cBuffer[1]=VGS_CONFIG_WATER_MODE;};
  if(iIndex==curItem++) {sprintf(i2cBuffer,"S %d",deviceSensors.waterFillPercentage); i2cBuffer[1]=VGS_SENSOR_WATER_CONTAINER_P;};
  if(iIndex==curItem++) {sprintf(i2cBuffer,"S %d",deviceSensors.isCatDrinking); i2cBuffer[1]=VGS_SENSOR_CAT_DRINKING;};
  if(iIndex==curItem++) {sprintf(i2cBuffer, "S %lu",deviceSensors.lastCatDetectionTime); i2cBuffer[1] = VGS_SENSOR_WATER_LAST_EATING_TIME;};
  if(iIndex==curItem++) {sprintf(i2cBuffer,"S %d",deviceSensors.waterFillLowAlarm); i2cBuffer[1]=VGS_SENSOR_WATER_CONTAINER_ALARM;};
  if(iIndex==curItem++) {sprintf(i2cBuffer,"C %d",deviceConfig.iWaterRecyclePerc); i2cBuffer[1]=VGS_CONFIG_WATER_RECYCLE_PERC;};
  if(iIndex==curItem++) {sprintf(i2cBuffer,"C %d",deviceConfig.iWaterCheckCatPresence); i2cBuffer[1]=VGS_CONFIG_WATER_RECYCLE_CHECK_PRESENCE;};
  if(iIndex==curItem++) {sprintf(i2cBuffer,"C %d",deviceConfig.iWaterDesiableAutoFill); i2cBuffer[1]= VGS_CONFIG_WATER_DISABLE_AUTO_FILL;};
  if(iIndex==curItem++) {sprintf(i2cBuffer,"C %d",deviceConfig.iWaterFillLowPerc); i2cBuffer[1]=VGS_CONFIG_WATER_PERC_LOW;};
  if(iIndex==curItem++) {sprintf(i2cBuffer,"S %lu",deviceSensors.systemTime); i2cBuffer[1]=VGS_SENSOR_SYSTEM_TIME;};
   
    

  Serial.print("Buffer sent to master : ");
  Serial.println(i2cBuffer);

  Wire.write(i2cBuffer);
  iIndex++;
  if(iIndex==11) iIndex=0;
}

//=====================================================================================================================================================================
//===                                                            EEPROM - Config management                                                                         ===
//=====================================================================================================================================================================
//
void firstInitialization(){
    Serial.println("EEPROM first initialization");
    for (int i = 0 ; i < EEPROM.length() ; i++) {
      Serial.print(".");
      EEPROM.write(i, 0);
    }
    resetConfigToDefaults();
    saveConfig();
    Serial_printf("deviceConfig.iCounter",deviceConfig.iCounter);
    Serial_printf("deviceConfig.refData",deviceConfig.refData);
    Serial.println("\nInitialization done !");
}

void saveConfig(){
  deviceConfig.iCounter++;
  deviceConfig.refData = VGS_CONFIG_REFDATA;
  EEPROM.put(0, deviceConfig);
}

void readConfig(){
    EEPROM.get(0, deviceConfig);
}

boolean isInitialized(){
    if(deviceConfig.refData!=VGS_CONFIG_REFDATA) return false;
    if(deviceConfig.iCounter < 0) return false;
    if(deviceConfig.lWaterContainerEmptyWeight < 0) return false;
    if(deviceConfig.lWaterContainerFullWeight < 0) return false;
    if(deviceConfig.iWaterMode < 0) return false;
    if(deviceConfig.iWaterFillLowPerc < 0) return false;
    if(deviceConfig.iWaterRecyclePerc < 0) return false;
    if(deviceConfig.iWaterCheckCatPresence < 0) return false;
    if(deviceConfig.iWaterDesiableAutoFill < 0) return false;
    return true;
}

// *********************************************************************************************************************************************************************
// ***                                          DEVICE FUNCTIONS                                                                                                     ***
// *********************************************************************************************************************************************************************
// 
//
// --------------------------- set time function -------------------------
//
void setDeviceTime(unsigned long epochTime){
  clockTime = epochTime - millis()/1000;
}
//
// --------------------------- get time function -------------------------
//
unsigned long getDeviceTime(){
  return clockTime + millis()/1000;
}
//---------------------------- update sensors ----------------------------
//
void updateSensors(){
  unsigned long curTime = millis();
  deviceSensors.systemTime = getDeviceTime();

  int val=0; //iR sensor value set to 0

  //Serial.print("\t Cat drinking : ");
  //Serial.println(val);

  //Wait 4 sec after having stopped the pump before reading again
  //work around due to an instability after stopping the pump 
  //it has tendencies to restart immediately
  if((curTime - deviceSensors.pumpStoppedDelay) > 4000){
    val= digitalRead(pinIRCat);
    if(val==1) deviceSensors.catDrinkingDelay = curTime;
  }

  if(lastIRValueCatDrinking==0 && val==1){
    lastIRValueCatDrinking=1;
    deviceSensors.isCatDrinking=1;
    deviceSensors.lastCatDetectionTime = getDeviceTime();
    Serial.println("Cat is drinking");
  }
  
  if( deviceSensors.isCatDrinking==1 && (curTime-deviceSensors.catDrinkingDelay)>30000){
    //cat is considered as stopping drinking if no move after 30 sec
    //stop the water
    Serial.println("Cat is no more drinking"); 
    deviceSensors.isCatDrinking=0;
    deviceSensors.pumpStoppedDelay = curTime;
    lastIRValueCatDrinking=0;
    deviceSensors.waterFilling=0;
  }
 
  if(deviceConfig.iWaterMode==VGS_WATER_MODE_AUTO){
    //Water Mode is AUTO
    //Activate the pump if the cat is drinking 
    //or 
    //if it's not drinking but the percetage of recycle has to be executed
    if(deviceSensors.isCatDrinking) {
      if(!deviceConfig.iWaterDesiableAutoFill) deviceSensors.waterFilling=1; //Activate the pump only if the disable auto fill is not set
    }
    else{
      if(!deviceConfig.iWaterCheckCatPresence || !deviceSensors.isCatPresent ){
        //Serial.println("Cat not present or presence check not required");
        //we don't need to check the presence OR
        //the cat is not detected on eating nor sleeping zone
        //check the timing
        if(deviceConfig.iWaterRecyclePerc>0){
          //Serial_printf("Percentage timing set to : ",deviceConfig.iWaterRecyclePerc);
          if(deviceSensors.pumpLastOnTime>deviceSensors.pumpLastOffTime){
            //Serial.println("Pump is currently on a ON period");
            //pump is currently in a ON period
            //check if the time On reached the % defined in the config
            int onDurationSec = VGS_ACTION_PUMP_RUN_PERIOD / 100 * deviceConfig.iWaterRecyclePerc;
            //Serial_printf("Pump percentage ON period duration : ",onDurationSec);
            //Serial_printf("Pump current running period duration : ",(millis() - deviceSensors.pumpLastOnTime) / 1000);
            if((millis() - deviceSensors.pumpLastOnTime) / 1000 >= onDurationSec){
              //the pump is running for a period longer then the percentage defined, let's stop it
              //Serial.println("Pump on period expired : stop pump");
              deviceSensors.waterFilling=0;
              deviceSensors.pumpLastOffTime=millis();
            }
            else{
              //Serial.println("Pump on period not expired : keep pump on");
              deviceSensors.waterFilling=1;
            }
          }
          else{
            //pump is currently in a OFF period
                //Serial.println("Pump is currently on a OFF period");
            int offDurationSec = VGS_ACTION_PUMP_RUN_PERIOD / 100 * (100-deviceConfig.iWaterRecyclePerc);

            //Serial_printf("Pump percentage OFF period duration : ",offDurationSec);
            //Serial_printf("Pump off period : ",(millis() - deviceSensors.pumpLastOffTime) / 1000);

            if((millis() - deviceSensors.pumpLastOffTime) / 1000 >= offDurationSec){
              //the pump is stopped for a period longer then the percentage defined, let's activate it
              //Serial.println("Pump off period expired : start pump");
              deviceSensors.waterFilling=1;
              deviceSensors.pumpLastOnTime=millis();
            }
            else{
              //Serial.println("Pump off period not expired : keep pump off");
              deviceSensors.waterFilling=0;
            }
          }
        }
      }
    }
  }
  else if(deviceConfig.iWaterMode==VGS_WATER_MODE_OFF) {
    //Water Mode is OFF
    deviceSensors.waterFilling=0;
  } else if(deviceConfig.iWaterMode==VGS_WATER_MODE_MANUAL){
    //Water Mode is ON
    deviceSensors.waterFilling=1;
  }

 
  
  //measeure the current level of filling
  long lMeasure = waterContainerScale.read_average(10);
  long lCurWaterContainerWeight=-1;
  long lTotalWeight=-1;

  if(deviceConfig.lWaterContainerFullWeight==0 || deviceConfig.lWaterContainerEmptyWeight==0){
    Serial.print("Scale not calibrated !! ");
    deviceSensors.waterFillPercentage = -1;
  }
  else{
    int multFactor = 1;
    if(deviceConfig.lWaterContainerFullWeight < deviceConfig.lWaterContainerEmptyWeight) multFactor=-1;

    lCurWaterContainerWeight = multFactor * (lMeasure - deviceConfig.lWaterContainerEmptyWeight);
    if(abs(lMeasure) < abs(deviceConfig.lWaterContainerEmptyWeight * 0.8) ){
      Serial.println("Container removed - cancel filling");
      deviceSensors.waterFilling = 0;
    }
    if(lCurWaterContainerWeight<0) lCurWaterContainerWeight=0;

    lTotalWeight = multFactor * (deviceConfig.lWaterContainerFullWeight - deviceConfig.lWaterContainerEmptyWeight);
    if(lTotalWeight>0) {
      deviceSensors.waterFillPercentage = (lCurWaterContainerWeight  * 100) / lTotalWeight;
      if(deviceSensors.waterFillPercentage>99) deviceSensors.waterFillPercentage=99; 
    }
    else {
      deviceSensors.waterFillPercentage = -1;
    }
  }

  if(deviceSensors.waterFilling) {
    digitalWrite(pinWaterFilling,HIGH);
  }
  else {
    digitalWrite(pinWaterFilling,LOW);
  }

  if((millis() - loggingDelay)>3000){
    loggingDelay = millis();
  
  Serial.print("Cur measured value : ");
  Serial.print(lMeasure);
  Serial.print("\tCur container weight : ");
  Serial.print(lCurWaterContainerWeight);
  Serial.print("\tTotal weight : ");
  Serial.print(lTotalWeight);
  Serial.print("\tWater % :");
  Serial.print(deviceSensors.waterFillPercentage);  
  Serial.print("\tEmpty :");
  Serial.print(deviceConfig.lWaterContainerEmptyWeight);
  Serial.print("\tFull :");
  Serial.print(deviceConfig.lWaterContainerFullWeight);
  Serial.print("\tFilling :");
  Serial.print(deviceSensors.waterFilling);
  Serial.print("\tCat :");
  Serial.println(deviceSensors.isCatDrinking);
  
  }
  

  
  
  if(deviceSensors.waterFillPercentage<deviceConfig.iWaterFillLowPerc){
    deviceSensors.waterFillLowAlarm=1;
  }
  else{
    deviceSensors.waterFillLowAlarm=0;
  }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++                                                          SETUP and main LOOP                                                                          +++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void setup() {
  Serial.begin(115200);           // start serial for output

  //EEPROM management
  #if FORCE_EEPROM_REINITIALIZATION
    Serial.println("FORCE_EEPROM_REINITIALIZATION enabled - resetting EEPROM config");
    firstInitialization();
  #else
    readConfig();
    if(!isInitialized()){
        Serial.println("Config invalid or not initialized, resetting defaults");
        firstInitialization();
    }
    readConfig();
  #endif
  logConfig();

  //force re-initialize
//  deviceConfig.iCounter=0;
//  deviceConfig.waterContainerEmptyDistance=150;
//  deviceConfig.waterContainerFullDistance=10;
//  deviceConfig.catDrinkingDistance=250;
//  deviceConfig.iWaterMode=0; //action device mode 0:OFF, 1:ON, 2:Auto in auto mode, water pump running when cat is detected, ON : water pump running, OFF : water pump off
//  deviceConfig.iWaterFillLowPerc=20; //percentage below wich an alarm should be sent
//  deviceConfig.refData=9999;
  
  //if(!isInitialized()){
      //initialize for the first time the EEPROM (or re-initialize);
      //firstInitialization();
  //}

  
  Wire.begin(deviceId);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);

  //Initialize scale for the water container
  waterContainerScale.begin(pinHX711DT, pinHX711SCK);

  pinMode(pinIRCat, INPUT);
  pinMode(pinWaterFilling, OUTPUT);
  pinMode(pinLedRed, OUTPUT);
  pinMode(pinLedGreen, OUTPUT);
  pinMode(pinLedBlue, OUTPUT);


  digitalWrite(pinLedBlue,HIGH);
  for(int i=0; i<3; i++){
    digitalWrite(pinLedRed,LOW);
    digitalWrite(pinLedGreen,HIGH);
    delay(500);
    digitalWrite(pinLedGreen,LOW);
    digitalWrite(pinLedRed,HIGH);
    delay(500);  
  }
  digitalWrite(pinLedGreen,HIGH);
  digitalWrite(pinLedRed,HIGH);
  digitalWrite(pinLedBlue,LOW);
  lastCommandTime=0;;
  connectedToMaster=false;
  redLedBlinkTime=millis();
  redLedBlinkDelay = BLINK_LED_DELAY;
  lastCommandMaxDelay = LAST_COMMAND_DELAY;

  loggingDelay=millis();
  
}

void loop() {
  if((millis()-lastCommandTime)>lastCommandMaxDelay) connectedToMaster = false;
  if(!connectedToMaster){
    digitalWrite(pinLedGreen,HIGH);
    if((millis()-redLedBlinkTime) > redLedBlinkDelay){
      redLedBlinkTime = millis();
      redLedOn = !redLedOn;
      if(redLedOn)  digitalWrite(pinLedRed,LOW);
      else digitalWrite(pinLedRed,HIGH); 
    }
  }
  else {
    digitalWrite(pinLedRed,HIGH);
    digitalWrite(pinLedGreen,LOW);
  }
  
  updateSensors();
  delay(10);
 
}
