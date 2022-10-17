#include "CONFIG.h"
//#include <ESP8266WiFi.h> //If using ESP8266, beware it hangs sometimes somewhere in BMSModule::decodecan. Likely just the flash not being quick enough. Adding IRAM_ATTR helped a bit. but not enough!
//#include <ESP8266mDNS.h> //These watchdog timer reboots are tolerable if you're not using a charger that kicks out 250A whenever it stops receiving heartbeats over CAN
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h> //https://www.arduino.cc/reference/en/libraries/arduinoota/
#include <TelnetStream.h> //https://github.com/jandrassy/TelnetStream
#include <mcp_can.h>
#include "emerson-Vertiv-R48/VertivPsu.h"
#include <TinyMqtt.h>   // https://github.com/hsaturn/TinyMqtt
#include "BMSModuleManager.h"
//#include <Filters.h> //https://github.com/JonHub/Filters
#include <Adafruit_ADS1X15.h>
#include <Preferences.h> //https://github.com/vshymanskyy/Preferences

Preferences prefs;

Adafruit_ADS1115 ads;

#include "secrets.h"  //with your SSID and password... 
                    //...OR comment that out and stick it here
#ifndef STASSID
#define STASSID "YourSSID"
#define STAPSK  "YourPassword"
#endif

// ------  Hardware dependent stuff ....

#define CAN0_CS 5 //ChipSelect for CAN0 MCP2515
#define CAN0_INT 0 //I'm not using interrupts so follow this through and edit the module to use it.

VertivPSU charger(CAN0_CS);

//I have 2 BMS CAN buses due to duplicated module numbers. If you don't have duplicate modules, use just one and edit / "BMS aggregation functions" at the bottom

#define CAN1_INT 0 //I'm not using interrupts so follow this through and edit the module to use it.
#define CAN2_INT 0
#define CAN1_CS 4    //ChipSelect for CAN1 MCP2515                    
#define CAN2_CS 13   //ChipSelect for CAN2 MCP2515
BMSModuleManager bms(CAN1_CS);
BMSModuleManager bms2(CAN2_CS);

#define ADS_SCL 22
#define ADS_SDA 21

#define RS485_TRA 2 //DriverEnable, go HI to send

const char* ssid = STASSID;
const char* password = STAPSK;

bool chCANdebug = 0; //turn on with D
bool batCANdebug = 0; //turn on with B
bool debug = 0; //turn off with d
bool batStats = 1; // b
bool pauseTelnet = 0;

const char *mqtt_broker = "192.168.178.240";  // put IP address of your MQTT broker here
const char *topic = "meterbox/grid";          // and topic to subscribe to
const int mqtt_port = 1883;
static MqttClient client;

//Curent filter//
float filterFrequency = 5.0 ;
//FilterOnePole lowpassFilter( LOWPASS, filterFrequency );

const int maxOutput = 1000; //edit this to limit TOTAL power output in watts (not individual unit output)
const int importbuffer = 20;
const int maxChargerpower = -2800; //max charging power, limited by temp, cell imbalance, etc
const int minChargerpower = -150; //stop charging less than this
const int startChargerPower = -300; //don't turn on charger less than this
const int minOutput = 50; //no inverter less than this
const int chargerIdleACoff = 600000; //10 mins

int importingnow = 0; //amount of electricity being imported from grid
int demand = 0; //current power inverter should deliver (default to zero)




//settings
const float OverVSetpoint = 4.2;
const float  UnderVSetpoint = 2.8;
const float ChargeVsetpoint = 4.1;
// batt start for inverter 47v; shutdown 43v. Nominal is 44.4!
const float ChargeHys = 0.1; // voltage drop required for charger to kick back on
const float    CellGap = 0.2; //max delta between high and low cell
const float  OverTSetpoint = 65.0;
const float  UnderTSetpoint = -10.0;
const float  ChargeTSetpoint = 0.0;
const float  DisTSetpoint = 40.0;
const float WarnToff = 5.0; //temp offset before raising warning
const float IgnoreTemp = 0; // 0 - use both sensors, 1 or 2 only use that sensor
const float IgnoreVolt = 2.9;//
const float balanceVoltage = 3.9;
const float balanceHyst = 0.04;
const float DeltaVolt = 0.5; //V of allowable difference between measurements
const int CAP = 22.5; //25; //battery size in Ah
int socvolt[4] = {3100, 10, 4100, 90}; 
/*socvolt[0] = 3100; //Voltage and SOC curve for voltage based SOC calc
socvolt[1] = 10; //Voltage and SOC curve for voltage based SOC calc
socvolt[2] = 4100; //Voltage and SOC curve for voltage based SOC calc
socvolt[3] = 90; //Voltage and SOC curve for voltage based SOC calc*/
const int Pstrings = 9; //// TOTAL strings in parallel used to divide voltage of pack
const int Scells = 12;//Cells in series
const int bms1Pstrings = 3; //cells in parallel on this CAN bus
const int bms2Pstrings = 6;

const float StoreVsetpoint = 3.8; // V storage mode charge max
const int discurrentmax = 300; // max discharge current in 0.1A
const float DisTaper = 0.3; //V offset to bring in discharge taper to Zero Amps at settings.DischVsetpoint
const float DischVsetpoint = 3.2;
const int chargecurrentmax = 500; //max charge current in 0.1A
const int chargecurrentend = 50; //end charge current in 0.1A
const int chargecurrentcold = 50;
int maxedDiscurrent = discurrentmax;
int maxedChargecurrent = chargecurrentmax;
//Variables for SOC calc
int SOC = 100; //State of Charge
int SOCset = 0;
int SOCtest = 0;
int SOCmem = 0;
int cellspresent = 0;
String error;

bool canCharge = 0;
bool canInvert = 0;
unsigned long lastMQTT = 0;
int timeout = 15000;
unsigned long chargerRampup = 0;
int chargerRamptime = 8500; //don't alter demand for this time, let charger ramp up
long unsigned int cleartime = 0;
long unsigned int lastChargeTime = 0;


bool fullyCharged = 0;
bool charging = 0;
bool inverting = 0;
bool balancing = 0;
float bmAmps; //negavive is TO battery
float bVolts;
//variables for current calulation
float ampsecond;
unsigned long lasttime;
float cAmps = 1;

bool chargeOverride = 0; //1 is on, toggled by c
bool invertOverride = 0; //or BMS things

byte serialpacket[8] = {36, 86, 0, 33, 0, 0, 128, 8};

float gridUse;
bool twiOk = 0;
bool flipCurrent = 1;

// functions


void setupOTA();
void readTelnet();
void currentlimit();
void SOCcharged(int y);
void updateSOC();
void cls();
float getHighCellVolt();
float getLowCellVolt();
float getAvgCellVolt();
float getHighTemperature();
float getLowTemperature();


/*
 * 
 *      MQTT Callback
 * 
 */



void IRAM_ATTR mqttCallback(const MqttClient* /* source */, const Topic& topic, const char* payload, size_t /* length */)
{ 
  if(!debug && !chCANdebug && !batCANdebug) cls();
  if(!chCANdebug && !batCANdebug && !pauseTelnet) TelnetStream << "-->                                  Grid flow: " << topic.c_str() << ", " << payload << endl;
  lastMQTT = millis();
  
  if(!SOCset) return; //let it settle
  if(chargerRampup > millis()) return charger.setCAmps(cAmps);
  
  
  importingnow = String(payload).toInt();
    
    // -- Compute demand signal --    
    //importingnow = importingnow-importbuffer; //target grid demand minus buffer
  //demand = bVolts * bmAmps/1000 * (bmAmps < 0 ? 1.04 : 0.9); //actual currently going to (-) / from (+) battery (+ 5% for mains)
  if(bmAmps == 0) demand = 0; //don't care what we asked it last time; we got nothing!
  if(!chCANdebug && !batCANdebug && !pauseTelnet) TelnetStream.println("-->                             Batt right now: " + (String)demand);
  demand = demand + importingnow; //add grid import to current demand, expects that grid import will be negative if exporting
  if(!chCANdebug && !batCANdebug && !pauseTelnet) TelnetStream.println("-->                                     Demand: " + (String)demand);
  
    //limit demand between maxs
  if (demand >= maxOutput) demand = maxOutput;
  if(demand < maxChargerpower) demand = maxChargerpower;

  digitalWrite(RS485_TRA, HIGH); //flashes LED off briefly, too
  delay(20);

  if(demand > minOutput){  //consuming power
      if(!canInvert) {
        demand = 0;
        inverting = 0;
      }else if(charging) {
        //sublime straight to inverting
        TelnetStream.println("                  ----------- STOP CHARGING ------------");
        if (client.connected()) client.publish("goatshed/status", "STOP CHARGING -> invert");
        demand = 0; //give it one cycle to stop
        charging = 0;
        charger.switchACpower(0);
        charger.setCAmps(1);
        lastChargeTime = millis();
        //charger.setCPerc(0);
      }else{
        demand -= importbuffer;
        serialpacket[4] =  demand >> 8;
        serialpacket[5] = demand >> 0;
        serialpacket[7] = 264 - serialpacket[4] - serialpacket[5];
        demand += importbuffer;
        Serial.write(serialpacket,8);
        Serial.flush();
        inverting = 1;
      }

      


  }else if(demand < minChargerpower && canCharge){ 
        inverting = 0;
        cAmps = ((0-demand)-importbuffer)/charger.voutput;
        if(cAmps > maxedChargecurrent*0.1) cAmps = maxedChargecurrent*0.1;
        if(!charging && demand < startChargerPower) {
          lastChargeTime = millis();
          TelnetStream.println("                  ----------- START CHARGING ------------");
          TelnetStream.println();
          if (client.connected()) client.publish("goatshed/status", "START CHARGING");
          //start charging... set initial to actual demand!
          //demand = importingnow;
          chargerRampup = millis() + chargerRamptime;
          //charger.setCPerc(100);
          charger.setCAmps(cAmps*0.8);
          charger.switchACpower(1);
          charging = 1;
          //cAmps = 1; //slow for first 5 s
        }else if(charging){
          lastChargeTime = millis();
          charger.setCAmps(cAmps);
          if(!chCANdebug && !batCANdebug && !pauseTelnet) TelnetStream.println("---------                       Set Charge Amps: " + (String)cAmps);
          //if(bmAmps > 2000) flipCurrent = !flipCurrent;
        }else{
          demand = 0; 
        }
  }else{
    //demand small, so ignored (or not allowed to charge)
    demand = 0;
    if(charging) {
        TelnetStream.println("                  ----------- STOP CHARGING ------------");
        if (client.connected()) client.publish("goatshed/status", "STOP CHARGING too small");
    }
    charging = 0;
    inverting = 0;
    charger.switchACpower(0);
    charger.setCAmps(1);
    //charger.setCPerc(0);
  }
    //importingnow = importingnow+importbuffer; //remove change to actual import value
    
  if(!chCANdebug && !batCANdebug && !pauseTelnet) TelnetStream.println("-->                                FinalDemand: " + (String)demand);
  if (client.connected() && demand != 0) client.publish("goatshed/status", "Demand: " + (String)demand);
    

    

      /*char msgStr[64];
      for (byte i = 0; i < 8; i++) {
        sprintf(msgStr, " %.2X", serialpacket[i]);
        TelnetStream.print(msgStr);
      }
      TelnetStream.println();*/
    digitalWrite(RS485_TRA, LOW);
   
}


/*
 * SETUP
 * 
 */

void setup() {
  Serial.begin(115200);
  //TelnetStream.println("Booting");
  Serial.println("alive");

  prefs.begin("goatshed");

  //pinMode(CAN1_CS, OUTPUT); 
  //digitalWrite(CAN1_CS, HIGH); //no cselect
  
  Serial.println("Wifi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    //ESP.restart();
  }

  
  TelnetStream.begin();
  Serial.println("Setup OTA");
  setupOTA();
  Serial.println("init Can0");
  charger.init(CAN0_INT);

  Serial.println("Setup ADS");
  ads.setGain(GAIN_SIXTEEN);
  twiOk = ads.begin();
  if(!twiOk) Serial.println("Failed to initialize ADS.");

  Serial.println("Init can1");
  bms.initCan(CAN1_INT);
  bms2.initCan(CAN2_INT);
  Serial.println("aftercan");

  //prefs.putInt("SOC", 86);
  //recover SOC
  SOC = prefs.getInt("SOC", 0); // default to 0
  if(SOC != 0) {
    SOCmem = 1;
    ampsecond = SOC * CAP * Pstrings * 36 ;
  }
  
  Serial.flush();

  client.connect(mqtt_broker, mqtt_port);  // Put here your broker ip / port
  client.setCallback(mqttCallback);
  client.subscribe(topic);


  Serial.println("aftermqtt");

  bms.setPstrings(bms1Pstrings);
  bms.setSensors(IgnoreTemp, IgnoreVolt, DeltaVolt);
  bms.setBalanceHyst(balanceHyst);

  bms2.setPstrings(bms2Pstrings);
  bms2.setSensors(IgnoreTemp, IgnoreVolt, DeltaVolt);
  bms2.setBalanceHyst(balanceHyst);
  
  Serial.println("alive. Serial reconfig...");
  pinMode(RS485_TRA, OUTPUT);
  digitalWrite(RS485_TRA, LOW); //receive
 
  Serial.flush();
  Serial.end();
  delay(20);

  Serial.begin(4800);
  
  if (client.connected()) client.publish("goatshed/status", " --- BOOT ---");
  cleartime = 0;
    
}






/*
 *    LOOP
 * 
 */


void loop() {


  //run loops
  client.loop();
  charger.tick();
  ArduinoOTA.handle();
  bms.checkCan();
  yield();
  bms2.checkCan();
  
  canInvert = bms.getPackVoltage() > 36 && getHighCellVolt() > UnderVSetpoint && invertOverride; //inverter stops at 43 anyway :(

  if(charging && getHighCellVolt() > ChargeVsetpoint) {
    TelnetStream.println("                  ----------- STOP CHARGING, fully charged ------------");
    if (client.connected()) client.publish("goatshed/status", "STOP CHARGING, fully charged");
    charger.switchACpower(0);
    charging = 0;
    fullyCharged = 1;
    if (getAvgCellVolt() > (ChargeVsetpoint - ChargeHys)) //avg over 3.9
         SOCcharged(2); //100
      else
         SOCcharged(1); //95
  }


  //reset fullyCharged at -0.2v
  if(fullyCharged && (bms.getPackVoltage() < 49.2) && (getHighCellVolt() < ChargeVsetpoint - ChargeHys)) fullyCharged = 0;
  
  canCharge = chargeOverride && !fullyCharged && charger.isConnected();


  // ---- Current measurement -----
  static unsigned long next_measure = millis()+4000;
  if(!twiOk) twiOk = ads.begin();
  if(twiOk && millis() > next_measure){
    next_measure = millis() + 250;
    
    int16_t results = ((float)ads.readADC_Differential_0_1()) * 256 / 32.768 * 1.3333; //+ve is OUT of batt
    //if(millis() > 1500 && millis() < 5000 && bmAmps > 1000) flipCurrent = 1; // no other way we can be using in that time
    
    bmAmps = results * (flipCurrent ? 1 : -1); //((float)results * 256.0) / 32768 * 1.3333;//75mv->100A shunt
    if(bmAmps < 35 && bmAmps > 0) bmAmps = 0; //filter out zero jitter
    if(bmAmps > -35 && bmAmps < 0) bmAmps = 0;
    
    if((charger.ioutput > 0 && bmAmps > 0) || (charger.ioutput == 0 && inverting && bmAmps < 0)){
      flipCurrent = !flipCurrent;
      bmAmps = bmAmps * -1;
    }
    
    //lowpassFilter.input(results);
    //bmAmps = lowpassFilter.output();
    if(SOCset) ampsecond = ampsecond - ((bmAmps * (millis() - lasttime) / 1000) / 1000);
    lasttime = millis();

  }
  


  /*
   * --- stuff do do every 1/2 sec ---
  *
  *
   */

  static unsigned long next_send = millis()+8000; //10s before first one to allow OTA in case of stupidity

  
  if (millis() > next_send)
  {
      next_send = millis() + 500;
      //report stuff whether we're connected to charger or not
      if (not client.connected()) client.connect(mqtt_broker, mqtt_port);
      if (client.connected()) client.publish("battery/amps", String(bmAmps/1000));
      if (client.connected()) client.publish("goatshed/uptime", String(millis()/1000));
      if (client.connected() && SOCset) client.publish("battery/soc", String(SOC));
      if (client.connected() && SOCset) client.publish("battery/amphour", String(ampsecond/3600));
      if (client.connected()) client.publish("battery/error", error);
    
    
    
   
    //if(!charging && millis() - lastChargeTime > chargerIdleACoff) charger.switchACpower(0);

    //      balancing
    
    if(SOCset && getHighCellVolt() > balanceVoltage && getHighCellVolt() > getLowCellVolt() + balanceHyst) {
        if(!pauseTelnet) TelnetStream.println("Balancing");
        balancing = 1;
        bms.balanceCells(0);    //1 is debug
        bms2.balanceCells(0);
    }else{
        balancing = 0;
    }

    updateSOC();
    currentlimit();
    bms.sendCommand();
    bms2.sendCommand();


    if(charger.isConnected()){      // don't do anything that could destabilise before charger is under control
              //  checks
             
              bms.sendbalancingtest();
              bms.getAvgCellVolt(); //needed to set scells
              bms2.sendbalancingtest();
              bms2.getAvgCellVolt(); //needed to set scells
             

              //check we have all cells!!
              if (cellspresent == 0 && bms.seriescells() == (Scells * bms1Pstrings) && bms2.seriescells() == (Scells * bms2Pstrings))
              {
                bms.setSensors(IgnoreTemp, IgnoreVolt, DeltaVolt);
                bms2.setSensors(IgnoreTemp, IgnoreVolt, DeltaVolt);
          
                cellspresent = bms.seriescells() + bms2.seriescells();
                if (client.connected()) client.publish("goatshed/status", "cellspresent SET " + cellspresent);
                
                error = "";
                invertOverride = 1;
                chargeOverride = 1;
                SOCset = 1;
              }
              else if(SOCset == 1)
              {
                if (cellspresent != bms.seriescells() + bms2.seriescells()) //detect a fault in cells detected
                {
                  if (debug != 0)
                  {
                    TelnetStream.println("  ");
                    TelnetStream.print("   !!! Series Cells Fault !!!");
                    TelnetStream.println("  ");
                  }
                  error = "Series Cells Fault: " + String(bms.seriescells()) + " + " + String(bms2.seriescells());
                  if (client.connected()) client.publish("goatshed/status", "Series Cells Fault: " + String(bms.seriescells())) + " + " + String(bms2.seriescells());
                  invertOverride = 0;
                  chargeOverride = 0;
                }
                else
                {
                  //no error
                  //if (client.connected()) client.publish("goatshed/status", "No error, all modules cool");
                  if(error != ""){
                    invertOverride = 1;
                    chargeOverride = 1;
                    error = "";
                  }
                }
              }else{
                if (client.connected()) client.publish("goatshed/status", "Series Cells: " + String(bms.seriescells())) + " + " + String(bms2.seriescells());
                TelnetStream.print("Cells found: " + String(bms.seriescells()));
                TelnetStream.println(" + " + String(bms2.seriescells()));
              }
              
              if(batStats && !pauseTelnet) TelnetStream.println("BMS1");
              bms.getAllVoltTemp(batStats && !pauseTelnet);  
              if(batStats && !pauseTelnet) TelnetStream.println("BMS2");
              bms2.getAllVoltTemp(batStats && !pauseTelnet);  
              bVolts = bms2.getPackVoltage();
                // assume they're the same voltage since in parallel!
              
              //TelnetStream.println(String(bVolts) + "v");
              if(!pauseTelnet) TelnetStream.println("----           --               ------           Amps: " + String(bmAmps/1000) + " @ " + String(bVolts) + " v, Watts: " + String(bmAmps/1000*bVolts));
          
           
              if (client.connected()) client.publish("battery/volt", String(bVolts));
              if (client.connected()) client.publish("battery/highcell", String(getHighCellVolt()));
              if (client.connected()) client.publish("battery/lowcell", String(getLowCellVolt()));
              if (client.connected()) client.publish("battery/balancing", String(balancing));
              if (client.connected() && SOCset) client.publish("battery/temp", String(bms.getAvgTemperature()));
        
              TelnetStream.flush();

        }else if(millis() > 60000){
          //not connected (or disconnected) after 1 min, BAD.
          ESP.restart();
        }
  }




  

  // ----- MQTT Timeout

  if(millis() > lastMQTT + timeout){
    lastMQTT += timeout;
    TelnetStream.println("----- !!!! ----- Timeout, input not received ----- !!!! -----");
    charger.switchACpower(0);
    charging = 0;
    charger.setCAmps(1);
    //inverter will die itself
  }





  if (SOCset == 1 && millis() - cleartime > 20000)     //here, check all batteries are still communicating
  {
    if (bms.checkcomms() && bms2.checkcomms())
    {
      if(debug) TelnetStream.println("BMS ok");
    }
    else
    {
      //missing module
      if (debug != 0)
      {
        TelnetStream.println("  ");
        TelnetStream.print("   !!! MODULE MISSING !!!");
        TelnetStream.println("  ");
      }
      error = "module missing"; //was there but has not talked in 20s
      invertOverride = 0;
      chargeOverride = 0; //recovery from this is only via manual input, telnet or reboot
    }
    //bms.clearmodules(); // Not functional
    cleartime = millis();
  }


  readTelnet();

}

// ***** End loop ***********




void cls(){
      if(pauseTelnet) return;
      TelnetStream.write(27);
      TelnetStream.print("[2J");
      TelnetStream.write(27);
      TelnetStream.print("[H"); 
      TelnetStream.println("R eboot | r estart charging | d ebug | C harger CAN | B att CAN | c harger on | i inverter on | % recalc SOC | ! 100% SOC | p rint batt stats");
}



void updateSOC()
{
  if (SOCset == 0 && SOCmem == 0)
  {
    if (millis() > 15000 && bms.seriescells() == (Scells * bms1Pstrings) && bms2.seriescells() == (Scells * bms2Pstrings))
    {
      //{3100, 10, 4100, 90}; 
      TelnetStream.println("LowVolt: " + String(getLowCellVolt() * 1000));
      SOC = map(uint16_t(getLowCellVolt() * 1000), socvolt[0], socvolt[2], socvolt[1], socvolt[3]);

      ampsecond = SOC * CAP * Pstrings * 36 ;
      SOCset = 1;
      TelnetStream.println("  ");
      TelnetStream.println("//////////////////////////////////////// SOC SET //////////////////////////////////////// " + String(SOC));
      if (client.connected()) client.publish("goatshed/status", "SOC SET: "+ String(SOC));
      error = "";
    }else{
      error = "Counted cells: " + String(bms.seriescells()) + " + "  + String(bms2.seriescells());
    }
  }

  int lastSOC = SOC;
  SOC = ((ampsecond * 0.27777777777778) / (CAP * Pstrings * 1000)) * 100;
  if(lastSOC != SOC) prefs.putInt("SOC", SOC);
  if (SOC >= 100)
  {
    ampsecond = (CAP * Pstrings * 1000) / 0.27777777777778 ; //reset to full
    SOC = 100;
  }


  if (SOC < 0)
  {
    SOC = 0; //reset SOC this way the can messages remain in range for other devices. Ampseconds will keep counting.
  }

  if (!pauseTelnet)
  {
    TelnetStream.print("------  ");
    TelnetStream.print(bmAmps);
    TelnetStream.print("mA");
    TelnetStream.print("  ");
    TelnetStream.print(SOC);
    TelnetStream.print("% SOC ");
    TelnetStream.print(ampsecond * 0.27777777777778, 2);
    TelnetStream.println ("mAh");
  }
}





void SOCcharged(int y)
{
  if (y == 1)
  {
    SOC = 95;
    prefs.putInt("SOC", SOC);
    ampsecond = (CAP * Pstrings * 1000) / 0.27777777777778 ; //reset to full, dependent on given capacity. 
  }
  if (y == 2)
  {
    SOC = 100;
    prefs.putInt("SOC", SOC);
    ampsecond = (CAP * Pstrings * 1000) / 0.27777777777778 ; //reset to full
  }
}


void currentlimit()
{
  
  if (invertOverride == 0) //error
  {
    maxedDiscurrent = 0;
  }else if(chargeOverride == 0){
    maxedChargecurrent = 0;
  }else{

    ///Start at no derating///
    maxedDiscurrent = discurrentmax; // *10 A
    maxedChargecurrent = chargecurrentmax;


    ///////All hard limits to into zeros
    if (getLowTemperature() < UnderTSetpoint)
    {
      //discurrent = 0; Request Daniel
      maxedChargecurrent = chargecurrentcold;
    }
    if (getHighTemperature() > OverTSetpoint)
    {
      maxedDiscurrent = 0;
      maxedChargecurrent = 0;
    }
    if (getHighCellVolt() > OverVSetpoint)
    {
      maxedChargecurrent = 0;
    }
    if (getHighCellVolt() > OverVSetpoint)
    {
      maxedChargecurrent = 0;
    }
    if (getLowCellVolt() < UnderVSetpoint || getLowCellVolt() < DischVsetpoint)
    {
      maxedDiscurrent = 0;
    }


    //Modifying discharge current///

    if (maxedDiscurrent > 0)
    {
      //Temperature based///

      if (getHighTemperature() > DisTSetpoint)
      {
        maxedDiscurrent = maxedDiscurrent - map(getHighTemperature(), DisTSetpoint, OverTSetpoint, 0, discurrentmax);
      }
      //Voltage based///
      if (getLowCellVolt() < (DischVsetpoint + DisTaper))
      {
        maxedDiscurrent = maxedDiscurrent - map(getLowCellVolt(), DischVsetpoint, (DischVsetpoint + DisTaper), discurrentmax, 0);
      }

    }

    //Modifying Charge current///
    if (maxedChargecurrent > chargecurrentcold) // >5A
    {
      //Temperature based///
      if (getLowTemperature() < ChargeTSetpoint) //0
      {
        maxedChargecurrent = maxedChargecurrent - map(getLowTemperature(), UnderTSetpoint, ChargeTSetpoint, (chargecurrentmax - chargecurrentcold), 0);
      }
      //Voltagee based///

      if (getHighCellVolt() > (ChargeVsetpoint - ChargeHys)) // 4.1 - 0.2
      {
        //TelnetStream.println(" maxedChargeCurrent ==== " + String(maxedChargecurrent));
       // TelnetStream.println(" getHighCellVolt ==== " + String(bms.getHighCellVolt()));
        //TelnetStream.println(" chargecurrentmax ==== " + String(chargecurrentmax));
       
        //TelnetStream.println(" :::: Minus :::: " + String(map(bms.getHighCellVolt(), (ChargeVsetpoint - ChargeHys), ChargeVsetpoint, 0, (chargecurrentmax - chargecurrentend))));
        maxedChargecurrent = maxedChargecurrent - (((getHighCellVolt() - (ChargeVsetpoint - ChargeHys)) / ChargeHys) * (chargecurrentmax - chargecurrentend));
        //maxedChargecurrent = maxedChargecurrent - map(bms.getHighCellVolt(), (ChargeVsetpoint - ChargeHys), ChargeVsetpoint, 0, (chargecurrentmax - chargecurrentend));
        //TelnetStream.println(" maxedChargeCurrent ==== " + String(maxedChargecurrent));
      }
    }

  }
  ///No negative currents///

  if (maxedDiscurrent < 0)
  {
    maxedDiscurrent = 0;
  }
  if (maxedChargecurrent < 0)
  {
    maxedChargecurrent = 0;
  }
}

void readTelnet(){
  switch (TelnetStream.read()) {
    case 'R':
      TelnetStream.println("eboot!");
      TelnetStream.stop();
      delay(100);
      ESP.restart();
      break;
    case '-':
      flipCurrent = !flipCurrent;
      break;
    case 'r':
      //restart charging
      TelnetStream.println("Restart charging");
      fullyCharged = 0;
    case 'd':
      debug = !debug;
      TelnetStream.println("ebug -------- > "+ String(debug));
      charger.setDebug(debug);
      bms.setDebug(debug);
      bms2.setDebug(debug);
      break;
    case 'C':
      chCANdebug = !chCANdebug;
      TelnetStream.println("ebug Charger CAN ------- > "+ String(chCANdebug));
      charger.setCanDebug(chCANdebug);
      break; 
    case 'b':
      batStats = !batStats;
      TelnetStream.println("attery stats ------- > "+ String(batStats));
      break;    
    case 'B':
      batCANdebug = !batCANdebug;
      TelnetStream.println("attery CAN debug ------- > "+ String(batCANdebug));
      bms.setCanDebug(batCANdebug);
      bms2.setCanDebug(batCANdebug);
      break;     
    case 'c':
      chargeOverride = !chargeOverride;
      pauseTelnet = 1;
      TelnetStream.println("harging to ------- > " + String(chargeOverride));
      TelnetStream.println("SPACE to continue...");
      break;
    case 'i':
      invertOverride = !invertOverride;
      pauseTelnet = 1;
      TelnetStream.println("nverter to ------- >  " + String(invertOverride));
      TelnetStream.println("SPACE to continue...");
      break;
    case '%': //recalc SOC based on voltage
      SOCset = 0;
      SOCmem = 0;
      break;
    case '!': //set SOC to 100%
      SOCcharged(1);
      break;
    case 'p': //print All batts
      debug = 1;
      batStats = 0;
      pauseTelnet = 1;
      cls();
      //bms.printAllCSV(millis(), bmAmps, SOC);
      bms.printPackDetails(2);
      bms2.printPackDetails(2);
      TelnetStream.println("SPACE to continue...");
      break;
    case ' ': //pause
      pauseTelnet = !pauseTelnet;
      break;
  }
}


void setupOTA(){
  ArduinoOTA.setHostname("GoatShed");


  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    TelnetStream.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    TelnetStream.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    cls();
    TelnetStream.printf("Progress: %u%%\r", (progress / (total / 100)));
    //TelnetStream.println("Progress: " + String(progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    TelnetStream.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      TelnetStream.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      TelnetStream.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      TelnetStream.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      TelnetStream.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      TelnetStream.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  //TelnetStream.println("Ready");
  //TelnetStream.print("IP address: ");
  //TelnetStream.println(WiFi.localIP());

  
}


/* BMS aggregation functions - replaces 'bms.doSomething'
 *  
 */
float getHighCellVolt(){
  return std::max(bms.getHighCellVolt(), bms2.getHighCellVolt());
}

float getLowCellVolt(){
  return std::min(bms.getLowCellVolt(), bms2.getLowCellVolt());
}

float getAvgCellVolt(){
  return (bms.getAvgCellVolt() + bms2.getAvgCellVolt()) / 2;
}

float getHighTemperature(){
  return std::max(bms.getHighTemperature(), bms2.getHighTemperature());
}

float getLowTemperature(){
  return std::min(bms.getLowTemperature(), bms2.getLowTemperature());
}
