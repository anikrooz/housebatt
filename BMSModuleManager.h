#pragma once
#include "config.h"
#include "BMSModule.h"
#include <mcp_can.h>

#define MAX_MODULE_ADDR     0x3E

class BMSModuleManager
{
  public:
    BMSModuleManager(int);
    bool initCan(int INTr);
    void checkCan();
    int seriescells();
    void clearmodules();
    void decodecan(long unsigned int id, uint8_t rx[8], int debug);
    void decodetemp(long unsigned int id, uint8_t rx[8], int debug, int type);
    void getAllVoltTemp(bool debug);
    void readSetpoints();
    void setBatteryID(int id);
    void setPstrings(int Pstrings);
    void setUnderVolt(float newVal);
    void setOverVolt(float newVal);
    void setOverTemp(float newVal);
    void setBalanceV(float newVal);
    void setBalanceHyst(float newVal);
    void setSensors(int sensor, float Ignore, float VoltDelta);
    void balanceCells(int debug);
    void sendCommand();
    void sendbalancingtest();
    float getPackVoltage();
    float getAvgTemperature();
    float getHighTemperature();
    float getLowTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighCellVolt();
    float getHighVoltage();
    float getLowVoltage();
    String getCellJson(String a);
    void setDebug(bool deb);
    void setCanDebug(bool deb);
    /*
      void processCANMsg(CAN_FRAME &frame);
    */
    void printAllCSV(unsigned long timestamp, float current, int SOC);
    void printPackSummary();
    void printPackDetails(int digits);
    int getNumModules();
    bool checkcomms();
    MCP_CAN *CAN1;
     //Balance testing
    int balinit = 0;
    int balon = 0;
    int balcycle = 0;   

  private:
    float packVolt;                         // All modules added together
    int Pstring;
    float LowCellVolt;
    float HighCellVolt;
    float lowestPackVolt;
    float highestPackVolt;
    float lowestPackTemp;
    float highestPackTemp;
    float highTemp;
    float lowTemp;
    float BalHys;
    BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.
    int batteryID;
    int numFoundModules;                    // The number of modules that seem to exist
    bool isFaulted;
    int spack;
    bool candebug = 0;
    bool debug = 0;
    int CAN1_INT;
    long unsigned int txId;
    long unsigned int rxId;
    uint8_t rxBuf[8];
    uint8_t tx[8];
    unsigned char len;
    bool ext;
    bool balancing;
    uint8_t balcnt;

    char msgString[128];
    /*
      void sendBatterySummary();
      void sendModuleSummary(int module);
      void sendCellDetails(int module, int cell);
    */

};
