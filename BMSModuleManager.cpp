#include "config.h"
#include "BMSModuleManager.h"
#include <TelnetStream.h>
#include <mcp_can.h>
//#include "BMSUtil.h"



//extern EEPROMSettings settings;


BMSModuleManager::BMSModuleManager(int CS)
{
  CAN1 = new MCP_CAN(CS);
  for (int i = 1; i <= MAX_MODULE_ADDR; i++) { //62
    modules[i].setExists(false);
    modules[i].setAddress(i);
  }
  lowestPackVolt = 1000.0f;
  highestPackVolt = 0.0f;
  lowestPackTemp = 200.0f;
  highestPackTemp = -100.0f;
  isFaulted = false;
  balancing = false;
  balcnt = 0;//counter to stop balancing for cell measurement
}

bool BMSModuleManager::initCan(int INTr){
  
  CAN1_INT = INTr;
  if (CAN1->begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK){
    Serial.println("Batt MCP2515 Initialized Successfully!");
    CAN1->setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
    return 1;
  }else{
    Serial.println("Error Initializing Batt MCP2515...");
    return 0;
  }
 

  if(INTr > 0) pinMode(CAN1_INT, INPUT);                            // Configuring pin for /INT input
  
}

void BMSModuleManager::checkCan(){
    
   // if(!digitalRead(CAN1_INT)) { //
   while(CAN1->checkReceive() == 3) {

      
      CAN1->readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
      yield();
      //ESP.wdtFeed();
      if (rxId < 0x300)//do VW BMS magic if ids are ones identified to be modules
      {
          decodecan(rxId, rxBuf, candebug); //do VW BMS if ids are ones identified to be modules
      }
      if ((rxId & 0x1FFFFFFF) < 0x1A555440 && (rxId & 0x1FFFFFFF) > 0x1A555400)   // Determine if ID is Temperature CAN-ID
      {
          decodetemp(rxId, rxBuf, candebug, 1);
      }
      if ((rxId & 0x1FFFFFFF) < 0x1A5555FF && (rxId & 0x1FFFFFFF) > 0x1A5555EF)   // Determine if ID is Temperature CAN-ID FOR MEB
      {
          decodetemp(rxId, rxBuf, candebug, 2);
      }
      
      if(candebug){
            char msgString[128];
            if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
                sprintf(msgString, "******** %.8lX:", (rxId & 0x1FFFFFFF));
            else
                sprintf(msgString, "******** %.3lX,false,0,%1d", rxId, len);
    
            TelnetStream.print(msgString);
    
            if ((rxId & 0x40000000) == 0x40000000)
            { // Determine if message is a remote request frame.
                sprintf(msgString, "****** REMOTE REQUEST FRAME");
                TelnetStream.print(msgString);
            }
            else
            {
                for (byte i = 0; i < len; i++)
                {
                    sprintf(msgString, " %.2X", rxBuf[i]);
                    TelnetStream.print(msgString);
                }
            }
            TelnetStream.println();
      }
      yield();
    }
}

void BMSModuleManager::setCanDebug(bool deb){
  candebug = deb;
}

void BMSModuleManager::setDebug(bool deb){
  debug = deb;
}

void BMSModuleManager::sendCommand(){
  txId  = 0x0BA;
  len = 8;
  tx[0] = 0x00;
  tx[1] = 0x00;
  tx[2] = 0x00;
  tx[3] = 0x00;
  tx[4] = 0x00;
  tx[5] = 0x00;
  tx[6] = 0x00;
  tx[7] = 0x00;
  len = 8;
  ext = 0;
  CAN1->sendMsgBuf(txId, ext, len, tx);
  delay(1);
  tx[0] = 0x45;
  tx[1] = 0x01;
  tx[2] = 0x28;
  tx[3] = 0x00;
  tx[4] = 0x00;
  tx[5] = 0x00;
  tx[6] = 0x00;
  tx[7] = 0x30;
  CAN1->sendMsgBuf(txId, ext, len, tx);
}

void BMSModuleManager::sendbalancingtest()
{
  if (balinit == 0)
  {
    txId  = 0x1A555418;
    len = 8;
    ext = 1;
    tx[0] = 0xFE;
    tx[1] = 0xFE;
    tx[2] = 0xFE;
    tx[3] = 0xFE;
    tx[4] = 0xFE;
    tx[5] = 0xFE;
    tx[6] = 0xFE;
    tx[7] = 0xFE;
    CAN1->sendMsgBuf(txId, ext, len, tx);
    delay(1);

    txId  = 0x1A555419;
    CAN1->sendMsgBuf(txId, ext, len, tx);
    delay(1);

    txId  = 0x1A555416;
    CAN1->sendMsgBuf(txId, ext, len, tx);
    delay(1);

    txId  = 0x1A555417;
    CAN1->sendMsgBuf(txId, ext, len, tx);
    delay(1);
    balinit = 1;
  }

  if (balcycle == 1)
  {
    if (balon == 1)
    {
      txId  = 0x1A555418;
      len = 8;
      ext = 1;
      tx[0] = 0x08;
      tx[1] = 0x00;
      tx[2] = 0x00;
      tx[3] = 0x00;
      tx[4] = 0x00;
      tx[5] = 0x00;
      tx[6] = 0x00;
      tx[7] = 0x00;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);
      txId  = 0x1A555419;
      len = 8;
      ext = 1;
      tx[0] = 0x00;
      tx[1] = 0x00;
      tx[2] = 0x00;
      tx[3] = 0x00;
      tx[4] = 0xFE;
      tx[5] = 0xFE;
      tx[6] = 0xFE;
      tx[7] = 0xFE;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555416;
      len = 8;
      ext = 1;
      tx[0] = 0x00;
      tx[1] = 0x08;
      tx[2] = 0x00;
      tx[3] = 0x08;
      tx[4] = 0x00;
      tx[5] = 0x08;
      tx[6] = 0x00;
      tx[7] = 0x08;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);
      txId  = 0x1A555417;
      len = 8;
      ext = 1;
      tx[0] = 0x00;
      tx[1] = 0x08;
      tx[2] = 0x00;
      tx[3] = 0x08;
      tx[4] = 0xFE;
      tx[5] = 0xFE;
      tx[6] = 0xFE;
      tx[7] = 0xFE;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);
    }
    else
    {
      txId  = 0x1A555418;
      len = 8;
      ext = 1;
      tx[0] = 0x00;
      tx[1] = 0x00;
      tx[2] = 0x00;
      tx[3] = 0x00;
      tx[4] = 0x00;
      tx[5] = 0x00;
      tx[6] = 0x00;
      tx[7] = 0x00;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);
      txId  = 0x1A555419;
      len = 8;
      ext = 1;
      tx[0] = 0x00;
      tx[1] = 0x00;
      tx[2] = 0x00;
      tx[3] = 0x00;
      tx[4] = 0xFE;
      tx[5] = 0xFE;
      tx[6] = 0xFE;
      tx[7] = 0xFE;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555416;
      len = 8;
      ext = 1;
      tx[0] = 0x00;
      tx[1] = 0x00;
      tx[2] = 0x00;
      tx[3] = 0x00;
      tx[4] = 0x00;
      tx[5] = 0x00;
      tx[6] = 0x00;
      tx[7] = 0x00;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);
      txId  = 0x1A555417;
      len = 8;
      ext = 1;
      tx[0] = 0x00;
      tx[1] = 0x00;
      tx[2] = 0x00;
      tx[3] = 0x00;
      tx[4] = 0xFE;
      tx[5] = 0xFE;
      tx[6] = 0xFE;
      tx[7] = 0xFE;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);
    }
  }



  balcycle++;

  if ( balcycle > 10)
  {
    balcycle = 0;
  }

  ext = 0;
}


bool BMSModuleManager::checkcomms()
{
  int g = 0;
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      g = 1;
      if (modules[y].isReset())
      {
        //Do nothing as the counter has been reset
      }
      else
      {
        modules[y].setExists(false);
        return false;
      }
    }
    modules[y].setReset(false);
    modules[y].setAddress(y);
  }
  if ( g == 0)
  {
    return false;
  }
  return true;
}

void BMSModuleManager::setBalanceHyst(float newVal)
{
  BalHys = newVal;
  TelnetStream.println();
  TelnetStream.println(BalHys, 3);
}

void BMSModuleManager::balanceCells(int debug)
{
  uint16_t balance = 0;//bit 0 - 5 are to activate cell balancing 1-6
  //TelnetStream.println();
  // TelnetStream.println(LowCellVolt + BalHys, 3);
    if (balcnt > 60)
  {
    balcnt = 0;
  }
  
    if (balcnt > 10)
  {
    if (balcnt == 11 || balcnt == 15 || balcnt == 20 || balcnt == 25 || balcnt == 30 || balcnt == 35 || balcnt == 40 || balcnt == 45 || balcnt == 50 || balcnt == 55)
    {
      balancing = false;
      for (int y = 1; y < 63; y++)
      {
        if (modules[y].isExisting() == 1)
        {
          balance = 0;
          for (int i = 0; i < 12; i++)
          {
            if ((LowCellVolt + BalHys) < modules[y].getCellVoltage(i))
            {
              balance = balance | (1 << i);
            }
            /*
              else
              {
              Serial.print(" | ");
              Serial.print(i);
              }
            */
          }
          if (balance > 0)
          {
            balancing = true;
          }
          if (debug == 1)
          {
            TelnetStream.println();
            TelnetStream.print("Module ");
            TelnetStream.print(y);
            TelnetStream.print(" | ");
            TelnetStream.println(balance, HEX);

          }

          tx[0] = 0X00;
          tx[1] = 0X00;
          tx[2] = 0X00;
          tx[3] = 0X00;
          tx[4] = 0X00;
          tx[5] = 0X00;
          tx[6] = 0X00;
          tx[7] = 0X00;

          for (int i = 0; i < 8; i++)
          {
            if (bitRead(balance, i) == 1)
            {
              tx[i] = 0x08;
            }
            else
            {
              tx[i] = 0x00;
            }
          }

          switch (y)
          {
            case (1):
              txId  = 0x1A55540A;
              break;
            case (2):
              txId  = 0x1A55540C;
              break;
            case (3):
              txId  = 0x1A55540E;
              break;
            case (4):
              txId  = 0x1A555410;
              break;
            case (5):
              txId  = 0x1A555412;
              break;
            case (6):
              txId  = 0x1A555414;
              break;
            case (7):
              txId  = 0x1A555416;
              break;
            case (8):
              txId  = 0x1A555418;
              break;
            case (9):
              txId  = 0x1A55541A;
              break;
            case (10):
              txId  = 0x1A5554AB;
              break;
            case (11):
              txId  = 0x1A5554AD;
              break;
            case (12):
              txId  = 0x1A5554AF;
              break;

            default:
              break;
          }
          len = 8;
          ext = 1;
          CAN1->sendMsgBuf(txId, ext, len, tx);

          delay(1);

          for (int i = 8; i < 13; i++)
          {
            if (bitRead(balance, i) == 1)
            {
              tx[i - 8] = 0x08;
            }
            else
            {
              tx[i - 8] = 0x00;
            }
          }
          tx[4] = 0xFE;
          tx[5] = 0xFE;
          tx[6] = 0xFE;
          tx[7] = 0xFE;

          switch (y)
          {
            case (1):
              txId  = 0x1A55540B;
              break;
            case (2):
              txId  = 0x1A55540D;
              break;
            case (3):
              txId  = 0x1A55540F;
              break;
            case (4):
              txId  = 0x1A555411;
              break;
            case (5):
              txId  = 0x1A555413;
              break;
            case (6):
              txId  = 0x1A555415;
              break;
            case (7):
              txId  = 0x1A555417;
              break;
            case (8):
              txId  = 0x1A555419;
              break;
            case (9):
              txId  = 0x1A55541B;
              break;
            case (10):
              txId  = 0x1A5554AC;
              break;
            case (11):
              txId  = 0x1A5554AE;
              break;
            case (12):
              txId  = 0x1A5554B0;
              break;

            default:
              break;
          }
          len = 8;
          ext = 1;
          CAN1->sendMsgBuf(txId, ext, len, tx);
        }
      }

      if (balancing == false)
      {
        balcnt = 0;
      }
    }
  }
  else
  {
    if (balcnt == 1)
    {
      tx[0] = 0X00;
      tx[1] = 0X00;
      tx[2] = 0X00;
      tx[3] = 0X00;
      tx[4] = 0X00;
      tx[5] = 0X00;
      tx[6] = 0X00;
      tx[7] = 0X00;

      len = 8;
      ext = 1;

      txId  = 0x1A55540A;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A55540C;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A55540E;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555410;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555412;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555414;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555416;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555418;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A55541A;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A5554AB;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A5554AD;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A5554AF;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);


      tx[0] = 0X00;
      tx[1] = 0X00;
      tx[2] = 0X00;
      tx[3] = 0X00;
      tx[4] = 0xFE;
      tx[5] = 0xFE;
      tx[6] = 0xFE;
      tx[7] = 0xFE;

      txId  = 0x1A55540B;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A55540D;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A55540F;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555411;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555413;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555415;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555417;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A555419;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A55541B;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A5554AC;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A5554AE;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      txId  = 0x1A5554B0;
      CAN1->sendMsgBuf(txId, ext, len, tx);
      delay(1);

      balancing = false;
    }
  }
  balcnt++;
  ext = 0;
}

/*
 * CAN1->sendMsgBuf(txId, ext, len, tx);
 * txId  = 0x1A555415;
 * tx[4] = 0xFE;
 * TelnetStream.println();
 * 
 */



int BMSModuleManager::seriescells()
{
  spack = 0;
  for (int y = 1; y <= MAX_MODULE_ADDR; y++)
  {
    if (modules[y].isExisting())
    {
      spack = spack + modules[y].getscells();
    }
  }
  return spack;
}

void BMSModuleManager::clearmodules()
{
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      modules[y].clearmodule();
      modules[y].setExists(false);
      modules[y].setAddress(y);
    }
  }
}

void IRAM_ATTR BMSModuleManager::decodetemp(long unsigned int id, uint8_t rx[8], int debug, int type)
{
  int CMU = 0;
  if (type == 1)
  {
    CMU = (id & 0xFF);
    if (CMU > 10 && CMU < 60)
    {
      CMU = CMU & 0x0F;
      CMU = (CMU * 0.5) + 1;
    }
    if (CMU > 0 && CMU < 15);
    {
      modules[CMU].setExists(true);
      modules[CMU].setReset(true);
      modules[CMU].decodetemp(rx, 1);
      if (debug == 1)
      {
        Serial.println();
        Serial.print(CMU);
        Serial.print(" | Temp Found");
      }
    }
  }
  if (type == 2)
  {
    CMU = (id & 0x0F);
    if (CMU > 0 && CMU < 15);
    {
      CMU++;
      if (rx[5] != 0xDF) //Check module is not initializing OR a "spoof module"
      {
        modules[CMU].setExists(true);
        modules[CMU].setReset(true);
        modules[CMU].decodetemp(rx, 2);
        if (debug == 1)
        {
          Serial.println();
          Serial.print(CMU);
          Serial.print("|  Temp Found");
        }
      }
    }
  }
}

void IRAM_ATTR BMSModuleManager::decodecan(long unsigned int rxId, uint8_t rx[8], int debug)
{
  int CMU, Id = 0;
  if (balancing == false)
  {
    switch (rxId)
    {
      ///////////////// one extender increment//////////

      case (0x1D0):
        CMU = 9;
        Id = 0;
        break;
      case (0x1D1):
        CMU = 9;
        Id = 1;
        break;
      case (0x1D2):
        CMU = 9;
        Id = 2;
        break;
      case (0x1D3):
        CMU = 9;
        Id = 3;
        break;

      case (0x1D4):
        CMU = 10;
        Id = 0;
        break;
      case (0x1D5):
        CMU = 10;
        Id = 1;
        break;
      case (0x1D6):
        CMU = 10;
        Id = 2;
        break;
      case (0x1D8):
        CMU = 11;
        Id = 0;
        break;
      case (0x1D9):
        CMU = 11;
        Id = 1;
        break;
      case (0x1DA):
        CMU = 11;
        Id = 2;
        break;
      case (0x1DC):
        CMU = 12;
        Id = 0;
        break;
      case (0x1DD):
        CMU = 12;
        Id = 1;
        break;
      case (0x1DE):
        CMU = 12;
        Id = 2;
        break;

      case (0x1E0):
        CMU = 13;
        Id = 0;
        break;
      case (0x1E1):
        CMU = 13;
        Id = 1;
        break;
      case (0x1E2):
        CMU = 13;
        Id = 2;
        break;

      case (0x1E4):
        CMU = 14;
        Id = 0;
        break;
      case (0x1E5):
        CMU = 14;
        Id = 1;
        break;
      case (0x1E6):
        CMU = 14;
        Id = 2;
        break;

      case (0x1E8):
        CMU = 15;
        Id = 0;
        break;
      case (0x1E9):
        CMU = 15;
        Id = 1;
        break;
      case (0x1EA):
        CMU = 15;
        Id = 2;
        break;

      case (0x1EC):
        CMU = 16;
        Id = 0;
        break;
      case (0x1ED):
        CMU = 16;
        Id = 1;
        break;
      case (0x1EE):
        CMU = 16;
        Id = 2;
        break;


      ///////////////////////standard ids////////////////


      case (0x1B0):
        CMU = 1;
        Id = 0;
        break;
      case (0x1B1):
        CMU = 1;
        Id = 1;
        break;
      case (0x1B2):
        CMU = 1;
        Id = 2;
        break;
      case (0x1B3):
        CMU = 1;
        Id = 3;
        break;

      case (0x1B4):
        CMU = 2;
        Id = 0;
        break;
      case (0x1B5):
        CMU = 2;
        Id = 1;
        break;
      case (0x1B6):
        CMU = 2;
        Id = 2;
        break;
      case (0x1B7):
        CMU = 2;
        Id = 3;
        break;

      case (0x1B8):
        CMU = 3;
        Id = 0;
        break;
      case (0x1B9):
        CMU = 3;
        Id = 1;
        break;
      case (0x1BA):
        CMU = 3;
        Id = 2;
        break;
      case (0x1BB):
        CMU = 3;
        Id = 3;
        break;

      case (0x1BC):
        CMU = 4;
        Id = 0;
        break;
      case (0x1BD):
        CMU = 4;
        Id = 1;
        break;
      case (0x1BE):
        CMU = 4;
        Id = 2;
        break;
      case (0x1BF):
        CMU = 4;
        Id = 3;
        break;

      case (0x1C0):
        CMU = 5;
        Id = 0;
        break;
      case (0x1C1):
        CMU = 5;
        Id = 1;
        break;
      case (0x1C2):
        CMU = 5;
        Id = 2;
        break;
      case (0x1C3):
        CMU = 5;
        Id = 3;
        break;

      case (0x1C4):
        CMU = 6;
        Id = 0;
        break;
      case (0x1C5):
        CMU = 6;
        Id = 1;
        break;
      case (0x1C6):
        CMU = 6;
        Id = 2;
        break;
      case (0x1C7):
        CMU = 6;
        Id = 3;
        break;

      case (0x1C8):
        CMU = 7;
        Id = 0;
        break;
      case (0x1C9):
        CMU = 7;
        Id = 1;
        break;
      case (0x1CA):
        CMU = 7;
        Id = 2;
        break;
      case (0x1CB):
        CMU = 7;
        Id = 3;
        break;

      case (0x1CC):
        CMU = 8;
        Id = 0;
        break;
      case (0x1CD):
        CMU = 8;
        Id = 1;
        break;
      case (0x1CE):
        CMU = 8;
        Id = 2;
        break;
      case (0x1CF):
        CMU = 8;
        Id = 3;
        break;

      default:
        return;
        break;
    }
    if (CMU > 0 && CMU < 64)
    {
      if (Id < 3)
      {
        if (rx[2] != 0xFF && rx[5] != 0xFF && rx[7] != 0xFF) //Check module is not initializing OR a "spoof module"
        {
          if (debug == 1)
          {
            TelnetStream.println();
            TelnetStream.print(CMU);
            TelnetStream.print(",");
            TelnetStream.print(Id);
            TelnetStream.print(" | ");
          }
          modules[CMU].setExists(true);
          modules[CMU].setReset(true);
          modules[CMU].decodecan(Id, rx);
        }
      }
      else
      {
        if (rx[2] != 0xFF) //Check module is not initializing OR a "spoof module"
        {
          if (debug == 1)
          {
            TelnetStream.println();
            TelnetStream.print(CMU);
            TelnetStream.print(",");
            TelnetStream.print(Id);
            TelnetStream.print(" | ");
          }
          modules[CMU].setExists(true);
          modules[CMU].setReset(true);
          modules[CMU].decodecan(Id, rx);
        }
      }
    }
  }
}


void BMSModuleManager::getAllVoltTemp(bool debug)
{
  packVolt = 0.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++) //62
  {
    if (modules[x].isExisting())
    {
      if(debug){
        sprintf(msgString, "Module %i exists. Reading voltage and temperature values", x);
        TelnetStream.println(msgString);
        sprintf(msgString,"Module voltage: %f", modules[x].getModuleVoltage());
        TelnetStream.println(msgString);
        sprintf(msgString,"Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
        TelnetStream.println(msgString);
        sprintf(msgString,"Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));
        TelnetStream.println(msgString);
      }
      packVolt += modules[x].getModuleVoltage();
      if (modules[x].getLowTemp() < lowestPackTemp) lowestPackTemp = modules[x].getLowTemp();
      if (modules[x].getHighTemp() > highestPackTemp) highestPackTemp = modules[x].getHighTemp();
    }
  }

  packVolt = packVolt / Pstring;
  if (packVolt > highestPackVolt) highestPackVolt = packVolt;
  if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;

  if (false){ //digitalRead(11) == LOW) {
    if (!isFaulted) TelnetStream.println("One or more BMS modules have entered the fault state!");
    isFaulted = true;
  }
  else
  {
    if (isFaulted) TelnetStream.println("All modules have exited a faulted state");
    isFaulted = false;
  }
}

float BMSModuleManager::getLowCellVolt()
{
  LowCellVolt = 5.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getLowCellV() <  LowCellVolt)  LowCellVolt = modules[x].getLowCellV();
    }
  }
  return LowCellVolt;
}

float BMSModuleManager::getHighCellVolt()
{
  HighCellVolt = 0.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getHighCellV() >  HighCellVolt)  HighCellVolt = modules[x].getHighCellV();
    }
  }
  return HighCellVolt;
}

float BMSModuleManager::getPackVoltage()
{
  return packVolt;
}

int BMSModuleManager::getNumModules()
{
  return numFoundModules;
}

float BMSModuleManager::getLowVoltage()
{
  return lowestPackVolt;
}

float BMSModuleManager::getHighVoltage()
{
  return highestPackVolt;
}

void BMSModuleManager::setBatteryID(int id)
{
  batteryID = id;
}

void BMSModuleManager::setPstrings(int Pstrings)
{
  Pstring = Pstrings;
}

void BMSModuleManager::setSensors(int sensor, float Ignore, float VoltDelta)
{
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      modules[x].settempsensor(sensor);
      modules[x].setIgnoreCell(Ignore);
      modules[x].setDelta(VoltDelta);
    }
  }
}

float BMSModuleManager::getAvgTemperature()
{
  float avg = 0.0f;
  lowTemp = 999.0f;
  highTemp = -999.0f;
  int y = 0; //counter for modules below -70 (no sensors connected)
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getAvgTemp() > -70)
      {
        avg += modules[x].getAvgTemp();
        if (modules[x].getHighTemp() > highTemp)
        {
          highTemp = modules[x].getHighTemp();
        }
        if (modules[x].getLowTemp() < lowTemp)
        {
          lowTemp = modules[x].getLowTemp();
        }
      }
      else
      {
        y++;
      }
    }
  }
  avg = avg / (float)(numFoundModules - y);

  return avg;
}

float BMSModuleManager::getHighTemperature()
{
  return highTemp;
}

float BMSModuleManager::getLowTemperature()
{
  return lowTemp;
}

float BMSModuleManager::getAvgCellVolt()
{
  numFoundModules = 0;
  float avg = 0.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getAverageV() > 0)
      {
        avg += modules[x].getAverageV();
        numFoundModules++;
      }
    }
  }
  avg = avg / (float)numFoundModules;

  return avg;
}

void BMSModuleManager::printPackSummary()
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;

  TelnetStream.println("");
  TelnetStream.println("");
  TelnetStream.println("");
  
  sprintf(msgString,"Modules: %i  Cells: %i  Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules, seriescells(),
                getPackVoltage(), getAvgCellVolt(), getAvgTemperature());
  TelnetStream.println(msgString);
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      sprintf(msgString,"                               Module #%i", y);
      TelnetStream.println(msgString);
      sprintf(msgString,"  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(),
                      modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());
      TelnetStream.println(msgString);
      if (faults > 0)
      {
        TelnetStream.println("  MODULE IS FAULTED:");
        if (faults & 1)
        {
          TelnetStream.print("    Overvoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 12; i++)
          {
            if (COV & (1 << i))
            {
              TelnetStream.print(i + 1);
              TelnetStream.print(" ");
            }
          }
          TelnetStream.println();
        }
        if (faults & 2)
        {
          TelnetStream.print("    Undervoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 12; i++)
          {
            if (CUV & (1 << i))
            {
              TelnetStream.print(i + 1);
              TelnetStream.print(" ");
            }
          }
          TelnetStream.println();
        }
        if (faults & 4)
        {
          TelnetStream.println("    CRC error in received packet");
        }
        if (faults & 8)
        {
          TelnetStream.println("    Power on reset has occurred");
        }
        if (faults & 0x10)
        {
          TelnetStream.println("    Test fault active");
        }
        if (faults & 0x20)
        {
          TelnetStream.println("    Internal registers inconsistent");
        }
      }
      if (alerts > 0)
      {
        TelnetStream.println("  MODULE HAS ALERTS:");
        if (alerts & 1)
        {
          TelnetStream.println("    Over temperature on TS1");
        }
        if (alerts & 2)
        {
          TelnetStream.println("    Over temperature on TS2");
        }
        if (alerts & 4)
        {
          TelnetStream.println("    Sleep mode active");
        }
        if (alerts & 8)
        {
          TelnetStream.println("    Thermal shutdown active");
        }
        if (alerts & 0x10)
        {
          TelnetStream.println("    Test Alert");
        }
        if (alerts & 0x20)
        {
          TelnetStream.println("    OTP EPROM Uncorrectable Error");
        }
        if (alerts & 0x40)
        {
          TelnetStream.println("    GROUP3 Regs Invalid");
        }
        if (alerts & 0x80)
        {
          TelnetStream.println("    Address not registered");
        }
      }
      if (faults > 0 || alerts > 0) TelnetStream.println();
    }
  }
}

void BMSModuleManager::printPackDetails(int digits)
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;
  int cellNum = 0;

 
  TelnetStream.println("");
  TelnetStream.println("");
  TelnetStream.println("");
  sprintf(msgString, "Modules: %i Cells: %i Strings: %i  Voltage: %fV   Avg Cell Voltage: %fV  Low Cell Voltage: %fV   High Cell Voltage: %fV Delta Voltage: %f mV   Avg Temp: %fC ", numFoundModules, seriescells(), Pstring, getPackVoltage(), getAvgCellVolt(), LowCellVolt, HighCellVolt, (HighCellVolt - LowCellVolt) * 1000, getAvgTemperature());
  TelnetStream.println(msgString);
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      TelnetStream.print("Module #");
      TelnetStream.print(y);
      if (y < 10) TelnetStream.print(" ");
      TelnetStream.print("  ");
      TelnetStream.print(modules[y].getModuleVoltage(), digits);
      TelnetStream.print("V");
      for (int i = 0; i < 13; i++)
      {
        if (cellNum < 10) TelnetStream.print(" ");
        TelnetStream.print("  Cell");
        TelnetStream.print(cellNum++);
        TelnetStream.print(": ");
        TelnetStream.print(modules[y].getCellVoltage(i), digits);
        TelnetStream.print("V");
      }
      TelnetStream.println();
      TelnetStream.print(" Temp 1: ");
      TelnetStream.print(modules[y].getTemperature(0));
      TelnetStream.print("C Temp 2: ");
      TelnetStream.print(modules[y].getTemperature(1));
      TelnetStream.print("C Temp 3: ");
      TelnetStream.print(modules[y].getTemperature(2));
      TelnetStream.print("C | Bal Stat: ");
      TelnetStream.println(modules[y].getBalStat(), HEX);
    }
  }
}
void BMSModuleManager::printAllCSV(unsigned long timestamp, float current, int SOC)
{
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      TelnetStream.print(timestamp);
      TelnetStream.print(",");
      TelnetStream.print(current, 0);
      TelnetStream.print(",");
      TelnetStream.print(SOC);
      TelnetStream.print(",");
      TelnetStream.print(y);
      TelnetStream.print(",");
      for (int i = 0; i < 8; i++)
      {
        TelnetStream.print(modules[y].getCellVoltage(i));
        TelnetStream.print(",");
      }
      TelnetStream.print(modules[y].getTemperature(0));
      TelnetStream.print(",");
      TelnetStream.print(modules[y].getTemperature(1));
      TelnetStream.print(",");
      TelnetStream.print(modules[y].getTemperature(2));
      TelnetStream.println();
    }
  }
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      TelnetStream.print(timestamp);
      TelnetStream.print(",");
      TelnetStream.print(current, 0);
      TelnetStream.print(",");
      TelnetStream.print(SOC);
      TelnetStream.print(",");
      TelnetStream.print(y);
      TelnetStream.print(",");
      for (int i = 0; i < 13; i++)
      {
        TelnetStream.print(modules[y].getCellVoltage(i));
        TelnetStream.print(",");
      }
      TelnetStream.print(modules[y].getTemperature(0));
      TelnetStream.print(",");
      TelnetStream.print(modules[y].getTemperature(1));
      TelnetStream.print(",");
      TelnetStream.print(modules[y].getTemperature(2));
      TelnetStream.println();
    }
  }
}
