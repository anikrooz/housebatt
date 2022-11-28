#include "config.h"
#include "BMSModule.h"
#include <TelnetStream.h>

#define MAX_MODULE_ADDR     0x3E

BMSModule::BMSModule()
{
  for (int i = 0; i < 13; i++)
  {
    cellVolt[i] = 0.0f;
    lowestCellVolt[i] = 5.0f;
    highestCellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  temperatures[2] = 0.0f;
  lowestTemperature = 200.0f;
  highestTemperature = -100.0f;
  lowestModuleVolt = 200.0f;
  highestModuleVolt = 0.0f;
  balstat = 0;
  exists = false;
  reset = false;
  moduleAddress = 0;
  timeout = 30000; //milliseconds before comms timeout;
}

void BMSModule::clearmodule()
{
  for (int i = 0; i < 13; i++)
  {
    cellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  temperatures[2] = 0.0f;
  balstat = 0;
  exists = false;
  reset = false;
  moduleAddress = 0;
}

void IRAM_ATTR BMSModule::decodetemp(uint8_t rx[8], int y)
{
   if (y==1) //0x00 in byte 2 means its an MEB message
  {
    type = 1;
    if (rx[7] == 0xFD)
    {
      if (rx[2] != 0xFD)
      {
        temperatures[0] = (rx[2] * 0.5) - 40;
      }
    }
    else
    {
      if (rx[0] < 0xDF)
      {
        temperatures[0] = (rx[0] * 0.5) - 43;
        balstat = rx[2] + (rx[3] << 8);
      }
      else
      {
        temperatures[0] = (rx[3] * 0.5) - 43;
      }
      if (rx[4] < 0xF0)
      {
        temperatures[1] = (rx[4] * 0.5) - 43;
      }
      else
      {
        temperatures[1] = 0;
      }
      if (rx[5] < 0xF0)
      {
        temperatures[2] = (rx[5] * 0.5) - 43;
      }
      else
      {
        temperatures[2] = 0;
      }
    }
  }
  else
  {
    type = 2;
    temperatures[0] = ((uint16_t(((rx[5] & 0x0F) << 4) | ((rx[4] & 0xF0) >> 4))) * 0.5) - 40; //MEB Bits 36-44
  }
}

void IRAM_ATTR BMSModule::decodecan(int Id, uint8_t rx[8], int Mod)
{
  switch (Id)
  {
    case 0:
      cmuerror = 0;
      cellVolt[0] = (uint16_t(rx[1] >> 4) + uint16_t(rx[2] << 4) + 1000) * 0.001;
      cellVolt[2] = (uint16_t(rx[5] << 4) + uint16_t(rx[4] >> 4) + 1000) * 0.001;
      cellVolt[1] = (rx[3] + uint16_t((rx[4] & 0x0F) << 8) + 1000) * 0.001;
      cellVolt[3] = (rx[6] + uint16_t((rx[7] & 0x0F) << 8) + 1000) * 0.001;

 
      break;
    case 1:
      cmuerror = 0;
      cellVolt[4] = (uint16_t(rx[1] >> 4) + uint16_t(rx[2] << 4) + 1000) * 0.001;
      cellVolt[6] = (uint16_t(rx[5] << 4) + uint16_t(rx[4] >> 4) + 1000) * 0.001;
      cellVolt[5] = (rx[3] + uint16_t((rx[4] & 0x0F) << 8) + 1000) * 0.001;
      cellVolt[7] = (rx[6] + uint16_t((rx[7] & 0x0F) << 8) + 1000) * 0.001;
 
      break;

    case 2:
      cmuerror = 0;
      cellVolt[8] = (uint16_t(rx[1] >> 4) + uint16_t(rx[2] << 4) + 1000) * 0.001;
      cellVolt[10] = (uint16_t(rx[5] << 4) + uint16_t(rx[4] >> 4) + 1000) * 0.001;
      cellVolt[9] = (rx[3] + uint16_t((rx[4] & 0x0F) << 8) + 1000) * 0.001;
      cellVolt[11] = (rx[6] + uint16_t((rx[7] & 0x0F) << 8) + 1000) * 0.001;
      //for my one dodgy mod3 cell 11...
      if(Mod == 3 && cellVolt[10] - cellVolt[11] > 0.3) cellVolt[11] = cellVolt[10];
      

      break;

    case 3:
      cmuerror = 0;
      cellVolt[12] = (uint16_t(rx[1] >> 4) + uint16_t(rx[2] << 4) + 1000) * 0.001;
      break;

    default:
      break;

  }
  if (getLowTemp() < lowestTemperature) lowestTemperature = getLowTemp();
  if (getHighTemp() > highestTemperature) highestTemperature = getHighTemp();

  for (int i = 0; i < 13; i++)
  {
    if (lowestCellVolt[i] > cellVolt[i] && cellVolt[i] >= IgnoreCell) lowestCellVolt[i] = cellVolt[i];
    if (highestCellVolt[i] < cellVolt[i] && cellVolt[i] > 5.0) highestCellVolt[i] = cellVolt[i];
  }

  if (cmuerror == 0)
  {
    lasterror = millis();
  }
  else
  {
    if (millis() - lasterror < timeout)
    {
      if (lasterror + timeout - millis() < 5000)
      {
        TelnetStream.println("  ");
        TelnetStream.print("Module");
        TelnetStream.print(moduleAddress);
        TelnetStream.print("Counter Till Can Error : ");
        TelnetStream.println(lasterror + timeout - millis() );
      }
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        cellVolt[i] = 0.0f;
      }
      moduleVolt = 0.0f;
      temperatures[0] = 0.0f;
      temperatures[1] = 0.0f;
      temperatures[2] = 0.0f;
    }
  }
}


/*
  Reading the status of the board to identify any flags, will be more useful when implementing a sleep cycle
*/

uint8_t BMSModule::getFaults()
{
  return faults;
}

uint8_t BMSModule::getAlerts()
{
  return alerts;
}

uint8_t BMSModule::getCOVCells()
{
  return COVFaults;
}

uint8_t BMSModule::getCUVCells()
{
  return CUVFaults;
}

float BMSModule::getCellVoltage(int cell)
{
  if (cell < 0 || cell > 13) return 0.0f;
  return cellVolt[cell];
}

float BMSModule::getLowCellV()
{
  float lowVal = 10.0f;
  for (int i = 0; i < 13; i++) if (cellVolt[i] < lowVal && cellVolt[i] > IgnoreCell) lowVal = cellVolt[i];
  return lowVal;
}

float BMSModule::getHighCellV()
{
  float hiVal = 0.0f;
  for (int i = 0; i < 13; i++)
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 5.0)
    {
      if (cellVolt[i] > hiVal) hiVal = cellVolt[i];
    }
  return hiVal;
}

float BMSModule::getAverageV()
{
  int x = 0;
  float avgVal = 0.0f;
  for (int i = 0; i < 13; i++)
  {
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 5.0)
    {
      x++;
      avgVal += cellVolt[i];
    }
  }

  scells = x;
  avgVal /= x;

  if (scells == 0)
  {
    avgVal = 0;
  }

  return avgVal;
}

int BMSModule::getscells()
{
  return scells;
}

float BMSModule::getHighestModuleVolt()
{
  return highestModuleVolt;
}

float BMSModule::getLowestModuleVolt()
{
  return lowestModuleVolt;
}

float BMSModule::getHighestCellVolt(int cell)
{
  if (cell < 0 || cell > 13) return 0.0f;
  return highestCellVolt[cell];
}

float BMSModule::getLowestCellVolt(int cell)
{
  if (cell < 0 || cell > 13) return 0.0f;
  return lowestCellVolt[cell];
}

float BMSModule::getHighestTemp()
{
  return highestTemperature;
}

float BMSModule::getLowestTemp()
{
  return lowestTemperature;
}

float BMSModule::getLowTemp()
{
  if (sensor == 0)
  {
    if (getAvgTemp() > 0.5)
    {
      if (temperatures[0] > 0.5)
      {
        if (temperatures[0] < temperatures[1] && temperatures[0] < temperatures[2])
        {
          return (temperatures[0]);
        }
      }
      if (temperatures[1] > 0.5)
      {
        if (temperatures[1] < temperatures[0] && temperatures[1] < temperatures[2])
        {
          return (temperatures[1]);
        }
      }
      if (temperatures[2] > 0.5)
      {
        if (temperatures[2] < temperatures[1] && temperatures[2] < temperatures[0])
        {
          return (temperatures[2]);
        }
      }
    }
  }
  else
  {
    return temperatures[sensor - 1];
  }
  return 0;
}

float BMSModule::getHighTemp()
{
  if (sensor == 0)
  {
    return (temperatures[0] < temperatures[1]) ? temperatures[1] : temperatures[0];
  }
  else
  {
    return temperatures[sensor - 1];
  }
}

float BMSModule::getAvgTemp()
{
  if (sensor == 0)
  {
    if ((temperatures[0] + temperatures[1] + temperatures[2]) / 3.0f > 0.5)
    {
      if (temperatures[0] > 0.5 && temperatures[1] > 0.5 && temperatures[2] > 0.5)
      {
        return (temperatures[0] + temperatures[1] + temperatures[2]) / 3.0f;
      }
      if (temperatures[0] < 0.5 && temperatures[1] > 0.5 && temperatures[2] > 0.5)
      {
        return (temperatures[1] + temperatures[2]) / 2.0f;
      }
      if (temperatures[0] > 0.5 && temperatures[1] < 0.5 && temperatures[2] > 0.5)
      {
        return (temperatures[0] + temperatures[2]) / 2.0f;
      }
      if (temperatures[0] > 0.5 && temperatures[1] > 0.5 && temperatures[2] < 0.5)
      {
        return (temperatures[0] + temperatures[1]) / 2.0f;
      }
      if (temperatures[0] > 0.5 && temperatures[1] < 0.5 && temperatures[2] < 0.5)
      {
        return (temperatures[0]);
      }
      if (temperatures[0] < 0.5 && temperatures[1] > 0.5 && temperatures[2] < 0.5)
      {
        return (temperatures[1]);
      }
      if (temperatures[0] < 0.5 && temperatures[1] < 0.5 && temperatures[2] > 0.5)
      {
        return (temperatures[2]);
      }
      if (temperatures[0] < 0.5 && temperatures[1] < 0.5 && temperatures[2] < 0.5)
      {
        return (-80);
      }
    }
  }
  else
  {
    return temperatures[sensor - 1];
  }
  return 0;
}

float BMSModule::getModuleVoltage()
{
  moduleVolt = 0;
  for (int j=0; j < 13; j++)
  {
    if (cellVolt[j] > IgnoreCell && cellVolt[j] < 5.0)
    {
      moduleVolt = moduleVolt + cellVolt[j];
    }
  }
  return moduleVolt;
}

float BMSModule::getTemperature(int temp)
{
  if (temp < 0 || temp > 2) return 0.0f;
  return temperatures[temp];
}

void BMSModule::setAddress(int newAddr)
{
  if (newAddr < 0 || newAddr > MAX_MODULE_ADDR) return;
  moduleAddress = newAddr;
}

int BMSModule::getAddress()
{
  return moduleAddress;
}

int BMSModule::getType()
{
  return type;
}

int BMSModule::getBalStat()
{
  return balstat;
}

bool BMSModule::isExisting()
{
  return exists;
}

bool BMSModule::isReset()
{
  return reset;
}

void BMSModule::settempsensor(int tempsensor)
{
  sensor = tempsensor;
}

void IRAM_ATTR BMSModule::setExists(bool ex)
{
  exists = ex;
}

void BMSModule::setDelta(float ex)
{
  VoltDelta = ex;
}

void IRAM_ATTR BMSModule::setReset(bool ex)
{
  reset = ex;
}

void BMSModule::setIgnoreCell(float Ignore)
{
  IgnoreCell = Ignore;
  TelnetStream.println();
  TelnetStream.println();
  TelnetStream.println(Ignore);
  TelnetStream.println();

}
