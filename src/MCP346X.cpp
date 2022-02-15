/*
  Arduino-MCP346X
  Dan Potts
  Main source

  Initial version 0.1 - 15/02/2022
*/

#include "MCP346X.h"

// Constructor
MCP346X::MCP346X(uint8_t pinCS)
{
  _pinCS = pinCS;
}

MCP346X::~MCP346X(){};

// Public
bool MCP346X::begin()
{
  //Serial.print("Pin = ");
  //Serial.println(_pinCS);
  pinMode(_pinCS, OUTPUT);
  digitalWrite(_pinCS, HIGH);
  SPI.begin();

  // Reset device registers
  _fastCommand(CMD_ADC_RESET);

  // todo: test adc by reading back status/device addr reg
  return true;
};

uint8_t MCP346X::readRegister(uint8_t reg)
{
  digitalWrite(_pinCS, LOW);
  uint8_t output = _transfer(reg, CMD_MODE_STATIC_READ, 0xFF);
  digitalWrite(_pinCS, HIGH);
  return output;
};

uint16_t MCP346X::readRegister16(uint8_t reg)
{
  digitalWrite(_pinCS, LOW);
  uint16_t output = _transfer16(reg, CMD_MODE_STATIC_READ, 0xFFFF);
  digitalWrite(_pinCS, HIGH);
  return output;
};

void MCP346X::setConfig0(uint8_t adcMode, uint8_t clkMode, uint8_t currentSource)
{
  if (adcMode == ADC_MODE_SHUTDOWN) {
    // Write zero to all bits
    digitalWrite(_pinCS, LOW);
    _transfer(REG_CONFIG0, CMD_MODE_INC_WRITE, 0);
    digitalWrite(_pinCS, HIGH);
    return;
  }

  uint8_t data = 0;
  data |= clkMode << 4;
  data |= currentSource << 2;
  data |= adcMode;

  digitalWrite(_pinCS, LOW);
  _transfer(REG_CONFIG0, CMD_MODE_INC_WRITE, data);
  digitalWrite(_pinCS, HIGH);
};

void MCP346X::setConfig1(uint8_t prescaler, uint8_t osr)
{
  uint8_t data = 0;
  data |= prescaler << 6;
  data |= osr << 2;
  data &= 0xFC;

  digitalWrite(_pinCS, LOW);
  _transfer(REG_CONFIG1, CMD_MODE_INC_WRITE, data);
  digitalWrite(_pinCS, HIGH);
};

void MCP346X::setConfig2(uint8_t biasCurrent, uint8_t gain, uint8_t autoZeroMux)
{
  uint8_t data = 0;
  data |= biasCurrent << 6;
  data |= gain << 3;
  data |= autoZeroMux << 2;
  data |= 0x3;  // last 2 bits are reserved and should be 1

  digitalWrite(_pinCS, LOW);
  _transfer(REG_CONFIG2, CMD_MODE_INC_WRITE, data);
  digitalWrite(_pinCS, HIGH);
};

void MCP346X::setConfig3(uint8_t convMode, uint8_t dataFormat, uint8_t crcFormat, uint8_t crcCom, uint8_t enOffCal, uint8_t enGainCal)
{
  uint8_t data = 0;
  data |= convMode << 6;
  data |= dataFormat << 4;
  data |= crcFormat << 3;
  data |= crcCom << 2;
  data |= enOffCal << 1;
  data |= enGainCal;

  digitalWrite(_pinCS, LOW);
  _transfer(REG_CONFIG3, CMD_MODE_INC_WRITE, data);
  digitalWrite(_pinCS, HIGH);
};

void MCP346X::setIRQ(uint8_t irqMode, uint8_t irqPinMode, uint8_t enFastCmd, uint8_t enSTP)
{
  uint8_t data = 0;
  data |= irqMode << 3;
  data |= irqPinMode << 2;
  data |= enFastCmd << 1;
  data |= enSTP;

  digitalWrite(_pinCS, LOW);
  _transfer(REG_IRQ, CMD_MODE_INC_WRITE, data);
  digitalWrite(_pinCS, HIGH);
};

void MCP346X::setMux(uint8_t inPositive, uint8_t inNegetive)
{
  uint8_t data = 0;
  data |= inPositive << 4;
  data |= inNegetive;

  digitalWrite(_pinCS, LOW);
  _transfer(REG_MUX, CMD_MODE_INC_WRITE, data);
  digitalWrite(_pinCS, HIGH);
};

// Private
uint8_t MCP346X::_transfer(uint8_t reg, uint8_t mode, uint8_t data)
{
  uint8_t commandByte = 0;
  commandByte |= ADDR_MCP3461 << 6;
  commandByte |= reg << 2;
  commandByte |= mode;

  SPI.beginTransaction(_spiSettings);
  _status = SPI.transfer(commandByte);
  uint8_t returnVal = SPI.transfer(data);
  SPI.endTransaction();

  return returnVal;
};

uint16_t MCP346X::_transfer16(uint8_t reg, uint8_t mode, uint16_t data)
{
  uint8_t commandByte = 0;
  commandByte |= ADDR_MCP3461 << 6;
  commandByte |= reg << 2;
  commandByte |= mode;

  SPI.beginTransaction(_spiSettings);
  _status = SPI.transfer(commandByte);
  uint16_t returnVal = SPI.transfer16(data);
  SPI.endTransaction();

  return returnVal;
};

void MCP346X::_fastCommand(uint8_t command)
{
  uint8_t commandByte = 0;
  commandByte |= (ADDR_MCP3461 & 0x3) << 6;
  commandByte |= (command & 0xF) << 2;
  commandByte |= (CMD_MODE_FAST & 0x3);

  digitalWrite(_pinCS, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer(commandByte);
  SPI.endTransaction();
  digitalWrite(_pinCS, HIGH);
};
