/*
  Arduino-MCP346X
  Dan Potts
  Main header

  Initial version 0.1 - 15/02/2022
*/

/*
  todo:

  multiple constructors for spi settings
  check status in begin()
  include register masks
  make sure user can't use mux settings that leave anlog inputs floating
  some identifiers need some work (CH_I_1)
*/

#ifndef MCP346X_H
#define MCP346X_H

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

// Library options
#define MCP346X_SPI_SPEED 2000000

// ADC Command byte struture:
//  CMD[7]    CMD[6]    |   CMD[5]      CMD[4]      CMD[3]      CMD[2]    |    CMD[1]      CMD[0]
//  Device address bits |   Register Address / Fast command bits          |    Command type bits


class MCP346X {
public:

  // Device registers
  static const uint8_t REG_ADCDATA = 0x0;         // 4/8/16-bits read only
  static const uint8_t REG_CONFIG0 = 0x1;         // 8-bits R/W
  static const uint8_t REG_CONFIG1 = 0x2;         // 8-bits R/W
  static const uint8_t REG_CONFIG2 = 0x3;         // 8-bits R/W
  static const uint8_t REG_CONFIG3 = 0x4;         // 8-bits R/W
  static const uint8_t REG_IRQ = 0x5;             // 8-bits R/W
  static const uint8_t REG_MUX = 0x6;             // 8-bits R/W
  static const uint8_t REG_SCAN = 0x7;            // 24-bits R/W
  static const uint8_t REG_TIMER = 0x8;           // 24-bits R/W
  static const uint8_t REG_OFFSETCAL = 0x9;       // 24-bits R/W
  static const uint8_t REG_GAINCAL = 0xA;         // 24-bits R/W
  static const uint8_t REG_LOCK = 0xD;            // 8-bits R/W
  static const uint8_t REG_CCRCCFG = 0xF;         // 16-bits R

  // Fast command types
  static const uint8_t CMD_ADC_CONV_START = 0xA;
  static const uint8_t CMD_ADC_STANDBY = 0xB;
  static const uint8_t CMD_ADC_SHUTDOWN = 0xC;
  static const uint8_t CMD_FULL_SHUTDOWN = 0xD;
  static const uint8_t CMD_ADC_RESET = 0xE;

  // Command modes
  static const uint8_t CMD_MODE_FAST = 0x0;
  static const uint8_t CMD_MODE_STATIC_READ = 0x1;
  static const uint8_t CMD_MODE_INC_WRITE = 0x2;
  static const uint8_t CMD_MODE_INC_READ = 0x3;

  // Device address
  static const uint8_t ADDR_MCP3461 = 0x1;

  // Config0
  // Clock selection
  static const uint8_t CLK_EXTERNAL = 0x0;        // default
  static const uint8_t CLK_INTERNAL = 0x2;        // internal RC oscillator, no output
  static const uint8_t CLK_INTERNAL_OUT = 0x3;    // internal RC oscillator, output on MCLK pin

  // Current source/sink
  static const uint8_t CS_NONE = 0x0;             // default
  static const uint8_t CS_900NA = 0x1;            // 0.9uA
  static const uint8_t CS_3700NA = 0x2;           // 3.7uA
  static const uint8_t CS_15UA = 0x3;             // 15uA

  // ADC modes
  static const uint8_t ADC_MODE_SHUTDOWN = 0x0;
  static const uint8_t ADC_MODE_STANDBY = 0x2;
  static const uint8_t ADC_MODE_CONV = 0x4;

  // Config1
  // AMCLK Prescaler
  static const uint8_t MCLK_1 = 0x0;              // default
  static const uint8_t MCLK_2 = 0x1;
  static const uint8_t MCLK_4 = 0x2;
  static const uint8_t MCLK_8 = 0x3;

  // Overslampling ratio
  static const uint8_t OSR_32 = 0x0;
  static const uint8_t OSR_64 = 0x1;
  static const uint8_t OSR_128 = 0x2;
  static const uint8_t OSR_256 = 0x3;             // default
  static const uint8_t OSR_512 = 0x4;
  static const uint8_t OSR_1024 = 0x5;
  static const uint8_t OSR_2048 = 0x6;
  static const uint8_t OSR_4096 = 0x7;
  static const uint8_t OSR_8192 = 0x8;
  static const uint8_t OSR_16384 = 0x9;
  static const uint8_t OSR_20480 = 0xA;
  static const uint8_t OSR_24576 = 0xB;
  static const uint8_t OSR_40960 = 0xC;
  static const uint8_t OSR_49152 = 0xD;
  static const uint8_t OSR_81920 = 0xE;
  static const uint8_t OSR_98304 = 0xF;

  // Config 2
  // Channel bias current
  static const uint8_t CH_I_0X5 = 0x0;
  static const uint8_t CH_I_0X66 = 0x1;
  static const uint8_t CH_I_1 = 0x2;              // default
  static const uint8_t CH_I_2 = 0x3;

  // Gain
  static const uint8_t GAIN_0X33 = 0x0;
  static const uint8_t GAIN_1 = 0x1;              // default
  static const uint8_t GAIN_2 = 0x2;
  static const uint8_t GAIN_4 = 0x3;
  static const uint8_t GAIN_8 = 0x4;
  static const uint8_t GAIN_16 = 0x5;
  static const uint8_t GAIN_32 = 0x6;             // 16x analog, x2 digital
  static const uint8_t GAIN_64 = 0x7;             // 16x analog, x4 digital

  // Auto-Zeroing mux setting
  static const uint8_t AZ_MUX_OFF = 0x0;          // default
  static const uint8_t AZ_MUX_ON = 0x1;

  // Config3
  // Conversion mode
  static const uint8_t CONV_CONTINUOUS = 0x3;
  static const uint8_t CONV_ONESHOT_STANDBY = 0x2;
  static const uint8_t CONV_ONESHOT_SHUTDOWN = 0x0;// default

  // Data format
  static const uint8_t DATA_FORM_32B_SID = 0x3;   // 32-bits /w sign & channel ID
  static const uint8_t DATA_FORM_32B_S = 0x2;     // 32-bits /w sign
  static const uint8_t DATA_FORM_32B = 0x1;       // 32-bits
  static const uint8_t DATA_FORM_16B = 0x0;       // 16-bits, default

  // CRC format
  static const uint8_t CRC_FORM_32B = 0x1;
  static const uint8_t CRC_FORM_16B = 0x0;        // default

  // CRC Checksum on communication
  static const uint8_t CRC_COM_ON = 0x1;
  static const uint8_t CRC_COM_OFF = 0x0;         // default

  // Offset calibration
  static const uint8_t OFFCAL_ON = 0x1;
  static const uint8_t OFFCAL_OFF = 0x0;          // default

  // Gain calibration
  static const uint8_t GAINCAL_ON = 0x1;
  static const uint8_t GAINCAL_OFF = 0x0;         // default

  // IRQ
  // IRQ Mode
  static const uint8_t IRQ_PIN_MDAT = 0x1;
  static const uint8_t IRQ_PIN_IRQ = 0x0;         // IRQ pin used to interrupts, default
  static const uint8_t IRQ_PINMODE_PP = 0x1;      // push-pull output
  static const uint8_t IRQ_PINMODE_HIZ = 0x0;     // open colletor, default

  // Enable fast commands
  static const uint8_t FASTCMD_ON = 0x1;          // default
  static const uint8_t FASTCMD_OFF = 0x0;

  // Enable conversion start interrupt output (wat)
  static const uint8_t EN_STP_ON = 0x1;           // default
  static const uint8_t EN_STP_OFF = 0x0;

  // MUX
  static const uint8_t MUX_VCM = 0xF;
  static const uint8_t MUX_INT_DIODE_M = 0xE;
  static const uint8_t MUX_INT_DIODE_P = 0xD;
  static const uint8_t MUX_REFIN_N = 0xC;
  static const uint8_t MUX_REFIN_P = 0xB;
  static const uint8_t MUX_RESERVED = 0xA;        // do not use!
  static const uint8_t MUX_AVDD = 0x9;
  static const uint8_t MUX_AGND = 0x8;
  static const uint8_t MUX_CH7 = 0x7;
  static const uint8_t MUX_CH6 = 0x6;
  static const uint8_t MUX_CH5 = 0x5;
  static const uint8_t MUX_CH4 = 0x4;
  static const uint8_t MUX_CH3 = 0x3;
  static const uint8_t MUX_CH2 = 0x2;
  static const uint8_t MUX_CH1 = 0x1;             // defalt for Vin-
  static const uint8_t MUX_CH0 = 0x0;             // default for Vin+

  // SCAN
  // TIMER
  // LOCK

  MCP346X(uint8_t pinCS);
  ~MCP346X();
  bool begin();
  uint8_t getStatusByte();

  uint8_t readRegister(uint8_t reg);
  uint16_t readRegister16(uint8_t reg);

  // Normal register writing
  void setConfig0(uint8_t adcMode, uint8_t clkMode, uint8_t currentSource);
  void setConfig1(uint8_t prescaler, uint8_t osr);
  void setConfig2(uint8_t biasCurrent, uint8_t gain, uint8_t autoZeroMux);
  void setConfig3(uint8_t convMode, uint8_t dataFormat, uint8_t crcFormat, uint8_t crcCom, uint8_t enOffCal, uint8_t enGainCal);
  void setIRQ(uint8_t irqMode, uint8_t irqPinMode, uint8_t enFastCmd, uint8_t enSTP);
  void setMux(uint8_t inPositive, uint8_t inNegetive);

  // Fast command mode
  void _fastCommand(uint8_t command);

private:

  uint8_t _pinCS;
  SPISettings _spiSettings = SPISettings(MCP346X_SPI_SPEED, MSBFIRST, SPI_MODE0);

  // ADC status, retrived whenever a normal (not fast) transfer is initiated
  uint8_t _status = 0;

  // Standard (not 'fast') command write and register read/write functions
  // Not using the incremental register feature (seperate transactions)
  uint8_t _transfer(uint8_t reg, uint8_t mode, uint8_t data = 0xFF);
  uint16_t _transfer16(uint8_t reg, uint8_t mode, uint16_t data = 0xFFFF);
};

#endif
