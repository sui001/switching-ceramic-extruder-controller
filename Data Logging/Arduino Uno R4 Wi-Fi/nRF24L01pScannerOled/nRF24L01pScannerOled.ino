// Rough and ready 2.4 GHz band scanner using nRF24L01+ module with 128x64 graphic OLED display
//
// ceptimus.  November 2016.

// modified by Xinbo Qu, Project AAA. Feb 2024. All rights Reserved.
// modify log: get average signal intensity value for both real nRF24L01 data and the area persentage on the SSD1306 OLED screen.
// modify PORTB reg operation into specific pin operation to adapt new Uno R4. 
// modify _NOP() into __asm__ __volatile__("nop") to achieve same effect of delay for a very short time period
// add threshold and stepper switch control by digital output

// modified for SPI SD card reader. use same SPI pins, with different Chip Select pins.
// For SPI devices, Arduino can communicated with them when the corresponding Chip Select pin is low, which means the default will be high.
// Procedure will be: CS->low, communicate, CS->high 

// update: the SD lib will occupy MISO pin, therefore conflict with default nRF24L01 module
// Fortunately, the given code of the nRF24L01 module uses the software SPI instead of the hardware SPI, 
// and bit-banging can be used to assign all SPI pins directly to any digital pins without relying on the hardware SPI 

#include "SSD1X06.h"

#include <SPI.h>
#include <SD.h>

/* nRF24L01+ module connections
 *  
 * Module   Arduino Uno R4 Wi-Fi
 * 1 GND ---- GND
 * 2 VCC ---- 3.3V  Note: 5V on VCC will destroy module (but other pins are 5V tolerant)
 * 3 CE ----- D9
 * 4 CSN ---- D10
 * 5 SCK ---- D2 (BB_SCK)
 * 6 MOSI --- D3 (BB_MOSI)
 * 7 MISO --- D4 (BB_MISO)
 * 8 IRQ ---- not connected
 */

/*
 * SSD1306 Connection
 * 
 * Module   Arduino Uno R4 Wi-Fi
 * 1 GND ---- GND
 * 2 VCC ---- 5V/3.3V
 * 3 SCL ---- A4
 * 4 SDA ---- A5
 */

/*
 * Arduino Uno R3 Connection
 * 
 * Uno R3   Arduino Uno R4 Wi-Fi
 * 1 GND ---- GND   Note: connect GND can prevent from ground offset
 * 2 D5  ---- D5
 * 3 D6  ---- D6
 * 4 D7  ---- D7
 */

/* SD Card Reader module connections
 *  
 * Module   Arduino Uno R4 Wi-Fi
 * 1 GND ---- GND
 * 2 VCC ---- 5V
 * 3 CS ----- D8  (SS)
 * 4 SCK ---- D13 (SCK)
 * 5 MOSI --- D11 (MOSI)
 * 6 MISO --- D12 (MISO)
 */

// param
#define DEBUG_SERIAL_OUTPUT true
#define THRESHOLD_1 0.25
#define THRESHOLD_2 0.5
#define en_PUL1 5       // 0            <= Average Value < THRESHOLD_1
#define en_PUL2 6       // THRESHOLD_1  <= Average Value < THRESHOLD_2
#define en_PUL3 7       // THRESHOLD_2  <= Average Value <= 1


// the nRF24L01+ can tune to 128 channels with 1 MHz spacing from 2.400 GHz to 2.527 GHz.
#define CHANNELS 128

// SPI definitions and macros
#define CE_pin 9
#define CS_pin 10
#define MOSI_pin 11
#define MISO_pin 12
#define SCK_pin 13

#define BB_MISO_pin 4
#define BB_MOSI_pin 3
#define BB_SCK_pin 2

#define SD_Reader_CS_pin 8

// SPI operation macro
#define CE_on digitalWrite(CE_pin, HIGH)
#define CE_off digitalWrite(CE_pin, LOW)
#define CS_on digitalWrite(CS_pin, HIGH)
#define CS_off digitalWrite(CS_pin, LOW)
#define MOSI_on digitalWrite(MOSI_pin, HIGH)
#define MOSI_off digitalWrite(MOSI_pin, LOW)
#define MISO_on (digitalRead(MISO_pin) == HIGH)  // input
#define SCK_on digitalWrite(SCK_pin, HIGH)
#define SCK_off digitalWrite(SCK_pin, LOW)

#define SD_Reader_CS_on digitalWrite(SD_Reader_CS_pin, HIGH)
#define SD_Reader_CS_off digitalWrite(SD_Reader_CS_pin, LOW)

#define BB_MISO_on (digitalRead(BB_MISO_pin) == HIGH)
#define BB_MOSI_on digitalWrite(BB_MOSI_pin, HIGH)
#define BB_MOSI_off digitalWrite(BB_MOSI_pin, LOW)
#define BB_SCK_on digitalWrite(BB_SCK_pin, HIGH)
#define BB_SCK_off digitalWrite(BB_SCK_pin, LOW)


// nRF24 Register map
enum {
  NRF24L01_00_CONFIG = 0x00,
  NRF24L01_01_EN_AA = 0x01,
  NRF24L01_02_EN_RXADDR = 0x02,
  NRF24L01_03_SETUP_AW = 0x03,
  NRF24L01_04_SETUP_RETR = 0x04,
  NRF24L01_05_RF_CH = 0x05,
  NRF24L01_06_RF_SETUP = 0x06,
  NRF24L01_07_STATUS = 0x07,
  NRF24L01_08_OBSERVE_TX = 0x08,
  NRF24L01_09_CD = 0x09,
  NRF24L01_0A_RX_ADDR_P0 = 0x0A,
  NRF24L01_0B_RX_ADDR_P1 = 0x0B,
  NRF24L01_0C_RX_ADDR_P2 = 0x0C,
  NRF24L01_0D_RX_ADDR_P3 = 0x0D,
  NRF24L01_0E_RX_ADDR_P4 = 0x0E,
  NRF24L01_0F_RX_ADDR_P5 = 0x0F,
  NRF24L01_10_TX_ADDR = 0x10,
  NRF24L01_11_RX_PW_P0 = 0x11,
  NRF24L01_12_RX_PW_P1 = 0x12,
  NRF24L01_13_RX_PW_P2 = 0x13,
  NRF24L01_14_RX_PW_P3 = 0x14,
  NRF24L01_15_RX_PW_P4 = 0x15,
  NRF24L01_16_RX_PW_P5 = 0x16,
  NRF24L01_17_FIFO_STATUS = 0x17,
  NRF24L01_1C_DYNPD = 0x1C,
  NRF24L01_1D_FEATURE = 0x1D,
  //Instructions
  NRF24L01_61_RX_PAYLOAD = 0x61,
  NRF24L01_A0_TX_PAYLOAD = 0xA0,
  NRF24L01_E1_FLUSH_TX = 0xE1,
  NRF24L01_E2_FLUSH_RX = 0xE2,
  NRF24L01_E3_REUSE_TX_PL = 0xE3,
  NRF24L01_50_ACTIVATE = 0x50,
  NRF24L01_60_R_RX_PL_WID = 0x60,
  NRF24L01_B0_TX_PYLD_NOACK = 0xB0,
  NRF24L01_FF_NOP = 0xFF,
  NRF24L01_A8_W_ACK_PAYLOAD0 = 0xA8,
  NRF24L01_A8_W_ACK_PAYLOAD1 = 0xA9,
  NRF24L01_A8_W_ACK_PAYLOAD2 = 0xAA,
  NRF24L01_A8_W_ACK_PAYLOAD3 = 0xAB,
  NRF24L01_A8_W_ACK_PAYLOAD4 = 0xAC,
  NRF24L01_A8_W_ACK_PAYLOAD5 = 0xAD,
};

// Bit mnemonics
enum {
  NRF24L01_00_MASK_RX_DR = 6,
  NRF24L01_00_MASK_TX_DS = 5,
  NRF24L01_00_MASK_MAX_RT = 4,
  NRF24L01_00_EN_CRC = 3,
  NRF24L01_00_CRCO = 2,
  NRF24L01_00_PWR_UP = 1,
  NRF24L01_00_PRIM_RX = 0,

  NRF24L01_07_RX_DR = 6,
  NRF24L01_07_TX_DS = 5,
  NRF24L01_07_MAX_RT = 4,

  NRF2401_1D_EN_DYN_ACK = 0,
  NRF2401_1D_EN_ACK_PAY = 1,
  NRF2401_1D_EN_DPL = 2,
};

enum TXRX_State {
  TXRX_OFF,
  TX_EN,
  RX_EN,
};

// global var

uint16_t signalStrength[CHANNELS];  // smooths signal strength with numerical range 0 - 0x7FFF

const int scansPerPeriod = 30;
uint16_t scanResults[scansPerPeriod][CHANNELS];
unsigned long scanTimes[scansPerPeriod];
int scanIndex = 0; // from 0 to scansPerPeriod - 1
unsigned long lastScanTime = 0;
bool mem_record_flag = false;


void setup() {
  //init serial debug if needed
  if (DEBUG_SERIAL_OUTPUT) {
    Serial.begin(9600);
  }

  //init oled display
  SSD1X06::start();
  delay(300);
  SSD1X06::fillDisplay(' ');
  SSD1X06::displayString6x8(1, 4, F("2.4 GHz band scanner"), 0);
  SSD1X06::displayString6x8(4, 4, F("By ceptimus. Nov '16"), 0);

  // prepare 'bit banging' SPI interface
  pinMode(MOSI_pin, OUTPUT);
  pinMode(SCK_pin, OUTPUT);
  pinMode(CS_pin, OUTPUT);
  pinMode(CE_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);

  pinMode(BB_MISO_pin, INPUT);
  pinMode(BB_MOSI_pin, OUTPUT);
  pinMode(BB_SCK_pin, OUTPUT);

  CS_on;
  CE_on;
  BB_MOSI_on;
  BB_SCK_on;
  delay(70);
  CS_off;
  CE_off;
  BB_MOSI_off;
  BB_SCK_off;
  delay(100);
  CS_on;
  delay(10);

  NRF24L01_Reset();
  delay(150);

  NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);     // switch off Shockburst mode
  NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x0F);  // write default value to setup register
  NRF24L01_SetTxRxMode(RX_EN);                    // switch to receive mode


  // init sd card reader
  pinMode(SD_Reader_CS_pin, OUTPUT);
  // SD_Reader_CS_on;
  // SD_Reader_CS_off;

  if (!SD.begin(SD_Reader_CS_pin)) {
    if (DEBUG_SERIAL_OUTPUT) {
      Serial.println("SD card initialization failed!");
    }
  }else {
    if (DEBUG_SERIAL_OUTPUT) {
      Serial.println("SD card initialized.");
    }
  }
  

  // init pulse output
  pinMode(en_PUL1, OUTPUT);
  digitalWrite(en_PUL1, LOW);
  pinMode(en_PUL2, OUTPUT);
  digitalWrite(en_PUL2, LOW);
  pinMode(en_PUL3, OUTPUT);
  digitalWrite(en_PUL3, LOW);

  // init default display content
  for (int x = 0; x < 128; x++) {
    uint8_t b = 0x01;  // baseline
    if (!(x % 10)) {
      b |= 0x06;  // graduation tick every 10 MHz
    }
    if (x == 10 || x == 60 || x == 110) {
      b |= 0xF8;  // scale markers at 2.41, 2.46, and 2.51 GHz
    }
    SSD1X06::displayByte(6, x, b);
  }
  SSD1X06::displayString6x8(7, 0, F("2.41"), 0);
  SSD1X06::displayString6x8(7, 50, F("2.46"), 0);
  SSD1X06::displayString6x8(7, 100, F("2.51"), 0);
  delay(1500);  // start up message
}

uint8_t refresh; //refresh counter for oled display

void loop() {

  uint32_t sumDisplayedStrength = 0;
  if(millis() - lastScanTime >= 1000) {
    mem_record_flag = true;
    lastScanTime = millis();
    scanTimes[scanIndex] = lastScanTime; 
  }

  for (uint8_t MHz = 0; MHz < CHANNELS; MHz++) {  // tune to frequency (2400 + MHz) so this loop covers 2.400 - 2.527 GHz (maximum range module can handle) when channels is set to 128.
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, MHz);
    CE_on;                                                         // start receiving
    delayMicroseconds(random(130, 230));                           // allow receiver time to tune and start receiving 130 uS seems to be the minimum time.  Random additional delay helps prevent strobing effects with frequency-hopping transmitters.
    CE_off;                                                        // stop receiving - one bit is now set if received power was > -64 dBm at that instant
    if (NRF24L01_ReadReg(NRF24L01_09_CD)) {                        // signal detected so increase signalStrength unless already maxed out
      signalStrength[MHz] += (0x7FFF - signalStrength[MHz]) >> 5;  // increase rapidly when previous value was low, with increase reducing exponentially as value approaches maximum
    } else {                                                       // no signal detected so reduce signalStrength unless already at minimum
      signalStrength[MHz] -= signalStrength[MHz] >> 5;             // decrease rapidly when previous value was high, with decrease reducing exponentially as value approaches zero
    }
    // Serial.print((signalStrength[MHz] + 0x0100) >> 9, HEX); // debugging without lcd display
    // Serial.print(" "); // debugging without lcd display

    if (!--refresh) {  // don't refresh whole display every scan (too slow)
      refresh = 19;    // speed up by only refreshing every n-th frequency loop - reset number should be relatively prime to CHANNELS
      int strength = (signalStrength[MHz] + 0x0040) >> 7;
      if (strength > 48) {
        strength = 48;  // limit to maximum height that fits display
      }

      for (uint8_t row = 0; row < 6; row++) {  // loop down 6 rows of display (6 x 8 pixels)
        uint8_t b = 0x00;
        if (strength > (6 - row) << 3) {  // all 8 pixels on this row of display to be set
          b = 0xFF;
        } else if (strength > (5 - row) << 3) {  // some pixels on this row to be set
          b = 0xFF << (((6 - row) << 3) - strength);
        }
        SSD1X06::displayByte(row, MHz, b);
      }
    }

    // get display strength of each channel
    int tmpStrength = (signalStrength[MHz] + 0x0040) >> 7;
    if (tmpStrength > 48) {
      tmpStrength = 48;  // limit to maximum height that fits display
    }
    sumDisplayedStrength += tmpStrength;
    if (mem_record_flag) {
      scanResults[scanIndex][MHz] = signalStrength[MHz];
    }
  }

  // based on display area persentage
  float averageDisplayedStrength = sumDisplayedStrength / (float)(CHANNELS * 48);
  if (DEBUG_SERIAL_OUTPUT) {
    Serial.print("Average Displayed Signal Strength: ");
    Serial.println(averageDisplayedStrength, DEC);
  }
  if ((0 <= averageDisplayedStrength) && (averageDisplayedStrength < THRESHOLD_1)) {
    digitalWrite(en_PUL2, LOW);
    digitalWrite(en_PUL3, LOW);
    digitalWrite(en_PUL1, HIGH);
  } else if ((THRESHOLD_1 <= averageDisplayedStrength) && (averageDisplayedStrength < THRESHOLD_2)) {
    digitalWrite(en_PUL1, LOW);
    digitalWrite(en_PUL3, LOW);
    digitalWrite(en_PUL2, HIGH);
  } else if ((THRESHOLD_2 <= averageDisplayedStrength) && (averageDisplayedStrength <= 1)) {
    digitalWrite(en_PUL1, LOW);
    digitalWrite(en_PUL2, LOW);
    digitalWrite(en_PUL3, HIGH);
  }

  if (mem_record_flag) {
    scanIndex++;
    mem_record_flag = false;
  }

  if (scanIndex >= scansPerPeriod) { 
    writeToSDCard(lastScanTime); 
    scanIndex = 0; 
  }

}

uint8_t _spi_write(uint8_t command) {
  uint8_t result = 0;
  uint8_t n = 8;
  BB_SCK_off;
  BB_MOSI_off;
  while (n--) {
    if (command & 0x80)
      BB_MOSI_on;
    else
      BB_MOSI_off;
    if (BB_MISO_on)
      result |= 0x01;
    BB_SCK_on;
    __asm__ __volatile__("nop");
    BB_SCK_off;
    command = command << 1;
    result = result << 1;
  }
  BB_MOSI_on;
  return result;
}

void _spi_write_address(uint8_t address, uint8_t data) {
  CS_off;
  _spi_write(address);
  __asm__ __volatile__("nop");
  _spi_write(data);
  CS_on;
}

uint8_t _spi_read() {
  uint8_t result = 0;
  uint8_t i;
  BB_MOSI_off;
  __asm__ __volatile__("nop");
  for (i = 0; i < 8; i++) {
    if (BB_MISO_on)  // if BB_MISO is HIGH
      result = (result << 1) | 0x01;
    else
      result = result << 1;
    BB_SCK_on;
    __asm__ __volatile__("nop");
    BB_SCK_off;
    __asm__ __volatile__("nop");
  }
  return result;
}

uint8_t _spi_read_address(uint8_t address) {
  uint8_t result;
  CS_off;
  _spi_write(address);
  result = _spi_read();
  CS_on;
  return (result);
}

/* Instruction Mnemonics */
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE 0x50
#define R_RX_PL_WID 0x60
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF

uint8_t NRF24L01_WriteReg(uint8_t address, uint8_t data) {
  CS_off;
  _spi_write_address(address | W_REGISTER, data);
  CS_on;
  return 1;
}

uint8_t NRF24L01_FlushTx() {
  return Strobe(FLUSH_TX);
}

uint8_t NRF24L01_FlushRx() {
  return Strobe(FLUSH_RX);
}

static uint8_t Strobe(uint8_t state) {
  uint8_t result;
  CS_off;
  result = _spi_write(state);
  CS_on;
  return result;
}

uint8_t NRF24L01_ReadReg(uint8_t reg) {
  CS_off;
  uint8_t data = _spi_read_address(reg);
  CS_on;
  return data;
}

void NRF24L01_SetTxRxMode(uint8_t mode) {
  if (mode == TX_EN) {
    CE_off;
    NRF24L01_WriteReg(NRF24L01_07_STATUS,
                      (1 << NRF24L01_07_RX_DR)  // reset the flag(s)
                        | (1 << NRF24L01_07_TX_DS)
                        | (1 << NRF24L01_07_MAX_RT));
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                      (1 << NRF24L01_00_EN_CRC)  // switch to TX mode
                        | (1 << NRF24L01_00_CRCO)
                        | (1 << NRF24L01_00_PWR_UP));
    delayMicroseconds(130);
    CE_on;
  } else if (mode == RX_EN) {
    CE_off;
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);  // reset the flag(s)
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);  // switch to RX mode
    NRF24L01_WriteReg(NRF24L01_07_STATUS,
                      (1 << NRF24L01_07_RX_DR)  //reset the flag(s)
                        | (1 << NRF24L01_07_TX_DS)
                        | (1 << NRF24L01_07_MAX_RT));
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                      (1 << NRF24L01_00_EN_CRC)  // switch to RX mode
                        | (1 << NRF24L01_00_CRCO)
                        | (1 << NRF24L01_00_PWR_UP)
                        | (1 << NRF24L01_00_PRIM_RX));
    delayMicroseconds(130);
    CE_on;
  } else {
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC));  // PowerDown
    CE_off;
  }
}

uint8_t NRF24L01_Reset() {
  NRF24L01_FlushTx();
  NRF24L01_FlushRx();
  uint8_t status1 = Strobe(0xFF);  // NOP
  uint8_t status2 = NRF24L01_ReadReg(0x07);
  NRF24L01_SetTxRxMode(TXRX_OFF);
  return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}




void writeToSDCard(unsigned long totalMilSec) {
  // SD_Reader_CS_off;
  unsigned long totalSec = totalMilSec / 1000;
  int minutes = (totalSec / 60);
  int seconds = totalSec % 60;
  String fileName = String(minutes) + "m" + String(seconds) + "s.csv";
  File dataFile = SD.open(fileName, FILE_WRITE);

    if (dataFile) {
    dataFile.print("Time (ms)");
    for (int channel = 0; channel < CHANNELS; channel++) {
      dataFile.print(", Channel ");
      dataFile.print(channel);
    }
    dataFile.println(); 

    for (int i = 0; i < scansPerPeriod; i++) {
      dataFile.print(scanTimes[i]);

      for (int j = 0; j < CHANNELS; j++) {
        dataFile.print(", ");
        dataFile.print(scanResults[i][j]);
      }
      dataFile.println(); 
    }
    dataFile.close();
    if (DEBUG_SERIAL_OUTPUT) {
      Serial.println("Data saved.");
    }
  } else {
    if (DEBUG_SERIAL_OUTPUT) {
      Serial.println("Error opening file.");
    }
  }
  // SD_Reader_CS_on;
}
