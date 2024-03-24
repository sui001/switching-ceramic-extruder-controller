// read data from signal scanner, replay data with pulse control
//
// written by Xinbo Qu, Project AAA. March 2024. All rights Reserved.

// currently have some delay than original data

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

#include <SPI.h>
#include <SD.h>

#define CHANNELS 128
#define DEBUG_SERIAL_OUTPUT true
#define SD_Reader_CS_pin 8
#define THRESHOLD_1 0.25
#define THRESHOLD_2 0.5
#define en_PUL1 5       // 0            <= Average Value < THRESHOLD_1
#define en_PUL2 6       // THRESHOLD_1  <= Average Value < THRESHOLD_2
#define en_PUL3 7       // THRESHOLD_2  <= Average Value <= 1

const char *filename = "/REPLAY/data.csv";
uint16_t scanResults[CHANNELS];
unsigned long scanTimes = 0;

void setup() {
  Serial.begin(9600);

  // init pulse output
  pinMode(en_PUL1, OUTPUT);
  digitalWrite(en_PUL1, LOW);
  pinMode(en_PUL2, OUTPUT);
  digitalWrite(en_PUL2, LOW);
  pinMode(en_PUL3, OUTPUT);
  digitalWrite(en_PUL3, LOW);

  if (!SD.begin(SD_Reader_CS_pin)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  File dataFile = SD.open(filename);
  if (dataFile) {
    String line;
    // ignore first line
    dataFile.readStringUntil('\n');

    while(dataFile.available()) {
      line = dataFile.readStringUntil('\n');
      // get scanTimes
      int firstCommaIndex = line.indexOf(',');
      if (firstCommaIndex != -1) {
        scanTimes = line.substring(0, firstCommaIndex).toInt();
        // parse scanResults
        parseData(line.substring(firstCommaIndex + 1));
      }

      float averageDisplayedStrength = calculateAverageSignalStrength();

      // wait until scanTimes
      while (millis() <= scanTimes) {
        // nothing
      }

      // output average strength
      if (DEBUG_SERIAL_OUTPUT) {
        Serial.print("Average Displayed Signal Strength: ");
        Serial.println(averageDisplayedStrength, DEC);
      }
      // control pulse
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

    }
    dataFile.close();
  } else {
    Serial.println("File does not exist!");
  }
}

void loop() {
  // nothing
}

void parseData(String dataLine) {
  int index = 0;
  int from = 0;
  for (int to = 0; to <= dataLine.length() && index < CHANNELS; to++) {
    if (dataLine.charAt(to) == ',' || to == dataLine.length()) {
      scanResults[index++] = dataLine.substring(from, to).toInt();
      from = to + 1;
    }
  }
}

float calculateAverageSignalStrength() {
  unsigned long sum = 0;
  for (int i = 0; i < CHANNELS; i++) {
    int tmpStrength = (scanResults[i] + 0x0040) >> 7;
    if (tmpStrength > 48) {
      tmpStrength = 48;  // limit to maximum height that fits display
    }
    sum += tmpStrength;
  }
  return sum / (float)(CHANNELS * 48);
}
