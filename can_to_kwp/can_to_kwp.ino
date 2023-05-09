// This arduino code is converting CAN PID queries to KWPFast quries with the the help of K-Line and CAN interface ICs.
// (c) Adam Varga - https://github.com/adamtheone/CAN-to-KWPFast
// Tested on and developer for the Arduino Nano Every! Schematics and more details in the github repo.
// Dependencies:
// - 'CAN' Arduino library (CANReceiverCallback.ino) - Sandeep Mistry
// - 'OBD9141' Arduino library (readDTC.ino)
// Includes -------------------------------------------------------------------
#include "CAN.h"
#include "OBD9141.h"
// Defines --------------------------------------------------------------------
#define KWP_RX_PIN 0
#define KWP_TX_PIN 1
#define KWP_EN_PIN 21

#define CAN_SPEED 500E3
#define MCP2515_CS_PIN 10
#define MCP2515_IRQ_PIN 2
// Global variables -----------------------------------------------------------
OBD9141 kwp;
bool kwpInitSuccess = false;
uint8_t kwpRetries = 0;

// Functions ------------------------------------------------------------------
//  KWP
void (*resetFunc)(void) = 0; // for soft-restart

uint8_t KWPRequest(uint8_t pid) {
  uint8_t message[5] = { 0x68, 0x6A, 0xF1, 0x01, pid }; // KWPFast query bytes
  return kwp.request(&message, 5);
}
//  CAN
int getCANRequestedPid(void) {
  if (CAN.packetExtended() || CAN.packetId() != 0x7DF) {
    if (CAN.packetExtended()) {
      Serial.print("Unexpected extended packet with id 0x");
    } else {
      Serial.print("Unexpected normal packet with id 0x");
    }
    Serial.println(CAN.packetId(), HEX);

    return -1;  // don't want to support extended packets for now
  }

  uint8_t payload[8];
  uint8_t packetLen = 0;

  while (CAN.available()) {
    payload[packetLen++] = CAN.read();
  }

  return packetLen > 2 ? payload[2] : -1;
}

void sendCanRespDummy(uint8_t pid) {
  Serial.print("Sending dummy: ");
  Serial.println(pid, HEX);
  CAN.beginPacket(0x7E8, 8);
  CAN.write(0x07);  // number of additional bytes
  uint8_t padding = 8;
  switch (pid) {
    case 0x4:  // LOAD
      //CAN.write(0x03); // number of additional bytes
      padding - 4;
      CAN.write(0x41);  // show current data
      CAN.write(0x4);   // PID
      CAN.write(127);   // 50%
      break;
    case 0x5:  // COOLANT
      //CAN.write(0x03); // number of additional bytes
      padding - 4;
      CAN.write(0x41);  // show current data
      CAN.write(0x5);   // PID
      CAN.write(90);    // 50C
      break;
    case 0xB:  // MANIFOLD
      //CAN.write(0x03); // number of additional bytes
      padding - 4;
      CAN.write(0x41);  // show current data
      CAN.write(0xB);   // PID
      CAN.write(111);   // 111kPA
      break;
    case 0xC:  // RPM
      //CAN.write(0x04); // number of additional bytes
      padding - 5;
      CAN.write(0x41);  // show current data
      CAN.write(0xC);   // PID
      CAN.write(19);    // 1216
      CAN.write(0);
      break;
    case 0xD:  // SPEED
      //CAN.write(0x03); // number of additional bytes
      padding - 4;
      CAN.write(0x41);  // show current data
      CAN.write(0xD);   // PID
      CAN.write(110);   // 110
      break;
    case 0x10:
      //CAN.write(0x03); // number of additional bytes
      padding - 4;
      CAN.write(0x41);  // show current data
      CAN.write(0xD);   // PID
      CAN.write(110);   // 110
      break;
    case 0x2f:  // fuel
      //CAN.write(0x03); // number of additional bytes
      padding - 4;
      CAN.write(0x41);  // show current data
      CAN.write(0x2f);  // PID
      CAN.write(25);    // 10%
      break;
    case 0x42:  // voltage
      //CAN.write(0x04); // number of additional bytes
      padding - 5;
      CAN.write(0x41);  // show current data
      CAN.write(0x42);  // PID
      CAN.write(47);    // 12v
      CAN.write(0);
      break;
    case 0x5C:  // OIL
      //CAN.write(0x03); // number of additional bytes
      padding - 4;
      CAN.write(0x41);  // show current data
      CAN.write(0x5C);  // PID
      CAN.write(110);   // 70C
      break;
    case 0x0F:  // Air temp
      //CAN.write(0x03); // number of additional bytes
      padding - 4;
      CAN.write(0x41);  // show current data
      CAN.write(0x0F);  // PID
      CAN.write(60);    // 20C
      break;
    case 0x11:  // throttle pos
      //CAN.write(0x03); // number of additional bytes
      padding - 4;
      CAN.write(0x41);  // show current data
      CAN.write(0x11);  // PID
      CAN.write(178);   // 70%
      break;
    case 0x0E:  //timing
      //CAN.write(0x03); // number of additional bytes
      padding - 4;
      CAN.write(0x41);  // show current data
      CAN.write(0x0E);  // PID
      CAN.write(64);    // -32
      break;
    default:
      Serial.print("Not handled packet!! ");
      Serial.println(pid, HEX);
      return;
  }
  for (uint8_t i = 0; i < padding; i++) {
    CAN.write(0x00);
  }
  CAN.endPacket();
}

void onCanReceive(int packetSize) {
  if (!kwpInitSuccess) {
    int pid = getCANRequestedPid();
    if (pid > 0) {
      sendCanRespDummy(pid);
    }
  }
}

void sendCanResp(uint8_t pid, uint8_t dataLen) {
  CAN.beginPacket(0x7E8, 8);
  CAN.write(dataLen+2);
  CAN.write(0x41);
  CAN.write(pid);
  Serial.print("Useful data: ");
  for (uint8_t i = 0; i < 5; i++) {
    if (i <= dataLen) {
      CAN.write(kwp.readUint8(i));
      Serial.print(kwp.readUint8(i), HEX);
      Serial.print(" ");
    } else {
      CAN.write(0x00);
    }
  }
  Serial.println();
  CAN.endPacket();
}

// Setup ----------------------------------------------------------------------
void setup() {
  // Serial setup
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("CAN to KWPFast started!");

  // CAN setup
  CAN.setPins(MCP2515_CS_PIN, MCP2515_IRQ_PIN);
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    delay(1500);
    resetFunc();
  }
  //-----------------------------------------------------
  // We specifically want to answer to the CAN master
  //  only if KWP is initialized successfully.
  //  Uncommenting the following line activates the dummy
  //  responses even before KWP is initialized.
  //-----------------------------------------------------
  // CAN.onReceive(onCanReceive);

  // KWPFast setup
  pinMode(KWP_EN_PIN, OUTPUT);
  digitalWrite(KWP_EN_PIN, HIGH);
  
  kwp.begin(Serial1, KWP_RX_PIN, KWP_TX_PIN);
}

void loop() {
  if (!kwpInitSuccess) {
    kwpInitSuccess = kwp.initKWP();
    if (kwpInitSuccess) {
      Serial.println("KWP Init successful!");
      CAN.onReceive(NULL);
      kwpRetries = 0;
    } else {
      Serial.println("KWP Init unsuccessful!");
    }
  } else {
    int packetSize = CAN.parsePacket();
    if (packetSize) {
      int pid = getCANRequestedPid();
      if (pid > 0) {
        Serial.print("can pid: 0x"); Serial.println(pid, HEX);
        uint8_t rxLen = KWPRequest((uint8_t)pid);
        if (rxLen) {
          sendCanResp(pid, rxLen-5);
        } else {
          Serial.println("No KWP response!");
          kwpRetries++;
          if (kwpRetries > 5) {
            kwpInitSuccess = false;
          }
          //sendCanRespDummy(pid);
        }
      }
    }
  }
}
