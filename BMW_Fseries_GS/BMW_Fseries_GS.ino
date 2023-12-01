/*
 *
 * Copyright (C) 2023 Tom de Bree
 *                     \
 * All credits to the Orignal Reverse engineering work and documenation
 * Project Gus and a forum post from Bimmerwelt
 * Based on info from https://openinverter.org/wiki/BMW_F-Series_Gear_Lever 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



#include <mcp_can.h>
#include <SPI.h>
#include "CRC8.h"

CRC8 crc8;

long unsigned int rxId;
uint32_t relayid = 0;
uint8_t relayidb[4];
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];  // Array to store serial string

uint8_t Dir = 0;

#define Off 0x00
#define Park 0x20
#define Reverse 0x40
#define Neutral 0x60
#define Drive 0x80
#define Sport 0x81


uint8_t ParkState = 0;

#define UnParked 0x00
#define Parked 0x01

bool routine = 0;
unsigned long looptime = 0;
uint16_t chargevoltage = 3950;
uint16_t chargecurrent = 200;
bool candebug = 0;

//Revied info
bool Up1, Up2, Down1, Down2 = false;
bool ParkBut = false;

bool DirChanged, ParkChange = false;
uint8_t PrkCnt = 0;


uint8_t Cnt3FD = 0;
uint16_t ShiftState = 0;

#define CAN0_INT 2  // Set INT to pin 2
MCP_CAN CAN0(9);    // Set CS to pin 9 de[ending on shield used

char mes[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };





void setup() {
  Serial.begin(115200);
  crc8.begin();

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  while (CAN_OK != CAN0.begin(CAN_500KBPS, MCP_16MHZ))  // init can bus : baudrate = 500k  8MHZ crystal
  {
    Serial.println("CAN BUS FAIL!");
    delay(100);
  }
  Serial.println("CAN BUS OK!");
  Serial.println("Time Stamp,ID,Extended,LEN,D1,D2,D3,D4,D5,D6,D7,D8");
}

void loop() {
  if (CAN0.checkReceive() == 3)  // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);  // Read data: len = data length, buf = data byte(s)
    if (rxId == 0x197) {
      RX197();
    }

    if (candebug == 1) {

      //if (rxId > 0x079 && rxId < 0x090)
      // {
      Serial.print(millis());
      if ((rxId & 0x80000000) == 0x80000000)  // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, ",0x%.8lX,true, %1d", (rxId & 0x1FFFFFFF), len);
      else
        sprintf(msgString, ",0x%.3lX,false,%1d", rxId, len);

      Serial.print(msgString);

      if ((rxId & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      } else {
        for (byte i = 0; i < len; i++) {
          sprintf(msgString, ", 0x%.2X", rxBuf[i]);
          Serial.print(msgString);
        }
      }
      Serial.println();
      //}

      /*
          if (rxId == 0x080)
          {
            Serial.println();
            sendcan();
            Serial.println("command recieved");
          }
      */
    }
  }

  if (Serial.available() > 0) {
    Serialcomms();
  }

  if (millis() - looptime > 100) {
    looptime = millis();

    UpdateShifter();
    sendcan();
  }
}

void sendcan() {

  mes[1] = Cnt3FD;
  mes[2] = Dir;
  mes[3] = 0x00;
  mes[4] = 0x00;
  mes[0] = crc8.get_crc8(mes, 5, 0x70, 1);

  CAN0.sendMsgBuf(0x3FD, 0, 5, mes);

  Cnt3FD++;

  if (Cnt3FD == 0xF) {
    Cnt3FD = 0;
  }

  mes[0] = 0xFF;
  mes[1] = 0;

  CAN0.sendMsgBuf(0x202, 0, 2, mes);
}


void Serialcomms() {
  byte incomingByte = Serial.read();
  switch (incomingByte) {
    case 'd':
      candebug = !candebug;
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}

void RX197() {
  Up1 = false;
  Up2 = false;
  Down1 = false;
  Down2 = false;
  ParkBut = false;


  switch (rxBuf[2]) {
    case 0x1E:
      Up1 = true;
      break;

    case 0x2E:
      Up2 = true;
      break;

    case 0x3E:
      Down1 = true;
      break;

    case 0x4E:
      Down2 = true;
      break;

    case 0x0E:
      DirChanged = false;
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }

  switch (rxBuf[3]) {
    case 0xD5:
      PrkCnt++;
      if (PrkCnt > 5) {
        ParkBut = true;
      }
      break;

    case 0xC0:
      PrkCnt = 0;
      ParkBut = false;
      ParkChange = false;
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}

void UpdateShifter() {

  switch (Dir) {
    case Off:
      Dir = Neutral;
      break;

    case Neutral:
      if (DirChanged == false) {
        if (Up2 == true) {
          Dir = Reverse;
          DirChanged = true;
        } else if (Down2 == true) {
          Dir = Drive;
          DirChanged = true;
        } else if (ParkBut == true && ParkChange == false) {
          Dir = Park;
          ParkChange = true;
        }
      }
      break;

    case Reverse:
      if (Down2 == true && DirChanged == false) {
        Dir = Neutral;
        DirChanged = true;
      } else if (ParkBut == true && ParkChange == false) {
        Dir = Park;
        ParkChange = true;
      }
      break;

    case Drive:
      if (Up1 == true && DirChanged == false) {
        Dir = Neutral;
        DirChanged = true;
      } else if (ParkBut == true && ParkChange == false) {
        Dir = Park;
        ParkChange = true;
      }
      break;

    case Park:
      if (ParkBut == true && ParkChange == false) {
        Dir = Neutral;
        ParkChange = true;
      }
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
  Serial.println();
  Serial.print("PC :");
  Serial.print(ParkChange);
  Serial.print("| PB :");
  Serial.print(ParkBut);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
