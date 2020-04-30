#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <Arduino.h>
#include <SoftwareSerial.h>

#include "Buzzer.h"
#include "Hexapod.h"
#include "UltraSonicSensor.h"

// states for processing incoming bluetooth data

#define P_WAITING_FOR_HEADER      0
#define P_WAITING_FOR_VERSION     1
#define P_WAITING_FOR_LENGTH      2
#define P_READING_DATA            3
#define P_WAITING_FOR_CHECKSUM    4
#define P_SIMPLE_WAITING_FOR_DATA 5

#define MAXPACKETDATA 48
extern unsigned char packetData[MAXPACKETDATA];
extern unsigned int packetLength;
extern unsigned int packetLengthReceived;
extern int packetState;

extern SoftwareSerial BlueTooth;  // Bluetooth pins: TX=3=Yellow wire,  RX=2=Green wire

unsigned int bluewriteword(int w);
void packetErrorChirp(char c);

extern byte lastCmd;
extern byte priorCmd;

int receiveDataHandler();
void processPacketData();
void sendSensorData();
void dumpPacket();


#endif