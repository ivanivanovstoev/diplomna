/*
 * SerialLoRa.cpp
 *
 *  Created on: Apr 8, 2018
 *      Author: tuba
 */

#include "SerialLoRa.h"
#include <SPI.h>
#include <LoRa.h>
SerialLoRa SLoRa;
#include "protocol_parser.h"
SerialLoRa::SerialLoRa (): frequency_(433E6), PABOOST(false), RST(14), DI00(26)
{}

SerialLoRa::~SerialLoRa ()
{
  // TODO Auto-generated destructor stub
}

bool SerialLoRa::Init(){
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI00);

  if (!LoRa.begin(frequency_,PABOOST)) {
     Serial.println( "Starting LoRa failed!");
   while (1);
  }

  Serial.println("LoRa Initial success!");
  //display.drawString(0, 10, "Wait for incomm data...");
  delay(1000);
  //LoRa.onReceive(cbk);
  LoRa.receive();
  return true;
}

void SerialLoRa::Main()
{
  Serial.print("Receiving : ");
  LoRa.receive();
  int packetSize = LoRa.parsePacket();
  Serial.println(packetSize);
  if (packetSize) {
    packet_ ="";
    packSize_ = String(packetSize,DEC);
    for (int i = 0; i < packetSize; i++) { packet_ += (char) LoRa.read(); }
    rssi_ = "RSSI " + String(LoRa.packetRssi(), DEC) ;
    //consume_((unsigned char *)packet_.c_str(), packetSize, LoRa.packetRssi());
    pp.AddData((unsigned char *)packet_.c_str(), packetSize);
  }
 // loraData();
}

size_t SerialLoRa::BlockingWrite(const unsigned char* msg, size_t size) {
  LoRa.beginPacket();
  Serial.println(1); /// WTF REmove this line and it hangs !!!!
  for (int i = 0;i < size; ++i)
    {
    LoRa.print(msg[i]);}
  LoRa.endPacket();
  return size;
}
