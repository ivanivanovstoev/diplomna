/*
 * SerialLoRa.cpp
 *
 * Copyright (C) 2018  Ivan Stoev
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
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
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
