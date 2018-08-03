/*
 * This is LoRa Serial Gateway
 * Data received from LoRa interface is forwarded to the USB interface and vice versa.
 * Information of received packets is write on the in-build OLED display.
 * 
 * The on-board OLED display is SSD1306 driver and I2C interface. In order to make the
 * OLED correctly operation, you should output a high-low-high(1-0-1) signal by soft-
 * ware to OLED's reset pin, the low-level signal at least 5ms.
 * 
 * OLED pins to ESP32 GPIOs via this connecthin:
 * OLED_SDA -- GPIO4
 * OLED_SCL -- GPIO15
 * OLED_RST -- GPIO16
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

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "SSD1306.h"
#include "images.h"

// Pin definetion of WIFI LoRa 32
// HelTec AutoMation 2017 support@heltec.cn
static const int8_t RST = 14;   // GPIO14 -- SX127x's RESET
static const int8_t DI00 = 26;   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6
#define PABOOST false

SSD1306 display(0x3c, 4, 15);
String rssi = "RSSI --";
String packSize = "--";
String packet ;
int loopc = 0;
int counter = 0;

void logo(){
  display.clear();
  display.drawXbm(0,5,logo_width,logo_height,logo_bits);
  display.display();
}

void DisplayOnData(bool lora);
void SerialWrite(int packetSize) {
  packet ="";
  packSize = String(packetSize,DEC);
  for (int i = 0; i < packetSize; i++) { packet += (char) LoRa.read(); }
  rssi = "LORA RSSI " + String(LoRa.packetRssi(), DEC) ;
  Serial.print(packet);
  DisplayOnData(true);
}

void DisplayOnData(bool lora){
  String heading ;
  if (lora)
    heading = "LoRa";
  else
    heading = "Serial";
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0 , 15 , heading);
  int offset = lora?50:70;
  display.drawString(offset , 15 , "Received "+ packSize + " bytes");
  display.drawString(0, 0, rssi);
  display.display();
}

void LoRaWrite(int bytes) {
  String storedData = "";
  storedData.reserve(bytes);
  LoRa.beginPacket();
  int count = 0;
  while ((Serial.available() > 0) && (count++ < 1024)) {
    // read the incoming byte:
    int incomingByte = Serial.read();
    if (incomingByte < 0)
      break;
    storedData += (char)incomingByte;
  }
  LoRa.print(storedData);
  LoRa.endPacket();
  DisplayOnData(false);
}

void setup() {

  Serial.begin(115200);
  while (!Serial);
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  logo();
  delay(1500);
  display.clear();
  display.drawString(0, 0,"LoRa Serial Gateway");

  pinMode(25,OUTPUT);

  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI00);

  if (!LoRa.begin(BAND,PABOOST)) {
    Serial.println("Starting LoRa failed!");
    display.drawString(0, 15, "Starting LoRa failed!");
    while (1);
  }
  display.drawString(0, 15, "LoRa Initial success!");
  display.display();
  delay(1000);
 // LoRa.onReceive(cbk);
  LoRa.receive();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize)
    SerialWrite(packetSize);
  delay(10);
  packetSize = Serial.available();
  if (packetSize > 0)
    LoRaWrite(packetSize);
}
