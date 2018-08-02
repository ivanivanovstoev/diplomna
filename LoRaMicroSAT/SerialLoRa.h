/*
 * SerialLoRa.h
 *
 *  Created on: Apr 8, 2018
 *      Author: tuba
 */

#ifndef SERIALLORA_H_
#define SERIALLORA_H_
#include "SerialInt.h"
#include <WString.h>
typedef void (*consume_callback_type)(const unsigned char* buffer, size_t count, int rssi);
class SerialLoRa
{
public:
  //SerialInt
  virtual void Write(std::vector<unsigned char> &&msg){}
  // blocking write
  virtual size_t BlockingWrite(const unsigned char* msg, size_t size);

  SerialLoRa ();
  virtual
  ~SerialLoRa ();

  bool Init();
  void Main();

 void SetConsumer(consume_callback_type consume)
 {
   consume_ = consume;
 }
protected:
  String rssi_ = "RSSI --";
  String packSize_ = "--";
  String packet_ ;
  const long frequency_;
  const bool PABOOST;
  const int8_t RST;   // GPIO14 -- SX127x's RESET
  const int8_t DI00;  // GPIO26 -- SX127x's IRQ(Interrupt Request)
  consume_callback_type consume_;

};

extern SerialLoRa SLoRa;

#endif /* SERIALLORA_H_ */
