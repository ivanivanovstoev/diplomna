/*
 * SerialInt.h
 *
 *  Created on: Apr 1, 2018
 *      Author: tuba
 */

#ifndef SERIALINT_H_
#define SERIALINT_H_
#include <vector>
#include <stddef.h>
#include <functional>

class SerialInt {
public:
  typedef void consume_callback_type(const unsigned char* buffer, size_t count, int rssi);
  typedef std::function<consume_callback_type> consume_function;

  //async write
   virtual void Write(std::vector<unsigned char> &&msg) = 0; // pass the write data to the do_write function via the io service in the other thread
   template<class Iter>
   void Write(Iter start, Iter end) // pass the write data to the do_write function via the io service in the other thread
   {
     Write(std::move(std::vector<unsigned char>(start, end)));
   }

   // blocking write
   virtual size_t BlockingWrite(const std::vector<unsigned char>& msg)
   {
     return BlockingWrite(msg.data(), msg.size());
   }
   virtual size_t BlockingWrite(const unsigned char* msg, size_t size) = 0;
   virtual void SetConsumer(consume_function consume)
   {
     consume_ = consume;
   }
   virtual ~SerialInt(){}
protected:
   consume_function consume_;
};



#endif /* SERIALINT_H_ */
