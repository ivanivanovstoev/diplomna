/*
 * SerialInt.h
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
