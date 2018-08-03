/*
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

#include "serial_port.h"

boost::asio::io_service & SerialPort::get_io_service()
  {
    return io_service;
  }

SerialPort::~SerialPort()
{
}


void SerialPort::assign(int a)
  {
    serial_port.assign(a);
  }


void SerialPort::SetConsumer(consume_function consum)
  {
    consume = consum;
  }


void SerialPort::SetErrorHandler(error_callback er)
  {
    error = er;
  }


void SerialPort::SetRaw()
  {
    int fd = serial_port.native();
    termios options;
    tcgetattr(fd, &options);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &options);
  }

void SerialPort::SetBaudRate(int baud)
{
   boost::asio::serial_port_base::baud_rate baud_option(baud); 
   serial_port.set_option(baud_option); 
}
void SerialPort::SetBits(int bits)
{
  boost::asio::serial_port_base::character_size csize( bits );
  serial_port.set_option(csize); 
}

void SerialPort::Open(const std :: string & device, unsigned int baud)
  {
    name=device;
    serial_port.open(device);
    if (not serial_port.is_open()) 
      throw std::runtime_error("Can't open serial port");
    boost::asio::serial_port_base::baud_rate baud_option(baud); 
    boost::asio::serial_port_base::flow_control no_flow_control( boost::asio::serial_port_base::flow_control::none);
    boost::asio::serial_port_base::stop_bits stop_bits(boost::asio::serial_port_base::stop_bits::one);
    boost::asio::serial_port_base::character_size character_size(8);
    boost::asio::serial_port_base::parity parity(boost::asio::serial_port_base::parity::none);
    serial_port.set_option(baud_option); 
    serial_port.set_option(no_flow_control);
    serial_port.set_option(stop_bits);
    serial_port.set_option(character_size);
    serial_port.set_option(parity);
    SetRaw();
  }

void SerialPort::LooseControllyngTTY()
{
  int fd = serial_port.native();
  ioctl(fd, TIOCNOTTY, 0);
}

void SerialPort::Drain()
{
  int fd = serial_port.native();
  tcdrain(fd);
  //  FlushBoth();
}
void SerialPort::FlushInput()
{
  int fd = serial_port.native();
  tcflush(fd, TCIFLUSH);
}
void SerialPort::FlushOutput()
{
  write_msgs.clear();
  int fd = serial_port.native();
  tcflush(fd, TCOFLUSH);
}

void SerialPort::FlushBoth()
{
  write_msgs.clear();
  int fd = serial_port.native();
  tcflush(fd, TCIOFLUSH);
}

SerialPort::SerialPort(boost :: asio :: io_service & io_service)
    : io_service(io_service), 
      serial_port(io_service) ,
      reading(false),
      shrink_counter(0)
  {
    error = boost::bind(&SerialPort::DoCancelOrClose, this,_1);//(const boost::system::error_code& error)

  }


void SerialPort::CallConsume(const unsigned char* msg, size_t bytes_transfered)
  {
    //std::cout << name << " is consuming" << std::endl;
    std::string a((char*)msg, bytes_transfered);
    HexDump(a);
    if (consume)
      consume(msg, bytes_transfered); // echo to standard output
  }


void SerialPort::HandleError(const boost :: system :: error_code & error_code)
  {
    if (error)
      error(error_code);
  }



void SerialPort::Write(std :: vector <unsigned char>&& msg)
  {
    //    io_service.post(boost::bind(&SerialPort::DoWrite, this, msg)); 
    io_service.post([this, msg]()mutable{ DoWrite(std::move(msg));});//;boost::bind(&SerialPort::DoWrite, this, msg));
    const unsigned val = 0x100;
    if (shrink_counter++>val)
    {
      write_msgs.shrink_to_fit();
      shrink_counter=0;
    }
  } 


size_t SerialPort::BlockingWrite(const std :: vector <unsigned char> msg)
  {
    size_t ret = serial_port.write_some(boost::asio::buffer(msg));
    if (ret >0)
      fsync(serial_port.native());
    return ret;
  } 


void SerialPort::Close()
  {
    io_service.post(boost::bind(&SerialPort::DoClose, this)); 
  } 


void SerialPort::ReadStart(void)
  {
    to_stop_reading=false;
    if (!reading)
    {
      reading=true;
      serial_port.async_read_some(boost::asio::buffer(read_msg, max_read_length), 
                                  boost::bind(&SerialPort::ReadComplete, 
                                              this, 
                                              boost::asio::placeholders::error, 
                                              boost::asio::placeholders::bytes_transferred));
      
    }
  }


void SerialPort::Stop (void)
  {
    serial_port.cancel();
  }



void SerialPort::ReadComplete(const boost :: system :: error_code & error, size_t bytes_transferred)
  {
    reading=false;
    if (!error) 
    {       
      CallConsume(read_msg, bytes_transferred);
      if(!to_stop_reading)
      {
        ReadStart(); // start waiting for another asynchronous read again
      }
    }
    else
    {
      //      std::cout << "some error happend" << std::endl;
      HandleError(error);
    }
  } 
        


void SerialPort::DoWrite(std :: vector <unsigned char> &&msg)
  {

    bool write_in_progress = !write_msgs.empty(); // is there anything currently being written? 
            
    write_msgs.push_back(std::move(msg)); // store in write buffer

    if (!write_in_progress) // if nothing is currently being written, then start 
      WriteStart(); 
  } 
// void SerialPort::DoWrite(const std :: vector < char > &msg)
//   {

//     bool write_in_progress = !write_msgs.empty(); // is there anything currently being written? 
//     write_msgs.push_back(msg); // store in write buffer
//     //    HexDump(msg);
//     if (!write_in_progress) // if nothing is currently being written, then start 
//       WriteStart(); 
//   } 


void SerialPort::WriteStart(void)
  { // Start an asynchronous write and call write_complete when it completes or fails
    current_write = std::move(write_msgs.front());
    write_msgs.pop_front();
    boost::asio::async_write(serial_port, 
           boost::asio::buffer(current_write),
           boost::bind(&SerialPort::WriteComplte, 
           this, 
           boost::asio::placeholders::error));

  } 


void SerialPort::WriteComplte(const boost :: system :: error_code & error)
  { // the asynchronous read operation has now completed or failed and returned an error 
    if (!error) 
    { // write completed, so send next write data
      if (!write_msgs.empty()) // if there is anthing left to be written 
        WriteStart(); // then start sending the next item in the buffer
    } else
      HandleError(error);
  }


void SerialPort::DoCancelOrClose(const boost :: system :: error_code & error)
  {
    if (error == boost::asio::error::operation_aborted) // if this call is the result of a timer cancel() 
      return; // ignorxe it because the connection cancelled the timer
    //    std::cout <<"Serial port error is " <<  error;
    int fd = serial_port.native();
    DoClose();
  }


void SerialPort::DoClose()
  {
    serial_port.close(); 
  } 

