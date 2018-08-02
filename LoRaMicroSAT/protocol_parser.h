/*
*  protocol_parser.h
*
*  Created on: Mar 23, 2018
*      Author: Tuba <tuba@abv.bg>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*
*/

#ifndef PROTOCOL_PARSER_H
#define PROTOCOL_PARSER_H

#include <stdint.h>
#include "SerialInt.h"
#include "SerialLoRa.h"

namespace sat_protocol {

enum SAT_MSG_ERR {
SAT_ERR_OK = 0x30,
SAT_ERR_SCRC = 0x31,
SAT_ERR_SEQ = 0x32,
SAT_ERR_MCRC = 0x33
} ;

enum msg_instruction_type
{
INS_ACK = 0,
INS_NACK,
INS_NO_ACK
};

enum msg_command_type
{
CMD_ACK = 0,
CMD_NACK,
CMD_BALISTIC,
CMD_INFO,
CMD_DATA
};

enum msg_ins_type
{
INS_SENSOR_INF = 0,
INS_SENSOR_DATA,
INS_GPS_DATA
};

enum class SatCommError
{
OK,
HeaderCRC,
BodyCRC,
Comm,
RemoteCRC,
RemoteSeq,
General
};

class Buffer{
public:
  Buffer():size_(0)
  {}
  size_t size(){return size_;}
  unsigned char *begin(){return &arena[0]; }
  unsigned char *end() {return &arena[size_];}
  size_t insert(unsigned char *where, const unsigned char *begin, size_t size);
  unsigned char& operator[](size_t index);
  const unsigned char& operator[](size_t index) const;
  unsigned char* data(){return &arena[0];}
  void movebegin(const unsigned char *begin, size_t size);
  bool empty(){return size_ == 0;}
private:
  const static int ARENA_SIZE = 2049;
  static unsigned char arena[ARENA_SIZE];
  size_t size_;
};

struct SatCommandMsg
{

SatCommandMsg():err_(SatCommError::General){}
SatCommandMsg(uint32_t deviceid, uint8_t command,
              uint8_t instruction, uint32_t packet_number,
              std::vector<unsigned char>&& msg);
SatCommandMsg(uint32_t deviceid, uint8_t command,
              uint8_t instruction, uint32_t packet_number,
              const unsigned char* msg,
              size_t size);
void InitSize();
uint16_t datasize_; // The actual size of m_bMsg + 2
uint32_t deviceid_;
uint8_t command_;
uint8_t instruction_;
uint32_t packet_number_;
SatCommError err_;
std::vector<unsigned char> msg_;// The actual size of hole message in bytes = m_bDataSize + 4 or
int rssi_;
};

//typedef void (card_callback_t)(SatommandMsg& m_oMsg);
//typedef std::function<card_callback_t> SatCallback;

class protocol_parser
{
public:
protocol_parser(uint32_t device_id);
virtual ~protocol_parser(){}
bool Init();
//void SetCallback(const SatCallback& callback);
// Sends ASCII encoded messages converts them to binary and inplace fills the CRC
bool SendASCIIMsg(const std::string& crmsg);
// Sends message only starting with command and instruction byte
bool SendMsg(uint8_t command, uint8_t instruction, const unsigned char* msg, size_t size, SatCommError err = SatCommError::OK);
bool SendMsg(uint8_t command, uint8_t instruction, std::vector<unsigned char>&& msg, SatCommError err = SatCommError::OK);
bool SendMsg(const SatCommandMsg& msg);

bool Discover(){return true;}
void AddData(const unsigned char* buffer, size_t count);
private:
void DispatchCommandMsg(SatCommandMsg& msg);
SatCommandMsg ParseCommandMsg(uint16_t nDataSize);

void do_POSTimer(int e);
void do_POSError(int e);
void RewindBuffer (bool prfailed);
bool HandleACKCommand(const SatCommandMsg& msg);
void upon_msg_received (SatCommandMsg& m_oMsg);

//SatCallback callback_;
std::string port_;
bool StatusPending_;
size_t PendingSize_;
Buffer buff_;
bool cardIn_;
SatCommError status_;
uint32_t packet_number_;
uint32_t device_id_;
};
}
void AddData(const unsigned char* buffer, size_t count, int rssi);
extern sat_protocol::protocol_parser pp;
#endif // PROTOCOL_PARSER_H
