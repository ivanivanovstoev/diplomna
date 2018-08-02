/*
* <one line to give the program's name and a brief idea of what it does.>
* Copyright (C) 2017  <copyright holder> <email>
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
#include <vector>
#include <stdint.h>
#include <string>
#include <functional>
#include <boost/asio/io_service.hpp>
#include <boost/asio/deadline_timer.hpp>
#include "serial_port.h"

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


struct SatCommandMsg
{
SatCommandMsg():err_(SatCommError::General){}
SatCommandMsg(uint32_t deviceid, unsigned char command, 
unsigned char instruction, uint32_t packet_number,  std::vector<unsigned char>&& msg);
uint16_t datasize_; // The actual size of m_bMsg + 2
uint32_t deviceid_;
uint8_t command_;
uint8_t instruction_;
uint32_t packet_number_;
SatCommError err_;
std::vector<unsigned char> msg_; // The actual size of hole message in bytes = m_bDataSize + 4 or 
};

typedef void (card_callback_t)(SatCommandMsg& m_oMsg);
typedef boost::function<card_callback_t> SatCallback;

class protocol_parser
{
public:
protocol_parser(boost::asio::io_service& io_service, uint32_t device_id, const std::string& serial);
virtual ~protocol_parser(){}
bool Init();
void SetCallback(const SatCallback& callback);
// Sends ASCII encoded messages converts them to binary and inplace fills the CRC
bool SendASCIIMsg(const std::string& crmsg); 
// Sends message only starting with command and instruction byte 
bool SendMsg(uint8_t command, uint8_t instruction, std::vector<unsigned char>&& msg, SatCommError err = SatCommError::OK);
bool SendMsg(const SatCommandMsg& msg);

bool Discover(){return true;}
private:
void DispatchCommandMsg(SatCommandMsg& msg);
SatCommandMsg ParseCommandMsg(uint16_t nDataSize);
void AddData(const unsigned char* buffer, size_t count);
void do_POSTimer(const boost::system::error_code& /*e*/);
void do_POSError(const boost::system::error_code& /*e*/);
void RewindBuffer (bool prfailed);
bool HandleACKCommand(const SatCommandMsg& msg);
void upon_msg_received (SatCommandMsg& m_oMsg);

SerialPort serial_;
SatCallback callback_;
std::string port_;
boost::asio::deadline_timer POSTimer_;
boost::asio::deadline_timer POSErrTimer_;
bool StatusPending_;
size_t PendingSize_;
std::vector<unsigned char> buff_;
bool cardIn_;
SatCommError status_;
uint32_t packet_number_;
uint32_t device_id_;
};
}
#endif // PROTOCOL_PARSER_H
