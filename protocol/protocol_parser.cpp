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

#include "protocol_parser.h"
#include <stdint.h>
#include "IostreamLogger.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "crc32.h"

using namespace sat_protocol;
static const int ADDEDBYTES = 3 /*Header*/ + 2 /*Size*/ + 4 /*CRC*/+ 1 /*Message CRC*/;
static const int HEADERSIZE = ADDEDBYTES - 5;

static const boost::posix_time::milliseconds POS_TIMER_INTERVAL_MS(100);// For tests(250);
static const boost::posix_time::milliseconds POS_TIMER_ERROR_INTERVAL_MS(1000);

template <typename ElemT>
struct HexTo {
    ElemT value;
    operator ElemT() const {return value;}
    friend std::istream& operator>>(std::istream& in, HexTo& out) {
        in >> std::hex >> out.value;
        return in;
    }
};
static uint32_t Read32Big(const unsigned char* x)
{
  return  (((uint32_t)x[0] << 24)&0xff000000)|(((uint32_t)x[1] << 16)&0x00ff0000)|(((uint32_t)x[2] << 8) &0x0000ff00)|((uint32_t)x[3] &0x000000ff); 
}
static uint16_t Read16Big(const unsigned char* x)
{
  return (((uint16_t) x[0] << 8 )&0xff00)|(((uint16_t) x[1])&0x00ff); 
}

static uint8_t CalcCRC8(const unsigned char* pabBuff, short nSize)
{
  int i;
  uint8_t bCRC = 0;
  
  if(nSize <= 0)
    return false;

  bCRC = pabBuff[0];
  for(i = 1; i < nSize; i++)
  {
    bCRC ^= pabBuff[i];
  }
  
  return bCRC;
}

static SatCommError Byte2Err(SAT_MSG_ERR ec)
{
  switch (ec)
  {
    case SAT_ERR_OK:
      return SatCommError::OK;
    case SAT_ERR_SCRC:
    case SAT_ERR_MCRC:
      return SatCommError::RemoteCRC;
    case SAT_ERR_SEQ:
      return SatCommError::RemoteSeq;
    default:
      return SatCommError::General;
  }
}

uint8_t ErrToByte (SatCommError err)
{

  switch (err)
  {
     case SatCommError::OK:
      return SAT_ERR_OK;
     case SatCommError::HeaderCRC:
      return SAT_ERR_SCRC;
     case SatCommError::BodyCRC: 
      return SAT_ERR_MCRC;
    case SatCommError::RemoteSeq:
      return SAT_ERR_SEQ;
    default:
      return SAT_ERR_OK;
  }
}

void protocol_parser::upon_msg_received (SatCommandMsg& m_oMsg)
{
  ILOG_MSG("upon_msg_received") << "Serial port: " << serial_.GetName();
  ILOG_MSG("upon_msg_received") << " deviceid_ " << m_oMsg.deviceid_ 
                                  <<" command_  and instruction_ ["
 << (int)(unsigned char) m_oMsg.command_<<"|" << (int)(unsigned char)m_oMsg.instruction_ <<
  "] paket number " << m_oMsg.packet_number_ << " err " << m_oMsg.err_ ;
}

// static void byteswap(uint16_t& s)
// {
//   s = ((s&0xFF00)>>8)|((s&0x00FF)<<8);
// }

bool protocol_parser::Init() {
  bool ret = true;
//   ret &=  SendMsg("6000024330"); //Reset to defaults
//   ret &=  SendMsg("600003433100"); //Set track 1 off
//   ret &=  SendMsg("600003433201"); //Set track 2 on
//   ret &=  SendMsg("600003433300"); //Set track 3 off
  SetCallback([this](SatCommandMsg& m_oMsg){upon_msg_received( m_oMsg);});
  return ret;
}

// ReaderMsg::ReaderMsg(UI::ReaderMsg&& o) :m_nSize(o.m_nSize), m_bCalcCrc(o.m_bCalcCrc),
//     m_bRecvCrc(o.m_bRecvCrc), m_bErrorCode(o.m_bErrorCode),m_abMsg(std::move(o.m_abMsg))
// {}

SatCommandMsg::SatCommandMsg(uint32_t deviceid, unsigned char command, 
                  unsigned char instruction, uint32_t packet_number,  std::vector<unsigned char>&& msg) :
          deviceid_(deviceid), command_(command), instruction_(instruction), packet_number_(packet_number),
          msg_(std::move(msg))    
{
  datasize_ = sizeof(deviceid_) + sizeof(command_) + sizeof(instruction_) + sizeof(packet_number_) + msg_.size();
}

void protocol_parser::SetCallback(const SatCallback& callback) {
  //ILOG_MSG("SetCallback") << "New callback set";
     callback_ = callback;
}

bool protocol_parser::HandleACKCommand(const SatCommandMsg& msg)
{
  if (msg.command_ == CMD_ACK)
  {
    return true;
  } else if (msg.command_ ==CMD_NACK )
  {
    return true;
  } else return false;
}

void protocol_parser::DispatchCommandMsg(SatCommandMsg& msg) {
  //ILOG_MSG("DispatchStatusMsg") << "Msg [" << msg.m_abMsg[0]<<msg.m_abMsg[1] << msg.m_abMsg[2] << "]";
  if (msg.err_ == SatCommError::General){
    ILOG_ERR("DispatchStatusMsg") << "Bad packet discarding wron data!";
    return;
  }
  if ((msg.instruction_ == INS_ACK) ||(msg.instruction_ == INS_NACK))
  {
    std::vector<unsigned char> packetnum;
    packetnum.reserve(4);
    packetnum[0] = (unsigned char) (msg.packet_number_ >> 24) & 0xFF;
    packetnum[1] = (unsigned char) (msg.packet_number_ >> 16) & 0xFF;
    packetnum[2] = (unsigned char) (msg.packet_number_ >> 8) & 0xFF;
    packetnum[3] = (unsigned char) msg.packet_number_ & 0xFF;
    if (msg.err_ == SatCommError::OK)
    {
      if (msg.instruction_ == INS_ACK)
        SendMsg(CMD_ACK, INS_NO_ACK, std::move(packetnum));
    }
    else 
      SendMsg(CMD_NACK, INS_NO_ACK, std::move(packetnum), msg.err_);
  }
  if (!HandleACKCommand(msg))
  {
    if (callback_)
      callback_(msg);
  }
}

void protocol_parser::AddData(const unsigned char* buffer, size_t count)
{
  buff_.insert(buff_.end(), buffer, buffer + count);

  while (true)
  {
    if (buff_.size() < ADDEDBYTES) //Not enought to process ?
      break;

    if ((buff_[0] ==  0x2B) && (buff_[1] ==  0x2B) && (buff_[2] ==  0x2B) )
    {
      uint16_t msgsize  = Read16Big(&buff_[3]);
      uint32_t sCrc = 0;
      crc32(sCrc, buff_.data(), HEADERSIZE);
      uint32_t msCrc = Read32Big(&buff_[5]);
      if (sCrc != msCrc)
      //Broken message swallow.
      { 
        ILOG_MSG("AddData")<< "Descarding wrong msg size CRC";
        RewindBuffer(false);
        continue;
      }

      if (msgsize + ADDEDBYTES > (uint16_t)buff_.size()) // Hole message not arrived yet.
        break;
      SatCommandMsg msg = ParseCommandMsg(msgsize);
      DispatchCommandMsg(msg);
      //Removin msgsize + 4 bytes that's:
      //1 - header; 2 - size ; 1 - CRC
      std::vector<unsigned char>(buff_.begin()+(msgsize + ADDEDBYTES ), buff_.end()).swap(buff_); 
    }
    else RewindBuffer(true);
  }
}

void protocol_parser::RewindBuffer (bool partial)
{
  size_t btor = partial ? 0:3;
  for (; btor < buff_.size(); ++btor)
  {
    if (buff_[btor] == 0x2B)
    {
      break; // New start found.
    }
  }
  if (btor > 0 && !buff_.empty()) {
    std::vector<unsigned char>(buff_.begin()+btor, buff_.end()).swap(buff_); 
  }
}

SatCommandMsg protocol_parser::ParseCommandMsg(uint16_t nDataSize)
{
  char bCrc;
  SatCommandMsg msg;
  if(nDataSize == 0)
  {
    msg.err_ = SatCommError::General;
    std::string str = "Empty message received!";
    msg.msg_ =  std::vector<unsigned char>(str.begin(), str.end());
    return msg;
  }
  int nMsgSize =  nDataSize + ADDEDBYTES ; //Size of the message without the CRC and the index of the CRC
  bCrc = CalcCRC8(&buff_[0], nMsgSize);
  if (bCrc !=  buff_[nMsgSize])
    msg.err_ = SatCommError::BodyCRC;
  else // Copy entire mesage for logging purpouses
    msg.err_ = Byte2Err((SAT_MSG_ERR) buff_[19]);
  msg.datasize_ = nDataSize;
  msg.deviceid_ = Read32Big(&buff_[9]);
  msg.packet_number_ = Read32Big(&buff_[15]);
  msg.command_ = buff_[13];
  msg.instruction_ = buff_[14];
  if (nMsgSize > 20)
  msg.msg_ = std::vector<unsigned char> (buff_.begin() + 20, buff_.begin() + nMsgSize - 1 ); // -1 for the message crc 

  return msg;
}

protocol_parser::protocol_parser(boost::asio::io_service& io_service, uint32_t device_id, const std::string& serial): serial_(io_service), port_(serial),
     POSTimer_(io_service, POS_TIMER_INTERVAL_MS), POSErrTimer_(io_service, POS_TIMER_ERROR_INTERVAL_MS), StatusPending_(false),
     cardIn_(false), status_(SatCommError::OK), packet_number_(0), device_id_(device_id)
{
     serial_.SetConsumer(boost::bind(&protocol_parser::AddData, this,_1,_2));
     serial_.Open(port_.c_str());
     serial_.SetBaudRate(9600);
     serial_.ReadStart();
     serial_.Drain();
     Init();
      //POSTimer_.async_wait(boost::bind(&protocol_parser::do_POSTimer, this, _1));
}

void protocol_parser::do_POSTimer(const boost::system::error_code& e)
{
  SendASCIIMsg("600002433A");
     POSTimer_.expires_from_now(POS_TIMER_INTERVAL_MS);
     POSTimer_.async_wait(boost::bind(&protocol_parser::do_POSTimer, this, _1));
}

void protocol_parser::do_POSError(const boost::system::error_code& e)
{
  SatCommandMsg dummy;
//   if (m_Status ==  SatComunicationError::HALF_INSERTED)
//     m_callback(dummy, SatComunicationError::BACK_INSERTED);
}

bool protocol_parser::SendASCIIMsg(const std::string& crmsg) 
{
  std::vector<unsigned char> ret= ASCIIToBin(crmsg);
  if (ret.size() > 0) {
    uint32_t dCRC = 0;
    crc32(dCRC, ret.data(), HEADERSIZE);
    ret[5] = (unsigned char) (dCRC >> 24) & 0xFF;
    ret[6] = (unsigned char) (dCRC >> 16) & 0xFF;
    ret[7] = (unsigned char) (dCRC >> 8) & 0xFF;
    ret[8] = (unsigned char) dCRC & 0xFF;
    ret[ret.size() - 1] = CalcCRC8(ret.data(), ret.size());
  }
  serial_.Write(std::move(ret));
  return true;
}

bool protocol_parser::SendMsg(uint8_t command, uint8_t instruction, std::vector<unsigned char>&& msg, SatCommError err)
{
  SatCommandMsg crmsg(device_id_, command, instruction, packet_number_++, std::move(msg));
  crmsg.err_ = err;
  return SendMsg(crmsg);
}

bool protocol_parser::SendMsg(const SatCommandMsg& crmsg) {
  std::vector<unsigned char> msg;
  uint32_t size = crmsg.datasize_ + ADDEDBYTES ;
  msg.resize(size);
  msg[0] = 0x2B;
  msg[1] = 0x2B;
  msg[2] = 0x2B;
  msg[3] = (unsigned char) (crmsg.datasize_ >> 8);
  msg[4] = (unsigned char) (crmsg.datasize_&0x00FF);
  uint32_t dCRC = 0;
  crc32(dCRC, msg.data(), HEADERSIZE);
  std::cout << "CRC is "<< boost::lexical_cast<std::string>(dCRC) <<std::endl; 
  msg[5] = (unsigned char) (dCRC >> 24) & 0xFF;
  msg[6] = (unsigned char) (dCRC >> 16) & 0xFF;
  msg[7] = (unsigned char) (dCRC >> 8) & 0xFF;
  msg[8] = (unsigned char) dCRC & 0xFF;
  uint32_t test = Read32Big(&msg[5]);
   std::cout << "Test  is "<< boost::lexical_cast<std::string>(test) <<std::endl;
  msg[9] = (unsigned char) (crmsg.deviceid_ >> 24) & 0xFF;
  msg[10] = (unsigned char) (crmsg.deviceid_ >> 16) & 0xFF;
  msg[11] = (unsigned char) (crmsg.deviceid_ >> 8) & 0xFF;
  msg[12] = (unsigned char) crmsg.deviceid_ & 0xFF;;
  msg[13] = crmsg.command_;
  msg[14] = crmsg.instruction_;
  msg[15] = (unsigned char) (crmsg.packet_number_ >> 24) & 0xFF;
  msg[16] = (unsigned char) (crmsg.packet_number_ >> 16) & 0xFF;
  msg[17] = (unsigned char) (crmsg.packet_number_ >> 8) & 0xFF;
  msg[18] = (unsigned char) crmsg.packet_number_ & 0xFF;
  msg[19] = ErrToByte(crmsg.err_);
  memcpy(&msg[20], &crmsg.msg_[0], crmsg.msg_.size());
  msg[20 + crmsg.msg_.size()] =  CalcCRC8(msg.data(), msg.size());
  std::string the_text ((char*)&crmsg.msg_[0], crmsg.msg_.size());
  std::cout << "Actual msg [" << the_text << "]" << std::endl;
  HexDump(msg);
  serial_.Write(std::move(msg));
  return true;
}



std::ostream& operator<< (std::ostream& os, SatCommError ethertype)
{
  switch (ethertype)
  {
    case SatCommError::OK : return os << "OK" ;
    case SatCommError::BodyCRC: return os << "BODY_CRC_ERR";
    case SatCommError::HeaderCRC: return os << "HEADER_CRC_ERR";
    case SatCommError::Comm: return os << "COMM_ERROR";
    case SatCommError::RemoteCRC: return os << "REMOTE_CRC_ERR";
    case SatCommError::RemoteSeq: return os << "REMOTE_SEQ_ERR";
    case SatCommError::General: return os << "ERROR";
  };
  return os << static_cast<std::uint16_t>(ethertype);
}

// void protocol_parser::handle_protocol(protocol_type type){}
// bool protocol_parser::crc_check(protocol_header* header) {}

