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

#include <iostream>

#include <boost/asio/io_service.hpp>
#include <boost/lexical_cast.hpp>
#include <thread>
#include <vector>
#include <limits>
#include "protocol_parser.h"
namespace sp = sat_protocol;
int main(int argc, char **argv) {
  boost::asio::io_service io_service;

  boost::asio::io_service::work _(io_service);
  int i = 0; 
  if (argc != 3) 
  {
    std::cout << "Usage: \n protocp [port name] -[parameter] \n Parameters are: \n -s send \n -l listen" << std::endl;
    return  (1);
  }
  if (!strcmp(argv[2], "-s"))
  {
    sat_protocol::protocol_parser deviceA(io_service, 1, argv[1]);
    deviceA.Init();
    std::cout << "Sending" << std::endl;
    std::thread run_thread([&]{ io_service.run(); });
    while (1)
    {
      i++;
      if (i == std::numeric_limits<int>::max()) i = 1;
      std::string smsg = "Sending packetnumber: ";
      smsg += boost::lexical_cast<std::string>(i);
    
      std::vector<unsigned char> msg (smsg.begin(), smsg.end()); 
      deviceA.SendMsg(sp::CMD_DATA, sp::INS_NO_ACK, std::move(msg));
      sleep(5);
    }
  } else {
    sat_protocol::protocol_parser deviceA(io_service, 2, argv[1]);
    deviceA.Init();
    std::cout << "Receiving!" << std::endl;
    io_service.run();
  }
  std::cout << "Stopped" << std::endl;
      sleep(50);
  return 0;
}
