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
