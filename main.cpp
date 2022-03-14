#include <iostream>
#include <string>
#include "SerialPort.h"
#include "c_library_v2/common/mavlink.h"

void usage() {
  std::cout << "./mavlink_listener <name_of_serial_port> <serial_port_baud>" << std::endl;
}

int main(int argc, char* argv[]) {
  std::string serial_port_name = "";
  int serial_port_baud = 0;
  if(argc < 3) {
    usage();
    return -1;
  }
  try {
    serial_port_name = std::string(argv[1]);
    serial_port_baud = std::stoi(argv[2]);
  } catch(...) {
    usage();
    return -1;
  }

  SerialPort serial(serial_port_name, serial_port_baud);
  mavlink_status_t status;
  mavlink_message_t msg;
  memset(&status, 0, sizeof(mavlink_status_t));
  memset(&msg, 0, sizeof(mavlink_message_t));
  while(true) {
    std::string str = serial.readString();
    if(str.length() > 0) {
      for(auto c : str) {
        if(mavlink_parse_char(0, (uint8_t)c, &msg, &status)) {
          std::cout << "Received mavlink message id: " << (int)msg.msgid << std::endl;
        }
      }
    }
  }
  return 0;
}
