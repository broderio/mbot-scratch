#include <iostream>
#include <string>
#include <vector>

#include "serial.hpp"
#include "server.hpp"


void handleSession(websocket::stream<tcp::socket>& ws) {
    std::cout << "Session started" << std::endl;
}

int main(int argc, char* argv[]) {
    // Initialize serial communication
    Serial::init();

    // Find MBot Wireless Host
    std::vector<Serial::PortInfo> ports = Serial::listPorts();
    std::string serialPort = "";
    for (const auto& port : ports) {
        if (port.hwid.find("303A:1001") != std::string::npos 
            || port.hwid.find("10C4:EA60") != std::string::npos 
            || port.hwid.find("303A:4001") != std::string::npos) {
            std::cout << "Found MBot Wireless Host on port " << port.device << std::endl;
            serialPort = port.device;
        }
    }
    if (serialPort.empty()) {
        std::cerr << "MBot Wireless Host not found" << std::endl;
        return 1;
    }

    // Open serial port
    Serial serial(serialPort, 921600);

    // Start server
    std::string address = "0.0.0.0";
    int port = 8765;
    Server server(address, port, handleSession);
    server.run();

    return 0;
}