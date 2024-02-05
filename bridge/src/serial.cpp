#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <Python.h>

#include "serial.hpp"

Serial::Serial(const std::string& port, unsigned int baudrate)
{
    struct termios tty;
    serial = open(port.c_str(), O_RDWR);
    if (serial == -1)
    {
        perror("Error: Unable to open serial port.\n");
        return;
    }

    if (tcgetattr(serial, &tty) != 0)
    {
        perror("Error: Unable to get serial port attributes.\n");
        return;
    }

    cfsetospeed(&tty, baudrate); // Set output baud rate
    cfsetispeed(&tty, baudrate); // Set input baud rate

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_cflag |= (CLOCAL | CREAD);        // Ignore modem control lines, enable receiver
    tty.c_cflag &= ~CSIZE;                  // Clear the size bits
    tty.c_cflag |= CS8;                     // 8-bit data
    tty.c_cflag &= ~PARENB;                 // No parity
    tty.c_cflag &= ~CSTOPB;                 // 1 stop bit

    tcflush(serial, TCIFLUSH);
    if (tcsetattr(serial, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
        return;
    }
}

void Serial::read(uint8_t* buffer, size_t size) {
    ::read(serial, buffer, size);
}

void Serial::write(const uint8_t* buffer, size_t size) {
    ::write(serial, buffer, size);
}

void Serial::init() {
    Py_Initialize();
}

std::vector<Serial::PortInfo> Serial::listPorts() {
    std::vector<Serial::PortInfo> ports;

    PyObject* list_ports_module = PyImport_ImportModule("serial.tools.list_ports");
    if (list_ports_module == NULL) {
        PyErr_Print();
        return ports;
    }

    PyObject* comports_func = PyObject_GetAttrString(list_ports_module, "comports");
    if (comports_func == NULL) {
        PyErr_Print();
        return ports;
    }

    PyObject* port_list = PyObject_CallObject(comports_func, NULL);
    if (port_list == NULL) {
        PyErr_Print();
        return ports;
    }

    for (Py_ssize_t i = 0; i < PyList_Size(port_list); i++) {
        PyObject* port_info = PyList_GetItem(port_list, i);
        PyObject* device = PyObject_GetAttrString(port_info, "device");
        PyObject* name = PyObject_GetAttrString(port_info, "name");
        PyObject* description = PyObject_GetAttrString(port_info, "description");
        PyObject* hwid = PyObject_GetAttrString(port_info, "hwid");

        if (device == NULL || name == NULL || description == NULL || hwid == NULL) {
            PyErr_Print();
            continue;
        }

        Serial::PortInfo info;
        info.device = PyUnicode_AsUTF8(device);
        info.name = PyUnicode_AsUTF8(name);
        info.description = PyUnicode_AsUTF8(description);
        info.hwid = PyUnicode_AsUTF8(hwid);

        ports.push_back(info);
    }

    return ports;
}

template <typename PackedType>
void serialize(const PackedType& src, uint8_t* dest) {
    std::memcpy(dest, &src, sizeof(PackedType));
}

template <typename PackedType>
void deserialize(const uint8_t* src, PackedType& dest) {
    std::memcpy(&dest, src, sizeof(PackedType));
}