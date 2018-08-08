#include "comm6w485.hpp"
#include <iostream>
Comm6W485::Comm6W485() : m_serial(),
                       m_baudrate(38400),
                       m_port(""),
                       m_timeout(1000)
{
    // TODO: initial serial library first
}

Comm6W485::~Comm6W485()
{
    if (m_serial.isOpen())
    { // If there is port open, close it
        m_serial.close();
    }
}

bool Comm6W485::connect()
{
    // Makesure there is port close
    if (m_serial.isOpen())
    { // If there is port open, close it
        m_serial.close();
    }

    serial::Timeout tm = serial::Timeout::simpleTimeout(m_timeout);

    // Configure the new setting with this class
    m_serial.setBaudrate(m_baudrate);
    m_serial.setPort(m_port);
    m_serial.setTimeout(tm);
    m_serial.setParity(serial::parity_none);

    try
    {
        m_serial.open();
    }
    catch (const std::invalid_argument e)
    {
        std::cerr << "[Comm6W485] Open error, invalid_argument, " << e.what() << std::endl;
    }
    catch (const serial::SerialException e)
    {
        std::cerr << "[Comm6W485] Open error, SerialException, " << e.what() << std::endl;
    }
    catch (const serial::IOException e)
    {
        std::cerr << "[Comm6W485] Open error, IOException, " << e.what() << std::endl;
    }

    return m_serial.isOpen();
}

bool Comm6W485::isConnect()
{
    return m_serial.isOpen();
}

std::string Comm6W485::getPort()
{
    return m_port;
}

bool Comm6W485::setPort(std::string port)
{
    m_port = port;
    return true;
}

bool Comm6W485::setBaudrate(unsigned int baudrate)
{
    m_baudrate = baudrate;
    return true;
}

unsigned int Comm6W485::getBaudrate()
{
    return m_baudrate;
}

bool Comm6W485::setTimeout(unsigned int timeout_ms)
{
    m_timeout = timeout_ms;
    return true;
}

unsigned int Comm6W485::getTimeout()
{
    return m_timeout;
}

std::vector<std::string> Comm6W485::getDeviceList()
{
    std::vector<std::string> list_port;

    // Get the device list from the serial librarys
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    // Push each port string to the vector and return
    for (serial::PortInfo portInfo : devices_found)
    {
        list_port.push_back(portInfo.port);
    }

    return list_port;
}

size_t Comm6W485::writeMultiBytes(std::vector<uint8_t> &data)
{
    if (!isConnect())
    {
        throw serial::PortNotOpenedException("writeMultiBytes Error");
    }
    return m_serial.write(data);
}

bool Comm6W485::writeByte(uint8_t data)
{
    if (!isConnect())
    {
        throw serial::PortNotOpenedException("writeByte Error");
    }
    return m_serial.write(&data, 1);
}

std::vector<uint8_t> Comm6W485::readMultiBytes(size_t count)
{
    // Clear buffer first
    _buffer.clear();

    if (!isConnect())
    {
        throw serial::PortNotOpenedException("readMultiBytes Error");
    }
    m_serial.read(_buffer, count);

    return _buffer;
}

uint8_t Comm6W485::readByte()
{
    if (!isConnect())
    {
        throw serial::PortNotOpenedException("readByte Error");
    }
    uint8_t data;
    m_serial.read(&data, 1);

    // Return received data
    return data;
}