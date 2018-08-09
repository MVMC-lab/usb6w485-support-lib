#include "comm6w485.hpp"
#include "hidapi.h"
#include "irqmanager.hpp"
#include "commif.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <boost/thread.hpp>
#include <boost/format.hpp>

Comm6W485::Comm6W485() : m_serial(),
                         m_hid(nullptr),
                         m_baudrate(38400),
                         m_port(""),
                         m_timeout(1000),
                         m_irqmanager(dynamic_cast<CommInterface*>(this), dynamic_cast<IRQInterface*>(this))
{
    // TODO: initial serial library first
}

Comm6W485::~Comm6W485()
{
    if (m_serial.isOpen())
    { // If there is port open, close it
        m_serial.close();
    }
    if (m_hid != nullptr)
    {
        // If there is hid open, close it
        hid_close(m_hid);
    }

    /* Free static HIDAPI objects. */
    hid_exit();
}

bool Comm6W485::connect()
{
    // Makesure there is port close
    if (m_serial.isOpen())
    { // If there is port open, close it
        m_serial.close();
    }

    // Makesure there is hid close
    if (m_hid != nullptr) {
        // If there is HID open, close it
        hid_close(m_hid);
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
        return false;
    }
    catch (const serial::SerialException e)
    {
        std::cerr << "[Comm6W485] Open error, SerialException, " << e.what() << std::endl;
        return false;
    }
    catch (const serial::IOException e)
    {
        std::cerr << "[Comm6W485] Open error, IOException, " << e.what() << std::endl;
        return false;
    }

    m_hid = hid_open(0x0483, 0x5720, NULL);
    if(!m_hid) {
        std::cerr << "[Comm6W485] HID Open error " << std::endl;
        return false;
    }

    // Set the hid_read() function to be non-blocking.
    hid_set_nonblocking(m_hid, 1);

    // If without any error, then successful
    return true;
}

bool Comm6W485::isConnect()
{

    if( m_serial.isOpen() && m_hid !=  nullptr) {
        return true;
    }

    return false;
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

std::vector<std::string> Comm6W485::getDeviceList(Comm6W485::InterfaceType type)
{
    std::vector<std::string> list_port;

    if (type == InterfaceType::INTERFACE_CDC)
    {
        // Get the device list from the serial librarys
        // TODO: neet to port to libusb, use VID&PID to identify the device
        std::vector<serial::PortInfo> devices_found = serial::list_ports();

        // Push each port string to the vector and return
        for (serial::PortInfo portInfo : devices_found)
        {
            list_port.push_back(portInfo.port);
        }

        return list_port;
    }
    else if (type == InterfaceType::INTERFACE_HID)
    {
        // Get the device list from the serial librarys
        if (hid_init())
        {
            std::cerr << "Can't not initialize the HIDAPI library" << std::endl;
            return list_port;
        }
        struct hid_device_info *devs, *cur_dev;
        devs = hid_enumerate(0x0483, 0x5720);

        cur_dev = devs;
        while (cur_dev)
        {
            boost::format fmt("%s_%s, %04hx %04hx, %ls, path:%s");
            fmt % cur_dev->manufacturer_string;
            fmt % cur_dev->product_string;
            fmt % cur_dev->vendor_id;
            fmt % cur_dev->product_id;
            fmt % cur_dev->serial_number;
            fmt % cur_dev->path;

            list_port.push_back(fmt.str());

            cur_dev = cur_dev->next;
        }

        hid_free_enumeration(devs);

        return list_port;
    }
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

bool Comm6W485::getISR() {
    if (!isConnect()) {
        throw serial::PortNotOpenedException("get ISR Error");
    }
    int res = 0;
    uint8_t buf[4] = {0};
    buf[0] = 1; // First byte is report number

    // Read the IN report 1, if there is zero, then not ISR
    hid_read(m_hid, buf, 2);

    if(buf[1] == 0) {
        return false;
    }
    else {
        return true;
    }
}

void Comm6W485::sendIRQ() {
    if( !isConnect()) {
        throw serial::PortNotOpenedException("send IRQ Error");
    }

    int res = 0;
    uint8_t buf[4] = {0};
    buf[0] = 2; // First byte is report number
    buf[1] = 1;
    res = hid_write(m_hid, buf, 2);
    // TODO: check function  result and throw error
}