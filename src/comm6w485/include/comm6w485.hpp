#ifndef _H_COMM6W485
#define _H_COMM6W485

#include "serial/serial.h"
#include "hidapi.h"
#include "irqmanager.hpp"
#include <vector>
#include "commif.hpp"

class Comm6W485 : public CommInterface, IRQInterface
{
public:
	enum class InterfaceType
	{
		INTERFACE_CDC,
		INTERFACE_HID,
	};
public:
  Comm6W485();
  ~Comm6W485();

  /* Control method */
  bool connect();
  bool isConnect();

  /* read and write method */

  // Serial I/O method
  size_t writeMultiBytes(std::vector<uint8_t> &data);
  bool writeByte(uint8_t data);

  std::vector<uint8_t> readMultiBytes(size_t count);
  uint8_t readByte();

  // HID I/O method
  bool getISR();
  void sendIRQ();

  /* Configure method */
  std::string getPort();
  bool setPort(std::string port);

  bool setBaudrate(unsigned int baudrate);
  unsigned int getBaudrate();

  bool setTimeout(unsigned int timeout_ms);
  unsigned int getTimeout();

  std::vector<std::string> getDeviceList(InterfaceType type);

protected:
  serial::Serial m_serial;
  hid_device* m_hid;

private:
  unsigned int m_baudrate;
  std::string m_port;
  unsigned int m_timeout;

  // IO buffer
  std::vector<uint8_t> _buffer;

  // IRQ manager
  IRQManager m_irqmanager;
};

#endif // !_H_COMM6W485