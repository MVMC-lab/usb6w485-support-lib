#ifndef _H_COMM6W485
#define _H_COMM6W485

#include "serial/serial.h"
#include "hidapi.h"
#include "irqmanager.hpp"
#include <vector>
#include "commif.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

class Comm6W485 : public CommInterface, public IRQInterface
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

  boost::shared_ptr<wchar_t[]> getSerialcode();
  bool setSerialcode(int index);
  


  bool setBaudrate(unsigned int baudrate);
  unsigned int getBaudrate();

  bool setTimeout(unsigned int timeout_ms);
  unsigned int getTimeout();

  std::vector<std::string> getDeviceList(InterfaceType type);

  IRQManager& getIRQManger();

protected:
  bool updateDeviceList(InterfaceType type);

protected:
  serial::Serial m_serial;
  hid_device* m_hid;

  std::vector<std::string> m_serial_port_list;
  std::vector<boost::shared_ptr<wchar_t[]>> m_hid_dev_list;


private:
  unsigned int m_baudrate;
  std::string m_port;
  boost::shared_ptr<wchar_t[]> m_target_hid_dev;
  unsigned int m_timeout;

  // IO buffer
  std::vector<uint8_t> _buffer;

  // IRQ manager
  IRQManager m_irqmanager;

  // Resource mutex
  boost::mutex m_mutex;
};

#endif // !_H_COMM6W485