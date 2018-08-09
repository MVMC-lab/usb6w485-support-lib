#ifndef _H_COMMIF
#define _H_COMMIF
#include <vector>
#include <stdint.h>

class CommInterface {
public:
	virtual size_t writeMultiBytes(std::vector<uint8_t> &data) = 0;
	virtual bool writeByte(uint8_t data) = 0;

	virtual std::vector<uint8_t> readMultiBytes(size_t count) = 0;
	virtual uint8_t readByte() = 0;
};

class IRQInterface {
public:
	virtual bool getISR() = 0;
	virtual void sendIRQ() = 0;

};

#endif // _H_COMMIF