#ifndef _H_IRQHANDLE
#define _H_IRQHANDLE

#include <stdint.h>
#include "commif.hpp"

typedef void (*IRQFunc) (CommInterface*);

class IRQHandle {
public:
	IRQHandle();
	IRQHandle(int id);
	IRQHandle(int id, IRQFunc func_cp);
	~IRQHandle();

	void registerCallback(IRQFunc func_cb);
	void callback(CommInterface *p_comm);

	int getID();
	void setID(int id);

protected:
	
	int ID;

	// The callback function pointer
	IRQFunc p_IRQFunc;
};

#endif