#ifndef _H_ISRMANAGER
#define _H_ISRMANAGER

#include <vector>

#include "irqhandle.hpp"
#include "commif.hpp"
#include <boost/optional.hpp>


using namespace std;

class IRQManager {

public:

	IRQManager(CommInterface *comm_if, IRQInterface *irq_if);

	~IRQManager();
public:

	void spinOnce();

	boost::optional<IRQHandle> findIRQHandle(int id);

	size_t getCount();

	void registerIRQHandle(IRQHandle& irqhandle);
	vector<IRQHandle> getIRQHandleList();

private:

	vector<IRQHandle> m_IRQHandles;

	CommInterface *p_comm;
	IRQInterface *p_irq;

	//TODO: add thread to polling the interrupt and data
};


#endif // _H_ISRMANAGER