#include <iostream>
#include "comm6w485.hpp"
#ifdef WIN32
#include <Windows.h>
#endif

#include <boost/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

using namespace std;

Comm6W485 comm_mst;
Comm6W485 comm_slv;


void isr_thread(IRQManager& manager) {
    while(true) {
        manager.spinOnce();
        //boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }
}

void irq_thread(IRQManager &manager)
{
    while (true)
    {
        manager.spinOnce();
        //boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }
}
vector<boost::chrono::high_resolution_clock::time_point> time_stamp;
void slave_rsp_cb(CommInterface *comm);

void select_device(Comm6W485 &comm)
{
    int count(0);

    cout << " ---- HID Device list ---- " << endl;
    vector<string> hid_list = comm.getDeviceList(Comm6W485::InterfaceType::INTERFACE_HID);

    count = (0);
    for (string info : hid_list ) {
        cout << "[" << count++ << "]: " << info << endl;
    }
    cout << "Select Dev : [0 ~ " << hid_list.size() - 1 << "] " << endl;
    int dev_select;
    cin >> dev_select;
    comm.setSerialcode(dev_select);

    cout << endl << " ---- Serial Port list ---- " << endl;
    vector<string> list = comm.getDeviceList(Comm6W485::InterfaceType::INTERFACE_CDC);

    count = (0);
    for (string info : list)
    {
        cout << "[" << count++ << "]: " << info << endl;
    }
    cout << "Select port: [0 ~ " << list.size() - 1 << "] " << endl;

    int select(0);
    cin >> select;
    comm.setBaudrate(38400);
    comm.setPort(list[select]);
    comm.setTimeout(10);
}

namespace pt = boost::property_tree;

int main(int argc, char const *argv[])
{

    // Select master Device
    select_device(comm_mst);

    // Select slave Device
    select_device(comm_slv);

    if (comm_mst.connect())
    {
        cout << "  ---- Master connect ----" << endl;
        cout << "connect " << comm_mst.getPort() << " successful! " << endl;
        cout << "connect ";
        wcout << comm_mst.getSerialcode();
        cout << " successful! " << endl;
    }

    if (comm_slv.connect()) {
        cout << "  ---- Slave connect ----" << endl;
        cout << "connect " << comm_slv.getPort() << " successful! " << endl;
        cout << "connect ";
        wcout << comm_slv.getSerialcode();
        cout << " successful! " << endl;
    }

    // for (uint8_t c : comm.readMultiBytes(100))
    // {
    //     cout << c;
    // }
    // cout << endl;

    IRQManager manager_mst = comm_mst.getIRQManger();
    IRQManager manager_slv = comm_slv.getIRQManger();

    IRQHandle irq1(0);

    irq1.registerCallback(
        [](CommInterface *comm) {
            vector<uint8_t> test_message;
            test_message.push_back('t');
            test_message.push_back('e');
            test_message.push_back('s');
            test_message.push_back('t');
            test_message.push_back('\r');
            test_message.push_back('\n');
            comm->writeMultiBytes(test_message);
            cout << "irq1 call!" << endl;
        });

    IRQHandle irq2(3);

    irq2.registerCallback( slave_rsp_cb);

    manager_slv.registerIRQHandle(irq1);
    manager_slv.registerIRQHandle(irq2);

    IRQHandle irq_response(0);
    irq_response.registerCallback(
        [](CommInterface *comm) {
            cout << "Master ISR called!" << endl;
            cout << "Data in ------" << endl;
            for(int i=0; i<10; i++) {
                char input = comm->readByte();
                if(input == '\n') {
                    cout << endl;
                    break;
                }
                cout << input;
            }
            cout << "------------" << endl;
            
        });

    manager_mst.registerIRQHandle(irq_response);

    boost::thread th_isr(isr_thread, manager_mst);
    boost::thread th_irq(irq_thread, manager_slv);

    // Execute IRQ sending test
    for (int i = 0; i < 100; i++)
    {
        comm_mst.sendIRQ();
        cout << "Count: " << i+1 << endl;
        boost::this_thread::sleep_for(boost::chrono::milliseconds(99));
    }

    for(int i=1; i < time_stamp.size(); i++) {
        boost::chrono::nanoseconds ns = boost::chrono::duration_cast<boost::chrono::nanoseconds> (time_stamp.at(i) - time_stamp.at(i-1));
        cout << "Time stamp: " <<  ns.count() / 1000000.0 << endl;
    }

    return 0;
}

void slave_rsp_cb(CommInterface *comm)
{
    cout << "ISR get and Send IRQl!" << endl;
    comm_slv.sendIRQ();
    time_stamp.push_back(boost::chrono::high_resolution_clock::now());
}