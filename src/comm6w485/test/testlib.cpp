#include <iostream>
#include "comm6w485.hpp"
using namespace std;

int main(int argc, char const *argv[])
{
    Comm6W485 comm;

    vector<string> list = comm.getDeviceList();

    int count(0);
    for (string info : list)
    {
        cout << "[" << count++ << "]: " << info << endl;
    }
    cout << "Select port: [0 ~ " << list.size() - 1 << "] " << endl;

    int select(0);
    cin >> select;
    comm.setBaudrate(38400);
    comm.setPort(list[select]);
    comm.setTimeout(10000);

    if (comm.connect())
    {
        cout << "connect " << list[select] << " successful! " << endl;
    }

    for (uint8_t c : comm.readMultiBytes(100))
    {
        cout << c;
    }
    cout << endl;

    system("pause");
    return 0;
}