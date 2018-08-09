// using standard exceptions
#include <iostream>
#include <exception>
using namespace std;


class IRQFuncNotExistException: public exception
{
	virtual const char* what() const;
};

class IRQWTFException : public exception
{
	virtual const char* what() const;
};

