// using standard exceptions
#include <exception>
using namespace std;


class IRQFuncNotExistException: public exception
{
	virtual const char* what() const throw();
};

class IRQWTFException : public exception
{
	virtual const char* what() const throw();
};

