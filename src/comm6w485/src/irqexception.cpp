#include "irqexception.hpp"


const char* IRQFuncNotExistException::what() const throw() {
	return "IRQFunction not register.";
}

const char* IRQWTFException::what() const throw() {
	return "Something wrong here!, I don't know why.";
}