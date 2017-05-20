#ifndef _MODULE_H_HSJ_
#define _MODULE_H_HSJ_ 

#include <iostream>
#include <utility>

template <typename Func,
		  typename... Args>
void moduleCall(const std::string& message,
				std::ostream& out,
				Func functor,
				Args&&... args)
{
	out<<"================"<<std::endl;
	out<<message<<std::endl;
	functor(std::forward<Args>(args)...);
	out<<"END"<<std::endl;
}

#endif