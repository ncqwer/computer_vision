#include <iostream>
#include <string>
#include "../sfm/sfm.h"
using namespace std;

int main(int argc, char const *argv[])
{
	auto func=[](int& i,int& j){
		++i;
		++j;
	};

	string message="add function called";
	int h=1;
	int s=2;
	moduleCall(message,std::cout,func,h,s);
	cout<<"h: "<<h<<"s: "<<s<<std::endl;
	moduleCall(message,std::cout,func,h,s);
	cout<<"h: "<<h<<"s: "<<s<<std::endl;

	return 0;
}