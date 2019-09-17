#include<iostream>
#include<fstream>
#include"system_control.h"
#include"reflect/context.h"
using namespace std;


int main() {
	//int times = 2;
	//while (times)
	{
	context::create_context("config/beans.xml");
	context *__context = context::get_context();

	system_control *__system = (system_control *)__context->get_bean("system_control");

	
		__system->process();
		//cout << times << endl;
		//times--;
	}
	
	system("pause");
}