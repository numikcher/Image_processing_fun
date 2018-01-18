#ifndef SMART_DASHBOARD_USER
#define SMART_DASHBOARD_USER

#pragma once

/*
This class created for comunication with the roboRio .NET dll.
@author Orr Shachaff
*/


// cli dlls.
#using <mscorlib.dll>
#using <System.dll>

// headers for cli -> native
#include <iostream>
#include <string>
#include <msclr\marshal_cppstd.h>

using namespace System;
using namespace System::Threading;
class SmartDashboardUser
{

public:
	// static constructor
	SmartDashboardUser();
	~SmartDashboardUser();
	static void Run(int p_team);
	static void Set(std::string p_str, std::string p_value);
	static std::string Get(std::string p_key);
	static bool ContainsKey(std::string p_key);


private:

};


#endif // !SMART_DASHBOARD_USER