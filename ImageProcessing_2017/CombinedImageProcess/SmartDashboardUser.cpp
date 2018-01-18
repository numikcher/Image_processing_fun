#include "SmartDashboardUser.h"
using namespace std;
using namespace SmartDashboard;


SmartDashboardUser::SmartDashboardUser()
{
}


SmartDashboardUser::~SmartDashboardUser()
{
}

/*
	Attach to roborio.
	@param int team number	
*/
void SmartDashboardUser::Run(int p_team)
{
	Dashboard::Run(p_team, RobotType::roboRIO);
	Thread::Sleep(2000);
}

/*
	Set value in SmrtDashboard
	@param std::string key 
	@param std::string value
*/
void SmartDashboardUser::Set(std::string p_key, std::string p_value)
{
	String ^key = gcnew String(p_key.c_str());
	String ^value = gcnew String(p_value.c_str());
	Dashboard::Set(key, value);

}

/*
*	Return if SmartDashboard contains key.
*	@param key.
*	@return bool value
*/
bool SmartDashboardUser::ContainsKey(std::string p_key)
{
	String ^key = gcnew String(p_key.c_str());
	return Dashboard::ContainsKey(key);
}
/*
	Get value in SmartDashboard by key.
	@param std::string p_key
	@return std::string value.
*/
std::string SmartDashboardUser::Get(std::string p_key)
{
	String ^ key = gcnew String(p_key.c_str());
	String ^returned = "null";
	try 
	{
		 returned = Dashboard::Get(key);
	}
	catch (Exception ^p)
	{
	}
	if (returned == nullptr)
	{
		returned = "null";
	}

	return (msclr::interop::marshal_as<std::string>(returned));
}


