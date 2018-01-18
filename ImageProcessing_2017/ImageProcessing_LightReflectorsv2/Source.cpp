
#include "SmartDashboardUser.h"

#pragma unmanaged

#include <opencv2\opencv.hpp>
#include "Process.h"

int main()
{
	SmartDashboardUser::Run(2630);
	Process::main();
	

	return 0;
}