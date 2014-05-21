/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#include "CStereoOdometryApp.h"

int main(int argc ,char * argv[])
{
	//here we do some command line parsing...
	//mission file could be first free parameter
	//mission name can be the  second free parameter
	MOOS::CommandLineParser P(argc,argv);
	std::string mission_file = P.GetFreeParameter(0, "Mission.moos");
	std::string app_name = P.GetFreeParameter(1, "StereoOdometry_libviso");

    CStereoOdometryApp TheApp;
    TheApp.Run(app_name,mission_file,argc,argv);
    return 0;
}
