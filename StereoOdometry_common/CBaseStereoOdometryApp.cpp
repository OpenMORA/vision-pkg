/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Stereo visual odometry using libviso2  
  *
  */

#include "CBaseStereoOdometryApp.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <time.h>       /* time_t, struct tm, time, localtime, strftime */

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;

const string STEREO_INPUT_NAME = string("STEREO_INPUT");

CBaseStereoOdometryApp::CBaseStereoOdometryApp() : 
	m_stereo_input_var_name("STEREO_RECT_OBS")
{
	// Why the compiler removes the reference to the automatic registration?? (JLBC @ MAY-2014)
	using namespace mrpt::obs;
	mrpt::utils::registerClass( CLASS_ID(CObservationStereoImages) );
}

CBaseStereoOdometryApp::~CBaseStereoOdometryApp()
{
}

bool CBaseStereoOdometryApp::OnStartUp()
{
	//! @moos_param STEREO_RECT_VARNAME (Default="STEREO_RECT_OBS") The name of the variable with the input stream of stereo images (they must be rectified)
	m_MissionReader.GetConfigurationParam("STEREO_RECT_VARNAME",m_stereo_input_var_name);

	//! @moos_subscribe	STEREO_RECT_OBS
	//! @moos_var STEREO_RECT_OBS Stereo images, after rectification, as mrpt::obs::CObservationStereoImages objects
	AddMOOSVariable(STEREO_INPUT_NAME /*var name*/, m_stereo_input_var_name /*real var name*/,m_stereo_input_var_name, 0.05 /*timeout*/); 

	//! @moos_subscribe	MORA_IMAGES_DIR
	AddMOOSVariable_OpenMORA("MORA_IMAGES_DIR",.0);

	return DoRegistrations();
}

bool CBaseStereoOdometryApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

// Main module loop code:
bool CBaseStereoOdometryApp::Iterate()
{
	// Detect external imgs dir:
	{
		CMOOSVariable *pVar = GetMOOSVar_OpenMORA("MORA_IMAGES_DIR");
		if(pVar && pVar->IsFresh())
		{
			pVar->SetFresh(false);
			mrpt::utils::CImage::IMAGES_PATH_BASE = pVar->GetStringVal();

			MOOSTrace("Setting external images dir to: %s\n",mrpt::utils::CImage::IMAGES_PATH_BASE.c_str());
		}
	}
	
	// new images?
	{
		CMOOSVariable *pVar = GetMOOSVar(STEREO_INPUT_NAME);
		if(pVar && pVar->IsFresh())
		{
			pVar->SetFresh(false);
			
			// Deserialize:
			mrpt::utils::CSerializablePtr obj = this->MOOS2MRPT_deserialize(*pVar);
			ASSERT_(obj.present())
			// This will launch an exception if types don't match:
			mrpt::obs::CObservationStereoImagesPtr obsStereo = mrpt::obs::CObservationStereoImagesPtr(obj);
			// Do stereo odometry:
			try
			{
				this->processVisualStereoOdometry(obsStereo);
			}
			catch (std::exception &e)
			{
				std::cerr << e.what() << std::endl;
				return false;
			}

		}
	}
	return true;
}

bool CBaseStereoOdometryApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CBaseStereoOdometryApp::DoRegistrations()
{
	RegisterMOOSVariables();
	RegisterMOOSVariables_OpenMORA();
	return true;
}


bool CBaseStereoOdometryApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	UpdateMOOSVariables(NewMail);
	UpdateMOOSVariables_OpenMORA(NewMail);
	return true;
}

