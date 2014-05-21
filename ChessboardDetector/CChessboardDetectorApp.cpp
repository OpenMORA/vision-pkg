/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Detects chessboards in input stereo images */

#include "CChessboardDetectorApp.h"

#include <mrpt/vision/chessboard_find_corners.h>
//#include <mrpt/slam/CObservation6DFeatures.h>
#include <mrpt/slam/CObservationStereoImages.h>
#include <mrpt/opengl/CGridPlaneXY.h>

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;


const string STEREO_INPUT_NAME = string("STEREO_INPUT");


CChessboardDetectorApp::CChessboardDetectorApp()  :
	m_stereo_input_var_name("STEREO1_OBS"),
	m_chessboard_nx(1),
	m_chessboard_ny(1),
	m_chessboard_lx(0.1),m_chessboard_ly(0.1)
{
}

CChessboardDetectorApp::~CChessboardDetectorApp()
{
}

bool CChessboardDetectorApp::OnStartUp()
{
	try
	{

	// If want a different mode than standard:
	// - REGULAR_ITERATE_AND_MAIL
	// - COMMS_DRIVEN_ITERATE_AND_MAIL
	// - REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL
	SetIterateMode(COMMS_DRIVEN_ITERATE_AND_MAIL);

	//! @moos_param STEREO_INPUT (Default="STEREO1_OBS") The name of the variable with the input stream of stereo images (they must be rectified)
	m_MissionReader.GetConfigurationParam("STEREO_INPUT",m_stereo_input_var_name);

	//! @moos_param CHESSBOARD_NX (Mandatory) Chessboard params: number of corners in X
	m_chessboard_nx = m_ini.read_int("", "CHESSBOARD_NX", 0, true /*mandatory*/ );

	//! @moos_param CHESSBOARD_NY (Mandatory) Chessboard params: number of corners in Y
	m_chessboard_ny = m_ini.read_int("", "CHESSBOARD_NY", 0, true /*mandatory*/ );

	//! @moos_param CHESSBOARD_LX (Mandatory) Chessboard params: distance between squares in X (in meters)
	m_chessboard_lx= m_ini.read_double("", "CHESSBOARD_LX", .0, true /*mandatory*/ );

	//! @moos_param CHESSBOARD_LY (Mandatory) Chessboard params: distance between squares in X (in meters)
	m_chessboard_ly= m_ini.read_double("", "CHESSBOARD_LY", .0, true /*mandatory*/ );


	//! @moos_subscribe	STEREO1_OBS
	//! @moos_var STEREO1_OBS Stereo images as mrpt::slam::CObservationStereoImages objects
	AddMOOSVariable(STEREO_INPUT_NAME /*var name*/, m_stereo_input_var_name /*real var name*/,m_stereo_input_var_name, 0.01 /*timeout*/);

	//! @moos_subscribe	MORA_IMAGES_DIR
	AddMOOSVariable_OpenMORA("MORA_IMAGES_DIR",.0);

	return DoRegistrations();
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		this->RequestQuit();
		return false;
	}
}

bool CChessboardDetectorApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

bool CChessboardDetectorApp::Iterate()
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
			mrpt::slam::CObservationStereoImagesPtr obsStereo = mrpt::slam::CObservationStereoImagesPtr(obj);
			// Do stereo odometry:
			try
			{
				this->processStereoImage(obsStereo);
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

bool CChessboardDetectorApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CChessboardDetectorApp::DoRegistrations()
{

	RegisterMOOSVariables();
	RegisterMOOSVariables_OpenMORA();
	return true;
}


bool CChessboardDetectorApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	UpdateMOOSVariables(NewMail);
	UpdateMOOSVariables_OpenMORA(NewMail);
	return true;
}


void CChessboardDetectorApp::processStereoImage( const mrpt::slam::CObservationStereoImagesPtr &obs)
{
	// Detect multiple-checkerboards:
	vector<vector<TPixelCoordf> > 	listCornerCoords[2];  // 0:L, 1:R

	// L & R loop:
	CImage img_detected[2];
	for (int i=0;i<2;i++)
	{
		const CImage & src = (i==0) ? obs->imageLeft : obs->imageRight;

		src.forceLoad();
		src.colorImage(img_detected[i]);
		img_detected[i].forceLoad();

		// Detect:
		mrpt::vision::findMultipleChessboardsCorners(
			img_detected[i],
			listCornerCoords[i],
			m_chessboard_nx,m_chessboard_ny );

		cout << "Img " << i << " board " << m_chessboard_nx << "x" << m_chessboard_ny << " -> " << listCornerCoords[i].size() <<  " boards detected." << endl;
		// Draw:
		for (size_t k=0;k<listCornerCoords[i].size();k++)
			img_detected[i].drawChessboardCorners(listCornerCoords[i][k],m_chessboard_nx,m_chessboard_ny, 1, 0 );
	}


	bool SHOW_GUI = true;

	if (SHOW_GUI)
	{
		if (!m_gui)
		{
			m_gui = mrpt::gui::CDisplayWindow3DPtr( new mrpt::gui::CDisplayWindow3D("Chessboard detection odometry", 1024,800) );
			//m_gui->setPos(0,0);
			mrpt::opengl::COpenGLScenePtr &scene = m_gui->get3DSceneAndLock();

			scene->createViewport("view_l")->setViewportPosition(0,0,1,1); // (0,0.5,0.5,0.5);
			scene->createViewport("view_r")->setViewportPosition(0.5,0.5,0.5,0.5);
			scene->getViewport("main")->setViewportPosition(0,0,1.0,0.5);

			scene->insert( mrpt::opengl::CGridPlaneXY::Create(-100,500,-500,500,0,10 ) );

			m_gui->unlockAccess3DScene();
			m_gui->setCameraPointingToPoint(60,0,0);
			m_gui->setCameraElevationDeg(85);
			m_gui->setCameraAzimuthDeg(-90);
			m_gui->setCameraZoom(80);
		}

		mrpt::opengl::COpenGLScenePtr &scene = m_gui->get3DSceneAndLock();
		mrpt::opengl::COpenGLViewportPtr gl_main  = scene->getViewport("main");
		mrpt::opengl::COpenGLViewportPtr gl_left  = scene->getViewport("view_l");
		mrpt::opengl::COpenGLViewportPtr gl_right = scene->getViewport("view_r");
		
		gl_left->setImageView_fast(img_detected[0]);
		gl_right->setImageView_fast(img_detected[1]);

		m_gui->unlockAccess3DScene();

		m_gui->repaint();
	} // end if GUI


}
