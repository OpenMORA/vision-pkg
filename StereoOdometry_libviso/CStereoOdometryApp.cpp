/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Stereo visual odometry using libviso2  
  *
  */

#include "CStereoOdometryApp.h"

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

CStereoOdometryApp::CStereoOdometryApp() : 
	m_log_deadreckon(false),
	m_viso(NULL),
	m_last_odo_incr_was_almost_null(false),
	m_new_kf_min_xyz(0.15),
	m_new_kf_min_ang(DEG2RAD(15)),
	m_forze_z0(false)
{
}

CStereoOdometryApp::~CStereoOdometryApp()
{
	if (m_viso) { delete m_viso; m_viso=NULL; }
}

bool CStereoOdometryApp::OnStartUp()
{
	if (!CBaseStereoOdometryApp::OnStartUp())
		return false;

	// If want a different mode than standard: 
	// - REGULAR_ITERATE_AND_MAIL
	// - COMMS_DRIVEN_ITERATE_AND_MAIL
	// - REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL
	SetIterateMode(COMMS_DRIVEN_ITERATE_AND_MAIL); 
	
	// Set high working frequencies:
	SetCommsFreq(100);
	SetAppFreq(100);

	//! @moos_param SAVE_ODO (Default=false) Save dead-reckoning from visual odometry to a plain text log file.
	m_log_deadreckon = m_ini.read_bool("","SAVE_ODO", false );

	if (m_log_deadreckon)
	{
		time_t rawtime;
		time (&rawtime);
		struct tm * timeinfo = localtime (&rawtime);
		char buf[200];
		strftime(buf,sizeof(buf),"stereo_odometry_%Y-%m-%d-%H-%M-%S.txt",timeinfo);
		
		m_log_file.open(buf);
		if (!m_log_file.is_open())
		{
			cerr << "**ERROR**: Couldn't open log file: " << buf << endl;
		}
		else
		{
			// Write log header:
			m_log_file << "% Stereo odometry log file\n"
				"%  TIMESTAMP   X       Y      Z      YAW (rad)   PITCH (rad)     ROLL (rad)\n"
				"% -------------------------------------------------------------------------\n";
		}
	}

	//! @moos_param FORZE_Z0 (Default=false) Forze Z coordinate to be always exactly zero.
	m_forze_z0 = m_ini.read_bool("","FORZE_Z0", false );
	
	//! @moos_param NEW_KF_MIN_XYZ (Default=0.25) Minimum linear distance (in meters) between reference KeyFrames.
	m_new_kf_min_xyz = m_ini.read_double("","NEW_KF_MIN_XYZ", m_new_kf_min_xyz);

	//! @moos_param NEW_KF_MIN_ANG (Default=15) Minimum angle (degrees) between reference KeyFrames.
	m_new_kf_min_ang = DEG2RAD( m_ini.read_double("","NEW_KF_MIN_ANG", RAD2DEG(m_new_kf_min_ang) ) );

	return DoRegistrations();
}

// The main method, where visual odometry happens:
void CStereoOdometryApp::processVisualStereoOdometry( const mrpt::slam::CObservationStereoImagesPtr &obs)
{
	ASSERT_(obs.present()) // Just in case
	
	// Upon first execution, create the VISO object with the correct parameters:
	if (!m_viso)
	{
		const mrpt::utils::TCamera & cam_l = obs->leftCamera;
		const mrpt::utils::TCamera & cam_r = obs->rightCamera;

		// Sanity checks:
		ASSERTMSG_( 
			std::abs(cam_l.fx() - cam_r.fx())<1e-6 && std::abs(cam_l.fy() - cam_r.fy())<1e-6 && 
			std::abs(cam_l.cx() - cam_r.cx())<1e-6 && std::abs(cam_l.cy() - cam_r.cy())<1e-6 && 
			std::abs(cam_l.fx() - cam_l.fy())<1e-6 && 
			cam_l.dist[0]==0 && cam_l.dist[1]==0 && cam_l.dist[2]==0 
			, "ERROR: This module assumes rectified images with coinciding image centers!!")
		
		ASSERTMSG_( obs->rightCameraPose.x()>0, "ERROR: Stereo baseline must be positive!!")

		// calibration parameters:
		VisualOdometryStereo::parameters param;
		param.calib.f  = cam_l.fx(); // focal length in pixels
		param.calib.cu = cam_l.cx(); // principal point (u-coordinate) in pixels
		param.calib.cv = cam_l.cy(); // principal point (v-coordinate) in pixels

		param.match.f  = param.calib.f;
		param.match.cu = param.calib.cu;
		param.match.cv = param.calib.cv;

		param.base     = obs->rightCameraPose.x(); // baseline in meters

		param.match.match_disp_tolerance = 3; // 2; // vertical tolerance in stereo
		param.bucket.max_features  = 30;  // 2
		param.bucket.bucket_width  = 50; 
		param.bucket.bucket_height = 50; 


		param.match.nms_tau = 5; // 50; 
		param.match.nms_n   = 5; 

		param.match.half_resolution = 1;
		param.match.outlier_flow_tolerance = 5; // 5; 
		param.match.outlier_disp_tolerance = 5; // 5; 

		param.reweighting  = 1 ;  // 1 
  

		MOOSTrace("Init VISO: image size=(%ux%u)\n",  static_cast<unsigned int>(cam_l.ncols),static_cast<unsigned int>(cam_l.nrows) );

		// init visual odometry
		m_viso = new VisualOdometryStereo(param);
	}

	// Stereo odometry processing:
	obs->imageLeft.forceLoad();  obs->imageRight.forceLoad();

	const size_t width_l = obs->imageLeft.getWidth(), height_l = obs->imageLeft.getHeight();
	const size_t width_r = obs->imageRight.getWidth(), height_r = obs->imageRight.getHeight();
	ASSERT_EQUAL_(width_l,width_r)
	ASSERT_EQUAL_(height_l,height_r)

	const mrpt::utils::CImage left_gray (obs->imageLeft,  mrpt::utils::FAST_REF_OR_CONVERT_TO_GRAY );
	const mrpt::utils::CImage right_gray(obs->imageRight, mrpt::utils::FAST_REF_OR_CONVERT_TO_GRAY );

	int32_t dims[3] = { left_gray.getWidth(), left_gray.getHeight(), left_gray.getRowStride() };
	
	bool img_buf_replace = m_last_odo_incr_was_almost_null;

	cout << "["<< mrpt::system::timestampTotime_t( obs->timestamp )  <<"] Computing stereo odometry...\n";
	if (m_viso->process( left_gray(0,0), right_gray(0,0), dims, img_buf_replace ))
	{
		// on success, update current pose
		// [rx ry rz tx ty tz]
		const std::vector<double> & tr = m_viso->getMotionVec();
		const mrpt::poses::CPose3D cam_incr = -mrpt::poses::CPose3D(tr[3],tr[4],tr[5], tr[0],tr[1],tr[2]);
		const mrpt::math::CMatrixDouble44 cam_incr_HM =  cam_incr.getHomogeneousMatrixVal();

		m_last_odo_incr_was_almost_null = 
			cam_incr.norm() < m_new_kf_min_xyz
			&& 
			mrpt::utils::max3( std::abs(cam_incr.yaw()),std::abs(cam_incr.pitch()),std::abs(cam_incr.roll() ) ) < m_new_kf_min_ang;

		// Relative pose from the car origin to the camera:
		const mrpt::math::CMatrixDouble44 car2cam_HM = obs->cameraPose.getHomogeneousMatrixVal();

		// Homework: Think about where this formula comes from ;-)
		const mrpt::math::CMatrixDouble44 odo_incr_HM = (car2cam_HM * cam_incr_HM) * car2cam_HM.transpose();
		mrpt::poses::CPose3D odo_incr( odo_incr_HM ); 

		// Accumulate current pose:
		mrpt::poses::CPose3D current_kf;
		current_kf.composeFrom(m_odometry_accum_pose,odo_incr);
		
		if (m_forze_z0)
			current_kf.z(0);
		      
		if (!m_last_odo_incr_was_almost_null) {
			m_odometry_accum_pose = current_kf;
		}

		if (m_log_file.is_open())
		{
			m_log_file << 
				mrpt::format(" %16.04f %10.05f %10.05f %10.05f %10.05f %10.05f %10.05f\n",
				mrpt::system::timestampToDouble(obs->timestamp),
				current_kf.x(), current_kf.y(), current_kf.z(),
				current_kf.yaw(), current_kf.pitch(), current_kf.roll() 
				);
		}
		
		bool SHOW_GUI = true;

		if (SHOW_GUI)
		{
			if (!m_gui)
			{
				m_gui = mrpt::gui::CDisplayWindow3DPtr( new mrpt::gui::CDisplayWindow3D("Stereo odometry", 1024,800) );
				m_gui->setPos(0,0);
				mrpt::opengl::COpenGLScenePtr &scene = m_gui->get3DSceneAndLock();

				scene->createViewport("view_l")->setViewportPosition(0,0.5,0.5,0.5);
				scene->createViewport("view_r")->setViewportPosition(0.5,0.5,0.5,0.5);
				scene->getViewport("main")->setViewportPosition(0,0,1.0,0.5);

				scene->insert( mrpt::opengl::CGridPlaneXY::Create(-100,500,-500,500,0,10 ) );

				m_gui->unlockAccess3DScene();
				m_gui->setCameraPointingToPoint(60,0,0);
				m_gui->setCameraElevationDeg(85);
				m_gui->setCameraAzimuthDeg(-90);
				m_gui->setCameraZoom(80);
			}

			// GUI text labels:
			m_gui->addTextMessage(
				5,5, 
				mrpt::format("Matches: %4i Inliers: %5.02f%%",(int)m_viso->getNumberOfMatches(), 100.0*m_viso->getNumberOfInliers()/m_viso->getNumberOfMatches() ), 
				mrpt::utils::TColorf(1,1,0), "mono",12, mrpt::opengl::FILL, 0 );

			m_gui->addTextMessage(
				5,25, 
				mrpt::format("Odo incr: Ax=%+7.04f Ay=%+7.04f Az=%+7.04f Athz=%+4.02f Athy=%+4.02f Athx=%+4.02f",odo_incr.x(),odo_incr.y(),odo_incr.z(), RAD2DEG(odo_incr.yaw()),RAD2DEG(odo_incr.pitch()),RAD2DEG(odo_incr.roll()) ), 
				mrpt::utils::TColorf(1,1,0), "mono",12, mrpt::opengl::FILL, 1 );

			m_gui->addTextMessage(
				5,45, 
				mrpt::format("Dead reckon: x=%+7.04f y=%+7.04f z=%+7.04f thz=%+4.02f thy=%+4.02f thx=%+4.02f", current_kf.x(),current_kf.y(),current_kf.z(), RAD2DEG(current_kf.yaw()),RAD2DEG(current_kf.pitch()),RAD2DEG(current_kf.roll()) ), 
				mrpt::utils::TColorf(1,1,0), "mono",12, mrpt::opengl::FILL, 2 );
			
			m_gui->addTextMessage(
				5,65, 
				mrpt::format("%s", (m_last_odo_incr_was_almost_null ? "REUSE KF":"NEW KF") ), 
				mrpt::utils::TColorf(m_last_odo_incr_was_almost_null ? 1:0,0, m_last_odo_incr_was_almost_null ? 0:1), "mono",12, mrpt::opengl::FILL, 3 );

			// Create visualization:
			mrpt::utils::CImage vis_l, vis_r;
			obs->imageLeft.colorImage(vis_l); 
			if (vis_l.isExternallyStored()) vis_l.loadFromFile( vis_l.getExternalStorageFileAbsolutePath() );
			obs->imageRight.colorImage(vis_r);
			if (vis_r.isExternallyStored()) vis_r.loadFromFile( vis_r.getExternalStorageFileAbsolutePath() );

			// Visualize matches:
			const std::vector<Matcher::p_match> visualMatches = m_viso->getMatches();

			for (size_t i=0;i<visualMatches.size();i++)
			{
				const Matcher::p_match & m = visualMatches[i];
				
				vis_l.line(
					m.u1p, m.v1p, // from
					m.u1c, m.v1c, // to
					mrpt::utils::TColor::white, 1 /*width*/ );
				vis_r.line(
					m.u2p, m.v2p, // from
					m.u2c, m.v2c, // to
					mrpt::utils::TColor::white, 1 /*width*/ );
			}

			mrpt::opengl::COpenGLScenePtr &scene = m_gui->get3DSceneAndLock();
			mrpt::opengl::COpenGLViewportPtr gl_main  = scene->getViewport("main");
			mrpt::opengl::COpenGLViewportPtr gl_left  = scene->getViewport("view_l");
			mrpt::opengl::COpenGLViewportPtr gl_right = scene->getViewport("view_r");
			gl_left->setImageView_fast(vis_l);
			gl_right->setImageView_fast(vis_r);
			
			if (!m_last_odo_incr_was_almost_null)
			{
				mrpt::opengl::CSetOfObjectsPtr gl_obj = mrpt::opengl::stock_objects::CornerXYZSimple();
				gl_obj->setPose(m_odometry_accum_pose);
				gl_main->insert(gl_obj);
			}
			m_gui->unlockAccess3DScene();

			m_gui->repaint();					
		} // end if GUI

	} 
	else 
	{
		cout << " ... failed!" << endl;
	}

}
