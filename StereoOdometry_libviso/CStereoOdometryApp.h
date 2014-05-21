/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CStereoOdometryApp_H
#define CStereoOdometryApp_H

#include "../StereoOdometry_common/CBaseStereoOdometryApp.h"

#include <viso_stereo.h>

class CStereoOdometryApp : public CBaseStereoOdometryApp
{
public:
    CStereoOdometryApp();
    virtual ~CStereoOdometryApp();

protected:
	/** called at startup */
	virtual bool OnStartUp();


	// Data:
	bool         m_log_deadreckon; //!< Save accumulated dead-reckoning
	std::ofstream  m_log_file;     //!< If m_log_deadreckon=true
	mrpt::poses::CPose3D m_odometry_accum_pose;


	VisualOdometryStereo  *m_viso;
	bool                   m_last_odo_incr_was_almost_null; //!< used to know if we should replace the "previous" img buffer

	double  m_new_kf_min_xyz, m_new_kf_min_ang; //!< Minimum dist. and angle to insert a new KF

	bool    m_forze_z0;  

	mrpt::gui::CDisplayWindow3DPtr m_gui;

	// Do the real work with each new image:
	void processVisualStereoOdometry(const mrpt::slam::CObservationStereoImagesPtr &obs);

};
#endif
