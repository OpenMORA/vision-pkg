/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CBaseStereoOdometryApp_H
#define CBaseStereoOdometryApp_H

#include <COpenMORAMOOSApp.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/poses/CPose3D.h>
#include <fstream>

class CBaseStereoOdometryApp : public COpenMORAApp
{
public:
    CBaseStereoOdometryApp();
    virtual ~CBaseStereoOdometryApp();

protected:
	/** called at startup */
	virtual bool OnStartUp();
	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
	/** called when work is to be done */
	virtual bool Iterate();
	/** called when app connects to DB */
	virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );

	bool DoRegistrations();

	std::string  m_stereo_input_var_name;  //!< MOOS var to subscribe

	// Do the real work with each new image:
	virtual void processVisualStereoOdometry(const mrpt::obs::CObservationStereoImagesPtr &obs) = 0;

};
#endif
