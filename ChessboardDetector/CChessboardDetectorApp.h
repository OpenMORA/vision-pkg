/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CChessboardDetectorApp_H
#define CChessboardDetectorApp_H

#include <COpenMORAMOOSApp.h>
#include <mrpt/slam/CObservationStereoImages.h>
#include <mrpt/gui/CDisplayWindow3D.h>

class CChessboardDetectorApp : public COpenMORAApp
{
public:
    CChessboardDetectorApp();
    virtual ~CChessboardDetectorApp();

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

	// DATA

	std::string m_stereo_input_var_name;
	int m_chessboard_nx,m_chessboard_ny;
	double m_chessboard_lx,m_chessboard_ly; //!< In meters

	mrpt::gui::CDisplayWindow3DPtr m_gui;

	void processStereoImage( const mrpt::slam::CObservationStereoImagesPtr &obs);

};
#endif
