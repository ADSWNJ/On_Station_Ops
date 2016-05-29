// ================================================================================================
//
//	On Station Ops v1.0
//	===================
//
//	Copyright (C) 2014	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	Description:
//
//	Informational and control utility for maneuvering space station modules and cargo. This MFD provides
//	a means to determine position and orientation of a tethered object (e.g. a manipulator arm or a module
//	on the end of a manipulator arm) relative to a free-floating target. Optionally, the MFD can provide
//	autopilot control to handle rotation and translation to a selectable offset. 
//
//	Dependencies:
//
//	RV Orientation - needed for autopilot calibration and units.
//	ModuleMessaging by Enjo - needed for inter-MFD communications.
//
//	Copyright Notice:
//
//	This program is free software: you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	For full licencing terms, pleaserefer to the GNU General Public License
//	(gpl-3_0.txt) distributed with this release, or see
//	http://www.gnu.org/licenses/.
//
//
//	Credits:
//
//	Orbiter Simulator			(c) 2003-2014 Martin Schweiger "Martins"
// 	MFDButtonPage					(c) 2012-2014 Szymon Ender "Enjo"
// 	Module Messaging SDK	(c) 2014 Szymon Ender "Enjo"
//	
//
//	Release History:
//
//	V1.00	Initial Release
// ================================================================================================

#define STRICT
#define ORBITER_MODULE
#include "windows.h"
#undef GetClassName

#include "orbitersdk.h"
#include "OnStationOps.hpp"
#include "OSO_Cores.hpp"
#include "OSO_DialogFunc.hpp"
#include "MFDPersist.hpp"


// =======================================================================
// Global variables

OSO_GCore *g_SC;      // points to the static core, root of all persistence
int g_MFDmode;      // holds the mode identifier for our MFD


// =======================================================================
// API interface

DLLCLBK void InitModule (HINSTANCE hDLL)
{
	static char *name = "OnStationOps";   // MFD mode name
	MFDMODESPECEX spec;
	spec.name = name;
	spec.key = OAPI_KEY_T;                // MFD mode selection key
	spec.context = NULL;
	spec.msgproc = OnStationOps::MsgProc;  // MFD mode callback function

	// Register the new MFD mode with Orbiter
	g_MFDmode = oapiRegisterMFDMode (spec);
}

DLLCLBK void ExitModule (HINSTANCE hDLL)
{
	// Unregister the custom MFD mode when the module is unloaded
	oapiUnregisterMFDMode (g_MFDmode);
}

DLLCLBK void opcPreStep(double SimT,double SimDT,double mjd) {

  if (!g_SC) return;  // Static core not initialized

  g_SC->VC->Update();
}

// ==============================================================
// MFD class implementation
//
// ... derive also from EnjoLib::IDrawsHUD to hook to the HUD drawer interface


// Constructor
OnStationOps::OnStationOps (DWORD w, DWORD h, VESSEL *vessel, UINT mfd)
: MFD2 (w, h, vessel)
{
  if (!g_SC) {										// Find the static global core 
    g_SC = new OSO_GCore;						//  ... if missing, then init the static core (only done once for this addon per run of Orbiter)
  }
  GC = g_SC;

  VC = (OSO_VCore*) GC->P.FindVC(vessel);		// Find the vessel core for our vessel
  if (!VC) {
    VC = new OSO_VCore(vessel,GC);				// ... if missing, then init the vessel core for our vessel (once for this vessel)
    GC->P.AddVC(vessel, VC);
  }
  GC->VC = VC;

  MC = (OSO_MCore*) GC->P.FindMC(mfd);			// Find the MFD core for this MFD position
  if (!MC) {
    MC = new OSO_MCore(mfd,GC);					// ... if missing, init the MFD core once per MFD position (e.g. left, right, extern)
    GC->P.AddMC(mfd, MC);
  }

  LC = (OSO_LCore*) GC->P.FindLC(vessel, mfd);	// Find the local core doe this vessel + mfd combination
  if (!LC) {
    LC = new OSO_LCore(vessel,mfd,GC);			// ... if missing, init the local core once per vessel + mfd position
    GC->P.AddLC(vessel, mfd, LC);
  }

  //
  // Set up core components for this MFD instance
  //

  font = oapiCreateFont (h/25, true, "Fixed", FONT_NORMAL, 0);

}

// ===========================================================================
// Destructor (called any time our MFD goes out of scope ... e.g. on F8 press)
OnStationOps::~OnStationOps ()
{
  return;
}


// ==============================================================
// MFD message parser
int OnStationOps::MsgProc (UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam)
{
	switch (msg) {
	case OAPI_MSG_MFD_OPENED:
		// Our new MFD mode has been selected, so we create the MFD and
		// return a pointer to it.
		return (int)(new OnStationOps (LOWORD(wparam), HIWORD(wparam), (VESSEL*)lparam, mfd));
	}
	return 0;
}



// ==============================================================
// Persistence functions
void OnStationOps::ReadStatus(FILEHANDLE scn) {
  char *line;
  int param = 0;
  float paramf = 0.0;

  while (oapiReadScenario_nextline(scn, line)) {
    if (!_strnicmp(line,"END_MFD",7)) {
      break;
    } else if (!_strnicmp(line, "OSO_FT", 6)) {
			OSO_DialogFunc::clbkFT(nullptr, line+7, LC);
    } else if (!_strnicmp(line, "OSO_TT", 6)) {
			OSO_DialogFunc::clbkTT(nullptr, line+7, LC);
    }  else if (!_strnicmp(line, "OSO_UNITS", 9)) {
      if (!_strnicmp(line+10, "US", 2)) {
        VC->unit = false;
      } else if (!_strnicmp(line+10, "METRIC", 6)) {
        VC->unit = true;
      }
    } else if (!_strnicmp(line, "OSO_PYR", 7)) {
			OSO_DialogFunc::clbkPYR(nullptr, line+8, LC);
    } else if (!_strnicmp(line, "OSO_XYZ", 7)) {
			OSO_DialogFunc::clbkXYZ(nullptr, line+8, LC);
	  } else if (!_strnicmp(line, "OSO_DIST", 8)) {
      if (sscanf_s(line+9,"%f",&paramf)) {
        VC->range = id(paramf);
      }
   }  else if (!_strnicmp(line, "OSO_APR", 7)) {
      if (sscanf_s(line+8,"%d",&param)) {
				if (param<0 || param>2) param = 0;
        VC->apR = param;
      }
   }  else if (!_strnicmp(line, "OSO_APT", 7)) {
      if (sscanf_s(line+8,"%d",&param)) {
				if (param<0 || param>2) param = 0;
        VC->apT = param;
      }
   }  else if (!_strnicmp(line, "OSO_HUD", 7)) {
      if (sscanf_s(line+8,"%d",&param)) {
				if (param<0 || param>1) param = 0;
        VC->showHud = (param==1);
      }
    }
  }
  return;
}

void OnStationOps::WriteStatus(FILEHANDLE scn) const {
	char buf[128];

	if (VC->tgt[0].label.length() > 0) oapiWriteScenario_string(scn, "OSO_FT", (char *) VC->tgt[0].label.c_str());
  if (VC->tgt[1].label.length() > 0) oapiWriteScenario_string(scn, "OSO_TT", (char *) VC->tgt[1].label.c_str());
  sprintf_s(buf,"%s",(VC->unit?"METRIC":"US"));
  oapiWriteScenario_string(scn, "OSO_UNITS", buf);
	sprintf_s(buf,"%.2f",ed(VC->range));
	oapiWriteScenario_string(scn, "OSO_DIST", buf);

	if (VC->rOfsStr.length() > 0) oapiWriteScenario_string(scn, "OSO_PYR", (char *) VC->rOfsStr.c_str());
	if (VC->rOfsStr.length() > 0) oapiWriteScenario_string(scn, "OSO_XYZ", (char *) VC->tOfsStr.c_str());

  sprintf_s(buf,"%d",VC->apR);
  oapiWriteScenario_string(scn, "OSO_APR", buf);
  sprintf_s(buf,"%d",VC->apT);
  oapiWriteScenario_string(scn, "OSO_APT", buf);

	if (VC->showHud) {
    oapiWriteScenario_string(scn, "OSO_HUD", "1");
  } else {
    oapiWriteScenario_string(scn, "OSO_HUD", "0");
  }
  return;
}



