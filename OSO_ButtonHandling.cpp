// ==============================================================
//
//	OSO (Button Handling Code)
//	============================
//
//	Copyright (C) 2013	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See OSO.cpp
//
// ==============================================================

#include "OnStationOps.hpp"
#include "OSO_DialogFunc.hpp"
//#include <EnjoLib/ModuleMessaging.hpp>

// ==============================================================
// MFD button hooks to Button Page library
//
char* OnStationOps::ButtonLabel (int bt)
{
	return LC->B.ButtonLabel(bt);
}

// Return button menus
int OnStationOps::ButtonMenu (const MFDBUTTONMENU **menu) const
{
	return LC->B.ButtonMenu(menu);
}

// Return clicked button
bool OnStationOps::ConsumeButton (int bt, int event) {
  return LC->B.ConsumeButton(this, bt, event);
}

// Return pressed keystroke
bool OnStationOps::ConsumeKeyBuffered (DWORD key) {
  return LC->B.ConsumeKeyBuffered(this, key);
}



// ==============================================================
// MFD Button Handler Callbacks
//

// Set Free Floating Target
	void OnStationOps::Button_FT() {
		char str[60];
		if (VC->tgt[0].tgtV) {
			strcpy(str,VC->tgt[0].label.c_str());
		} else {
			str[0] = '\0';
		}
		oapiOpenInputBox( "Enter Free Floating Target, Target:Port, or Target:A<nn> attachment (? for more details)",OSO_DialogFunc::clbkFT, str, 60, LC);
	  return;
	}

// Set Tethered Target
	void OnStationOps::Button_TT() {
		char str[60];
		if (VC->tgt[1].tgtV) {
			strcpy(str,VC->tgt[1].label.c_str());
		} else {
			str[0] = '\0';
		}
		oapiOpenInputBox( "Enter Tethered Target, Target:Port, or Target:A attachment (? for more details)",OSO_DialogFunc::clbkTT, str, 60, LC);
	  return;
	}

// Set Range to reference point (range measured from target's point and direction)
  void OnStationOps::Button_RNG() {
		char str[60];
		sprintf_s(str,"%.2f", ed(VC->range));
		if (VC->unit) {
			oapiOpenInputBox( "Enter Range Offset to Free Target (in m)",OSO_DialogFunc::clbkDist, str, 60, LC);
		} else {
			oapiOpenInputBox( "Enter Range Offset to Free Target (in ft)",OSO_DialogFunc::clbkDist, str, 60, LC);
		}
	  return;
	}

// Set Units
  void OnStationOps::Button_UNT() {
		VC->unit = !VC->unit;
	  return;
	}

// Set Pitch Yaw Roll Offset (measured from our ship)
  void OnStationOps::Button_PYR() {
		oapiOpenInputBox( "Enter Pitch Yaw Roll offset (in degrees)",OSO_DialogFunc::clbkPYR, (char *) VC->rOfsStr.c_str(), 60, LC);
	  return;
	}

// Set Translation X Y Z Offset (measured from our ship, applied to the tether point)
  void OnStationOps::Button_XYZ() {
		if (VC->unit) {
			oapiOpenInputBox( "Enter X Y Z Offset for Tethered Target (in m)",OSO_DialogFunc::clbkXYZ, (char *) VC->tOfsStr.c_str(), 60, LC);
		} else {
			oapiOpenInputBox( "Enter X Y Z Offset for Tethered Target (in ft)",OSO_DialogFunc::clbkXYZ, (char *) VC->tOfsStr.c_str(), 60, LC);
		}
	  return;
	}


// Set Rotational AutoPilot On/Off
  void OnStationOps::Button_APR() {

		if (!VC->apRefRatesLoaded) {
			LC->SetMessage(string() + "Error!\n\nNo reference thruster parameters found for this vessel class.\n\nTo fix this, you need to launch RV Orientation in this same vessel and hit APC to auto-calibrate.\nThen restart this scenario.");
			return;
		}
		if ((VC->tgt[0].tgtV == nullptr) || (VC->tgt[1].tgtV == nullptr)) {
			LC->SetMessage(string() + "Error!\n\nAutopilot inactive until TGT selected.");
			return;
		}

		if (VC->IsConnectedToMe(VC->v,VC->tgt[0].tgtV)  ) {
			LC->SetMessage(string() + "Error!\n\nAutopilot inactive when free target is attached to us.");
			VC->apR = 0;
			VC->apT = 0;
			return;
    }

		switch (VC->apR) {
		case 0:
		case 1:
			VC->apR = 2;
			break;
		case 2:
			VC->apR = 0;
			VC->AutoPilotZeroRot();
			break;
		}
		VC->apRotErr = _V(0,0,0);
	  return;
	}

// Set Translational AutoPilot On/Off
  void OnStationOps::Button_APT() {

		if (!VC->apRefRatesLoaded) {
			LC->SetMessage(string() + "Error!\n\nNo reference thruster parameters found for this vessel class.\n\nTo fix this, you need to launch RV Orientation in this same vessel and hit APC to auto-calibrate.\nThen restart this scenario.");
			return;
		}
		if ((VC->tgt[0].tgtV == nullptr) || (VC->tgt[1].tgtV == nullptr)) {
			LC->SetMessage(string() + "Error!\n\nAutopilot inactive until TGT selected.");
			return;
		}

		if (VC->IsConnectedToMe(VC->v,VC->tgt[0].tgtV)  ) {
			LC->SetMessage(string() + "Error!\n\nAutopilot inactive when free target is attached to us.");
			VC->apR = 0;
			VC->apT = 0;
			return;
    }


		switch (VC->apT) {
		case 0:
		case 1:
			VC->apT = 2;
			break;
		case 2:
			VC->apT = 0;
			VC->AutoPilotZeroTra();
			break;
		}
		VC->apTraErr = _V(0,0,0);
	  return;
	}

// Set Rotational AutoPilot On/Off, in zero rotation mode
  void OnStationOps::Button_ZRR() {

		if (!VC->apRefRatesLoaded) {
			LC->SetMessage(string() + "Error!\n\nNo reference thruster parameters found for this vessel class.\n\nTo fix this, you need to launch RV Orientation in this same vessel and hit APC to auto-calibrate.\nThen restart this scenario.");
			return;
		}
		if ((VC->tgt[0].tgtV == nullptr) || (VC->tgt[1].tgtV == nullptr)) {
			LC->SetMessage(string() + "Error!\n\nAutopilot inactive until TGT selected.");
			return;
		}

		if (VC->IsConnectedToMe(VC->v,VC->tgt[0].tgtV)  ) {
			LC->SetMessage(string() + "Error!\n\nAutopilot inactive when free target is attached to us.");
			VC->apR = 0;
			VC->apT = 0;
			return;
    }


		switch (VC->apR) {
		case 0:
		case 2:
			VC->apR = 1;
			break;
		case 1:
			VC->apR = 0;
			VC->AutoPilotZeroRot();
			break;
		}
		VC->apRotErr = _V(0,0,0);
	  return;
	}

// Set Translational AutoPilot On/Off, in station keeping mode
  void OnStationOps::Button_ZRT() {

		if (!VC->apRefRatesLoaded) {
			LC->SetMessage(string() + "Error!\n\nNo reference thruster parameters found for this vessel class.\n\nTo fix this, you need to launch RV Orientation in this same vessel and hit APC to auto-calibrate.\nThen restart this scenario.");
			return;
		}
		if ((VC->tgt[0].tgtV == nullptr) || (VC->tgt[1].tgtV == nullptr)) {
			LC->SetMessage(string() + "Error!\n\nAutopilot inactive until TGT selected.");
			return;
		}

		if (VC->IsConnectedToMe(VC->v,VC->tgt[0].tgtV)  ) {
			LC->SetMessage(string() + "Error!\n\nAutopilot inactive when free target is attached to us.");
			VC->apR = 0;
			VC->apT = 0;
			return;
    }


		switch (VC->apT) {
		case 0:
		case 2:
			VC->apT = 1;
			break;
		case 1:
			VC->apT = 0;
			VC->AutoPilotZeroTra();
			break;
		}
		VC->apTraErr = _V(0,0,0);
	  return;
	}


// Toggle HUD On/Off
  void OnStationOps::Button_HUD() {
		VC->showHud = !VC->showHud;
		LC->SetMessage(string() + "HUD Not Ready Yet!");
	  return;
	}

// OK button / dismiss message
  void OnStationOps::Button_OK() {
		LC->B.SwitchPage(this,0);
		LC->showMessage = false;
	  return;
	}

// NXT button / next page of message
  void OnStationOps::Button_NXT() {
		LC->messagePage++;
		if (LC->messagePage<LC->messagePageMax) {
			LC->B.SwitchPage(this,3);
		} else {
			LC->B.SwitchPage(this,4);
		}
	  return;
	}

// PRV button / prev page of message
  void OnStationOps::Button_PRV() {
		LC->messagePage--;
		if (LC->messagePage>0) {
			LC->B.SwitchPage(this,3);
		} else {
			LC->B.SwitchPage(this,2);
		}
	  return;
	}

	void OnStationOps::Button_APD() {
  if (VC->dumpAP) {
    fclose(VC->apD);
  } else {
    VC->apD_err = fopen_s(&(VC->apD), ".\\Config\\MFD\\OSO\\OSO_AP_Dump.csv","w");
    if (VC->apD_err) {
      LC->showMessage = true;

      sprintf_s(LC->cmessage,"Error!\n\nError %d opening AP dump file!", VC->apD_err);
      LC->smessage = LC->cmessage;
      LC->SetMessage(LC->smessage);
      return; // Couldn't open dump file
    }

    fprintf_s(VC->apD, "SIMT,STEPAVG,Dx,Dx.V,Dx.L,Dx.Th,Dx.C,Dx.A,Dx.B,Dy,Dy.V,Dy.L,Dy.Th,Dy.C,Dy.A,Dy.B,Dz,Dz.V,Dz.L,Dz.Th,Dz.C,Dz.A,DZ.B,Px,Px.V,Px.L,Px.Th,Px.C,Px.A,Px.B,Py,Py.V,Py.L,Py.Th,Py.C,Py.A,Py.B,Pz,Pz.V,Pz.L,Pz.Th,Pz.C,Pz.A,Pz.B,Pz.Tgt,AppInCone,MaxDirErr,MaxPosErr,RotActive,AttActive,AppActive,ApToggle,  Range,OurLat,WPLat,OurLong,WPLong,Mode\n");
    //VC->dumpSimT = VC->simT;
  }
  VC->dumpAP = !VC->dumpAP;
  return;
};