// ==============================================================
//
//	Bare Bones Auto Pilot (OSO)
//	============================
//
//	Copyright (C) 2014	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See OSO.cpp
//
// ==============================================================


#ifndef __OSO_H
#define __OSO_H

#include "OSO_Cores.hpp"   

class OnStationOps: public MFD2
{
public:
	OnStationOps (DWORD w, DWORD h, VESSEL *vessel, UINT mfd);
	~OnStationOps ();
	
  char *ButtonLabel (int bt);
	int ButtonMenu (const MFDBUTTONMENU **menu) const;
  bool ConsumeKeyBuffered (DWORD key);
  bool ConsumeButton (int bt, int event);
  
  bool Update (oapi::Sketchpad *skp);
  static int MsgProc (UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam);

  // Button Press Handlers
  void Button_FT();
  void Button_TT();
  void Button_UNT();
  void Button_RNG();
  void Button_PYR();
  void Button_XYZ();
  void Button_APR();
  void Button_APT();
  void Button_ZRR();
  void Button_ZRT();
  void Button_HUD();
  void Button_OK();
  void Button_NXT();
  void Button_PRV();
  void Button_APD();

  const char* GetModuleName() const;

  // Persistence functions
  void ReadStatus(FILEHANDLE scn);
  void WriteStatus(FILEHANDLE scn) const;

  // Unit conversions
  double id( double d ) const;
  double ed( double d ) const;

protected:
  OSO_GCore* GC;
  OSO_LCore* LC;
  OSO_MCore* MC;
  OSO_VCore* VC;

  int Line( int row );
  int Col( int pos );
  int Col2( int pos );
  void ShowMessage(oapi::Sketchpad *skp);

  oapi::Font *font;


};

#endif // !__OSO_H