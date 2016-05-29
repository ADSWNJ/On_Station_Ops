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



#ifndef _OSO_BUTTON_CLASS
#define _OSO_BUTTON_CLASS
#include "MFDButtonPage.hpp"

class OnStationOps;

class OSO_Buttons : public MFDButtonPage<OnStationOps>
{
  public:
    OSO_Buttons();
  protected:
    bool SearchForKeysInOtherPages() const;
  private:
};
#endif // _OSO_BUTTON_CLASS

