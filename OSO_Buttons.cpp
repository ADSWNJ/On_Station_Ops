// ==============================================================
//
//	OSO (MFD Button Management)
//	============================
//
//	Copyright (C) 2013	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See OSO.cpp
//
// ==============================================================

#include "MFDButtonPage.hpp"
#include "OSO_Buttons.hpp"
#include "OnStationOps.hpp"


OSO_Buttons::OSO_Buttons() 
{
    // Base menu descriptions
    static const MFDBUTTONMENU mnu0[] =
    {
      {"Tethered Tgt", 0, '1'},
      {"Free Tgt", 0, '2'},
      {"Set Units", 0, 'U'},
      {"Set Distance", 0, 'D'},
      {"Request PYR Ofs", 0, 'P'},
      {"Request XYZ Ofs", 0, 'X'},
      {"AutoPilot Rot", 0, 'R'},
      {"AutoPilot Trans", 0, 'T'},
      {"Zero Rel Rot", 0, 'Z'},
      {"Zero Rel Trans", 0, 'A'}
//      {"HUD On/Off", 0, 'H'},
    };
    RegisterPage(mnu0, sizeof(mnu0) / sizeof(MFDBUTTONMENU));
    RegisterFunction("TT", OAPI_KEY_1,  &OnStationOps::Button_TT);
    RegisterFunction("FT", OAPI_KEY_2,  &OnStationOps::Button_FT);
    RegisterFunction("UNT", OAPI_KEY_U, &OnStationOps::Button_UNT);
    RegisterFunction("DST", OAPI_KEY_D, &OnStationOps::Button_RNG);
    RegisterFunction("PYR", OAPI_KEY_P,  &OnStationOps::Button_PYR);
    RegisterFunction("XYZ", OAPI_KEY_X,  &OnStationOps::Button_XYZ);
    RegisterFunction("APR", OAPI_KEY_R, &OnStationOps::Button_APR);
    RegisterFunction("APT", OAPI_KEY_T, &OnStationOps::Button_APT);
    RegisterFunction("ZRR", OAPI_KEY_Z, &OnStationOps::Button_ZRR);
    RegisterFunction("ZRT", OAPI_KEY_A, &OnStationOps::Button_ZRT);
//   RegisterFunction("HUD", OAPI_KEY_H, &OnStationOps::Button_HUD);

    // Single page message page 
    static const MFDBUTTONMENU mnu1[] =
    {
      {"OK", 0, 'K'}
    };
    RegisterPage(mnu1, sizeof(mnu1) / sizeof(MFDBUTTONMENU));
    RegisterFunction("OK", OAPI_KEY_K,  &OnStationOps::Button_OK);

    // Page 1 of n message page
    static const MFDBUTTONMENU mnu2[] =
    {
      {"OK", 0, 'K'},
      {"NXT", 0, 'N'}
    };
    RegisterPage(mnu2, sizeof(mnu2) / sizeof(MFDBUTTONMENU));
    RegisterFunction("OK", OAPI_KEY_K,  &OnStationOps::Button_OK);
    RegisterFunction("NXT", OAPI_KEY_N,  &OnStationOps::Button_NXT);

    // Page n of m message page (1 < n < m)
    static const MFDBUTTONMENU mnu3[] =
    {
      {"OK", 0, 'K'},
      {"NXT", 0, 'N'},
      {"PRV", 0, 'P'}
    };
    RegisterPage(mnu3, sizeof(mnu3) / sizeof(MFDBUTTONMENU));
    RegisterFunction("OK", OAPI_KEY_K,  &OnStationOps::Button_OK);
    RegisterFunction("NXT", OAPI_KEY_N,  &OnStationOps::Button_NXT);
    RegisterFunction("PRV", OAPI_KEY_P,  &OnStationOps::Button_PRV);

    // Page n of n message page
    static const MFDBUTTONMENU mnu4[] =
    {
      {"OK", 0, 'K'},
      {"PRV", 0, 'P'}
    };
    RegisterPage(mnu4, sizeof(mnu4) / sizeof(MFDBUTTONMENU));
    RegisterFunction("OK", OAPI_KEY_K,  &OnStationOps::Button_OK);
    RegisterFunction("PRV", OAPI_KEY_P,  &OnStationOps::Button_PRV);
		
		return;
}

bool OSO_Buttons::SearchForKeysInOtherPages() const
{
    return false;
}



