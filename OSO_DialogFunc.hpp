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


#ifndef __OSO_DIALOGFunc
#define __OSO_DIALOGFunc

class OSO_DialogFunc
{
    public:
      static bool clbkFT(void *id, char *str, void *usrdata);
      static bool clbkTT(void *id, char *str, void *usrdata);
      static bool clbkTgt(void *id, char *str, void *usrdata, bool ttDlg);
      static bool clbkPYR(void *id, char *str, void *usrdata);
      static bool clbkXYZ(void *id, char *str, void *usrdata);
      static bool clbkDist(void *id, char *str, void *usrdata);

			static double id( bool unit, double d );

    protected:
    private:

};

#endif // OSO_DIALOGTGT