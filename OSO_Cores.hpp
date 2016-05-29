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

#include "windows.h"
#include "orbitersdk.h"
#include "OSO_Buttons.hpp"
#include "MFDPersist.hpp"
#include <list>
#include <string>
#include <set>
#include <cstddef>
//#include <EnjoLib/IMessagingSender.hpp>
using namespace std;

#ifndef _OSO_CORE_CLASSES
#define _OSO_CORE_CLASSES

// Forward reference needed, as the cores all talk to each other. (This is also why the cores are kept together in this single hpp file)
class OSO_GCore;

class MMTest_Common {
public:

	std::string name;

};
//+++++
// Vessel Persistence core. One of these is instantiated per Vessel flown with RV Orientation up.
//+++++

class OSO_VCore { //: public EnjoLib::IMessagingSender {
  public:
    // Core references ... instantiation, vessel reference and GC.
    OSO_VCore(VESSEL *vin, OSO_GCore* gcin);
    ~OSO_VCore();
    OSO_GCore* GC;

		// Implement EnjoLib::IMessagingSender methods
    const char * GetModuleName() const { return "OnStationOps"; }

		// Implement vessel functions
		bool IsConnectedToMe(VESSEL *me, VESSEL *tgt);
		bool IsConnectedToMe(VESSEL *me, VESSEL *tgt, int depth, set<VESSEL *> &s);
		void CalcRotTransform(VECTOR3 &OriDir, VECTOR3 &OriRot, VECTOR3 &rotA, MATRIX3 &rotM );
		void Update();
		void AutoPilotCtrlLoad();
		void AutoPilotExecute();
		void AutoPilotThrust(THGROUP_TYPE thrustPositive,THGROUP_TYPE thrustNegative,
      double ofs, double ofsRate,
      double &dVs, double &apErr,
      double rateCtrl[12][2],                                         // Rate Control Array
      double &thrust, double &thrustEst, double &rateLim);
		bool AutoPilotRateFetch();
		void AutoPilotZeroRot();
		void AutoPilotZeroTra();
		void CalculateApRefMaster(double apRateSet[6][2], double apMassSet[2]);
		void CalculateApRef();

		// Add Vessel data here
    VESSEL *v;

		// Vessel Data for Tethered Target and Free Target
		struct LCL_ORI_STRUCT {
			VECTOR3 pos, dir, rot;
			VECTOR3 posRate, dirRate, rotRate, rotARate;
			VECTOR3 oPos, oDir, oRot;
			VECTOR3 rotA, oRotA;
		} l, *tOri[3];

		struct TGT_STRUCT {
			string label;
			int dock;
			int attachment;
			bool hasDock;
			bool hasAtt;
			bool attParent;
			OBJHANDLE hTgtV;
			VESSEL *tgtV;
			DOCKHANDLE hTgtD;
			ATTACHMENTHANDLE hTgtA;
			VECTOR3 rPos, rDir, rRot;
			VECTOR3 gPos, gDir, gRot;
			MATRIX3 rotM;
			struct LCL_ORI_STRUCT l;
		} tgt[2];  // FT is 0, TT is 1

		double lastTime;
		bool ratesInitialized;
		bool ratesValid;

		bool unit;

		bool dumpAP;
		FILE *apD;
    errno_t apD_err;

		double range;
		VECTOR3 rOfs;
		VECTOR3 tOfs;
		string rOfsStr;
		string tOfsStr;

		int apR, apT;
		bool showHud;

		VECTOR3 dispRotA, dispRotARate, dispPos, dispPosRate;

    double apCalMasterRate[6][2];                                   // Master rates, 1 per direction, low and high weights
    double apCalMasterMass[2];                                      // Master mass - average of mass through each burn
		bool apRefRatesLoaded;
    double apRefMass;                                               // Reference mass (recalculated when 0.1% away from current mass)
    VECTOR3 apRefRot;                                               // Reference rotation rates (deg/s) for apRefMass
    VECTOR3 apRefAtt;                                               // Reference attitude rates (m/s) for apRefMass
    double apRefMaster[7][2];                                       // Reference Master Calibration Power Coefficients
		double apRotPCtrl[12][2];
		double apRotYCtrl[12][2];
		double apRotRCtrl[12][2];
		double apAttXCtrl[12][2];
		double apAttYCtrl[12][2];
		double apAttZCtrl[12][2];
		bool apRotActive;
		bool apTraActive;
		bool apTraAllow;
		int apToggle;
    // Autopilot Error Aggregators
    VECTOR3 apRotErr, apTraErr;                                     // Aggregates errors where actual rate < 75% demanded rate (boots thrust)
    // Autopilot reference thrust calculations
    VECTOR3 apRotThrust, apTraThrust;                               // Autopilot rotation and attitude thrust settings
    VECTOR3 apRotThrustE, apTraThrustE;                             // Desired thrust, estimate, rate
    VECTOR3 apRotRateLim, apTraRateLim;                             // Resulting thrust, estimate, rate

		bool apRanT; 
		bool apRanRP, apRanRY,apRanRR; 

		double simStep;
  private:
};

//+++++
// MFD Panel Persistence core. One of these is instantiated per MFD panel position used by RV Orientation (e.g. usually left, right, extmfd)
//+++++

class OSO_MCore {
  public:
    // MFD Panel references ... instantiation, mfd position reference and GC
    OSO_MCore(UINT mfdin, OSO_GCore* gcin);
    UINT m;
    OSO_GCore* GC;
 
    // Add MFD panel data here
};

//+++++
// Local Persistence core. One of these is instantiated per Vessel AND MFD panel location. Local defaults for that combination.
//+++++

class OSO_LCore {
  public:
    // Local references ... instantiation, references for vesseland mfd position, and links to the appropriate VC, MC and GC
    OSO_LCore(VESSEL *vin, UINT mfdin, OSO_GCore* gcin);
    VESSEL *v;
    UINT m;
    OSO_GCore* GC;
    OSO_VCore* VC;
    OSO_MCore* MC;

    // Add local vessel+panel data here

		void SetMessage(const string &rawMessage);

    OSO_Buttons B;
    bool showMessage;
		bool switchMenu;
    int messagePage;
		int messagePageMax;
    char cmessage[256];
    string smessage;
    vector<string> message;

};

//+++++
// Global Persistence core. One of these is instantiated for the whole orbiter session, on the first launch of this MFD type
//+++++

class OSO_GCore {
  public:
    // Global references ... instantiation and a link to the persistence library (running the linked lists)
    OSO_GCore();
    MFDPersist P;

    // Add global RV Orientation core data here


    OSO_VCore* VC;    // Focus Vessel core

};


/**
 * \ingroup vec
 * \brief Multiplication of vector with vector
 * \param a vector operand
 * \param b vector operand
 * \return Result of element-wise a*b.
 */
inline double operator* (const VECTOR3 &a, const VECTOR3 &b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}





#endif // _OSO_CORE_CLASSES