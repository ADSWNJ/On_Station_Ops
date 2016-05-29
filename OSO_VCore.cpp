// ==============================================================
//
//	OSO (Vessel Core Persistence)
//	==============================
//
//	Copyright (C) 2013	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See OSO.cpp
//
// ==============================================================

#include "OSO_Cores.hpp"
#include "ParseFunctions.h"
#include <stdio.h>
#include <math.h>
#include <set>


OSO_VCore::OSO_VCore(VESSEL *vin, OSO_GCore* gcin) {
  GC = gcin;
  v = vin;

	tgt[0].tgtV = nullptr;
	tgt[1].tgtV = nullptr;

	tOri[0] = &(tgt[0].l);
	tOri[1] = &(tgt[1].l);
	tOri[2] = &l;

	range = 0;
	rOfsStr = "";
	tOfsStr = "";
	rOfs = _V(0,0,0);
	tOfs = _V(0,0,0);
	unit = true;
	apR = 0;
	apT = 0;
	showHud = false;
	
	ratesInitialized = false;
	ratesValid = false;

	AutoPilotCtrlLoad();
	apRefRatesLoaded = AutoPilotRateFetch();  // Pull in the autopilot from RVO
  if (apRefRatesLoaded) {
    // Calculate the Master Reference AP parameters set from our rates and weights
    CalculateApRefMaster(apCalMasterRate,apCalMasterMass);

    // Calculate reference rates for our current weight
    CalculateApRef();
  }
	apTraErr = _V(0,0,0);
	apRotErr = _V(0,0,0);
	return;
};

OSO_VCore::~OSO_VCore() {
}


void OSO_VCore::Update() {
	if (tgt[1].tgtV == nullptr || tgt[0].tgtV == nullptr) return;		// Need FT and TT to be valid before we do calcs

	for (int i=0; i<2; i++) {	// tgt[0] is the free target, tgt[1] is the tethered target
		if (tgt[i].hasDock) {
			tgt[i].tgtV->GetDockParams(tgt[i].hTgtD,tgt[i].rPos,tgt[i].rDir,tgt[i].rRot);		// Get the pos/dir/rot from the dock 
		} else if (tgt[i].hasAtt) {
			tgt[i].tgtV->GetAttachmentParams(tgt[i].hTgtA,tgt[i].rPos,tgt[i].rDir,tgt[i].rRot); // ... or from the attachment
		} else {
			tgt[i].rPos = _V(0.0,0.0,0.0);																													// ... or from the target vessel origin and default z and y axes
			tgt[i].rDir = _V(0.0,0.0,1.0);
			tgt[i].rRot = _V(0.0,1.0,0.0);
		}

	
		if (i==0) {
			tgt[i].rDir = -tgt[i].rDir;																				// Inward pointing direction on free target, outwards on the tethered target (to align directions for docking)
			tgt[i].rPos -= tgt[i].rDir * range;																// Add range to the free target Range is the desired distance to the target, expressed in target terms
		}

		tgt[i].tgtV->Local2Global(tgt[i].rPos,tgt[i].gPos);									// Switch position from target coords to global 
		v->Global2Local(tgt[i].gPos,tgt[i].l.pos);													// ... and back to our local coords

		tgt[i].tgtV->Local2Global(tgt[i].rDir+tgt[i].rPos,tgt[i].gDir);			// Make direction a point offset from the target pos
		v->Global2Local(tgt[i].gDir,tgt[i].l.dir);													// ... bring it local
		tgt[i].l.dir -= tgt[i].l.pos;																				// ... and remove the offset (so it sits on our origin now)

		tgt[i].tgtV->Local2Global(tgt[i].rRot+tgt[i].rPos,tgt[i].gRot);			// Same for rotation...
		v->Global2Local(tgt[i].gRot,tgt[i].l.rot);													// ... back to us
		tgt[i].l.rot -= tgt[i].l.pos;																				// ... and center on our origin

		CalcRotTransform(tgt[i].l.dir, tgt[i].l.rot, tgt[i].l.rotA, tgt[i].rotM); // Find the angle transform from our ship coords to the target    
	}

	l.pos = tgt[0].l.pos - tgt[1].l.pos - tOfs;														// For position - it's FT (plus range) minus TT minus the requested offset

	l.rotA = tgt[0].l.rotA - tgt[1].l.rotA - rOfs;												// For rotations, it's FT rot - TT rot - requested offset
	for (int i=0; i<3; i++) {
		while (l.rotA.data[i] > 180.0) l.rotA.data[i] -= 360.0;							// Make offset conform to -180 to +180
		while (l.rotA.data[i] < -180.0) l.rotA.data[i] += 360.0;
	}

	if (!ratesValid) {
		if (!ratesInitialized) {
			lastTime = oapiGetSimTime();
			for (int i=0; i<3; i++) {
				tOri[i]->oPos = tOri[i]->pos;
				tOri[i]->oDir = tOri[i]->dir;
				tOri[i]->oRot = tOri[i]->rot;
				tOri[i]->oRotA = tOri[i]->rotA;
			}
			ratesInitialized = true;
			return;
		} else {
			if (oapiGetSimTime() > lastTime + 0.05) {
				ratesValid = true;
			} else {
				return;
			}
		}
	}

	simStep = oapiGetSimTime() - lastTime;
	if (simStep>=0.1) {																													// Keep things to max 10 updates a second
		for (int i=0; i<3; i++) {
			VECTOR3 tmpDirRate,tmpRotRate, tmpRotARate; 
			tOri[i]->posRate = (tOri[i]->pos - tOri[i]->oPos) / simStep;
			tmpDirRate = (tOri[i]->dir - tOri[i]->oDir) / simStep;
			if (length(tmpDirRate) < 100.0) tOri[i]->dirRate = tmpDirRate;
			tmpRotRate = (tOri[i]->rot - tOri[i]->oRot) / simStep;
			if (length(tmpRotRate) < 100.0) tOri[i]->rotRate = tmpRotRate;
			tmpRotARate = (tOri[i]->rotA - tOri[i]->oRotA) / simStep;
			if (length(tmpRotARate) < 100.0) tOri[i]->rotARate = tmpRotARate;

			tOri[i]->oPos = tOri[i]->pos;
			tOri[i]->oDir = tOri[i]->dir;
			tOri[i]->oRot = tOri[i]->rot;
			tOri[i]->oRotA = tOri[i]->rotA;
		}
		lastTime = oapiGetSimTime();
	}

	apRanT = false;
	apRanRP = false;
	apRanRY = false;
	apRanRR	= false;

	if (apR>0 || apT>0) {
		if (IsConnectedToMe(v,tgt[0].tgtV)||!IsConnectedToMe(v,tgt[1].tgtV)) {
			if (apR>0) {
				AutoPilotZeroRot();
				apR = 0;
			}
			if (apT>0) {
				AutoPilotZeroTra();
				apT = 0;
			}
		}
		AutoPilotExecute();
	}
	return;
}


void OSO_VCore::CalcRotTransform(VECTOR3 &OriDir, VECTOR3 &OriRot, VECTOR3 &rotA, MATRIX3 &rotM ) {
	// Calculates the rotation needed to rotate ship coordinates onto OriDir (Z+) and OriRot (Y+), returning
	// the three Euler rotation angles and a rotation matrix.
  //
  // Use atan2 throughout as it understands all 4 quadrants and is safe on the cardinal points. Clean the matrix and rotations
	// if the angles are right. 

  VECTOR3 lDir = OriDir;
	MATRIX3 mPitch, mYaw, mRoll;
	double aPitch, aRoll, aYaw; // in radians

	// Work pitch initially to transform lDir.y to 0
	if (abs(lDir.y) > 1e-5) {											// Check if in the y=0 plane already - if so, aPitch = 0, mPitch is Identity
    aPitch = atan2(lDir.y, lDir.z);							// Pitch angle is the up (Y) over the Forward (Z). 
    if (aPitch < -PI * 0.5) {											// Bring pitch into the range -0.5 PI to 1.5 PI
      aPitch += 2.0 *PI;
    } else if (aPitch > PI * 1.5) {
      aPitch -= 2.0 * PI;
    }
		if (abs(aPitch - PI)<PI * 0.5) {						// If pitch is in quadrant 2 or 3, treat as pitch +- from PI radians (180 degrees)
			aPitch = aPitch - PI;
		}
    mPitch = _M(1,0,0,  0,cos(-aPitch),sin(-aPitch),  0,-sin(-aPitch),cos(-aPitch)); // Matrix to reverse out the pitch
  } else {
    mPitch = _M(1,0,0, 0,1,0, 0,0,1);           // No pitch correction needed. Note - if dDir.z is negative, treat that as a 180 deg yaw. 
		aPitch = 0.0;
  }
  lDir = mul(mPitch, lDir);                     // Apply the pitch ... if DEBUGGING, check y goes to 0

  // Work the yaw next, keeping things on the y=0 plane
  if (abs(lDir.x) > 1e-5) {											// Check if we are already on the z-axis (i.e. x=0). 
    aYaw = atan2(lDir.x, lDir.z);								// Yaw is right over the forward, arctan
		if (aYaw < -PI) {
      aYaw += 2.0 *PI;
    } else if (aYaw > PI) {
      aYaw -= 2.0 * PI;
    }
    mYaw = _M(cos(-aYaw),0,sin(-aYaw),  0,1,0,  -sin(-aYaw),0,cos(-aYaw));
  } else {																			// On the z-axis, so either pointing fwd or back
    if (lDir.z<0.0) {
      mYaw = _M(-1,0,0, 0,1,0, 0,0,-1);					// 180 yaw neded 
			aYaw = PI;	
    } else {
      mYaw = _M(1,0,0, 0,1,0, 0,0,1);						// No yaw correction needed. 
			aYaw = 0.0;	
    }
  }
  lDir = mul(mYaw, lDir);											  // Apply the yaw ... if DEBUGGING, check x and y now 0 and z is fwd

  // Construct the direction matrix (pitch and yaw) ... then add in the roll last.
  rotM = mul(mYaw, mPitch);

  // For the roll, look at dRot. Start by applying the pitch and yaw rotation to the rotation vector, in a temporary dRot transformed variable. 
  VECTOR3 lRot = mul(rotM,OriRot);
  
  if (abs(lRot.x) > 1e-5) {											// See if the roll vector is on the z-axis (up or down)
    aRoll = atan2(lRot.x, lRot.y);							// Roll angle is arctan right over up ... negative to roll it to y=1
		if (aRoll < -PI) {
      aRoll += 2.0 *PI;
    } else if (aRoll > PI) {
      aRoll -= 2.0 * PI;
    }
		mRoll = _M(cos(-aRoll),sin(-aRoll),0,  -sin(-aRoll),cos(-aRoll),0,  0,0,1);
  } else {																			// Roll vector is striahgt up or down
    if (lRot.y<0.0) {
      mRoll = _M(-1,0,0,0,-1,0,0,0,1);          // 180 degrees roll correction needed.
			aRoll = PI;
    } else {
      mRoll = _M(1,0,0,0,1,0,0,0,1);            // No roll correction needed. 
			aRoll = 0.0;
    }
  }
  lRot = mul(mRoll, lRot);                      // Apply the roll ... check for {0,1,0}
 
  // Complete the port orientation matrix by multiplying in the roll data.
  rotM = mul(mRoll, rotM);

  // Post-clean the matrix of rounding noise
  for (int i=0; i<10; i++) {
    if (abs(rotM.data[i])<1e-5) {
      rotM.data[i] = 0.0;
    } else if (abs(rotM.data[i]-1.0)<1e-5) { 
      rotM.data[i] = 1.0;
    } else if (abs(rotM.data[i]+1.0)<1e-5) { 
      rotM.data[i] = -1.0;
    }
  }
	rotA.x = aPitch * DEG;
	rotA.y = aYaw * DEG;
	rotA.z = aRoll * DEG;

	// Post-clean the rots
	for (int i=0; i<3; i++) {
    if (abs(rotA.data[i])<1e-4) {
      rotA.data[i] = 0.0;
    } else if (abs(rotA.data[i]-90.0)<1e-4) { 
      rotA.data[i] = 90.0;
    } else if (abs(rotM.data[i]+90.0)<1e-4) { 
      rotA.data[i] = -90.0;
    } else if (abs(rotM.data[i]-180.0)<1e-4) { 
      rotA.data[i] = 180.0;
    }
	}

  return;
}


bool OSO_VCore::IsConnectedToMe(VESSEL *me, VESSEL *tgt) {
	set<VESSEL *> s;
	return IsConnectedToMe(me, tgt, 0, s);
}

bool OSO_VCore::IsConnectedToMe(VESSEL *me, VESSEL *tgt, int depth, set<VESSEL *> &s) {
  if (me==tgt) return true;
  if (depth >= 10) return false;                 // Nesting protection ... if we are trying to go too deep in a nest for a weird reason (unseen yet by the author!), we default to not attached

	if (s.find(tgt) != s.end()) {
    return false;									              // Backtrack prevention ... i.e. don't reconsider a vessel we have already checked
  }
	s.insert(tgt);


  OBJHANDLE objh;
  ATTACHMENTHANDLE ah;
  DOCKHANDLE dh;
  VESSEL *t2;

  //
  // Check each parent attachment
  //
  for (unsigned int i=0; i<tgt->AttachmentCount(true); i++) {
    ah = tgt->GetAttachmentHandle(true,i);              // Find the attachment handle
    if (ah) {                                           // Should always be a value
      objh = tgt->GetAttachmentStatus(ah);              // See if anything attached
      if (objh) {
        t2 = oapiGetVesselInterface(objh);              // If yes, then fund the vessel's interface (i.e. convert OBJHANDLE to VESSEL*)
        if (IsConnectedToMe(me,t2,depth,s)) return true; // And recursively search to see if this vessel is me or connected to me.
      }
    }
  }

  //
  // Check each child attachment ... same process as with parents
  //
  for (unsigned int i=0; i<tgt->AttachmentCount(false); i++) {
    ah = tgt->GetAttachmentHandle(false,i);
    if (ah) {
      objh = tgt->GetAttachmentStatus(ah);
      if (objh) {
        t2 = oapiGetVesselInterface(objh);
        if (IsConnectedToMe(me,t2,depth,s)) return true;
      }
    }
  }

  //
  // Check each dock
  //
  for (unsigned int i=0; i<tgt->DockCount(); i++) {
    dh = tgt->GetDockHandle(i);
    if (dh) {
      objh = tgt->GetDockStatus(dh);
      if (objh) {
        t2 = oapiGetVesselInterface(objh);
        if (IsConnectedToMe(me,t2,depth, s)) return true;
      }
    }
  }
  return false;
}


bool OSO_VCore::AutoPilotRateFetch() {
  // Fetch calibration data for this ship class, if found
  FILE* rf;
  char clName[128];
  char buf[256];
  char *tok;
  char *bp;
  bool goodFetch = false;
  double params[14];
  int i;

	strcpy_s(clName,128,v->GetClassName());

  fopen_s(&rf, ".\\Config\\MFD\\RVO\\RVO_Rates.cfg","r");

  while (fgets(buf,255,rf)!=NULL) {
    bp = buf;
    if (!ParseWhiteSpace(&bp,&tok)) continue;   // Skip leading tabs and any comments
    if (!ParseQuotedString(&bp,&tok)) continue; // Bad parse
    if (_stricmp(tok,clName)!=0) continue;      // Not our ship
    for (i=0; i<14;i++) {
      if (!ParseDouble(&bp,&(params[i]))) break; // Bad parse
    }
    if (i<14) continue;                         // Bad parse

    apCalMasterMass[0] = params[0];
    for (int p=0; p<6; p++) {
      apCalMasterRate[p][0] = params[p+1];
    }
    apCalMasterMass[1] = params[7];
    for (int p=0; p<6; p++) {
      apCalMasterRate[p][1] = params[p+8];
    }
    goodFetch = true;                           // Found and successfully loaded a ship rate file (note - keep going in case user has calibrated more)
  }

  fclose(rf);
  return goodFetch;
}

void OSO_VCore::CalculateApRefMaster(double apRateSet[6][2], double apMassSet[2]) {
  // Loads apRefMaster coefficients
  //
  // Method ...
  // We have two sets of rates, representing deg/sec and m/sec acceleration rates for low and high weight.
  // There is a direct power correlation between them, of the form Y=AX^B. E.g. Right thrust = A times Mass ^ B
  // Taking natural logs, we get LOG(Y) = LOG(A) + B * LOG(X), which is a straight line now.
  // For the slope ... B = ( LOG(Y1) - LOG(Y0) ) / ( LOG(X1) - LOG(X0) )
  // For the intercept at LOG(X) = 0 ... LOG(A) = LOG(Y0) - B * LOG(X1) ... and trivially A = EXP(LOG(A))
  //
  double X0 = apMassSet[0];
  double X1 = apMassSet[1];

  for (int i=0; i<6; i++) {
    double Y0 = apRateSet[i][0];
    double Y1 = apRateSet[i][1];
    double B = (log(Y1) - log(Y0))/(log(X1) - log(X0));
    double LOGA = log(Y0) - (B * log(X1));
    apRefMaster[i][0] = LOGA; 
    apRefMaster[i][1] = B; 
  }
  return;
}

void OSO_VCore::CalculateApRef() {
  // Loads apRefMass, apRefRot and apRefAtt via the apRefMaster
  //
  // Method...
  // We are calculating the reference rate set from apRefMaster, for our current weight
  // The equation is Y = AX^B ... i.e. LOG(Y) = LOG(A) + B * LOG(X) ... i.e. Y = EXP(LOG(A) + B * LOG(X)). 
  // We have LOG(A) and B in the apRefMaster, so this won't take long...
  //
  apRefMass = v->GetMass();
  double LOGX = log(apRefMass);
  for (int i=0; i<3; i++) apRefRot.data[i] = exp(apRefMaster[i][0] + (apRefMaster[i][1] * LOGX));
  for (int i=3; i<6; i++) apRefAtt.data[i-3] = exp(apRefMaster[i][0] + (apRefMaster[i][1] * LOGX));
  return;
}

void OSO_VCore::AutoPilotCtrlLoad() {
  // Initialize Control Rates

  double RotPCtrl[12][2] = {
    {40.00, 0.500}, {20.00, 0.500}, {10.00, 0.500},
    { 4.00, 0.200}, { 2.00, 0.200}, { 1.00, 0.060},
    { 0.40, 0.020}, { 0.30, 0.015}, { 0.20, 0.010},
    { 0.12, 0.006}, { 0.08, 0.004}, { 0.04, 0.002} };
  double RotYCtrl[12][2] = {
    {40.00, 0.500}, {20.00, 0.500}, {10.00, 0.500},
    { 4.00, 0.200}, { 2.00, 0.200}, { 1.00, 0.060},
    { 0.40, 0.020}, { 0.30, 0.015}, { 0.20, 0.010},
    { 0.12, 0.006}, { 0.08, 0.004}, { 0.04, 0.002} };
  double RotRCtrl[12][2] = {
    {40.00, 0.500}, {20.00, 0.500}, {10.00, 0.500},
    { 4.00, 0.200}, { 2.00, 0.200}, { 1.00, 0.060},
    { 0.40, 0.020}, { 0.30, 0.015}, { 0.20, 0.010},
    { 0.12, 0.006}, { 0.08, 0.004}, { 0.04, 0.002} };
  double AttXCtrl[12][2] = {
    {1000.00, 5.000}, {400.00, 3.000}, {100.00, 1.000},
    { 40.00, 1.000}, { 30.00, 0.800}, { 20.00, 0.600},
    { 10.00, 0.400}, {  3.00, 0.150}, {  1.00, 0.080},
    {  0.60, 0.040}, {  0.30, 0.020}, {  0.10, 0.005} };
  double AttYCtrl[12][2] = {
    {1000.00, 5.000}, {400.00, 3.000}, {100.00, 1.000},
    { 40.00, 1.000}, { 30.00, 0.800}, { 20.00, 0.600},
    { 10.00, 0.400}, {  3.00, 0.150}, {  1.00, 0.080},
    {  0.60, 0.040}, {  0.30, 0.020}, {  0.10, 0.005} };
  double AttZCtrl[12][2] = {
    {1000.00,5.000}, {500.00, 3.000}, {100.00, 1.000},
    { 40.00, 1.000}, { 30.00, 0.800}, { 20.00, 0.600},
    { 10.00, 0.400}, {  3.00, 0.150}, {  1.00, 0.080},
    {  0.60, 0.040}, {  0.30, 0.020}, {  0.10, 0.005} };


  for (int i=0; i<12; i++) {
	  for (int j=0; j<2; j++) {
      apRotPCtrl[i][j] = RotPCtrl[i][j];
      apRotYCtrl[i][j] = RotYCtrl[i][j];
      apRotRCtrl[i][j] = RotRCtrl[i][j];
      apAttXCtrl[i][j] = AttXCtrl[i][j];
      apAttYCtrl[i][j] = AttYCtrl[i][j];
      apAttZCtrl[i][j] = AttZCtrl[i][j];
		}
  }
  return;
}

void OSO_VCore::AutoPilotZeroRot() {
  v->SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0.0);
}
void OSO_VCore::AutoPilotZeroTra() {
  v->SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_UP, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_BACK, 0.0);
}

void OSO_VCore::AutoPilotExecute() {

  //
  // Master Auto Pilot Execution Sequence
  //  
  //


	// If in ZRR or ZRT mode, kill the respective Pos offset or rot offset. 
	VECTOR3 apPos = (apT==1? _V(0,0,0) : l.pos);
	VECTOR3 apRotA = (apR==1? _V(0,0,0) : l.rotA);
	VECTOR3 apPosRate = l.posRate;
	VECTOR3 apRotARate = l.rotARate;


  apRotActive = false;                                          // For dump purposes
  apTraActive = false;                                          // For dump purposes      

  // 0. Clear out any rates we are looking after
  v->SetThrusterGroupLevel(THGROUP_MAIN, 0.0);
  v->SetThrusterGroupLevel(THGROUP_RETRO, 0.0);
  v->SetThrusterGroupLevel(THGROUP_HOVER, 0.0);
  if (apR) AutoPilotZeroRot();
  if (apT) AutoPilotZeroTra();

  // 1. Reset mass reference calce if mass outside of 0.1% of reference
  if ((abs(v->GetMass() - apRefMass) > 0.001 * apRefMass)) CalculateApRef();

  // 2. Cross-Interlock code
  if (apR && ((length(apRotA)>30.0) || (length(apRotARate) > 2.00))) {									// All Rot till it settles at coarse level
    apTraAllow = false;
  } else if (apT && ((length(apPos)>200.0) || (length(apPosRate)>10.0))) {							// 5:1 Att:Rot until att settles coarse level
    apTraAllow = true;
    apToggle = 5;
  } else if (apR && ((length(apRotA)>5.0) || (length(apRotARate) > 1.00))) {						// All Rot till it settles at mid level
    apTraAllow = false;
  } else if (apT && ((length(apPos)>0.5) || (length(apPosRate)>0.05))) {								// 2:1 Att:Rot until rot settles fine level
    apTraAllow = true;
    if (apToggle>2) apToggle = 2;
  } else {
    apTraAllow = apT>0;		                                                              // Else ... 1 in 5 Rot (keeps Att accurate).
    if (!apR) apToggle = 5;
  }

  // Interlocks in place now

  if (apR && (!apTraAllow || !apToggle)) {

    // 3. Run the AP Rotation

    apRotActive = true;

		// Weirdness ... if the |Yaw| is >= 90, then we need to reverse the roll.
		if (abs(apRotA.y)<=90.0) { 
			// Roll Right/Left Control
			AutoPilotThrust(THGROUP_ATT_BANKRIGHT,THGROUP_ATT_BANKLEFT,apRotA.z,apRotARate.z,
				apRefRot.z, apRotErr.z, apRotRCtrl,
				apRotThrust.z, apRotThrustE.z, apRotRateLim.z);
		} else {
			// Inverse Roll Right/Left Control
			AutoPilotThrust(THGROUP_ATT_BANKLEFT,THGROUP_ATT_BANKRIGHT,apRotA.z,apRotARate.z,
				apRefRot.z, apRotErr.z, apRotRCtrl,
				apRotThrust.z, apRotThrustE.z, apRotRateLim.z);
		}
		apRanRR	= true;
				
		if (abs(apRotA.z)<30.0) {		// if roll too big, then just work on roll
				
			// Yaw Right/Left Control
			AutoPilotThrust(THGROUP_ATT_YAWRIGHT,THGROUP_ATT_YAWLEFT,apRotA.y,apRotARate.y,
				apRefRot.y, apRotErr.y, apRotYCtrl,
				apRotThrust.y, apRotThrustE.y, apRotRateLim.y);
			apRanRY = true;
			if (abs(apRotA.y)<30.0) {		// only work on pitch when roll and yaw settled down
				// Pitch Up/Down Control
				AutoPilotThrust(THGROUP_ATT_PITCHUP,THGROUP_ATT_PITCHDOWN,apRotA.x,apRotARate.x,
					apRefRot.x, apRotErr.x, apRotPCtrl,
					apRotThrust.x, apRotThrustE.x, apRotRateLim.x);
				apRanRP = true;
			}
		}
    apToggle = 5;


  } else if (apTraAllow && apToggle) {

    // 4. Run the AP Translation 

    if (apR && (apToggle > 0)) apToggle--;
    apTraActive = true;

    // Right-Left Control
    AutoPilotThrust(THGROUP_ATT_RIGHT,THGROUP_ATT_LEFT,apPos.x,apPosRate.x,
      apRefAtt.x, apTraErr.x, apAttXCtrl,
      apTraThrust.x, apTraThrustE.x, apTraRateLim.x);
		apRanT = true;

    // Up-Down Control
    AutoPilotThrust(THGROUP_ATT_UP,THGROUP_ATT_DOWN,apPos.y,apPosRate.y,
      apRefAtt.y, apTraErr.y, apAttYCtrl,
      apTraThrust.y, apTraThrustE.y, apTraRateLim.y);

		// Forward-Back Control
    AutoPilotThrust(THGROUP_ATT_FORWARD,THGROUP_ATT_BACK,apPos.z,apPosRate.z,
      apRefAtt.z, apTraErr.z, apAttZCtrl,
      apTraThrust.z, apTraThrustE.z, apTraRateLim.z);
  }

/*  // Dump debug data on AP response if asked...
  if (dumpAP) {

    apD_err = fprintf_s(apD,"%.2f,%.4f,     %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%i,%.3f,%.3f,%d,%d,%d,%d,%d,   %.3f,%.2f,%.2f,%.2f,%.2f,%d\n",
                      simT - dumpSimT, simStepAvg,
                      oriDir.x, oriDirRate.x, apRotRateLim.x, apRotThrust.x, apRotThrustE.x, apRefRot.x, apRotErr.x,
                      oriDir.y, oriDirRate.y, apRotRateLim.y, apRotThrust.y, apRotThrustE.y, apRefRot.y, apRotErr.y,
                      oriDir.z, oriDirRate.z, apRotRateLim.z, apRotThrust.z, apRotThrustE.z, apRefRot.z, apRotErr.z,
                      oriPos.x, oriPosRate.x, apAttRateLim.x, apAttThrust.x, apAttThrustE.x, apRefAtt.x, apAttErr.x,
                      oriPos.y, oriPosRate.y, apAttRateLim.y, apAttThrust.y, apAttThrustE.y, apRefAtt.y, apAttErr.y,
                      oriPos.z, oriPosRate.z, apAttRateLim.z, apAttThrust.z, apAttThrustE.z, apRefAtt.z, apAttErr.z,

                      zTarget, (apAppConeOK? 1 : 0), maxDirErr, maxPosErr,
                      apRotActive,apAttActive,apAppActive,apAttAllow,apToggle,
                      rtvOurRange, rtvOurLat, rtvWPlat[curWP], rtvOurLong, rtvWPlong[curWP], mode
                      
                      );

  }
*/

  return;
}



void OSO_VCore::AutoPilotThrust(THGROUP_TYPE thrustPositive,THGROUP_TYPE thrustNegative, // Specific thrust calcs
      double ofs, double ofsRate,
      double &dVs, double &apErr,
      double rateCtrl[12][2],                                         // Rate Control Array
      double &thrust, double &thrustEst, double &rateLim) {

  // AutoPilotThrust ... implements control algorithm for rot, att and app thrusts 
  // General idea ... looking to trend the offset rate to a value controlled by the offset
  // We ramp the Hi and Lo checks down ofsIter times, dividing by 10 each time, to achieve fine control


  int    sign;

  sign = (ofs < 0.0? 1 : -1);
  rateLim = rateCtrl[11][1] * sign;   // Default to lowest rate

  // Find target and rate
  for (int i = 0; i<11; i++) {
    if (abs(ofs) > rateCtrl[i][0]) {
      rateLim = rateCtrl[i][1] * sign;
      if (abs(ofs)>100.0) {
        if (rateLim > (150.0 * dVs)) {
          rateLim = 150.0 * dVs;                   // Rate limits for realistic thrust ships - e.g. CEV Orion
        }
      } else if (abs(ofs)<10.0) {
        if (rateLim > (10.0 * dVs)) {
          rateLim = 10.0 * dVs;                    // Min maneuveing rate at low offsets
        }
      } else if (rateLim > ((150.0 * dVs) * (abs(ofs)/100.0))) {
        rateLim = (150.0 * dVs) * (abs(ofs)/100.0); // Taper from 150x dVs to 15x dVs as ofs comes from 100 down to 10
      }
      break;
    }
  }

  double noiseOfs = rateCtrl[11][0] * 0.50;       // Half the lowest offset limit
  double noiseRate = rateCtrl[11][1] * 0.80;   

  if (((rateLim < rateCtrl[11][1] * 1.01) && (abs(ofs) < noiseOfs) && (abs(ofsRate) < noiseRate))) { 
    thrust = 0.0;
    thrustEst = 0.0;
    return;                                     // Low enough that we don't need to do anything
  }

  if (ofs >= 0.0) {
    sign = (ofsRate > rateLim ? -1 : 1);
  } else {
    sign = (ofsRate < rateLim ? 1 : -1);
  }


  thrustEst = abs((rateLim - ofsRate) / dVs) / simStep;   // Estimated cycles to achieve target

  double thrustMod = 30.0;
  if (dVs < 2.0) thrustMod = 30.0;

  if (thrustEst > thrustMod) {                         // More than thrustMod cycles away ...  go to full thrust
    thrust = 1.0;
  } else {                                       
    thrust = thrustEst / thrustMod;                    // Binary chop to the end
  }

  double quietFac = 1.00;
 
  if (abs(rateLim)>0.005) {
    double rateErr = ofsRate/rateLim;                 // Bias the thrusters to overcome repeated non-capture of the desired rate
    if (rateErr < -0.25) {
      apErr += 0.05;                                  // Aggregate bias relative to percentage velocity error
    } else if (rateErr < 0.0) {
      apErr += 0.04;
    } else if (rateErr < 0.10) {
      apErr += 0.03;
    } else if (rateErr < 0.50) {
      apErr += 0.02;
    } else if (rateErr < 0.90) {
      apErr += 0.01;
    } else {
      apErr = (0.95 * apErr) - 0.02;   // Cool off the bias
      if (apErr < 1.0) apErr = 1.0; 
    }
    if (apErr>4.00) apErr = 4.0;
    thrust *= apErr;
  }
  if ((abs(rateLim-ofsRate) < abs(0.30*rateLim))&&((rateLim*ofsRate)>0.0)) thrust = 0.0; // Within 30% of target rate and on same sign ... go quiet
  if ((abs(ofs) < 0.02*quietFac) && (abs(ofsRate) < 0.01*quietFac)) thrust = 0.0; // Close on target, rate low ... go quiet

  if (thrust>1.0) thrust=1.0;

  if (sign == -1) {               
    v->IncThrusterGroupLevel(thrustPositive, thrust);
  } else {                        
    v->IncThrusterGroupLevel(thrustNegative, thrust);
  }

  thrust *= sign;
}
