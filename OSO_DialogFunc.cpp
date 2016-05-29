// ==============================================================
//
//	OSO (Dialog Function Handlers)
//	===============================
//
//	Copyright (C) 2013	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See OSO.cpp
//
// ==============================================================

#include "OnStationOps.hpp"
#include "OSO_DialogFunc.hpp"
#include "OSO_Cores.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <math.h>

// Callback from OSO FT Input Box
bool OSO_DialogFunc::clbkFT(void *id, char *str, void *usrdata) {
  return clbkTgt(id,str,usrdata,false);
}
// Callback from OSO TT Input Box
bool OSO_DialogFunc::clbkTT(void *id, char *str, void *usrdata) {
  return clbkTgt(id,str,usrdata,true);
}

// Callback from PYR Input Box
bool OSO_DialogFunc::clbkPYR(void *id, char *str, void *usrdata) {
	float paramf[3];
  if (sscanf_s(str,"%f %f %f",&(paramf[0]),&(paramf[1]),&(paramf[2]))<3) return true;

	for (int i=0; i<3; i++) {
		if (paramf[i]<-360.0 || paramf[i]>=360.0) {
			return true;
		}
	}

	OSO_LCore* LC = (OSO_LCore*) usrdata;
//  OSO_GCore* GC = LC->GC;
  OSO_VCore* VC = LC->VC;

	VC->rOfsStr = str;
	for (int i=0; i<3; i++) {
		VC->rOfs.data[i] = paramf[i];
	}
	return true;
}

// Callback from XYZ Input Box
bool OSO_DialogFunc::clbkXYZ(void *id, char *str, void *usrdata) {
	float paramf[3];
  if (sscanf_s(str,"%f %f %f",&(paramf[0]),&(paramf[1]),&(paramf[2]))<3) return true;

	OSO_LCore* LC = (OSO_LCore*) usrdata;
//  OSO_GCore* GC = LC->GC;
  OSO_VCore* VC = LC->VC;

	VC->tOfsStr = str;
	for (int i=0; i<3; i++) {
		VC->tOfs.data[i] = paramf[i];
	}
	return true;
}

// Callback from Dist Input Box
bool OSO_DialogFunc::clbkDist(void *x, char *str, void *usrdata) {
	float paramf;
  if (sscanf_s(str,"%f",&paramf)<1) return true;

	OSO_LCore* LC = (OSO_LCore*) usrdata;
//  OSO_GCore* GC = LC->GC;
  OSO_VCore* VC = LC->VC;

	VC->range = id(VC->unit, paramf);
	return true;
}


// Common target handler
bool OSO_DialogFunc::clbkTgt(void *id, char *str, void *usrdata, bool ttDlg) {
	char aVc[50];
	string answer = str;
	string ansVes;
	string ansPos;
	unsigned int colon;
	OBJHANDLE hTgtV;
	VESSEL* tgtV;
	DOCKHANDLE hDock = nullptr;
	ATTACHMENTHANDLE hAtt = nullptr;
	bool attParent = true;
	VECTOR3 rPos = {0,0,0};
	VECTOR3 rDir = {0,0,0};
	VECTOR3 rRot = {0,0,0};
	OSO_VCore::TGT_STRUCT *t;
	std::stringstream ss;


	if (strlen(str) == 0) return true;		// Assume canceled dialog

  OSO_LCore* LC = (OSO_LCore*) usrdata;
//  OSO_GCore* GC = LC->GC;
  OSO_VCore* VC = LC->VC;

	if (ttDlg) {
		t = &(VC->tgt[1]);
	} else {
		t = &(VC->tgt[0]);
	}

	if (answer == "?") {
		// Help mode
		LC->SetMessage("\
Target selection help:\n\
\n\
Use one of these formats:\n\
\n\
1. Target     (selects center)\n\
2. Target:nn  (selects port nn)\n\
3. Target:Ann (attachment nn)\n\
4. T?         (list targets)\n\
5. Target:?   (list ports)\n\
6. Target:A?  (list attachments)\n\
7. ?          (this help screen)\n\
\n\
Target can be a name or a number (see T?).\n\
\n\
Examples:\n\
OFSS    ... selects OFSS center\n\
OFSS:1  ... selects OFSS port 1\n\
OFSS:A1 ... selects attachment 1\n\
3:2     ... selects port 2 of 3rd vessel\n\
T?      ... list possible targets\n\
OFSS:?  ... list OFSS ports\n\
OFSS:A? ... list OFSS attachments\n\
\n\
\n\
Description:\n\
\n\
You are selecting two targets in this MFD: a tethered target and a free-floating target. \
The tethered target is usually a manipulator arm or a module attached to the arm. \
The free-floating target is usually a partially-built space station or other module \
you are attaching things to, or grappling for the first time. The tethered target \
can also be our own ship.\
\n\n\
On each target, you can select the coordinate origin of the \
target (just type the target name), a docking port (type target name followed by : \
and a number), or an attachment point (type target name followed by :A and a number). \
\n\n\
You can list out the available targets using T?. For a specific target, you can also \
list out the available ports and details (type target name followed by :?), or the \
available attachment points and details (type target name followed by :A?). \
\n\n\
Hit OK to continue."
			);
		return true;
	}

	if (answer == "t?" || answer == "T?" ) {
		// target list mode
		ss << "Target vessel list:\n\n";
		for (unsigned int i=0; i<oapiGetVesselCount(); i++) {
			hTgtV = oapiGetVesselByIndex(i);
			tgtV = oapiGetVesselInterface(hTgtV);
			ss << std::dec << (i+1) << " " << tgtV->GetName() << "\n";
		}
		LC->SetMessage(ss.str());
		return true;
	}

	// Split answer into vessel and port
	colon = answer.find(":");
	if (colon == string::npos) {
		ansVes = answer;
		ansPos = "";
	} else {
		ansVes = answer.substr(0,colon);
		ansPos = answer.substr(colon+1,string::npos);
	}

	// Shortcut ... . for us
	if (ansVes == ".") ansVes = VC->v->GetName();

	// See if ansVes is a vessel
	strcpy_s(aVc, ansVes.c_str());
	hTgtV = oapiGetVesselByName(aVc);
	if (hTgtV == nullptr) {
		long int i;
		char *p;
		i = strtoul(ansVes.c_str(),&p,10);
		if (i>0 && i<=(long int) oapiGetVesselCount() && *p == '\0') {
			hTgtV = oapiGetVesselByIndex(i-1);
		} else {
			LC->SetMessage(string() + "Unrecognized target: " + ansVes);
			return true;
		}
	}
	tgtV = oapiGetVesselInterface(hTgtV);		// good vessel

	// If TT, then check tethered, else check untethered
	if (VC->IsConnectedToMe(VC->v,tgtV) != ttDlg) {
		if (ttDlg) {
			LC->SetMessage(string() + "Error:\nTarget " + ansVes + " isn't attached.\n\nPlease select a target that is tethered to us, or select our vessel.");
		} else {
			LC->SetMessage(string() + "Error:\nTarget " + ansVes + " is attached.\n\nPlease select a vessel that is not attached to us (or is not us).");
		}
		return true;
	}

	if (ansPos != "") {

		// Assess pos data
		if (ansPos == "?") { // Port list
			ss << "Target vessel: " << tgtV->GetName() << "\n\nPorts:\n" << setprecision(2) << fixed;
			for (unsigned int i=0; i<tgtV->DockCount(); i++) {
				hDock = tgtV->GetDockHandle(i);
				tgtV->GetDockParams(hDock,rPos,rDir,rRot);
				ss << (i+1) << " {" << rPos.x << ", " << rPos.y << ", " << rPos.z << "}\n";
			} 
			if (tgtV->DockCount() == 0) {
				ss << "(none)\n";
			}
			LC->SetMessage(ss.str());
			return true;
		}

		unsigned int pCnt= tgtV->AttachmentCount(true), cCnt = tgtV->AttachmentCount(false);
		//
		if (ansPos == "a?" || ansPos == "A?" ) { // Attachment list

			ss << "Target vessel: " << tgtV->GetName() << "\n\nAttachments:\n" << setprecision(2) << fixed;
			for (unsigned int i=0; i<pCnt; i++) {
				hAtt = tgtV->GetAttachmentHandle(true, i);
				tgtV->GetAttachmentParams(hAtt,rPos,rDir,rRot);
				ss << (i+1) << " " << tgtV->GetAttachmentId(hAtt) << " p{" << rPos.x << ", " << rPos.y << ", " << rPos.z << "}\n";
			}
			for (unsigned int i=0; i<cCnt; i++) {
				hAtt = tgtV->GetAttachmentHandle(false, i);
				tgtV->GetAttachmentParams(hAtt,rPos,rDir,rRot);
				ss << (i+1+pCnt) << " " << tgtV->GetAttachmentId(hAtt) << " c{" << rPos.x << ", " << rPos.y << ", " << rPos.z << "}\n";
			}
			if (pCnt + cCnt == 0) {
				ss << "(none)\n";
			}
			LC->SetMessage(ss.str());
			return true;
		}

		if (ansPos[0] == 'A' || ansPos[0] == 'a') { // Parse target:Ann attachment selection
			unsigned long int i;
			char *p;
			ansPos[0] = 'A';
			i = strtoul(ansPos.substr(1,string::npos).c_str(),&p,10);
			if (i>0 && i<=(pCnt + cCnt) && *p == '\0') {
				i--;
				if (i<pCnt) {
					attParent = true;
					hAtt = tgtV->GetAttachmentHandle(attParent, i);
					tgtV->GetAttachmentParams(hAtt,rPos,rDir,rRot);
				} else {
					attParent = false;
					hAtt = tgtV->GetAttachmentHandle(attParent, i-pCnt);
					tgtV->GetAttachmentParams(hAtt,rPos,rDir,rRot);
				}
			} else {
				LC->SetMessage(string() + "Unrecognized target: " + ansVes);
				return true;
			}
		} else { // Parse target:nn port selection
			unsigned long int i;
			char *p;
			i = strtoul(ansPos.c_str(),&p,10);
			if (i>0 && i<=tgtV->DockCount() && *p == '\0') {
				i--;
				hDock = tgtV->GetDockHandle(i);
				tgtV->GetDockParams(hDock,rPos,rDir,rRot);
			} else {
				LC->SetMessage(string() + "Unrecognized target: " + ansVes);
				return true;
			}
		}
	}

	t->label = tgtV->GetName();
	t->hTgtV = hTgtV;
	t->tgtV = tgtV;
	if (ansPos != "") {
		t->label += ":" + ansPos;
	}
	t->hasDock = (hDock != nullptr);
	t->hasAtt = (hAtt != nullptr);
	t->attParent = attParent;
	t->hTgtD = hDock;
	t->hTgtA = hAtt;
	t->rPos = rPos;
	t->rDir = (ttDlg? rDir : -rDir); // Free Target direction points into the port
	t->rRot = rRot;
	return true;
}

double OSO_DialogFunc::id( bool unit, double d ) {
  // Internalizes distances (if in US, converts to meters)
	double r = (unit? d : d / 3.2808399);
	if (abs(r) < 0.0001) r = 0.0; // noise suppressor
  return r;
}