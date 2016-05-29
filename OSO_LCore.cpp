// ==========================================================================
//
//	OSO (Local (Vessel+MFD Panel) Core Persistence)
//	================================================
//
//	Copyright (C) 2013	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See OSO.cpp
//
// ==========================================================================

#include "OSO_Cores.hpp"

OSO_LCore::OSO_LCore(VESSEL *vin, UINT mfdin, OSO_GCore* gcin) {
  GC = gcin;
  v = vin;
  m = mfdin;

  VC = (OSO_VCore*) GC->P.FindVC(v);
  MC = (OSO_MCore*) GC->P.FindMC(m);

  showMessage = false;
  return;
}


void OSO_LCore::SetMessage(const string &rawMessage) {
	
	showMessage = true;
	messagePage = 0;
	message.clear();

	string tmpMessage = rawMessage;
	int lineCount = 0;

	while (tmpMessage.length() >0) {
		string tmpLine = tmpMessage.substr(0,34);
		unsigned int i = tmpLine.find('\n');
		if ((i==string::npos)&&(tmpLine.length()==34)) {
			i = tmpLine.find_last_of(' ');
			if (i==string::npos || i<25) {
				i = 33;
				tmpMessage.insert(i++,"-\n");
			}
		}
		message.push_back(tmpMessage.substr(0,i));
		lineCount++;
		if (i!=string::npos) {
			tmpMessage.erase(0,i+1);
		} else {
			break;
		}
		if (tmpMessage.length() > 1) {
			if (tmpMessage[0] == ' ' && tmpMessage[1] != ' ') {
				tmpMessage.erase(0,1);
			}
		}
	}
	messagePageMax = lineCount/21;
	switchMenu = true;
	return;
}
