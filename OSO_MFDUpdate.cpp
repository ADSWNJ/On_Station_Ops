// ==============================================================
//
//	OSO (MFD Update)
//	=================
//
//	Copyright (C) 2013	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See OSO.cpp
//
// ==============================================================

#include "OnStationOps.hpp"

bool OnStationOps::Update (oapi::Sketchpad *skp)
{

  Title (skp, "OnStationOps");
	// Draws the MFD title

  int l = 4;
  char buf[128];

	skp->SetTextAlign (oapi::Sketchpad::LEFT, oapi::Sketchpad::BOTTOM);
	skp->SetTextColor (0x00FFFF);
	if (LC->showMessage) {
		ShowMessage(skp);
		return true;
	}



  if (VC->tgt[1].tgtV) {
    sprintf_s(buf, 128, "TT:   %s", VC->tgt[1].label.c_str() );
		if (VC->IsConnectedToMe(VC->v,VC->tgt[1].tgtV)  ) {
      sprintf_s(buf, 128, "%s ++", buf);
    }
  } else {
    sprintf_s(buf, 128, "TT:   %s", "<not set>");
  }
	skp->Text (Col(0), Line(l), buf, strlen(buf));

	if (VC->apT) {
		strcpy_s(buf, 128, (VC->apT==1? ">> ZRT active <<":">> APT active <<"));
		skp->Text (Col(3), Line(l), buf, strlen(buf));
	}
	l++;

  if (VC->tgt[0].tgtV) {
    sprintf_s(buf, 128, "FT:   %s", VC->tgt[0].label.c_str() );
		if (VC->IsConnectedToMe(VC->v,VC->tgt[0].tgtV)  ) {
      sprintf_s(buf, 128, "%s ++", buf);
    }
  } else {
    sprintf_s(buf, 128, "FT:   %s", "<not set>");
  }
	skp->Text (Col(0), Line(l), buf, strlen(buf));

	if (VC->apR) {
		strcpy_s(buf, 128, (VC->apR==1? ">> ZRR active <<":">> APR active <<"));
		skp->Text (Col(3), Line(l), buf, strlen(buf));
	}
	l++;

	if (VC->tgt[0].tgtV == nullptr || VC->tgt[1].tgtV == nullptr) {
		return true;
	}

	sprintf_s(buf, 128, "%s%s%s%s", (VC->apRanRP? "P":" "), (VC->apRanRY? "Y":" "), (VC->apRanRR? "R":" "), (VC->apRanT? "XYZ":"   "));
	skp->Text (Col(3), Line(l), buf, strlen(buf));

  sprintf_s(buf, 128, "Dist: %.2f %s", ed(VC->range), (VC->unit? "m" : "ft" ));
	skp->Text (Col(0), Line(l), buf, strlen(buf));
	
	l++; l++;
	if (!VC->ratesValid) return true;

	VC->dispRotA = VC->l.rotA;
	VC->dispRotARate = VC->l.rotARate;
	VC->dispPos = VC->l.pos;
	VC->dispPosRate = VC->l.posRate;

	for (int i=0; i<3; i++) {
		if (abs(VC->dispRotA.data[i])<0.00020) VC->dispRotA.data[i] = 0.0;
		if (abs(VC->dispRotARate.data[i])<0.00020) VC->dispRotARate.data[i] = 0.0;
		if (abs(VC->dispPos.data[i])<0.00005) VC->dispPos.data[i] = 0.0;
		if (abs(VC->dispPosRate.data[i])<0.00005) VC->dispPosRate.data[i] = 0.0;
	}

	l++;
  sprintf_s(buf, 128, "      Att     Rate    Action");
  skp->Text (Col(0), Line(l++), buf, strlen(buf));


  if (abs(VC->dispRotA.x)<0.5) {
      sprintf_s(buf, 128, "P%9.2f%9.3f    ", VC->dispRotA.x, VC->dispRotARate.x);
  } else if (VC->dispRotA.x>0) {
    sprintf_s(buf, 128, "P%9.2f%9.3f   Pitch Up", VC->dispRotA.x, VC->dispRotARate.x);
  } else {
    sprintf_s(buf, 128, "P%9.2f%9.3f   Pitch Down", VC->dispRotA.x, VC->dispRotARate.x);
  }
  skp->Text (Col(0), Line(l++), buf, strlen(buf));
	
	if (abs(VC->dispRotA.y)<0.5) {
      sprintf_s(buf, 128, "Y%9.2f%9.3f    ", VC->dispRotA.y, VC->dispRotARate.y);
  } else if (VC->dispRotA.y>0) {
    sprintf_s(buf, 128, "Y%9.2f%9.3f   Yaw Right", VC->dispRotA.y, VC->dispRotARate.y);
  } else {
    sprintf_s(buf, 128, "Y%9.2f%9.3f   Yaw Left", VC->dispRotA.y, VC->dispRotARate.y);
  }
  skp->Text (Col(0), Line(l++), buf, strlen(buf));

  if (abs(VC->dispRotA.z)<0.5) {
      sprintf_s(buf, 128, "R%9.2f%9.3f    ", VC->dispRotA.z, VC->dispRotARate.z);
  } else if ((VC->dispRotA.z>0)&&(abs(VC->dispRotA.y)<=90.0)) {
    sprintf_s(buf, 128, "R%9.2f%9.3f   Roll Right", VC->dispRotA.z, VC->dispRotARate.z);
  } else {
    sprintf_s(buf, 128, "R%9.2f%9.3f   Roll Left", VC->dispRotA.z, VC->dispRotARate.z);
  }
  skp->Text (Col(0), Line(l++), buf, strlen(buf));


  l++;
	if (VC->unit) {
		sprintf_s(buf, 128, "m     Pos     Vel     Action");
	} else {
		sprintf_s(buf, 128, "ft    Pos     Vel     Action");
	}
  skp->Text (Col(0), Line(l++), buf, strlen(buf));

  if (abs(VC->dispPos.x)<0.04) {
      sprintf_s(buf, 128, "X%9.2f%9.3f    ", ed(VC->dispPos.x), ed(VC->dispPosRate.x));
  } else if (VC->dispPos.x>0) {
    sprintf_s(buf, 128, "X%9.2f%9.3f   Go Right", ed(VC->dispPos.x), ed(VC->dispPosRate.x));
  } else {
    sprintf_s(buf, 128, "X%9.2f%9.3f   Go Left", ed(VC->dispPos.x), ed(VC->dispPosRate.x));
  }
  skp->Text (Col(0), Line(l++), buf, strlen(buf));

	if (abs(VC->dispPos.y)<0.04) {
      sprintf_s(buf, 128, "Y%9.2f%9.3f    ", ed(VC->dispPos.y), ed(VC->dispPosRate.y));
  } else if (VC->dispPos.y>0) {
    sprintf_s(buf, 128, "Y%9.2f%9.3f   Go Up", ed(VC->dispPos.y), ed(VC->dispPosRate.y));
  } else {
    sprintf_s(buf, 128, "Y%9.2f%9.3f   Go Down", ed(VC->dispPos.y), ed(VC->dispPosRate.y));
  }
  skp->Text (Col(0), Line(l++), buf, strlen(buf));

  if (abs(VC->dispPos.z)<0.04) {
      sprintf_s(buf, 128, "Z%9.2f%9.3f    ", ed(VC->dispPos.z), ed(VC->dispPosRate.z));
  } else if (VC->dispPos.z>0) {
    sprintf_s(buf, 128, "Z%9.2f%9.3f   Go Forwards", ed(VC->dispPos.z), ed(VC->dispPosRate.z));
  } else {
    sprintf_s(buf, 128, "Z%9.2f%9.3f   Go Backwards", ed(VC->dispPos.z), ed(VC->dispPosRate.z));
  }
  skp->Text (Col(0), Line(l++), buf, strlen(buf));

	l++;
	sprintf_s(buf, 128, "Offsets:");
  skp->Text (Col(0), Line(l++), buf, strlen(buf));
	l++;
	sprintf_s(buf, 128, "TT P:%6.2f,  Y:%6.2f,  R:%6.2f", ed(VC->tgt[1].l.rotA.x), ed(VC->tgt[1].l.rotA.y), ed(VC->tgt[1].l.rotA.z));
  skp->Text (Col(0), Line(l++), buf, strlen(buf));
	sprintf_s(buf, 128, "TT X:%6.2f,  Y:%6.2f,  Z:%6.2f", ed(VC->tgt[1].l.pos.x), ed(VC->tgt[1].l.pos.y), ed(VC->tgt[1].l.pos.z));
  skp->Text (Col(0), Line(l++), buf, strlen(buf));
	sprintf_s(buf, 128, "RQ P:%6.2f,  Y:%6.2f,  R:%6.2f", ed(VC->rOfs.x), ed(VC->rOfs.y), ed(VC->rOfs.z));
  skp->Text (Col(0), Line(l++), buf, strlen(buf));
	sprintf_s(buf, 128, "RQ X:%6.2f,  Y:%6.2f,  Z:%6.2f", ed(VC->tOfs.x), ed(VC->tOfs.y), ed(VC->tOfs.z));
  skp->Text (Col(0), Line(l++), buf, strlen(buf));

	return true;
}

// MFD Line formatting helper
void OnStationOps::ShowMessage(oapi::Sketchpad *skp) {

  int l = 3;
	if (LC->switchMenu == true) {
		if (LC->messagePageMax == 0) {
			LC->B.SwitchPage(this,1); // OK menu
		} else {
			LC->B.SwitchPage(this,2); // OK NXT menu
		}
		LC->switchMenu = false;
	}

	unsigned int startLine = LC->messagePage * 21;
	unsigned int endLine = startLine + 21;
	if (endLine > LC->message.size()) {
		endLine = LC->message.size();
	}

	for (unsigned int i=startLine; i<endLine; i++) {
		skp->Text (Col(0), Line(l++), LC->message[i].c_str(), LC->message[i].length());
	}

  return;
}


// MFD Positioning Helper Functions
int OnStationOps::Line( int row ) {  // row is 0-24, for 24 rows. e.g. Line(12)
  int ret;
  ret = (int) ((H-(int)(ch/4)) * row / 25) + (int) (ch/4);
  return ret;
};

int OnStationOps::Col( int pos ) {  // pos is 0-5, for 6 columns. Eg Col(3) for middle
  int ret = (int) ((W-(int)(cw/2)) * pos / 6) + int (cw/2);
  return ret;
};

int OnStationOps::Col2( int pos ) {  // pos is 0-11, for 12 columns. Eg Col(6) for middle
  int ret = (int) ((W-(int)(cw/2)) * pos / 12) + int (cw/2);
  return ret;
};


// MFD + HUD Conversion Routines

double OnStationOps::id( double d ) const {
  // Internalizes distances (if in US, converts to meters)
	double r = (VC->unit? d : d / 3.2808399);
	if (abs(r) < 0.0001) r = 0.0; // noise suppressor
  return r;
}

double OnStationOps::ed( double d ) const {
  // Externalizes distances (if in US, converts to feet)
  double r = (VC->unit? d : d * 3.2808399);
	if (abs(r) < 0.0001) r = 0.0; // noise suppressor
	return r;
}
