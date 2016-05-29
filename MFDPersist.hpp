// ==============================================================
//
//	MFDPersist
//	==========
//
//	Copyright (C) 2013	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	Description, Credits and Copyright Notice: see MFDPersist.cpp
//
//	Release History:
//
//	V1.00	Initial Release
//
// ==============================================================


#ifndef __MFDPersist
#define __MFDPersist

#include "windows.h"
#include "orbitersdk.h"
#include <list>
using namespace std;


class MFDPersistLookup {
  public:
    VESSEL *v;
    UINT  mfd;
    void* core;
};

class MFDPersist {
  public:
    MFDPersist();
    void* FindVC(VESSEL *v);
    void* FindMC(UINT mfd);
    void* FindLC(VESSEL *v, UINT mfd);
    void AddVC(VESSEL *v, void *VC);
    void AddMC(UINT mfd, void *MC);
    void AddLC(VESSEL *v, UINT mfd, void *LC);

  private:
    list<MFDPersistLookup*>  VCL;
    list<MFDPersistLookup*>  MCL;
    list<MFDPersistLookup*>  LCL;
    void* Find(list<MFDPersistLookup*> &L, VESSEL *v, UINT mfd);
    void Add(list<MFDPersistLookup*> &L, VESSEL *v, UINT mfd, void *C);
};

#endif // __MFDPersist