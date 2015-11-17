
// CTR.h : main header file for the CTR application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols


// CCTRApp:
// See CTR.cpp for the implementation of this class
//

class CCTRApp : public CWinApp
{
public:
	CCTRApp();


// Overrides
public:
	virtual BOOL InitInstance();

// Implementation
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CCTRApp theApp;
