//======================================================================== 
// Project: OCTOR   
// ---------------------------------------------------------------------- 
// Package: PXC Series Continuous Display
// Authors: Vilas Kumar Chitrakaran
// Start Date: Tue Sep 30 09:30:10 EDT 2003
// Compiler: GCC 2.95.3
// Operating System: QNX 6.2.1 
// ----------------------------------------------------------------------  
// File: pxcDisplay.cpp
// The PXC series display program.
//========================================================================  

#include "PXCCont.hpp"

int main()
{
	PXCCont *window;

	window = new PXCCont;
	if( window->d_status.isStatusError() )
	{
		cout << window->d_status.getMessageText() << endl;
		return(-1);
	}
	window->run();
	delete window;

	return 0;
} 