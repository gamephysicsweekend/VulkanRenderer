//
//  main.cpp
//
#include "application.h"

/*
====================================================
main
====================================================
*/
int main( int argc, char * argv[] ) {
	g_application = new Application;
	g_application->Initialize();

	g_application->MainLoop();

	delete g_application;
	return 0;
}
