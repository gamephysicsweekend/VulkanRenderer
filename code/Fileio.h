//
//	Fileio.h
//
#pragma once

bool GetFileData( const char * fileName, unsigned char ** data, unsigned int & size );
bool SaveFileData( const char * fileName, const void * data, unsigned int size );