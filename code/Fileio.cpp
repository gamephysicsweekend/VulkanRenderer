//
//	Fileio.cpp
//
#include "Fileio.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include <direct.h>
#define GetCurrentDir _getcwd

static char g_ApplicationDirectory[ FILENAME_MAX ];
static bool g_WasInitialized = false;

/*
=================================
InitializeFileSystem
=================================
*/
void InitializeFileSystem() {
	if ( g_WasInitialized ) {
		return;
	}
	g_WasInitialized = true;

	const bool result = GetCurrentDir( g_ApplicationDirectory, sizeof( g_ApplicationDirectory ) );
	assert( result );
	if ( result ) {
		printf( "ApplicationDirectory: %s\n", g_ApplicationDirectory );
	} else {
		printf( "ERROR: Unable to get current working directory!\n");
	}
}

/*
=================================
RelativePathToFullPath
=================================
*/
void RelativePathToFullPath( const char * relativePathName, char * fullPath ) {
	InitializeFileSystem();

	sprintf( fullPath, "%s/%s", g_ApplicationDirectory, relativePathName );
}

/*
====================================================
GetFileData
Opens the file and stores it in data
====================================================
*/
bool GetFileData( const char * fileNameLocal, unsigned char ** data, unsigned int & size ) {
	InitializeFileSystem();

	char fileName[ 2048 ];
	sprintf( fileName, "%s/%s", g_ApplicationDirectory, fileNameLocal );
	
	// open file for reading
	FILE * file = fopen( fileName, "rb" );
	
	// handle any errors
	if ( file == NULL ) {
		return false;
	}
	
	// get file size
	fseek( file, 0, SEEK_END );
	fflush( file );
	size = ftell( file );
	fflush( file );
	rewind( file );
	fflush( file );
	
	// create the data buffer
	*data = (unsigned char*)malloc( ( size + 1 ) * sizeof( unsigned char ) );
    
	// handle any errors
	if ( *data == NULL ) {
		printf( "ERROR: Could not allocate memory!  %s\n", fileName );
		fclose( file );
		return false;
	}

	// zero out the memory
	memset( *data, 0, ( size + 1 ) * sizeof( unsigned char ) );
	
	// read the data
	unsigned int bytesRead = (unsigned int)fread( *data, sizeof( unsigned char ), size, file );
    fflush( file );
    
    assert( bytesRead == size );
	
	// handle any errors
	if ( bytesRead != size ) {
		printf( "ERROR: reading file went wrong %s\n", fileName );
		fclose( file );
		if ( *data != NULL ) {
			free( *data );
		}
		return false;
	}
	
	fclose( file );
	printf( "Read file was success %s\n", fileName );
	return true;
}

/*
====================================================
SaveFileData
====================================================
*/
bool SaveFileData( const char * fileNameLocal, const void * data, unsigned int size ) {
	InitializeFileSystem();

	char fileName[ 2048 ];
	sprintf( fileName, "%s/%s", g_ApplicationDirectory, fileNameLocal );

	// open file for writing
	FILE * file = fopen( fileName, "wb" );
	
	// handle any errors
	if ( file == NULL ) {
		printf("ERROR: open file for write failed: %s\n", fileName );
		return false;
	}
	
	unsigned int bytesWritten = (unsigned int )fwrite( data, 1, size, file );
    assert( bytesWritten == size );
	
	// handle any errors
	if ( bytesWritten != size ) {
		printf( "ERROR: writing file went wrong %s\n", fileName );
		fclose( file );
		return false;
	}
	
	fclose( file );
	printf( "Write file was success %s\n", fileName );
	return true;
}