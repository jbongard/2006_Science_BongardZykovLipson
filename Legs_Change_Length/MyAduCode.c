// MyAduCode.c
// Demonstrates the AduHid DLL controlling an ADU device
//       from Ontrak Control Systems Inc. (www.ontrak.net)
// Copyright 2002 - Ontrak Control Systems Inc.



#include "stdafx.h"

#include "AduHid.h"
#include "stdio.h"  /* for the printf function */
#include <string.h>



void HoistPullRobot ()

{
    void * hDevice;
	char sBuffer[8];
  
	hDevice = OpenAduDevice(0);

	
	

	WriteAduDevice(hDevice, "sk0", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk0", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
    
	printf("Relay 0 is %s\n", sBuffer);

    
	WriteAduDevice(hDevice, "sk1", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk1", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
 
    printf("Relay 1 is %s\n", sBuffer);


	WriteAduDevice(hDevice, "rk2", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk2", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
 
    printf("Relay 2 is %s\n", sBuffer);


	WriteAduDevice(hDevice, "rk3", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk3", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
 
    printf("Relay 3 is %s\n", sBuffer);
	
	CloseAduDevice(hDevice);
}


void HoistReleaseString ()

{
    void * hDevice;
	char sBuffer[8];
  
	hDevice = OpenAduDevice(0);


	WriteAduDevice(hDevice, "rk0", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk0", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
    
	printf("Relay 0 is %s\n", sBuffer);

    
	WriteAduDevice(hDevice, "rk1", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk1", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
 
    printf("Relay 1 is %s\n", sBuffer);


	WriteAduDevice(hDevice, "sk2", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk2", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
 
    printf("Relay 2 is %s\n", sBuffer);


	WriteAduDevice(hDevice, "sk3", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk3", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
 
    printf("Relay 3 is %s\n", sBuffer);
	
	CloseAduDevice(hDevice);
}





void HoistIdle ()

{
    void * hDevice;
	char sBuffer[8];
  
	hDevice = OpenAduDevice(0);


	WriteAduDevice(hDevice, "rk0", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk0", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
    
	printf("Relay 0 is %s\n", sBuffer);

    
	WriteAduDevice(hDevice, "rk1", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk1", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
 
    printf("Relay 1 is %s\n", sBuffer);


	WriteAduDevice(hDevice, "rk2", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk2", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
 
    printf("Relay 2 is %s\n", sBuffer);


	WriteAduDevice(hDevice, "rk3", 4, 0, 0);
    WriteAduDevice(hDevice, "rpk3", 4, 0, 0);
    memset(sBuffer, 0, sizeof(sBuffer));
    ReadAduDevice(hDevice, sBuffer, 7, 0, 0);
 
    printf("Relay 3 is %s\n", sBuffer);
	
	CloseAduDevice(hDevice);
}




int main(int argc, char* argv[])
{
	HoistPullRobot();
	getch();


	HoistReleaseString();
	getch();

	
	HoistIdle();
	getch();


	return 0;


} // end main

