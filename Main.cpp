#include "UDP_Talker.h"
#include <Windows.h>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <ctime>
#include <mutex>

#include "SimConnect.h"
#include "AddGravity.h"
using namespace std;

//Mutex
string s = "YAME";
LPCWSTR mtx = (LPCWSTR)s.c_str();

HANDLE hSimConnect = NULL;
UDP_Talker udp_talker(4124);

bool quit = false;
int Counter = 0;
double Time = 0;
double deltaTime = 0;
double prevTime = 0;

enum DATA_DEFINE_ID {
	DEFINITION_MOTIONDATA,
	//...
};
enum DATA_REQUEST_ID {
	REQUEST_MOTIONDATA,
	//...
};

struct SimResponse {
	double IAS;					//	[0]															
	double MACH;				//	[1]															
	double TAS;					//	[2]															
	double GS;					//	[3]												
	double AOA;					//	[4]												
	double VS;					//	[5]												
	double HGT;					//	[6]

	double BANK;				//	[7]
	double HDG;					//	[8]												
	double PITCH;				//	[9]

	double WX;					//	[10]												
	double WY;					//	[11]												
	double WZ;					//	[12]

	double WX_dot;				//	[13]												
	double WY_dot;				//	[14]												
	double WZ_dot;				//	[15]

	double AX;					//	[16]											
	double AY;					//	[17]												
	double AZ;					//	[18]

	double TIME;				//	[19]	Not Used!
};

void CALLBACK ProcessData(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext) {
	switch (pData->dwID)
	{
	case SIMCONNECT_RECV_ID_SIMOBJECT_DATA:
	{
		SIMCONNECT_RECV_SIMOBJECT_DATA* pObjData = (SIMCONNECT_RECV_SIMOBJECT_DATA*)pData;
		SimResponse* pS = (SimResponse*)&pObjData->dwData;
		switch (pObjData->dwRequestID)
		{
		case REQUEST_MOTIONDATA:
			vector<string> v_MotionData;

			//Add Gravity (Caution: Pitch is inverted!!!)
			double accLonWithGrav = AddGravityToAccLon(pS->AX, -pS->PITCH);
			double accVertWithGrav = AddGravityToAccVert(pS->AY, -pS->PITCH, pS->BANK);
			double accLatWithGrav = AddGravityToAccLat(pS->AZ, -pS->PITCH, pS->BANK);
			v_MotionData.push_back(to_string(accLonWithGrav));	//	[00]
			v_MotionData.push_back(to_string(accLatWithGrav));	//	[01]
			v_MotionData.push_back(to_string(accVertWithGrav));	//	[02]

			v_MotionData.push_back("0.01");						//	[03]	[Speed_lon]
			v_MotionData.push_back("0.01");						//	[04]	[Speed_lat]
			v_MotionData.push_back(to_string(pS->VS));			//	[05]

			v_MotionData.push_back(to_string(-pS->WY));			//	[06]	angular rates
			v_MotionData.push_back(to_string(pS->WX));			//	[07]
			v_MotionData.push_back(to_string(-pS->WZ));			//	[08]

			v_MotionData.push_back(to_string(pS->HDG));			//	[09]
			v_MotionData.push_back(to_string(-pS->BANK));		//	[10]
			v_MotionData.push_back(to_string(-pS->PITCH));		//	[11]

			v_MotionData.push_back(to_string(pS->TAS));			//	[12]

			v_MotionData.push_back("0.01");						//	[13]
			v_MotionData.push_back("0.01");						//	[NLG]
			v_MotionData.push_back("0.01");						//	[MLG_L]
			v_MotionData.push_back("0.01");						//	[MLG_R]

			v_MotionData.push_back(to_string(pS->HGT));			//	[17]

			v_MotionData.push_back("0.00");						//	[18]	[MechInfo Flaps		(0/1)]
			v_MotionData.push_back("0.00");						//	[19]	[MechInfo Gear		(0/1)]
			v_MotionData.push_back("0.00");						//	[20]	[MechInfo Spdbrk	(0/1)]
			v_MotionData.push_back("0.00");						//	[21]	[MechInfo Canopy	(0/1)]

			v_MotionData.push_back("0.0");						//	[22]	[Stall Warning (0/1)]

			v_MotionData.push_back(to_string(pS->AOA));			//	[23]
			v_MotionData.push_back(to_string(pS->TIME));		//	[24]

			//Calculate times:
			//double now = std::chrono::system_clock::now().time_since_epoch().count() / 10000000.0;
			//deltaTime = now - prevTime;
			//prevTime = now;
			//v_MotionData.push_back(to_string(now));				//	[19]
			//v_MotionData.push_back(to_string(deltaTime));		//	[20]
			//v_MotionData.push_back(to_string(Counter++));		//	[21]
			//v_MotionData.push_back("FS2020");					//	[22]

			//Now send the data via UDP
			udp_talker.Talk(v_MotionData);

			break;
		}
		break;
	}

	case SIMCONNECT_RECV_ID_QUIT:
	{
		quit = true;
		break;
	}

	default:
		break;
	}
}

bool initSimEvents()
{
	// DATA
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "AIRSPEED INDICATED", "meter per second");						//[0]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "AIRSPEED MACH", "MACH");										//[1]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "AIRSPEED TRUE", "meter per second");							//[2]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "GROUND VELOCITY", "meter per second");							//[3]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "INCIDENCE ALPHA", "degree");									//[4]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "VERTICAL SPEED", "meter per second");							//[5]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "RADIO HEIGHT", "meter");										//[6]

	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "PLANE BANK DEGREES", "radian");									//[7]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "PLANE HEADING DEGREES TRUE", "radian");							//[8]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "PLANE PITCH DEGREES", "radian");								//[9]

	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "ROTATION VELOCITY BODY Z", "radian per second");				//[10]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "ROTATION VELOCITY BODY Y", "radian per second");				//[11]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "ROTATION VELOCITY BODY X", "radian per second");				//[12]

	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "ROTATION ACCELERATION BODY Z", "degree per second squared");	//[13]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "ROTATION ACCELERATION BODY Y", "degree per second squared");	//[14]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "ROTATION ACCELERATION BODY X", "degree per second squared");	//[15]

	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "ACCELERATION BODY Z", "Gforce");				//[16]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "ACCELERATION BODY Y", "Gforce");				//[17]
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "ACCELERATION BODY X", "Gforce");				//[18]

	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_MOTIONDATA, "ZULU TIME", "second");											//[19]


	SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_MOTIONDATA, DEFINITION_MOTIONDATA, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_VISUAL_FRAME);

	// Process incoming SimConnect Server messages
	while (!quit)
	{
		// Continuously call SimConnect_CallDispatch until quit - ProcessData will handle simulation events
		SimConnect_CallDispatch(hSimConnect, ProcessData, NULL);
		Sleep(1);
	}

	SimConnect_Close(hSimConnect);
	cout << endl << "FS2020 closed :-(" << endl << endl;

	quit = false;			//to enable another start when the sim reconnects
	return true;
}

int main()
{
	HANDLE hMutex = OpenMutex(MUTEX_ALL_ACCESS, 0, mtx);			// Try to open the mutex.

	if (!hMutex)													//Mutex doesn't exist --> We are the only instance
	{
		hMutex = CreateMutex(0, 0, mtx);
		cout << "Starting FS2020_to_DCS converter (\"Hexpod special edition\")" << endl;
		cout << "Version: v0.01" << endl << endl;

		while (true)
		{
			cout << "Waiting for Connection...";

			while (!SUCCEEDED(SimConnect_Open(&hSimConnect, "Client Event Demo", NULL, 0, NULL, 0)))
			{
				cout << '.';
				Sleep(300);
			}

			cout << endl << "Connected to FS2020 :-)" << endl << endl;

			initSimEvents();
		}
	}
	else
	{
		return 0;													// The mutex exists so this is the second instance --> Close the application
	}
}