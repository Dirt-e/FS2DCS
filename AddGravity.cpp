#include "AddGravity.h"

#define rad2deg 57.2957795131
#define deg2rad 1.0/rad2deg
#define Gravity 1

double AddGravityToAccLon(double CoordAccLon, double pitch_rad)
{
	double GravityLon = sin(pitch_rad) * Gravity;

	return CoordAccLon + GravityLon;
}

double AddGravityToAccVert(double CoordAccVert, double pitch_rad, double roll_rad)
{
	double GravityVert = cos(pitch_rad) * cos(roll_rad) * Gravity;

	return CoordAccVert + GravityVert;
}

double AddGravityToAccLat(double CoordAccLat, double pitch_rad, double roll_rad)
{
	double GravityLat = cos(pitch_rad) * sin(roll_rad) * Gravity;

	return CoordAccLat + GravityLat;
}