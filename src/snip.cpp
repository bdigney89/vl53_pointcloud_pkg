#include <stdio.h>
#include "vl53l5cx_plugin_plane_algo.h"
#include "math.h"
 
 
const double VL53L5_Zone_Pitch8x8[64] = {
62.85,	66.50,	69.40,	71.08,	71.08,	69.40,	66.50,	62.85,
66.50,	70.81,	75.05,	77.50,	77.50,	75.05,	70.81,	66.50,
69.40,	75.05,	78.15,	81.76,	81.76,	78.15,	75.05,	69.40,
71.08,	77.50,	81.76,	86.00,	86.00,	81.76,	77.50,	71.08,
71.08,	77.50,	81.76,	86.00,	86.00,	81.76,	77.50,	71.08,
69.40,	75.05,	78.15,	81.76,	81.76,	78.15,	75.05,	69.40,
66.50,	70.81,	75.05,	77.50,	77.50,	75.05,	70.81,	66.50,
62.85,	66.50,	69.40,	71.08,	71.08,	69.40,	66.50,	62.85
};
 
 
const double VL53L5_Zone_Yaw8x8[64] = {
135.00, 125.40, 113.20, 98.13,  81.87, 66.80, 54.60, 45.00,
144.60, 135.00, 120.96, 101.31, 78.69, 59.04, 45.00, 35.40,
156.80, 149.04, 135.00, 108.45, 71.55, 45.00, 30.96, 23.20,
171.87, 168.69, 161.55, 135.00, 45.00, 18.45, 11.31,  8.13,
188.13, 191.31, 198.45, 225.00, 315.00, 341.55, 348.69, 351.87,
203.20, 210.96, 225.00, 251.55, 288.45, 315.00, 329.04, 336.80,
203.20, 225.00, 239.04, 258.69, 281.31, 300.96, 315.00, 324.60,
225.00, 234.60, 246.80, 261.87, 278.13, 293.20, 305.40, 315.00
};
 
double SinOfPitch[64], CosOfPitch[64], SinOfYaw[64], CosOfYaw[64];
 
uint8_t ComputeSinCosTables(void)
{
	uint8_t ZoneNum;
	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
	{
		SinOfPitch[ZoneNum] = sin((VL53L5_Zone_Pitch8x8[ZoneNum])*Pi/180);
		CosOfPitch[ZoneNum] = cos((VL53L5_Zone_Pitch8x8[ZoneNum])*Pi/180);
		SinOfYaw[ZoneNum] = sin(VL53L5_Zone_Yaw8x8[ZoneNum]*Pi/180);
		CosOfYaw[ZoneNum] = cos(VL53L5_Zone_Yaw8x8[ZoneNum]*Pi/180);
	}
 
	return 0;
}
 
uint8_t ConvertDist2XYZCoords8x8(
		VL53L5CX_ResultsData *p_ResultsData,
		XYZ_Coord_t *p_XYZ_ZoneCoordinates)
{
	uint8_t ZoneNum;
	double Hyp;
	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
	{
		if ((p_ResultsData->nb_target_detected[ZoneNum] > 0)
				&& (p_ResultsData->distance_mm[ZoneNum] > 0)
				&& ((p_ResultsData->target_status[ZoneNum] == 5)
						|| (p_ResultsData->target_status[ZoneNum] == 6)
						|| (p_ResultsData->target_status[ZoneNum] == 12)
						|| (p_ResultsData->target_status[ZoneNum] == 9)) )
		{
			Hyp = p_ResultsData->distance_mm[ZoneNum]/SinOfPitch[ZoneNum];
			p_XYZ_ZoneCoordinates->Xpos[ZoneNum] =
					CosOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp;
			p_XYZ_ZoneCoordinates->Ypos[ZoneNum] =
					SinOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp;
			p_XYZ_ZoneCoordinates->Zpos[ZoneNum] =
					p_ResultsData->distance_mm[ZoneNum];
		}
		else
		{
			p_XYZ_ZoneCoordinates->Xpos[ZoneNum] = 0;
			p_XYZ_ZoneCoordinates->Ypos[ZoneNum] = 0;
			p_XYZ_ZoneCoordinates->Zpos[ZoneNum] = 0;
		}
	}
	return 0;
}