#pragma once

#include "Vector2.hpp"
#include "Vector3.hpp"
#include "Interface.hpp"

namespace GTA
{
	#pragma region Forward Declarations
	ref class Blip;
	ref class Camera;
	ref class Entity;
	ref class Ped;
	ref class Prop;
	ref class Rope;
	ref class Vehicle;
	ref class Pickup;
	value class Model;
	value class RaycastResult;
	#pragma endregion


	public ref class LidarScanner sealed
	{
	public:
		LidarScanner(float horizontalMin, float horizontalMax, int horizontalResultion,
			float verticalMin, float verticalMax, int verticalResolution,
			float range);
		
		void Scan(Math::Vector3 position, Math::Vector3 forward, Math::Vector3 right, Math::Vector3 up,
			array<float> ^depth, array<unsigned short> ^label);
		static unsigned short LidarScanner::GetLabelFromRaycast(RaycastResult &result);

		// Parameters for the simulated LIDAR scanner, self explaining
		float horizontalMin, horizontalMax, horizontalStep;
		int horizontalResolution;
		float verticalMin, verticalMax, verticalStep;
		int verticalResolution;
		float range;

		// Labels the scanner uses
		const static unsigned short LBL_UNK = 0;
		const static unsigned short LBL_PED = 1;
		const static unsigned short LBL_VEH = 2;
		const static unsigned short LBL_BICYCLE = 3;
		const static unsigned short LBL_BIKE = 4;
		const static unsigned short LBL_PED_IN_VEH = 5;
		const static unsigned short LBL_PED_ON_BICYCLE = 6;
		const static unsigned short LBL_PED_ON_BIKE = 7;
		const static unsigned short LBL_PROP = 8;
	};
}