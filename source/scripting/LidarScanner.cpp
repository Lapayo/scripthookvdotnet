#include "LidarScanner.hpp"
#include "Native.hpp"
#include "NativeMemory.hpp"
#include "ScriptDomain.hpp"

#include "World.hpp"
#include "Entity.hpp"
#include "Vehicle.hpp"
#include "Ped.hpp"
#include "Prop.hpp"
#include "Raycast.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

namespace GTA
{
	using namespace System;
	using namespace System::Collections::Generic;

	private ref struct NativeLidarScan : IScriptTask
	{
		virtual void Run()
		{
			for (int v = 0; v < scanner->verticalResolution; v++)
			{
				for (int h = 0; h < scanner->horizontalResolution; h++)
				{
					double verticalAngle = (scanner->verticalMin + v * scanner->verticalStep) * M_PI / 180.0 + M_PI_2;
					double horizontalAngle = (scanner->horizontalMin + h * scanner->horizontalStep) * M_PI / 180.0 + M_PI;

					float x = scanner->range * (float)(std::sin(verticalAngle) * std::cos(horizontalAngle));
					float y = scanner->range * (float)(std::sin(verticalAngle) * std::sin(horizontalAngle));
					float z = scanner->range * (float)(std::cos(verticalAngle));

					Math::Vector3 dest = x * forward - y * right + z * up + source;

					// Call to raycasting method
					// All - except simple ped collision http://www.dev-c.com/nativedb/func/info/65287525d951f6be
					RaycastResult result = World::Raycast(source, dest, (IntersectOptions)(4294967295 - 4));

					unsigned int idx = scanner->horizontalResolution * v + h;
					if (result.DitHitAnything)
					{
						float dist = result.HitCoords.DistanceTo(source);

						depth[idx] = dist;

						if (result.DitHitEntity)
						{
							label[idx] = LidarScanner::GetLabelFromRaycast(result);
						}
						else {
							label[idx] = LidarScanner::LBL_UNK;
						}
					}
					else {
						depth[idx] = scanner->range;
						label[idx] = LidarScanner::LBL_UNK;
					}
				}
			}
		}

		Math::Vector3 source, forward, right, up;
		LidarScanner ^scanner;
		array<float> ^depth;
		array<unsigned short> ^label;
	};

	LidarScanner::LidarScanner(float horizontalMin, float horizontalMax, int horizontalResolution,
		float verticalMin, float verticalMax, int verticalResolution, float range)
		: horizontalMin(horizontalMin), horizontalMax(horizontalMax), horizontalResolution(horizontalResolution),
		verticalMin(verticalMin), verticalMax(verticalMax), verticalResolution(verticalResolution),
		range(range)
	{
		// calculate step size
		this->horizontalStep = (horizontalMax - horizontalMin) / horizontalResolution;
		this->verticalStep = (verticalMax - verticalMin) / verticalResolution;
	}

	/**
	 * Runs a full lidar scan in the native env, data returned through referenced depth, label
	 */
	void LidarScanner::Scan(Math::Vector3 source, Math::Vector3 forward,
		Math::Vector3 right, Math::Vector3 up, array<float> ^depth, array<unsigned short> ^label) {
		// Put task on main thread, task writes into the result arrays
		const auto task = gcnew NativeLidarScan();
		task->scanner = this;
		task->depth = depth;
		task->label = label;
		task->source = source;
		task->forward = forward;
		task->right = right;
		task->up = up;

		ScriptDomain::CurrentDomain->ExecuteTask(task);

	}

	/**
	Generates a label for a given raycast result
	*/
	unsigned short LidarScanner::GetLabelFromRaycast(RaycastResult &result)
	{
		if (result.HitEntity->GetType() == Vehicle::typeid)
		{
			if (((Vehicle^)result.HitEntity)->Model.IsBicycle)
			{
				return LidarScanner::LBL_BICYCLE;
			}
			else if (((Vehicle^)result.HitEntity)->Model.IsBike)
			{
				return LidarScanner::LBL_BIKE;
			}
			else
			{
				return LidarScanner::LBL_VEH;
			}
		}
		else if (result.HitEntity->GetType() == Ped::typeid)
		{
			if (((Ped^)result.HitEntity)->IsInVehicle() || ((Ped^)result.HitEntity)->IsOnBike)
			{
				Model m = ((Ped^)result.HitEntity)->CurrentVehicle->Model;
				if (m.IsBicycle)
				{
					return LidarScanner::LBL_BICYCLE;
				}
				else if (m.IsBike)
				{
					return LidarScanner::LBL_PED_ON_BIKE;
				}
				else
				{
					return LidarScanner::LBL_PED_IN_VEH;
				}
			}
			else
			{
				return LidarScanner::LBL_PED;
			}
		}
		else if (result.HitEntity->GetType() == Prop::typeid)
		{
			return LidarScanner::LBL_PROP;
		}

		return LidarScanner::LBL_UNK;
	}
	
}