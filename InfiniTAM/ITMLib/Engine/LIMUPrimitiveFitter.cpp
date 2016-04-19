// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "LIMUPrimitiveFitter.h"
#include "DeviceAgnostic/ITMVisualisationEngine.h"  //for castray

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

/**
 * Cylinder casting :
 * Casting rays that forming a cylinder
 * @centerPt: center point of the cylinder (in number of voxel)
 * @radius: radius of the cynlinder (in meter)
 * @depth: height of the cynlinder (in meter)
 * @incrementalTheta: in degree
 *
 * Note: in this function, voxel contains maybe changed, so scene is not constant
 */
template<class TVoxel, class TIndex>
static void GenericCylinderCast(ITMScene<TVoxel,TIndex> *scene,  Vector3f centerPt, Vector3f rayDirection, float radius, float incrementalTheta, float depth, const ITMRenderState *renderState)
{
	float mu = scene->sceneParams->mu;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;
	Vector4f pointsRay;
	TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	//Change from degree to rad
	incrementalTheta = incrementalTheta/180*PI;
	//Get unit vector of rayDirection
	float invl = 1/sqrt(rayDirection.x*rayDirection.x + rayDirection.y*rayDirection.y + rayDirection.z*rayDirection.z);
	rayDirection *= invl;


#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif

	if ( ( fabs(rayDirection.x) < 1e-2 ) && ( fabs(rayDirection.x) < 1e-2 ) )   //rayDirection is the z-direction
	{
		std::cout<<"ray is nearly parallel to z"<<std::endl;
		//Some temporary values
		float uv = rayDirection.x * rayDirection.y;
		float wv = rayDirection.z * rayDirection.y;
		float u2w2 = rayDirection.x*rayDirection.x + rayDirection.z*rayDirection.z;

		for (float theta = 0 ; theta < 2*PI ; theta += incrementalTheta)
		{
			Vector3f raidusVector(rayDirection.y*cos(theta) - uv*sin(theta),
								+u2w2*sin(theta),
								-rayDirection.x*cos(theta) - wv*sin(theta));
			//Here, convert radius in number of voxel
			float radius_coef = radius * oneOverVoxelSize / sqrt(raidusVector.x * raidusVector.x + raidusVector.y * raidusVector.y + raidusVector.z * raidusVector.z);
			Vector3f rayCenter = radius_coef*raidusVector + centerPt;

			std::cout<<"ray center = "<<rayCenter<<std::endl;
			castRay<TVoxel, TIndex>(
				pointsRay,
				rayCenter,
				rayDirection,
				voxelData,
				voxelIndex,
				oneOverVoxelSize,
				mu,
				depth
			);
		}
	}
	else
	{
		std::cout<<"ray is not parallel to z"<<std::endl;
		//Some temporary values
		float wu = rayDirection.z * rayDirection.x;
		float wv = rayDirection.z * rayDirection.y;
		float u2v2 = rayDirection.x*rayDirection.x + rayDirection.y*rayDirection.y;

		for (float theta = 0 ; theta < 2*PI ; theta += incrementalTheta)
		{
			Vector3f raidusVector(rayDirection.y*cos(theta) + wu*sin(theta),
								-rayDirection.x*cos(theta) + wv*sin(theta),
								-u2v2*sin(theta));
			float radius_coef = radius / sqrt(raidusVector.x * raidusVector.x + raidusVector.y * raidusVector.y + raidusVector.z * raidusVector.z);
			Vector3f rayCenter = radius_coef*raidusVector + centerPt;

			std::cout<<"radius center = "<<raidusVector<<std::endl;
			castRay<TVoxel, TIndex>(
				pointsRay,
				rayCenter,
				rayDirection,
				voxelData,
				voxelIndex,
				oneOverVoxelSize,
				mu,
				depth
			);
		}
	}

}

template<class TVoxel, class TIndex>
LIMUPrimitiveFitter<TVoxel, TIndex>::LIMUPrimitiveFitter(const ITMLibSettings *settings, ITMScene<TVoxel,TIndex> *scene)
{
	this->scene = scene;
}

template<class TVoxel, class TIndex>
LIMUPrimitiveFitter<TVoxel,TIndex>::~LIMUPrimitiveFitter()
{
}

template<class TVoxel, class TIndex>
void LIMUPrimitiveFitter<TVoxel, TIndex>::FindRingPatch(Vector3f centerPt,
		Vector3f rayDirection, float radius, float incrementalTheta, float depth, const ITMRenderState *renderState)
{
	GenericCylinderCast(this->scene, centerPt, rayDirection, radius, incrementalTheta, depth, renderState);
}

template<class TVoxel, class TIndex>
void LIMUPrimitiveFitter<TVoxel, TIndex>::ProcessOneSeed(int x, int y, ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState)
{
	//Get the normal and position of the target point
	Vector2i imgSize = renderState->raycastResult->noDims;
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	Vector4f seedPos3D = pointsRay[imgSize.x*imgSize.y / 2];  //get the center point

	//Get the normal from the voxel
	Vector3f point(seedPos3D.x / seedPos3D.w, seedPos3D.y / seedPos3D.w, seedPos3D.z / seedPos3D.w);
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();
	Vector3f normal = computeSingleNormalFromSDF(voxelData, voxelIndex, point);

	std::cout << "The point is at " << seedPos3D << std::endl;
	std::cout << "The normal is " << normal << std::endl;

	//Cast the cylinder, need the visualization engine
	FindRingPatch(point, normal, 0.02, 10, 0.1, renderState);

}

//This enables to put a template class in different file
template class ITMLib::Engine::LIMUPrimitiveFitter<ITMVoxel, ITMVoxelIndex>;
