// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "LIMUPrimitiveFitter.h"
#include "DeviceAgnostic\ITMRepresentationAccess.h"
#include "ITMVisualisationEngine_CPU.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

template<class TVoxel, class TIndex>
LIMUPrimitiveFitter<TVoxel, TIndex>::LIMUPrimitiveFitter(const ITMLibSettings *settings)
{

}

template<class TVoxel, class TIndex>
LIMUPrimitiveFitter<TVoxel,TIndex>::~LIMUPrimitiveFitter()
{
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

	//Cast the cylinder, need the visualization engin


}

//This enables to put a template class in different file
template class ITMLib::Engine::LIMUPrimitiveFitter<ITMVoxel, ITMVoxelIndex>;