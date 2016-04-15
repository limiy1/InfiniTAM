// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMeshingEngine_CPU.h"
#include "../../DeviceAgnostic/ITMMeshingEngine.h"

using namespace ITMLib::Engine;

template<class TVoxel>
ITMMeshingEngine_CPU<TVoxel,ITMVoxelBlockHash>::ITMMeshingEngine_CPU(void) 
{
}

template<class TVoxel>
ITMMeshingEngine_CPU<TVoxel,ITMVoxelBlockHash>::~ITMMeshingEngine_CPU(void) 
{
}

template<class TVoxel>
void ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash>::VertexScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	Vector3f *vertices = mesh->vertices->GetData(MEMORYDEVICE_CPU);
	const TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry *hashTable = scene->index.GetEntries();

	int noVertice = 0, noMaxVertices = mesh->noMaxVertices, noTotalEntries = scene->index.noTotalEntries;
	float factor = scene->sceneParams->voxelSize;

	mesh->vertices->Clear();

	for (int entryId = 0; entryId < noTotalEntries; entryId++)
	{
		Vector3i globalPos;
		const ITMHashEntry &currentHashEntry = hashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector3f points[8]; float sdfVals[8];
			int edgePattern = getEdgePattern(points, sdfVals, globalPos, Vector3i(x, y, z), localVBA, hashTable);

			if (edgePattern <= 0) continue;

			if (edgePattern & 1)
			{
				vertices[noVertice] = sdfInterp(points[0], points[1], sdfVals[0], sdfVals[1]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 2)
			{
				vertices[noVertice] = sdfInterp(points[1], points[2], sdfVals[1], sdfVals[2]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 4)
			{
				vertices[noVertice] = sdfInterp(points[2], points[3], sdfVals[2], sdfVals[3]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 8)
			{
				vertices[noVertice] = sdfInterp(points[3], points[0], sdfVals[3], sdfVals[0]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 16)
			{
				vertices[noVertice] = sdfInterp(points[4], points[5], sdfVals[4], sdfVals[5]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 32)
			{
				vertices[noVertice] = sdfInterp(points[5], points[6], sdfVals[5], sdfVals[6]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 64)
			{
				vertices[noVertice] = sdfInterp(points[6], points[7], sdfVals[6], sdfVals[7]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 128)
			{
				vertices[noVertice] = sdfInterp(points[7], points[4], sdfVals[7], sdfVals[4]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 256)
			{
				vertices[noVertice] = sdfInterp(points[0], points[4], sdfVals[0], sdfVals[4]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 512)
			{
				vertices[noVertice] = sdfInterp(points[1], points[5], sdfVals[1], sdfVals[5]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 1024)
			{
				vertices[noVertice] = sdfInterp(points[2], points[6], sdfVals[2], sdfVals[6]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
			if (edgePattern & 2048)
			{
				vertices[noVertice] = sdfInterp(points[3], points[7], sdfVals[3], sdfVals[7]) * factor;
				if (noVertice < noMaxVertices - 1) ++noVertice;
			}
		}
	}

	mesh->noTotalVertices = noVertice;
}

template<class TVoxel>
void ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash>::MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	ITMMesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CPU);
	const TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry *hashTable = scene->index.GetEntries();

	int noTriangles = 0, noMaxTriangles = mesh->noMaxTriangles, noTotalEntries = scene->index.noTotalEntries;
	float factor = scene->sceneParams->voxelSize;

	mesh->triangles->Clear();

	for (int entryId = 0; entryId < noTotalEntries; entryId++)
	{
		Vector3i globalPos;
		const ITMHashEntry &currentHashEntry = hashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector3f vertList[12];
			int cubeIndex = buildVertList(vertList, globalPos, Vector3i(x, y, z), localVBA, hashTable);
			
			if (cubeIndex < 0) continue;

			for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3)
			{
				triangles[noTriangles].p0 = vertList[triangleTable[cubeIndex][i]] * factor;
				triangles[noTriangles].p1 = vertList[triangleTable[cubeIndex][i + 1]] * factor;
				triangles[noTriangles].p2 = vertList[triangleTable[cubeIndex][i + 2]] * factor;

				if (noTriangles < noMaxTriangles - 1) noTriangles++;
			}
		}
	}

	mesh->noTotalTriangles = noTriangles;
}

template<class TVoxel>
ITMMeshingEngine_CPU<TVoxel,ITMPlainVoxelArray>::ITMMeshingEngine_CPU(void) 
{}

template<class TVoxel>
ITMMeshingEngine_CPU<TVoxel,ITMPlainVoxelArray>::~ITMMeshingEngine_CPU(void) 
{}

template<class TVoxel>
void ITMMeshingEngine_CPU<TVoxel, ITMPlainVoxelArray>::MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMPlainVoxelArray> *scene)
{}

template class ITMLib::Engine::ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
