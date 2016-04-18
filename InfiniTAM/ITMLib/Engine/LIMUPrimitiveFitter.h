// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Utils/ITMLibSettings.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMTrackingState.h"
#include "../Objects/ITMRenderState.h"

#include "../Engine/ITMSceneReconstructionEngine.h"
#include "../Engine/ITMVisualisationEngine.h"
#include "../Engine/ITMSwappingEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		*/
		template<class TVoxel, class TIndex>
		class LIMUPrimitiveFitter
		{
		public:
			/// Process a single seed fit
			void ProcessOneSeed(int x, int y, ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState);

			/** \brief Constructor
			*/
			explicit LIMUPrimitiveFitter(const ITMLibSettings *settings);
			~LIMUPrimitiveFitter();
		};
	}
}

