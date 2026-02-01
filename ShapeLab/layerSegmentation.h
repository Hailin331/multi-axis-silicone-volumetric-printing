#pragma once
#include "PolygenMesh.h"
#include "QMeshPatch.h"

class layerSegmentation
{
public:
	layerSegmentation(QMeshPatch* inputMesh) { 
		surfaceMesh = inputMesh;
	};
	~layerSegmentation() {};

	void regionDetection(int holeNum);
	void generateSplitedMeshPatch_CCFtoolpathGeneration_cutMesh(
		QMeshPatch* patch, QMeshPatch* skeletonPatch, QMeshPatch* newPatch);

	void computeDistanceField_AllSeedPoints_holeRegionDetection_singleBoundaryCVT(
		QMeshPatch* patch, double boundShrinkRatio, int boundaryIndex, bool includeBoundary);

	void boundarySegementation_reorderSeedIndex(QMeshPatch* newPatch);

	void repairNormalDirection(QMeshPatch* patch);

	bool _fixNewCutFaceOrientation(QMeshNode* n1, QMeshNode* n2, QMeshNode* n3, QMeshFace* origFace);

	QMeshPatch* surfaceMesh;

	int meshgenus = 0;

};
