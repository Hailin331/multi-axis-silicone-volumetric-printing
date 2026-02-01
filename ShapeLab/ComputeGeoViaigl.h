#pragma warning(disable : 4996)
#ifndef COMPUTEGEOVIAIGL_H
#define COMPUTEGEOVIAIGL_H
#include <igl/readOBJ.h>
#include <igl/exact_geodesic.h>
#include <igl/parula.h>
#include <igl/isolines_map.h>
#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <queue>
#include "QMeshPatch.h"
#include "PolygenMesh.h"
#include "QMeshNode.h"
#include "QMeshEdge.h"
#include "QMeshFace.h"
#include "../GLKLib/GLKObList.h"

class ComputeGeoViaigl
{

public:

	ComputeGeoViaigl( QMeshPatch* inputmesh) {
 		surfacemesh = inputmesh;
	}
	~ComputeGeoViaigl() {

	}
	void Compute_Geo();

private:
	std::string mesh_path;
	std::vector<std::vector<int>> findBoundaryLoops(const Eigen::MatrixXi& F);
	QMeshPatch* surfacemesh;

};

#endif // COMPUTEGEOVIAIGL_H
