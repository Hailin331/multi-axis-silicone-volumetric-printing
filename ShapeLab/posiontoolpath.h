#pragma once
#include "fake_eigen_all.h"   // 一定要放在最前面
#include "QMeshPatch.h"
#include "PolygenMesh.h"
#include "QMeshNode.h"
#include "QMeshEdge.h"
#include "QMeshFace.h"
#include "../GLKLib/GLKObList.h"
#include "Eigen/unsupported/Eigen/KroneckerProduct"
#include <Eigen/PardisoSupport>
#include <cmath>
#include <queue>
#include <algorithm>
#include <random>
#include <array>
#include <set>
#include <chrono>
#include<fileIO.h>
#include <fstream>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QMessageBox>
#include <QDir>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <igl/cotmatrix.h>
#include <igl/grad.h>
#include <igl/massmatrix.h>
#include <igl/min_quad_with_fixed.h>    // 二次能量带固定点最小化

class posiontoolpath
{


public:

	posiontoolpath(QMeshPatch* inputmesh, QMeshPatch*inputtoolpath, std::string face_input_path,std::string inputdoundary) {

		surfacemesh = inputmesh;
		vector_path = face_input_path;
		integratedDivs.resize(inputmesh->GetNodeNumber());
		start_boundary_node = inputdoundary;
		nodefieldvalue.resize(inputmesh->GetNodeNumber());
		edgevisit.resize(inputmesh->GetEdgeNumber()); 
		toolpath = inputtoolpath;
		int i = 0;
		for (GLKPOSITION facepos = surfacemesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
		{
			QMeshFace* face = (QMeshFace*)surfacemesh->GetFaceList().GetNext(facepos);
			face->SetIndexNo(i);
 			i++;
 
		}
		i = 0;
		for (GLKPOSITION nodepos = surfacemesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
		{
			QMeshNode* node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(nodepos);
			node->SetIndexNo(i);
 			i++;
		}
		i = 0;
		for (GLKPOSITION edgepos = surfacemesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
		{
			QMeshEdge* edge = (QMeshEdge*)surfacemesh->GetEdgeList().GetNext(edgepos);
			edge->SetIndexNo(i);
			i++;
		}


	 
  		for (GLKPOSITION edgepos = surfacemesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
		{
			QMeshEdge* edge = (QMeshEdge*)surfacemesh->GetEdgeList().GetNext(edgepos);
			if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) {
				edge->isoutterboundary = true;
			}

		}
		


	}
	~posiontoolpath() {

	}
	void fieldcomputing();
	
private:
	void assignvectortoface();
	void determineboundarynode();
	void computeIntegratedDivergence();
	void buildLaplacian	(Eigen::SparseMatrix<double>& L);
	double computeEdgeAngle	(QMeshNode* Node, QMeshEdge* connectEdge);
	int computesignletoolpath(QMeshPatch* surfaceMesh, double isovalue);

	QMeshPatch* surfacemesh;
	QMeshPatch* toolpath;

	std::vector<std::vector<QMeshNode*>> reorderednode;
	std::vector<QMeshNode*>noresamplenode;
	std::string vector_path;
	std::string start_boundary_node;
	Eigen::VectorXd integratedDivs;
	Eigen::SparseMatrix<double> L;
	Eigen::VectorXd B;
	Eigen::VectorXd nodefieldvalue;
	std::vector<int> meshboundarynode;
	std::vector<bool>edgevisit;
	double posweighting = 1;
	double startnodeweighting = 0.01;
	int toolpathnum = 135;

};

