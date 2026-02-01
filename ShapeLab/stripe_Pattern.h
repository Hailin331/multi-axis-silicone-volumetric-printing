#pragma once
#include "QMeshPatch.h"
#include "PolygenMesh.h"
#include "QMeshNode.h"
#include "QMeshEdge.h"
#include "QMeshFace.h"
#include "../GLKLib/GLKObList.h"
#include "Eigen/unsupported/Eigen/KroneckerProduct"
#include <Eigen/PardisoSupport>
#include<complex>
#include"math_utils.h"
#include <array> // For std::array
#include <random> // 包含随机数生成库
#include <iostream>
#include <fstream>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
class stripe_Pattern
{
public:
	stripe_Pattern(QMeshPatch* inputMesh,double frequency) {
		surfaceMesh = inputMesh;
		int i = 0;
		for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
		{
			QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
			face->SetIndexNo(i);
			face->errorface = false;
			face->CalPlaneEquation();
			//face->SetNormal(0, 0, -1);
 			i++;
		}
		i = 0;
		for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
		{
			QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
			node->SetIndexNo(i);
			i++;
			//nodesumangle.push_back(computeallangelofnode(node));
		}
		i = 0;
		for (GLKPOSITION edgepos = surfaceMesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
		{
			QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
			edge->SetIndexNo(i);
			i++;
		}
		guide_complex.resize(surfaceMesh->GetNodeNumber());
		edge_complex.resize(surfaceMesh->GetEdgeNumber());
		reference_edge.resize(surfaceMesh->GetNodeNumber());
		transport_complex.resize(surfaceMesh->GetEdgeNumber());
		node_paramaterization.resize(surfaceMesh->GetNodeNumber());
		edge_omega.resize(surfaceMesh->GetEdgeNumber());
		edge_cross_sheet.resize(surfaceMesh->GetEdgeNumber());
		cotanweight.resize(surfaceMesh->GetEdgeNumber());
		pattern_frequency = frequency;
		anticolckjudge.resize(surfaceMesh->GetFaceNumber());
		edgeindexofface.resize(surfaceMesh->GetFaceNumber());
		coor_corner.resize(surfaceMesh->GetFaceNumber());
		facepara.resize(surfaceMesh->GetFaceNumber());
		edgepolylines_index.resize(surfaceMesh->GetEdgeNumber());
		stripe_points_edge_orientation.resize(10*surfaceMesh->GetEdgeNumber(), 0);
		edgeindexofface.resize(surfaceMesh->GetFaceNumber());
		matching_information.resize(surfaceMesh->GetFaceNumber());
		nodesumangle.resize(surfaceMesh->GetNodeNumber());
		refedge_ptr.resize(surfaceMesh->GetNodeNumber());
 	}
	~stripe_Pattern() {
	}

public:
	void require_paramaterization();
	void require_Coordinates();
	void require_Polylines(std::vector<double>& output_stripe_points_pos, std::vector<int> & output_stripe_points_edge_index,
		std::vector<int>& output_stripe_points_edge_orientation, std::vector<std::vector<int>>& output_edgepolylines_index, std::vector<std::vector<std::array<int, 2>>>&output_matching_information);
 
private:
	void find_referenceedge();
	void require_transport_vector();
	void require_edge_omega(std::vector<double>& frequencies);
	void require_guide_complex(); 
	void require_vertex_energy_matrix();
	void require_mass_matrix();
	void require_cotanweighting();
	double computeEdgeAngle
	(QMeshNode* Node, QMeshEdge* connectEdge) const;
	double computearea(QMeshNode* node);
	void computeerror();
	void judgeanticlockwise();
	Eigen::Vector3d rotateVector3D(const Eigen::Vector3d& vec, const Eigen::Vector3d& axis, double angle);
	int computeRank(const Eigen::SparseMatrix<double>& mat, double tol);
	std::vector<double> crossings_mod_2pi(double val1, double val2);
	void connect_isolines_on_singularities();
	// Function to normalize a complex number (make it a unit complex number)
	std::complex<double> tounit(const std::complex<double>& z);
	std::vector<std::array<int, 2>>match_crossings(const std::vector<std::vector<int>>& crossings);
	std::vector<std::vector<int>> remove_crossings(std::vector<std::pair<double, std::array<int, 2>>>& values, int n,
		std::vector<std::vector<int>>& face_indices);
	double find_min(const std::vector<std::vector<Eigen::Vector3d>>& points, const Eigen::Vector3d& v, const Eigen::Vector3d& vector,
		size_t ignored_edge);
	double signed_angle_between_vectors(const Eigen::Vector3d& startvec, const Eigen::Vector3d& endvec, const Eigen::Vector3d& normal_dir);
	double sumpofangle(QMeshNode* node, QMeshEdge* refedge, QMeshEdge* endedge);
	double computeallangelofnode(QMeshNode* node);
private:
	QMeshPatch* surfaceMesh = NULL;
	std::vector<std::complex<double>> guide_complex;
	std::vector <std::complex<double>> edge_complex;
	std::vector<Eigen::Vector3d>reference_edge;
	std::vector <std::complex<double>> transport_complex;
	std::vector<std::complex<double>> node_paramaterization;
	std::vector <double>edge_omega;
	std::vector<bool> edge_cross_sheet;
	std::vector<double> cotanweight;
	std::vector<double>nodesumangle;
	std::vector<std::array<double, 3>>anticolckjudge;
	std::vector<std::array<int, 3>>edgeindexofface;
	std::vector<std::array<double, 3>> coor_corner;
	std::vector<int>facepara;
	std::vector<QMeshEdge*>refedge_ptr;
	Eigen::SparseMatrix<double> matA;
	Eigen::SparseMatrix<double>matB;
	Eigen::VectorXd solutioncomplex;

	/*need to be returned*/
	std::vector<double> stripe_points_pos;
	std::vector<int> stripe_points_edge_index;
	std::vector<int> stripe_points_edge_orientation;
	std::vector<std::vector<int>>edgepolylines_index;
	std::vector<std::vector<std::array<int, 2>>>matching_information;



	double pattern_frequency;
};

