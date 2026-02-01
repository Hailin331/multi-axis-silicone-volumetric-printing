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
#include "ComputeGeoViaigl.h"
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
#include <sstream>

#pragma once
class DeformToolPath
{
public:
	DeformToolPath(QMeshPatch* inputMesh, QMeshPatch* toolpath, QMeshNode* lnode, double freq, bool rot,double inputcutvalue,bool iteratedComp,
		double upperfreinput,double lowwerfreinput,double uppercutinput,double lowwercutinput) {
		surfaceMesh = inputMesh;
		Toolpath = toolpath;
		rotationflag = rot;
		scalevaluelist.resize(surfaceMesh->GetFaceNumber());
		scalevaluelist.reserve(surfaceMesh->GetFaceNumber());
		Facerotationmatrix.resize(surfaceMesh->GetFaceNumber());
		Facerotationmatrix.reserve(surfaceMesh->GetFaceNumber());

		InitialPos3D.resize(surfaceMesh->GetNodeNumber(), 3);
		InitialNor3D.resize(surfaceMesh->GetFaceNumber(), 3);
		DeformedPos3D.resize(surfaceMesh->GetNodeNumber(), 3);
		DeformedNor3D.resize(surfaceMesh->GetFaceNumber(), 3);

		FaceInitialpos3D.resize(surfaceMesh->GetFaceNumber());
		FaceCurrentpos3D.resize(surfaceMesh->GetFaceNumber());
		Deformedfacearea.resize(surfaceMesh->GetFaceNumber());
		Initialfacearea.resize(surfaceMesh->GetFaceNumber());

 		lastnode = lnode;
 		int i = 0;
 		for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
		{
			QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
			if (node->layerheight < 0.2) {
				node->layerheight = 0.2;
			}
			if (node->layerheight > 1.2) {
				node->layerheight = 1.2;
			}
			
		}

		for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
		{
			QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
			face->SetIndexNo(i);
			face->CalPlaneEquation();
			i++;
			double avewidth = 0;
			for (int j = 0; j < 3; j++)
			{
				avewidth+=getwidth(face->GetNodeRecordPtr(j)->layerheight);
			}
			avewidth = avewidth / 3.0;
			scalevaluelist.at(face->GetIndexNo()) = avewidth;
		}
		i = 0;
		for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
		{
			QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
 			node->SetIndexNo(i);
			node->DisToBoundary = 2000;
			node->DisToInnerBoundary = 2000;
			node->geoFieldValue = 0;
			i++;
		}
		i = 0;
		for (GLKPOSITION edgepos = surfaceMesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
		{
			QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
			edge->SetIndexNo(i);
			i++;
		}
 		double maxyy = -INFINITY;
		QMeshNode* boundnode;
		for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
		{
			QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
			Eigen::Vector3d pos;
			node->GetCoord3D(pos);
			node->CalNormal();
			if (pos(1) > maxyy) {
				maxyy = pos(1);
				boundnode = node;
 			}
 		}
	

		QMeshNode* intialnode = boundnode;
		QMeshEdge* Prevedge = NULL;
		for (GLKPOSITION edgepos = boundnode->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
		{
			QMeshEdge* edge = (QMeshEdge*)boundnode->GetEdgeList().GetNext(edgepos);
			if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) {
				Prevedge = edge; break;
			}
		}
		QMeshEdge* nextedge = Prevedge;
		Prevedge->isoutterboundary = true;

		while (true)
		{
			bool findnode = false;
			if (boundnode == nextedge->GetStartPoint())boundnode = nextedge->GetEndPoint();
			else boundnode = nextedge->GetStartPoint();
			if (boundnode == intialnode)break;
			for (GLKPOSITION edgepos = boundnode->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
			{
				QMeshEdge* edge = (QMeshEdge*)boundnode->GetEdgeList().GetNext(edgepos);
				if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) {
					if (edge != nextedge)
					{
						nextedge = edge;
						nextedge->isoutterboundary = true;
						break;
					}
				}
			}
		}
		for (GLKPOSITION edgepos = surfaceMesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
		{
			QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
			if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) {
				if (edge->isoutterboundary == false)
				{
					edge->isinnerboundary = true;
					hasinnerbound = true;

				}
			}
		}

		if (hasinnerbound) {
			/*inner boundary num is set to 1*/
			distoinnerbound.resize(surfaceMesh->GetNodeNumber());
			for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
			{
				QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
				distoinnerbound[node->GetIndexNo()].resize(1);
			}
		}
		itecompflag = iteratedComp;
		/*printing parameters*/
		fre = freq;
 		initialadjust = 0.25;
 		cutvalue = inputcutvalue;
		outterboundarynum = 1;
		innerboundarynum = 1;
		boundratio = 1.0;
		insertlength = 0.1;
		StripFirst = true;
		removelinedistance = 1.5;
 		stripcutvalue =0;
		largegap = 2.0;
  		upperfre = upperfreinput;
		lowerfre = lowwerfreinput;
		uppercutvalue = uppercutinput;
		lowercutvalue = lowwercutinput;
		if (!rotationflag)fre = 0;
	}

	DeformToolPath(QMeshPatch* inputMesh, QMeshPatch* toolpath,double freq,bool rot) {

 	}

	~DeformToolPath() {
		if (true)
		{
		}//std::cout << "delet Deformationpath" << std::endl;
	}
	void assignvectortoface();
	void Generatetri();
	void GenerateInitialGuess();
	void Deformation3D();
	void ComputeRotation();
	void FullfillMartrixA(Eigen::SparseMatrix<double> &Amat);
	void Ite_ComputePathon3D();
	void ComputeScaling();
	void faceAreaCompute(std::vector<double> & facearea);
	void OutputDifferenceArea();
	Eigen::Matrix<double, 3, 3> removemean( Eigen::Matrix<double, 3, 3>& nodeposmat);
	void FullfillMatrixB(Eigen::VectorXd& vecB);
	void computeoutterboundary(int buondnum, std::vector<std::vector<QMeshNode*>>& nodelist);
 	void computesignalboundtoolpath( std::vector<QMeshNode*>& newline,double isovalue, std::vector<std::vector<QMeshNode*>>& reorderednode);
  	void smoothtoolpath(QMeshPatch* patch,int windowsize);
	void updatedistooutter(std::vector<Eigen::Vector3d>poslist, std::vector<double>heightlist);
	void connectboundaryedge(bool A_statflag,bool outterboundary,int boundnum,QMeshNode* lnode, std::vector<QMeshNode*>& boundnodevec, std::vector<std::vector<QMeshNode*>>&vec);
	void ConverttoInitial();
	void ConverttoDeforma();

 	void checknoextrusionnode(QMeshPatch* toolpath);
 	double distance(QMeshNode* node1, QMeshNode* node2);
  	int compute_allinner_boundary(int buondnum, std::vector<std::vector<QMeshNode*>>&nodelist);
 	void compute_single_innertoolpath(std::vector<QMeshNode*>& newline, double isovalue, std::vector<std::vector<QMeshNode*>>& nodelist);
	void resamplingSinglePatch(QMeshPatch* patch);
	void getallpathlength(double &pathlength);
	std::vector<QMeshNode*> a_star( QMeshNode*startnode, QMeshNode*endnode);
	void reorderthestripeline();
	QMeshNode* buildstriptoolpath(QMeshNode* lnode);
	double compute_volume();
	void getvolumeoflayer();
	double getarea(double height);
	double getwidth(double height);
	double getvelocity(double height);
  	void savedeformedmesh();
	void savescalefield();
	void Generatefinaltoolpath();
	void Trimboundtoolpath(std::vector<std::vector<QMeshNode*>>&Boundnodelist,std::vector<bool>&IsboundClose,bool isoutboundary);
 
private:
 	bool deformstate = false;
	bool itecompflag=true;
 	double stripcutvalue = 0;
	double cutvalue = -0.3;
	double insertlength = 0.01;//1.0
 	double largegap = 3;
	double resampleLength = 1.5;
	double alpathlength=0;
	int outterboundarynum = 2;
	int innerboundarynum = 2;
 	double initialadjust = 0.5;
	double minwidthpath = 1.0;
	double averagewidth = 1.0;
	double removelinedistance = 2.5;
	bool hasinnerbound = false;
	bool StripFirst = false;
	double ErrThreshold = 0.02;
  	QMeshNode* lastnode=NULL;
	QMeshPatch* surfaceMesh = NULL;
	QMeshPatch* Toolpath = NULL;
	Eigen::MatrixXd InitialPos3D;
	Eigen::MatrixXd InitialNor3D;
	Eigen::MatrixXd DeformedPos3D;
	Eigen::MatrixXd DeformedNor3D;

	std::vector<double> scalevaluelist;
	std::vector<double> Initialfacearea;
	std::vector<double> Deformedfacearea;
	std::vector<Eigen::Matrix3d> Facerotationmatrix;
	std::vector<Eigen::Matrix<double, 3, 3>> FaceInitialpos3D;
	std::vector<Eigen::Matrix<double, 3, 3>> FaceCurrentpos3D;
 	std::vector<std::vector<double>>distoinnerbound;
	std::string AreaDifferencepath = "../Model/IsoSurface/TTA/Darea.txt";
	std::string Scalepath = "../Model/IsoSurface/TTA/Scale.txt";
	std::vector < Eigen::Vector2d > nodegra;
	std::vector<QMeshNode*>nodevec;
 	std::vector<std::pair<bool,std::vector<int>>>rerderednodelist;
	std::vector<QMeshNode*>stripenode_order;
	Eigen::MatrixXd dismatrix;
	std::vector<bool>outterboundaryclose;
	std::vector<bool>innerboundaryclose;
	std::vector<double> stripe_points_pos;
	std::vector<int> stripe_points_edge_index;
	std::vector<int> stripe_points_edge_orientation;
	std::vector<std::vector<int>>edgepolylines_index;
	std::vector<std::vector<std::array<int, 2>>>matching_information;

	std::vector<std::vector<QMeshNode*>> innernodelist;
	std::vector<std::vector<QMeshNode*>> tempinnernodelist;
	std::vector<std::vector<QMeshNode*>> nodelist;
	std::vector<std::vector<QMeshNode*>> tempnodelist;


	int allboundnum = 0;

	double boundratio = 1.0;
 	double fre = 0;
	bool rotationflag = true;
	bool iterationComp = true;
	double volumeoflayer = 0;
	double upperfre;
	double lowerfre;
	double uppercutvalue;
	double lowercutvalue;
	int itenum = 0;

 };

