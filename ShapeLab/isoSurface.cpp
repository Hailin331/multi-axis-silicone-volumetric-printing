#include "stdafx.h"
#include "isoSurface.h"
#include <omp.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "GLKGeometry.h"

using namespace std;
using namespace Eigen;

isoSurface::isoSurface(QMeshPatch* Patch) { tetMesh = Patch; }

isoSurface::~isoSurface() { }


void isoSurface::ComputeLayerHeightUsingGradient(PolygenMesh* isosurfaceset)
{
	int layerindex = 0;
	for (GLKPOSITION posMesh = isosurfaceset->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* layer = (QMeshPatch*)isosurfaceset->GetMeshList().GetNext(posMesh);
		layerindex = layer->GetIndexNo();
		if (layerindex == 0) { std::cout << "start" << std::endl; continue; }
		for (GLKPOSITION nodepos = layer->GetNodeList().GetHeadPosition(); nodepos != nullptr;) {
			QMeshNode* node = (QMeshNode*)layer->GetNodeList().GetNext(nodepos);
			node->layerheight =0.01 / node->WeightVector.norm();
			cout << "nodeheight" << node->layerheight<<endl;
		}
	}
}

void isoSurface::ComputeVectorofNode()
{
	double volsum = 0;
	Eigen::Vector3d WeightedVec;
	for (GLKPOSITION Nodepos = tetMesh->GetNodeList().GetHeadPosition(); Nodepos != nullptr;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Nodepos);
		for (GLKPOSITION trapos = Node->GetTetraList().GetHeadPosition(); trapos != nullptr;) {
			QMeshTetra* tet = (QMeshTetra*)Node->GetTetraList().GetNext(trapos);
			WeightedVec(0) += tet->InitialVector(0) * tet->CalVolume();
			WeightedVec(1) += tet->InitialVector(1) * tet->CalVolume();
			WeightedVec(2) += tet->InitialVector(2) * tet->CalVolume();
			volsum += tet->CalVolume();
		}
		Node->WeightVector(0) = WeightedVec(0) / volsum;
		Node->WeightVector(1) = WeightedVec(1) / volsum;
		Node->WeightVector(2) = WeightedVec(2) / volsum;
		WeightedVec.setZero();
		volsum = 0;
	}
}

void isoSurface::generateIsoSurface(PolygenMesh* isoSurface, int layerNum) {

	//ComputeVectorofNode();

	/* Direct substract iso-surface from tetrahedral mesh*/

	for (int i = 0; i < layerNum; i++) {

		double isoCurveValue = (0.9+i) * 1 / (double)layerNum;
		//double isoCurveValue = i / (double)layerNum+0.08;
	
		QMeshPatch* layer = generatesingleIsoSurface(isoCurveValue, isoSurface, false);

		if (layer->GetNodeNumber() == 0) {
			cout << "this layer have no node!" << endl; continue;
		}

		layer->drawPrincipleStressField = true;
		layer->surfacecount = i;
		layer->drawThisIsoLayer = true;
		//layer->drawlayerheight = true;
		layer->drawPrincipleStressField = false;
		layer->isoSurfaceValue = isoCurveValue;
		isoSurface->meshList.AddTail(layer);
		layer->SetIndexNo(isoSurface->GetMeshList().GetCount()-1); //index begin from 0
		//cout << layer->GetIndexNo() << " Layer, isoValue = " << isoCurveValue << ", nodeNum = " << layer->GetNodeNumber() << endl;
	}
	// ComputeLayerHeightUsingGradient(isoSurface);
}

QMeshPatch* isoSurface::generatesingleIsoSurface(double isoValue, PolygenMesh* isoSurface, bool support) {

    QMeshPatch* layer = new QMeshPatch;
    layer->isoSurfaceValue = isoValue;

    const double eps = 1.0e-5;

     for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        const double d = std::fabs(node->scalarField - isoValue);
        if (d < eps) {
            node->scalarField = (node->scalarField > isoValue) ? (isoValue + eps) : (isoValue - eps);
        }
    }

     int nextIsoNodeId = 1;
    int nextIsoEdgeId = 1;
    int nextIsoFaceId = 1;

    // 2) build node list for isoSurface (from edge list)
    for (GLKPOSITION Pos = tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
        QMeshEdge* edge = (QMeshEdge*)tetMesh->GetEdgeList().GetNext(Pos);

        edge->installedIsoNode = nullptr;
        edge->isLocateIsoNode = false;

        QMeshNode* s = edge->GetStartPoint();
        QMeshNode* t = edge->GetEndPoint();

        const double a = s->scalarField;
        const double b = t->scalarField;

         const bool cross =
            (a < isoValue && b > isoValue) ||
            (a > isoValue && b < isoValue);

        if (!cross) continue;

        const double alpha = (isoValue - a) / (b - a);

        double p1[3], p2[3];
        s->GetCoord3D(p1[0], p1[1], p1[2]);
        t->GetCoord3D(p2[0], p2[1], p2[2]);

        double pp[3], wvec[3];
        const double oneMinus = 1.0 - alpha;

         for (int j = 0; j < 3; j++) {
            pp[j] = oneMinus * p1[j] + alpha * p2[j];
            wvec[j] = oneMinus * s->WeightVector[j] + alpha * t->WeightVector[j];
        }

        QMeshNode* isoNode = new QMeshNode;
        isoNode->relatedInitTetEdge = edge;
        isoNode->SetMeshPatchPtr(layer);
        isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
        isoNode->SetIndexNo(nextIsoNodeId++); // start from 1

        for (int j = 0; j < 3; j++) isoNode->WeightVector[j] = wvec[j];

        layer->GetNodeList().AddTail(isoNode);

        edge->installedIsoNode = isoNode;
        edge->isLocateIsoNode = true;
    }

    // 3) build edge list for isoSurface (from face list)
     static const int nextEdgeIdx[3] = { 1, 2, 0 };
    static const int prevEdgeIdx[3] = { 2, 0, 1 };

    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        face->installedIsoEdge = nullptr;
        face->isLocatedIsoEdge = false;

        QMeshNode* n0 = face->GetNodeRecordPtr(0);
        QMeshNode* n1 = face->GetNodeRecordPtr(1);
        QMeshNode* n2 = face->GetNodeRecordPtr(2);

        const int positiveNum =
            (n0->scalarField > isoValue) +
            (n1->scalarField > isoValue) +
            (n2->scalarField > isoValue);

        if (positiveNum == 0 || positiveNum == 3) continue;

        QMeshEdge* isoEdge = new QMeshEdge;

        int index = 0;

        if (positiveNum == 1) {
            // find positive node index
            if (n0->scalarField > isoValue) index = 0;
            else if (n1->scalarField > isoValue) index = 1;
            else index = 2;

            // Edge index+1
            QMeshEdge* eA = face->GetEdgeRecordPtr(index + 1);
            QMeshNode* startNode = eA->installedIsoNode;
            isoEdge->SetStartPoint(startNode);

            // Edge (index+2)%3 +1 -> nextEdgeIdx[ index ] is (index+1)%3
             QMeshEdge* eB = face->GetEdgeRecordPtr(prevEdgeIdx[index] + 1);
            QMeshNode* endNode = eB->installedIsoNode;
            isoEdge->SetEndPoint(endNode);

            isoEdge->SetMeshPatchPtr(layer);
            isoEdge->SetIndexNo(nextIsoEdgeId++);

            (startNode->GetEdgeList()).AddTail(isoEdge);
            (endNode->GetEdgeList()).AddTail(isoEdge);

            layer->GetEdgeList().AddTail(isoEdge);
            face->installedIsoEdge = isoEdge;
            face->isLocatedIsoEdge = true;
        }
        else { // positiveNum == 2
            // find negative node index
            if (n0->scalarField < isoValue) index = 0;
            else if (n1->scalarField < isoValue) index = 1;
            else index = 2;

             QMeshEdge* eA = face->GetEdgeRecordPtr(prevEdgeIdx[index] + 1);
            QMeshNode* startNode = eA->installedIsoNode;
            isoEdge->SetStartPoint(startNode);

            QMeshEdge* eB = face->GetEdgeRecordPtr(index + 1);
            QMeshNode* endNode = eB->installedIsoNode;
            isoEdge->SetEndPoint(endNode);

            isoEdge->SetMeshPatchPtr(layer);
            isoEdge->SetIndexNo(nextIsoEdgeId++);

            (startNode->GetEdgeList()).AddTail(isoEdge);
            (endNode->GetEdgeList()).AddTail(isoEdge);

            layer->GetEdgeList().AddTail(isoEdge);
            face->installedIsoEdge = isoEdge;
            face->isLocatedIsoEdge = true;
        }
    }

    // 4) build face list for isoSurface (from tetra list)
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

         QMeshFace* tf[4] = {
            tetra->GetFaceRecordPtr(1),
            tetra->GetFaceRecordPtr(2),
            tetra->GetFaceRecordPtr(3),
            tetra->GetFaceRecordPtr(4)
        };

        int isoEdgeNum = 0;
        for (int i = 0; i < 4; i++) if (tf[i]->isLocatedIsoEdge) isoEdgeNum++;

        if (isoEdgeNum == 0) continue;
        if (isoEdgeNum == 2 || isoEdgeNum == 1) {
            std::cout << "Error! isoEdgeNum cannot equal to 1 or 2!" << std::endl << std::endl;
            continue;
        }

         bool dir[4];
        for (int i = 0; i < 4; i++) dir[i] = tetra->IsNormalDirection(tetra->GetFaceIndex(tf[i]));

        if (isoEdgeNum == 3) {
            QMeshFace* isoFace = new QMeshFace;
            if (tetra->PSLNum > 0) isoFace->isCriticalFace = true;
            isoFace->principleStressDir = tetra->tau_max;
            isoFace->principleStress = tetra->sigma_max;

             QMeshFace* FaceList[3];
            int faceIndex = 0;
            for (int i = 0; i < 4; i++) {
                if (tf[i]->isLocatedIsoEdge) FaceList[faceIndex++] = tf[i];
            }

             const bool firstDir = tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[0]));
            if (firstDir) {
                if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
                    || FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
                    std::swap(FaceList[1], FaceList[2]);
                }
            }
            else {
                if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
                    || FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
                    std::swap(FaceList[1], FaceList[2]);
                }
            }

            for (int i = 0; i < 3; i++) {
                isoFace->SetEdgeRecordPtr(i, FaceList[i]->installedIsoEdge);
                const bool dflag = tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[i]));
                isoFace->SetDirectionFlag(i, dflag);
                if (dflag) FaceList[i]->installedIsoEdge->SetLeftFace(isoFace);
                else       FaceList[i]->installedIsoEdge->SetRightFace(isoFace);
            }

            isoFace->SetEdgeNum(3);
            isoFace->CalPlaneEquation();
            isoFace->SetMeshPatchPtr(layer);
            isoFace->SetIndexNo(nextIsoFaceId++);
            layer->GetFaceList().AddTail(isoFace);
        }
        else if (isoEdgeNum == 4) {

            QMeshFace* isoFace[2] = { new QMeshFace, new QMeshFace };
            for (int i = 0; i < 2; i++) {
                if (tetra->PSLNum > 0) isoFace[i]->isCriticalFace = true;
                isoFace[i]->principleStress = tetra->sigma_max;
                isoFace[i]->principleStressDir = tetra->tau_max;
            }

            QMeshFace* FaceList[4] = { tf[0], tf[1], tf[2], tf[3] };

            const bool firstDir = tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[0]));

            // sorting（保留原逻辑，减少循环内重复指针解引用）
            if (firstDir) {
                for (int i = 0; i < 2; i++) {
                    if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
                        || FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
                        std::swap(FaceList[1], FaceList[i + 2]);
                    }
                }
                if (!(FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
                    || FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())) {
                    std::swap(FaceList[2], FaceList[3]);
                }
            }
            else {
                for (int i = 0; i < 2; i++) {
                    if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
                        || FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
                        std::swap(FaceList[1], FaceList[i + 2]);
                    }
                }
                if (!(FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
                    || FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())) {
                    std::swap(FaceList[2], FaceList[3]);
                }
            }

            QMeshEdge* midEdge1 = new QMeshEdge;
            midEdge1->isMiddleEdge1 = true;
            if (firstDir) {
                midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());
                if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
                    midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
                else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
                    midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
                else std::cout << "Wrong case 1" << std::endl;
            }
            else {
                midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
                if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
                    midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
                else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
                    midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
                else std::cout << "Wrong case 2" << std::endl;
            }

            QMeshEdge* midEdge2 = new QMeshEdge;
            midEdge2->isMiddleEdge = true;
            if (firstDir) {
                midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
                if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
                    midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
                else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
                    midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
                else std::cout << "Wrong case 1" << std::endl;
            }
            else {
                midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());
                if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
                    midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
                else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
                    midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
                else std::cout << "Wrong case 2" << std::endl;
            }

            QMeshEdge* midEdge = nullptr;

            if (midEdge1->CalLength() <= midEdge2->CalLength()) {
                midEdge = midEdge1;

                isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
                isoFace[0]->SetDirectionFlag(0, tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[0])));
                isoFace[0]->SetEdgeRecordPtr(1, FaceList[1]->installedIsoEdge);
                isoFace[0]->SetDirectionFlag(1, tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[1])));
                isoFace[0]->SetEdgeRecordPtr(2, midEdge);
                isoFace[0]->SetDirectionFlag(2, false);

                for (int i = 0; i < 4; i++) {
                    if (i == 0 || i == 1) {
                        const bool dflag = tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[i]));
                        if (dflag) FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
                        else       FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
                    }
                }
                midEdge->SetRightFace(isoFace[0]);

                isoFace[1]->SetEdgeRecordPtr(0, FaceList[2]->installedIsoEdge);
                isoFace[1]->SetDirectionFlag(0, tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[2])));
                isoFace[1]->SetEdgeRecordPtr(1, FaceList[3]->installedIsoEdge);
                isoFace[1]->SetDirectionFlag(1, tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[3])));
                isoFace[1]->SetEdgeRecordPtr(2, midEdge);
                isoFace[1]->SetDirectionFlag(2, true);

                for (int i = 0; i < 4; i++) {
                    if (i == 2 || i == 3) {
                        const bool dflag = tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[i]));
                        // 注意：你原代码这里疑似写错(给 isoFace[0])，但“功能不变”我们不改
                        if (dflag) FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
                        else       FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
                    }
                }
                midEdge->SetLeftFace(isoFace[1]);
            }
            else {
                midEdge = midEdge2;

                isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
                isoFace[0]->SetDirectionFlag(0, tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[0])));
                isoFace[0]->SetEdgeRecordPtr(1, midEdge);
                isoFace[0]->SetDirectionFlag(1, true);
                isoFace[0]->SetEdgeRecordPtr(2, FaceList[3]->installedIsoEdge);
                isoFace[0]->SetDirectionFlag(2, tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[3])));

                for (int i = 0; i < 4; i++) {
                    if (i == 0 || i == 3) {
                        const bool dflag = tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[i]));
                        if (dflag) FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
                        else       FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
                    }
                }
                midEdge->SetLeftFace(isoFace[0]);

                isoFace[1]->SetEdgeRecordPtr(0, FaceList[1]->installedIsoEdge);
                isoFace[1]->SetDirectionFlag(0, tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[1])));
                isoFace[1]->SetEdgeRecordPtr(1, FaceList[2]->installedIsoEdge);
                isoFace[1]->SetDirectionFlag(1, tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[2])));
                isoFace[1]->SetEdgeRecordPtr(2, midEdge);
                isoFace[1]->SetDirectionFlag(2, false);

                for (int i = 0; i < 4; i++) {
                    if (i == 1 || i == 2) {
                        const bool dflag = tetra->IsNormalDirection(tetra->GetFaceIndex(FaceList[i]));
                        if (dflag) FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[1]);
                        else       FaceList[i]->installedIsoEdge->SetRightFace(isoFace[1]);
                    }
                }
                midEdge->SetRightFace(isoFace[1]);
            }

            // push back midEdge
            midEdge->SetMeshPatchPtr(layer);
            midEdge->SetIndexNo(nextIsoEdgeId++);
            layer->GetEdgeList().AddTail(midEdge);

            for (int i = 0; i < 2; i++) {
                isoFace[i]->SetEdgeNum(3);
                isoFace[i]->CalPlaneEquation();
                isoFace[i]->SetMeshPatchPtr(layer);
                isoFace[i]->SetIndexNo(nextIsoFaceId++);
                layer->GetFaceList().AddTail(isoFace[i]);
            }
        }
    }

    // 5) give each node face list
    for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);

        QMeshNode* a = face->GetNodeRecordPtr(0);
        QMeshNode* b = face->GetNodeRecordPtr(1);
        QMeshNode* c = face->GetNodeRecordPtr(2);

        a->GetFaceList().AddTail(face);
        b->GetFaceList().AddTail(face);
        c->GetFaceList().AddTail(face);
    }

    return layer;
}

void isoSurface::smoothingIsoSurface(PolygenMesh* isoSurface) {

	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoLayer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);

		for (GLKPOSITION Pos = isoLayer->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* thisEdge = (QMeshEdge*)isoLayer->GetEdgeList().GetNext(Pos);
			if (thisEdge->IsBoundaryEdge()) {
				thisEdge->GetStartPoint()->isBoundaryNode = true;
				thisEdge->GetEndPoint()->isBoundaryNode = true;
			}
		}

		//laplacian smoothness
		for (GLKPOSITION Pos = isoLayer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoLayer->GetNodeList().GetNext(Pos);

			if (thisNode->isBoundaryNode) continue;
			else {
				double pp[3] = { 0 }; int neighNum = 0;
				for (GLKPOSITION Pos = thisNode->GetEdgeList().GetHeadPosition(); Pos;) {
					QMeshEdge* neighEdge = (QMeshEdge*)thisNode->GetEdgeList().GetNext(Pos);

					QMeshNode* neighNode = neighEdge->GetStartPoint();
					if (neighNode == thisNode) neighNode = neighEdge->GetEndPoint();

					double p1[3];
					neighNode->GetCoord3D(p1[0], p1[1], p1[2]);

					for (int i = 0; i < 3; i++) pp[i] += p1[i];
					neighNum++;
				}
				for (int i = 0; i < 3; i++) pp[i] /= neighNum;
				thisNode->SetCoord3D(pp[0], pp[1], pp[2]);
			}
		}

	}
}

void isoSurface::outputSurface(PolygenMesh* isoSurface) {

}