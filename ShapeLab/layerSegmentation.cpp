#include "stdafx.h"
#include "layerSegmentation.h"
 
using namespace std;
using namespace Eigen;

void layerSegmentation::regionDetection(int holeNum) {

	int meshgenus = 0;
	int bNodeinprocessed = 0;

	//get boundary node and install
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
		if (Edge->IsBoundaryEdge()) {
			Edge->GetStartPoint()->isBoundaryNode = true;
			Edge->GetEndPoint()->isBoundaryNode = true;
		}
	}
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		double xx, yy, zz; Node->GetCoord3D(xx, yy, zz);
		if (Node->isBoundaryNode)
		{
			bNodeinprocessed++; Node->processed = false;
		}
	}
	
	cout<<"boundary node num = "<<bNodeinprocessed<<endl;

	if (bNodeinprocessed == 0) 
		std::cout << "This is a closed mesh! cannot used to generate tool path!" << std::endl;

	while (bNodeinprocessed != 0) {
		QMeshNode* startNode; //find the start node, should always be found
		for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
			startNode = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
			if (startNode->isBoundaryNode && startNode->processed == false) break;
		}
		startNode->regionIndex = meshgenus;
		startNode->processed = true;
		bNodeinprocessed--;

		QMeshNode* nextNode; bool nNodeDetected;
		int iterNum = 0;
		do {
			nNodeDetected = false;
			//Notice that, node->getnodelist won't return anything since we didn't install the information!
			for (GLKPOSITION Pos = startNode->GetEdgeList().GetHeadPosition(); Pos;) {
				QMeshEdge* connectEdge = (QMeshEdge*)startNode->GetEdgeList().GetNext(Pos);
				if (!connectEdge->IsBoundaryEdge()) continue;

				nextNode = connectEdge->GetEndPoint();
				if (nextNode == startNode) nextNode = connectEdge->GetStartPoint();

				if (nextNode->processed == false) {
					bNodeinprocessed--;
					nNodeDetected = true;
					nextNode->processed = true;
					startNode = nextNode;
					break;
				}
			}
			nextNode->regionIndex = meshgenus;
			iterNum++;
			if (iterNum > 1000) break;
		} while (nNodeDetected == true);
		
		meshgenus++;
	}

	surfaceMesh->seedPointNum = meshgenus;
	std::cout<< "The boundary ring of this mesh is "<< meshgenus << std::endl <<std::endl;

	//--------------------------------------------------------
	// reorder the hole and boundary region
	std::vector<int> nodeNumCount(meshgenus); for (int i = 0; i < meshgenus; i++)nodeNumCount[i] = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		if (node->isBoundaryNode == false) continue;
		nodeNumCount[node->regionIndex] += 1;
	}
	//for (int i = 0; i < meshgenus; i++) std::cout << nodeNumCount[i] << "||"; cout<< std::endl;

	//https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
	std::multimap<int, std::size_t> mm;
	for (std::size_t i = 0; i != nodeNumCount.size(); ++i)
		mm.insert({ nodeNumCount[i], i });
	std::vector<int> b;
	std::vector<std::size_t> sortRegionIndex;
	for (const auto& kv : mm) {
		b.push_back(kv.first);
		sortRegionIndex.push_back(kv.second);
	}
	std::reverse(b.begin(), b.end()); std::reverse(sortRegionIndex.begin(), sortRegionIndex.end()); // reverse order
	//cout << "sorted number = "; for (int i = 0; i < meshgenus; i++) std::cout << b[i] << "||"; cout << std::endl;
	//cout << "sorted index = "; for (int i = 0; i < meshgenus; i++) std::cout << sortRegionIndex[i] << "||"; cout << std::endl;

	int bounaryCount = 0;
	//nodeNumCount.maxCoeff(&bounaryCount); //boundary ring should have the largest node count

	// reorder the region index
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		node->isHole = false;
		if (node->isBoundaryNode == false) continue;

		for (int i = 0; i < meshgenus; i++) {
			if (node->regionIndex == sortRegionIndex[i]) {
				node->regionIndex = i;
				break;
			}
		}
		
		if (node->regionIndex < meshgenus - holeNum) {
			node->isHole = false; node->nodeHoleIndex = -1;
		}
		else {
			node->isHole = true; node->nodeHoleIndex = node->regionIndex - (meshgenus - holeNum);
		}
	}

	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		node->seedIndex = node->regionIndex;
		//if (node->isBoundaryNode == true) std::cout << node->seedIndex << std::endl;
	}

	std::cout << meshgenus << " || " << holeNum << std::endl;

	std::cout << " finish reorder node seed index and detect hole region! " << std::endl << std::endl;
}

//The boundShrinkRatio should be applied to model with complex geometry
void layerSegmentation::computeDistanceField_AllSeedPoints_holeRegionDetection_singleBoundaryCVT(
	QMeshPatch* patch, double boundShrinkRatio, int boundaryIndex, bool includeBoundary) {

	/* compute distance field from seed point */
	//int index = 0;
	//for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
	//	Node->seedValue = Eigen::VectorXd::Zero(patch->seedPointNum);
	//	Node->geoDisFlag.resize(patch->seedPointNum);
	//	Node->geoFieldValueSet.resize(patch->seedPointNum);
	//	Node->geoFieldValueSet_initialized.resize(patch->seedPointNum);

	//	Node->heatmethodIndexSet.resize(patch->seedPointNum);
	//	Node->SetIndexNo(index); index++;
	//	//std::cout << Node->seedIndex << std::endl;
	//}

	//for (int seedIndex = 0; seedIndex < patch->seedPointNum; seedIndex++) {

	//	QMeshPatch* computePatch = patch;

	//	//fieldNormalize = false;
	//	heatMethodField* heatFieldOperator = new heatMethodField(computePatch);

	//	for (GLKPOSITION Pos = computePatch->GetNodeList().GetHeadPosition(); Pos;) {
	//		QMeshNode* Node = (QMeshNode*)computePatch->GetNodeList().GetNext(Pos);
	//		Node->selected = false; Node->geoFieldValue = 0.0;

	//		if (Node->seedIndex == seedIndex) {
	//			Node->selected = true;
	//			Node->geoFieldValue = 1.0;
	//		}
	//	}

	//	heatFieldOperator->normalized = false;
	//	heatFieldOperator->runHeatMethod();

	//	Eigen::VectorXd fieldValue(computePatch->GetNodeNumber());
	//	int index = 0;
	//	for (GLKPOSITION Pos = computePatch->GetNodeList().GetHeadPosition(); Pos;) {
	//		QMeshNode* Node = (QMeshNode*)computePatch->GetNodeList().GetNext(Pos);

	//		if (seedIndex != 0) Node->geoFieldValue = Node->geoFieldValue * boundShrinkRatio;

	//		// !!!!!!!! this is for the new code - compute without boundary.
	//		//if (includeBoundary == false && seedIndex == 0) Node->geoFieldValue = 9999.99;

	//	/*	if (seedIndex != boundaryIndex && seedIndex < computePatch->seedPointNum - computePatch->holeNum)
	//			Node->geoFieldValue = 999999.99;*/

	//		fieldValue(index) = Node->geoFieldValue; index++;
	//	}
	//	//if(seedIndex == 0) std::cout << fieldValue << std::endl;

	//	Eigen::VectorXd fieldValueInitialized(computePatch->GetNodeNumber());
	//	double min = fieldValue.minCoeff(); double max = fieldValue.maxCoeff();
	//	for (int i = 0; i < computePatch->GetNodeNumber(); i++) fieldValueInitialized[i] = (fieldValue[i] - min) / (max - min);

	//	index = 0;
	//	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
	//		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
	//		//if (Node->geoFieldValue < 1.0) std::cout << Node->geoFieldValue << std::endl;
	//		Node->geoFieldValueSet[seedIndex] = fieldValue(index);
	//		
	//		Node->geoFieldValueSet_initialized[seedIndex] = fieldValueInitialized(index);

	//		Node->seedValue(seedIndex) = fieldValue(index);

	//		/*if (seedIndex != boundaryIndex && seedIndex < computePatch->seedPointNum - computePatch->holeNum)
	//			Node->seedValue(seedIndex) = 1000;*/

	//		index++;
	//		if (seedIndex == 0) Node->boundaryValue = Node->geoFieldValueSet[seedIndex];
	//	}

	//	delete heatFieldOperator;
	//	/*computePatch->ClearAll();
	//	delete computePatch;*/
	//}
	////}

	//// detect the cloest seed point for node
	//for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

	//	int minDisIndex = 0;

	//	/*if (Node->seedIndex != -1) Node->cloestSeedIndex = Node->seedIndex;
	//	else {
	//		Node->seedValue.maxCoeff(&minDisIndex);
	//		Node->cloestSeedIndex = minDisIndex;
	//	}*/

	//	if (Node->seedIndex != -1)
	//		Node->cloestSeedIndex = Node->seedIndex;
	//	else {
	//		Node->seedValue.minCoeff(&minDisIndex);
	//		Node->cloestSeedIndex = minDisIndex;
	//		//std::cout << Node->seedValue.maxCoeff() << ", " << Node->seedValue.minCoeff() << std::endl;
	//	}

	///*	for (int seedIndex = 0; seedIndex < patch->seedPointNum; seedIndex++)
	//		Node->seedValue(seedIndex) = Node->geoFieldValueSet[seedIndex];*/

	//}

	//patch->drawgeoField = true;
}

void layerSegmentation::boundarySegementation_reorderSeedIndex(QMeshPatch* patch)
{

	//// finish write this function !!!!!! 11-13-2022

	//for (int boundaryIndex = 0; boundaryIndex < patch->seedPointNum - patch->holeNum; boundaryIndex++) {
	//	std::vector<QMeshNode*> boundaryNodeSet;
	//	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
	//		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
	//		if (Node->nodeBoundaryIndex = boundaryIndex) boundaryNodeSet.push_back(Node);
	//	}
	//}

	//

	//cout << "boundary node num = " << bNodeinprocessed << endl;

	//if (bNodeinprocessed == 0)
	//	std::cout << "This is a closed mesh! cannot used to generate tool path!" << std::endl;

	//while (bNodeinprocessed != 0) {
	//	QMeshNode* startNode; //find the start node, should always be found
	//	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
	//		startNode = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
	//		if (startNode->isBoundaryNode && startNode->processed == false) break;
	//	}
	//	startNode->regionIndex = meshgenus;
	//	startNode->processed = true;
	//	bNodeinprocessed--;

	//	QMeshNode* nextNode; bool nNodeDetected;
	//	int iterNum = 0;
	//	do {
	//		nNodeDetected = false;
	//		//Notice that, node->getnodelist won't return anything since we didn't install the information!
	//		for (GLKPOSITION Pos = startNode->GetEdgeList().GetHeadPosition(); Pos;) {
	//			QMeshEdge* connectEdge = (QMeshEdge*)startNode->GetEdgeList().GetNext(Pos);
	//			if (!connectEdge->IsBoundaryEdge()) continue;

	//			nextNode = connectEdge->GetEndPoint();
	//			if (nextNode == startNode) nextNode = connectEdge->GetStartPoint();

	//			if (nextNode->processed == false) {
	//				bNodeinprocessed--;
	//				nNodeDetected = true;
	//				nextNode->processed = true;
	//				startNode = nextNode;
	//				break;
	//			}
	//		}
	//		nextNode->regionIndex = meshgenus;
	//		iterNum++;
	//		if (iterNum > 1000) break;
	//	} while (nNodeDetected == true);

	//	meshgenus++;
	//}

	//surfaceMesh->seedPointNum = meshgenus;
	//std::cout << "The boundary ring of this mesh is " << meshgenus << std::endl << std::endl;
}

//---------------Guoxin Fang - 10 Nov. 2022 Manchester University
void layerSegmentation::generateSplitedMeshPatch_CCFtoolpathGeneration_cutMesh(
	QMeshPatch* patch, QMeshPatch* skeletonPatch, QMeshPatch* newPatch) {

	//--------------------------------------------------
	// compute node number for new mesh

	int nodeNum = patch->GetNodeNumber();

	// VD node on the edge of original mesh
	for (GLKPOSITION Pos = skeletonPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)skeletonPatch->GetNodeList().GetNext(Pos);
		if (Node->relatedInitSurfaceEdge == nullptr) continue;
		if (Node->skeletonSelected) continue;
		Node->relatedInitSurfaceEdge->CCFSplitEdge = true;
		nodeNum += 2;
	}
	// VD node on the edge of original mesh
	for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);
		if (face->VDCornerCase == 0 || face->VDCornerCase == 1) nodeNum += 3;
	}

	//--------------------------------------------------
	// compute face number for new mesh

	int faceNum = patch->GetFaceNumber();

	for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);

		if (face->VDCornerCase == 0) faceNum += 5;
		else if (face->VDCornerCase == 1) faceNum += 5;
		else if (face->VDCornerCase == 2) continue;
		else {
			int edgeCount = 0;
			for (int i = 0; i < 3; i++) { if (face->GetEdgeRecordPtr(i + 1)->CCFSplitEdge) edgeCount++; }
			if (edgeCount == 2) {
				faceNum += 2; face->isCCFCut = true;
			}
			if (edgeCount == 3) {
				std::cout << "WARNING! edgeCount == 3 should not happen here!" << std::endl;
				//faceNum--; face->isCCFCutCenter = true;			
			}
		}
	}

	std::vector<QMeshNode*> newNodeSet;
	float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
	unsigned int* faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);

	// ------------------------ Build node/face table ------------------------

	int nodeIndex = 0; 		double pp[3];

	// ------------------------------------
	// build node table
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
		Node->SetIndexNo(nodeIndex);
		newNodeSet.push_back(Node);
		nodeIndex++;
	}
	/*for (GLKPOSITION Pos = skeletonPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)skeletonPatch->GetNodeList().GetNext(Pos);
		if (Node->relatedInitSurfaceEdge == nullptr) continue;
		if (Node->skeletonSelected) continue;
		Node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
		Node->SetIndexNo(nodeIndex);
		nodeIndex++;
	}*/

	// ------------------------ generate new node on the initial edge being cutting ------------------------
	double alpha = 0.9;
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		if (edge->CCFSplitEdge) {

			QMeshNode* sCutNode = new QMeshNode;
			QMeshNode* eCutNode = new QMeshNode;

			Vector3d origPP; edge->VDNode->GetCoord3D(origPP);
			Vector3d sNodePP; edge->GetStartPoint()->GetCoord3D(sNodePP);
			Vector3d eNodePP; edge->GetEndPoint()->GetCoord3D(eNodePP);

			Vector3d scutpp = alpha * origPP + (1 - alpha) * sNodePP;
			sCutNode->SetCoord3D(scutpp);

			Vector3d ecutpp = alpha * origPP + (1 - alpha) * eNodePP;
			eCutNode->SetCoord3D(ecutpp);

			sCutNode->SetIndexNo(nodeIndex);
			eCutNode->SetIndexNo(nodeIndex + 1);

			for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = scutpp[i];
			for (int i = 0; i < 3; i++) nodeTable[3 * (nodeIndex + 1) + i] = ecutpp[i];

			nodeIndex += 2;

			edge->sCutNode = sCutNode; edge->eCutNode = eCutNode;
			
			Eigen::Vector3d pp; sCutNode->GetCoord3D(pp); 
			//std::cout << "sCutNode->GetCoord3D(pp) = " << pp.transpose() << std::endl;
			eCutNode->GetCoord3D(pp); 
			//std::cout << "eCutNode->GetCoord3D(pp) = " << pp.transpose() << std::endl;

			newNodeSet.push_back(sCutNode);
			newNodeSet.push_back(eCutNode);

		}
	}

	// ------------------------ generate new node for VD corner, based on original face ------------------------
	std::vector<Eigen::Vector3i> faceNodeTable;

	for (GLKPOSITION Pos = skeletonPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)skeletonPatch->GetNodeList().GetNext(Pos);
		if (Node->isVDCorner_init) {
			QMeshFace* connectedFace = Node->connnectedVDFace;

			if (connectedFace->VDCornerCase == 0) {
				Vector3d initPP; Node->GetCoord3D(initPP);

				for (int vertIndex = 0; vertIndex < 3; vertIndex++) {

					Vector3d pos;
					connectedFace->GetNodeRecordPtr(vertIndex)->GetCoord3D(pos);
					Vector3d newpos = alpha * initPP + (1 - alpha) * pos;
					for (int i = 0; i < 3; i++)
						nodeTable[3 * nodeIndex + i] = newpos[i];

					QMeshNode* offsetNode = new QMeshNode;
					offsetNode->SetIndexNo(nodeIndex); offsetNode->SetCoord3D(newpos);
					newNodeSet.push_back(offsetNode);

					nodeIndex += 1;

				}

				// ----------- build cut face table 
				for (int i = 0; i < 3; i++) {
					QMeshNode* cornerNode = connectedFace->GetNodeRecordPtr(i);

					for (int j = 0; j < 3; j++) {
						QMeshEdge* thisEdge = connectedFace->GetEdgeRecordPtr(j + 1);
						if (thisEdge->GetStartPoint() == cornerNode) {
							Eigen::Vector3i faceIndex;
							faceIndex << cornerNode->GetIndexNo(), nodeIndex - 3 + i, thisEdge->sCutNode->GetIndexNo();
							faceNodeTable.push_back(faceIndex);
							
						}
						else if (thisEdge->GetEndPoint() == cornerNode) {
							Eigen::Vector3i faceIndex;
							faceIndex << cornerNode->GetIndexNo(), nodeIndex - 3 + i, thisEdge->eCutNode->GetIndexNo();
							faceNodeTable.push_back(faceIndex);
						}
					}

				}

				// correct normal direction

				for (int i = 0; i < 6; i++) {
					Eigen::Vector3i faceIndex = faceNodeTable[faceNodeTable.size() - 6 + i];
					bool flipFace = this->_fixNewCutFaceOrientation(
						newNodeSet[faceIndex(0)], newNodeSet[faceIndex(1)], newNodeSet[faceIndex(2)], connectedFace);

					if (flipFace) {
						int centerIndex = faceIndex(0); faceIndex(0) = faceIndex(2); faceIndex(2) = centerIndex;
					}

					faceNodeTable[faceNodeTable.size() - 6 + i] = faceIndex;
				}

				


			}
			else if (connectedFace->VDCornerCase == 1) {

				// find the edge not being cutted
				std::vector<QMeshEdge*> edgelist;
				for (int i = 0; i < 3; i++) edgelist.push_back(connectedFace->GetEdgeRecordPtr(i + 1));

				QMeshFace* neighborFace = connectedFace->connectedVDCornerFace;
				if (neighborFace->VDCornerCase != 2) std::cout << "ERROR! neighborFace->VDCornerCase!=2" << std::endl;
				for (int i = 0; i < 3; i++) {
					QMeshEdge* edge = neighborFace->GetEdgeRecordPtr(i + 1);

					bool sameEdgeFound = false;
					for (int j = 0; j < 3; j++) {
						if (edge == edgelist[j]) {
							edgelist.erase(edgelist.begin() + j);
							sameEdgeFound = true; break;
						}
					}
					if (!sameEdgeFound)
						edgelist.push_back(neighborFace->GetEdgeRecordPtr(i + 1));
				}

				if (edgelist.size() != 4) std::cout << "ERROR! edgelist.size() !=4" << std::endl;

				QMeshEdge* targetEdge; QMeshEdge* uncutEdge;
				for (int i = 0; i < edgelist.size(); i++) {
					if (!edgelist[i]->CCFSplitEdge) {
						uncutEdge = edgelist[i];
					}
				}
				for (int i = 0; i < edgelist.size(); i++) {
					QMeshEdge* thisEdge = edgelist[i];
					if (thisEdge == uncutEdge) continue;
					else {
						if (thisEdge->GetStartPoint() != uncutEdge->GetStartPoint() && thisEdge->GetStartPoint() != uncutEdge->GetEndPoint()
							&& thisEdge->GetEndPoint() != uncutEdge->GetStartPoint() && thisEdge->GetEndPoint() != uncutEdge->GetEndPoint()) {
							targetEdge = thisEdge; break;
						}
					}
				}

				// keep initial pos
				Vector3d initPP; Node->GetCoord3D(initPP);
				for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = initPP[i];

				QMeshNode* centerNode = new QMeshNode; centerNode->SetIndexNo(nodeIndex); centerNode->SetCoord3D(initPP);
				newNodeSet.push_back(centerNode);

				nodeIndex += 1;

				// two other offset node
				Vector3d pos;
				targetEdge->GetStartPoint()->GetCoord3D(pos);
				Vector3d newpos = alpha * initPP + (1 - alpha) * pos;
				for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = newpos[i];

				QMeshNode* offSetNode1 = new QMeshNode; offSetNode1->SetIndexNo(nodeIndex); offSetNode1->SetCoord3D(newpos);
				newNodeSet.push_back(offSetNode1);

				nodeIndex += 1;

				targetEdge->GetEndPoint()->GetCoord3D(pos);
				newpos = alpha * initPP + (1 - alpha) * pos;
				for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = newpos[i];

				QMeshNode* offSetNode2 = new QMeshNode; offSetNode2->SetIndexNo(nodeIndex); offSetNode2->SetCoord3D(newpos);
				newNodeSet.push_back(offSetNode2);

				nodeIndex += 1;

				//---------------------------------
				// ----------- build cut face table 

				// first, unchanged node - index = nodeIndex - 3;
				Eigen::Vector3i faceIndex;
				faceIndex << nodeIndex - 3, uncutEdge->GetStartPoint()->GetIndexNo(), uncutEdge->GetEndPoint()->GetIndexNo();
				faceNodeTable.push_back(faceIndex);

				int newFaceCount = 0;
				for (int i = 0; i < 4; i++) {
					if (edgelist[i] == uncutEdge) continue;
					if (edgelist[i] == targetEdge) continue;

					// one element
					if (edgelist[i]->GetStartPoint() == uncutEdge->GetStartPoint()) {
						faceIndex << nodeIndex - 3, uncutEdge->GetStartPoint()->GetIndexNo(), edgelist[i]->sCutNode->GetIndexNo();
						faceNodeTable.push_back(faceIndex); newFaceCount++;
					}
					if (edgelist[i]->GetEndPoint() == uncutEdge->GetStartPoint()) {
						faceIndex << nodeIndex - 3, uncutEdge->GetStartPoint()->GetIndexNo(), edgelist[i]->eCutNode->GetIndexNo();
						faceNodeTable.push_back(faceIndex); newFaceCount++;
					}

					// another one
					if (edgelist[i]->GetStartPoint() == uncutEdge->GetEndPoint()) {
						faceIndex << nodeIndex - 3, uncutEdge->GetEndPoint()->GetIndexNo(), edgelist[i]->sCutNode->GetIndexNo();
						faceNodeTable.push_back(faceIndex); newFaceCount++;
					}
					if (edgelist[i]->GetEndPoint() == uncutEdge->GetEndPoint()) {
						faceIndex << nodeIndex - 3, uncutEdge->GetEndPoint()->GetIndexNo(), edgelist[i]->eCutNode->GetIndexNo();
						faceNodeTable.push_back(faceIndex); newFaceCount++;
					}
				}
				if (newFaceCount != 2) std::cout << "ERROR! newFaceCount!=2" << std::endl;

				//second region
				QMeshNode* targetEdgeSNode = targetEdge->GetStartPoint();
				faceIndex << nodeIndex - 2, targetEdgeSNode->GetIndexNo(), targetEdge->sCutNode->GetIndexNo();
				faceNodeTable.push_back(faceIndex);

				newFaceCount = 0;
				for (int i = 0; i < 4; i++) {
					if (edgelist[i] == uncutEdge) continue;
					if (edgelist[i] == targetEdge) continue;
					// one element
					if (edgelist[i]->GetStartPoint() == targetEdgeSNode) {
						faceIndex << nodeIndex - 2, targetEdgeSNode->GetIndexNo(), edgelist[i]->sCutNode->GetIndexNo();
						faceNodeTable.push_back(faceIndex); newFaceCount++;
					}
					if (edgelist[i]->GetEndPoint() == targetEdgeSNode) {
						faceIndex << nodeIndex - 2, targetEdgeSNode->GetIndexNo(), edgelist[i]->eCutNode->GetIndexNo();
						faceNodeTable.push_back(faceIndex); newFaceCount++;
					}
				}
				if (newFaceCount != 1) std::cout << "ERROR! newFaceCount!=2" << std::endl;


				// third region
				QMeshNode* targetEdgeENode = targetEdge->GetEndPoint();

				faceIndex << nodeIndex - 1, targetEdgeENode->GetIndexNo(), targetEdge->eCutNode->GetIndexNo();
				faceNodeTable.push_back(faceIndex);

				newFaceCount = 0;
				for (int i = 0; i < 4; i++) {
					if (edgelist[i] == uncutEdge) continue;
					if (edgelist[i] == targetEdge) continue;
					// one element
					if (edgelist[i]->GetStartPoint() == targetEdgeENode) {
						faceIndex << nodeIndex - 1, targetEdgeENode->GetIndexNo(), edgelist[i]->sCutNode->GetIndexNo();
						faceNodeTable.push_back(faceIndex); newFaceCount++;
					}
					if (edgelist[i]->GetEndPoint() == targetEdgeENode) {
						faceIndex << nodeIndex - 1, targetEdgeENode->GetIndexNo(), edgelist[i]->eCutNode->GetIndexNo();
						faceNodeTable.push_back(faceIndex); newFaceCount++;
					}
				}
				if (newFaceCount != 1) std::cout << "ERROR! newFaceCount!=2" << std::endl;

				// check normal direction and flip if detected
				for (int i = 0; i < 7; i++) {
					Eigen::Vector3i faceIndex = faceNodeTable[faceNodeTable.size() - 7 + i];
					bool flipFace = this->_fixNewCutFaceOrientation(
						newNodeSet[faceIndex(0)], newNodeSet[faceIndex(1)], newNodeSet[faceIndex(2)], connectedFace);

					if (flipFace) {
						int centerIndex = faceIndex(0); faceIndex(0) = faceIndex(2); faceIndex(2) = centerIndex;
					}

					faceNodeTable[faceNodeTable.size() - 7 + i] = faceIndex;
				}


			}
		}
	}

	// ------------------------------------
	// build face table

	int faceIndex = 0;
	for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);

		if (face->isCCFCutCenter) continue;
		if (face->VDCornerCase != -1) continue;

		if (!face->isCCFCut) {
			for (int i = 0; i < 3; i++)
				faceTable[3 * faceIndex + i] = face->GetNodeRecordPtr(i)->GetIndexNo();
			faceIndex++;
		}
		else {
			// this is the face where being cut by iso-line
			QMeshEdge* noCutEdge; int noCutIndex = 0;
			for (int i = 0; i < 3; i++) {
				if (face->GetEdgeRecordPtr(i + 1)->CCFSplitEdge == false) {
					noCutEdge = face->GetEdgeRecordPtr(i + 1); noCutIndex = i; break;
				}
			}
			QMeshEdge* e1 = face->GetEdgeRecordPtr((noCutIndex + 1) % 3 + 1);
			QMeshEdge* e2 = face->GetEdgeRecordPtr((noCutIndex + 2) % 3 + 1);

			//QMeshNode* cutNode1 = e1->VDNode;
			//QMeshNode* cutNode2 = e2->VDNode;

			QMeshNode* face_n1 = noCutEdge->GetStartPoint(); 	QMeshNode* face_n2 = noCutEdge->GetEndPoint();

			QMeshNode* face_n3;
			for (int i = 0; i < 3; i++) {
				if (face->GetNodeRecordPtr(i) != face_n1 && face->GetNodeRecordPtr(i) != face_n2) {
					face_n3 = face->GetNodeRecordPtr(i); break;
				}
			}

			QMeshNode* cutSmallFaceN1; 			QMeshNode* cutSmallFaceN2;
			QMeshNode* cutLargeFaceN1; 			QMeshNode* cutLargeFaceN2;

			if (face_n3 == e1->GetStartPoint()) { cutSmallFaceN1 = e1->sCutNode; cutLargeFaceN1 = e1->eCutNode; }
			else { cutSmallFaceN1 = e1->eCutNode; cutLargeFaceN1 = e1->sCutNode; }

			if (face_n3 == e2->GetStartPoint()) { cutSmallFaceN2 = e2->sCutNode; cutLargeFaceN2 = e2->eCutNode; }
			else { cutSmallFaceN2 = e2->eCutNode; cutLargeFaceN2 = e2->sCutNode; }

			std::vector<QMeshNode*> reconstructNodeSet{
			face_n3,cutSmallFaceN2,cutSmallFaceN1,
			face_n1,cutLargeFaceN2,cutLargeFaceN1,
			face_n1,cutLargeFaceN2,face_n2};

			if (e2->GetStartPoint() == face_n1 || e2->GetEndPoint() == face_n1)
			{
				reconstructNodeSet[7] = cutLargeFaceN1;
			}

			// fix the orientation

			for (int faceIndex = 0; faceIndex < 3; faceIndex++) {
				if (this->_fixNewCutFaceOrientation(reconstructNodeSet[3 * faceIndex], 
					reconstructNodeSet[3 * faceIndex+1],
					reconstructNodeSet[3 * faceIndex+2], 
					face)) 
				{
					//std::cout << " reconstructNodeSet ==== flip face!!!!" << std::endl;
					QMeshNode* nodeChange = reconstructNodeSet[3 * faceIndex];
					reconstructNodeSet[3 * faceIndex] = reconstructNodeSet[3 * faceIndex + 2];
					reconstructNodeSet[3 * faceIndex + 2] = nodeChange;
				}
			}
			
			/*int index[9] = { face_n3->GetIndexNo(), cutSmallFaceN2->GetIndexNo(), cutSmallFaceN1->GetIndexNo(),
							face_n1->GetIndexNo(), cutLargeFaceN2->GetIndexNo(), cutLargeFaceN1->GetIndexNo(),
							face_n1->GetIndexNo(), cutLargeFaceN2->GetIndexNo(), face_n2->GetIndexNo() };*/

			/*int index[9] = { face_n3->GetIndexNo(), cutSmallFaceN2->GetIndexNo(), cutSmallFaceN1->GetIndexNo(),
							face_n1->GetIndexNo(), cutLargeFaceN2->GetIndexNo(), cutLargeFaceN1->GetIndexNo(),
							face_n2->GetIndexNo(), cutLargeFaceN2->GetIndexNo(), face_n1->GetIndexNo()};*/

			/*if (e2->GetStartPoint() == face_n1 || e2->GetEndPoint() == face_n1)
			{
				index[7] = cutLargeFaceN1->GetIndexNo();
			}*/




//			for (int i = 0; i < 9; i++) faceTable[3 * faceIndex + i] = index[i];

			for (int i = 0; i < 9; i++) faceTable[3 * faceIndex + i] = reconstructNodeSet[i]->GetIndexNo();

			faceIndex += 3;
		}

	}

	//---------------------------------------
	// put VD corner case back to face table
	std::cout << "VD corner -- faceNodeTable size = " << faceNodeTable.size() << std::endl;
	for (int i = 0; i < faceNodeTable.size(); i++) {
		Vector3i list = faceNodeTable[i];
		for (int i = 0; i < 3; i++) faceTable[3 * faceIndex + i] = list(i);
		faceIndex++;
	}

	/*for (int i = 0; i < 3* faceNum; i++) {
		if (i % 3 == 0) std::cout << endl;
		std::cout << faceTable[i] << " || ";
	}

	for (int i = 0; i < 3 * nodeNum; i++) {
		if (i % 3 == 0) std::cout << endl;
		std::cout << nodeTable[i] << " , ";
	}*/

	//--------------------------------------------
	// construct the new patch
	newPatch->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

	for (GLKPOSITION pos = newPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)newPatch->GetFaceList().GetNext(pos);
		face->selected = true;
		for (int i = 0; i < 3; i++) {
			if (face->GetEdgeRecordPtr(i + 1)->IsBoundaryEdge() == false) face->selected = false;
		}
	}

	std::vector<QMeshNode*> nodeSetNew;
	for (GLKPOSITION pos = newPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)newPatch->GetNodeList().GetNext(pos);
		nodeSetNew.push_back(node);
	}
	for (int i = 0; i < nodeSetNew.size(); i++)
	{
		Vector3d thisPos; Vector3d findPos;
		nodeSetNew[i]->GetCoord3D(thisPos);
		for (int j = 0; j < nodeSetNew.size(); j++) {
			if (i == j) continue;
			nodeSetNew[j]->GetCoord3D(findPos);
			if ((thisPos - findPos).norm() < 0.0000001) {
				std::cout << "ERROR, two point too close to each other!" << endl;
			}
		}
	}

	std::vector<QMeshEdge*> edgeSetNew;
	for (GLKPOSITION pos = newPatch->GetEdgeList().GetHeadPosition(); pos != nullptr;) {
		QMeshEdge* edge = (QMeshEdge*)newPatch->GetEdgeList().GetNext(pos);
		if(edge->IsBoundaryEdge()) edgeSetNew.push_back(edge);
	}

	for (int i = 0; i < edgeSetNew.size(); i++)
	{
		//std::cout << edgeSetNew[i]->GetStartPoint()->GetIndexNo() << " || " << edgeSetNew[i]->GetEndPoint()->GetIndexNo() << std::endl;
	}


	return;

	//--------------------------------------------
	// fix the order of generated mesh
	/*nodeNum = newPatch->GetNodeNumber();
	faceNum = newPatch->GetFaceNumber();
	unsigned int* faceTableNew = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);

	nodeIndex = 0;
	for (GLKPOSITION Pos = newPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)newPatch->GetNodeList().GetNext(Pos);
		Node->SetIndexNo(nodeIndex); nodeIndex++;
	}
	for (GLKPOSITION pos = patch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(pos);
		
	}*/


	//--------------------------------------------
	// transfer the flag to newpatch - for toolpath generation
	int index = 0;
	VectorXi nodeHoleSelection = VectorXi::Zero(patch->GetNodeNumber());
	VectorXi boundarySelection = VectorXi::Zero(patch->GetNodeNumber());

	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		if (Node->isHole) nodeHoleSelection(index) = 1;
		nodeHoleSelection(index) = Node->nodeHoleIndex;
		index++;
	}

	//std::cout << nodeHoleSelection.transpose() << std::endl;

	index = 0;
	for (GLKPOSITION Pos = newPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)newPatch->GetNodeList().GetNext(Pos);
		if (index < patch->GetNodeNumber()) {
			if (nodeHoleSelection(index) > -1) { Node->isHole = true; Node->selected = true; }
			//if (boundarySelection(index) == 1) Node->selected = true;
		}
		else Node->selected = true;
		index++;
	}

}

void layerSegmentation::repairNormalDirection(QMeshPatch* patch) {

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);

		std::vector<QMeshFace*> neighborFacelist;
		// find face neighbor
		for (int i = 0; i < 3; i++) {
			QMeshEdge* edge = face->GetEdgeRecordPtr(i + 1);
			if (edge->IsBoundaryEdge()) continue;
			
			if (edge->GetLeftFace() == face) neighborFacelist.push_back(edge->GetRightFace());
			else neighborFacelist.push_back(edge->GetLeftFace());
		}

		Vector3d norm; double D;
		Vector3d neighborNorm = Vector3d::Zero();
		for (int i = 0; i < neighborFacelist.size(); i++) {
			neighborFacelist[i]->CalPlaneEquation(); neighborFacelist[i]->GetPlaneEquation(norm(0), norm(1), norm(2), D);
			neighborNorm += norm;
		}
		neighborNorm = neighborNorm.normalized();

		face->CalPlaneEquation(); face->GetPlaneEquation(norm(0), norm(1), norm(2), D);

		if (neighborFacelist.size() == 0) {
			std::cout << "ERROR! neighborFacelist count number = " << neighborFacelist.size() << " ----- " << norm.transpose() << std::endl;
			face->selected = true;
		}
		else {
			//std::cout << norm.transpose() << " ----- " << neighborNorm.transpose() << std::endl;
		}

		if (norm.dot(neighborNorm) < 0) {
			std::cout << "rebuild mesh order" << std::endl;
			QMeshEdge* edge = face->GetEdgeRecordPtr(1);
			QMeshNode* leftNode = edge->GetStartPoint();
			QMeshNode* rightNode = edge->GetEndPoint();
			edge->SetStartPoint(rightNode);
			edge->SetEndPoint(leftNode);
		}

	}

}

bool layerSegmentation::_fixNewCutFaceOrientation(QMeshNode* n1, QMeshNode* n2, QMeshNode* n3, QMeshFace* origFace) {

	double x[3]; double y[3]; double z[3];
	n1->GetCoord3D(x[0], y[0], z[0]);
	n2->GetCoord3D(x[1], y[1], z[1]);
	n3->GetCoord3D(x[2], y[2], z[2]);

	double A = y[0] * (z[1] - z[2])
		+ y[1] * (z[2] - z[0])
		+ y[2] * (z[0] - z[1]);

	double B = z[0] * (x[1] - x[2])
		+ z[1] * (x[2] - x[0])
		+ z[2] * (x[0] - x[1]);

	double C = x[0] * (y[1] - y[2])
		+ x[1] * (y[2] - y[0])
		+ x[2] * (y[0] - y[1]);

	double D = -x[0] * (y[1] * z[2] - y[2] * z[1])
		- x[1] * (y[2] * z[0] - y[0] * z[2])
		- x[2] * (y[0] * z[1] - y[1] * z[0]);

	double  tt = A * A + B * B + C * C;
	tt = sqrt(tt);
	A = A / tt;   B = B / tt;   C = C / tt;   D = D / tt;

	Vector3d cutFaceDir; cutFaceDir << A, B, C;
	Vector3d origFaceDir; origFace->CalPlaneEquation();
	origFace->GetPlaneEquation(origFaceDir(0), origFaceDir(1), origFaceDir(2), D);

	cutFaceDir = cutFaceDir.normalized();  origFaceDir = origFaceDir.normalized();

	// debug usage
	if (fabs(cutFaceDir.dot(origFaceDir) < 0.99)) {
		/*std::cout << cutFaceDir.transpose() << std::endl;
		std::cout << origFaceDir.transpose() << std::endl;*/

		/*std::cout << x[0] << "," << y[0] << "," << z[0] << std::endl;
		std::cout << x[1] << "," << y[1] << "," << z[1] << std::endl;
		std::cout << x[2] << "," << y[2] << "," << z[2] << std::endl;

		std::cout << "------------------------------------" << std::endl;*/

		/*for (int i = 0; i < 3; i++) {
			Vector3d pp; origFace->GetNodeRecordPtr(i)->GetCoord3D(pp);
			std::cout << pp.transpose() << std::endl;
		}
		
		std::cout << "------------------------------------" << std::endl;

		std::cout << cutFaceDir.dot(origFaceDir) << std::endl;*/
	}
		

	if (cutFaceDir.dot(origFaceDir) < 0) return true;
	else return false;

}