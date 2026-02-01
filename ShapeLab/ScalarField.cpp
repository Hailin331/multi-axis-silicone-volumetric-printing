#include "stdafx.h"
#include "ScalarField.h"
#include <iostream>
#include <fstream>
#include <math.h>
 

using namespace std;
using namespace Eigen;
void ScalarField::initializematrixoptimize() {

		int handletranum = 0;
		int fixnum = 0;
		int selectnum = 0;

  		// -----------------------------
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			bool tetraIsHandle = false;
			bool tetraIsSelected = false;
			bool tetraIsFixed = false;

 			QMeshFace* faces[4] = {
				Tetra->GetFaceRecordPtr(1),
				Tetra->GetFaceRecordPtr(2),
				Tetra->GetFaceRecordPtr(3),
				Tetra->GetFaceRecordPtr(4)
			};

			for (int i = 0; i < 4; i++) {
				QMeshFace* face = faces[i];

 				QMeshNode* n0 = face->GetNodeRecordPtr(0);
				QMeshNode* n1 = face->GetNodeRecordPtr(1);
				QMeshNode* n2 = face->GetNodeRecordPtr(2);

 				if (!tetraIsHandle) {
					if (n0->handleindex == n1->handleindex &&
						n1->handleindex == n2->handleindex &&
						n0->handleindex != INFINITE)
					{
						tetraIsHandle = true;
						Tetra->ishandletra = true;
						handletranum++;
						face->handleface = true;
 
					}
				}

				// selected face
				if (!tetraIsSelected) {
					if (n0->selected && n1->selected && n2->selected) {
						tetraIsSelected = true;
						Tetra->isselected = true;
						face->selectface = true;
						selectnum++;
					}
				}

				// fixed face
				if (!tetraIsFixed) {
					if (n0->isFixed && n1->isFixed && n2->isFixed) {
						tetraIsFixed = true;
						Tetra->isfixtra = true;
						face->fixedface = true;
						fixnum++;
					}
				}

 				if (tetraIsHandle && tetraIsSelected && tetraIsFixed) break;
			}
		}

		int facePair = 0;
		for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
			if (face->GetLeftTetra() != nullptr && face->GetRightTetra() != nullptr) facePair += 1;
		}

		int rownum = 3 * (handletranum)+3 * facePair + 3 * fixnum + selectnum;
		AMatRowNumupdate = rownum + 3 * tetMesh->GetTetraNumber();

		// -----------------------------
		// construct the matrix for compute scalar field
		// -----------------------------
		fixnum = 0;

 		if (!NumhNode.empty()) {
			std::fill(NumhNode.begin(), NumhNode.end(), 0);
		}

 		for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
			node->scalarField = 0.0;

			if (node->isFixed) {
				fixnum++;
				node->scalarField = 0.0;
			}
			if (node->handleindex != INFINITE) {
 				NumhNode[node->handleindex] += 1;
			}
		}

		AMatRowNumalign = 3 * tetMesh->GetTetraNumber() + fixnum;
		for (int i = 0; i < (int)NumhNode.size(); i++) {
			AMatRowNumalign += NumhNode[i];
		}

		AMatalign.resize(AMatRowNumalign, tetMesh->GetNodeNumber()); // A

		std::vector<Eigen::Triplet<double>> ParaTriplet;
 		ParaTriplet.reserve((size_t)(12 * tetMesh->GetTetraNumber() + 8 * tetMesh->GetNodeNumber()));

		int eqnum = 0;

		// alignment constraint
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			int tId = tetra->GetIndexNo();
			for (int i = 0; i < 4; i++) {
				int nId = tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();
				for (int k = 0; k < 3; k++) {
					ParaTriplet.push_back(Eigen::Triplet<double>(
						3 * tId + k,
						nId,
						alingweight * tetra->VolumeMatrix(i, k)
					));
				}
			}
			eqnum += 3;
		}

  		std::vector<std::vector<int>> groupNodes;
		groupNodes.resize(NumhNode.size());
		for (size_t h = 0; h < NumhNode.size(); ++h) {
			if (NumhNode[h] > 0) groupNodes[h].reserve((size_t)NumhNode[h]);
		}
		for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
			if (node->handleindex != INFINITE) {
				groupNodes[node->handleindex].push_back(node->GetIndexNo());
			}
		}

 		for (size_t h = 0; h < groupNodes.size(); ++h) {
			const auto& ids = groupNodes[h];
			const int m = (int)ids.size();
			if (m <= 0) continue;

			for (int ii = 0; ii < m; ++ii) {
				const int id_i = ids[ii];

				ParaTriplet.push_back(Eigen::Triplet<double>(
					eqnum,
					id_i,
					handleweight * (double)(m - 1)
				));

 				for (int jj = 0; jj < m; ++jj) {
					if (jj == ii) continue;
					ParaTriplet.push_back(Eigen::Triplet<double>(
						eqnum,
						ids[jj],
						-handleweight
					));
				}
				eqnum++;
			}
		}

		// fixface constraint 
		for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
			if (node->isFixed) {
				ParaTriplet.push_back(Eigen::Triplet<double>(eqnum, node->GetIndexNo(), fixweight));
				eqnum++;
			}
		}

		AMatalign.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

		Eigen::SparseMatrix<double> ATA(tetMesh->GetNodeNumber(), tetMesh->GetNodeNumber());
		ATA = AMatalign.transpose() * AMatalign;
		alignSolver.compute(ATA);

		///***************************************construct the matrix for updata vector field*************************************/
		eqnum = 0;
		ParaTriplet.clear();

		// [6] reserve£ºrownum  
		ParaTriplet.reserve((size_t)(6 * facePair + 24 * tetMesh->GetTetraNumber() + 12 * tetMesh->GetTetraNumber()));

		Eigen::VectorXd vecb(rownum); // b
		vecb.setZero();

		// harmonic field
		for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
			QMeshTetra* L = face->GetLeftTetra();
			QMeshTetra* R = face->GetRightTetra();
			if (L != nullptr && R != nullptr) {
				int lid = L->GetIndexNo();
				int rid = R->GetIndexNo();
				for (int i = 0; i < 3; i++) {
					ParaTriplet.push_back(Eigen::Triplet<double>(eqnum + i, 3 * lid + i, harmonicweight));
					ParaTriplet.push_back(Eigen::Triplet<double>(eqnum + i, 3 * rid + i, -harmonicweight));
				}
				eqnum += 3;
			}
		}

		// handle
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Trat = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			if (Trat->ishandletra) {
 				QMeshFace* faces[4] = {
					Trat->GetFaceRecordPtr(1),
					Trat->GetFaceRecordPtr(2),
					Trat->GetFaceRecordPtr(3),
					Trat->GetFaceRecordPtr(4)
				};

				for (int i = 0; i < 4; i++) {
					QMeshFace* face = faces[i];
					if (face->handleface) {
						double xx, yy, zz;
						face->GetNormal(xx, yy, zz);

						double normaldirchange = -1.0;
						if (zz > 0) normaldirchange = 1.0;

						int tid = Trat->GetIndexNo();
						ParaTriplet.push_back(Eigen::Triplet<double>(eqnum, 3 * tid, handleweight));
						ParaTriplet.push_back(Eigen::Triplet<double>(eqnum + 1, 3 * tid + 1, handleweight));
						ParaTriplet.push_back(Eigen::Triplet<double>(eqnum + 2, 3 * tid + 2, handleweight));

						vecb(eqnum) = normaldirchange * xx * handleweight;
						vecb(eqnum + 1) = normaldirchange * yy * handleweight;
						vecb(eqnum + 2) = normaldirchange * zz * handleweight;

						eqnum += 3;
					}
				}
			}
		}

		// fix
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Trat = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			if (Trat->isfixtra) {
				QMeshFace* faces[4] = {
					Trat->GetFaceRecordPtr(1),
					Trat->GetFaceRecordPtr(2),
					Trat->GetFaceRecordPtr(3),
					Trat->GetFaceRecordPtr(4)
				};

				for (int i = 0; i < 4; i++) {
					QMeshFace* face = faces[i];
					QMeshNode* n0 = face->GetNodeRecordPtr(0);
					QMeshNode* n1 = face->GetNodeRecordPtr(1);
					QMeshNode* n2 = face->GetNodeRecordPtr(2);

					if (n0->isFixed && n1->isFixed && n2->isFixed) {
						double xx, yy, zz;
						face->GetNormal(xx, yy, zz);

						int tid = Trat->GetIndexNo();
						ParaTriplet.push_back(Eigen::Triplet<double>(eqnum, 3 * tid, fixweight));
						ParaTriplet.push_back(Eigen::Triplet<double>(eqnum + 1, 3 * tid + 1, fixweight));
						ParaTriplet.push_back(Eigen::Triplet<double>(eqnum + 2, 3 * tid + 2, fixweight));

						vecb(eqnum) = xx * fixweight;
						vecb(eqnum + 1) = yy * fixweight;
						vecb(eqnum + 2) = zz * fixweight;

						eqnum += 3;
					}
				}
			}
		}

		// select
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Trat = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

			if (Trat->isselected) {
				double xx = 0.0, yy = 0.0, zz = 0.0;

				QMeshFace* faces[4] = {
					Trat->GetFaceRecordPtr(1),
					Trat->GetFaceRecordPtr(2),
					Trat->GetFaceRecordPtr(3),
					Trat->GetFaceRecordPtr(4)
				};

				for (int i = 0; i < 4; i++) {
					QMeshFace* face = faces[i];
					QMeshNode* n0 = face->GetNodeRecordPtr(0);
					QMeshNode* n1 = face->GetNodeRecordPtr(1);
					QMeshNode* n2 = face->GetNodeRecordPtr(2);
					if (n0->selected && n1->selected && n2->selected) {
						face->GetNormal(xx, yy, zz);
					}
				}

				int tid = Trat->GetIndexNo();
				 
				ParaTriplet.push_back(Eigen::Triplet<double>(eqnum, 3 * tid, xx * selectweight));
				ParaTriplet.push_back(Eigen::Triplet<double>(eqnum, 3 * tid + 1, yy * selectweight));
				ParaTriplet.push_back(Eigen::Triplet<double>(eqnum, 3 * tid + 2, zz * selectweight));
				eqnum++;
			}
		}

		Eigen::SparseMatrix<double> AMatInitial(rownum, 3 * tetMesh->GetTetraNumber());
		AMatInitial.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

		Eigen::SparseMatrix<double> ATAinitial(3 * tetMesh->GetTetraNumber(), 3 * tetMesh->GetTetraNumber());
		ATAinitial = AMatInitial.transpose() * AMatInitial;

		Eigen::PardisoLU<Eigen::SparseMatrix<double>> InitialSolver;
		InitialSolver.compute(ATAinitial);

		Eigen::VectorXd ATbInitial(3 * tetMesh->GetTetraNumber());
		ATbInitial = AMatInitial.transpose() * vecb;

		Eigen::VectorXd guideField(3 * tetMesh->GetTetraNumber()); // x
		guideField = InitialSolver.solve(ATbInitial);

		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* tra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			int tid = tra->GetIndexNo();
			for (int i = 0; i < 3; i++) {
				tra->guidevector(i) = guideField(3 * tid + i);
			}
			tra->guidevector = tra->guidevector.normalized();
		}

		// aling / preserve
		for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Trat = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
			int tid = Trat->GetIndexNo();
			for (int i = 0; i < 3; i++) {
				ParaTriplet.push_back(Eigen::Triplet<double>(eqnum, 3 * tid + i, preserveweight));
				eqnum++;
			}
		}

		AMatupdate.resize(AMatRowNumupdate, tetMesh->GetTetraNumber() * 3);
		AMatupdate.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

		Eigen::SparseMatrix<double> ATAUpdate(tetMesh->GetTetraNumber() * 3, tetMesh->GetTetraNumber() * 3);
		ATAUpdate = AMatupdate.transpose() * AMatupdate;
		UpdateSolver.compute(ATAUpdate);

		vecbUpdate = vecb;
		vecbUpdate.resize(AMatRowNumupdate);
	
}


void ScalarField::AlineToGuideField()
{
	Eigen::VectorXd guideField(tetMesh->GetNodeNumber()); //x
	Eigen::VectorXd b(AMatRowNumalign); //b
	b.setZero();

 
 	int eqnum = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {//alignment constraint
		QMeshTetra* tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		for (int i = 0; i < 3; i++)
		{
			b(3 * tetra->GetIndexNo() + i) = tetra->guidevector(i) * alingweight;
		}
		eqnum = eqnum + 3;
	}

	Eigen::VectorXd ATb(tetMesh->GetNodeNumber());
	ATb = AMatalign.transpose() * b;
	guideField = alignSolver.solve(ATb);
	
	Eigen::VectorXd guideFieldNormalize(tetMesh->GetNodeNumber());
	// compute max and min phis
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	for (int i = 0; i < tetMesh->GetNodeNumber(); i++) {
		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);
	}
	double range = maxPhi - minPhi;

	for (int i = 0; i < tetMesh->GetNodeNumber(); i++)  guideFieldNormalize(i) =  (guideField(i) - minPhi) / range;


	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->scalarField = guideFieldNormalize(Node->GetIndexNo());
 	}
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tetra->InitialVector = Vector3d::Zero();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++)
			Tetra->InitialVector(i) += Tetra->GetNodeRecordPtr(j + 1)->scalarField * Tetra->VolumeMatrix(j, i);
		}
			Tetra->vectorField = Tetra->InitialVector.normalized();
	}


}

void ScalarField::UpdateGuideField()
{
	Eigen::VectorXd guideField(tetMesh->GetNodeNumber()); //x

	//aling
	int equm = AMatRowNumupdate - 3*tetMesh->GetTetraNumber();
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Trat = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		for (int i = 0; i < 3; i++)
		{
			//ParaTriplet.push_back(Eigen::Triplet<double>(equm, 3 * Trat->GetIndexNo() + i, weight));
			vecbUpdate(equm) = Trat->vectorField(i)* preserveweight;
			equm++;
		}
	}


	Eigen::VectorXd ATb(tetMesh->GetTetraNumber() * 3);
	ATb = AMatupdate.transpose() * vecbUpdate;
	guideField = UpdateSolver.solve(ATb);


	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* tra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		for (int i = 0; i < 3; i++)
		{
			tra->guidevector(i) = guideField(3 * tra->GetIndexNo() + i);
		}
		tra->guidevector = tra->guidevector.normalized();
	}
}

double ScalarField::outputenergy()
{
	double ene=0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* tra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		ene += pow(tra->vectorField(0) - tra->guidevector(0), 2) + pow(tra->vectorField(1) - tra->guidevector(1), 2) + pow(tra->vectorField(2) - tra->guidevector(2), 2);
	}
	return ene;
}

void ScalarField::compScalarField() {

	this->compTetMeshVolumeMatrix();

	initializematrixoptimize();

	double preenergy = INFINITY;
	for (int i = 0; i <15; i++)
	{
		AlineToGuideField();
		double energy = outputenergy();
		cout << "energy=" << energy << endl;
		if (fabs(preenergy- energy)<1)
		{
			break;
		}
		preenergy = energy;
		UpdateGuideField();
	}

}



// function when vector field is assigned to the element and compute corresponding scalar field
void ScalarField::compScalarField_initMesh() {

	this->compTetMeshVolumeMatrix();
	
	int nodeIndex = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		node->SetIndexNo(nodeIndex); nodeIndex++;
	}

	// ---- method 1: apply laplacian to all the element (including constrained)

	Eigen::SparseMatrix<double> Parameter(3 * tetMesh->GetTetraNumber(), tetMesh->GetNodeNumber()); //A
	std::cout << 3 * tetMesh->GetTetraNumber() << "," << tetMesh->GetNodeNumber() << std::endl;

	Eigen::VectorXd guideField(tetMesh->GetNodeNumber()); //x

	Eigen::VectorXd b(3 * tetMesh->GetTetraNumber()); //b
	b.setZero();

	std::vector<Eigen::Triplet<double>> ParaTriplet;

	int TetIndex = 0;
	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
		Tetra->isTensileorCompressSelect = true;
		double weight = 1.0;
		Tetra->SetIndexNo(TetIndex); TetIndex++; ;


		for (int j = 0; j < 3; j++) {
			for (int i = 0; i < 4; i++) {
				QMeshNode* Node = Tetra->GetNodeRecordPtr(i + 1);
				ParaTriplet.push_back(Eigen::Triplet<double>(
					Tetra->GetIndexNo() * 3 + j, Node->GetIndexNo(), -Tetra->VolumeMatrix(i, j) * weight)); // infill A
			}
		}


		Vector3d CCFvectorField; CCFvectorField << 0.0, 0.0, 1.0;
		for (int i = 0; i < 3; i++) 
			//b(3*Tetra->GetIndexNo() + i) = Tetra->vectorField(i) * weight; // infill B
		    b(3 * Tetra->GetIndexNo() + i) = CCFvectorField(i) * weight; // infill B for CCF printing

	}

	Parameter.setFromTriplets(ParaTriplet.begin(), ParaTriplet.end());

	Eigen::SparseMatrix<double> ATA(tetMesh->GetNodeNumber(), tetMesh->GetNodeNumber());
	ATA = Parameter.transpose() * Parameter;
	Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;

	Solver.compute(ATA);

	Eigen::VectorXd ATb(tetMesh->GetNodeNumber());
	ATb = Parameter.transpose() * b;
	guideField = Solver.solve(ATb);

	Eigen::VectorXd guideFieldNormalize(tetMesh->GetNodeNumber());
	// compute max and min phis
	double minPhi = INFINITY;
	double maxPhi = -INFINITY;

	for (int i = 0; i < tetMesh->GetNodeNumber(); i++) {
		if (minPhi > guideField(i)) minPhi = guideField(i);
		if (maxPhi < guideField(i)) maxPhi = guideField(i);
	}
	double range = maxPhi - minPhi;

	for (int i = 0; i < tetMesh->GetNodeNumber(); i++)
		guideFieldNormalize(i) = 1 - (guideField(i) - minPhi) / range;

	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
		Node->scalarField = guideFieldNormalize(Node->GetIndexNo());
		//Node->scalarField_init = guideField(Node->GetIndexNo());
	}
}

void ScalarField::compTetMeshVolumeMatrix() {


	for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);

		Eigen::MatrixXd VolumeMatrix(4, 3);

		Vector3d aa, bb, cc, dd, pp;
		Tet->CalCenterPos(pp(0), pp(1), pp(2));

		Tet->GetNodeRecordPtr(1)->GetCoord3D(aa(0), aa(1), aa(2));
		Tet->GetNodeRecordPtr(2)->GetCoord3D(bb(0), bb(1), bb(2));
		Tet->GetNodeRecordPtr(3)->GetCoord3D(cc(0), cc(1), cc(2));
		Tet->GetNodeRecordPtr(4)->GetCoord3D(dd(0), dd(1), dd(2));

		Vector3d vap = pp - aa;
		Vector3d vbp = pp - bb;

		Vector3d vab = bb - aa;
		Vector3d vac = cc - aa;
		Vector3d vad = dd - aa;


		Vector3d vbc = cc - bb;
		Vector3d vbd = dd - bb;


		Vector3d bd_bc = vbd.cross(vbc);
		Vector3d ac_ad = vac.cross(vad);
		Vector3d ad_ab = vad.cross(vab);
		Vector3d ab_ac = vab.cross(vac);


		double volumeTet = Tet->CalVolume() * 1/6;//6->1/6

		VolumeMatrix.row(0) = bd_bc / (2*volumeTet);//volumeTet->2*volumeTet   a
		VolumeMatrix.row(1) = ac_ad / (2*volumeTet);//b
		VolumeMatrix.row(2) = ad_ab / (2*volumeTet);//c
		VolumeMatrix.row(3) = ab_ac / (2*volumeTet);//d
		Tet->VolumeMatrix = VolumeMatrix;

	}
}


