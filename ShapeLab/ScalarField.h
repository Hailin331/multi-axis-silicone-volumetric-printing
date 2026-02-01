#pragma once
#include "..\QMeshLib\PolygenMesh.h"
#include <Eigen/PardisoSupport>

class ScalarField
{
public:
	ScalarField(QMeshPatch* mesh, bool support) { 
		tetMesh = mesh; NumhNode.resize(10,0);
	};
	~ScalarField() {};

	void compScalarField_initMesh();
	void compScalarField();
	void UpdateGuideField();

 	void initializematrixoptimize();

	void AlineToGuideField();
	double outputenergy();

private:
	QMeshPatch* tetMesh;
	void compTetMeshVolumeMatrix();
	//void compTetMeshVolumeMatrix_supportStructure(QMeshPatch* supportPatch);
	std::vector<int> NumhNode;
	Eigen::SparseMatrix<double> AMatalign;
	Eigen::SparseMatrix<double> AMatupdate;
 
	int AMatRowNumalign,fixnum = 0;
	int AMatRowNumupdate = 0;
	
	double alingweight = 10000, fixweight = 10000, handleweight = 10000, selectweight = 1800, harmonicweight= 8000, preserveweight = 3000;//the selection weight is used to eliminate the staircase of side surface 
	Eigen::PardisoLU <Eigen::SparseMatrix<double>> alignSolver;
	Eigen::PardisoLU <Eigen::SparseMatrix<double>> UpdateSolver;
	Eigen::VectorXd vecbUpdate;

  	Eigen::Vector3d fixVecDir;
	Eigen::Vector3d handleVecDir;


 };
