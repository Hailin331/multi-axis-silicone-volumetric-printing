#include "DeformToolPath.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include<stripe_Pattern.h>

void DeformToolPath::Generatetri()
{

//	std::cout << "facenum" << surfaceMesh->GetFaceNumber() << std::endl;
	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);

		Eigen::Matrix3d nodeposmat;
		for (int i = 0; i < 3; i++)
		{
			QMeshNode* node = face->GetNodeRecordPtr(i);
			node->GetCoord3D(nodeposmat(i, 0), nodeposmat(i, 1), nodeposmat(i, 2));
			node->GetCoord3D(InitialPos3D(node->GetIndexNo(), 0), InitialPos3D(node->GetIndexNo(), 1), InitialPos3D(node->GetIndexNo(), 2));
		}


		Eigen::Vector3d v1;
		Eigen::Vector3d v2;
		// Step 1: Calculate two vectors from the triangle's vertices.
		for (int i = 0; i < 3; i++)
		{
			v1(i) = nodeposmat(1, i) - nodeposmat(0, i);
			v2(i) = nodeposmat(2, i) - nodeposmat(0, i);
		}
		 
		Eigen::Matrix<double,3,3> Initiallocal;
		Initiallocal.setZero();
		for (int i = 0; i < 3; i++)//3D->2D
		{
			Initiallocal(i, 0) = nodeposmat(0, i);
			Initiallocal(i, 1) = nodeposmat(1, i);
			Initiallocal(i, 2) = nodeposmat(2, i);
		}
		Initiallocal = removemean(Initiallocal);
		FaceInitialpos3D.at(face->GetIndexNo()) = Initiallocal;
		FaceCurrentpos3D.at(face->GetIndexNo()) = Initiallocal;
		face->GetNormal(InitialNor3D(face->GetIndexNo(), 0), InitialNor3D(face->GetIndexNo(), 1), InitialNor3D(face->GetIndexNo(), 2));
 	}
}
//Get the initialGuess
void DeformToolPath::GenerateInitialGuess() {
	
	//for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
	//{
	//	QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
	//	Eigen::Vector3d pos3d,nor;
	//	node->GetCoord3D(pos3d(0), pos3d(1), pos3d(2));
	//	node->GetNormal(nor(0), nor(1), nor(2));
	//	for (int i = 0; i < 3; i++)
	//	{
	//		InitialPos3D(node->GetIndexNo(), i) = pos3d(i);
	//		InitialNor3D(node->GetIndexNo(), i) = nor(i);
	//	}
	//}
	//double tempvar;

	//for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
	//{
	//	QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
	//	Eigen::Matrix<double,2,3> posmat = Eigen::MatrixXd::Zero(2, 3);
	//	for (int i = 0; i < 3; i++)
	//	{
	//		face->GetNodeRecordPtr(i)->GetCoord3D(posmat(0, i), posmat(1, i), tempvar);//earse the z and directly set the surface to the 2D plan
	////		face->GetNodeRecordPtr(i)->SetCoord3D(posmat(0, i), posmat(1, i), 0);//earse the z and directly set the surface to the 2D plan
	//	}
	//	posmat = removemean(posmat);
	//	FaceCurrentpos2D[face->GetIndexNo()]=posmat;
	//}

}

void DeformToolPath::ComputeRotation() {
	int nFace = surfaceMesh->GetFaceNumber();

#pragma omp parallel for
	for (int i = 0; i < nFace; i++) {
		Eigen::Matrix3d Simatrix;
		Eigen::Matrix3d scalematrix = scalevaluelist.at(i) * Eigen::Matrix3d::Identity();

		//FaceInitialpos3D.at(i) = FaceCurrentpos3D.at(i);

		Simatrix = scalematrix*removemean(FaceInitialpos3D.at(i)) * (removemean(FaceCurrentpos3D.at(i))).transpose();
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(Simatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

		Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();

		Facerotationmatrix.at(i)= rotation;
 
 	}
	//std::cout << surfaceMesh->GetFaceNumber() << "... " << count << std::endl;
}

Eigen::Matrix<double, 3, 3> DeformToolPath::removemean( Eigen::Matrix<double, 3, 3>& nodeposmat) {
	// Calculate the mean position
	Eigen::Vector3d centerpos = nodeposmat.rowwise().mean(); // Compute the mean for each row

	nodeposmat.col(0) -= centerpos;
	nodeposmat.col(1) -= centerpos;
	nodeposmat.col(2) -= centerpos;

	return nodeposmat;
}


void DeformToolPath::FullfillMartrixA(Eigen::SparseMatrix<double>& Amat) {
	int nFace = surfaceMesh->GetFaceNumber();
	int nNode = surfaceMesh->GetNodeNumber();
	int rows = nFace * 3 * 3;
	int cols = nNode * 3;
	std::vector<Eigen::Triplet<double>> triplets;
	triplets.reserve(nFace * 3 * 3);  

	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;) {
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		int fIdx = face->GetIndexNo();
		for (int i = 0; i < 3; i++) {
			int nodeI = face->GetNodeRecordPtr(i)->GetIndexNo();
			for (int j = 0; j < 3; j++) {
				int nodeJ = face->GetNodeRecordPtr(j)->GetIndexNo();
				double val = (i == j) ? (2.0 / 3.0) : (-1.0 / 3.0);
			 
				triplets.emplace_back(3 * (3 * fIdx + i), 3 * nodeJ, val);
				 
				triplets.emplace_back(3 * (3 * fIdx + i) + 1, 3 * nodeJ + 1, val);
			 
				triplets.emplace_back(3 * (3 * fIdx + i) + 2, 3 * nodeJ + 2, val);
			}
		}
	}
	Amat.resize(rows, cols);
	Amat.setFromTriplets(triplets.begin(), triplets.end());
	Amat.makeCompressed();  
}

void DeformToolPath::FullfillMatrixB(Eigen::VectorXd& vecB){

	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos !=NULL;)
	{
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		Eigen::Matrix3d scalematrix = scalevaluelist.at(face->GetIndexNo()) * Eigen::Matrix3d::Identity();
		//std::cout << scalematrix << std::endl;
		Eigen::Matrix<double,3,3> rotinitial=Facerotationmatrix.at(face->GetIndexNo())* scalematrix*removemean(FaceInitialpos3D.at(face->GetIndexNo()));
		//std::cout << "intial" << removemean2(FaceInitialpos2D.at(face->GetIndexNo())) << std::endl;
	//	std::cout <<"faceindex"<<face->GetIndexNo() << rotinitial << std::endl;
		for (int i = 0; i < 3; i++)//Node
		{
			for (int j = 0; j < 3; j++)//xyz
			{
				vecB(3 * (3 * face->GetIndexNo() + i) + j) = rotinitial(j, i);
			}
		}
	}

}

void DeformToolPath::savescalefield() {
	QString filenameStr = QString("../model/isoSurface/Geoditance for TTA2/scalevalue%1.txt").arg(surfaceMesh->GetIndexNo());
	QFileInfo fileInfo(filenameStr);
	QDir dir = fileInfo.absoluteDir();
	if (!dir.exists()) {
		dir.mkpath(".");
	}
	QFile exportFile(filenameStr);
	if (!exportFile.open(QFile::WriteOnly | QFile::Truncate)) {
		//QMessageBox::warning(this, tr("Error"), tr("Cannot open file for writing."));
		std::cout << "file open error" << std::endl;
		return;
	}
	QTextStream out(&exportFile);
 
	//Ð´Èë
	double minwidth = 200000;
	for (GLKPOSITION posNode = surfaceMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(posNode);
		double averagewidth = 0;
		int neighborfacenumber = 0;
		for (GLKPOSITION posFace = node->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
			QMeshFace* face = (QMeshFace*)node->GetFaceList().GetNext(posFace);
				neighborfacenumber++;
				averagewidth += scalevaluelist.at(face->GetIndexNo());
			}
		averagewidth = averagewidth / (double)neighborfacenumber;
		if (minwidth > averagewidth) {
			minwidth = averagewidth;
		}
  	}

	for (GLKPOSITION posNode = surfaceMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(posNode);
		double averagewidth = 0;
		int neighborfacenumber = 0;
		for (GLKPOSITION posFace = node->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
			QMeshFace* face = (QMeshFace*)node->GetFaceList().GetNext(posFace);
			neighborfacenumber++;
			averagewidth += scalevaluelist.at(face->GetIndexNo());
		}
		averagewidth = averagewidth / (double)neighborfacenumber;
		out << 1.0 / (averagewidth / minwidth)<< "\n";
	}

	exportFile.close();
	std::cout << "save SCALE" << std::endl;	

}

void DeformToolPath::savedeformedmesh()
{
 
	QString filenameStr = QString("../model/isoSurface/Deformed shape for TTA2/%1.obj")
		.arg(surfaceMesh->GetIndexNo());
 	if (!filenameStr.endsWith(".obj", Qt::CaseInsensitive)) {
		filenameStr += ".obj";
	}


	QFileInfo fileInfo(filenameStr);
	QDir dir = fileInfo.absoluteDir();
	if (!dir.exists()) {
		dir.mkpath(".");
	}

	QFile exportFile(filenameStr);
	if (!exportFile.open(QFile::WriteOnly | QFile::Truncate)) {
		//QMessageBox::warning(this, tr("Error"), tr("Cannot open file for writing."));
		std::cout << "file open error" << std::endl;
		return;
	}

	QTextStream out(&exportFile);

 	for (GLKPOSITION posNode = surfaceMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(posNode);
		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		out << "v " << xx << " " << yy << " " << zz << "\n";
	}

 	for (GLKPOSITION posFace = surfaceMesh->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(posFace);
 		out << "f "
			<< face->GetNodeRecordPtr(0)->GetIndexNo()+1 << " "
			<< face->GetNodeRecordPtr(1)->GetIndexNo()+1 << " "
			<< face->GetNodeRecordPtr(2)->GetIndexNo()+1 << "\n";
	}

	exportFile.close();
	std::cout << "save deformed mesh" << std::endl;
}


//deformation
void DeformToolPath::Deformation3D() {
	//generate MatA
	int loopnum = 10;
	Generatetri();
	//GenerateInitialGuess();
//	std::cout << "finish INitial 2D" << std::endl;
	Eigen::SparseMatrix<double> Amat;
	Eigen::VectorXd vecB;
	FullfillMartrixA(Amat);
	Eigen::PardisoLDLT<Eigen::SparseMatrix<double>> Solver;
	Solver.compute(Amat.transpose() * Amat);
	vecB.resize(3 * 3 * surfaceMesh->GetFaceNumber()); vecB.setZero();

	for (int j = 0; j <loopnum ; j++)
	{
		///**local***/
		//auto start = std::chrono::high_resolution_clock::now();
		ComputeRotation();
		FullfillMatrixB(vecB);
		Eigen::VectorXd Atxvecb = Amat.transpose() * vecB;
		Eigen::VectorXd currentpos = Solver.solve(Atxvecb);

		for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
		{
			QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		//	FaceCurrentpos2D.at(face->GetIndexNo()) = Eigen::Matrix<double, 2, 3>::Zero();
			for (int i = 0; i < 3; i++)
			{
				FaceCurrentpos3D.at(face->GetIndexNo())(0, i) = currentpos(face->GetNodeRecordPtr(i)->GetIndexNo()*3);
				FaceCurrentpos3D.at(face->GetIndexNo())(1, i) = currentpos(face->GetNodeRecordPtr(i)->GetIndexNo()*3+1);
				FaceCurrentpos3D.at(face->GetIndexNo())(2, i) = currentpos(face->GetNodeRecordPtr(i)->GetIndexNo() * 3 + 2);

 			}
		}
	}

	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);

		for (int i = 0; i < 3; i++)
		{
			face->GetNodeRecordPtr(i)->SetCoord3D(FaceCurrentpos3D.at(face->GetIndexNo())(0, i), FaceCurrentpos3D.at(face->GetIndexNo())(1, i), FaceCurrentpos3D.at(face->GetIndexNo())(2, i));
		}
		double xx, yy, zz;
		face->CalPlaneEquation();
		face->GetNormal(xx, yy, zz);
		DeformedNor3D(face->GetIndexNo(), 0) = xx;
		DeformedNor3D(face->GetIndexNo(), 1) = yy;
		DeformedNor3D(face->GetIndexNo(), 2) = zz;
	}
	
	for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
	{
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		DeformedPos3D(node->GetIndexNo(), 0) = xx;
		DeformedPos3D(node->GetIndexNo(), 1) = yy;
		DeformedPos3D(node->GetIndexNo(), 2) = zz;

	}
	deformstate = true;
}
void DeformToolPath::ComputeScaling() {

	double maxscale=-INFINITE,minscale=INFINITE;
	double avescale = 0;
	for (size_t i = 0; i < scalevaluelist.size(); i++)
	{
		if (maxscale < scalevaluelist.at(i)) maxscale = scalevaluelist.at(i);
		if (minscale > scalevaluelist.at(i)) minscale = scalevaluelist.at(i);
		avescale+= scalevaluelist.at(i);
	}
	avescale = avescale / (double)scalevaluelist.size();
	averagewidth = avescale;
	//std::cout << "maxscale" << maxscale << std::endl;
	//std::cout << "minscale" << minscale << std::endl;
	minwidthpath = maxscale;
	//std::cout << "minwidthpath=" << minwidthpath << std::endl;
	for (size_t i = 0; i < scalevaluelist.size(); i++)
	{
		scalevaluelist.at(i) = scalevaluelist.at(i)/ avescale;
		scalevaluelist.at(i) = 1.0 / scalevaluelist.at(i);
	}
}

void DeformToolPath::faceAreaCompute(std::vector<double>& facearea) {

	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		Eigen::Vector3d pos0,pos1, pos2;
		face->GetNodeRecordPtr(0)->GetCoord3D(pos0);
		face->GetNodeRecordPtr(1)->GetCoord3D(pos1);
		face->GetNodeRecordPtr(2)->GetCoord3D(pos2);
		Eigen::Vector3d vec1, vec2;
		vec1 = pos1 - pos0;
		vec2 = pos2 - pos0;
		facearea.at(face->GetIndexNo()) = 0.5*vec1.cross(vec2).norm();
	}
}


void DeformToolPath::OutputDifferenceArea() {

	std::ofstream Triarea(AreaDifferencepath);
	std::ofstream Scalearea(Scalepath);
	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != nullptr;) {
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		Triarea <<( Deformedfacearea .at(face->GetIndexNo()))/(Initialfacearea.at(face->GetIndexNo())) << std::endl;
		Scalearea<< (
			(scalevaluelist.at(face->GetIndexNo()))- 
			sqrt((Deformedfacearea.at(face->GetIndexNo())) / (Initialfacearea.at(face->GetIndexNo())))
			)
			/ (scalevaluelist.at(face->GetIndexNo()))  <<std::endl;
	}
	Triarea.close();
	Scalearea.close();
	//std::cout << "area output" << std::endl;
}
void DeformToolPath::ConverttoInitial() {

	for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != nullptr;) {
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
		node->SetCoord3D(InitialPos3D(node->GetIndexNo(), 0), InitialPos3D(node->GetIndexNo(), 1), InitialPos3D(node->GetIndexNo(), 2));
 	}
	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != nullptr;) {
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		face->SetNormal(InitialNor3D(face->GetIndexNo(), 0), InitialNor3D(face->GetIndexNo(), 1), InitialNor3D(face->GetIndexNo(), 2));
 	}
	 
}
void DeformToolPath::ConverttoDeforma(){

	for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != nullptr;) {
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
		node->SetCoord3D(DeformedPos3D(node->GetIndexNo(), 0), DeformedPos3D(node->GetIndexNo(), 1), DeformedPos3D(node->GetIndexNo(), 2));
	}
	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != nullptr;) {
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		face->SetNormal(DeformedNor3D(face->GetIndexNo(), 0), DeformedNor3D(face->GetIndexNo(), 1), DeformedNor3D(face->GetIndexNo(), 2));
	}
}


void DeformToolPath::assignvectortoface() {
	std::vector<Eigen::Vector3d> vectorpos;
	std::vector<Eigen::Vector3d> vectordir;
	std::string face_input_path = "../model/isoSurface/cut_TTA2/facevector2.txt";
	std::ifstream infile(face_input_path);
	if (!infile.is_open()) {
		std::cerr << "Cannot open file: " << face_input_path << std::endl;
		return;
	}
	std::string line;
	while (std::getline(infile, line)) {
		std::stringstream ss(line);
		std::string item;
		std::vector<double> values;
		while (std::getline(ss, item, ',')) {
			try {
				values.push_back(std::stod(item));
			}
			catch (const std::exception& e) {
				std::cerr << "Error parsing double: " << item << std::endl;
			}
		}
		if (values.size() == 6) {
			Eigen::Vector3d pos(values[0], values[1], values[2]);
			Eigen::Vector3d dir(values[3], values[4], values[5]);
			vectorpos.push_back(pos);
			vectordir.push_back(dir);
		}
		else {
			std::cerr << "Invalid line (not 6 values): " << line << std::endl;
		}
	}

	infile.close();
	std::cout << "vectornode=" << vectordir.size() << std::endl;
	/*assign vector*/
	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		//double nodeangle = atan2(solutioncomplex(2 * node->GetIndexNo() + 1), solutioncomplex(2 * node->GetIndexNo()));
		//std::cout << "nodeangle" << nodeangle << std::endl;
		Eigen::Vector3d facecenter;
		face->CalCenterPos(facecenter(0), facecenter(1), facecenter(2));
		int minindex = 0;
		double mindis = std::numeric_limits<double>::max();
		for (int i = 0; i < vectorpos.size(); i++)
		{
			double dis = (facecenter - vectorpos[i]).norm();
			if (dis < mindis) {
				mindis = dis;
				minindex = i;
			}
		}
		face->fieldVec = vectordir[minindex].normalized();
	}
}


void DeformToolPath::Generatefinaltoolpath() {

	//field controlled deformation
	faceAreaCompute(Initialfacearea);
	ComputeScaling();
	Deformation3D();
	faceAreaCompute(Deformedfacearea);
	getvolumeoflayer();
	surfaceMesh->drawlayerheight = false;
	if (!rotationflag) {
		fre = 0;
	}
	//compute geo_distance
	ComputeGeoViaigl* computegeo = new ComputeGeoViaigl(surfaceMesh);
	computegeo->Compute_Geo();

	if (rotationflag)
	{
		/*compute vecfield*/
		for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);
			Eigen::Vector3d gradient, normal;
			gradient.setZero();
			Face->CalPlaneEquation();
			Face->GetNormal(normal(0), normal(1), normal(2));
			normal.normalize();

			for (int i = 0; i < 3; i++) {
				QMeshNode* Node = Face->GetNodeRecordPtr(i % 3);
				QMeshNode* Node1 = Face->GetNodeRecordPtr((i + 1) % 3);
				QMeshNode* Node2 = Face->GetNodeRecordPtr((i + 2) % 3);

				double ui = Node->DisToBoundary;
				double pp1[3], pp2[3];
				Node1->GetCoord3D(pp1[0], pp1[1], pp1[2]);
				Node2->GetCoord3D(pp2[0], pp2[1], pp2[2]);

				Eigen::Vector3d ei = Eigen::Vector3d(pp2[0] - pp1[0], pp2[1] - pp1[1], pp2[2] - pp1[2]);
				gradient += ui * normal.cross(ei);
			}
			gradient /= (2.0 * Face->CalArea());
			gradient.normalize();
			//cout << gradient << endl;
			for (int i = 0; i < 3; i++) {
				Face->fieldVec(i) = gradient(i);
			}
		}


		//rotation 90¡ã
		double rotationAngle = 90.0 * 3.14159 / 180; // Define the rotation angle in radians (e.g., M_PI/4 for 45¡ã)

		for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);
			Eigen::Vector3d facenor;
			Face->GetNormal(facenor(0), facenor(1), facenor(2));
			facenor.normalize(); // Ensure the normal is a unit vector

			// Rodrigues' rotation formula
			Eigen::Vector3d& v = Face->fieldVec; // Reference to fieldVec for convenience
			Eigen::Vector3d rotatedVec = v * cos(rotationAngle)
				+ facenor.cross(v) * sin(rotationAngle)
				+ facenor * (facenor.dot(v)) * (1 - cos(rotationAngle));

			Face->fieldVec = rotatedVec.normalized(); // Update fieldVec with the rotated vector
		}


		int smoothloop = 10; // itenum smooth the vec field
		const int Fnum = surfaceMesh->GetFaceNumber();

 		std::vector<Eigen::Vector3d> cur(Fnum), out(Fnum);

 		auto snapshot_field = [&]() {
			for (GLKPOSITION pos = surfaceMesh->GetFaceList().GetHeadPosition(); pos;) {
				QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(pos);
				int idx = face->GetIndexNo();
				cur[idx] = face->fieldVec;
			}
			};

		std::vector<double> faceArea(Fnum, 0.0);
		for (GLKPOSITION pos = surfaceMesh->GetFaceList().GetHeadPosition(); pos;) {
			QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(pos);
			face->CalArea();
			faceArea[face->GetIndexNo()] = face->GetArea();
		}

		for (int iter = 0; iter < smoothloop; ++iter) {
			snapshot_field();

			for (GLKPOSITION pos = surfaceMesh->GetFaceList().GetHeadPosition(); pos;) {
				QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(pos);
				const int idx = face->GetIndexNo();

				const Eigen::Vector3d ref = cur[idx]; // 
				Eigen::Matrix3d T = Eigen::Matrix3d::Zero();
				double wsum = 0.0;

				
				if (ref.squaredNorm() > 0) {
					const double w0 = std::max(1e-12, 1e-6 * faceArea[idx]);
					T += w0 * (ref.normalized() * ref.normalized().transpose());
					wsum += w0;
				}

			 
				for (int k = 0; k < 3; ++k) {
					QMeshEdge* e = face->GetEdgeRecordPtr(k + 1);
					QMeshFace* nbrs[2] = { e->GetLeftFace(), e->GetRightFace() };
					for (QMeshFace* nbr : nbrs) {
						if (!nbr || nbr == face) continue;
						const int j = nbr->GetIndexNo();
						const Eigen::Vector3d vn = cur[j];
						if (vn.squaredNorm() == 0) continue;

						const double w = faceArea[j]; //  
						const Eigen::Vector3d vhat = vn.normalized();
						T += w * (vhat * vhat.transpose());  
						wsum += w;
					}
				}

				Eigen::Vector3d newdir = ref; // 
				if (wsum > 0) {
					 
					Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(T);
					if (es.info() == Eigen::Success) {
						 
						newdir = es.eigenvectors().col(2);
					}
				}

 				if (ref.squaredNorm() > 0 && newdir.dot(ref) < 0) newdir = -newdir;

 				if (newdir.squaredNorm() > 0) newdir.normalize();
				out[idx] = newdir;
			}

 			for (GLKPOSITION pos = surfaceMesh->GetFaceList().GetHeadPosition(); pos;) {
				QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(pos);
				face->fieldVec = out[face->GetIndexNo()];
			}
		}
 
		nodegra.resize(surfaceMesh->GetNodeNumber());
		for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != nullptr;) {
			QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);

			Eigen::Vector3d vecgra; vecgra.setZero();
			double allarea = 0;
			for (GLKPOSITION neighface = node->GetFaceList().GetHeadPosition(); neighface != nullptr;)
			{
				Eigen::Vector3d vecface;
				QMeshFace* face = (QMeshFace*)node->GetFaceList().GetNext(neighface);
				vecface = face->fieldVec;
				vecgra = vecgra + face->GetArea() * vecface;
				allarea += face->GetArea();
			}
			vecgra = vecgra / allarea;
			node->nodegra2d = vecgra;
			node->nodegra2d.normalize();
		}
	}

	surfaceMesh->drawgeoField = true;
	surfaceMesh->drawlayerheight = false;


	if (itecompflag) {
		Ite_ComputePathon3D();
		double pathvolume = compute_volume();
		itenum++;
		if (rotationflag) {
			while ((fabs(pathvolume - volumeoflayer) > ErrThreshold * volumeoflayer) && itenum < 5)
			{
				Toolpath->ClearAll();
				stripenode_order.clear();
				if (pathvolume < volumeoflayer) {
					lowerfre = fre;
					fre = 0.5 * (lowerfre + upperfre);
					Ite_ComputePathon3D();
					pathvolume = compute_volume();
				}
				else
				{
					upperfre = fre;
					fre = 0.5 * (lowerfre + upperfre);
					Ite_ComputePathon3D();
					pathvolume = compute_volume();
				}
				itenum++;
			}
		}
		else
		{
			while ((fabs(pathvolume - volumeoflayer) > ErrThreshold * volumeoflayer) && itenum < 5)
			{
				Toolpath->ClearAll();
				if (pathvolume < volumeoflayer) {
					uppercutvalue = cutvalue;
					cutvalue = 0.5 * (lowercutvalue + uppercutvalue);
					Ite_ComputePathon3D();
					pathvolume = compute_volume();

				}
				else
				{
					lowercutvalue = cutvalue;
					cutvalue = 0.5 * (lowercutvalue + uppercutvalue);
					Ite_ComputePathon3D();
					pathvolume = compute_volume();
				}
				itenum++;
			}
		}
	}
	else
	{
		Ite_ComputePathon3D();
	}

	for (int i = 0; i < nodelist.size(); i++)
	{
		for (int j = 0; j < nodelist[i].size(); j++) {
			delete nodelist[i][j];
		}
	}
	for (int i = 0; i < innernodelist.size(); i++)
	{
		for (int j = 0; j < innernodelist[i].size(); j++) {
			delete innernodelist[i][j];
		}
	}
	double pathvolume = compute_volume();
	std::cout << "index=" << surfaceMesh->GetIndexNo() << "pathvolume=" << pathvolume << "layervolume" << volumeoflayer << std::endl;
 }

 void DeformToolPath::Trimboundtoolpath(std::vector<std::vector<QMeshNode*>>& Boundnodelist, std::vector<bool>& IsboundClose, bool isoutboundary) {

	 IsboundClose.assign(Boundnodelist.size(), false);
	  //if (!isoutboundary)std::cout << Boundnodelist.size() << std::endl;

	 for (int pathindex = 0; pathindex < (int)Boundnodelist.size(); ++pathindex)
	 {
		 const int edgeNum = surfaceMesh->GetEdgeNumber();
		 std::vector<bool> cutedge(edgeNum, false);
		 int linesize1 = (int)Boundnodelist[pathindex].size();
	//	 if (!isoutboundary)std::cout <<"start erase" << linesize1 << std::endl;

 		 Boundnodelist[pathindex].erase(
			 std::remove_if(
				 Boundnodelist[pathindex].begin(),



				 Boundnodelist[pathindex].end(),
				 [&](QMeshNode* node) {

					 if (!node) return true;

					 bool toRemove = false;
					 if (isoutboundary)
					 {
					
						 toRemove = (node->DisToInnerBoundary <
							 cutvalue + std::min(innerboundarynum, 1) * initialadjust * averagewidth +
							 (innerboundarynum - 1) * averagewidth);
					 }
					else
					{
						toRemove = (node->DisToBoundary < cutvalue + std::min(outterboundarynum, 1) * initialadjust * averagewidth);
					}
					 if (toRemove) {
						 if (node->relatedEdgeIndex >= 0 && node->relatedEdgeIndex < edgeNum) {
							 QMeshEdge* edge = surfaceMesh->GetEdgeRecordPtr(node->relatedEdgeIndex + 1);
							 if (edge != nullptr) {
 								 int eid = edge->GetIndexNo();
								  cutedge[eid] = true;
							 }
						 }
						 delete node;
						 return true;
					 }
					 return false;
				 }),
			 Boundnodelist[pathindex].end()
		 );
		//if (!isoutboundary)std::cout << "bounnum=" << Boundnodelist[pathindex].size() << std::endl;
 		 int linesize2 = (int)Boundnodelist[pathindex].size();
		 bool becuted = (linesize2 != linesize1);
		 (void)becuted;

 		 if (linesize2 < 3) {
			 for (auto& ptr : Boundnodelist[pathindex]) {
				 delete ptr;
				 ptr = nullptr;
			 }
			 Boundnodelist.erase(Boundnodelist.begin() + pathindex);
			 IsboundClose.erase(IsboundClose.begin() + pathindex);
			 --pathindex;
			 continue;
		 }

 		 bool hasCutEdgeOnPath = false;
		 for (QMeshNode* node : Boundnodelist[pathindex]) {
			 if (!node) continue;
			 if (node->relatedEdgeIndex < 0 || node->relatedEdgeIndex >= edgeNum) continue;

			 QMeshEdge* edge = surfaceMesh->GetEdgeRecordPtr(node->relatedEdgeIndex + 1);
			 if (!edge) continue;

			 int eid = edge->GetIndexNo();
			 if (cutedge[eid]) { // 
				 hasCutEdgeOnPath = true;
				 break;
			 }
		 }

		 std::vector<int> startindex;
		 startindex.reserve(Boundnodelist[pathindex].size());

		 for (int i = 0; i < (int)Boundnodelist[pathindex].size(); ++i)
		 {
			 QMeshNode* ni = Boundnodelist[pathindex][i];
			 if (!ni) continue;
			 if (ni->relatedEdgeIndex < 0 || ni->relatedEdgeIndex >= edgeNum) continue;

			 QMeshEdge* edge = surfaceMesh->GetEdgeRecordPtr(ni->relatedEdgeIndex + 1);
			 if (!edge) continue;

			 QMeshFace* lface = edge->GetLeftFace();
			 QMeshFace* rface = edge->GetRightFace();
			 bool flage = false;

			 if (lface != NULL && rface != NULL) {

				 bool sflage = true;
				 for (int k = 1; k <= 3; k++)
				 {
					 QMeshEdge* ledge = lface->GetEdgeRecordPtr(k);
					 if ( ledge == edge) continue;
					 if (isoutboundary)
					 {
						 if (((ledge->GetStartPoint()->DisToBoundary - ni->DisToBoundary) *
							 (ledge->GetEndPoint()->DisToBoundary - ni->DisToBoundary)) < 0) {
							 sflage = false;
						 }
					 }
					 else
					 {
						 if (((ledge->GetStartPoint()->DisToInnerBoundary - ni->DisToInnerBoundary) *
							 (ledge->GetEndPoint()->DisToInnerBoundary - ni->DisToInnerBoundary)) < 0) {
							 sflage = false;
						 }
					 }

					 int id = ledge->GetIndexNo();
					 if (cutedge[id]) flage = true;
				 }
				 if (sflage) flage = true;

				 sflage = true;

				 for (int k = 1; k <= 3; k++)
				 {
					 QMeshEdge* redge = rface->GetEdgeRecordPtr(k);
					 if (!redge || redge == edge) continue;
					 if (isoutboundary)
					 {
						 if (((redge->GetStartPoint()->DisToBoundary - ni->DisToBoundary) *
							 (redge->GetEndPoint()->DisToBoundary - ni->DisToBoundary)) < 0) {
							 sflage = false;
						 }
					 }
					 else
					 {
						 if (((redge->GetStartPoint()->DisToInnerBoundary - ni->DisToInnerBoundary) *
							 (redge->GetEndPoint()->DisToInnerBoundary - ni->DisToInnerBoundary)) < 0) {
							 sflage = false;
						 }
					 }

					 int id = redge->GetIndexNo();
					 if (cutedge[id]) flage = true;
				 }
				 if (sflage) flage = true;
			 }

			 if (lface == NULL) flage = true;
			 if (rface == NULL) flage = true;

			 if (flage) startindex.push_back(i);
		 }

		 int start = 0;
		 if (!startindex.empty()) {
			 int minindex = startindex[0];
			 double mininnerdis = std::numeric_limits<double>::max();
			 for (int idx : startindex) {
				 if (isoutboundary)
				 {
					 if (Boundnodelist[pathindex][idx]->DisToInnerBoundary < mininnerdis) {
						 mininnerdis = Boundnodelist[pathindex][idx]->DisToInnerBoundary;
						 minindex = idx;
					 }
				 }
				 else
				 {
					 if (Boundnodelist[pathindex][idx]->DisToBoundary < mininnerdis) {
						 mininnerdis = Boundnodelist[pathindex][idx]->DisToBoundary;
						 minindex = idx;
					 }
				 }
			 }
			 start = minindex;
		 }

 		 std::set<int> visited;
		 std::vector<QMeshNode*> reordervec;
		 reordervec.reserve(Boundnodelist[pathindex].size());

		 visited.insert(start);
		 reordervec.push_back(Boundnodelist[pathindex][start]);

		 while (reordervec.size() < Boundnodelist[pathindex].size())
		 {
			 QMeshNode* currentnode = reordervec.back();
			 double minDistance = std::numeric_limits<double>::max();
			 int cloestIdx = -1;

			 for (int k = 0; k < (int)Boundnodelist[pathindex].size(); ++k) {
				 if (visited.find(k) != visited.end()) continue;
				 double dist = distance(currentnode, Boundnodelist[pathindex][k]);
				 if (dist < minDistance) {
					 minDistance = dist;
					 cloestIdx = k;
				 }
			 }

			 if (cloestIdx == -1) break;
			 reordervec.push_back(Boundnodelist[pathindex][cloestIdx]);
			 visited.insert(cloestIdx);
		 }

 		 std::vector<std::vector<QMeshNode*>> lines;
		 lines.reserve(15);

		 bool hasLargeJump = true;

		 std::vector<QMeshNode*> curline;
		 curline.reserve(reordervec.size());
		 curline.push_back(reordervec[0]);

		 auto flush_curline = [&](std::vector<QMeshNode*>& seg) {
			 if (seg.empty()) return;

			 if ((int)seg.size() < 3) {
				 for (QMeshNode* p : seg) delete p;
			 }
			 else {
				 lines.push_back(std::move(seg));
			 }
			 seg.clear();
		};

		 for (int i = 1; i < (int)reordervec.size(); ++i)
		 {
			 double dist = distance(reordervec[i - 1], reordervec[i]);
			 if (dist > largegap) {
				 hasLargeJump = true;
				 flush_curline(curline);          // filter short path
				 curline.push_back(reordervec[i]);
			 }
			 else {
				 curline.push_back(reordervec[i]);
			 }
		 }
		 flush_curline(curline);                  

 		 if (lines.empty()) {

			 Boundnodelist.erase(Boundnodelist.begin() + pathindex);
			 IsboundClose.erase(IsboundClose.begin() + pathindex);
			 --pathindex;
			 continue;
		 }


		 const bool isOpen = (hasCutEdgeOnPath || hasLargeJump);
		 const bool isClosed = !isOpen;

 		 if ((int)lines.size() == 1) {
			 IsboundClose[pathindex] = isClosed;

 			 Boundnodelist[pathindex] = std::move(lines[0]);
	 
		 }
		 else {
 			 Boundnodelist[pathindex] = std::move(lines[0]);
			 IsboundClose[pathindex] = false;

			 int insertPos = pathindex + 1;
			 for (int i = 1; i < (int)lines.size(); ++i) {
				 Boundnodelist.insert(Boundnodelist.begin() + insertPos, std::move(lines[i]));
				 IsboundClose.insert(IsboundClose.begin() + insertPos, false);
				 ++insertPos;
			 }

			 pathindex = insertPos - 1;
		 }
	 }
 }
 


 void DeformToolPath::Ite_ComputePathon3D() {

	 if (!deformstate) {
		 ConverttoDeforma();
		 deformstate = true;
	 }
	 if (rotationflag)
	 {//compute complex number field

		 surfaceMesh->drawgeoField = true;
		 surfaceMesh->drawlayerheight = false;
		 //compute scalar value on node
		 stripe_Pattern* spoperate = new stripe_Pattern(surfaceMesh, fre);
		 spoperate->require_paramaterization();
		 spoperate->require_Coordinates();
		 spoperate->require_Polylines(stripe_points_pos, stripe_points_edge_index, stripe_points_edge_orientation, edgepolylines_index, matching_information);
		 delete spoperate;
	 }
	 if (deformstate) {
		 ConverttoInitial();
		 deformstate = false;
	 }
	 if (itenum == 0) {
		 allboundnum = 0;
		 if (hasinnerbound && innerboundarynum != 0) {//extract the inner boundry
			 allboundnum = compute_allinner_boundary(innerboundarynum, innernodelist);
		 }
		 if (outterboundarynum != 0)//extract the outter boundary
			 computeoutterboundary(outterboundarynum, nodelist);
	 }

	 //std::cout << "nodelistsiez=" << nodelist[0].size() << std::endl;
	 //std::cout << "innernodelistsiez=" << innernodelist[0].size() << std::endl;
	 if (!rotationflag)
	 {
		 tempinnernodelist.clear();
		 tempnodelist.clear();
		 for (int i = 0; i < innernodelist.size(); i++)
		 {
			 std::vector<QMeshNode*>tempvec;
			 tempvec.reserve(innernodelist[i].size());
			 for (int j = 0; j < innernodelist[i].size(); j++) {
				 QMeshNode* tempnode = new QMeshNode(innernodelist[i][j]);
				 tempvec.push_back(tempnode);
			 }
			 tempinnernodelist.push_back(tempvec);
		 }

		 for (int i = 0; i < nodelist.size(); i++)
		 {
			 std::vector<QMeshNode*>tempvec;
			 tempvec.reserve(nodelist[i].size());
			 for (int j = 0; j < nodelist[i].size(); j++) {
				 QMeshNode* tempnode = new QMeshNode(nodelist[i][j]);
				 tempvec.push_back(tempnode);
			 }
			 tempnodelist.push_back(tempvec);
		 }

		 Trimboundtoolpath(tempnodelist, outterboundaryclose, true);
		 if (innerboundarynum != 0) {
			 Trimboundtoolpath(tempinnernodelist, innerboundaryclose, false);
		 }
	 }
	 else if (rotationflag)
	 {
		 if (itenum == 0) {
			 Trimboundtoolpath(nodelist, outterboundaryclose, true);
			 if (innerboundarynum != 0) {
				 Trimboundtoolpath(innernodelist, innerboundaryclose, false);
			 }
		 }
		 tempinnernodelist.clear();
		 tempnodelist.clear();
		 for (int i = 0; i < innernodelist.size(); i++)
		 {
			 std::vector<QMeshNode*>tempvec;
			 tempvec.reserve(innernodelist[i].size());
			 for (int j = 0; j < innernodelist[i].size(); j++) {
				 QMeshNode* tempnode = new QMeshNode(innernodelist[i][j]);
				 tempvec.push_back(tempnode);
			 }
			 tempinnernodelist.push_back(tempvec);
		 }

		 for (int i = 0; i < nodelist.size(); i++)
		 {
			 std::vector<QMeshNode*>tempvec;
			 tempvec.reserve(nodelist[i].size());
			 for (int j = 0; j < nodelist[i].size(); j++) {
				 QMeshNode* tempnode = new QMeshNode(nodelist[i][j]);
				 tempvec.push_back(tempnode);
			 }
			 tempnodelist.push_back(tempvec);
		 }
	 }
	/*compute the zigzag*/
 	for (int i = 0; i < stripe_points_pos.size(); i++)
	{
		QMeshEdge* edge = surfaceMesh->GetEdgeRecordPtr(stripe_points_edge_index[i] + 1);
		Eigen::Vector3d  nodepos,nodes,nodee,nor_e,nors,norofnode;
		double dis1, dis2,distoin1,distoin2,dis,distoin,hofnode1,hofnode2,hofnode;
		if (stripe_points_edge_orientation[i] > 0) {
			edge->GetStartPoint()->GetCoord3D(nodes);
			edge->GetEndPoint()->GetCoord3D(nodee);
			dis1 = edge->GetStartPoint()->DisToBoundary;
			distoin1 = edge->GetStartPoint()->DisToInnerBoundary;
			dis2 = edge->GetEndPoint()->DisToBoundary;
			distoin2 = edge->GetEndPoint()->DisToInnerBoundary;
			hofnode1 = edge->GetStartPoint()->layerheight;
			hofnode2 = edge->GetEndPoint()->layerheight;
			edge->GetStartPoint()->GetNormal(nors[0], nors[1], nors[2]);
			edge->GetEndPoint()->GetNormal(nor_e[0], nor_e[1], nor_e[2]);
		}
		else
		{
			edge->GetEndPoint()->GetCoord3D(nodes);
			edge->GetStartPoint()->GetCoord3D(nodee);
			dis1 = edge->GetEndPoint()->DisToBoundary;
			distoin1 = edge->GetEndPoint()->DisToInnerBoundary;
			dis2 = edge->GetStartPoint()->DisToBoundary;
			distoin2 = edge->GetStartPoint()->DisToInnerBoundary;
			hofnode1 = edge->GetEndPoint()->layerheight;
			hofnode2 = edge->GetStartPoint()->layerheight;
			edge->GetStartPoint()->GetNormal(nor_e[0], nor_e[1], nor_e[2]);
			edge->GetEndPoint()->GetNormal(nors[0], nors[1], nors[2]);
		}

		nodepos = (stripe_points_pos[i] * nodes + (1 - stripe_points_pos[i]) * nodee);
		dis= stripe_points_pos[i] * dis1 + (1 - stripe_points_pos[i]) * dis2;
		distoin = stripe_points_pos[i] * distoin1 + (1 - stripe_points_pos[i]) * distoin2;
		hofnode= stripe_points_pos[i] * hofnode1 + (1 - stripe_points_pos[i]) * hofnode2;

		for (int k = 0; k < 3; k++)
		{
			norofnode[k] = stripe_points_pos[i] * nors[k] + (1 - stripe_points_pos[i]) * nor_e[k];
		}


		QMeshNode* isonode = new QMeshNode;
		isonode->SetCoord3D(nodepos);
		isonode->DisToBoundary = dis;
		isonode->DisToInnerBoundary = distoin;
		isonode->relatedEdgeIndex = edge->GetIndexNo();
		isonode->SetNormal(norofnode[0], norofnode[1], norofnode[2]);
		isonode->layerheight = hofnode;
	//	isonode->SetIndexNo(i);
		Toolpath->GetNodeList().AddTail(isonode);
	}

	for (int i = 0; i < matching_information.size(); i++)
	{
		for (int j = 0; j < matching_information[i].size(); j++) {
			std::array<int, 2> matchinfo = matching_information[i][j];
			QMeshNode* snode = Toolpath->GetNodeRecordPtr(matchinfo[0] + 1);
			QMeshNode* enode = Toolpath->GetNodeRecordPtr(matchinfo[1] + 1);
			double dis = (snode->DisToBoundary + enode->DisToBoundary) / 2.0;
			double distoin= (snode->DisToInnerBoundary + enode->DisToInnerBoundary) / 2.0;
 			QMeshEdge* edge = new QMeshEdge;
			edge->SetStartPoint(snode);
			edge->SetEndPoint(enode);
			snode->GetEdgeList().AddTail(edge);
			enode->GetEdgeList().AddTail(edge);
			Toolpath->GetEdgeList().AddTail(edge);
		}
	}

	for (GLKPOSITION nodepos = Toolpath->GetNodeList().GetHeadPosition(); nodepos != nullptr;) {
		QMeshNode* node = (QMeshNode*)Toolpath->GetNodeList().GetNext(nodepos);
		if (node->GetEdgeNumber() == 0) {
			if (nodepos == nullptr) {
				Toolpath->GetNodeList().RemoveTail();
			}
			else
			{
				Toolpath->GetNodeList().RemoveAt(nodepos->prev);
			}
  		}
 	}	
	
 
	int i = 0;
	for (GLKPOSITION nodepos = Toolpath->GetNodeList().GetHeadPosition(); nodepos != nullptr;) {
		QMeshNode* node = (QMeshNode*)Toolpath->GetNodeList().GetNext(nodepos);
		node->SetIndexNo(i);
		i++;
	}

	QMeshNode* striplastnode = NULL;
	std::vector<QMeshNode*>nodevec;//

	if (fre != 0&&StripFirst) {
		rerderednodelist.clear();
 		reorderthestripeline();
  		buildstriptoolpath(lastnode);
		striplastnode = stripenode_order[stripenode_order.size() - 3];
		if (outterboundarynum != 0) connectboundaryedge(true, true, outterboundarynum, striplastnode, nodevec, tempnodelist);
		if (nodevec.size() > 2)
		striplastnode = nodevec[nodevec.size() - 2];
		else
		{
			striplastnode = NULL;
		}
		//if (thecomputenum != 0) std::cout << "here" << std::endl;
		Toolpath->GetNodeList().RemoveAllWithoutFreeMemory();

	}
	else if(fre != 0 &&!StripFirst)
	{
		if (outterboundarynum != 0) connectboundaryedge(false, true, outterboundarynum, lastnode, nodevec, tempnodelist);
		reorderthestripeline();

		striplastnode = buildstriptoolpath(nodevec[nodevec.size() - 2]);

		Toolpath->GetNodeList().RemoveAllWithoutFreeMemory();

	}
	else
	{
		if (outterboundarynum != 0) connectboundaryedge(false, true, outterboundarynum, lastnode, nodevec, tempnodelist);
		if(nodevec.size()>2)
		striplastnode = nodevec[nodevec.size() - 2];
		else
		{
			striplastnode = NULL;
		}
	}
	//reinstall all the node;
	if (lastnode != NULL) {
		QMeshNode* lanode = new QMeshNode;
		Eigen::Vector3d pos,nor;
		lastnode->GetCoord3D(pos);
		lastnode->GetNormal(nor(0), nor(1), nor(2));
		lanode->SetCoord3D(pos);
		lanode->SetNormal(nor(0), nor(1), nor(2));
		Toolpath->GetNodeList().AddTail(lanode);
	}

	if (StripFirst) {
		/*stripe*/
		for (int i = 0; i < stripenode_order.size(); i++)
		{
			Toolpath->GetNodeList().AddTail(stripenode_order[i]);
			stripenode_order[i]->SetIndexNo(i);
		}
		/*outboundary*/
		for (int i = 0; i < nodevec.size(); i++)
		{
			Toolpath->GetNodeList().AddTail(nodevec[i]);
			nodevec[i]->SetIndexNo(Toolpath->GetNodeNumber());
		}
	}
	else
	{
		/*boutboundary*/
		for (int i = 0; i < nodevec.size(); i++)
		{
 			Toolpath->GetNodeList().AddTail(nodevec[i]);
			nodevec[i]->SetIndexNo(i);
		}
		/*stripe*/
		for (int i = 0; i < stripenode_order.size(); i++)
		{
			Toolpath->GetNodeList().AddTail(stripenode_order[i]);
			stripenode_order[i]->SetIndexNo(Toolpath->GetNodeNumber());
		}
	}
	std::cout << "itenum=" << itenum << std::endl;

	if (hasinnerbound&&innerboundarynum!=0) {
		std::vector<QMeshNode*>innernodevec;
 		connectboundaryedge(true,false,allboundnum,striplastnode, innernodevec, tempinnernodelist);
		/*innerboundary*/
		for (int i = 0; i < innernodevec.size(); i++)
		{
			if (i == 0)innernodevec[i]->noextrusionnode = true;

			Toolpath->GetNodeList().AddTail(innernodevec[i]);
			innernodevec[i]->SetIndexNo(Toolpath->GetNodeNumber());
		}
	}
	/*add all edge*/
	Toolpath->GetEdgeList().RemoveAll();
	for (int i = 1; i <= Toolpath->GetNodeNumber(); i++)
	{
		if (i != Toolpath->GetNodeNumber()) {
			QMeshNode* snode = Toolpath->GetNodeRecordPtr(i);
			QMeshNode* enode = Toolpath->GetNodeRecordPtr(i + 1);
			QMeshEdge* nedge = new QMeshEdge;
			nedge->SetStartPoint(snode);
			nedge->SetEndPoint(enode);
			Toolpath->GetEdgeList().AddTail(nedge);
		}
	}
 	/*resample all the edge*/
	resamplingSinglePatch(Toolpath);
 }


void DeformToolPath::smoothtoolpath(QMeshPatch* patch, int windowsize) {
 
	std::vector<QMeshNode*>nodevec;
	//nodevec.push_back((QMeshNode*)patch->GetNodeList().GetHead());
	int i = 0;
	for (GLKPOSITION edgepos = patch->GetEdgeList().GetHeadPosition(); edgepos != nullptr;) {
		QMeshEdge* edge = (QMeshEdge*)patch->GetEdgeList().GetNext(edgepos);

		nodevec.push_back(edge->GetEndPoint());
	 
 	}
	int n = nodevec.size();
	for (int i = 0; i < n; i++)
	{
		Eigen::Vector3d sumpos;
		Eigen::Vector3d sumnor;
		sumpos.setZero();
		sumnor.setZero();
		int count = 0;
		for (int j = std::max(0, i - windowsize); j <= std::min(n -1,i+ windowsize); ++j) {
			Eigen::Vector3d pos3d,nor3d;
			nodevec[j]->GetCoord3D(pos3d);
			nodevec[j]->GetNormal(nor3d(0), nor3d(1), nor3d(2));
			sumpos += pos3d;
			sumnor += nor3d;
			count++;
		}
		Eigen::Vector3d posorigin;
	//	nodevec[i]->GetCoord3D(posorigin);
		//if ((posorigin - sumpos / (double)count).norm() > 1.5) std::cout << "smooth>1.5" << std::endl;
	//	nodevec[i]->SetCoord3D(sumpos(0) / count, sumpos(1) / count, sumpos(2) / count);
		Eigen::Vector3d smoothvec = { sumnor(0) /(double) count, sumnor(1) / (double)  count, sumnor(2) / (double)count };
		smoothvec.normalize();
		nodevec[i]->SetNormal(smoothvec(0), smoothvec(1), smoothvec(2));
	}
}

std::vector<QMeshNode*>DeformToolPath::a_star(QMeshNode* startnode, QMeshNode* endnode) {

	std::vector<QMeshNode*>path;
	if (startnode->relatedEdgeIndex == -1 || endnode->relatedEdgeIndex == -1) std::cout << "serrrrrrrrrrrrrrrrrrr" << std::endl;
	if (startnode->relatedEdgeIndex == endnode->relatedEdgeIndex) 		return std::vector<QMeshNode*>();
	
	/*find start node on mesh*/
	QMeshEdge* sedge = surfaceMesh->GetEdgeRecordPtr(startnode->relatedEdgeIndex + 1);
	QMeshNode* snode = sedge->GetStartPoint();
	QMeshNode* enode = sedge->GetEndPoint();
	QMeshNode* ssnode = NULL;
	QMeshNode* senode = NULL;
	Eigen::Vector3d posstart, posend, poss, pose,vecs,vece,vect;
	startnode->GetCoord3D(posstart);

	endnode->GetCoord3D(posend);
	snode->GetCoord3D(poss);
	enode->GetCoord3D(pose);
	vecs = poss - posstart;
	vece = pose - posstart;
	vect = pose - poss;
	if (vecs.dot(vect) > vece.dot(vect)) {
		ssnode = snode;
	}
	else
	{
		ssnode = enode;
	}

	QMeshNode* endnode_endnode = surfaceMesh->GetEdgeRecordPtr(endnode->relatedEdgeIndex + 1)->GetEndPoint();
	QMeshNode* endnode_startnode = surfaceMesh->GetEdgeRecordPtr(endnode->relatedEdgeIndex + 1)->GetStartPoint();

	/*findpath*/
	std::vector<double>dist(surfaceMesh->GetNodeNumber(), std::numeric_limits<double>::max());
	std::vector<QMeshNode*>prev(surfaceMesh->GetNodeNumber(),NULL);
	
	dist[ssnode->GetIndexNo()] = 0;
	using Pair = std::pair<double, QMeshNode*>;
	std::priority_queue<Pair, std::vector<Pair>, std::greater<>>pq;
	pq.push({ 0,ssnode });
	while (!pq.empty())
	{
		QMeshNode* u = pq.top().second;
		pq.pop();
		if (u == endnode_endnode) { 
			senode = endnode_endnode;
			break; }
		if (u == endnode_startnode) {
			senode = endnode_startnode;
			break;
		}
		for (GLKPOSITION edgepos = u->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
		{
			QMeshEdge* edge = (QMeshEdge*)u->GetEdgeList().GetNext(edgepos);
			QMeshNode* neighbornode = NULL;
			if (edge->GetStartPoint() == u)neighbornode = edge->GetEndPoint();
			else neighbornode = edge->GetStartPoint();
			double alt = dist[u->GetIndexNo()] +edge->CalLength();
			if (alt < dist[neighbornode->GetIndexNo()]) {
				dist[neighbornode->GetIndexNo()] = alt;
				prev[neighbornode->GetIndexNo()] = u;
				pq.push({ alt+distance(neighbornode,endnode),neighbornode});
 			}
		}

	}
 	for (int at = senode->GetIndexNo(); prev[at]!=NULL; at= prev[at]->GetIndexNo())
	{
		if (at >= surfaceMesh->GetNodeNumber())std::cout << "nodeindexerr" << std::endl;
		Eigen::Vector3d vec3,norvec3;
	    surfaceMesh->GetNodeRecordPtr(at + 1)->GetCoord3D(vec3);
		surfaceMesh->GetNodeRecordPtr(at + 1)->GetNormal(norvec3(0), norvec3(1), norvec3(2));

		QMeshNode* node = new QMeshNode;
		node->isconnectnode = true;
		node->SetCoord3D(vec3);
		node->SetNormal(norvec3(0), norvec3(1), norvec3(2));
		path.push_back(node);
	}
	std::reverse(path.begin(), path.end());

	for (int i = 0; i < path.size(); i++)
	{
		path[i]->noextrusionnode = true;
	}

	if (path.size() > 3)
	{
		for (int iternum = 0; iternum < 3; iternum++)
		{
			int windowsize = 1;

			int n = path.size();
			for (int i = 0; i < n; i++)
			{
				Eigen::Vector3d sumpos;
				Eigen::Vector3d sumnor;
				sumpos.setZero();
				sumnor.setZero();
				int count = 0;
				for (int j = std::max(0, i - windowsize); j <= std::min(n - 1, i + windowsize); ++j) {
					Eigen::Vector3d pos3d, nor3d;
					path[j]->GetCoord3D(pos3d);
					path[j]->GetNormal(nor3d(0), nor3d(1), nor3d(2));
					sumpos += pos3d;
					sumnor += nor3d;
					count++;
				}
				Eigen::Vector3d posorigin;
  				path[i]->SetCoord3D(sumpos(0) / count, sumpos(1) / count, sumpos(2) / count);
				path[i]->SetNormal(sumnor(0) / count, sumnor(1) / count, sumnor(2) / count);
			}
		}
	}

	return path;
}



void DeformToolPath::connectboundaryedge(bool A_statflag,bool outterboundary,int boundnum, QMeshNode* lnode , 
	std::vector<QMeshNode*>& boundnodevec, std::vector<std::vector<QMeshNode*>>& vec){

	if (vec.size() == 0) {
		return;
	}
	/*reorder all the bounarynode*/
	int startnodeindex = 0;
	double mindis = INFINITY;
	//std::cout << "vecsize=" << vec.size() << std::endl;
	if (lnode != NULL&&((outterboundary)||((!outterboundary && innerboundaryclose[0])))){

		for (int i = 0; i < vec[0].size(); i++)
		{
			QMeshNode* node = vec[0][i];
			Eigen::Vector3d pos1, pos2;
			node->GetCoord3D(pos1);
			lnode->GetCoord3D(pos2);
			if ((pos1 - pos2).norm() < mindis) {
				mindis = (pos1 - pos2).norm();
				startnodeindex = i;
			}
		}
		std::vector<QMeshNode*>subvec;
		for (int i = startnodeindex; i < vec[0].size(); i++)
		{
			subvec.push_back(vec[0][i]);
		}
		for (int i = 0; i < startnodeindex; i++)
		{
			subvec.push_back(vec[0][i]);
		}
		vec[0] = subvec;
		if ( A_statflag)
		{
			std::vector<QMeshNode*>A_starpath = a_star(lnode, subvec[0]);
			for (int i = 0; i < A_starpath.size(); i++)
			{
				boundnodevec.push_back(A_starpath[i]);
			}
		}
	}
	else if (lnode != NULL && (!outterboundary && !innerboundaryclose[0])) {
		if (A_statflag)
		{
			Eigen::Vector3d pos1, pos2,pos3;
			lnode->GetCoord3D(pos1);
			vec[0][0]->GetCoord3D(pos2);
			vec[0][vec[0].size() - 1]->GetCoord3D(pos3);
			if ((pos1 - pos2).norm() > (pos1 - pos3).norm()) {
				std::reverse(vec[0].begin(), vec[0].end());
			}
			std::vector<QMeshNode*>A_starpath = a_star(lnode, vec[0][0]);
			for (int i = 0; i < A_starpath.size(); i++)
			{
				boundnodevec.push_back(A_starpath[i]);
			}
		}
	}
	if (vec.size() != 1) {
		for (int i = 1; i < vec.size(); i++)
		{
			if(outterboundary)
			if ((!outterboundaryclose[i])) continue;
			if(!outterboundary)
			if ((!innerboundaryclose[i])) continue;
			mindis = std::numeric_limits<double>::max();
			for (int j = 0; j < vec[i].size(); j++)
			{
				QMeshNode* node = vec[i][j];
				Eigen::Vector3d pos1, pos2;
				node->GetCoord3D(pos1);
				vec[i - 1][vec[i - 1].size() - 1]->GetCoord3D(pos2);
				if ((pos1 - pos2).norm() < mindis) {
					mindis = (pos1 - pos2).norm();
					startnodeindex = j;
				}
			}
			
 			std::vector<QMeshNode*>subvec;
			for (int k = startnodeindex; k < vec[i].size(); k++)
			{
				subvec.push_back(vec[i][k]);
			}
			for (int k = 0; k < startnodeindex; k++)
			{
				subvec.push_back(vec[i][k]);
			}
			vec[i] = subvec;
		}
	}
		//QMeshNode* snode = lnode;
		for (int i = 0; i < vec.size(); i++)
		{
			std::vector<QMeshNode*> avec;
			if(outterboundary)
			if ((!outterboundaryclose[i])) {
				if(i!=0) 
				avec = a_star(vec[i - 1][vec[i - 1].size() - 2], vec[i][0]);
 			}

			if(!outterboundary)
			if ((!innerboundaryclose[i])) {
				if (i != 0)
					avec = a_star(vec[i - 1][vec[i - 1].size() - 2], vec[i][0]);
			}
				QMeshNode* node1 = vec[i][0];
				QMeshNode* node2 = vec[i][1];
				QMeshNode* nodee1 = vec[i][vec[i].size() - 1];
				QMeshNode* nodee2 = vec[i][vec[i].size() - 2];

				Eigen::Vector3d ppos1, ppos2, ppos3, ppos4, ppos5, ppos6,nor1,nor2,nor3,nor4;

				node1->GetCoord3D(ppos1);
				node1->GetNormal(nor1(0), nor1(1), nor1(2));
				node2->GetCoord3D(ppos2);
				nodee1->GetCoord3D(ppos3);
				nodee1->GetNormal(nor3(0), nor3(1), nor3(2));
				nodee2->GetCoord3D(ppos4);
 
 				ppos5 = ppos1 + (ppos1 - ppos2).normalized() * insertlength;
				ppos6 = ppos3 + (ppos3 - ppos4).normalized() * insertlength;

				QMeshNode* nodes = new QMeshNode;
				QMeshNode* nodee = new QMeshNode;
				nodes->SetCoord3D(ppos5);
				nodes->SetNormal(nor1(0), nor1(1), nor1(2));
				nodes->layerheight = node1->layerheight;
				nodes->printvelocity = node1->printvelocity;
				nodee->SetCoord3D(ppos6);
				nodee->SetNormal(nor3(0), nor3(1), nor3(2));
				nodee->layerheight = nodee1->layerheight;
				nodee->printvelocity = nodee1->printvelocity;

				nodee->terminalnode = true;
				nodes->terminalnode =true;
				node1->terminalnodemovp = true;
				nodee1->terminalnodemovp = true;

				vec[i].insert(vec[i].begin(), nodes);
				vec[i].push_back(nodee);
				if (avec.size() != 0) {
					vec[i].insert(vec[i].begin(), avec.begin(), avec.end());
				}
  			for (int j = 0; j < vec[i].size(); j++) {
				boundnodevec.push_back(vec[i][j]);
			}
 		}
	
}



void DeformToolPath::computeoutterboundary(int buondnum, std::vector<std::vector<QMeshNode*>>& nodelist) {

	std::vector<std::vector<QMeshNode*>> reorderednode;

	if (buondnum == 0) return;

	if (fre == 0) {
		int count = 0;
		while (true)
		{
			double isovalue = initialadjust*averagewidth + (double)count * boundratio * averagewidth;
			std::vector<QMeshNode*>newnewline;
			int prelinenum = reorderednode.size();
			computesignalboundtoolpath(newnewline, isovalue, reorderednode);
			if (newnewline.size() == 0 || prelinenum == reorderednode.size()) {
				break;
			}
			count++;
		}
		outterboundarynum = count;
	}
	else
	{

		for (int i = 0; i < buondnum; i++)
		{
			//std::cout << "compute outter" << std::endl;
			std::vector<QMeshNode*>newnewline;
			double isovalue = initialadjust * averagewidth + (double)i * boundratio * averagewidth;
			int prelinenum = reorderednode.size();
			computesignalboundtoolpath(newnewline, isovalue, reorderednode);
			if (newnewline.size() == 0 || prelinenum == reorderednode.size())break;
		}

	}
	outterboundarynum = reorderednode.size();
	nodelist = reorderednode;
}
void DeformToolPath::updatedistooutter(std::vector<Eigen::Vector3d>poslist,std::vector<double>heightlist) {

 
	for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
	{
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
		Eigen::Vector3d posnode, posnode2;
		node->GetCoord3D(posnode);
		double mindis = INFINITE;
		int nodeindex = 0;
		for (int i = 0; i < poslist.size(); i++)
		{
			posnode2 = poslist[i];
			double dis = (posnode - posnode2).norm();
			if (mindis > dis) {
				mindis = dis;
				nodeindex = i;
			}
		}
 		 double width = boundratio*getarea(heightlist[nodeindex]) / (heightlist[nodeindex]);


		if (node->DisToBoundary < 0) {
			node->DisToBoundary = -mindis - width;
		}
		else
		{
			node->DisToBoundary = mindis - width;
		}
	}
}

int DeformToolPath::compute_allinner_boundary(int buondnum,std::vector<std::vector<QMeshNode*>>&nodelist) {
	
	if (buondnum == 0) return 0;

 	innerboundaryclose.clear();
	std::vector<bool>().swap(innerboundaryclose);

 	for (int i = 0; i < buondnum; i++)
	{
		std::vector<QMeshNode*>newline;
		double isovalue = initialadjust * averagewidth + boundratio * (double)i * averagewidth;
		compute_single_innertoolpath(newline, isovalue, nodelist);
 	}
  std::cout << "nodelistnum======" << nodelist[0].size() << std::endl;
 	return buondnum;
}


void DeformToolPath::compute_single_innertoolpath(std::vector<QMeshNode*>& newline, double isovalue, std::vector<std::vector<QMeshNode*>>& nodelist) {

	std::vector<bool>edgevisit(surfaceMesh->GetEdgeNumber(), false);

	for (int j = 0; j < edgevisit.size(); j++) {
		edgevisit[j] = false;
	}
	while (true)//extract isoline for all inner boundary
	{
		/*construct outterboundary*/
		QMeshEdge* startedge = NULL;
		QMeshEdge* Priorstartedge = NULL;
		for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {//find the start edge
			QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
			if (edgevisit[Edge->GetIndexNo()])continue;
			double boutvalue_s = Edge->GetStartPoint()->DisToInnerBoundary;
			double boutvalue_e = Edge->GetEndPoint()->DisToInnerBoundary;

			if ((boutvalue_e - isovalue) * (boutvalue_s - isovalue) < 0) {
				startedge = Edge;
				if (Edge->GetLeftFace() == NULL || Edge->GetRightFace() == NULL) {
					Priorstartedge = Edge; break;
				}
			}
		}

		if (Priorstartedge != NULL) startedge = Priorstartedge;
		if (startedge == NULL) break;
		newline.clear();

		double boutvalue_s = startedge->GetStartPoint()->DisToInnerBoundary;
		double boutvalue_e = startedge->GetEndPoint()->DisToInnerBoundary;

		double alpha = (isovalue - boutvalue_s) / (boutvalue_e - boutvalue_s);

		QMeshNode* isonode = new QMeshNode;

		double p1[3], p2[3], p3[3], pn[3], pn1[3], pn2[3];
		startedge->GetStartPoint()->GetCoord3D(p1);
		startedge->GetEndPoint()->GetCoord3D(p2);
		startedge->GetStartPoint()->GetNormal(pn1[0], pn1[1], pn1[2]);
		startedge->GetEndPoint()->GetNormal(pn2[0], pn2[1], pn2[2]);

		//compute the parameters for this isonode
		for (int j = 0; j < 3; j++) {
			p3[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];
			pn[j] = (1.0 - alpha) * pn1[j] + alpha * pn2[j];
		}
		isonode->SetCoord3D(p3[0], p3[1], p3[2]);
		isonode->SetNormal(pn[0], pn[1], pn[2]);
		isonode->layerheight = (1.0 - alpha) * startedge->GetStartPoint()->layerheight + alpha * startedge->GetEndPoint()->layerheight;
		isonode->relatedEdgeIndex = startedge->GetIndexNo();
		isonode->DisToBoundary = (1.0 - alpha) * startedge->GetStartPoint()->DisToBoundary + alpha * startedge->GetEndPoint()->DisToBoundary;
		isonode->DisToInnerBoundary = isovalue;
		//install
		newline.push_back(isonode);
		//mark the edge
		edgevisit.at(startedge->GetIndexNo()) = true;

		//find the first face
		QMeshFace* nextface = NULL;
		nextface = startedge->GetLeftFace();
		if (startedge->GetLeftFace() == NULL) nextface = startedge->GetRightFace();

		//find all isonodes along the inner loop
		while (true)
		{
			bool findnextedge = false;
			for (int k = 0; k < 3; k++)
			{
				QMeshEdge* edge = nextface->GetEdgeRecordPtr(k + 1);
				if (edgevisit.at(edge->GetIndexNo()))continue;

				double boutvalue_s1 = edge->GetStartPoint()->DisToInnerBoundary;
				double boutvalue_e1 = edge->GetEndPoint()->DisToInnerBoundary;

				QMeshNode* snode = edge->GetStartPoint();
				QMeshNode* enode = edge->GetEndPoint();
				if ((boutvalue_s1 - isovalue) * (boutvalue_e1 - isovalue) > 0) continue;

				startedge = edge;
				findnextedge = true;
			}

			if (findnextedge == false) break;

			edgevisit.at(startedge->GetIndexNo()) = true;

			QMeshNode* newisonode = new QMeshNode;

			double boutvalue_ss = startedge->GetStartPoint()->DisToInnerBoundary;//startedge->GetStartPoint()->DisToBoundary;
			double boutvalue_ee = startedge->GetEndPoint()->DisToInnerBoundary;

			double alpha1 = (isovalue - boutvalue_ss) / (boutvalue_ee - boutvalue_ss);

			double pp1[3], pp2[3], pp3[3], pn[3], pn1[3], pn2[3];
			startedge->GetStartPoint()->GetCoord3D(pp1);
			startedge->GetEndPoint()->GetCoord3D(pp2);
			startedge->GetStartPoint()->GetNormal(pn1[0], pn1[1], pn1[2]);
			startedge->GetEndPoint()->GetNormal(pn2[0], pn2[1], pn2[2]);

			for (int j = 0; j < 3; j++) {
				//compute the position for this isonode
				pp3[j] = (1.0 - alpha1) * pp1[j] + alpha1 * pp2[j];
				pn[j] = (1.0 - alpha) * pn1[j] + alpha * pn2[j];
			}

			newisonode->layerheight = (1.0 - alpha1) * startedge->GetStartPoint()->layerheight + alpha1 * startedge->GetEndPoint()->layerheight;
			newisonode->SetCoord3D(pp3[0], pp3[1], pp3[2]);
			newisonode->SetNormal(pn[0], pn[1], pn[2]);
			newisonode->relatedEdgeIndex = startedge->GetIndexNo();
			newisonode->DisToBoundary = (1.0 - alpha1) * startedge->GetStartPoint()->DisToBoundary + alpha1 * startedge->GetEndPoint()->DisToBoundary;
			newisonode->DisToInnerBoundary = isovalue;
			if (startedge->GetLeftFace() == nextface) nextface = startedge->GetRightFace();
			else nextface = startedge->GetLeftFace();

			newline.push_back(newisonode);

			if (nextface == NULL) break;
		}
		if(newline.size()!=0)
		nodelist.push_back(newline);
	}
}

void DeformToolPath::computesignalboundtoolpath(std::vector<QMeshNode*>& newline, double isovalue,std::vector<std::vector<QMeshNode*>> &reorderednode) {

	std::vector<bool>edgevisit;
	edgevisit.resize (surfaceMesh->GetEdgeNumber(),false);
 	while (true)
	{
		QMeshEdge* startedge = NULL;
		bool findnextedge = false;
		for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {

			QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
 			if (edgevisit.at(Edge->GetIndexNo()))continue;

			double boutvalue_s1 = Edge->GetStartPoint()->DisToBoundary;
			double boutvalue_e1 = Edge->GetEndPoint()->DisToBoundary;

			if ((boutvalue_s1 - isovalue) * (boutvalue_e1 - isovalue) > 0) continue;

			startedge = Edge;
			findnextedge = true;
			break;
		}

		if (findnextedge == false) break;

		edgevisit.at(startedge->GetIndexNo()) = true;

		QMeshNode* newisonode = new QMeshNode;

		double boutvalue_ss = startedge->GetStartPoint()->DisToBoundary;
		double boutvalue_ee = startedge->GetEndPoint()->DisToBoundary;
		double distoinner_s = startedge->GetStartPoint()->DisToInnerBoundary;
		double distoinner_e = startedge->GetEndPoint()->DisToInnerBoundary;
 

		double alpha1 = (isovalue - boutvalue_ss) / (boutvalue_ee - boutvalue_ss);

		double pp1[3], pp2[3], pp3[3], distoinner,pn1[3],pn2[3],pn[3];

		startedge->GetStartPoint()->GetCoord3D(pp1);
		startedge->GetEndPoint()->GetCoord3D(pp2);
		startedge->GetStartPoint()->GetNormal(pn1[0],pn1[1],pn1[2]);
		startedge->GetEndPoint()->GetNormal(pn2[0], pn2[1], pn2[2]);

		distoinner = (1.0 - alpha1) * distoinner_s + alpha1 * distoinner_e;
		for (int j = 0; j < 3; j++) {
			//compute the position for this isonode
			pp3[j] = (1.0 - alpha1) * pp1[j] + alpha1 * pp2[j];
			pn[j]= (1.0 - alpha1) * pn1[j] + alpha1 * pn2[j];
  		}

		newisonode->layerheight = (1.0 - alpha1) * startedge->GetStartPoint()->layerheight + alpha1 * startedge->GetEndPoint()->layerheight;
		newisonode->SetCoord3D(pp3[0], pp3[1], pp3[2]);
		newisonode->DisToInnerBoundary = distoinner;
		newisonode->relatedEdgeIndex = startedge->GetIndexNo();
		newisonode->DisToBoundary = isovalue;
		newisonode->SetNormal(pn[0], pn[1], pn[2]);
		//std::cout << "pn=" << pn[0] << "," << pn[1] << "," << pn[2] << std::endl;
 		//std::cout << "newisonode_DisToInnerBOundary" << newisonode->DisToInnerBOundary << std::endl;
		newline.push_back(newisonode);
	}

	if (newline.size() < 4) {
		newline.clear();
		return;
	}
	reorderednode.push_back(newline);
}


void DeformToolPath::reorderthestripeline() {
	stripcutvalue = initialadjust * averagewidth;
 	std::vector<bool>nodevisit(Toolpath->GetNodeNumber(),false);
//	std::cout << "toolpathnodenum=" << Toolpath->GetNodeNumber() << std::endl;
	while (true)
	{
		QMeshNode* snode = NULL;
		std::vector<int> substripeline;
		for (GLKPOSITION nodepos = Toolpath->GetNodeList().GetHeadPosition(); nodepos != nullptr;) {
			QMeshNode* node = (QMeshNode*)Toolpath->GetNodeList().GetNext(nodepos);
			if (nodevisit[node->GetIndexNo()])continue;
			if (node->GetEdgeNumber() == 1) {
				snode = node; break;
			}
		}

		if (snode == NULL) {

			for (GLKPOSITION nodepos = Toolpath->GetNodeList().GetHeadPosition(); nodepos != nullptr;) {
				QMeshNode* node = (QMeshNode*)Toolpath->GetNodeList().GetNext(nodepos);
				if (!nodevisit[node->GetIndexNo()]) {
					snode = node; break;
				}
			}
		}

		if (snode == NULL)break;//all node is checked;

		nodevisit[snode->GetIndexNo()] = true;
		substripeline.push_back(snode->GetIndexNo());

		if (snode->GetEdgeNumber() == 0) { break; }

		QMeshEdge* sedge = (QMeshEdge*)snode->GetEdgeList().GetHead();
		
		bool openloop = true;

		QMeshNode* initialnode = snode;

		while (true)
		{
			if (sedge->GetStartPoint() == snode) {
				snode = sedge->GetEndPoint();
			}
			else
			{
				snode = sedge->GetStartPoint();
			}

			if (snode == initialnode) { openloop = false; break; }//closeloop terminal node
			nodevisit[snode->GetIndexNo()] = true;
			substripeline.push_back(snode->GetIndexNo());
			
			if (snode->GetEdgeNumber() == 1) break;//openloop terminal node

 			for (GLKPOSITION edgepos = snode->GetEdgeList().GetHeadPosition(); edgepos != nullptr;) {
				QMeshEdge* edge = (QMeshEdge*)snode->GetEdgeList().GetNext(edgepos);
				if (edge != sedge) {
					sedge = edge; break;
				}
			}
		}
			rerderednodelist.push_back({ openloop,substripeline });
 	}

// clamp to [0,1]
auto clamp01 = [](double t) {
	return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
	};

// 
auto clip_one_polyline = [&](const std::vector<int>& ids,
	auto getter, auto setter) -> std::vector<std::vector<int>> {

		std::vector<std::vector<int>> segs;
		std::vector<int> cur;
		if (ids.empty()) return segs;

		const double eps = 1e-12;

 		auto move_B_to_iso = [&](QMeshNode* A, QMeshNode* B, double vA, double vB) {
			double denom = vB - vA;
			if (std::abs(denom) < eps) {
 				setter(B, stripcutvalue);
				return;
			}
			double t = clamp01((stripcutvalue - vA) / denom); // A + t*(B-A)
			Eigen::Vector3d pA, pB;
			A->GetCoord3D(pA);
			B->GetCoord3D(pB);
			Eigen::Vector3d pI = (1.0 - t) * pA + t * pB;
			B->SetCoord3D(pI);
 			double hA = A->layerheight, hB = B->layerheight;
			B->layerheight = (1.0 - t) * hA + t * hB;
 			setter(B, stripcutvalue);
			};

 		for (size_t i = 1; i < ids.size(); ++i) {
			QMeshNode* A = Toolpath->GetNodeRecordPtr(ids[i - 1] + 1);
			QMeshNode* B = Toolpath->GetNodeRecordPtr(ids[i] + 1);
			double vA = getter(A);
			double vB = getter(B);
			bool aOut = (vA >= stripcutvalue);
			bool bOut = (vB >= stripcutvalue);

			if (i == 1 && aOut) {
				cur.push_back(ids[i - 1]);
			}

			if (aOut && bOut) {
				if (cur.empty()) cur.push_back(ids[i - 1]);
				cur.push_back(ids[i]);
			}
			else if (aOut && !bOut) {
 				move_B_to_iso(A, B, vA, vB);
				if (cur.empty()) cur.push_back(ids[i - 1]);  
				cur.push_back(ids[i]);
				segs.push_back(cur);
				cur.clear();
			}
			else if (!aOut && bOut) {
 				move_B_to_iso(A, B, vA, vB);
				cur.clear();
				cur.push_back(ids[i]); //  
			}
			else {
 			}
		}

		if (!cur.empty()) segs.push_back(cur);
		return segs;
	};

 auto clip_all_by_field = [&](auto getter, auto setter) {
	std::vector<std::pair<bool, std::vector<int>>> newList;
	newList.reserve(rerderednodelist.size());

	for (const auto& seg : rerderednodelist) {
		const std::vector<int>& ids = seg.second;
		auto pieces = clip_one_polyline(ids, getter, setter);
		for (auto& vecIds : pieces) {
			if (vecIds.size() >= 2) {  
				newList.push_back({ true, std::move(vecIds) });
			}
		}
	}
	rerderednodelist.swap(newList);
	};

 clip_all_by_field(
 	[](QMeshNode* n) -> double { return n->DisToBoundary; },
 	[](QMeshNode* n, double v) { n->DisToBoundary = v; }
);

 clip_all_by_field(
	[](QMeshNode* n) -> double { return n->DisToInnerBoundary; },
	[](QMeshNode* n, double v) { n->DisToInnerBoundary = v; }
);


auto segment_length = [&](const std::vector<int>& ids) -> double {
	if (ids.size() < 2) return 0.0;
	double L = 0.0;
	for (size_t j = 1; j < ids.size(); ++j) {
		Eigen::Vector3d p0, p1;
		Toolpath->GetNodeRecordPtr(ids[j - 1] + 1)->GetCoord3D(p0);
		Toolpath->GetNodeRecordPtr(ids[j] + 1)->GetCoord3D(p1);
		L += (p1 - p0).norm();
	}
	return L;
};

std::vector<std::pair<bool, std::vector<int>>> newList;
newList.reserve(rerderednodelist.size());

for (const auto& seg : rerderednodelist) {
	const auto& ids = seg.second;
	if (ids.size() < 2) continue;

	std::vector<int> cur;
	cur.reserve(ids.size());
	cur.push_back(ids[0]);

	for (size_t j = 1; j < ids.size(); ++j) {
		Eigen::Vector3d pprev, pcur;
		Toolpath->GetNodeRecordPtr(ids[j - 1] + 1)->GetCoord3D(pprev);
		Toolpath->GetNodeRecordPtr(ids[j] + 1)->GetCoord3D(pcur);
		double gap = (pcur - pprev).norm();

		if (gap > largegap) {
 			if (cur.size() >= 2) {
				double L = segment_length(cur);
				if (L >= removelinedistance) {
					newList.push_back({ true, cur });  
				}
			}
			cur.clear();
			cur.push_back(ids[j]);  
		}
		else {
			cur.push_back(ids[j]);
		}
	}
	//double cutstripnew = averagewidth*2;
 
 	if (cur.size() >= 2) {
		double L = segment_length(cur);
		if (L >= removelinedistance) {
			newList.push_back({ true, cur });
		}
	}
}

rerderednodelist.swap(newList);
 }

QMeshNode* DeformToolPath::buildstriptoolpath(QMeshNode*lnode) {

	std::vector<bool> substripelinevist(rerderednodelist.size(), false);
 	int startlineindex = 0;
	/*connect outterboundary*/
	 double mindis = INFINITY;
	bool reverseflag = false;
	int nodeindex = 0;
	std::vector<QMeshNode*>A_starpath;

	if (lnode != NULL)
	{

		for (int i = 0; i < rerderednodelist.size(); i++)
		{
			if (substripelinevist[i]) continue;
			if (rerderednodelist[i].first) {//openloop
				QMeshNode* snode = Toolpath->GetNodeRecordPtr(rerderednodelist[i].second[0] + 1);
				QMeshNode* enode = Toolpath->GetNodeRecordPtr(rerderednodelist[i].second[rerderednodelist[i].second.size() - 1] + 1);
				double dis_s = distance(snode, lnode);
				double dis_e = distance(enode, lnode);
				if (dis_s < mindis || dis_e < mindis) {
					if (dis_s < dis_e) {
						mindis = dis_s;
						startlineindex = i;
						reverseflag = false;
					}
					else
					{
						mindis = dis_e;
						reverseflag = true;
						startlineindex = i;
					}
				}
			}
			else//closeloop
			{
				for (int k = 0; k < rerderednodelist[i].second.size(); k++)
				{
					QMeshNode* snode = Toolpath->GetNodeRecordPtr(rerderednodelist[i].second[k] + 1);
					double dis_s = distance(snode, lnode);
					if (dis_s < mindis) {
						mindis = dis_s;
						startlineindex = i;
						nodeindex = k;
					}
				}
			}
		}


		if (rerderednodelist[startlineindex].first)//openloop
		{
			//std::cout << "open loop" << std::endl;

			if (reverseflag) {
				std::reverse(rerderednodelist[startlineindex].second.begin(), rerderednodelist[startlineindex].second.end());
			}
		}
		else//close loop
		{
			//std::cout << "close loop" << std::endl;
			std::vector<int> indexset;
			for (int i = nodeindex; i < rerderednodelist[startlineindex].second.size(); i++)
			{
				indexset.push_back(rerderednodelist[startlineindex].second[i]);
			}
			for (int i = 0; i < nodeindex; i++)
			{
				indexset.push_back(rerderednodelist[startlineindex].second[i]);
			}
			rerderednodelist[startlineindex].second = indexset;
		}
		if(outterboundarynum!=0&&!StripFirst)
		A_starpath = a_star(lnode, Toolpath->GetNodeRecordPtr(rerderednodelist[startlineindex].second[0] + 1));

	}
	 
 	int finalnodeindex = 0;
	/*connect stripe line*/
	while (true)
	{
		if (A_starpath.size() != 0) {
			for (int i = 0; i < A_starpath.size(); i++)
			{
				stripenode_order.push_back(A_starpath[i]);
			}
		}

		QMeshNode* firstnode = Toolpath->GetNodeRecordPtr(1 + rerderednodelist[startlineindex].second[0]);
		QMeshNode* secondnode = Toolpath->GetNodeRecordPtr(1 + rerderednodelist[startlineindex].second[1]);
		Eigen::Vector3d pos1, pos2, posnew,vec,normal1;
		firstnode->GetCoord3D(pos1);
		secondnode->GetCoord3D(pos2);
		vec = pos1 - pos2;
		vec.normalize();
		posnew = pos1 + (insertlength) * vec;
		QMeshNode* insertfnode1 = new QMeshNode;
		QMeshNode* insertfnode2 = new QMeshNode;

		firstnode->GetNormal(normal1(0), normal1(1), normal1(2));
		insertfnode1->SetNormal(normal1(0), normal1(1), normal1(2));
		insertfnode2->SetNormal(normal1(0), normal1(1), normal1(2));

		insertfnode1->layerheight = firstnode->layerheight;
		insertfnode2->layerheight = firstnode->layerheight;
 
		insertfnode1->SetCoord3D(posnew);
		posnew= posnew+ insertlength * vec;
		insertfnode2->SetCoord3D(posnew);

		stripenode_order.push_back(insertfnode2);
		stripenode_order.push_back(insertfnode1);
		insertfnode2->terminalnode = true;
		insertfnode1->terminalnodemovp = true;
		for (int i = 0; i < rerderednodelist[startlineindex].second.size(); i++)
		{
  			stripenode_order.push_back(Toolpath->GetNodeRecordPtr(1+ rerderednodelist[startlineindex].second[i]));
		}

		QMeshNode* tfirstnode = Toolpath->GetNodeRecordPtr(1 + rerderednodelist[startlineindex].second[rerderednodelist[startlineindex].second.size()-1]);
		QMeshNode* tsecondnode = Toolpath->GetNodeRecordPtr(1 + rerderednodelist[startlineindex].second[rerderednodelist[startlineindex].second.size() - 2]);
 		tfirstnode->GetCoord3D(pos1);
		tsecondnode->GetCoord3D(pos2);
		//std::cout << "pos1=" << pos1 << "pos2" << pos2 << std::endl;
		vec = pos1 - pos2;
		vec.normalize();
		posnew = pos1 + insertlength * vec;
		QMeshNode* inserttailnode = new QMeshNode;
		QMeshNode* inserttailnode2 = new QMeshNode;

		tfirstnode->GetNormal(normal1(0), normal1(1), normal1(2));
		inserttailnode->SetNormal(normal1(0), normal1(1), normal1(2));
		inserttailnode2->SetNormal(normal1(0), normal1(1), normal1(2));

		inserttailnode->layerheight = firstnode->layerheight;
		inserttailnode2->layerheight = firstnode->layerheight;

  
		inserttailnode->SetCoord3D(posnew);
		posnew = posnew + insertlength * vec;
		inserttailnode2->SetCoord3D(posnew);

		stripenode_order.push_back(inserttailnode);
		stripenode_order.push_back(inserttailnode2);

		inserttailnode2->terminalnode = true;
		inserttailnode->terminalnodemovp = true;

		substripelinevist[startlineindex] = true;
		QMeshNode* tailnode = Toolpath->GetNodeRecordPtr(rerderednodelist[startlineindex].second[rerderednodelist[startlineindex].second.size() - 1] + 1);

		mindis = INFINITY;
		//bool reverseflag = false;
		//int nodeindex = 0;
		int checkstate = startlineindex;
		for (int i = 0; i < rerderednodelist.size(); i++)
		{
			if (substripelinevist[i]) continue;
			if (rerderednodelist[i].first) {//openloop
				QMeshNode* snode = Toolpath->GetNodeRecordPtr(rerderednodelist[i].second[0] + 1);
				QMeshNode* enode = Toolpath->GetNodeRecordPtr(rerderednodelist[i].second[rerderednodelist[i].second.size() - 1] + 1);
				double dis_s = distance(snode, tailnode);
				double dis_e = distance(enode, tailnode);
				if (dis_s < mindis || dis_e < mindis) {
					if (dis_s < dis_e) {
						mindis = dis_s;
						startlineindex = i;
						reverseflag = false;
					}
					else
					{
						mindis = dis_e;
						reverseflag = true;
						startlineindex = i;
					}
				}
			}
			else//closeloop
			{
				for (int k = 0; k < rerderednodelist[i].second.size(); k++)
				{
					QMeshNode* snode = Toolpath->GetNodeRecordPtr(rerderednodelist[i].second[k] + 1);
					double dis_s = distance(snode, tailnode);
					if (dis_s < mindis) {
						mindis = dis_s;
						startlineindex = i;
						nodeindex = k;
					}
				}
			}
		}


		if (checkstate == startlineindex) { 
			finalnodeindex = rerderednodelist[startlineindex].second[rerderednodelist[startlineindex].second.size() - 1];
			break;
		}//startindex is not updated

		if (rerderednodelist[startlineindex].first)//openloop
		{
			if (reverseflag) {
				std::reverse(rerderednodelist[startlineindex].second.begin(), rerderednodelist[startlineindex].second.end());
			}
		}
		else
		{
			std::vector<int> indexset;
			for (int i = nodeindex; i < rerderednodelist[startlineindex].second.size(); i++)
			{
				indexset.push_back(rerderednodelist[startlineindex].second[i]);
			}
			for (int i = 0; i < nodeindex; i++)
			{
				indexset.push_back(rerderednodelist[startlineindex].second[i]);
			}
			rerderednodelist[startlineindex].second = indexset;
		}

		QMeshNode* enode= Toolpath->GetNodeRecordPtr(rerderednodelist[startlineindex].second[0] + 1);
		A_starpath = a_star(tailnode, enode);
		
	}
	return Toolpath->GetNodeRecordPtr(finalnodeindex + 1);
 }

 void DeformToolPath::getallpathlength(double& pathlength) {
	 pathlength = alpathlength;
 }

 void DeformToolPath::getvolumeoflayer() {
	 double sumvolume = 0;
	 for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		 QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);
		 double layerheight = 0;
		 for (int i = 0; i < 3; i++)
		 {
			 layerheight += face->GetNodeRecordPtr(i)->layerheight;
		 }
		 layerheight = layerheight / 3.0;
		 sumvolume += layerheight * face->CalArea();
 	 }
	 volumeoflayer = sumvolume;
 }

 double DeformToolPath::compute_volume() {
	 double printingvolume=0;
	 for (GLKPOSITION Pos = Toolpath->GetEdgeList().GetHeadPosition(); Pos;) {
		 QMeshEdge* Edge = (QMeshEdge*)Toolpath->GetEdgeList().GetNext(Pos);
		 if (Edge->GetEndPoint()->noextrusionnode) { Edge->invisible = true; continue; }
		 if (Edge->GetStartPoint()->terminalnodemovp || Edge->GetEndPoint()->terminalnodemovp) {
			 double sheight = Edge->GetStartPoint()->layerheight;
			 double eheight = Edge->GetEndPoint()->layerheight;
			 double sarea = getarea(sheight);
			 double earea = getarea(eheight);
			 printingvolume += 1.0 / 3.0 * (Edge->CalLength()) * (sarea + earea + sqrt(sarea * earea));
			 continue; }
		 double sheight = Edge->GetStartPoint()->layerheight;
		 double eheight = Edge->GetEndPoint()->layerheight;
		 double sarea = getarea(sheight);
		 double earea = getarea(eheight);
		 printingvolume += 1.0 / 3.0 * Edge->CalLength() * (sarea + earea + sqrt(sarea * earea));
	 }
	 return printingvolume;
 }

 double DeformToolPath::getarea(double height) {
		 // std::cout << "1.75" << std::endl;
		 return 1.84 * 0.01 / getvelocity(height);//
 }

 double DeformToolPath::getvelocity(double height) {
 
 
	 std::vector<std::pair<double, double>> data = {
		{0.477,0.033},//
		{0.531,0.03},//
		{0.5985,0.0267},//
		{0.6165,0.0233},//
		{0.639,0.020},//
		{0.7065,0.01667},//
		{0.765,0.01333},//
		{0.945,0.01},//
	 };
 
	 double value = 0;
 	 for (size_t i = 0; i < data.size() - 1; i++) {
		 double x1 = data[i].first, y1 = data[i].second;
		 double x2 = data[i + 1].first, y2 = data[i + 1].second;

 		 if (height >= x1 && height <= x2) {
 			 value= (y1 + (y2 - y1) * (height - x1) / (x2 - x1));
		 }
	 }

 	 if (height > data.back().first) value= (data.back().second);
	 if (height < data.front().first) value= (data.front().second);
 

	 return value;  
 }


 double DeformToolPath::getwidth(double height) {
	 double normalwidth= getarea(height) / height;
	 double ratio = 1.0;//(0.45 - 1.0)* (getvelocity(height) - 0.02) / (0.01- 0.02) + 1.0;
	 return normalwidth* ratio;

  }
 
 void DeformToolPath::resamplingSinglePatch(QMeshPatch* patch) {

	 for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		 QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		 Node->resampleChecked = false;
		 Node->printvelocity = getvelocity(Node->layerheight);
	 }


	 for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		 QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		 QMeshNode* snode = Edge->GetStartPoint();
		 QMeshNode* enode = Edge->GetEndPoint();
		 if (snode->terminalnode && enode->terminalnode) {
			 Edge->isConnectEdge = true;
		 }
	 }
	 double lsum = 0.0;
	 //length = 0.5;

	 QMeshNode* sNode = (QMeshNode*)patch->GetNodeList().GetHead();
	// sNode->resampleChecked = true;
	  
	 QMeshNode* sPoint = sNode;
	 QMeshNode* ePoint;

	 for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		 QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

		 ePoint = Edge->GetStartPoint();
		 if (ePoint == sPoint) { ePoint = Edge->GetEndPoint(); }
		 else
		 {
			 std::cout << "the direction is wrong!" << std::endl;
		 }


		 if (ePoint == patch->GetNodeList().GetTail()) {
			 //std::cout<<"End Node connected Edge"<< Edge->GetIndexNo()<< "::"<<sNode->GetIndexNo()<<","<<ePoint->GetIndexNo()<<endl;
			 ePoint->resampleChecked = true;
			 break;
		 }

		 lsum += Edge->CalLength();
		 if (ePoint->terminalnode || ePoint->terminalnodemovp) {
			 ePoint->resampleChecked = true;
			 sNode = ePoint;
			 sPoint = sNode;
			 lsum = 0;
		 }
		 if (lsum > resampleLength) {
			 //std::cout<<"Inner Edge"<< Edge->GetIndexNo()<< "::"<<sNode->GetIndexNo()<<","<<ePoint->GetIndexNo()<<endl;
			 ePoint->resampleChecked = true;

			 sNode = ePoint;
			 sPoint = sNode;
			 lsum = 0;
		 }
		 else {
			 sPoint = ePoint;
		 }
	 }
	 for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		 QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		 if (Node->terminalnode || Node->terminalnodemovp) {
			 Node->resampleChecked = true;
		 }
	 }
	 //rebuild the edge
	 patch->GetEdgeList().RemoveAll();
	 sNode = (QMeshNode*)patch->GetNodeList().GetHead();
	 sNode->resampleChecked = false;
	 for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		 QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		 if (Node->resampleChecked) {

			 Eigen::Vector3d nordir;
			 Node->GetNormal(nordir(0), nordir(1), nordir(2));

			 QMeshEdge* isoEdge = new QMeshEdge;
			 isoEdge->SetStartPoint(sNode);
			 isoEdge->SetEndPoint(Node);

			 isoEdge->SetMeshPatchPtr(patch);
			 isoEdge->SetIndexNo(patch->GetEdgeList().GetCount());

			 (sNode->GetEdgeList()).AddTail(isoEdge);
			 (Node->GetEdgeList()).AddTail(isoEdge);
			 patch->GetEdgeList().AddTail(isoEdge);

 			 sNode = Node;
		 }
	 }
 
 
	 for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		 QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		 if (Edge->GetStartPoint()->terminalnodemovp && Edge->GetEndPoint()->terminalnode) {
			 Edge->GetStartPoint()->noextrusionnode = false;
			 Edge->GetEndPoint()->noextrusionnode = false;
		 }
		 if (Edge->GetEndPoint()->terminalnodemovp && Edge->GetStartPoint()->terminalnode) {
			 Edge->GetEndPoint()->noextrusionnode = false;
			 Edge->GetStartPoint()->noextrusionnode = true;
		 }
		 if (Edge->GetStartPoint()->terminalnodemovp && !Edge->GetEndPoint()->terminalnode) {
			 Edge->GetEndPoint()->noextrusionnode = false;
			 Edge->GetStartPoint()->noextrusionnode = false;
		 }
		 if (!Edge->GetStartPoint()->terminalnode && Edge->GetEndPoint()->terminalnodemovp) {
			 Edge->GetStartPoint()->noextrusionnode = false;
			 Edge->GetEndPoint()->noextrusionnode = false;
		 }
		 if (Edge->GetEndPoint()->isconnectnode) {
			 Edge->GetEndPoint()->noextrusionnode = true;
		 }
 	 }


	 
	 //double sumlength;
	 for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		 QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

		 if(!Edge->GetEndPoint()->noextrusionnode)
		 alpathlength += Edge->CalLength();
	 }
	 for (int i = 0; i < patch->GetEdgeNumber()-1; i++)
	 {
		 QMeshEdge* edge = patch->GetEdgeRecordPtr(i + 1);
		 QMeshEdge* nextedge = patch->GetEdgeRecordPtr(i + 2);
		 if (edge->GetEndPoint() != nextedge->GetStartPoint()) {
			 std::cout << "edge is not connected====================================================" << std::endl;
		 }
	 }
 }

 void DeformToolPath::checknoextrusionnode(QMeshPatch* patch) {
	 bool startflag = true;
	 std::vector<QMeshNode*>finalnodevec;
	 for (GLKPOSITION edgepos = patch->GetEdgeList().GetHeadPosition(); edgepos != nullptr;) {
		 QMeshEdge* edge = (QMeshEdge*)patch->GetEdgeList().GetNext(edgepos);
		 if (startflag) {
			 double nx, ny, nz;
			 finalnodevec.push_back(edge->GetStartPoint());
			 startflag = false;
		 }
		 finalnodevec.push_back(edge->GetEndPoint());
	 }
	 double noexvalue = 0;

	 std::vector<bool>ismovel(finalnodevec.size(), false);
	 if (surfaceMesh->GetIndexNo() != 0) {
		 finalnodevec[0]->noextrusionnode = true;
		 finalnodevec[1]->noextrusionnode = true;
	 }
 		 for (int i = 2; i < finalnodevec.size() - 1; i++)
		 {
			 Eigen::Vector3d nextpos, prevpos, thispos, nectpos, thisnor, vec1, vec2, vec3;
			 finalnodevec[i + 1]->GetCoord3D(nextpos);
			 finalnodevec[i - 1]->GetCoord3D(prevpos);
			 finalnodevec[i]->GetCoord3D(thispos);
			 vec1 = nextpos - thispos;
			 vec2 = thispos - prevpos;
			 bool samenodeflag = false;

			 if (vec1.norm() < 0.0001 || vec2.norm() < 0.0001) {
				 samenodeflag = true;
			 }
			 vec1.normalize();
			 vec2.normalize();
			 if (vec1.dot(vec2) < noexvalue || samenodeflag) {
				 finalnodevec[i]->noextrusionnode = true;
				 //std::cout << "movel node" << std::endl;
				 ismovel[i] = true;
			 }
			 if (ismovel[i - 1] && !ismovel[i] && !ismovel[i - 2]) {
				 ismovel[i] = true;
				 finalnodevec[i]->noextrusionnode = true;
				 ismovel[i - 2] = true;
				 finalnodevec[i - 2]->noextrusionnode = true;

			 }
		 }
	  
	 
 }
  
 
double DeformToolPath::distance(QMeshNode*node1,QMeshNode*node2) {
	Eigen::Vector3d pos1,pos2,vec;
	node1->GetCoord3D(pos1);
	node2->GetCoord3D(pos2);
	vec = pos2 - pos1;
	return vec.norm();
}

 