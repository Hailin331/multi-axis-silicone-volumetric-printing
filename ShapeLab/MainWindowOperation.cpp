#include "stdafx.h"
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "isoSurface.h"
#include "ScalarField.h"
//#include "DeformTet.h"
#include "layerSegmentation.h"
#include <fstream>



#include "fileIO.h"

#include"IsoLayerGeneration.h"

#include "alphanum.hpp"

 
#include "GlobalVar.h"

using namespace std;


 
 

// volume decomposition by scalar field optimization
void MainWindow::Compute_ISO_Surface() {

	//bool planarLayerSlicing = false;
	//bool readInstalledField_staticAnalysis = true; // for making figures of the paper
	auto t0 = std::chrono::steady_clock::now();

	QMeshPatch* patch;
	for (GLKPOSITION polygenPos = polygenMeshList.GetHeadPosition(); polygenPos;)   
	{
		PolygenMesh* polyele = (PolygenMesh*)polygenMeshList.GetNext(polygenPos);
		if (polyele->meshType==TET)
		{
			patch = (QMeshPatch*)polyele->GetMeshList().GetHead();
			break;
		}
	}
	double allvolume = 0;
	int tetIndex = 0;
	for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);
		tetra->SetIndexNo(tetIndex);
		tetIndex++;
		allvolume += tetra->CalVolume();

	}
	std::cout << "volume=" << allvolume << std::endl;
	int nodeindex = 0;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		node->SetIndexNo(nodeindex);
		nodeindex++;
	}
	int edgeindex = 0;
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		edge->SetIndexNo(edgeindex);
		edgeindex++;
	}
	int faceindex = 0;
	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		face->SetIndexNo(faceindex);
		faceindex++;
	}
 
		ScalarField* sFieldComp =new ScalarField(patch, false);
		sFieldComp->compScalarField();
		patch->drawScalarField = true;
		patch->drawVectorField = true;
	//build isosuface

	////////GENERATE ISO LAYER***********************************************************
	PolygenMesh* isosurfaceSet = this->_buildPolygenMesh(CURVED_LAYER, "Init Curved Layer");


	//////// generate surface and project principal stress direction onto surface
	isoSurface* isoSurfaceOperator = new isoSurface(patch);
	isoSurfaceOperator->generateIsoSurface(isosurfaceSet, 340);//340 for TTA  
	auto t1 = std::chrono::steady_clock::now();
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
	std::cout << "foo took " << ms << " ms\n";

	updateTree();
	pGLK->refresh(true);
	return;
}

 

PolygenMesh* MainWindow::_detectPolygenMesh(mesh_type type) {

	PolygenMesh* detectedMesh = NULL;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* thispolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (thispolygenMesh->meshType == type) {
			detectedMesh = thispolygenMesh; break;
		}
	}
	return detectedMesh;

}

void MainWindow::_buildFileNameSetbySorting(std::vector<std::string>& files, std::string fieldPath) {

	std::cout << fieldPath << std::endl;
	DIR* dp;
	struct dirent* ep;
	dp = opendir(fieldPath.c_str());


	if (dp == NULL) { perror("Couldn't open the directory"); return; }
	while (ep = readdir(dp))
	{
		if ((string(ep->d_name) != ".") && (string(ep->d_name) != ".."))
		{
			if (string(ep->d_name).find(".obj") == string::npos &&
				string(ep->d_name).find(".txt") == string::npos) continue;
			files.push_back(string(ep->d_name));
		}
	}
	(void)closedir(dp);

	cout << "There are " << files.size() << " files in the current directory." << endl;
	auto alphanum_greater = [](const std::string& a, const std::string& b) {
		return doj::alphanum_less<std::string>()(b, a); // µßµ¹ a ºÍ b
		};
	//std::sort(files.begin(), files.end(), alphanum_greater);
	sort(files.begin(), files.end(), doj::alphanum_less<std::string>());
	//sort(files.begin(), files.end(), doj::alphan<std::string>());

}

 

void MainWindow::outputIsoLayersetandlayerheight()
{

	/*-------------------------------
	build the folder to save isolayer
	-------------------------------*/
	std::string ModelName = "TTA";
	std::string folderPath = "..\\Model\\IsoSurface\\"+ ModelName;

	////////-- System command to build folder
	string command = "mkdir " + folderPath;
	string command_height = "mkdir " + folderPath +"\\layerheight";

	//string command_toolpath = "mkdir " + folderPath + "\\toolpath";

	system(command.c_str());
	system(command_height.c_str());

	/*-------------------------------------------
	//find the isosurface polygenmesh in the system
	-------------------------------------------*/

	PolygenMesh* isosurfaceSet = NULL;

	for (GLKPOSITION posMesh = polygenMeshList.GetHeadPosition(); posMesh != nullptr;) {
		PolygenMesh* mesh = (PolygenMesh*)polygenMeshList.GetNext(posMesh);
		if (mesh->meshType == CURVED_LAYER) { isosurfaceSet = mesh; break; }
	}
	std::cout << "LayerNUM=" << isosurfaceSet->GetMeshList().GetCount() << std::endl;
	if (isosurfaceSet == NULL) { printf(" No iso-surface detected in the system! \n"); return; }

	/*---------------
	Output isoSurface
	---------------*/

	bool splitMode = false;
	bool singleOutputMode = false;
	bool offMode = false;
	int maxLayerNum = 600;

	fileIOObject->outputISOSurfaceMesh(
		isosurfaceSet, splitMode, singleOutputMode, ModelName, maxLayerNum, offMode);//save the obj to the filefolder

	system("python my_script.py");	//remesh TTA and save new obj to TTA2

	isosurfaceSet->ClearAll();
	isosurfaceSet = NULL;
	/*input the remeshed OBJ for procession*/
	 ModelName = "TTA2";
	 folderPath = "..\\Model\\IsoSurface\\" + ModelName;
	std::vector<std::string> layersFileSet;
	_buildFileNameSetbySorting(layersFileSet, folderPath);

	isosurfaceSet = this->_buildPolygenMesh(CURVED_LAYER, "isoSurface");

	fileIOObject->inputInstalledIsoSurface(
		isosurfaceSet, layersFileSet, folderPath);
	std::cout << "layer output finished" << std::endl;
	/*calculate the new layerheight*/
	IsoLayerGeneration* Lheight = new IsoLayerGeneration();
	Lheight->_installLayerDis_toNode(isosurfaceSet);
	delete Lheight;
	/*save the new layerheight*/
	for (GLKPOSITION posMesh = isosurfaceSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoSurface = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(posMesh);
		fileIOObject->outputlayerheight(isoSurface, ModelName);
	}

	updateTree();
	pGLK->refresh(true);
	return;
}

void MainWindow::outputIsoLayer()
{
	PolygenMesh* initialModel = (PolygenMesh*)polygenMeshList.GetHead();
	if (initialModel == NULL || initialModel->meshType != TET) { printf(" No tet model detected in the system! \n"); return; }
	/*-------------------------------
	build the folder to save isolayer
	-------------------------------*/

	std::string folderPath = "..\\Model\\IsoSurface\\" + initialModel->getModelName();

	//-- System command to build folder
	string command = "mkdir " + folderPath;
	string command_saveField = "mkdir " + folderPath + "\\Field";
	string command_support = "mkdir " + folderPath + "\\Support";
	string command_supportTrim = "mkdir " + folderPath + "\\Support_Trim";
	string command_supportSplit = "mkdir " + folderPath + "\\Support_Split";
	string command_layerFieldRender = "mkdir " + folderPath + "\\layerFieldRender";

	string command_toolpath = "mkdir " + folderPath + "\\toolpath";

	system(command.c_str()); system(command_saveField.c_str());
	system(command_support.c_str()); system(command_supportTrim.c_str());
	system(command_supportSplit.c_str()); system(command_toolpath.c_str());
	system(command_layerFieldRender.c_str());

	/*-------------------------------------------
	find the isosurface polygenmesh in the system
	-------------------------------------------*/

	PolygenMesh* isosurfaceSet = NULL;
	PolygenMesh* isosurfaceSet_support = NULL;
	for (GLKPOSITION posMesh = polygenMeshList.GetHeadPosition(); posMesh != nullptr;) {
		PolygenMesh* mesh = (PolygenMesh*)polygenMeshList.GetNext(posMesh);
		if (mesh->meshType == SUPPORT_LAYERS) { isosurfaceSet_support = mesh; break; }
	}
	for (GLKPOSITION posMesh = polygenMeshList.GetHeadPosition(); posMesh != nullptr;) {
		PolygenMesh* mesh = (PolygenMesh*)polygenMeshList.GetNext(posMesh);
		if (mesh->meshType == CURVED_LAYER) { isosurfaceSet = mesh; break; }
	}
	std::cout << "LayerNUM=" << isosurfaceSet->GetMeshList().GetCount() << std::endl;
	if (isosurfaceSet == NULL) { printf(" No iso-surface detected in the system! \n"); return; }

	/*---------------
	Output isoSurface
	---------------*/

	if (isosurfaceSet != NULL && isosurfaceSet_support != NULL) {
		fileIOObject->updateInitialSurfaceName(isosurfaceSet, isosurfaceSet_support);
	}

	bool splitMode = true;
	bool singleOutputMode = false;
	bool offMode = false;
	int maxLayerNum = 200;

	//notice that the iso-surface toolpath stress field will also be installed.
	// delete by SUN HL 2027/7/20
	//fileIOObject->minPrincipleStressValue = ((QMeshPatch*)initialModel->GetMeshList().GetHead())->minPrincipleStressValue;
	//fileIOObject->maxPrincipleStressValue = ((QMeshPatch*)initialModel->GetMeshList().GetHead())->maxPrincipleStressValue;
	
	fileIOObject->outputISOSurfaceMesh(
		isosurfaceSet, splitMode, singleOutputMode, initialModel->getModelName(), maxLayerNum, offMode);


	std::cout << "layer output finished" << std::endl;
	/*if (isosurfaceSet_support != NULL) {
		fileIOObject->outputISOSurfaceMesh_support(isosurfaceSet_support, initialModel->getModelName());
	}*/
	/*----------------------------------------------------------------------------
	Output vector and scalr field for drawing (for SIGGRAPH Asia paper 2020-03-12)
	----------------------------------------------------------------------------*/

	//fileIOObject->saveFieldforRendering(initialModel);
}
 
void MainWindow::ReadmeshandInfo() {
	// Fixed relative path (Windows-style). QDir will normalize separators if needed.
	const QString filenameStr = QDir::cleanPath("..\\Model\\TTAcut.tet");

	// Check whether the file exists before importing
	QFileInfo fileInfo(filenameStr);
	if (!fileInfo.exists() || !fileInfo.isFile()) {
		std::cerr << "File not found: " << filenameStr.toStdString() << std::endl;
		return;
	}

	// Get file suffix (extension)
	const QString fileSuffix = fileInfo.suffix();

	// Convert QString to std::string for functions expecting const char*
	const std::string filenameStd = filenameStr.toLocal8Bit().constData();
	const char* filename = filenameStd.c_str();

	// Set polygen name from file base name (without extension)
	const std::string modelName = fileInfo.completeBaseName().toStdString();

	if (QString::compare(fileSuffix, "obj", Qt::CaseInsensitive) == 0) {

		PolygenMesh* polygenMesh = new PolygenMesh(UNDEFINED);
		polygenMesh->ImportOBJFile(const_cast<char*>(filename), modelName);
		polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
		pGLK->AddDisplayObj(polygenMesh, true);
		polygenMeshList.AddTail(polygenMesh);

	}
	else if (QString::compare(fileSuffix, "tet", Qt::CaseInsensitive) == 0) {

		PolygenMesh* polygenMesh = new PolygenMesh(TET);
		std::cout << filename << std::endl;
		std::cout << modelName << std::endl;
		polygenMesh->ImportTETFile(const_cast<char*>(filename), modelName);
		polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
		pGLK->AddDisplayObj(polygenMesh, true);
		polygenMeshList.AddTail(polygenMesh);

	}
	else {
		// Unsupported file format
		std::cerr << "Unsupported file suffix: " << fileSuffix.toStdString() << std::endl;
		return;
	}
	readSelection();
	updateTree();
	shiftToOrigin();
	pGLK->refresh(true);

}
void MainWindow::DeformationToolpath() {

	PolygenMesh* isosurfaceSet = NULL;
	PolygenMesh* toolpath = this->_buildPolygenMesh(Tool_PATH, "toolpath");


	for (GLKPOSITION posMesh = polygenMeshList.GetHeadPosition(); posMesh != nullptr; ) {
		PolygenMesh* mesh = (PolygenMesh*)polygenMeshList.GetNext(posMesh);
		if(mesh->GetMeshList().GetCount()!=0&&mesh->getModelName()=="isoSurface")
		isosurfaceSet = mesh;
		else
		{
			continue;
		}
		// Angles in radians
		const double roangleY =0.0 * 3.14159269 / 180.0;
		const double roangleX = 0.0 * 3.14159269 / 180.0;

		// Compose rotations: original code did (pos' = Rx * (Ry * pos)), so R = Rx * Ry
		const Eigen::Matrix3d Ry = Eigen::AngleAxisd(roangleY, Eigen::Vector3d::UnitY()).toRotationMatrix();
		const Eigen::Matrix3d Rx = Eigen::AngleAxisd(roangleX, Eigen::Vector3d::UnitX()).toRotationMatrix();
		const Eigen::Matrix3d R = Rx * Ry;

		// Choose pivot: centroid or origin
		const bool rotateAboutCentroid = false;

		Eigen::Vector3d center(0, 0, 0);
		if (rotateAboutCentroid) {
			size_t cnt = 0;
			for (GLKPOSITION possurMesh = isosurfaceSet->GetMeshList().GetHeadPosition(); possurMesh != nullptr; ) {
				QMeshPatch* smesh = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(possurMesh);
				for (GLKPOSITION posnode = smesh->GetNodeList().GetHeadPosition(); posnode != nullptr; ) {
					QMeshNode* node = (QMeshNode*)smesh->GetNodeList().GetNext(posnode);
					Eigen::Vector3d p; node->GetCoord3D(p);
					center += p; ++cnt;
				}
			}
			if (cnt > 0) center /= double(cnt);
		}
		else {
			center.setZero(); // rotate about origin
		}
		std::cout << "layernum==" << isosurfaceSet->GetMeshList().GetCount() << std::endl;
		// Single pass: rotate positions and normals
		for (GLKPOSITION possurMesh = isosurfaceSet->GetMeshList().GetHeadPosition(); possurMesh != nullptr; ) {
			QMeshPatch* smesh = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(possurMesh);
			for (GLKPOSITION posnode = smesh->GetNodeList().GetHeadPosition(); posnode != nullptr; ) {
				QMeshNode* node = (QMeshNode*)smesh->GetNodeList().GetNext(posnode);

				// Position
				Eigen::Vector3d pos; node->GetCoord3D(pos);
				pos = R * (pos - center) + center;
				node->SetCoord3D(pos);
			}
		}

		break;
	}

	double k = 0;
	Eigen::Vector3d sumpos; sumpos.setZero();
 
 		for (GLKPOSITION possurMesh = isosurfaceSet->GetMeshList().GetHeadPosition(); possurMesh != nullptr;) {
			QMeshPatch* smesh = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(possurMesh);

			for (GLKPOSITION posface = smesh->GetFaceList().GetHeadPosition(); posface != nullptr;) {
				QMeshFace* face = (QMeshFace*)smesh->GetFaceList().GetNext(posface);
				face->CalPlaneEquation();
			}
		}
	 
	sumpos = sumpos/k;
 


	char waypointFile[500];
 	sprintf(waypointFile, "%s", "../model/waypoint.txt");
	ofstream WPFile(waypointFile);
	//auto t0 = std::chrono::steady_clock::now();
 
	struct Job {
		int index;              // i
		QMeshPatch* smesh;       //  
		QMeshPatch* subpath;     //  
		bool rotationflag;
	};

	auto t0 = std::chrono::steady_clock::now();

 
	std::vector<Job> jobs;
	jobs.reserve(340); // 

	int i = 0;
	for (GLKPOSITION possurMesh = isosurfaceSet->GetMeshList().GetHeadPosition();
		possurMesh != nullptr; )
	{
		QMeshPatch* smesh = (QMeshPatch*)isosurfaceSet->GetMeshList().GetNext(possurMesh);

		//if (i > 100) break;

		bool rotationflag = (i % 2 != 0);

 		QMeshPatch* subpath = new QMeshPatch;
		toolpath->GetMeshList().AddTail(subpath);

		smesh->SetIndexNo(i);
		//std::cout << "index=" << i << std::endl;

		jobs.push_back(Job{ i, smesh, subpath, rotationflag });

		++i;
	}

 
	const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
	const unsigned maxConcurrent = hw; //  

	std::vector<std::future<void>> futures;
	futures.reserve(jobs.size());

	auto launchOne = [](const Job& job) {
 

		if (job.rotationflag) {
			double upboundfre = 5.5;
			double lowboundfre = 3;
			double freeuqncy = 4;

 			DeformToolPath pathcomp(
				job.smesh, job.subpath, NULL,
				freeuqncy, job.rotationflag,
				0, true,
				upboundfre, lowboundfre,
				0, 0
			);
			pathcomp.Generatefinaltoolpath();
		}
		else {
			double freeuqncy = 0;
			double uppercutvalue = 6;
			double lowercutvalue = -6;

			DeformToolPath pathcomp(
				job.smesh, job.subpath, NULL,
				freeuqncy, job.rotationflag,
				0, true,
				0, 0,
				uppercutvalue, lowercutvalue
			);
			pathcomp.Generatefinaltoolpath();
		}
		};

 	size_t next = 0;
	while (next < jobs.size()) {

 		while (futures.size() < maxConcurrent && next < jobs.size()) {
			const Job& job = jobs[next++];
			futures.emplace_back(std::async(std::launch::async, launchOne, std::cref(job)));
		}

	 
		for (size_t k = 0; k < futures.size(); ++k) {
			if (futures[k].wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
				futures[k].get();
				futures.erase(futures.begin() + k);
				break;
			}
		}
	}

 	for (auto& f : futures) f.get();

 	for (const auto& job : jobs) {
		job.subpath->SetIndexNo(job.index);
	}

	auto t1 = std::chrono::steady_clock::now();
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
	std::cout << "foo took " << ms << " ms\n";

	for (GLKPOSITION possurMesh = toolpath->GetMeshList().GetHeadPosition(); possurMesh != nullptr;) {
		QMeshPatch* subpath = (QMeshPatch*)toolpath->GetMeshList().GetNext(possurMesh);

		for (GLKPOSITION edgepos = subpath->GetEdgeList().GetHeadPosition(); edgepos != nullptr;) {
			QMeshEdge* edge = (QMeshEdge*)subpath->GetEdgeList().GetNext(edgepos);
			QMeshNode* node = edge->GetEndPoint();

			double nx, ny, nz;
			node->GetNormal(nx, ny, nz);
 			if (nz < 0.0) {
				node->SetNormal(-nx, -ny, -nz);
 			}
		}
	}

 	std::vector<QMeshNode*> nodevec;
	bool startflag = true;
	for (GLKPOSITION possurMesh = toolpath->GetMeshList().GetHeadPosition(); possurMesh != nullptr;) {
		QMeshPatch* subpath = (QMeshPatch*)toolpath->GetMeshList().GetNext(possurMesh);

		for (GLKPOSITION edgepos = subpath->GetEdgeList().GetHeadPosition(); edgepos != nullptr;) {
			QMeshEdge* edge = (QMeshEdge*)subpath->GetEdgeList().GetNext(edgepos);

			if (startflag) {
				double nx, ny, nz;
				QMeshNode* startNode = edge->GetStartPoint();
				startNode->GetNormal(nx, ny, nz);

 				if (nz < 0.0) {
					startNode->SetNormal(-nx, -ny, -nz);
 				}
				nodevec.push_back(startNode);
				startflag = false;
			}
			nodevec.push_back(edge->GetEndPoint());
		}
	}


	Eigen::Vector3d pos3d, nor3d;

	for (int i = 0; i < nodevec.size() ; i++)
	{
		Eigen::Vector3d pos3dnew;
		nodevec[i]->GetCoord3D(pos3d);
		nodevec[i]->GetNormal(nor3d(0), nor3d(1), nor3d(2));	
		WPFile << pos3d(0) << " " << pos3d(1) << " " << pos3d(2) << " " << nor3d(0) << " " << nor3d(1) << " " << nor3d(2) << " " <<  nodevec[i]->printvelocity  << " " << nodevec[i]->noextrusionnode << " " << nodevec[i]->layerheight<<std::endl;	 
	}

	WPFile.close();

	updateTree();
	pGLK->refresh(true);
	return;

}
 
 

void MainWindow::ReadISOLayerandLHeight()
{
	PolygenMesh* isosurfaceSet = NULL;
 	for (GLKPOSITION posMesh = polygenMeshList.GetHeadPosition(); posMesh != nullptr;) {
		PolygenMesh* mesh = (PolygenMesh*)polygenMeshList.GetNext(posMesh);
		if (mesh->meshType == CURVED_LAYER) { isosurfaceSet = mesh; break; }
	}
	if (isosurfaceSet == NULL)
		isosurfaceSet = this->_buildPolygenMesh(CURVED_LAYER, "isoSurface");
	isosurfaceSet->ClearAll();

	std::string modelname = "TTA2";
	std::string folderPath = "..\\Model\\IsoSurface\\" + modelname;
	std::vector<std::string> layersFileSet;
	std::vector<std::string> LheightFileSet;

	_buildFileNameSetbySorting(layersFileSet, folderPath);
	_buildFileNameSetbySorting(LheightFileSet, folderPath + "\\layerheight");

	fileIOObject->inputInstalledIsoSurface(
		isosurfaceSet, layersFileSet, LheightFileSet, folderPath);
 
	updateTree();
	pGLK->refresh(true);
	return;
}
 
 
  
 