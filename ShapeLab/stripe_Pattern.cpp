#include "stripe_Pattern.h"
#include <cmath>

int stripe_Pattern::computeRank(const Eigen::SparseMatrix<double>& mat, double tol = 1e-6) {
	// Ensure the matrix is square (you can handle rectangular matrices separately if needed)
	Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
	qr.compute(mat);

	if (qr.info() != Eigen::Success) {
		throw std::runtime_error("QR decomposition failed");
	}

	Eigen::SparseMatrix<double> R = qr.matrixR();

	int rank = 0;
	for (int k = 0; k < std::min(R.rows(), R.cols()); ++k) {
		if (std::abs(R.coeff(k, k)) > tol) {
			rank++;
		}
	}

	return rank;
}

Eigen::Vector3d stripe_Pattern::rotateVector3D(const Eigen::Vector3d& vec, const Eigen::Vector3d& axis, double angle) {
	// Ensure the axis is a unit vector
	Eigen::Vector3d unitAxis = axis.normalized();

	// Create the skew-symmetric matrix for the axis
	Eigen::Matrix3d K;
	K << 0, -unitAxis.z(), unitAxis.y(),
		unitAxis.z(), 0, -unitAxis.x(),
		-unitAxis.y(), unitAxis.x(), 0;

	// Compute the 3D rotation matrix using Rodrigues' formula
	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity() +
		std::sin(angle) * K +
		(1 - std::cos(angle)) * (K * K);

	// Apply the rotation matrix to the vector
	return rotationMatrix * vec;
}


void stripe_Pattern::find_referenceedge() {
	for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
	{
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
		bool boundarynode = false;
		QMeshEdge* edge1; QMeshEdge*edge2;
		for (GLKPOSITION edgepos = node->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
		{
			QMeshEdge* edge = (QMeshEdge*)node->GetEdgeList().GetNext(edgepos);
			if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) {
				if (boundarynode == false) {
					boundarynode = true;
					edge1 = edge;
					}
				else
				{
					edge2 = edge;
				}
			}
 		}
		if (boundarynode == false) {
			node->isBoundaryNode = false;
			refedge_ptr[node->GetIndexNo()] = (QMeshEdge*)node->GetEdgeList().GetHead(); }
		else
		{
			node->isBoundaryNode = true;
			QMeshFace* sface;
			if (edge1->GetLeftFace() == NULL)  sface = edge1->GetRightFace();
			else sface = edge1->GetLeftFace();
			QMeshEdge* secondedge=NULL;
			//if(sface==NULL) std::cout<<"face=NULL!!!!!!!!!!!!!!!!" << std::endl;
			for (int i = 0; i < 3; i++)
			{
				QMeshEdge* nedge = sface->GetEdgeRecordPtr(i + 1);
				if (nedge->GetStartPoint() != node && nedge->GetEndPoint() != node) continue;
				if (nedge == edge1) continue;
				secondedge = nedge;
				break;
			}
			if (secondedge == NULL)std::cout << "edge=NULL!!!!!!!!!!!!!!!!" << std::endl;
			Eigen::Vector3d posnode, pos1, pos2, vec1, vec2, nor;
			node->GetCoord3D(posnode);
			if (edge1->GetStartPoint() != node) edge1->GetStartPoint()->GetCoord3D(pos1);
			else edge1->GetEndPoint()->GetCoord3D(pos1);
			if (secondedge->GetStartPoint() != node) secondedge->GetStartPoint()->GetCoord3D(pos2);
			else secondedge->GetEndPoint()->GetCoord3D(pos2);
 			vec1 = pos1 - posnode;
			vec2 = pos2 - posnode;
			sface->GetNormal(nor(0), nor(1), nor(2));
			double angle = signed_angle_between_vectors(vec1, vec2, nor);
			if (angle > 0) {
				refedge_ptr[node->GetIndexNo()] = edge1;
			}
			else
			{
				refedge_ptr[node->GetIndexNo()] = edge2;
			}
		}
		nodesumangle[node->GetIndexNo()]= computeallangelofnode(node);
	}
}
void stripe_Pattern::require_guide_complex() {
	int errorcount = 0;
	for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
	{
		//Eigen::Vector3d nordir(0, 0, 1);
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
		QMeshEdge* refedge = refedge_ptr[node->GetIndexNo()];
		int facenum = 0;
		Eigen::Vector3d norsum = { 0,0,0 };
		for (GLKPOSITION facepos=node->GetFaceList().GetHeadPosition();facepos!=NULL;)
		{
			QMeshFace* face = (QMeshFace*)node->GetFaceList().GetNext(facepos);
			Eigen::Vector3d norface;
			face->GetNormal(norface(0), norface(1), norface(2));
			facenum++;
			norsum = norsum + norface;
		}
		norsum = norsum / (double)facenum;
		//std::cout << "norsum=" << norsum << std::endl;
		//if(norsum(2)>=0)std::cout<<"norsumErr"<< norsum<<std::endl;
		Eigen::Vector3d pos1,pos2;
		node->GetCoord3D(pos1);
		if (refedge->GetStartPoint() != node)  refedge->GetStartPoint()->GetCoord3D(pos2);
		else  refedge->GetEndPoint()->GetCoord3D(pos2);

		Eigen::Vector3d edgevec = pos2 - pos1;
		edgevec = edgevec.normalized();
		Eigen::Vector3d guidevec = node->nodegra2d.normalized();
		//double dot = edgevec.dot(guidevec);
		//double cross = edgevec(0) * guidevec(1) - edgevec(1) * guidevec(0);
		double angle = signed_angle_between_vectors(edgevec, guidevec, norsum);
		//double cross = edgevec(0) * guidevec(1) - edgevec(1) * guidevec(0);
		//double dot = edgevec.dot(guidevec);
		//double angle2 = std::atan2(cross, dot);
		//if (fabs(angle - angle2) > 0.01) {
		//	std::cout << "angle=" << angle << ",angle2=" << angle2 << std::endl; errorcount++;
		//}
		//else {
		//	std::cout << "angle=" << angle << ",angle2=" << angle2 << std::endl;
		//}
		std::complex<double> complexedge;
		complexedge.real(cos(angle)); 
		complexedge.imag(sin(angle));
		//std::cout << "rotate=" << rotateVector3D(edgevec, nordir, angle).dot(guidevec) << std::endl;
		guide_complex.at(node->GetIndexNo())=complexedge;
	}
//	std::cout << "errorcount=" << errorcount << "nodenumer=" << surfaceMesh->GetNodeNumber() << std::endl;
}

void stripe_Pattern::require_transport_vector() {//transport from startnode to endnode,inverse transport the angle is opposite

	for (GLKPOSITION edgepos = surfaceMesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
	{
		QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
		Eigen::Vector3d edgevec, edgevecopp, normaldir(0,0,1);
	/*	edge->GetStartPoint()->GetCoord3D(pos1);
		edge->GetEndPoint()->GetCoord3D(pos2);
		edgevec = pos2 - pos1;
		edgevec.normalize();
		edgevecopp = -edgevec;
		edgevecopp.normalize();*/

		QMeshEdge* startrefedge = refedge_ptr[edge->GetStartPoint()->GetIndexNo()];
		QMeshEdge* endrefedge = refedge_ptr[edge->GetEndPoint()->GetIndexNo()];

 
		double thetaij = sumpofangle(edge->GetStartPoint(), startrefedge, edge)/nodesumangle[edge->GetStartPoint()->GetIndexNo()];
		double thetaji = sumpofangle(edge->GetEndPoint(), endrefedge, edge)/ nodesumangle[edge->GetEndPoint()->GetIndexNo()];

		Eigen::Vector3d posnode, pos1, pos2, vec1, vec2;
		edge->GetStartPoint()->GetCoord3D(posnode);
		if (edge->GetStartPoint() != startrefedge->GetStartPoint()) startrefedge->GetStartPoint()->GetCoord3D(pos1);
		else startrefedge->GetEndPoint()->GetCoord3D(pos1);
		edge->GetEndPoint()->GetCoord3D(pos2);
		vec1 = pos1 - posnode;
		vec2 = pos2 - posnode;
 		double dot = vec1.x() * vec2.x() + vec1.y() * vec2.y();
		double cross = vec1.x() * vec2.y() - vec1.y() * vec2.x();
		double theta = std::atan2(cross,dot);
		if (theta < 0) {
			theta += 2 * M_PI;
		}
		//if (fabs(theta - thetaij) > 0.01)std::cout << "thetaij=" << thetaij << ",theta=" << theta << std::endl;
		//std::cout << "thetaij=" << thetaij << ",thetaji=" << thetaji << std::endl;
		//if (thetaij > 2 * M_PI || thetaji > 2 * M_PI) std::cout << "angle >2M_PI" << std::endl;
		//std::cout <<"rotate" << rotateVector3D(refstart, normaldir, thetaij).dot(edgevec) << "," << rotateVector3D(refend, normaldir, thetaji).dot(edgevecopp) << std::endl;
		double transportij = -thetaij + thetaji + M_PI;
		//std::cout <<"transportij=" << transportij << std::endl;
		std::complex<double> complextranspot=from_angle(transportij);
		transport_complex.at(edge->GetIndexNo())=complextranspot;
 	}
}

void stripe_Pattern::require_edge_omega(std::vector<double>& frequencies){
	for (GLKPOSITION edgepos = surfaceMesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
	{
		QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
		std::complex<double> complex_xi, complex_xj,complex_rij;
		complex_xi = guide_complex.at(edge->GetStartPoint()->GetIndexNo());
		complex_xj = guide_complex.at(edge->GetEndPoint()->GetIndexNo());
		complex_rij = transport_complex.at(edge->GetIndexNo());
		//double dot = (complex_rij * complex_xi).real() * complex_xj.real() + (complex_rij * complex_xi).imag() * complex_xj.imag();
	//	std::cout << "dot=" << dot << std::endl;
		double angle_xi = std::arg(complex_xi);
		double angle_xj = std::arg(complex_xj);
		double angle_rij = std::arg(complex_rij);
		double dot = angle_xj - (angle_xi + angle_rij);

		while (dot > M_PI) dot -= 2 * M_PI;
		while (dot <= -M_PI) dot += 2 * M_PI;

		double s;
		if (dot<0.5*M_PI&&dot>=-0.5*M_PI)
		{
			s = 1;
		}
		else
		{
			//std::cout << "flip" << dot << std::endl;
			s = -1;
		}
		edge_cross_sheet.at(edge->GetIndexNo()) = s < 0;
		edge->CalLength();
		double lij = edge->GetLength();
		double phi_i = std::arg(complex_xi);
		double phi_j = std::arg(s*complex_xj);
		QMeshNode* snode = edge->GetStartPoint();
		QMeshNode* enode = edge->GetEndPoint();
		Eigen::Vector3d pos1, pos2, vecedge,refstart;
		snode->GetCoord3D(pos1);
		enode->GetCoord3D(pos2);
		QMeshEdge* refedge_start = refedge_ptr[snode->GetIndexNo()];
		double theta_i =  sumpofangle(edge->GetStartPoint(), refedge_start, edge) / nodesumangle[edge->GetStartPoint()->GetIndexNo()];
 		double theta_j = theta_i + std::arg(complex_rij);
		double omega_ij = (lij / 2.0) * ((frequencies.at(edge->GetStartPoint()->GetIndexNo()) * cos(phi_i - theta_i))+
		(frequencies.at(edge->GetEndPoint()->GetIndexNo()))*cos(phi_j-theta_j));
		edge_omega.at(edge->GetIndexNo())=omega_ij;
	}
}

void stripe_Pattern::judgeanticlockwise() {
	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);

		QMeshEdge* firstedge = face->GetEdgeRecordPtr(1);
		QMeshEdge* secondedge;
		QMeshEdge* thirdedge;
		QMeshNode* enode = firstedge->GetEndPoint();
		for (GLKPOSITION edgepos = enode->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
		{
			QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
			if (edge == firstedge)continue;
			if (edge->GetLeftFace() == face || edge->GetRightFace() == face) {
				secondedge = edge; break;
			}
		}
		for (int i = 2; i <= 3; i++)
		{
			if (face->GetEdgeRecordPtr(i) != firstedge && face->GetEdgeRecordPtr(i) != secondedge) {
				thirdedge = face->GetEdgeRecordPtr(i); 
				break;
			}
		}

		Eigen::Vector3d pos1, pos2,pos3,vec1,vec2,cross_product,nordir;
		firstedge->GetStartPoint()->GetCoord3D(pos1);
		firstedge->GetEndPoint()->GetCoord3D(pos2);
		if (secondedge->GetStartPoint() == firstedge->GetEndPoint()) {
			secondedge->GetEndPoint()->GetCoord3D(pos3);
		}
		else
		{
			secondedge->GetStartPoint()->GetCoord3D(pos3);
		}
		if (secondedge->GetStartPoint() != firstedge->GetEndPoint() && secondedge->GetEndPoint() != firstedge->GetEndPoint()) {
			//std::cout << "second edge err" << std::endl;
		}
		vec1 = pos2 - pos1; 
		vec2 = pos3 - pos2; 		
		if (vec1.norm() < 0.001 || vec2.norm() < 0.001) {
		//	std::cout << "vec1||vec2,zero" << std::endl;
		}
		vec1.normalize();
		vec2.normalize();
		cross_product = vec1.cross(vec2);
 
		nordir = { 0,0,1 };
		double aligment = cross_product.normalized().dot(nordir.normalized());
		//std::cout << "alig" << aligment << std::endl;
		if (aligment < 0) {
			QMeshEdge* tempedge;
			tempedge = secondedge;
			secondedge = thirdedge;
			thirdedge = tempedge;
		}
		std::array<int, 3> edgeindex = { firstedge->GetIndexNo(),secondedge->GetIndexNo(),thirdedge->GetIndexNo() };
		//std::cout << "edgeindex=" << edgeindex[0]<<","<< edgeindex[1]<<","<< edgeindex[2] << std::endl;
		edgeindexofface[face->GetIndexNo()]= edgeindex;
		QMeshNode* node1;
		QMeshNode* node2;
		QMeshNode* node3;
		if (aligment > 0) {
			node1 = firstedge->GetStartPoint();
			node2 = firstedge->GetEndPoint();
		}
		else
		{
			node1 = firstedge->GetEndPoint();
			node2 = firstedge->GetStartPoint();
		}
		for (int i = 0; i < 3; i++)
		{
			if (face->GetNodeRecordPtr(i) != node1 && face->GetNodeRecordPtr(i) != node2) {
				node3 = face->GetNodeRecordPtr(i); break;
			}
		}
		double ali1,ali2, ali3;
		if (node1 == firstedge->GetStartPoint())ali1 = 1;  else ali1 = -1;
		if (node2 == secondedge->GetStartPoint()) ali2 = 1; else ali2 = -1;
		if (node3 == thirdedge->GetStartPoint()) ali3 = 1; else ali3 = -1;

		if (node3 != thirdedge->GetStartPoint() && node3 != thirdedge->GetEndPoint()) {
		//	std::cout << "node3errrrrrrrrrrr" << std::endl;
		}
		std::array<double, 3> ali = { ali1,ali2,ali3 };
		anticolckjudge[face->GetIndexNo()]=ali;
	}

	for (GLKPOSITION edgepos = surfaceMesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
	{
		QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
		if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) continue;
		QMeshFace* lface = edge->GetLeftFace();
		QMeshFace* rface = edge->GetRightFace();
		int lindex, rindex;
		for (int i = 0; i < 3; i++)
		{
			if (edge == surfaceMesh->GetEdgeRecordPtr(edgeindexofface[lface->GetIndexNo()][i] + 1)) {
				lindex = i;
			}
			if (edge == surfaceMesh->GetEdgeRecordPtr(edgeindexofface[rface->GetIndexNo()][i] + 1)) {
				rindex = i;
			}
		}
		double li = anticolckjudge[lface->GetIndexNo()].at(lindex);
		double ri = anticolckjudge[rface->GetIndexNo()].at(rindex);
 		if ((li*ri ) !=-1) {
			//std::cout << "errrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr" << std::endl;
		}
	}
}

double stripe_Pattern::computeEdgeAngle
(QMeshNode* Node, QMeshEdge* connectEdge) const {
	Eigen::Vector3d p0, p1, p2, v1, v2;
	double cotA, cotB;
	 
	QMeshNode* Node1 = connectEdge->GetEndPoint(); if (Node == Node1) Node1 = connectEdge->GetStartPoint();

	if (connectEdge->GetLeftFace() != nullptr) {
		QMeshFace* LeftFace = connectEdge->GetLeftFace();
		QMeshNode* LFNode = LeftFace->GetNodeRecordPtr(0);

		if (LFNode == Node1 || LFNode == Node) LFNode = LeftFace->GetNodeRecordPtr(1);
		if (LFNode == Node1 || LFNode == Node) LFNode = LeftFace->GetNodeRecordPtr(2);
		//if (Node->GetIndexNo() == Node1->GetIndexNo() || Node->GetIndexNo() == LFNode->GetIndexNo() || Node1->GetIndexNo() == LFNode->GetIndexNo())
		Node->GetCoord3D(p0(0), p0(1), p0(2));
		Node1->GetCoord3D(p1(0), p1(1), p1(2));
		LFNode->GetCoord3D(p2(0), p2(1), p2(2));
		v1 = p2 - p1;
		v2 = p2 - p0;
		cotA = (v1.dot(v2) / v1.cross(v2).norm());
	}
	else cotA = 0;

	if (connectEdge->GetRightFace() != nullptr) {
		QMeshFace* RightFace = connectEdge->GetRightFace();
		QMeshNode* RFNode = RightFace->GetNodeRecordPtr(0);

		if (RFNode == Node1 || RFNode == Node) RFNode = RightFace->GetNodeRecordPtr(1);
		if (RFNode == Node1 || RFNode == Node) RFNode = RightFace->GetNodeRecordPtr(2);
		//if (Node->GetIndexNo() == Node1->GetIndexNo() || Node->GetIndexNo() == RFNode->GetIndexNo() || Node1->GetIndexNo() == RFNode->GetIndexNo())
		Node->GetCoord3D(p0(0), p0(1), p0(2));
		Node1->GetCoord3D(p1(0), p1(1), p1(2));
		RFNode->GetCoord3D(p2(0), p2(1), p2(2));
		v1 = p2 - p1;
		v2 = p2 - p0;
		cotB =( v1.dot(v2) / v1.cross(v2).norm());
	}
	else cotB = 0;
	//cout << "cotA = " << cotA << ", cotB = " << cotB << endl;
	return(0.5 * (cotA + cotB));
}

void stripe_Pattern::require_cotanweighting() {
	for (GLKPOSITION edgepos = surfaceMesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
	{
		QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
		cotanweight.at(edge->GetIndexNo()) = computeEdgeAngle(edge->GetStartPoint(), edge);
	}
}
void stripe_Pattern::require_vertex_energy_matrix() {

	require_cotanweighting();

	matA.resize(2 * surfaceMesh->GetNodeNumber(), 2 * surfaceMesh->GetNodeNumber());

	std::vector<Eigen::Triplet<double>> triplets;
	triplets.reserve(12 * surfaceMesh->GetEdgeNumber());
	for (GLKPOSITION edgepos = surfaceMesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
	{
		QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
		double omega_ij = edge_omega.at(edge->GetIndexNo());
		double w = cotanweight.at(edge->GetIndexNo());
	//	std::cout << "w=" << w << std::endl;
		int i = 2*edge->GetStartPoint()->GetIndexNo();
		int j = 2* edge->GetEndPoint()->GetIndexNo();


		triplets.emplace_back(i + 0, i + 0, w);
		triplets.emplace_back(i + 1, i + 1, w);

		triplets.emplace_back(j + 0, j + 0, w);
		triplets.emplace_back(j + 1, j + 1, w);

		std::complex<double> rij = w * from_angle(omega_ij);
	//	std::cout << "omega_ij=" << rij << std::endl;

		triplets.emplace_back(i + 0, j + 0, -rij.real());
		triplets.emplace_back(i + 1, j + 0, rij.imag());

		triplets.emplace_back(j + 0, i + 0, -rij.real());
		triplets.emplace_back(j + 0, i + 1, rij.imag());

		if (edge_cross_sheet.at(edge->GetIndexNo())) {
			rij *= -1;// std::cout << "flip" << std::endl;
		}
		triplets.emplace_back(i + 0, j + 1, -rij.imag());
		triplets.emplace_back(i + 1, j + 1, -rij.real());

		triplets.emplace_back(j + 1, i + 0, -rij.imag());
		triplets.emplace_back(j + 1, i + 1, -rij.real());
	}
	matA.setFromTriplets(triplets.begin(), triplets.end());
	Eigen::SparseMatrix<double> eye;
	eye.resize(2 * surfaceMesh->GetNodeNumber(), 2 * surfaceMesh->GetNodeNumber());
	eye.setIdentity();
	eye = eye * 1e-4;
	matA += eye;
	for (GLKPOSITION edgepos = surfaceMesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
	{
		QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
		//double omega_ij = edge_omega.at(edge->GetIndexNo());
		//double w = cotanweight.at(edge->GetIndexNo());
		int i = 2*edge->GetStartPoint()->GetIndexNo();
		int j = 2*edge->GetEndPoint()->GetIndexNo();
	//	if (abs(matA.coeff(i, i) )< 0.001 || abs(matA.coeff(j, j)) < 0.001) std::cout << "Mataerr" << std::endl;
	}
	//std::cout <<"rank" << computeRank(matA)<<"rows="<<2*surfaceMesh->GetNodeNumber() << std::endl;
}



double stripe_Pattern::computearea(QMeshNode*node) {
	double average = 0.0;
	for (GLKPOSITION facepos = node->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)node->GetFaceList().GetNext(facepos);
		average += face->GetArea();
	}
	return average / static_cast<double>(node->GetFaceNumber());
	//return average / 3.0;

}

void stripe_Pattern::require_mass_matrix() {
	std::vector<Eigen::Triplet<double>> triplets;
	for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
	{
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
		/*triplets.emplace_back(2 * node->GetIndexNo(), 2 * node->GetIndexNo(), computearea(node));
		triplets.emplace_back(2 * node->GetIndexNo() + 1, 2 * node->GetIndexNo() + 1, computearea(node));*/
		triplets.emplace_back(2 * node->GetIndexNo(), 2 * node->GetIndexNo(), 1);
		triplets.emplace_back(2 * node->GetIndexNo() + 1, 2 * node->GetIndexNo() + 1, 1);
	//	if (computearea(node) < 0.01) std::cout << "errr" << computearea(node) << std::endl;
	}
	matB.resize(2 * surfaceMesh->GetNodeNumber(), 2 * surfaceMesh->GetNodeNumber());
	matB.setFromTriplets(triplets.begin(), triplets.end());
}
void stripe_Pattern::computeerror() {

	double errsum=0;
	for (GLKPOSITION edgepos = surfaceMesh->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
	{
		QMeshEdge* edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(edgepos);
		std::complex<double>complex_s(solutioncomplex(edge->GetStartPoint()->GetIndexNo() * 2), solutioncomplex(edge->GetStartPoint()->GetIndexNo() * 2+1));
		std::complex<double>complex_e(solutioncomplex(edge->GetEndPoint()->GetIndexNo() * 2), solutioncomplex(edge->GetEndPoint()->GetIndexNo() * 2 + 1));
		std::complex<double>omageij = from_angle(edge_omega.at(edge->GetIndexNo()));
		double mag_s = std::abs(complex_s);
		double mag_e = std::abs(complex_e);
		complex_e /= mag_e;
		complex_s /= mag_s;
		std::complex<double>err_complex = (complex_e - omageij * complex_s);
		errsum = errsum + std::abs(err_complex * std::conj(err_complex));
	//	std::cout << "complex_s="<< complex_s<<"complex_e="<< complex_e << "err_complex" << err_complex * std::conj(err_complex) << std::endl;
	}
	//std::cout << "sumerr" << errsum<<std::endl;
}

void stripe_Pattern::require_paramaterization() {
	std::vector<double> frequencies;
	frequencies.resize(surfaceMesh->GetNodeNumber());
	for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
	{
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
		frequencies[node->GetIndexNo()] = pattern_frequency;
	}

	find_referenceedge();
	require_guide_complex();
	require_transport_vector();
	require_edge_omega(frequencies);
	require_cotanweighting();
	require_vertex_energy_matrix();
	require_mass_matrix();

	solutioncomplex.resize(surfaceMesh->GetNodeNumber() * 2);

	smallest_eig_positive_definite(matA, matB, solutioncomplex, 100);
	//*cos,sin//
	judgeanticlockwise();

	/*std::string output_path = "D:/File/CUHK/LAB/TTA/TTA/SpatialFiberToolpathGenerator-main/model/isoSurface/Geoditance for TTA2/scalar" + std::to_string(surfaceMesh->GetIndexNo()) + ".txt";
	std::ofstream fout(output_path);
	if (!fout) {
		std::cerr << "Cannot open file for writing: " << output_path << std::endl;
		return;
	}
	for (GLKPOSITION nodepos = surfaceMesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
	{
		QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(nodepos);
		double nodeangle = atan2(solutioncomplex(2*node->GetIndexNo() + 1), solutioncomplex(2*node->GetIndexNo()));
		//std::cout << "nodeangle" << nodeangle << std::endl;
		node->scalarField = cos(nodeangle);
		fout << node->scalarField << std::endl;

	}*/

 
	//computeerror();
}

void stripe_Pattern::require_Coordinates() {

	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		QMeshNode* snode;
		QMeshEdge* sedge = surfaceMesh->GetEdgeRecordPtr(edgeindexofface[face->GetIndexNo()][0] + 1);

		if (anticolckjudge[face->GetIndexNo()][0] > 0) snode = sedge->GetStartPoint();
		else snode = sedge->GetEndPoint();

		std::complex<double> corner_coor = tounit({ solutioncomplex(2 * snode->GetIndexNo()), solutioncomplex(2 * snode->GetIndexNo() + 1) });
		coor_corner[face->GetIndexNo()][0] = std::arg(corner_coor);
	//	std::cout<<face->GetIndexNo()<<"," << snode->GetIndexNo() << "," << "coor_corner0=" << std::arg(corner_coor) << std::endl;
		for (int i = 0; i < 3; i++)
		{
			QMeshEdge*edge = surfaceMesh->GetEdgeRecordPtr(edgeindexofface[face->GetIndexNo()][i] + 1);
			QMeshEdge* nextedge = surfaceMesh->GetEdgeRecordPtr(edgeindexofface[face->GetIndexNo()][(i+1)%3] + 1);

			std::complex<double> psi_i; //(solutioncomplex(2 * edge->GetStartPoint()->GetIndexNo()), solutioncomplex(2 * edge->GetStartPoint()->GetIndexNo() + 1));
			std::complex<double>psi_j; //(solutioncomplex(2 * edge->GetEndPoint()->GetIndexNo()), solutioncomplex(2 * edge->GetEndPoint()->GetIndexNo() + 1));
			if (anticolckjudge[face->GetIndexNo()][i] > 0) {
				psi_i = tounit({ solutioncomplex(2 * edge->GetStartPoint()->GetIndexNo()), solutioncomplex(2 * edge->GetStartPoint()->GetIndexNo() + 1) });
				psi_j = tounit({ solutioncomplex(2 * edge->GetEndPoint()->GetIndexNo()), solutioncomplex(2 * edge->GetEndPoint()->GetIndexNo() + 1)});
			}
			else
			{
				psi_i = tounit({ solutioncomplex(2 * edge->GetEndPoint()->GetIndexNo()), solutioncomplex(2 * edge->GetEndPoint()->GetIndexNo() + 1) });
				psi_j = tounit({ solutioncomplex(2 * edge->GetStartPoint()->GetIndexNo()), solutioncomplex(2 * edge->GetStartPoint()->GetIndexNo() + 1) });
			}

			double cij = anticolckjudge[face->GetIndexNo()][i];
			double omega_ij = cij * edge_omega[edge->GetIndexNo()];
			auto crosses_sheets = [&](QMeshEdge* taredge,double s) {
				QMeshEdge* hedge = surfaceMesh->GetEdgeRecordPtr(edgeindexofface[face->GetIndexNo()][0] + 1);
				std::complex<double>xi;
				std::complex<double>xj;
				if (anticolckjudge[face->GetIndexNo()][0] > 0) {
					xi= guide_complex[hedge->GetStartPoint()->GetIndexNo()];
				}
				else {
					xi = guide_complex[hedge->GetEndPoint()->GetIndexNo()];
				}

				if (s > 0) {
					xj = guide_complex[taredge->GetStartPoint()->GetIndexNo()];
				}
				else
				{
					xj = guide_complex[taredge->GetEndPoint()->GetIndexNo()];
				}
				int i = 0;
				while (hedge!=taredge)
				{
					std::complex<double> r = transport_complex[hedge->GetIndexNo()];
					
					if (anticolckjudge[face->GetIndexNo()][i] < 0) {
						r = std::conj(r);
					}
					xi = r * xi;
					i++;
					hedge = surfaceMesh->GetEdgeRecordPtr(edgeindexofface[face->GetIndexNo()][i] + 1);
				}
				double dot = xi.real() * xj.real() + xi.imag() * xj.imag();
				//if (dot <= 0) std::cout << "flip" << std::endl;
				return  dot <= 0; 
			};
			if (crosses_sheets(edge, anticolckjudge[face->GetIndexNo()][i])) {
				psi_i = std::conj(psi_i);
				omega_ij *= -cij;
		/*		psi_i = std::conj(psi_i);
				omega_ij *= cij;*/
			}
			if (crosses_sheets(nextedge, anticolckjudge[face->GetIndexNo()][(i + 1) % 3])) {
				psi_j = std::conj(psi_j);
				omega_ij *= cij;
				//psi_j = std::conj(psi_j);
				//omega_ij *= -cij;
			}
			std::complex<double> rij = from_angle(omega_ij);
			if (nextedge != sedge) {
				QMeshNode* nextnode;
				if (anticolckjudge[face->GetIndexNo()][(i + 1) % 3] > 0) {
					nextnode = nextedge->GetStartPoint();
				}
				else {
					nextnode = nextedge->GetEndPoint();
				}
				coor_corner[face->GetIndexNo()][(i + 1) % 3] = coor_corner[face->GetIndexNo()][i] + omega_ij - std::arg(rij * psi_i / psi_j);
			}
			else
			{
				QMeshNode* nextnode;
				if (anticolckjudge[face->GetIndexNo()][(i + 1) % 3] > 0) {
					nextnode = nextedge->GetStartPoint();
				}
				else {
					nextnode = nextedge->GetEndPoint();
				}
				double alpha= coor_corner[face->GetIndexNo()][i] + omega_ij - std::arg(rij * psi_i / psi_j);
				facepara[face->GetIndexNo()]= static_cast<int>(std::round(
					(alpha - coor_corner[face->GetIndexNo()][(i + 1) % 3]) / (2.0 * M_PI)));//旋转一圈后与原始值做差
 			//	if (facepara[face->GetIndexNo()] == 0) {
				//	//std::cout<<"nosignular_para=" << (alpha - coor_corner[face->GetIndexNo()][(i + 1) % 3]) / (2.0 * M_PI) << std::endl;
				//}
				//else
				//{
				//	//std::cout << "para=" << (alpha - coor_corner[face->GetIndexNo()][(i + 1) % 3]) / (2.0 * M_PI) << std::endl;
				//}
			}
		}
	}
	 
	//for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
	//{
	//	//std::cout << "face" << std::endl;
	//	QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
	//	for (int i = 0; i < 3; i++)
	//	{
	//		double alpha = coor_corner[face->GetIndexNo()][i];
	//		if (anticolckjudge[face->GetIndexNo()][i] > 0) {
	//			QMeshEdge* edge = surfaceMesh->GetEdgeRecordPtr(edgeindexofface[face->GetIndexNo()][i] + 1);
 //				edge->GetStartPoint()->geoFieldValue = cos(alpha);
	//		}
	//		else
	//		{
	//			QMeshEdge* edge = surfaceMesh->GetEdgeRecordPtr(edgeindexofface[face->GetIndexNo()][i] + 1);
 //				edge->GetEndPoint()->geoFieldValue = cos(alpha);
	//		}
	//	}
	//}
}

std::complex<double> stripe_Pattern::tounit(const std::complex<double>& z)
{
	double magnitude = std::abs(z); // Compute the magnitude of the complex number
	if (magnitude == 0.0) {
		// Avoid division by zero; return a zero complex number if input is zero
		return std::complex<double>(0.0, 0.0);
	}
	return z / magnitude; // Normalize the complex number
}

void stripe_Pattern::connect_isolines_on_singularities() {

	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		if (facepara[face->GetIndexNo()] == 0) continue;

		std::vector<std::vector<int>> edge_indeces(3);
		std::vector<std::vector<Eigen::Vector3d>> face_points;

		int idx = 0;
		Eigen::Vector3d facevec;
		facevec.setZero();
		for (int i = 0; i < 3; i++)
		{
			facevec = facevec + face->GetNodeRecordPtr(i)->nodegra2d;
		}
		facevec = facevec / 3.0;

		for (int i = 0; i < 3; i++)
		{
			double corner_i = coor_corner[face->GetIndexNo()][i];
			double corner_j = coor_corner[face->GetIndexNo()][(i + 1) % 3];
			// std::cout << "corner_i=" << corner_i << "," << "corner_j=" << corner_j << std::endl;

			std::vector<double>iso_points_pos;
			if (i == 2 ) {//?
				iso_points_pos = crossings_mod_2pi(corner_i, corner_j + 2* facepara[face->GetIndexNo()] * M_PI);
			}
			else
			{
				iso_points_pos = crossings_mod_2pi(corner_i, corner_j);
			}

			std::vector<int> current_edge_stripe_index;
			if (edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]].size() == 0) {
				for (int j = 0; j < iso_points_pos.size(); j++)
				{
					current_edge_stripe_index.emplace_back(stripe_points_pos.size());
					stripe_points_edge_orientation[stripe_points_pos.size()] = anticolckjudge[face->GetIndexNo()][i];
					stripe_points_edge_index.push_back(edgeindexofface[face->GetIndexNo()][i]);
					stripe_points_pos.push_back(iso_points_pos[j]);
					edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]] = current_edge_stripe_index;
				}
				edge_indeces[i] = current_edge_stripe_index;
			}
			else
			{
				//the direction is inverse
				for (int k = edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]].size()-1; k >=0 ; k--)/*error*/
				{
					edge_indeces[i].push_back(edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]][k]);
				}
				if (iso_points_pos.size() != edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]].size()) {
				//	std::cout << "errrrrrrrrrrrrrrrr" << iso_points_pos.size()<<","<< edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]].size()<< std::endl;
					//std::cout << corner_i << ",,,,," << corner_j + 2.0 * facepara[face->GetIndexNo()] * M_PI << std::endl;
					//std::cout << corner_i << ",,,,," << corner_j<< std::endl;
					QMeshEdge* newedge = surfaceMesh->GetEdgeRecordPtr(edgeindexofface[face->GetIndexNo()][i] + 1);
					newedge->GetLeftFace()->errorface = true;
					newedge->GetRightFace()->errorface = true;
				}
			}
		}
		/*generate the points on each face*/
		for (int i = 0; i < 3; i++)
		{
			QMeshEdge* edge = surfaceMesh->GetEdgeRecordPtr(edgeindexofface[face->GetIndexNo()][i] + 1);
			std::vector<Eigen::Vector3d> edgepoints;
			for (int j = 0; j < edge_indeces[i].size(); j++)
			{
				QMeshNode* snode;
				QMeshNode* enode;
				if (stripe_points_edge_orientation[edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]][j]] > 0) {
					snode = edge->GetStartPoint();
					enode = edge->GetEndPoint();
				}
				else
				{
					snode = edge->GetEndPoint();
					enode = edge->GetStartPoint();
				}
				double strippos = stripe_points_pos[edge_indeces[i][j]];
				Eigen::Vector3d s_pos, n_pos, pos;
				snode->GetCoord3D(s_pos);
				enode->GetCoord3D(n_pos);
				pos = strippos * s_pos + (1 - strippos) * n_pos;
				edgepoints.push_back(pos);
			}
			face_points.push_back(edgepoints);
		}
		/*remove the crossing according to the direction field*/
		std::vector<std::pair<double, std::array<int, 2>>> min_values;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < face_points[i].size(); j++)
			{
				Eigen::Vector3d v = face_points[i][j];
				double min = find_min(face_points, v, facevec, i);
				min_values.push_back({min, { i,j }});
			}
		}
	
		int n = abs(facepara[face->GetIndexNo()]);
 
		std::vector<std::vector<int>> crossing_indices = remove_crossings(min_values, n, edge_indeces);
	//	std::cout << "crossing_indices_size=" << crossing_indices[0].size() <<","<< crossing_indices[1].size()<<","<< crossing_indices[2].size() << std::endl;
		/*set matchings*/

		if ((crossing_indices[0].size() + crossing_indices[1].size() + crossing_indices[2].size()) %2!=0) {
			face->errorface = true;
 		}
		int maxval= std::max(crossing_indices[0].size(), std::max(crossing_indices[1].size(), crossing_indices[2].size()));
		if ((crossing_indices[0].size() + crossing_indices[1].size() + crossing_indices[2].size() - 2*maxval)<0) {
			face->errorface = true;
 		}

		std::vector<std::array<int, 2>> matchings = match_crossings(crossing_indices);
	//	std::cout << "matching_size=" << matchings.size() << std::endl;
		matching_information[face->GetIndexNo()] = matchings;
	}
}

void stripe_Pattern::require_Polylines(std::vector<double>& output_stripe_points_pos, std::vector<int>& output_stripe_points_edge_index,
	std::vector<int>& output_stripe_points_edge_orientation, std::vector<std::vector<int>>& output_edgepolylines_index, std::vector<std::vector<std::array<int, 2>>>& output_matching_information) {

	for (GLKPOSITION facepos = surfaceMesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(facepos);
		if (facepara[face->GetIndexNo()] != 0) { continue; }
		std::vector<std::vector<int>> edge_indeces(3);
		int idx = 0;
		for (int i = 0; i < 3; i++)
		{
			double corner_i = coor_corner[face->GetIndexNo()][i];
			double corner_j = coor_corner[face->GetIndexNo()][(i + 1) % 3];
			 //std::cout << "corner_i=" << corner_i << "," << "corner_j=" << corner_j << std::endl;
			std::vector<double>iso_points_pos = crossings_mod_2pi(corner_i, corner_j);
			std::vector<int> current_edge_stripe_index;
			if (edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]].size() == 0) {
				for (int j = 0; j < iso_points_pos.size(); j++)
				{
					current_edge_stripe_index.emplace_back(stripe_points_pos.size());
					stripe_points_edge_orientation[stripe_points_pos.size()]=anticolckjudge[face->GetIndexNo()][i];
					stripe_points_edge_index.emplace_back(edgeindexofface[face->GetIndexNo()][i]);
					stripe_points_pos.emplace_back(iso_points_pos[j]);
				}
				if (current_edge_stripe_index.size() != 0) {
				edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]] = current_edge_stripe_index;
				}
				edge_indeces[i] = current_edge_stripe_index;
			}
			else {
				for (int k = edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]].size() - 1; k >= 0; k--)
				//for (int k =0; k < edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]].size() ; k++)
 				{
					edge_indeces[i].push_back(edgepolylines_index[edgeindexofface[face->GetIndexNo()][i]][k]);
				}
			}
		}

		if ((edge_indeces[0].size() + edge_indeces[1].size() + edge_indeces[2].size()) % 2 != 0) {
			face->errorface = true;
		}
		int maxval = std::max(edge_indeces[0].size(), std::max(edge_indeces[1].size(), edge_indeces[2].size()));
		if ((edge_indeces[0].size() + edge_indeces[1].size() + edge_indeces[2].size() - 2 * maxval) < 0) {
			face->errorface = true;
		}

		std::vector<std::array<int, 2>> matchings = match_crossings(edge_indeces);
		matching_information[face->GetIndexNo()] = matchings;
	}

	connect_isolines_on_singularities();

	//std::cout << "size=" << stripe_points_pos.size() << std::endl;


	output_stripe_points_pos.resize(stripe_points_pos.size());
	output_stripe_points_edge_index.resize(stripe_points_edge_index.size());
	output_stripe_points_edge_orientation.resize(stripe_points_edge_orientation.size());
	output_edgepolylines_index.resize(edgepolylines_index.size());
	output_matching_information.resize(matching_information.size());

	output_stripe_points_pos = stripe_points_pos;
	output_stripe_points_edge_index = stripe_points_edge_index;
	output_stripe_points_edge_orientation = stripe_points_edge_orientation;
	output_edgepolylines_index = edgepolylines_index;
	output_matching_information = matching_information;
}


std::vector<std::array<int, 2>>stripe_Pattern::match_crossings(const std::vector<std::vector<int>>& crossings) {
	assert(crossings.size() == 3);

	int idx_ij = 2;
	if (crossings[0].size() >= crossings[1].size() && crossings[0].size() >= crossings[2].size()) {
		idx_ij = 0;
	}
	else if (crossings[1].size() >= crossings[2].size() && crossings[1].size() >= crossings[0].size()) {
		idx_ij = 1;
	}
	int idx_jk = (idx_ij + 1) % 3;
	int idx_ki = (idx_ij + 2) % 3;

	const std::vector<int>& ij = crossings[idx_ij];
	const std::vector<int>& jk = crossings[idx_jk];
	const std::vector<int>& ki = crossings[idx_ki];

	auto nij = ij.size();
	auto njk = jk.size();
	auto nki = ki.size();

	assert(nij >= njk && nij >= nki);
	//assert(nij <= njk + nki);
	if (nij > njk + nki) {
		//std::cout << "nij > njk + nki" << std::endl;
	//	assert(nij <= njk + nki);
	}
	std::vector<std::array<int, 2>> matchings;


	if ((nij + njk + nki) % 2 != 0)//odd number
	{
		/*std::cout << "(nij + njk + nki)% 2 != 0" << std::endl;
		std::cout << "nij="<<nij<<"njk="<<njk<<"nki"<<nki<< std::endl;*/

		return matchings;
	}

	if (nij == njk + nki) {
		for (int m = 0; m < nki; ++m) {
			matchings.push_back({ ij[m], ki[nki - m - 1] });
		}
		for (int m = 0; m < njk; ++m) {
			matchings.push_back({ ij[nki + m], jk[njk - m - 1] });
		}
	}
	else {
		int n_remaining_crossings = (nij + njk + nki) / 2;
		int m = 0;
		while (n_remaining_crossings > njk) {
			matchings.push_back({ ij[m], ki[nki - m - 1] });
			++m;
			n_remaining_crossings -= 1;
		}

		int l = 0;
		while (n_remaining_crossings > nki - m) {
			matchings.push_back({ ij[nij - 1 - l], jk[l] });
			++l;
			n_remaining_crossings -= 1;
		}
		int p = 0;
		while (n_remaining_crossings > 0) {
			matchings.push_back({ jk[njk - 1 - p], ki[p] });
			++p;
			n_remaining_crossings -= 1;
		}
	}
	return matchings;
}


std::vector<std::vector<int>> stripe_Pattern::remove_crossings(std::vector<std::pair<double, std::array<int, 2>>>& values, int n,
	std::vector<std::vector<int>>& face_indices) {

	using elem_type = std::pair<double, std::array<int, 2>>;
	std::sort(values.begin(), values.end(), [](elem_type& a, elem_type& b) { return a.first > b.first; });

	std::array<std::vector<bool>, 3> removed;
	for (int i = 0; i < 3; ++i) removed[i] = std::vector<bool>(face_indices[i].size(), false);

	assert(face_indices.size() == 3);
	size_t idx_ij = 0;
	if (face_indices[1].size() >= face_indices[2].size() && face_indices[1].size() >= face_indices[0].size()) {
		idx_ij = 1;
	}
	else if (face_indices[2].size() >= face_indices[1].size() && face_indices[2].size() >= face_indices[0].size()) {
		idx_ij = 2;
	}

	int m = face_indices[idx_ij].size() - face_indices[(idx_ij + 1) % 3].size() - face_indices[(idx_ij + 2) % 3].size();
	//if (m > 0) {
	//std::cout << "m>0" << std::endl;
	//}

	 

	int n_removed = 0; // total number of removed points
	int i = 0;

	while (n_removed < n) {
		std::array<int, 2> indices = values[i].second;
		if (indices[0] == idx_ij) {
			--m;
		}
		else {
			if (n - n_removed <= m) {
				++i;
				continue;
			}
			++m;
		}
		//std::cout << "removedsize" << removed.size() << "indices[0]=" << indices[0] << "removedindices0size" << removed[indices[0]].size() << "index1=" << indices[1] << std::endl;
		removed[indices[0]][indices[1]] = true;
		++n_removed;
		++i;
		if (i == values.size())break;
	}
	//if (m > 0) {
	//	std::cout << "m>0" << std::endl;
	//}
	//assert(m <= 0);
	std::vector<std::vector<int>> crossing_indices(3);
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < face_indices[i].size(); ++j)
			if (!removed[i][j]) crossing_indices[i].push_back(face_indices[i][j]);
	//std::cout << crossing_indices[idx_ij].size()<<"m=====" << m << "," << std::endl;

	return crossing_indices;
}

std::vector<double> stripe_Pattern::crossings_mod_2pi(double val1, double val2) {
		std::vector<double> barys;
	if (val1 == val2) return barys;

	int max_crossings = std::ceil(std::abs(val1 - val2) / (2 * M_PI));
	double phase_intial = 0.0;// (double)(surfaceMesh->GetIndexNo() % 8)* M_PI / 16.0;
	if (val1 < val2) {
		for (int i = 0; i < max_crossings; ++i) {
			int k = std::ceil((val1- phase_intial) / (2 * M_PI)) + i;
			double isoval = 2 * M_PI * k+ phase_intial;

			if (isoval < val2) {
				barys.push_back((isoval - val2) / (val1 - val2));
			}
		}
	}
	else {
		for (int i = max_crossings - 1; i >= 0; --i) {
			int k = std::ceil((val2- phase_intial) / (2 * M_PI)) + i;
			double isoval = 2 * M_PI * k+ phase_intial;

			if (isoval < val1) {
				barys.push_back((isoval - val2) / (val1 - val2));
			}
		}
	}

	return barys;
}

double stripe_Pattern::find_min(const std::vector<std::vector<Eigen::Vector3d>>& points, const Eigen::Vector3d& v, const Eigen::Vector3d& vector,
	size_t ignored_edge) {
	
	double min = std::numeric_limits<double>::max();
	for (size_t i = 0; i < points.size(); ++i) {
		if (i == ignored_edge) continue;
		for (size_t j = 0; j < points[i].size(); ++j) {
			Eigen::Vector3d w = points[i][j];
			if (abs((v - w).dot(vector)) < min) {
				min = abs((v - w).dot(vector));
			}
		}
	}
	return min;
}

double stripe_Pattern::signed_angle_between_vectors(const Eigen::Vector3d& startvec, const Eigen::Vector3d& endvec, const Eigen::Vector3d& normal_dir) {
	Eigen::Vector3d v1 = startvec.normalized();
	Eigen::Vector3d v2 = endvec.normalized();
	Eigen::Vector3d axis = v1.cross(v2);
	double sin_theta = axis.norm();          // always >= 0
	double cos_theta = v1.dot(v2);

	// 使用n判断正负
	double sign = (normal_dir.normalized().dot(axis) >= 0) ? 1.0 : -1.0;
	double angle = std::atan2(sign * sin_theta, cos_theta); // [-pi, pi]
	return angle;
}




double stripe_Pattern::sumpofangle(QMeshNode* node, QMeshEdge* refedge, QMeshEdge* endedge) {
	if (refedge == endedge)return 0;
	double sumangle = 0;
	QMeshFace* sface;
	QMeshFace* seface;
	if(refedge->GetLeftFace()!=NULL)
	 seface = refedge->GetLeftFace();
	else
	{
		seface = refedge->GetRightFace();
	}
	Eigen::Vector3d nodepos3d, pos1, pos2,vecstart,vecend,normaldir;
	node->GetCoord3D(nodepos3d);

	for (GLKPOSITION facepos = node->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)node->GetFaceList().GetNext(facepos);
		face->stripcheck = false;
	}
	for (GLKPOSITION edgepos = node->GetEdgeList().GetHeadPosition(); edgepos != NULL;)
	{
		QMeshEdge* edge = (QMeshEdge*)node->GetEdgeList().GetNext(edgepos);
		edge->stripecheck = false;
	}
	if (refedge->GetStartPoint() != node) refedge->GetStartPoint()->GetCoord3D(pos1);
	else
	{
		refedge->GetEndPoint()->GetCoord3D(pos1);
	}
	vecstart = pos1 - nodepos3d;
	seface->GetNormal(normaldir(0),normaldir(1),normaldir(2));
 	for (int i = 0; i < 3; i++)
	{
		QMeshEdge* edge = seface->GetEdgeRecordPtr(i+1);
		if (edge == refedge)continue;
		if (edge->GetStartPoint() == node || edge->GetEndPoint() == node) {
			
			if (edge->GetStartPoint() != node) edge->GetStartPoint()->GetCoord3D(pos2);
			else edge->GetEndPoint()->GetCoord3D(pos2);
			vecend = pos2 - nodepos3d;
			if (signed_angle_between_vectors(vecstart, vecend, normaldir) > 0) {
				sface = seface;
			}
			else
			{
				sface = refedge->GetRightFace();
			}
			break;
		}
	}

	sface->stripcheck = true;
	refedge->stripecheck = true;
	QMeshEdge* sedge = refedge;
	while (true)
	{
		bool updateflag = false;
		for (int i = 0; i < 3; i++)
		{
			QMeshEdge* edge = sface->GetEdgeRecordPtr(i + 1);
			sface->GetNormal(normaldir(0), normaldir(1), normaldir(2));
			if (edge->stripecheck)continue;
			if (edge->GetStartPoint() == node || edge->GetEndPoint() == node) {
				if (edge->GetStartPoint() != node) edge->GetStartPoint()->GetCoord3D(pos2);
				else edge->GetEndPoint()->GetCoord3D(pos2);
				vecend = pos2 - nodepos3d;
				double anglevalue = signed_angle_between_vectors(vecstart, vecend, normaldir);
				if (anglevalue < 0) {
					std::cout << "anglevalue_errrrrrrrrr!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
				}
				else
				{
					sumangle += anglevalue; 
					edge->stripecheck = true;
					vecstart = vecend;
					updateflag = true;
					sedge = edge;
					break;
				}
			}
		}
		if (sedge == endedge)break;
		if (!updateflag)break;
		if (sedge->GetLeftFace() == NULL || sedge->GetRightFace() == NULL) break;
		if (sedge->GetLeftFace()->stripcheck) sface = sedge->GetRightFace();
		else sface = sedge->GetLeftFace();
		sface->stripcheck = true;
	}
	return sumangle;
}

double stripe_Pattern::computeallangelofnode(QMeshNode* node) {
	double sumofangle = 0;
	Eigen::Vector3d nordir;
	Eigen::Vector3d node3d,pos3d1,pos3d2,vec1,vec2;
	node->GetCoord3D(node3d);
	for (GLKPOSITION facepos = node->GetFaceList().GetHeadPosition(); facepos != NULL;)
	{
		QMeshFace* face = (QMeshFace*)node->GetFaceList().GetNext(facepos);
		face->GetNormal(nordir(0), nordir(1), nordir(2));
		QMeshEdge* edge1=NULL;
		QMeshEdge* edge2=NULL;
		int count=0;
		for (int i = 0; i < 3; i++)
		{
			QMeshEdge* edge = face->GetEdgeRecordPtr(i + 1);
			if (edge->GetStartPoint() != node && edge->GetEndPoint() != node)continue;
			if (count == 0) {
				edge1 = edge;
				count++;
			}
			else
			{
				edge2 = edge;
				count++;
			}
 		}
		if (edge1->GetStartPoint() != node) edge1->GetStartPoint()->GetCoord3D(pos3d1);
		else edge1->GetEndPoint()->GetCoord3D(pos3d1);
		if (edge2->GetStartPoint() != node) edge2->GetStartPoint()->GetCoord3D(pos3d2);
		else edge2->GetEndPoint()->GetCoord3D(pos3d2);
		vec1 = pos3d1 - node3d;
		vec2 = pos3d2 - node3d;
		sumofangle +=fabs( signed_angle_between_vectors(vec1, vec2, nordir));
	}
	//if (fabs(sumofangle - 2*M_PI) > 0.01&&!node->isBoundaryNode) {
	//	//std::cout << "sum angle error"<< sumofangle << std::endl;
	//}
	if (node->isBoundaryNode)
	{
		sumofangle =   sumofangle / M_PI;
	}
	else
	{
		sumofangle = sumofangle / (2 * M_PI);
	}
	return sumofangle;
}
