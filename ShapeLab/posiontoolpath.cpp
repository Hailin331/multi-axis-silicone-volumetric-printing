#include "posiontoolpath.h"


void posiontoolpath::assignvectortoface() {
	std::vector<Eigen::Vector3d> vectorpos;
	std::vector<Eigen::Vector3d> vectordir;

    std::ifstream infile(vector_path);
    if (!infile.is_open()) {
        std::cerr << "Cannot open file: " << vector_path << std::endl;
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
    for (GLKPOSITION facepos = surfacemesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
    {
        QMeshFace* face = (QMeshFace*)surfacemesh->GetFaceList().GetNext(facepos);
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

void posiontoolpath::determineboundarynode() {

    std::ifstream infile(start_boundary_node);
    if (!infile.is_open()) {
        std::cerr << "Cannot open file: " << vector_path << std::endl;
        return;
    }
     std::string line;
     std::vector<int> boundarynode;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string item;
        while (std::getline(ss, item, ',')) {
            try {
                boundarynode.push_back(std::stoi(item));
            }
            catch (const std::exception& e) {
                std::cerr << "Error parsing int: " << item << std::endl;
            }
        }
    }
    infile.close();
    meshboundarynode = boundarynode;
 }

int posiontoolpath::computesignletoolpath(QMeshPatch* surfaceMesh, double isovalue) {


	int initialsize = reorderednode.size();

	for (int i = 0; i < edgevisit.size(); i++)
	{
 			edgevisit[i] = false;
 	}

	/*construct outterboundary*/
	QMeshEdge* startedge = NULL;
	int k = 0;
    std::vector<QMeshNode*>newline;

    for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
        QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
        double boutvalue_s = Edge->GetStartPoint()->scalarField;
        double boutvalue_e = Edge->GetEndPoint()->scalarField;

        if (((boutvalue_e - isovalue) * (boutvalue_s - isovalue) < 0) && (Edge->isoutterboundary)) {
            startedge = Edge;
            break;
        }
    }
   
 


	double boutvalue_s = startedge->GetStartPoint()->scalarField;
	double boutvalue_e = startedge->GetEndPoint()->scalarField;

	double alpha = (isovalue - boutvalue_s) / (boutvalue_e - boutvalue_s);

	QMeshNode* isonode = new QMeshNode;
	double p1[3], p2[3], p3[3];
	startedge->GetStartPoint()->GetCoord3D(p1);
	startedge->GetEndPoint()->GetCoord3D(p2);
    Eigen::Vector3d facenor1, facenor2,facenorsum;
    double facearea, faceareasum;
    facenor1.setZero(); facenor2.setZero(); facenorsum.setZero();
    if (startedge->GetLeftFace() != NULL) {
        startedge->GetLeftFace()->GetNormal(facenor1(0), facenor1(1), facenor1(2));
        facearea = startedge->GetLeftFace()->CalArea();
        faceareasum += facearea;
        facenorsum = facenorsum + facenor1* facearea;
    }
    if (startedge->GetRightFace() != NULL) {
        startedge->GetRightFace()->GetNormal(facenor2(0), facenor2(1), facenor2(2));
        facearea = startedge->GetLeftFace()->CalArea();
        faceareasum += facearea;
        facenorsum = facenorsum + facenor2* facearea;
    }
    facenorsum = facenorsum / faceareasum;
	for (int j = 0; j < 3; j++) {
		//compute the position for this isonode
		p3[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];
	}

	isonode->SetCoord3D(p3[0], p3[1], p3[2]);
    isonode->SetNormal(facenorsum(0), facenorsum(1), facenorsum(2));
	newline.push_back(isonode);
	edgevisit.at(startedge->GetIndexNo()) = true;

	//QMeshEdge* Initialedge = startedge;
	QMeshFace* nextface = NULL;
	nextface = startedge->GetLeftFace();

	if (startedge->GetLeftFace() == NULL) nextface = startedge->GetRightFace();

	while (true)
	{
		bool findnextedge = false;
		for (int k = 0; k < 3; k++)
		{
			QMeshEdge* edge = nextface->GetEdgeRecordPtr(k + 1);
			if (edgevisit.at(edge->GetIndexNo()))continue;

			double boutvalue_s = edge->GetStartPoint()->scalarField;
			double boutvalue_e = edge->GetEndPoint()->scalarField;

			QMeshNode* snode = edge->GetStartPoint();
			QMeshNode* enode = edge->GetEndPoint();
			if ((boutvalue_s - isovalue) * (boutvalue_e - isovalue) > 0) continue;

			startedge = edge;
			findnextedge = true;
		}

		if (findnextedge == false) break;

		edgevisit.at(startedge->GetIndexNo()) = true;

		QMeshNode* newisonode = new QMeshNode;

		double boutvalue_s = startedge->GetStartPoint()->scalarField;
		double boutvalue_e = startedge->GetEndPoint()->scalarField;

		double alpha = (isovalue - boutvalue_s) / (boutvalue_e - boutvalue_s);

        //Eigen::Vector3d facenor1, facenor2, facenorsum;
         facearea=0, faceareasum=0;
        facenor1.setZero(); facenor2.setZero(); facenorsum.setZero();

        if (startedge->GetLeftFace() != NULL) {

            startedge->GetLeftFace()->GetNormal(facenor1(0), facenor1(1), facenor1(2));
            facearea = startedge->GetLeftFace()->CalArea();
            faceareasum += facearea;
            facenorsum = facenorsum + facenor1 * facearea;
        }

        if (startedge->GetRightFace() != NULL) {
            startedge->GetRightFace()->GetNormal(facenor2(0), facenor2(1), facenor2(2));
            facearea = startedge->GetLeftFace()->CalArea();
            faceareasum += facearea;
            facenorsum = facenorsum + facenor2 * facearea;
        }
        facenorsum = facenorsum / faceareasum;

		double p1[3], p2[3], p3[3];
		startedge->GetStartPoint()->GetCoord3D(p1);
		startedge->GetEndPoint()->GetCoord3D(p2);

		for (int j = 0; j < 3; j++) {
			//compute the position for this isonode
			p3[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];
		}

		newisonode->SetCoord3D(p3[0], p3[1], p3[2]);
        newisonode->SetNormal(facenorsum(0), facenorsum(1), facenorsum(2));
		newline.push_back(newisonode);
		//if (startedge->isoutterboundary) break;
		//if (startedge->isoutterboundary)break;
		if (startedge->isoutterboundary) {

			if (newline.size() < 3)
			{
				for (int i = 0; i < newline.size(); i++)
				{
					delete newline.at(i);
				}
			}
			else
			{
				reorderednode.push_back(newline);
 			}
			newline.clear();
			std::vector<QMeshNode*>().swap(newline);
			for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
				QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

				if ((Edge->GetStartPoint()->zigzagValue - isovalue) * (Edge->GetEndPoint()->zigzagValue - isovalue) < 0) {

					if (!edgevisit[Edge->GetIndexNo()]) {

						if (Edge->isoutterboundary) {

							startedge = Edge;

							edgevisit.at(startedge->GetIndexNo()) = true;

							QMeshNode* newnewisonode = new QMeshNode;

							double boutvalue_ss = startedge->GetStartPoint()->scalarField;
							double boutvalue_ee = startedge->GetEndPoint()->scalarField;

							double alpha1 = (isovalue - boutvalue_ss) / (boutvalue_ee - boutvalue_ss);

							double pp1[3], pp2[3], pp3[3];
							startedge->GetStartPoint()->GetCoord3D(pp1);
							startedge->GetEndPoint()->GetCoord3D(pp2);


                            facearea = 0, faceareasum = 0;
                            facenor1.setZero(); facenor2.setZero(); facenorsum.setZero();

                            if (startedge->GetLeftFace() != NULL) {

                                startedge->GetLeftFace()->GetNormal(facenor1(0), facenor1(1), facenor1(2));
                                facearea = startedge->GetLeftFace()->CalArea();
                                faceareasum += facearea;
                                facenorsum = facenorsum + facenor1 * facearea;
                            }

                            if (startedge->GetRightFace() != NULL) {
                                startedge->GetRightFace()->GetNormal(facenor2(0), facenor2(1), facenor2(2));
                                facearea = startedge->GetLeftFace()->CalArea();
                                faceareasum += facearea;
                                facenorsum = facenorsum + facenor2 * facearea;
                            }
                            facenorsum = facenorsum / faceareasum;


							for (int j = 0; j < 3; j++) {
								//compute the position for this isonode
								pp3[j] = (1.0 - alpha1) * pp1[j] + alpha1 * pp2[j];
							}

							newnewisonode->SetCoord3D(pp3[0], pp3[1], pp3[2]);
                            newnewisonode->SetNormal(facenorsum(0), facenorsum(1), facenorsum(2));
							newline.push_back(newnewisonode);
							std::cout << "findnext" << std::endl;
							if (startedge->GetLeftFace() == NULL) { nextface = startedge->GetRightFace(); break; }
							else
							{
								nextface = startedge->GetLeftFace();
								break;
							}
						}
					}
				}
			}
		}
		else
		{
			if (nextface == startedge->GetLeftFace())nextface = startedge->GetRightFace();
			else
			{
				nextface = startedge->GetLeftFace();
			}
		}
	}

 

    for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
        QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
        double boutvalue_s = Edge->GetStartPoint()->scalarField;
        double boutvalue_e = Edge->GetEndPoint()->scalarField;

        if (((boutvalue_e - isovalue) * (boutvalue_s - isovalue) < 0) && !(edgevisit[Edge->GetIndexNo()])) {
            double alpha = (isovalue - boutvalue_s) / (boutvalue_e - boutvalue_s);

            QMeshNode* isonode = new QMeshNode;
            double p1[3], p2[3], p3[3];
            startedge->GetStartPoint()->GetCoord3D(p1);
            startedge->GetEndPoint()->GetCoord3D(p2);

            facearea = 0, faceareasum = 0;
            facenor1.setZero(); facenor2.setZero(); facenorsum.setZero();

            if (startedge->GetLeftFace() != NULL) {

                startedge->GetLeftFace()->GetNormal(facenor1(0), facenor1(1), facenor1(2));
                facearea = startedge->GetLeftFace()->CalArea();
                faceareasum += facearea;
                facenorsum = facenorsum + facenor1 * facearea;
            }

            if (startedge->GetRightFace() != NULL) {
                startedge->GetRightFace()->GetNormal(facenor2(0), facenor2(1), facenor2(2));
                facearea = startedge->GetLeftFace()->CalArea();
                faceareasum += facearea;
                facenorsum = facenorsum + facenor2 * facearea;
            }
            facenorsum = facenorsum / faceareasum;

            for (int j = 0; j < 3; j++) {
                //compute the position for this isonode
                p3[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];
            }

            isonode->SetCoord3D(p3[0], p3[1], p3[2]);
            isonode->SetNormal(facenorsum(0), facenorsum(1), facenorsum(2));
            noresamplenode.push_back(isonode);
         }
    }


	return reorderednode.size();
}

 

// ------------------------------------------------------------
// 输入：
//   L          #V×#V  负半正定 cot 拉普拉斯（igl::cotmatrix 得到）
//   rhs        #V×1   integratedDivs  （右端）
//   fixed      std::vector<int>       // 0-based 固定顶点索引
//   fixed_val  double 或 VectorXd     // 这些顶点的目标值
// 输出：
//   x ( #V×1 ) 满足
//        (-L)x = rhs,  且 x(fixed) = fixed_val
// ------------------------------------------------------------
Eigen::VectorXd solve_poisson_with_dirichlet(
    const Eigen::SparseMatrix<double>& L,
    const Eigen::VectorXd& rhs,
    const std::vector<int>& fixed,
    const double                       fixed_val = 0.0)
{
    const int n = L.rows();
    const int n_f = static_cast<int>(fixed.size());

    /* --------- 1. 构造映射 old_id → new_id --------- */
    Eigen::VectorXi map(n);      // -1 表示被固定
    map.setConstant(-1);
    int new_id = 0;
    for (int i = 0; i < n; ++i)
        if (std::find(fixed.begin(), fixed.end(), i) == fixed.end())
            map(i) = new_id++;

    const int n_free = new_id;                  // 自由未知元个数
    Eigen::SparseMatrix<double> Lr(n_free, n_free);
    Eigen::VectorXd rhs_r(n_free);
    std::vector<Eigen::Triplet<double>> triplet;

    /* --------- 2. 填充删行删列后的矩阵 / 右端 --------- */
    for (int k = 0; k < L.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(L, k); it; ++it)
        {
            int i = it.row(), j = it.col();
            double v = -it.value();              // 我们要求解 (-L)x = rhs
            if (map(i) != -1 && map(j) != -1)
                triplet.emplace_back(map(i), map(j), v);          // 自由?自由
            else if (map(i) != -1 && map(j) == -1)
                rhs_r(map(i)) += v * fixed_val;                   // 自由?固定
        }

    for (int i = 0; i < n; ++i)
        if (map(i) != -1) rhs_r(map(i)) += rhs(i);                 // 加原 RHS

    Lr.setFromTriplets(triplet.begin(), triplet.end());

    /* --------- 3. 求解 --------- */
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(Lr);
    Eigen::VectorXd xr = solver.solve(rhs_r);

    /* --------- 4. 拼回完整解 --------- */
    Eigen::VectorXd x(n);
    x.setConstant(fixed_val);
    for (int i = 0; i < n; ++i)
        if (map(i) != -1) x(i) = xr(map(i));

    return x;
}

void posiontoolpath::fieldcomputing() {

    assignvectortoface();
  //  determineboundarynode();

    //for (GLKPOSITION Pos1 = surfacemesh->GetNodeList().GetHeadPosition(); Pos1;) {
    //    QMeshNode* node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(Pos1);
    //    //node->scalarField = nodefieldvalue(node->GetIndexNo());
    //    //  std::cout << node->scalarField << std::endl;
    //    if (node->isFixed)meshboundarynode.push_back(node->GetIndexNo());
    //}
    //std::cout << meshboundarynode.size() << std::endl;
   // return;
 /*   for (int i = 0; i < meshboundarynode.size(); i++)
    {
        QMeshNode* node = surfacemesh->GetNodeRecordPtr(meshboundarynode[i] + 1);
        node->scalarField = 1;
    }
   return;*/

    //Eigen::MatrixXd V; // 顶点
    //Eigen::MatrixXi F; // 三角面
    //Eigen::MatrixXd VECTORS; // 每个三角面引导场(nf,3)或(nf,2)
    //Eigen::VectorXd b;
    //V.resize(surfacemesh->GetNodeNumber(), 3);
    //F.resize(surfacemesh->GetFaceNumber(), 3);
    //VECTORS.resize(surfacemesh->GetFaceNumber(), 3);
    //Eigen::VectorXd divB;


    //for (GLKPOSITION Pos1 = surfacemesh->GetNodeList().GetHeadPosition(); Pos1;) {
    //    QMeshNode* node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(Pos1);
    //    Eigen::Vector3d pos3d;
    //    node->GetCoord3D(pos3d);
    //    for (int i = 0; i < 3; i++)
    //    {
    //        V(node->GetIndexNo(), i) = pos3d(i);
    //    }
    //}
    //for (GLKPOSITION Pos1 = surfacemesh->GetFaceList().GetHeadPosition(); Pos1;) {
    //    QMeshFace* face = (QMeshFace*)surfacemesh->GetFaceList().GetNext(Pos1);
    //    F(face->GetIndexNo(), 0) = face->GetNodeRecordPtr(0)->GetIndexNo();
    //    F(face->GetIndexNo(), 1) = face->GetNodeRecordPtr(1)->GetIndexNo();
    //    F(face->GetIndexNo(), 2) = face->GetNodeRecordPtr(2)->GetIndexNo();
    //    for (int i = 0; i < 3; i++)
    //    {
    //        VECTORS(face->GetIndexNo(), i) = face->fieldVec(i);
    //    }
    //}
    //
    //igl::cotmatrix(V, F, L);  // V: 顶点坐标(n,3), F: 三角面(nf,3)


    //// ---------- 1. 梯度矩阵 G ----------
    //Eigen::SparseMatrix<double> G;           // (#F*3) × #V
    //igl::grad(V, F, G);

    //// ---------- 2. 质量矩阵 M ----------
    //Eigen::SparseMatrix<double> M;           // #V × #V, 对角
    //igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_BARYCENTRIC, M);

    //// ---------- 3. M 的逆（逐对角求倒数即可） ----------
    //Eigen::SparseMatrix<double> Minv(M.rows(), M.cols());
    //Minv.reserve(M.nonZeros());
    //for (int k = 0; k < M.outerSize(); ++k)
    //    for (Eigen::SparseMatrix<double>::InnerIterator it(M, k); it; ++it)
    //        Minv.insert(it.row(), it.col()) = 1.0 / it.value();

    //// ---------- 4. 把面向量场堆成 U_stack ----------
    //Eigen::VectorXd U_stack(3 * F.rows());   // (#F*3) × 1
    //for (int f = 0; f < F.rows(); ++f)
    //    U_stack.segment<3>(3 * f) = VECTORS.row(f).transpose();
    //Eigen::VectorXd area;                // #F × 1
    //igl::doublearea(V, F, area);  // #F × 1
    //area *= 0.5;                         // 变成真正的面积

    //Eigen::VectorXd S_U = U_stack;                      // 临时
    //for (int f = 0; f < F.rows(); ++f)       // 逐面乘面积
    //    S_U.segment<3>(3 * f) *= area(f);

    //divB = -(G.transpose() * S_U).cwiseQuotient(M.diagonal());

    //// ---------- 5. 计算散度 ----------
    // divB = -Minv * (G.transpose() * U_stack);   // #V × 1
    // std::vector<int> boundary = meshboundarynode;   // 你的输入
    // Eigen::VectorXi known(boundary.size());
    // for (int i = 0; i < boundary.size(); ++i) known(i) = boundary[i];  // 0-based
    // Eigen::VectorXd knownVal = Eigen::VectorXd::Zero(known.size());
    // igl::min_quad_with_fixed_data<double> data;
    // bool pd = false;  // -L 只是半正定
    // L = -L;
    // igl::min_quad_with_fixed_precompute(
    //     L,                     // Q
    //     known,                  // 固定索引
    //     Eigen::SparseMatrix<double>(), // 无 Aeq
    //     pd,
    //     data);                  // 输出
    // Eigen::VectorXd Beq;                // 空向量，列数必须与 Aeq.rows 对应

    // igl::min_quad_with_fixed_solve(data, divB, knownVal, Beq, nodefieldvalue);

    //Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
    //solver.compute(-L); // 注意cotmatrix是负拉普拉斯
    //nodefieldvalue = solver.solve(divB);
    //return;
    L.resize(surfacemesh->GetNodeNumber() + meshboundarynode.size(), surfacemesh->GetNodeNumber() );
    B.resize(surfacemesh->GetNodeNumber() + meshboundarynode.size());
    B.setZero();
    buildLaplacian(L);
    computeIntegratedDivergence();
    for (int i = 0; i < integratedDivs.size(); i++)
    {
        B(i) = integratedDivs(i)* posweighting;
    }
    for (int i = 0; i < meshboundarynode.size(); i++)
    {
        B(surfacemesh->GetNodeNumber() + i) = startnodeweighting;
    }
    Eigen::SparseMatrix<double>ATA(surfacemesh->GetNodeNumber(), surfacemesh->GetNodeNumber());

    ATA = L.transpose() * L;
    Eigen::VectorXd ATB(surfacemesh->GetNodeNumber());
    ATB = L.transpose() * B;
    Eigen::PardisoLU <Eigen::SparseMatrix<double>> Solver;
  //  Solver.compute(ATA);
    Solver.compute(L);
    nodefieldvalue = Solver.solve(B);

 /*   if (Solver.info() != Eigen::Success) {
        std::cerr << "Solving failed!" << std::endl;
        return;
    }*/

    // compute max and min phis
    double minPhi = INFINITY;
    double maxPhi = -INFINITY;

    for (int i = 0; i < surfacemesh->GetNodeNumber(); i++) {
        if (minPhi > nodefieldvalue(i)) minPhi = nodefieldvalue(i);
        if (maxPhi < nodefieldvalue(i)) maxPhi = nodefieldvalue(i);
    }
    double range = maxPhi - minPhi;
    std::cout << "maxPhi=" << maxPhi << "minPhi=" << minPhi << std::endl;
    for (int i = 0; i < surfacemesh->GetNodeNumber(); i++)
        nodefieldvalue(i) = (nodefieldvalue(i) - minPhi) / range;

    for (GLKPOSITION Pos1 = surfacemesh->GetNodeList().GetHeadPosition(); Pos1;) {
        QMeshNode* node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(Pos1);
        node->scalarField = nodefieldvalue(node->GetIndexNo());
      //  std::cout << node->scalarField << std::endl;
    }

    for (GLKPOSITION Pos = surfacemesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* Face = (QMeshFace*)surfacemesh->GetFaceList().GetNext(Pos);
        Eigen::Vector3d gradient, normal;
        gradient.setZero();
        Face->CalPlaneEquation();
        Face->GetNormal(normal(0), normal(1), normal(2));
        normal.normalize();

        for (int i = 0; i < 3; i++) {
            QMeshNode* Node = Face->GetNodeRecordPtr(i % 3);
            QMeshNode* Node1 = Face->GetNodeRecordPtr((i + 1) % 3);
            QMeshNode* Node2 = Face->GetNodeRecordPtr((i + 2) % 3);

            double ui = Node->scalarField;
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

    double minoffset = 0.005;
    double maxoffset = 0.995;

    for (int i = 0; i < toolpathnum; i++)
    {
        double isoCurveValue = static_cast<double>(i) * (maxoffset - minoffset) / static_cast<double>(toolpathnum - 1) + minoffset;
        computesignletoolpath(surfacemesh, isoCurveValue);
    }

    for (int i = 0; i < reorderednode.size(); i++)
    {
        for (int j = 0; j < reorderednode[i].size(); j++) {
            toolpath->GetNodeList().AddTail(reorderednode[i][j]);
        }
    }
    char waypointFile[500];
     sprintf(waypointFile, "%s", "../model/waypoint.txt");

    std::ofstream WPFile(waypointFile);

    for (int i = 0; i < reorderednode.size(); i++)
    {
        for (int j = 0; j < reorderednode[i].size(); j++) {
            Eigen::Vector3d pos,posn;

            reorderednode[i][j]->GetCoord3D(pos);
            reorderednode[i][j]->GetNormal(posn(0),posn(1),posn(2));

            WPFile << pos(0) << " " << pos(1) << " " << pos(2) << " " << posn(0) << " " << posn(1) << " " << posn(2) << " " << 0.5 << " " << 1 << " " << 0 << " " << 0.45 << std::endl;
        }
        WPFile << 1 << " " << 1<< " " <<1 << " " << 0 << " " << 0 << " " << 1 << " " << 0.5 << " " << 1 << " " << 1 << " " << 0.45 << std::endl;

    } 
    WPFile.close();

    for (int i = 0; i < reorderednode.size(); i++)
    {
        for (int j = 1; j < reorderednode[i].size(); j++) {
            QMeshEdge* edge = new QMeshEdge;
            edge->SetStartPoint(reorderednode[i][j-1]);
            edge->SetEndPoint(reorderednode[i][j]);
            toolpath->GetEdgeList().AddTail(edge);
         }
    }

    for (int i = 0; i < noresamplenode.size(); i++)
    {
        toolpath->GetNodeList().AddTail(noresamplenode[i]);

    }
    std::cout << "toolpathnodenum=" << toolpath->GetNodeNumber() << std::endl;
 }


void posiontoolpath::computeIntegratedDivergence()  {

    for (GLKPOSITION Pos = surfacemesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(Pos);
        double integratedDiv = 0.0;
        Eigen::Vector3d p; Node->GetCoord3D(p(0), p(1), p(2));

        for (GLKPOSITION Pos1 = Node->GetFaceList().GetHeadPosition(); Pos1;) {
            QMeshFace* connectFace = (QMeshFace*)Node->GetFaceList().GetNext(Pos1);

            Eigen::Vector3d gradient = connectFace->fieldVec;
            int index = connectFace->getNodePtrNumber(Node);
            if (index == -1) std::cout << "faild to detect the index for this face!";

            QMeshNode* Node1 = connectFace->GetNodeRecordPtr((index + 1) % 3);
            QMeshNode* Node2 = connectFace->GetNodeRecordPtr((index + 2) % 3);

            Eigen:: Vector3d  p1, p2;
            Node1->GetCoord3D(p1(0), p1(1), p1(2));
            Node2->GetCoord3D(p2(0), p2(1), p2(2));
            Eigen::Vector3d e1 = p1 - p;
            Eigen::Vector3d e2 = p2 - p;
            Eigen::Vector3d ei = p2 - p1;

            double theta1 = acos((-e2).dot(-ei) / (e2.norm() * ei.norm()));
            double cot1 = 1.0 / tan(theta1);

            double theta2 = acos((-e1).dot(ei) / (e1.norm() * ei.norm()));
            double cot2 = 1.0 / tan(theta2);

            integratedDiv += e1.dot(gradient) * cot1 + e2.dot(gradient) * cot2;
        }
        integratedDivs[Node->GetIndexNo()] = 0.5 * integratedDiv;
    }
}

void posiontoolpath::buildLaplacian
(Eigen::SparseMatrix<double>& L)  {
    std::vector<Eigen::Triplet<double>> LTriplet;

    for (GLKPOSITION Pos = surfacemesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(Pos);
         double sumCoefficients = 0.0;

        for (GLKPOSITION Pos1 = Node->GetEdgeList().GetHeadPosition(); Pos1;) {
            QMeshEdge* connectEdge = (QMeshEdge*)Node->GetEdgeList().GetNext(Pos1);
            double coefficient = computeEdgeAngle(Node, connectEdge);
            sumCoefficients += coefficient;
            QMeshNode* Node1 = connectEdge->GetEndPoint(); if (Node == Node1) Node1 = connectEdge->GetStartPoint();

           LTriplet.push_back(Eigen::Triplet<double>(Node->GetIndexNo(), Node1->GetIndexNo(), coefficient* posweighting));
           
        }
         
            LTriplet.push_back(Eigen::Triplet<double>(Node->GetIndexNo(), Node->GetIndexNo(), -sumCoefficients* posweighting));
    }
    for (int i = 0; i < meshboundarynode.size(); i++)
    {
        LTriplet.push_back(Eigen::Triplet<double>(i + surfacemesh->GetNodeNumber(), meshboundarynode[i], startnodeweighting));
    }
    L.setFromTriplets(LTriplet.begin(), LTriplet.end());
}

double posiontoolpath::computeEdgeAngle
(QMeshNode* Node, QMeshEdge* connectEdge)  {
    //Eigen::Vector3d p0, p1, p2, v1, v2;
    //double cotA, cotB;

    //QMeshNode* Node1 = connectEdge->GetEndPoint(); if (Node == Node1) Node1 = connectEdge->GetStartPoint();

    //if (connectEdge->GetLeftFace() != nullptr) {
    //    QMeshFace* LeftFace = connectEdge->GetLeftFace();
    //    QMeshNode* LFNode = LeftFace->GetNodeRecordPtr(0);

    //    if (LFNode == Node1 || LFNode == Node) LFNode = LeftFace->GetNodeRecordPtr(1);
    //    if (LFNode == Node1 || LFNode == Node) LFNode = LeftFace->GetNodeRecordPtr(2);
    //    //if (Node->GetIndexNo() == Node1->GetIndexNo() || Node->GetIndexNo() == LFNode->GetIndexNo() || Node1->GetIndexNo() == LFNode->GetIndexNo())
    //    Node->GetCoord3D(p0(0), p0(1), p0(2));
    //    Node1->GetCoord3D(p1(0), p1(1), p1(2));
    //    LFNode->GetCoord3D(p2(0), p2(1), p2(2));
    //    v1 = p2 - p1;
    //    v2 = p2 - p0;
    //    cotA = fabs(v1.dot(v2) / v1.cross(v2).norm());
    //}
    //else cotA = 0;

    //if (connectEdge->GetRightFace() != nullptr) {
    //    QMeshFace* RightFace = connectEdge->GetRightFace();
    //    QMeshNode* RFNode = RightFace->GetNodeRecordPtr(0);

    //    if (RFNode == Node1 || RFNode == Node) RFNode = RightFace->GetNodeRecordPtr(1);
    //    if (RFNode == Node1 || RFNode == Node) RFNode = RightFace->GetNodeRecordPtr(2);
    //    //if (Node->GetIndexNo() == Node1->GetIndexNo() || Node->GetIndexNo() == RFNode->GetIndexNo() || Node1->GetIndexNo() == RFNode->GetIndexNo())
    //    Node->GetCoord3D(p0(0), p0(1), p0(2));
    //    Node1->GetCoord3D(p1(0), p1(1), p1(2));
    //    RFNode->GetCoord3D(p2(0), p2(1), p2(2));
    //    v1 = p2 - p1;
    //    v2 = p2 - p0;
    //    cotB = fabs(v1.dot(v2) / v1.cross(v2).norm());
    //}
    //else cotB = 0;
    ////cout << "cotA = " << cotA << ", cotB = " << cotB << endl;
    //return(0.5 * (cotA + cotB));

    Eigen::Vector3d p0, p1, p2, v1, v2;
    double cotA, cotB;

    QMeshNode* Node1 = connectEdge->GetEndPoint(); if (Node == Node1) Node1 = connectEdge->GetStartPoint();

    if (connectEdge->GetLeftFace() != nullptr) {
        QMeshFace* LeftFace = connectEdge->GetLeftFace();
        QMeshNode* LFNode = LeftFace->GetNodeRecordPtr(0);

        if (LFNode == Node1 || LFNode == Node) LFNode = LeftFace->GetNodeRecordPtr(1);
        if (LFNode == Node1 || LFNode == Node) LFNode = LeftFace->GetNodeRecordPtr(2);
        if (Node->GetIndexNo() == Node1->GetIndexNo() || Node->GetIndexNo() == LFNode->GetIndexNo() || Node1->GetIndexNo() == LFNode->GetIndexNo())
            std::cout << "Left face is in a mess!" << endl;
        Node->GetCoord3D(p0(0), p0(1), p0(2));
        Node1->GetCoord3D(p1(0), p1(1), p1(2));
        LFNode->GetCoord3D(p2(0), p2(1), p2(2));
        v1 = p2 - p1;
        v2 = p2 - p0;
        cotA = v1.dot(v2) / v1.cross(v2).norm();
    }
    else cotA = 0;

    if (connectEdge->GetRightFace() != nullptr) {
        QMeshFace* RightFace = connectEdge->GetRightFace();
        QMeshNode* RFNode = RightFace->GetNodeRecordPtr(0);

        if (RFNode == Node1 || RFNode == Node) RFNode = RightFace->GetNodeRecordPtr(1);
        if (RFNode == Node1 || RFNode == Node) RFNode = RightFace->GetNodeRecordPtr(2);
        if (Node->GetIndexNo() == Node1->GetIndexNo() || Node->GetIndexNo() == RFNode->GetIndexNo() || Node1->GetIndexNo() == RFNode->GetIndexNo())
            std::cout << "Right face is in a mess!" << endl;
        Node->GetCoord3D(p0(0), p0(1), p0(2));
        Node1->GetCoord3D(p1(0), p1(1), p1(2));
        RFNode->GetCoord3D(p2(0), p2(1), p2(2));
        v1 = p2 - p1;
        v2 = p2 - p0;
        cotB = v1.dot(v2) / v1.cross(v2).norm();
    }
    else cotB = 0;
    //cout << "cotA = " << cotA << ", cotB = " << cotB << endl;
    return(0.5 * (cotA + cotB));
}