#include "ComputeGeoViaigl.h"

std::vector<std::vector<int>> ComputeGeoViaigl::findBoundaryLoops(const Eigen::MatrixXi& F) {
    std::map<std::pair<int, int>, int> edge_count;
    std::map<int, std::set<int>> boundary_adjacency;

    // 统计每条边出现的次数
    for (int i = 0; i < F.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int v1 = F(i, j);
            int v2 = F(i, (j + 1) % 3);

            if (v1 > v2) std::swap(v1, v2);
            edge_count[std::make_pair(v1, v2)]++;
        }
    }

    // 构建边界邻接表
    for (const auto& edge : edge_count) {
        if (edge.second == 1) {
            boundary_adjacency[edge.first.first].insert(edge.first.second);
            boundary_adjacency[edge.first.second].insert(edge.first.first);
        }
    }

    // 提取边界循环
    std::vector<std::vector<int>> loops;
    std::set<int> visited;

    for (const auto& vertex : boundary_adjacency) {
        if (visited.find(vertex.first) == visited.end()) {
            std::vector<int> loop;
            std::queue<int> q;
            q.push(vertex.first);

            while (!q.empty()) {
                int v = q.front();
                q.pop();

                if (visited.find(v) != visited.end()) continue;

                visited.insert(v);
                loop.push_back(v);

                for (int neighbor : boundary_adjacency[v]) {
                    if (visited.find(neighbor) == visited.end()) {
                        q.push(neighbor);
                    }
                }
            }

            if (!loop.empty()) {
                loops.push_back(loop);
            }
        }
    }

    return loops;
}


void ComputeGeoViaigl::Compute_Geo() {

    // 标志位：true = 计算到外边界的距离, false = 计算到内边界的距离
    bool computefixednode = false;  // 修改此标志位来选择
   
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    V.resize(surfacemesh->GetNodeNumber(), 3);
    F.resize(surfacemesh->GetFaceNumber(), 3);
    for (GLKPOSITION nodepos = surfacemesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
    {
        QMeshNode* node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(nodepos);
        Eigen::Vector3d pos3d;
        node->GetCoord3D(pos3d);
        for (int i = 0; i < 3; i++)
        {
            V(node->GetIndexNo(), i) = pos3d(i);
        }
    }

    for (GLKPOSITION facepos = surfacemesh->GetFaceList().GetHeadPosition(); facepos != NULL;)
    {
        QMeshFace* face = (QMeshFace*)surfacemesh->GetFaceList().GetNext(facepos);
        for (int i = 0; i < 3; i++)
        {
            F(face->GetIndexNo(), i) = face->GetNodeRecordPtr(i)->GetIndexNo();
        }
    }

    // 找到所有边界循环
    std::vector<std::vector<int>> boundary_loops = findBoundaryLoops(F);

    if (boundary_loops.empty()) {
        std::cerr << "No boundary found! The mesh might be closed." << std::endl;
        return;
    }

    // 选择边界
    std::vector<int> selected_boundary;
    std::vector<int> selected_boundary_inner;
    std::vector<int> fixednode;

    if (computefixednode) {
        for (GLKPOSITION nodepos = surfacemesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
        {
            QMeshNode* node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(nodepos);
            if (node->isFixed) {
                fixednode.push_back(node->GetIndexNo());
            }
        }
    }

    if (boundary_loops.size() == 1) {
        // 只有一个边界
        selected_boundary = boundary_loops[0];
        std::cout << "Only one boundary found with " << selected_boundary.size() << " vertices." << std::endl;
    }
    else {
        // 多个边界，找出最大和最小的
        size_t max_size = 0, min_size = SIZE_MAX;
        int max_idx = 0, min_idx = 0;

        for (size_t i = 0; i < boundary_loops.size(); ++i) {
            size_t size = boundary_loops[i].size();
            if (size > max_size) {
                max_size = size;
                max_idx = i;
            }
            if (size < min_size) {
                min_size = size;
                min_idx = i;
            }
        }

        //  if (compute_to_outer_boundary) {
        selected_boundary = boundary_loops[max_idx];
        //    std::cout << "Computing distance to OUTER boundary (" << selected_boundary.size() << " vertices)" << std::endl;
        //}
        //else {
        selected_boundary_inner.clear();
        for (size_t i = 0; i < boundary_loops.size(); ++i) {
            if (static_cast<int>(i) == max_idx) continue;
            const auto& loop = boundary_loops[i];
            selected_boundary_inner.insert(selected_boundary_inner.end(), loop.begin(), loop.end());
        }
        //  std::cout << "Computing distance to INNER boundary (" << selected_boundary.size() << " vertices)" << std::endl;
      //}
    }

    // 转换为VectorXi
    Eigen::VectorXi VS(selected_boundary.size());
    Eigen::VectorXi VS_inner(selected_boundary_inner.size());
    Eigen::VectorXi Vs_fixed(fixednode.size());
    for (size_t i = 0; i < selected_boundary.size(); ++i) {
        VS(i) = selected_boundary[i];
    }
    for (size_t i = 0; i < selected_boundary_inner.size(); ++i) {
        VS_inner(i) = selected_boundary_inner[i];
    }
    if (computefixednode) {
        for (size_t i = 0; i < fixednode.size(); ++i) {
            Vs_fixed(i) = fixednode[i];
        }
    }

    // 边界顶点作为源点
    Eigen::VectorXi FS; // 留空
    // 所有顶点作为目标
    Eigen::VectorXi VT = Eigen::VectorXi::LinSpaced(V.rows(), 0, V.rows() - 1);
    Eigen::VectorXi FT; // 留空

    // 计算测地距离
    Eigen::VectorXd d;
    Eigen::VectorXd d_inner;
    Eigen::VectorXd disfixed;
    igl::exact_geodesic(V, F, VS, FS, VT, FT, d);

    if (selected_boundary_inner.size() != 0) {
        igl::exact_geodesic(V, F, VS_inner, FS, VT, FT, d_inner);
    }
    if (computefixednode) {
        igl::exact_geodesic(V, F, Vs_fixed, FS, VT, FT, disfixed);
    }
    std::cout << "Geodesic distances computed." << std::endl;

    int i = 0;
    for (GLKPOSITION nodepos = surfacemesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
    {
        QMeshNode* node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(nodepos);
        if (selected_boundary_inner.size() != 0) {
            node->DisToInnerBoundary = d_inner(i);
        }
        if (computefixednode) {
            node->geoFieldValue = disfixed(i);
        }
        node->DisToBoundary = d(i);
        i++;
    }
    if (true)//change geo to layerheight
    {
        double mixgeo = std::numeric_limits<double>::lowest();
        double mingeo = std::numeric_limits<double>::max();
        for (GLKPOSITION nodepos = surfacemesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
        {
            QMeshNode* node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(nodepos);
            if (node->geoFieldValue > mixgeo) {
                mixgeo = node->geoFieldValue;
            }
            if (node->geoFieldValue < mingeo) {
                mingeo = node->geoFieldValue;
            }
        }
       // double maxlayerheight = 0.8;
        //double minlayerheight = 0.3;
        for (GLKPOSITION nodepos = surfacemesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
        {
            QMeshNode* node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(nodepos);
            node->geoFieldValue = (mixgeo - node->geoFieldValue) / (mixgeo - mingeo);
        }
    }

    // === save ===
  /* std::string output_path = "D:/File/CUHK/LAB/TTA/TTA/SpatialFiberToolpathGenerator-main/model/isoSurface/Geoditance for TTA2/outter" + std::to_string(surfacemesh->GetIndexNo()) + ".txt";
    std::string output_path_inner = "D:/File/CUHK/LAB/TTA/TTA/SpatialFiberToolpathGenerator-main/model/isoSurface/Geoditance for TTA2/innerg"+ std::to_string(surfacemesh->GetIndexNo()) + ".txt";
    std::string geofixed = "D:/File/CUHK/LAB/TTA/TTA/SpatialFiberToolpathGenerator-main/model/isoSurface/Geoditance for TTA2/geofieldfixed" + std::to_string(surfacemesh->GetIndexNo()) + ".txt";

    std::ofstream fout(output_path);
    std::ofstream fout_inner(output_path_inner);
    //std::ofstream fout_geofixed(geofixed);

    //if (computefixednode) {
    //    for (GLKPOSITION nodepos = surfacemesh->GetNodeList().GetHeadPosition(); nodepos != NULL;)
    //    {
    //        QMeshNode* node = (QMeshNode*)surfacemesh->GetNodeList().GetNext(nodepos);
    //        fout_geofixed << node->geoFieldValue << std::endl;
    //    }
    //}
    if (!fout) {
        std::cerr << "Cannot open file for writing: " << output_path << std::endl;
        return ;
    }
    for (int i = 0; i < d.size(); ++i) {
        fout << d[i] << std::endl;
        if (selected_boundary_inner.size() != 0) {
            fout_inner << d_inner[i] << std::endl;
        }
    }*/

}