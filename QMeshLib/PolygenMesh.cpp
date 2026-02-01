// PMBody.cpp: implementation of the PMBody class.
//
//////////////////////////////////////////////////////////////////////
#define _CRT_SECURE_NO_DEPRECATE

#include <math.h>
#include <memory.h>

#include "PolygenMesh.h"

#include <QDebug>

#include "GlobalVar.h"



#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

PolygenMesh::PolygenMesh(mesh_type type)
{
    ClearAll();
    m_drawListID=-1;
    m_bVertexNormalShading=false;
    isTransparent = false;
    m_drawListNumber = 6;
    meshType = type;
}

PolygenMesh::~PolygenMesh()
{
    ClearAll();
    if (m_drawListID!=-1) glDeleteLists(m_drawListID, m_drawListNumber);
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

void PolygenMesh::CompBoundingBox(double boundingBox[])
{
    GLKPOSITION PosMesh;
    GLKPOSITION Pos;
    double xx,yy,zz;

    boundingBox[0]=boundingBox[2]=boundingBox[4]=1.0e+32;
    boundingBox[1]=boundingBox[3]=boundingBox[5]=-1.0e+32;

    for(PosMesh=meshList.GetHeadPosition();PosMesh!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(PosMesh));
        for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
            QMeshNode *node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
            node->GetCoord3D(xx,yy,zz);

            if (xx<boundingBox[0]) boundingBox[0]=xx;
            if (xx>boundingBox[1]) boundingBox[1]=xx;
            if (yy<boundingBox[2]) boundingBox[2]=yy;
            if (yy>boundingBox[3]) boundingBox[3]=yy;
            if (zz<boundingBox[4]) boundingBox[4]=zz;
            if (zz>boundingBox[5]) boundingBox[5]=zz;
        }
    }
}

void PolygenMesh::DeleteGLList()
{
    if (m_drawListID!=-1) {
        glDeleteLists(m_drawListID, m_drawListNumber);
        m_drawListID=-1;
    }
}

void PolygenMesh::BuildGLList(bool bVertexNormalShading)
{
    if (m_drawListID!=-1) glDeleteLists(m_drawListID, m_drawListNumber);
    m_drawListID = glGenLists(m_drawListNumber);

    _buildDrawShadeList(bVertexNormalShading);
    _buildDrawMeshList();
    _buildDrawNodeList();
    _buildDrawProfileList();
    _buildDrawFaceNormalList();
    _buildDrawNodeNormalList();
    computeRange();
}

void PolygenMesh::_buildDrawShadeList(bool bVertexNormalShading)
{
    GLKPOSITION Pos;
    GLKPOSITION PosFace;
    GLKPOSITION PosNode;
    QMeshFace* face;
    QMeshNode* node;
    QMeshPatch* mesh;
    double xx, yy, zz, dd;		float rr, gg, bb;
    int k, i, num, meshIndex;
    glNewList(m_drawListID, GL_COMPILE);

    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);

	//drawOriginalCoordinate();
    if (isTransparent) {
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    meshIndex = 0;
    for (Pos = meshList.GetHeadPosition(); Pos != NULL; meshIndex++) {
        mesh = (QMeshPatch*)(meshList.GetNext(Pos));

        if (this->meshType == CURVED_LAYER && mesh->drawThisIsoLayer == false) continue;

        glBegin(GL_TRIANGLES);

        for (PosFace = (mesh->GetFaceList()).GetHeadPosition(); PosFace != NULL;)
        {
            face = (QMeshFace*)((mesh->GetFaceList()).GetNext(PosFace));

            rr = gg = bb = 0.8f;
            glColor3f(rr, gg, bb);

            if (isTransparent)  glColor4f(rr, gg, bb, 0.8);

            if (mesh->drawPrincipleStressField) //FEM stress distribution result
            {
                QMeshTetra* Tetra;
                if (face->GetLeftTetra() == NULL) Tetra = face->GetRightTetra();
                else Tetra = face->GetLeftTetra();
                if (Tetra != nullptr) {
                    //_changeValueToColor(-10,10, Tetra->eleStress[0], rr, gg, bb);
                    _changeValueToColor(mesh->maxPrincipleStressValue / 3, mesh->minPrincipleStressValue / 3, Tetra->sigma_max, rr, gg, bb);
                    glColor3f(rr, gg, bb);
                }
            }

            if (this->meshType == CURVED_LAYER) {
                _changeValueToColor(mesh->GetIndexNo(), rr, gg, bb);
             //   rr = 0.1; gg = 0.1; bb = 0.1;
                if (face->errorface) {
                    glColor3f(0.3, 0.3, 0.3);
                }
                else
                {
                    glColor3f(rr, gg, bb);
                }
            } 
            if (this->meshType == UNDEFINED) {
                rr = 1;
                gg = 0;
                bb = 0;
                glColor3f(rr, gg, bb);
            }

            //if (face->selected) { rr = 0.2f; glColor3f(rr, gg, bb); }
            if (this->meshType == CURVED_LAYER && mesh->drawPrincipleStressField) {
                
                double PS = face->principleStress; int bEdge = 1;
                for (int i = 0; i < 3; i++) {
                    QMeshEdge* edge = face->GetEdgeRecordPtr(i+1);
                    if (!edge->IsBoundaryEdge()) {
                        bEdge += 1;
                        if (edge->GetLeftFace() == face) PS += edge->GetRightFace()->principleStress;
                        else PS += edge->GetLeftFace()->principleStress;
                    }
                }
                PS = PS / bEdge;

                _changeValueToColor(mesh->maxPrincipleStressValue/3, mesh->minPrincipleStressValue / 3, PS, rr, gg, bb);

                glColor3f(rr, gg, bb);
            }
          
            num = face->GetEdgeNum();
            for (k = 0; k < num - 2; k++) {
                for (i = 0; i < 3; i++) {
                    if (i < 1)
                        node = face->GetNodeRecordPtr(i);
                    else
                        node = face->GetNodeRecordPtr(i + k);
                    bVertexNormalShading = true;
                    if (bVertexNormalShading) { // && face->m_nIdentifiedPatchIndex>0) {
                        double normal[3];
                        node->CalNormal(normal);
                        glNormal3dv(normal);
                    }
                    else {
                        face->GetPlaneEquation(xx, yy, zz, dd);
                        glNormal3d(xx, yy, zz);
                    }
                    node->GetCoord3D(xx, yy, zz);
                    glVertex3d(xx, yy, zz);
                }
            }
        
        }
        glEnd();
    }

    if (isTransparent) {
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
    }

    glEndList();
}

void PolygenMesh::_buildDrawMeshList()
{
    GLKPOSITION Pos;
    GLKPOSITION PosEdge;
    float rr, gg, bb;

    if (meshList.GetCount()==0) return;

    glNewList(m_drawListID+1, GL_COMPILE);
    glDisable(GL_LIGHTING);

    glLineWidth(0.5);

    QMeshEdge* edge;
    QMeshPatch* mesh;
    double xx, yy, zz;

    glBegin(GL_LINES);
    int meshIndex = 0;
    for (Pos = meshList.GetHeadPosition(); Pos != NULL; meshIndex++) {
        mesh = (QMeshPatch*)(meshList.GetNext(Pos));
       if ((this->meshType == CURVED_LAYER|| this->meshType ==Tool_PATH) && mesh->drawThisIsoLayer == false) continue;

        for (PosEdge = (mesh->GetEdgeList()).GetHeadPosition(); PosEdge != NULL;) {
            edge = (QMeshEdge*)((mesh->GetEdgeList()).GetNext(PosEdge));
            if (edge->GetEndPoint()->noextrusionnode) edge->invisible = true;
            else edge->invisible = false;
         if (!shownoextrusion && this->meshType == Tool_PATH && edge->invisible)continue;
         if (edge->unseeedge)continue;
            rr = 0.2; gg = 0.2; bb = 0.2;
            glColor3f(rr, gg, bb);
            edge->GetStartPoint()->GetCoord3D(xx, yy, zz);
            glVertex3d(xx, yy, zz);
            edge->GetEndPoint()->GetCoord3D(xx, yy, zz);
            glVertex3d(xx, yy, zz);
        }
    }
    glEnd();
    glEndList();
}

void PolygenMesh::_buildDrawNodeList()
{
    GLKPOSITION Pos;
    GLKPOSITION PosNode;
    float rr, gg, bb;

    if (meshList.GetCount()==0) return;

    glNewList(m_drawListID+2, GL_COMPILE);
    glDisable(GL_LIGHTING);

    QMeshNode *node;
    QMeshPatch *mesh;
    PolygenMesh* pmesh;
    double xx,yy,zz,nx,ny,nz;

    glEnable(GL_POINT_SMOOTH);
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    // glPointSize(4.0);

    glPointSize(4.0);
    glBegin(GL_POINTS);// plot points
    int meshIndex = 0;
    
    double maxlayerheight = 0.0;
    double minlayerheight = 0.0;
    for (Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
        pmesh = (PolygenMesh*)(meshList.GetNext(Pos));

        if (pmesh->meshType== CURVED_LAYER)
        {
            maxlayerheight = pmesh->maxlayerheight;
            minlayerheight = pmesh->minlayerheight;
            std::cout << "maxlayerheight=" << maxlayerheight;
            std::cout << "minlayerheight=" << minlayerheight;
        }

    }

    for (Pos = meshList.GetHeadPosition(); Pos != NULL; meshIndex++) {
        mesh = (QMeshPatch*)(meshList.GetNext(Pos));
        
       // if (this->meshType == CURVED_LAYER && mesh->drawThisIsoLayer == false) continue;
      //  if (this->meshType == Tool_PATH && mesh->drawThisIsoLayer == false) continue;

        for (PosNode = (mesh->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
            node = (QMeshNode*)((mesh->GetNodeList()).GetNext(PosNode));
            //if (!shownoextrusion&&(((!node->resampleChecked)&&this->meshType==Tool_PATH)||node->noextrusionnode)) continue;
            //if (!shownoextrusion&&(node->noextrusionnode)) continue;

            //if (!node->isFixed) {
            //   // rr = 0.6; gg = 0.6; bb = 1.0;
            //    continue;
            //}
            rr = 0.75; gg = 0.75; bb = 0.75;
            if (node->selected) {
                rr = 0.6; gg = 0.6; bb = 1.0;
            }
            else if (node->isFixed) {
                rr = 0.3; gg = 0.8; bb = 1.0;
            }
            else if (node->isHandle) {
                rr = 0.8; gg = 0.3; bb = 1.0;
            }
            else {}
            if (node->m_nIdentifiedPatchIndex >= 0)
                _changeValueToColor(node->m_nIdentifiedPatchIndex, rr, gg, bb);
             //kaguya
           
             
            //std::cout << node->geoFieldValue << std::endl;

            //rr = 0.15; gg = 0.75; bb = 0.75;

            /*if (node->isSkeletonNode == true) {
                rr = 0.1; gg = 0.1; bb = 0.1;
            }*/

 

            /*if (node->isCornerNodeIndex != -1) {
                rr = 0.6; gg = 0.0; bb = 0.0;
            }*/

            /*if (node->isFootNodeIndex != -1) {
                rr = 0.8; gg = 0.0; bb = 0.0;
            }*/
            if(node->skeletonSelected) { rr = 0.8; gg = 0.0; bb = 0.0; }

            if (mesh->drawgeoField) {
                _changeValueToColor(1, -1, node->geoFieldValue, rr, gg, bb);
                //_changeValueToColor(1, 0, node->geoFieldValueSet[1], rr, gg, bb);
            }

            //_changeValueToColor(1, 0, node->boundaryValue, rr, gg, bb);

            /*if (node->selected) { rr = 0.8; gg = 0.0; bb = 0.0; }
            else { rr = 0.2; gg = 0.2; bb = 0.2; }*/

            if(node->nodeHoleIndex>-1) { rr = 0.8; gg = 0.8; bb = 0.0; }

            if (mesh->skeletonPatch) {
                if (node->selected) {
                    rr = 0.8; gg = 0.0; bb = 0.0;
                }
                if (node->selected1) {
                    rr = 0.0; gg = 0.8; bb = 0.0;
                }
                if (node->selected2) {
                    rr = 0.0; gg = 0.0; bb = 0.8;
                }
                if (node->selected3) {
                    rr = 0.0; gg = 0.8; bb = 0.8;
                }
                if (node->isFixed) {
                    rr = 0.0; gg = 0.8; bb = 0.0;
                }
                if (node->isHandle) {
                    rr = 0.0; gg = 0.0; bb = 0.8;
                }
               
                //std::cout << "find selected node on skeleton patch" << std::endl;
            }

            /*if (node->seedIndex == mesh->seedPointNum - 1) {
                rr = 0.0; gg = 0.8; bb = 0.8;
                if (node->skeletonSelected) {
                    rr = 0.0; gg = 0.0; bb = 0.0;
                }
            }*/

            if (node->isFootNodeIndex >-1) { rr = 0.0; gg = 0.0; bb = 1.0; }
            else {
                if (node->seedIndex >= 0) _changeValueToColor(node->seedIndex, rr, gg, bb);
            }

            if (mesh->drawScalarField)  
                _changeValueToColor(1, 0, node->scalarField, rr, gg, bb);
 
            if (mesh->drawtoolpathscalar)
            {
                _changeValueToColor(1, 0, node->geoFieldValue, rr, gg, bb);
            }
          /* if (node->isouterline)
            {
                rr = 0;
                gg = 0;
                bb = 0;
            }
           if (node->isinnerline)
            {
                rr = 0;
                gg = 0;
                bb = 0;
                if (this->meshType==UNDEFINED)
                {
                    glPointSize(10.0);
                }  
            }*/
           if (this->meshType == BOUNDARY)
           {
               rr = 0;
               gg = 0;
               bb = 1;

               glPointSize(20.0);
           }
            if (mesh->istoolpath)
            {
                if (!node->discardnode)
                {
                    rr = 0;
                    gg = 0;
                    bb = 0;
                }
                else
                {
                    rr = 1;
                    gg = 1;
                    bb = 1;
                }
                glPointSize(2.0);
            }
            if (this->meshType == CENTERLLINE)
            {
                rr = 0;
                gg = 0;
                bb = 0;
                glPointSize(8.0);
            }
            else if (this->meshType == SAMPLENODE)
            {
                rr = 1;
                gg = 0;
                bb = 0;
                glPointSize(8.0);
            }
            else if (this->meshType == SAMPLENODEPAIR)
            {
                rr = 0;
                gg = 0;
                bb = 1;
                glPointSize(8.0);
            }
            else
            {
                glPointSize(2.0);
            }
       
            if (this->meshType == Tool_PATH&&node->resampleChecked) {
                _changeValueToColor(0.036, 0.012, node->printvelocity, rr, gg, bb);
            }
            if (mesh->drawThisIsoLayer == false) continue;
    /*        if (this->meshType == Tool_PATH && node->resampleChecked) {
                _changeValueToColor(0.15, 0, node->rotationabgle, rr, gg, bb);
            }*/

            _changeValueToColor(1, 0, node->scalarField, rr, gg, bb);
            if (this->meshType == Tool_PATH && node->resampleChecked) {
                _changeValueToColor(0.036, 0.012, node->printvelocity, rr, gg, bb);
            }

            if (this->meshType == Tool_PATH && mesh->showselection) {
 
                if (node->selected) {
                    rr = 1; gg = 0.0; bb = 0.0;
                }
                else if (node->selected1) {
                    rr = 0.0; gg = 0.8; bb = 0.0;
                }
                else if (node->selected2) {
                    rr = 0.0; gg = 0.0; bb = 0.8;
                }
                else if (node->selected3) {
                    rr = 0.0; gg = 0.8; bb = 0.8;
                }
                else if (node->selected4) {
                    rr = 0.8; gg = 0.8; bb = 0.0;
                }
                else if (node->selected5) {
                    rr = 0.8; gg = 0.0; bb = 0.8;
                }
                else if (node->selected6) {
                    rr = 0.8; gg = 0.8; bb = 0.8;
                }

                else if (node->isFixed) {
                    rr = 0.8; gg = 0.8; bb = 0.0;
                }
                else if (node->isHandle) {
                    rr = 0.8; gg = 0.0; bb = 0.8;
                }
                else
                {
                    rr = 0.0; gg = 0.0; bb = 0.0;

                }
            }
            if (node->selected) {
                rr = 1; gg = 0.0; bb = 0.0;
            }
            else if (node->handleindex==1) {
                rr = 0.0; gg = 0.8; bb = 0.0;
            }
            else if (node->handleindex==2) {
                rr = 0.0; gg = 0.0; bb = 0.8;
            }
            else if (node->handleindex==3) {
                rr = 0.0; gg = 0.8; bb = 0.8;
            }
            else if (node->isFixed) {
                rr = 0.8; gg = 0.8; bb = 0.0;
            }
            else if (node->handleindex==0) {
                rr = 0.8; gg = 0.0; bb = 0.8;
            }
            else
            {
                rr = 0.0; gg = 0.0; bb = 0.0;

            }
              //  _changeValueToColor(0.9, 0.4, node->layerheight, rr, gg, bb);

            //}
            //    if (mesh->drawThisIsoLayer) {
            //        if (node->terminalnode) {
            //            rr = 0.8; gg = 0.0; bb = 0.0;
            //        }
            //        else if (node->terminalnodemovp) {
            //            rr = 0.0; gg = 1.0; bb = 0.0;
            //        }
            //        else
            //        {
            //            rr = 0.8; gg = 0.8; bb = 0.8;

            //        }
  
            //}
          //  if (node->unseenode) continue;
            if (this->meshType == CURVED_LAYER && mesh->drawlayerheight)
            {

               _changeValueToColor(1.2, 0.2, node->layerheight, rr, gg, bb);//1.8-0.4


            }
            glColor3f(rr, gg, bb);
            node->SetColor(rr, gg, bb);

            node->GetNormal(nx, ny, nz);
            node->GetCoord3D(xx, yy, zz);
            glNormal3d(nx, ny, nz);
            glVertex3d(xx, yy, zz);
        }
    }
    glEnd();
    glEndList();
}

void PolygenMesh::_buildDrawProfileList()
{
  
    if (meshList.GetCount()==0) return;

    glNewList(m_drawListID+3, GL_COMPILE);
  
    for (GLKPOSITION PosMesh = meshList.GetHeadPosition(); PosMesh != NULL;) {
        QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(PosMesh));

        glLineWidth(1.0);

        // if (this->meshType == CURVED_LAYER && mesh->drawThisIsoLayer == false) continue;

        glColor3f(0.3, 0.3, 0.3);

        if (mesh->GetEdgeNumber() == 0) continue;
        double edgeLength = 0;
        for (GLKPOSITION Pos_stressE = mesh->GetEdgeList().GetHeadPosition(); Pos_stressE != NULL;) {
            QMeshEdge* Edge = (QMeshEdge*)(mesh->GetEdgeList().GetNext(Pos_stressE));
            edgeLength += Edge->CalLength();
        }
        edgeLength /= mesh->GetEdgeNumber(); // average edge length

        /*--------Draw the principle stress vector--------*/
        if (this->meshType == CURVED_LAYER)  // for layer
        {
            //for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
            //    QMeshNode* node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
            //    double length = edgeLength*3;

            //    //if (Tet->vectorField.norm() < 0.01) continue;
            //    double pos[3], n[3];
            //    //  for (int i = 0; i < 3; i++) n[i] = Tet->vectorField(i);
            //    for (int i = 0; i < 3; i++) n[i] = node->WeightVector[i]*10;

            //    //---- Draw Array (BODY) ----//
            //    glColor3f(0.5, 0.5, 0.5);

            //    node->GetCoord3D(pos);

            //    glBegin(GL_LINES);
            //    glVertex3d(pos[0], pos[1], pos[2]);
            //    glVertex3d(pos[0] + n[0] * length, pos[1] + n[1] * length, pos[2] + n[2] * length);

            //    glEnd();

            //    //---- Draw Array (TIP) ----//

            //    //this->drawSingleArrayTip(x, y, z, x + n[0] * length, y + n[1] * length, z + n[2] * length);
            //    double endPoint[3] = { pos[0] + n[0] * length, pos[1] + n[1] * length, pos[2] + n[2] * length };
            //    drawSingleArrayTip(endPoint, n, length / 10);
            //}
        }



        if (mesh->drawVectorField) {

            //for (GLKPOSITION Pos = mesh->GetTetraList().GetHeadPosition(); Pos;) {
            //    QMeshTetra* Tet = (QMeshTetra*)mesh->GetTetraList().GetNext(Pos);
            //    double length = edgeLength;
            //    //spase drawing vector field
            //    if (Tet->GetIndexNo() % 10 != 0) continue;
            //    //if (Tet->vectorField.norm() < 0.01) continue;
            //    double pos[3], n[3];
            //  //  for (int i = 0; i < 3; i++) n[i] = Tet->vectorField(i);
            //    for (int i = 0; i < 3; i++) n[i] = Tet->guidevector(i);

            //    //---- Draw Array (BODY) ----//
            //    glColor3f(0.5, 0.5, 0.5);

            //    Tet->CalCenterPos(pos[0], pos[1], pos[2]);

            //    glBegin(GL_LINES);
            //    glVertex3d(pos[0], pos[1], pos[2]);
            //    glVertex3d(pos[0] + n[0] * length, pos[1] + n[1] * length, pos[2] + n[2] * length);

            //    glEnd();

            //    //---- Draw Array (TIP) ----//

            //    //this->drawSingleArrayTip(x, y, z, x + n[0] * length, y + n[1] * length, z + n[2] * length);
            //    double endPoint[3] = { pos[0] + n[0] * length, pos[1] + n[1] * length, pos[2] + n[2] * length };
            //    drawSingleArrayTip(endPoint, n, length / 2);

            //}
        }

        if (this->meshType == CURVED_LAYER)
        {
            for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos;) {
                QMeshFace* Face = (QMeshFace*)mesh->GetFaceList().GetNext(Pos);
                double length = edgeLength;
                //spase drawing vector field
                //if (Tet->vectorField.norm() < 0.01) continue;
                double pos[3], n[3];
                //  for (int i = 0; i < 3; i++) n[i] = Tet->vectorField(i);
                for (int i = 0; i < 3; i++) n[i] = Face->fieldVec(i);

                //---- Draw Array (BODY) ----//
                glColor3f(0.5, 0.5, 0.5);

                Face->CalCenterPos(pos[0], pos[1], pos[2]);

                glBegin(GL_LINES);
                glVertex3d(pos[0], pos[1], pos[2]);
                glVertex3d(pos[0] + n[0] * length, pos[1] + n[1] * length, pos[2] + n[2] * length);

                glEnd();

                //---- Draw Array (TIP) ----//

                //this->drawSingleArrayTip(x, y, z, x + n[0] * length, y + n[1] * length, z + n[2] * length);
                double endPoint[3] = { pos[0] + n[0] * length, pos[1] + n[1] * length, pos[2] + n[2] * length };
                drawSingleArrayTip(endPoint, n, length / 2);

                //}
            }
        }
        if (false) {
            double n[3], length, pos[3]; float rr, gg, bb;
            glLineWidth(1.0);

            double edgeLength = 0;
            for (GLKPOSITION Pos_stressE = mesh->GetEdgeList().GetHeadPosition(); Pos_stressE != NULL;) {
                QMeshEdge* Edge = (QMeshEdge*)(mesh->GetEdgeList().GetNext(Pos_stressE));
                edgeLength += Edge->CalLength();
            }
            edgeLength /= mesh->GetEdgeNumber(); // average edge length
            length = edgeLength;
            for (GLKPOSITION tetpos = mesh->GetTetraList().GetHeadPosition(); tetpos != nullptr;) {
                QMeshTetra* tet = (QMeshTetra*)mesh->GetTetraList().GetNext(tetpos);

                for (int i = 0; i < 3; i++) n[i] = tet->vectorField(i);

                Eigen::Vector3d possum;
                Eigen::Vector3d pos3d;
                for (int i = 0; i < 4; i++)
                {
                    QMeshNode* node = tet->GetNodeRecordPtr(i + 1);
                    node->GetCoord3D(pos3d);
                    possum = possum + pos3d;
                }
                possum = possum / 4;
                for (int i = 0; i < 3; i++) {

                    pos[i] = possum(i);
                }

                //if (fabs(face->fieldVec.norm()) < 0.001) continue;

                rr = 0.2, gg = 0.2, bb = 0.2;

                //length = 100 * edgeLength * face->fieldVec.norm();

                //---- Draw Array (BODY) ----//
                glColor3f(rr, gg, bb);

                glBegin(GL_LINES);
                glVertex3d(pos[0], pos[1], pos[2]);
                glVertex3d(pos[0] + n[0] * length, pos[1] + n[1] * length, pos[2] + n[2] * length);

                glEnd();

                //---- Draw Array (TIP) ----//

                //this->drawSingleArrayTip(x, y, z, x + n[0] * length, y + n[1] * length, z + n[2] * length);
                double endPoint[3] = { pos[0] + n[0] * length, pos[1] + n[1] * length, pos[2] + n[2] * length };
                drawSingleArrayTip(endPoint, n, length / 2);
            }

        }
    }
    glEndList();

}

void PolygenMesh::_buildDrawFaceNormalList()
{
    if (meshList.GetCount()==0) return;

    glNewList(m_drawListID+4, GL_COMPILE);
    glDisable(GL_LIGHTING);

    glColor3f(0.5, 0.0, 0.5);

    glLineWidth(1.0);

    if (this->meshType == TET) {

        glBegin(GL_LINES);
        for (GLKPOSITION meshPos = meshList.GetHeadPosition(); meshPos != NULL;) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(meshPos));

            if (mesh->visStreamLine == true) {
                std::cout << "streamline.size " << mesh->streamline.size() << std::endl;
                for (int i = 0; i < mesh->streamline.size(); i++) {
                    std::vector<Eigen::Vector3d> singleLine = mesh->streamline[i];
                    for (int j = 0; j < singleLine.size() - 1; j++) {
                        glVertex3d(singleLine[j](0), singleLine[j](1), singleLine[j](2));
                        glVertex3d(singleLine[j + 1](0), singleLine[j + 1](1), singleLine[j + 1](2));
                    }
                }
            }
            else {
                if (mesh->GetEdgeNumber() == 0) break;
                QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetHead();
                double length = edge->CalLength();
                //std::cout << "Face normal length is: " << length << std::endl;
                for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos != NULL;) {
                    QMeshFace* face = (QMeshFace*)(mesh->GetFaceList().GetNext(Pos));

                    // only show the surface normals
                    if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL) continue;

                    double x, y, z, nx, ny, nz;
                    face->CalCenterPos(x, y, z);
                    face->CalPlaneEquation();
                    face->GetNormal(nx, ny, nz);
                    glVertex3d(x, y, z);
                    glVertex3d(x + nx * length, y + ny * length, z + nz * length);
                }
            }
        }
        glEnd();

    }
    else {
        glBegin(GL_LINES);
        for (GLKPOSITION meshPos = meshList.GetHeadPosition(); meshPos != NULL;) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(meshPos));
            
            if (this->meshType == CURVED_LAYER && mesh->drawThisIsoLayer == false) continue;

            //kaguya
            if (mesh->visStreamLine == true) {
                for (int i = 0; i < mesh->streamline.size(); i++) {
                    std::vector<Eigen::Vector3d> singleLine = mesh->streamline[i];
                    for (int j = 0; j < singleLine.size()-1; j++) {
                        glVertex3d(singleLine[j](0), singleLine[j](1), singleLine[j](2));
                        glVertex3d(singleLine[j + 1](0), singleLine[j + 1](1), singleLine[j + 1](2));
                    }
                }
            }

            else {
                if (mesh->GetEdgeNumber() == 0) break;
                QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetHead();
                double length = edge->CalLength();
                for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos != NULL;) {
                    QMeshFace* face = (QMeshFace*)(mesh->GetFaceList().GetNext(Pos));
                    double x, y, z, nx, ny, nz;
                    face->CalCenterPos(x, y, z);
                    face->CalPlaneEquation();
                    face->GetNormal(nx, ny, nz);
                    glVertex3d(x, y, z);
                    glVertex3d(x + nx * length, y + ny * length, z + nz * length);
                }
            }
            
        }
        glEnd();
    }

    glEndList();
}

void PolygenMesh::_buildDrawNodeNormalList()
{
    if (meshList.GetCount()==0) return;

    glNewList(m_drawListID+5, GL_COMPILE);
    glDisable(GL_LIGHTING);

    glColor3f(0.0, 0.5, 0.0);

    glLineWidth(1.0);

    glBegin(GL_LINES);
    for (GLKPOSITION meshPos = meshList.GetHeadPosition(); meshPos != NULL;) {
        QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(meshPos));
        //if (mesh->GetEdgeNumber() == 0) break;
        if (this->meshType == CURVED_LAYER && mesh->drawThisIsoLayer == false) continue;
        if (this->meshType == Tool_PATH && mesh->drawThisIsoLayer == false) continue;

        QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetHead();
        double length = edge->CalLength();
        //double length = 1.0;
        for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos != NULL;) {
            QMeshNode* node = (QMeshNode*)(mesh->GetNodeList().GetNext(Pos));
            if (this->meshType == Tool_PATH && node->unseenode) {
                continue;
            }
            double x, y, z;
            node->GetCoord3D(x, y, z);
            double n[3];
            node->GetNormal(n[0],n[1],n[2]);

            glVertex3d(x, y, z);
            glVertex3d(x + n[0] * length, y + n[1] * length, z + n[2] * length);
        }
    }
    glEnd();
    
    glEndList();
}

void PolygenMesh::_changeValueToColor(int nType, float& nRed, float& nGreen, float& nBlue)
{
    float color[][3] = {
        {220,20,60},
        {107,200,35},
        {30,144,255},
        {255,105,180},
        {244,164,96},
        {176,196,222},
        {255,100,70},
        {128,255,128},
        {128,128,255},
        {255,255,128},
        {0,128,0},
        {255,128,255},
        {255,214,202},
        {128,128,192},
        {255,165,0}, //orange
        {255,128,192},
        //		{39, 64, 139},//RoyalBlue
                {128,128,64},
                {0,255,255},
                {238,130,238},//violet
                {220,220,220},//gainsboro
                {188, 143, 143}, // rosy brown
                {46, 139, 87},//sea green
                {210, 105, 30 },//chocolate
                {237, 150, 100},
                {100, 149, 237},//cornflower blue
                {243, 20, 100},
                // 26th
                {0,0,0}
    };

    //	printf("%d ",nType);
    nRed = color[nType % 25][0] / 255.0f;
    nGreen = color[nType % 25][1] / 255.0f;
    nBlue = color[nType % 25][2] / 255.0f;
}


void PolygenMesh::drawOriginalCoordinate() {
	double axisLeng = 30.0;
	glLineWidth(2.0);
	glBegin(GL_LINES);

	// X-axis - Red Color
	glColor3f(1.0, 0.0, 0.0); glVertex3d(0.0, 0.0, 0.0);
	glVertex3d(axisLeng, 0.0, 0.0);

	// Y-axis - green Color
	glColor3f(0.0, 1.0, 0.0);
	glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, axisLeng, 0.0);

	// Z-axis - black Color
	glColor3f(0.0, 0.0, 0.0);
	glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, 0.0, axisLeng);

	//glColor3f(1, 0.1, 0.1);glVertex3d(0.0, 0.0, 0.0);
	//glVertex3d(-7.72805,- 11.4536,- 3.48036); //voxel searching debug

	glEnd();

	
}

void PolygenMesh::drawShade()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID);
}

void PolygenMesh::drawMesh()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+1);
}

void PolygenMesh::drawNode()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+2);
}

void PolygenMesh::drawProfile()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+3);
}

void PolygenMesh::drawFaceNormal()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+4);
}

void PolygenMesh::drawNodeNormal()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+5);
}

void PolygenMesh::ClearAll()
{
    GLKPOSITION Pos;

    for(Pos=meshList.GetHeadPosition();Pos!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(Pos));
        delete mesh;
    }
    meshList.RemoveAll();
}

void PolygenMesh::computeRange()
{
    double range=0.0,ll,xx,yy,zz;
    GLKPOSITION Pos;
    GLKPOSITION PosNode;

    for(Pos=meshList.GetHeadPosition();Pos!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(Pos));
        for(PosNode=(mesh->GetNodeList()).GetHeadPosition();PosNode!=NULL;) {
            QMeshNode *node=(QMeshNode *)((mesh->GetNodeList()).GetNext(PosNode));

            node->GetCoord3D(xx,yy,zz);
            ll=xx*xx+yy*yy+zz*zz;

            if (ll>range) range=ll;
        }
    }

    m_range=(float)(sqrt(range));
}

void PolygenMesh::_changeValueToColor(double maxValue, double minValue, double Value,
                                 float & nRed, float & nGreen, float & nBlue)
{
//	Value=fabs(Value);

    if (Value<minValue)
    {
        nRed=0.0;
        nGreen=0.0;
        nBlue=0.0;
       
        return;
    }

    if ((maxValue-minValue)<0.000000000001)
    {
        nRed=0.0;
        nGreen=0.0;
        nBlue=1.0;
        return;
    }

    double temp=(Value-minValue)/(maxValue-minValue);

//    nRed=(float)(1.0-temp);	nGreen=(float)(1.0-temp); nBlue=(float)(1.0-temp);	return;

    if (temp>0.75)
    {
        nRed=1;
        nGreen=(float)(1.0-(temp-0.75)/0.25);
        if (nGreen<0) nGreen=0.0f;
        nBlue=0;
        return;
    }
    if (temp>0.5)
    {
        nRed=(float)((temp-0.5)/0.25);
        nGreen=1;
        nBlue=0;
        return;
    }
    if (temp>0.25)
    {
        nRed=0;
        nGreen=1;
        nBlue=(float)(1.0-(temp-0.25)/0.25);
        return;
    }
    else
    {
        nRed=0;
        nGreen=(float)(temp/0.25);
        nBlue=1;
    }

 
}

void PolygenMesh::ImportOBJFile(char *filename, std::string modelName)
{
    QMeshPatch *newMesh = new QMeshPatch;
    if (newMesh->inputOBJFile(filename)){
        meshList.AddTail(newMesh);
        computeRange();
        setModelName(modelName);
    }
    else
        delete newMesh;
}

void PolygenMesh::ImportTETFile(char *filename, std::string modelName)
{
	QMeshPatch *newMesh = new QMeshPatch;
	if (newMesh->inputTETFile(filename, false)) {
		meshList.AddTail(newMesh);
		computeRange();
		setModelName(modelName);
	}
	else
		delete newMesh;
}

void PolygenMesh::drawSingleEdge(QMeshEdge* edge) {

    double xx, yy, zz;
    edge->GetStartPoint()->GetCoord3D(xx, yy, zz);
    glVertex3d(xx, yy, zz);
    edge->GetEndPoint()->GetCoord3D(xx, yy, zz);
    glVertex3d(xx, yy, zz);

}

void PolygenMesh::drawSingleNode(QMeshNode* node) {

    double nx, ny, nz, xx, yy, zz;
    node->GetNormal(nx, ny, nz);
    node->GetCoord3D(xx, yy, zz);
    glNormal3d(nx, ny, nz);
    glVertex3d(xx, yy, zz);

}

void PolygenMesh::drawSingleFace(QMeshFace* face) {

    double xx, yy, zz, dd;
    for (int i = 0; i < 3; i++) {
        QMeshNode* node = face->GetNodeRecordPtr(i);

        if (m_bVertexNormalShading) {
            double normal[3];
            node->CalNormal(normal); glNormal3dv(normal);
        }
        else {
            face->CalPlaneEquation();
            face->GetPlaneEquation(xx, yy, zz, dd);
            glNormal3d(xx, yy, zz);
        }

        node->GetCoord3D(xx, yy, zz);
        glVertex3d(xx, yy, zz);
    }
}

void PolygenMesh::drawSingleArrayTip(double pp[3], double dir[3], double arrowLength) {

    double bone_hight = 0.3 * arrowLength;
    double bone_radius = 0.06 * arrowLength;

    Eigen::Vector3d endPP = { pp[0],pp[1],pp[2] };

    Eigen::Vector3d A = { dir[0],dir[1],dir[2] };
    Eigen::Vector3d B = { 0, 1.0, 0 };

    Eigen::Matrix3d rotationMatrix;
    rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(B, A);

    Eigen::Vector3d pp1 = { bone_radius * sin(0) , 0.0 , bone_radius * cos(0) };
    Eigen::Vector3d pp2 = { bone_radius * sin(120 * 3.14 / 180) , 0.0 , bone_radius * cos(120 * 3.14 / 180) };
    Eigen::Vector3d pp3 = { bone_radius * sin(240 * 3.14 / 180) , 0.0 , bone_radius * cos(240 * 3.14 / 180) };
    Eigen::Vector3d ppCenter = { 0.0, bone_hight, 0.0 };

    pp1 = rotationMatrix * pp1 + endPP;
    pp2 = rotationMatrix * pp2 + endPP;
    pp3 = rotationMatrix * pp3 + endPP;
    ppCenter = rotationMatrix * ppCenter + endPP;

    glBegin(GL_TRIANGLES);

    glColor3f(0.9, 0.1, 0.1);

    glVertex3d(pp1(0), pp1(1), pp1(2));
    glVertex3d(pp2(0), pp2(1), pp2(2));
    glVertex3d(ppCenter(0), ppCenter(1), ppCenter(2));

    glVertex3d(pp2(0), pp2(1), pp2(2));
    glVertex3d(pp3(0), pp3(1), pp3(2));
    glVertex3d(ppCenter(0), ppCenter(1), ppCenter(2));

    glVertex3d(pp3(0), pp3(1), pp3(2));
    glVertex3d(pp1(0), pp1(1), pp1(2));
    glVertex3d(ppCenter(0), ppCenter(1), ppCenter(2));

    glEnd();

    //glBegin(GL_QUAD_STRIP);
    //for (int i = 0; i <= 360; i += 60) {
    //	double p = i * 3.14 / 180;
    //	glColor3f(0.9, 0.1, 0.1);

    //	Eigen::Vector3d pp1 = { bone_radius * sin(p) , 0.0 , bone_radius * cos(p) };
    //	Eigen::Vector3d pp2 = { 0.0, bone_hight, 0.0 };

    //	pp1 = rotationMatrix * pp1  + endPP;
    //	pp2 = rotationMatrix * pp2 + endPP;

    //	glVertex3d(pp1(0), pp1(1), pp1(2));
    //	glVertex3d(pp2(0), pp2(1), pp2(2));
    //	/*glVertex3d(bone_radius * sin(p) * dir[0] + pp[0], bone_radius * cos(p) * dir[1] + pp[1], pp[2]);
    //	glVertex3d(pp[0], bone_hight * dir[1] + pp[1], pp[2]);*/
    //}
    //glEnd();

}