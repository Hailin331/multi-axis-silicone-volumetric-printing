#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSignalMapper>
#include <QStandardItemModel>
#include "../GLKLib/GLKLib.h"
#include "../QMeshLib/PolygenMesh.h"
#include <omp.h>
#include <QTimer>
#include <QLabel>
#include<fileIO.h>
 #include<DeformToolPath.h>
#include<posiontoolpath.h>
#include <fstream>
#include "dirent.h"
#include <memory> // Required for std::make_unique
#include <chrono>
#include <future>
#include <thread>

#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x

using namespace std;

class DeformTet;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
	// Qtimer - defined function
    //void doTimerGcodeMoving();

private:
    Ui::MainWindow *ui;
    GLKLib *pGLK;

    /* add for Gcode generation */
    //QTimer Gcode_timer; //Gcode Simulation timer
    //int gocodetimerItertime;
    //int simuLayerInd;
    //Eigen::MatrixXf Gcode_Table;
    //unsigned int operationTime = 0;
    /* ------------------------ */
	GLKObList polygenMeshList;

private:
    void createActions();
    void createTreeView();

    PolygenMesh *getSelectedPolygenMesh();

    QSignalMapper *signalMapper;
    QStandardItemModel* treeModel;
	DeformTet *Deformation;

private:
    fileIO* fileIOObject;
protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);

private slots:
    void open();
    void save();
	void saveSelection();
	void readSelection();
    void changeIsoLayerDisplay();
    PolygenMesh* _buildPolygenMesh(mesh_type type, std::string name);

    void signalNavigation(int flag);
    void shiftToOrigin();
    void updateTree();
	void mouseMoveEvent(QMouseEvent *event);
    void on_pushButton_clearAll_clicked();
    void on_treeView_clicked(const QModelIndex &index);
   
 
     void ReadISOLayerandLHeight();
     void Compute_ISO_Surface();
    void outputIsoLayer();
    void outputIsoLayersetandlayerheight();
     void _buildFileNameSetbySorting(std::vector<std::string>& files, std::string fieldPath);
      void DeformationToolpath();
      void ReadmeshandInfo();
    //  double getv(double height);
    PolygenMesh* MainWindow::_detectPolygenMesh(mesh_type type);
};

#endif // MAINWINDOW_H
