Platform: Windows + Visual Studio + QT-plugin (tested version: VS2019 + QT5.12.3 + msvc2017_64)

Install Steps:

Install the PyMeshLab

Install Visual Studio Extension plug-in (QT VS Tool) to open the *.pro file and generate the project

Set 'ShapeLab' as the start up project

Enable OpenMP to get best performace at: ShapeLab Property Pages -> Configuration Properties -> C/C++ -> Language -> Open MP Support -> Select 'Yes (/openmp)'

Open Console at: ShapeLab Property Pages -> Configuration Properties -> Linker -> System -> SubSystem -> Select 'Console (/SUBSYSTEM:CONSOLE)'

Support large obj at: ShapeLab Property Pages -> Configuration Properties -> C/C++ -> Command Line -> Additional Options -> add "/bigobj"

Install Intel oneAPI Math Kernel Library (oneMKL download) and enable it at: ShapeLab Property Pages -> Configuration Properties -> Intel Libraries for oneAPI -> Intel oneAPI Math Kernel Library (oneMKL) -> Use oneMKL -> Select 'Parallel'

And change the code generation method at: ShapeLab & QMeshLab & GLKLib Property Pages -> Configuration Properties -> C/C++ -> Code Generation -> Runtime Library -> Select 'Multi-threaded(/MT) for release configuration'. Note that this option will be 'Multi-threaded Debug (/MTd) for debug configuration.

Please ensure the MKL is added into the Environment Variables (e.g. C:\Program Files (x86)\Intel\oneAPI\compiler\2024.1\bin)


Usage

Step1: Click Read Model and selection to input the .tet file

Step2: Click Generate ISO_Surface to slicing

Step3: Click Output ISO_Surface to remesh and output the sliced iso layers. And the remeshed isolayers can also be read by click ReadISOlayerandLheight

step4: Deformation Toolpath to generate toolpath on each layer
