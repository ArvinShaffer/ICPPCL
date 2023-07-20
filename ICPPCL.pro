QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
#QMAKE_CXXFLAGS += "-fno-sized-deallocation"
#QMAKE_CXXFLAGS += -DBOOST_NO_CXX11_SCOPED_ENUMS
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += /usr/local/include/vtk-9.1 /usr/local/include/pcl-1.13  /usr/local/include/opencv4 /usr/include/eigen3 /usr/local/include/opencascade


#LIBS += -L/usr/lib/x86_64-linux-gnu/ \
#-lboost_filesystem -lboost_system


LIBS += -L/usr/local/lib   \
-lvtkFiltersHyperTree-9.1        -lvtkImagingFourier-9.1        -lvtkIOLSDyna-9.1        -lvtkRenderingAnnotation-9.1 \
-lvtkChartsCore-9.1                   -lvtkFiltersImaging-9.1          -lvtkImagingGeneral-9.1        -lvtkIOMINC-9.1          -lvtkRenderingContext2D-9.1 \
-lvtkCommonColor-9.1                  -lvtkFiltersModeling-9.1         -lvtkImagingHybrid-9.1         -lvtkIOMovie-9.1         -lvtkRenderingContextOpenGL2-9.1 \
-lvtkCommonComputationalGeometry-9.1  -lvtkFiltersParallel-9.1         -lvtkImagingMath-9.1           -lvtkIONetCDF-9.1        -lvtkRenderingCore-9.1 \
-lvtkCommonCore-9.1                   -lvtkFiltersParallelImaging-9.1  -lvtkImagingMorphological-9.1  -lvtkIOParallel-9.1      -lvtkRenderingFreeType-9.1 \
-lvtkCommonDataModel-9.1              -lvtkFiltersPoints-9.1           -lvtkImagingSources-9.1        -lvtkIOParallelXML-9.1   -lvtkRenderingGL2PSOpenGL2-9.1 \
-lvtkCommonExecutionModel-9.1         -lvtkFiltersProgrammable-9.1     -lvtkImagingStatistics-9.1     -lvtkIOPLY-9.1           -lvtkRenderingImage-9.1 \
-lvtkCommonMath-9.1                   -lvtkFiltersSelection-9.1        -lvtkImagingStencil-9.1        -lvtkIOSQL-9.1           -lvtkRenderingLabel-9.1 \
-lvtkCommonMisc-9.1                   -lvtkFiltersSMP-9.1              -lvtkInfovisCore-9.1           -lvtkIOTecplotTable-9.1  -lvtkRenderingLOD-9.1 \
-lvtkCommonSystem-9.1                 -lvtkFiltersSources-9.1          -lvtkInfovisLayout-9.1         -lvtkIOVideo-9.1         -lvtkRenderingOpenGL2-9.1 \
-lvtkCommonTransforms-9.1             -lvtkFiltersStatistics-9.1       -lvtkInteractionImage-9.1      -lvtkIOXML-9.1           -lvtkRenderingQt-9.1 \
-lvtkDICOMParser-9.1                  -lvtkFiltersTexture-9.1          -lvtkInteractionStyle-9.1      -lvtkIOXMLParser-9.1     -lvtkRenderingVolume-9.1 \
-lvtkDomainsChemistry-9.1             -lvtkFiltersTopology-9.1         -lvtkInteractionWidgets-9.1    -lvtkjpeg-9.1            -lvtkRenderingVolumeOpenGL2-9.1 \
-lvtkDomainsChemistryOpenGL2-9.1      -lvtkFiltersVerdict-9.1          -lvtkIOAMR-9.1                 -lvtkjsoncpp-9.1         -lvtksqlite-9.1 \
-lvtkfreetype-9.1                -lvtkIOCore-9.1                -lvtklibharu-9.1         -lvtksys-9.1 \
-lvtkexpat-9.1                        -lvtkGeovisCore-9.1              -lvtkIOEnSight-9.1             -lvtklibxml2-9.1         -lvtktiff-9.1 \
-lvtkFiltersAMR-9.1                   -lvtkgl2ps-9.1                   -lvtkIOExodus-9.1              -lvtklz4-9.1             -lvtkverdict-9.1 \
-lvtkFiltersCore-9.1                  -lvtkglew-9.1                                -lvtkmetaio-9.1          -lvtkViewsContext2D-9.1 \
-lvtkFiltersExtraction-9.1            -lvtkGUISupportQt-9.1                         -lvtkViewsCore-9.1 \
-lvtkFiltersFlowPaths-9.1             -lvtkGUISupportQtSQL-9.1         -lvtkIOGeometry-9.1                 -lvtkViewsInfovis-9.1 \
-lvtkFiltersGeneral-9.1               -lvtkhdf5-9.1                    -lvtkIOImage-9.1                      -lvtkViewsQt-9.1 \
-lvtkFiltersGeneric-9.1               -lvtkhdf5_hl-9.1                 -lvtkIOImport-9.1              -lvtkParallelCore-9.1    -lvtkzlib-9.1 \
-lvtkFiltersGeometry-9.1              -lvtkImagingColor-9.1            -lvtkIOInfovis-9.1             -lvtkpng-9.1 \
-lvtkFiltersHybrid-9.1                -lvtkImagingCore-9.1             -lvtkIOLegacy-9.1               \
-lopencv_calib3d -lopencv_core -lopencv_dnn -lopencv_features2d -lopencv_flann -lopencv_gapi -lopencv_highgui -lopencv_imgcodecs  \
-lpcl_common    -lpcl_io_ply  -lpcl_keypoints  -lpcl_outofcore    -lpcl_registration      -lpcl_segmentation  -lpcl_tracking \
-lpcl_io      -lpcl_ml         -lpcl_people       -lpcl_sample_consensus  -lpcl_stereo        -lpcl_visualization \
-lpcl_filters   -lpcl_kdtree  -lpcl_octree     -lpcl_recognition  -lpcl_search            -lpcl_surface -lpcl_features \
#-lvtkalglib-9.1   -lvtkIOExport-9.1 -lvtkNetCDF-9.1   -lvtknetcdfcpp-9.1 -lvtkoggtheora-9.1 -lvtkproj4-9.1 -lvtkexoIIc-9.1  -lvtkIOExportOpenGL2-9.1

SOURCES += \
    main.cpp \
    icppcl.cpp

HEADERS += \
    icppcl.h

FORMS += \
    icppcl.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    icppcl.qrc
