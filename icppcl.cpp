#include "icppcl.h"
#include "ui_icppcl.h"

ICPPCL::ICPPCL(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::ICPPCL)
{
    ui->setupUi(this);
    QObject::connect(ui->actionOpen, &QAction::triggered, this, &ICPPCL::open);

    //Sampling
    QObject::connect(ui->actionUniform_Sampling, &QAction::triggered, this, &ICPPCL::uniformSampling);

    QObject::connect(ui->dataTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(itemSelected(QTreeWidgetItem*, int)));
    QObject::connect(ui->dataTree, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(itemPopmenu(const QPoint&)));

    //point cloud initialization
    cloud_now.reset(new PointCloudT);

    //visualization
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
}

void ICPPCL::open()
{
    QStringList files = QFileDialog::getOpenFileNames(this, tr("Open point cloud file"), QString::fromLocal8Bit(model_dirname.c_str()), tr("Point cloud data(*.pcd *.ply *.obj);;All file(*.*)"));

    if (files.isEmpty()) return;

    viewer->removeAllPointClouds();

    PointCloudT::Ptr c_temp;

    for (int i = 0; i != files.size(); ++i) {
        c_temp.reset(new PointCloudT);
        QString file = files[i];
        std::string file_s = file.toStdString();
        std::string subname = getFileName(file_s);

        ui->statusbar->showMessage(QString::fromLocal8Bit(subname.c_str()) + ": " + QString::number(i) + "/" + QString::number(files.size()) + " point cloud loading...");

        int status = -1;
        if (file.endsWith(".pcd", Qt::CaseInsensitive))
        {
            status = pcl::io::loadPCDFile(file_s, *c_temp);
        }
        else if (file.endsWith(".ply", Qt::CaseInsensitive))
        {
            status = pcl::io::loadPLYFile(file_s, *c_temp);
        }
        else if (file.endsWith(".obj", Qt::CaseInsensitive))
        {
            status = pcl::io::loadOBJFile(file_s, *c_temp);
        }
        else
        {
            QMessageBox::information(this, tr("File form error"), tr("Can't open files except .ply .pcd .obj"));
        }
        if (status != 0)
        {
            QMessageBox::critical(this, tr("Reading file error"), tr("We can not open the file"));
            return ;
        }

        if (c_temp->points[0].r == 0 && c_temp->points[0].g == 0 && c_temp->points[0].b == 0)
        {
            setCloudRGB(c_temp, 255, 255, 255);
        }
        setCloudAlpha(c_temp, 255);

        cloud_show.push_back(c_temp);
        total_points += c_temp->points.size();
    }
    setConsole("Load Cloud(s)", "We now have " + QString::number(total_points) + " Points.");
    ui->statusbar->showMessage("Load Point Cloud Done.");
    updatePointcloud();
}

void ICPPCL::itemSelected(QTreeWidgetItem* item, int count)
{
    count = ui->dataTree->indexOfTopLevelItem(item);
    for (int i = 0; i != cloud_show.size(); ++i)
    {
        viewer->updatePointCloud(cloud_show[i], "cloud" + QString::number(i).toStdString());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud" + QString::number(i).toStdString());
    }
    *cloud_now = *cloud_show[count];

    //The size of the point cloud corresponding to the selected tiem becomes larger
    QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
    int selected_item_count = ui->dataTree->selectedItems().size();
    for (int i = 0; i != selected_item_count; ++i) {
        int cloud_id = ui->dataTree->indexOfTopLevelItem(itemList[i]);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud" + QString::number(cloud_id).toStdString());
    }
    ui->qvtkWidget->update();
    setConsole("itemSelected", QString::number(selected_item_count) + " item(s) selected.");
}


void ICPPCL::itemPopmenu(const QPoint &)
{
    QTreeWidgetItem* curItem = ui->dataTree->currentItem(); //Get the currently clicked node
    if (curItem == NULL) return ; //In this case, the position of the right button is not within the scope of the treeItem, that is, right-clicking in an empty position

    QString name = curItem->text(0);
    int id = ui->dataTree->indexOfTopLevelItem(curItem);
    std::string cloud_id = "cloud " + QString::number(id).toStdString();

    QAction hideItemAction("Hide", this);
    QAction showItemAction("show", this);
    QAction changeColorAction("Change color", this);
    QAction randomColorAction("Random color", this);

    connect(&hideItemAction, &QAction::triggered, this, &ICPPCL::pointHide);
    connect(&showItemAction, &QAction::triggered, this, &ICPPCL::pointShow);
    connect(&changeColorAction, &QAction::triggered, this, &ICPPCL::pointcolorChanged);
    connect(&randomColorAction, &QAction::triggered, this, &ICPPCL::pointcolorRandom);

    QMenu menu(ui->dataTree);
    menu.addAction(&hideItemAction);
    menu.addAction(&showItemAction);
    menu.addAction(&changeColorAction);
    menu.addAction(&randomColorAction);

    menu.exec(QCursor::pos()); //Show at current mouse position
    setConsole("popMenu", "popMenu");
}

void ICPPCL::pointcolorChanged()
{
    QColor color = QColorDialog::getColor(Qt::white, this, "Select color for point cloud");

    if (color.isValid()) //Determine if the selected color is valid
    {
        QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
        int selected_item_count = ui->dataTree->selectedItems().size();
        if (selected_item_count == 0) {
            for (int i = 0; i != cloud_show.size(); ++i) {
                for (int j = 0; j != cloud_show[i]->points.size(); ++j) {
                    cloud_show[i]->points[j].r = color.red();
                    cloud_show[i]->points[j].g = color.green();
                    cloud_show[i]->points[j].b = color.blue();
                }
            }
        }
        else {
            for (int i = 0; i != selected_item_count; ++i) {
                int cloud_id = ui->dataTree->indexOfTopLevelItem(itemList[i]);
                for (int j = 0; j != cloud_show[cloud_id]->size(); ++j) {
                    cloud_show[cloud_id]->points[j].r = color.red();
                    cloud_show[cloud_id]->points[j].g = color.green();
                    cloud_show[cloud_id]->points[j].b = color.blue();
                }
            }
            // output window
            setConsole("change cloud color", "to " + QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()));
        }
    }
    updatePointcloud();
}

void ICPPCL::pointcolorRandom()
{
    unsigned int r = 255 * (rand() / (RAND_MAX + 1.0f));
    unsigned int g = 255 * (rand() / (RAND_MAX + 1.0f));
    unsigned int b = 255 * (rand() / (RAND_MAX + 1.0f));
    QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
    int selected_item_count = ui->dataTree->selectedItems().size();
    if (selected_item_count == 0) {
        for (int i = 0; i != cloud_show.size(); ++i) {
            for (int j = 0; j != cloud_show[i]->points.size(); ++j) {
                cloud_show[i]->points[j].r = r;
                cloud_show[i]->points[j].g = g;
                cloud_show[i]->points[j].b = b;
            }
        }

    }
    else {
        for (int i = 0; i != selected_item_count; ++i) {
            int cloud_id = ui->dataTree->indexOfTopLevelItem(itemList[i]);
            for (int j = 0; j != cloud_show[cloud_id]->size(); ++j) {
                cloud_show[cloud_id]->points[j].r = r;
                cloud_show[cloud_id]->points[j].g = g;
                cloud_show[cloud_id]->points[j].b = b;
            }
        }
        // output window
        setConsole("Change cloud color", "to random color." );
    }
    updatePointcloud();
    ui->qvtkWidget->update();
}

void ICPPCL::pointShow()
{
    QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
    int selected_item_count = ui->dataTree->selectedItems().size();
    if (selected_item_count == 0) {
        for (int i = 0; i != cloud_show.size(); ++i) {
            setCloudAlpha(cloud_show[i], 255);
        }
    }
    else {
        for (int i = 0; i != selected_item_count; ++i) {
            int cloud_id = ui->dataTree->indexOfTopLevelItem(itemList[i]);
            setCloudAlpha(cloud_show[cloud_id], 255);
        }
        // output window
        setConsole("Hide", "");
    }
    updatePointcloud();
    ui->qvtkWidget->update();
}


void ICPPCL::pointHide()
{
    QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
    int selected_item_count = ui->dataTree->selectedItems().size();
    if (selected_item_count == 0)
    {
        for (int i = 0; i != cloud_show.size(); ++i)
        {
            setCloudAlpha(cloud_show[i], 0);
        }
    }
    else
    {
        for (int i = 0; i != selected_item_count; ++i)
        {
            int cloud_id = ui->dataTree->indexOfTopLevelItem(itemList[i]);
            setCloudAlpha(cloud_show[cloud_id], 0);
        }
        setConsole("Hide", "");
    }
    updatePointcloud();
    ui->qvtkWidget->update();
}


std::string ICPPCL::getFileName(std::string file_name)
{
    std::string subname;
    for (auto i = file_name.end() - 1; *i != '/'; --i)
    {
        subname.insert(subname.begin(), *i);
    }
    return subname;
}

void ICPPCL::print4x4MatrixT(const Eigen::Matrix4d &matrix)
{
    printf("Rotation matrix : \n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void ICPPCL::setConsole(QString operation, QString detail)
{
    int rows = ui->consoleTable->rowCount();
    ui->consoleTable->setRowCount(++rows);
    QDateTime time = QDateTime::currentDateTime(); //Get the current time of the system
    QString t_str = time.toString("MM-dd hh:mm:ss"); // set display format
    ui->consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(t_str));
    ui->consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(operation));
    ui->consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(detail));

    ui->consoleTable->scrollToBottom();
}

void ICPPCL::keyboardEventOccured(const pcl::visualization::KeyboardEvent &event, void *non)
{
    if (event.getKeySym() == "space" && event.keyDown())
        next = true;
}

void ICPPCL::updatePointcloud()
{
    viewer->removeAllPointClouds();
    for (int i = 0; i != cloud_show.size(); ++i)
    {
        viewer->addPointCloud(cloud_show[i], "cloud " + QString::number(i).toStdString());
        viewer->updatePointCloud(cloud_show[i], "cloud " + QString::number(i).toStdString());
    }
    updateDataTree();
}

void ICPPCL::updateDataTree()
{
    ui->dataTree->clear();

    //Update resource management tree
    for (int i = 0; i != cloud_show.size(); ++i)
    {
        QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit(std::to_string(i).c_str()));
        cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
        ui->dataTree->addTopLevelItem(cloudName);
    }
}

void ICPPCL::setCloudAlpha(PointCloudT::Ptr &cloud, unsigned int a)
{
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        cloud->points[i].a = a;
    }
}

void ICPPCL::setCloudRGB(PointCloudT::Ptr &cloud, unsigned int r, unsigned int g, unsigned int b)
{
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;
        cloud->points[i].a = 255;
    }
}

// Sampling
// uniform sampling

void ICPPCL::uniformSampling()
{
    if (cloud_show.size() < 1)
    {
        setConsole("transformLastCloud", " insufficient point cloud.");
        return ;
    }

    PointCloudT::Ptr cloud_in(cloud_show.back());
    PointCloudT::Ptr us_cloud(new PointCloudT);


    pcl::UniformSampling<pcl::PointXYZRGBA> us;
    us.setInputCloud(cloud_in);
    us.setRadiusSearch(0.005f);
    us.filter(*us_cloud);

    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();  //创建平移旋转变换矩阵。本例只演示平移点云。
    transformation.translation() << 0.1, 0.0, 0.0;   //沿x轴方向平移10m

    pcl::transformPointCloud(*us_cloud, *us_cloud, transformation);  //平移点云


    cloud_show.push_back(us_cloud);
    updatePointcloud();
    ui->qvtkWidget->update();
}




ICPPCL::~ICPPCL()
{
    delete ui;
}

