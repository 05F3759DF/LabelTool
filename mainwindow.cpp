#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // 设置 GPS 局部坐标系起点
    coord.SetCoordinateParameters(CoordinateConvertion::_BEIJINGLOCAL);

    // 初始化图像
    img.create(GRID / VELOPIXELSIZE + 1, GRID / VELOPIXELSIZE + 1, CV_8UC3);
    veloImg.create(GRID / VELOPIXELSIZE + 1, GRID / VELOPIXELSIZE + 1, CV_8UC3);
    originMap = cv::Mat(GRID, GRID, CV_8U);
    originMap.setTo(0);
    maskMap = cv::Mat(GRID, GRID, CV_8U);
    maskMap.setTo(0);
    // 把显示 velodyne 点的图像所有点置为特殊值（-1000）
    veloMap = cv::Mat(VELOGRID, VELOGRID, CV_32F);
    for (int i = 0; i < veloMap.rows; i++) {
        for (int j = 0; j < veloMap.cols; j++) {
            veloMap.at<float>(i, j) = -1000;
        }
    }

    // 开始时，数据未初始化
    initialized = false;

    // 颜色表
    colorTable[1] = cv::Scalar(0, 0, 255, 255);
    colorTable[2] = cv::Scalar(102, 217, 255, 255);
    colorTable[3] = cv::Scalar(0, 255, 0, 255);
    colorTable[4] = cv::Scalar(255, 0, 0, 255);

    colorTable[101] = cv::Scalar(255, 255, 0, 25);
    colorTable[102] = cv::Scalar(0, 0, 255, 25);
    colorTable[103] = cv::Scalar(255, 0, 255, 25);
    colorTable[104] = cv::Scalar(255, 0, 128, 25);

    // 当前标注数据的类型
    currentType = 1;
    ui->radioButton_type1->setChecked(true);
    connect(ui->radioButton_blank, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            currentType = 0;
        }
    });
    connect(ui->radioButton_type1, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            currentType = 1;
        }
    });
    connect(ui->radioButton_type2, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            currentType = 2;
        }
    });
    connect(ui->radioButton_type3, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            currentType = 3;
        }
    });
    connect(ui->radioButton_type4, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            currentType = 4;
        }
    });
    connect(ui->radioButton_other, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            int currentOtherIndex = ui->comboBox->currentIndex();
            currentType = currentOtherIndex + 5;
        }
    });

    // 当前绘图的类型
    ui->radioButton_point->setChecked(true);
    graphType = "Point";
    connect(ui->radioButton_point, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            graphType = "Point";
            pointStack.clear();
        }
    });
    connect(ui->radioButton_line, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            graphType = "Line";
            pointStack.clear();
        }
    });
    connect(ui->radioButton_polygon, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            graphType = "Polygon";
            pointStack.clear();
        }
    });

    // 可视化点的形状
    ui->radioButton_circle->setChecked(true);
    pointType = "Circle";
    connect(ui->radioButton_circle, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            pointType = "Circle";
        }
    });
    connect(ui->radioButton_square, &QRadioButton::clicked, [ = ](bool checked) {
        if (checked) {
            pointType = "Square";
        }
    });
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::openFile);
    connect(ui->actionExport, &QAction::triggered, this, &MainWindow::exportFile);
    connect(ui->actionExport_Background, &QAction::triggered, this, &MainWindow::exportBackground);
    connect(ui->actionExport_LabelMap, &QAction::triggered, this, &MainWindow::exportLabelMap);
    connect(ui->actionLoad, &QAction::triggered, this, &MainWindow::loadFile);
    connect(ui->actionScan, &QAction::triggered, this, &MainWindow::scan);
    connect(ui->pushButton_set, &QPushButton::clicked, this, &MainWindow::setTime);
    connect(ui->pushButton_execute, &QPushButton::clicked, this, &MainWindow::execute);

    // 每 100 毫秒调用一次图像更新
    connect(&renderTimer, &QTimer::timeout, this, &MainWindow::updateImage);
    renderTimer.start(100);

    ui->labelMapImage->setMouseTracking(true);
    ui->labelImage->setMouseTracking(true);
    ui->centralWidget->setMouseTracking(true);
    setMouseTracking(true);
}

MainWindow::~MainWindow() {
    delete ui;
}

bool MainWindow::openFile() {
    QString filename = QFileDialog::getOpenFileName(this, "Config", ".", "*.ini");
    if (filename.isEmpty()) {
        return false;
    }

    setting = new QSettings(filename, QSettings::IniFormat, this);
    velodyneFilename = setting->value("/Sensor/velodyne", "").toString();
    gpsFilename = setting->value("/Sensor/gps", "").toString();
    imuFilename = setting->value("/Sensor/imu", "").toString();
    calibFilename = setting->value("/Sensor/velocalib", "db32.xml").toString();
    ladybugFilename = setting->value("/Sensor/ladybug", "").toString();
    ladybugTimeFilename = setting->value("/Sensor/ladybugtime", "").toString();

    if (velodyneFilename.isEmpty() || gpsFilename.isEmpty() || imuFilename.isEmpty() ||
        ladybugFilename.isEmpty() || ladybugTimeFilename.isEmpty()) {
        return false;
    }

    if (!innerCalib.loadCalib(calibFilename.toStdString())) {
        return false;
    }

    ui->comboBox->clear();
    labelGroup.clear();

    labelGroup[0] = "Unknown";
    labelGroup[1] = "Convex";
    labelGroup[2] = "Concave";
    labelGroup[3] = "Free";
    labelGroup[4] = "Water";
    setting->beginGroup("/Label");
    auto keys = setting->childKeys();
    for (auto k: keys) {
        if (k.startsWith("label_")) {
            int index = k.mid(6).toInt();
            if (index < 5) {
                continue;
            }
            QString label = setting->value(k).toString();
            labelGroup[index] = label;
            ui->comboBox->addItem(label);
            QString color = "color_" + QString::number(index);
            if (keys.indexOf(color) != -1) {
                QStringList colorValue = setting->value(keys[keys.indexOf(color)]).toStringList();
                colorTable[index] = cv::Scalar(colorValue[2].toInt(), colorValue[1].toInt(), colorValue[0].toInt(), 255);
            }
        }
    }
    setting->endGroup();
    connect(ui->comboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [ = ]() {
        if (ui->radioButton_other->isChecked()) {
            currentType = ui->comboBox->currentIndex() + 5;
        }
    });
    return true;
}

void MainWindow::exportFile() {
    // 导出被标注的点，CSV 格式，第一列为行，第二列为列，第三列为类型
    QString filename = ui->lineEdit_gpstime->text() + ".csv";
    QVector<QPair<QPoint, int>> pointList;
    for (int i = 0; i < originMap.rows; i++) {
        for (int j = 0; j < originMap.cols; j++) {
            if (originMap.at<uchar>(i, j)) {
                pointList.append(qMakePair(QPoint(i, j), originMap.at<uchar>(i, j)));
            }
        }
    }
    FILE *f = fopen(filename.toStdString().c_str(), "w");
    fprintf(f, "row, col, value\n");
    for (auto p: pointList) {
        fprintf(f, "%d, %d, %d\n", p.first.x(), p.first.y(), p.second);
    }
    fclose(f);

    // 导出被标注的激光图像
    QString imgName = ui->lineEdit_gpstime->text() + ".png";
    cv::imwrite(imgName.toStdString(), img);
    QString camName = ui->lineEdit_gpstime->text() + "_cam.png";
    cv::imwrite(camName.toStdString(), cam);
    QString camRawName = ui->lineEdit_gpstime->text() + "_cam_raw.png";
    cv::imwrite(camRawName.toStdString(), cam_raw);
    QMessageBox::information(this, "Info", QString("Labels have been saved to %1.").arg(filename));
}

void MainWindow::exportBackground() {
    QString imgBackgroundName = ui->lineEdit_gpstime->text() + "_background.png";
    cv::imwrite(imgBackgroundName.toStdString(), imgBackground);

    QMessageBox::information(this, "Info", QString("Background has been saved to %1.").arg(imgBackgroundName));
}

void MainWindow::exportLabelMap() {
    QString labelMapName = "label_map.csv";
    FILE *f = fopen(labelMapName.toStdString().c_str(), "w");
    fprintf(f, "label, value\n");
    for (auto p: labelGroup.keys()) {
        fprintf(f, "%s, %d\n", labelGroup[p].toStdString().c_str(), p);
    }
    fclose(f);

    QMessageBox::information(this, "Info", QString("Label map has been saved to %1.").arg(labelMapName));
}

void MainWindow::loadFile() {
    QString filename = QFileDialog::getOpenFileName(this, "Label", ".", "*.csv");
    if (filename.isEmpty()) {
        return;
    }
    char buf[255];
    int row, col, value;
    FILE *f = fopen(filename.toStdString().c_str(), "r");
    fscanf(f, "%[^\n]s", buf);
    while (fscanf(f, "%d, %d, %d", &row, &col, &value) == 3) {
        setGridMask(originMap, row, col, value);
    }
    fclose(f);
}

void MainWindow::scan() {
    gpsTimeList.clear();
    posList.clear();
    veloPosList.clear();
    veloTimeList.clear();
    cameraTimeList.clear();
    imageFrameList.clear();
    char data[2048];
    int sec, usec, caplen, len;
    int timestamp, nsv1, nsv2;
    double lon, lat, alt, yaw, pitch, roll, tmp;
    FILE *fgps = fopen(gpsFilename.toStdString().c_str(), "r");
    FILE *fimu = fopen(imuFilename.toStdString().c_str(), "r");
    fscanf(fgps, "%[^\n]", data);
    fscanf(fimu, "%[^\n]", data);
    while (!feof(fgps)) {
        fscanf(fgps, "%d, %lf, %lf, %lf, %d, %d", &timestamp, &lat, &lon, &alt, &nsv1, &nsv2);
        fscanf(fimu, "%d, %lf, %lf, %lf, %lf, %lf, %lf", &timestamp, &yaw, &pitch, &roll, &tmp, &tmp, &tmp);
        yaw = yaw / 180.0 * M_PI;
        pitch = pitch / 180.0 * M_PI;
        roll = roll / 180.0 * M_PI;
        double lbh[3], xyz[3];
        lbh[0] = lat;
        lbh[1] = lon;
        lbh[2] = alt;
        coord.GPS84LBH_LocalXYZ(lbh, xyz);
        gpsTimeList.append(timestamp);
        posList.append(Position(xyz[1], xyz[0], xyz[2], yaw, pitch, roll));
    }
    fclose(fgps);
    qDebug() << "GPS is loaded";
    FILE *fvelodyne = fopen(velodyneFilename.toStdString().c_str(), "rb");
    fseeko64(fvelodyne, 0, SEEK_END);
    int64 total = ftello64(fvelodyne);
    //    fread(data, 1, 24, fvelodyne);
    int64 count = 24;
    fseeko64(fvelodyne, count, SEEK_SET);
    while (count < total) {
        fread((char*)&sec, sizeof(int), 1, fvelodyne);
        fread((char*)&usec, sizeof(int), 1, fvelodyne);
        fread((char*)&caplen, sizeof(int), 1, fvelodyne);
        fread((char*)&len, sizeof(int), 1, fvelodyne);
        count += 16;
        if (len != 1248) {
            //            fread(data, 1, len, fvelodyne);
            count += len;
            fseeko64(fvelodyne, count, SEEK_SET);
            continue;
        }
        QDateTime dateTime = QDateTime::fromMSecsSinceEpoch((long long)sec * 1000);
        int time = (long long)dateTime.time().msecsSinceStartOfDay() + usec / 1000;
        veloTimeList.append(time);
        veloPosList.append(count);
        count += 1248;
        //        fread(data, 1, 1248, fvelodyne);
        fseeko64(fvelodyne, count, SEEK_SET);
    }
    fclose(fvelodyne);
    qDebug() << "velodyne is loaded";
    FILE *fladybug = fopen(ladybugTimeFilename.toStdString().c_str(), "r");
    int frame = 0;
    while (!feof(fladybug)) {
        fscanf(fladybug, "%d", &timestamp);
        cameraTimeList.append(timestamp);
        imageFrameList.append(frame++);
    }
    fclose(fladybug);
    video.open(ladybugFilename.toStdString());
    if (!video.isOpened()) {
        qDebug() << "fail to open video!";
        return;
    }
    qDebug() << "ladybug is loaded";
    QMessageBox::information(this, "Info", QString("Files have been loaded."));
}

void MainWindow::setTime() {
    if (ui->lineEdit_gpstime->text().isEmpty()) {
        return;
    }
    int time = ui->lineEdit_gpstime->text().toInt();
    auto pos = std::lower_bound(veloTimeList.begin(), veloTimeList.end(), time);
    if (pos == veloTimeList.end()) {
        return;
    }
    auto index = pos - veloTimeList.begin();
    // 向前数 180 个packet（大约0.1s），向后数 1800 个packet（大约1s）
    // 作为默认可视化的范围
    auto startIndex = index - 180;
    auto endIndex = index + 1800;

    if (startIndex < 0) {
        startIndex = 0;
    }
    if (endIndex >= veloTimeList.size()) {
        endIndex = veloTimeList.size() - 1;
    }
    ui->lineEdit_starttime->setText(QString::number(startIndex));
    ui->lineEdit_endtime->setText(QString::number(endIndex));
}

void MainWindow::execute() {
    originMap.setTo(0);
    maskMap.setTo(0);
    veloImg.setTo(0);
    for (int i = 0; i < veloMap.rows; i++) {
        for (int j = 0; j < veloMap.cols; j++) {
            veloMap.at<float>(i, j) = -1000;
        }
    }
    int time = ui->lineEdit_gpstime->text().toInt();
    int gpsIndex = std::lower_bound(gpsTimeList.begin(), gpsTimeList.end(), time) - gpsTimeList.begin();
    auto currentPos = posList[gpsIndex];
    FILE *fvelodyne = fopen(velodyneFilename.toStdString().c_str(), "rb");
    char data[2048];
    int startIndex = ui->lineEdit_starttime->text().toInt();
    int endIndex = ui->lineEdit_endtime->text().toInt();
    qDebug() << "velotime" << veloTimeList[startIndex] << veloTimeList[endIndex];
    qDebug() << "----------------------------------------";

    for (int index = startIndex; index <= endIndex; index++) {
        int veloTime = veloTimeList[index];
        int gpsIndexForVelo = std::lower_bound(gpsTimeList.begin(), gpsTimeList.end(), veloTime) - gpsTimeList.begin();
        if (gpsIndexForVelo == 0 || gpsIndexForVelo == gpsTimeList.size()) {
            continue;
        }
        auto pointPos = posList[gpsIndexForVelo];
        double deltaX = pointPos.x - currentPos.x;
        double deltaY = pointPos.y - currentPos.y;
        double deltaZ = pointPos.z - currentPos.z;
        X::Matrix rot, rotInv;
        //        X::createRotMatrix_XYZ(rot, pointPos.pitch, pointPos.roll, -pointPos.heading);
        //        X::createRotMatrix_ZYX(rotInv, -currentPos.pitch, -currentPos.roll, currentPos.heading);
        X::createRotMatrix_XYZ(rot, 0, 0, -pointPos.heading);
        X::createRotMatrix_ZYX(rotInv, 0, 0, currentPos.heading);
        fseeko64(fvelodyne, veloPosList[index] + 42, SEEK_SET);
        fread(data, 1, 1200, fvelodyne);

        for (int i = 0; i < 12; i++) {
            int blockStart = i * 100;
            unsigned int blockID = (unsigned char)data[blockStart] + (unsigned char)data[blockStart + 1] * 0x100;
            int blockOffset = blockID == 0xEEFF ? 0 : 32;
            unsigned int rotationData = (unsigned char)data[blockStart + 2] + (unsigned char)data[blockStart + 3] * 0x100;
            double theta = rotationData / 18000.0 * M_PI;
            for (int j = 0; j < 32; j++) {
                int index = j + blockOffset;
                int distanceData = (unsigned char)data[blockStart + 4 + j * 3] + (unsigned char)data[blockStart + 5 + j * 3] * 0x100;
                int intensity = (unsigned char)data[blockStart + 6 + j * 3];

                // 激光点无效
                if (distanceData == 0) {
                    continue;
                }

                double distance = distanceData * innerCalib.unit + innerCalib.distCorrection[index];

                // 太远的激光点不考虑
                if (distance > 3000) {
                    continue;
                }

                // 计算激光点坐标
                double cosVertAngle = cos(innerCalib.vertCorrection[index]);
                double sinVertAngle = sin(innerCalib.vertCorrection[index]);

                double cosRotAngle = cos(theta);
                double sinRotAngle = sin(theta);

                double xyDistance = distance * cosVertAngle;

                X::Point3d point;
                point.x = xyDistance * sinRotAngle * 0.01;
                point.y = xyDistance * cosRotAngle * 0.01;
                point.z = distance * sinVertAngle * 0.01; // to m

                // 旋转到车体的坐标系方向上
                rotate(point, M_PI_2, 0, 0);

                // 只考虑车前方的激光点
                if (point.y < 0) {
                    continue;
                }

                // 如果激光点在车身上，那么不考虑
                if (point.y < 3 && point.y > -1.8 && abs(point.x) < 1.5) {
                    continue;
                }

                // 高于激光安装位置的点，不考虑
                if (point.z > 0) {
                    continue;
                }

                // 把激光点按照 GPS 坐标进行坐标变换
                X::Point3d p(point.x, point.y, point.z);
                X::rotatePoint3d(p, rot);
                p.x = p.x + deltaX;
                p.y = p.y + deltaY;
//                p.z = p.z + deltaZ;
                X::rotatePoint3d(p, rotInv);
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;

                // 计算激光点在栅格地图上的行列
                int r = -point.y / VELOPIXELSIZE + VELOGRID;
                int c = point.x / VELOPIXELSIZE + VELOGRID / 2;
                setGridMaskF(veloMap, r, c, point.z);
            }
        }
    }
    fclose(fvelodyne);

    // 查找和激光帧对应的图像用于可视化
    int cameraIndexForVelo = std::lower_bound(cameraTimeList.begin(), cameraTimeList.end(), time) - cameraTimeList.begin();
    if (cameraIndexForVelo == 0 || cameraIndexForVelo == cameraTimeList.size()) {
        return;
    }
    video.set(CV_CAP_PROP_POS_FRAMES, imageFrameList[cameraIndexForVelo]);
    video >> camImg;
    if (!initialized) {
        initialized = true;
    }
}

void MainWindow::fillGrid(cv::Mat mat, int m, int n, double pixelSize, cv::Scalar color) {
    if (m < 0 || n < 0 || m >= mat.rows * pixelSize || n >= mat.cols * pixelSize) {
        return;
    }
    cv::rectangle(mat, cv::Point(n / pixelSize, m / pixelSize),
                  cv::Point((n + 1) / pixelSize - 1, (m + 1) / pixelSize - 1), color, -1);
}

void MainWindow::fillCircle(cv::Mat mat, int m, int n, double pixelSize, cv::Scalar color) {
    if (m < 0 || n < 0 || m >= mat.rows * pixelSize || n >= mat.cols * pixelSize) {
        return;
    }

    cv::circle(mat, cv::Point((n + 0.5) / pixelSize, (m + 0.5) / pixelSize),
               0.75 / pixelSize, cv::Scalar(255, 255, 255), 2);
    cv::circle(mat, cv::Point((n + 0.5) / pixelSize, (m + 0.5) / pixelSize),
               0.75 / pixelSize, cv::Scalar(0, 0, 0), 1);

    cv::circle(mat, cv::Point((n + 0.5) / pixelSize, (m + 0.5) / pixelSize),
               0.53 / pixelSize, cv::Scalar(0, 0, 0), 2);

    cv::circle(mat, cv::Point((n + 0.5) / pixelSize, (m + 0.5) / pixelSize),
               0.5 / pixelSize, color, -1);

    cv::circle(mat, cv::Point((n + 0.5) / pixelSize, (m + 0.5) / pixelSize),
               0.2 / pixelSize, cv::Scalar(255, 255, 255), -1);
}

void MainWindow::setGridMask(cv::Mat mat, int m, int n, int value) {
    if (m < 0 || n < 0 || m >= mat.rows || n >= mat.cols) {
        return;
    }
    mat.at<uchar>(m, n) = value;
}

void MainWindow::setGridMaskF(cv::Mat mat, int m, int n, float value) {
    if (m < 0 || n < 0 || m >= mat.rows || n >= mat.cols) {
        return;
    }
    mat.at<float>(m, n) = value;
}

QVector<QPoint> MainWindow::calculateLine(QPoint p1, QPoint p2) {
    QVector<QPoint> result;
    if (p1.x() == p2.x()) {
        int minY = p1.y() > p2.y() ? p2.y() : p1.y();
        int maxY = p1.y() < p2.y() ? p2.y() : p1.y();
        for (int i = minY; i <= maxY; i++) {
            result.append(QPoint(p2.x(), i));
        }
    } else if (p1.y() == p2.y()) {
        int minX = p1.x() > p2.x() ? p2.x() : p1.x();
        int maxX = p1.x() < p2.x() ? p2.x() : p1.x();
        for (int i = minX; i <= maxX; i++) {
            result.append(QPoint(i, p2.y()));
        }
    } else {
        int minX = p1.x() > p2.x() ? p2.x() : p1.x();
        int maxX = p1.x() < p2.x() ? p2.x() : p1.x();
        int minY = p1.y() > p2.y() ? p2.y() : p1.y();
        int maxY = p1.y() < p2.y() ? p2.y() : p1.y();
        double k1 = double(p1.y() - p2.y()) / (p1.x() - p2.x());
        double b1 = p2.y() - k1 * p2.x();
        for (int i = minX; i <= maxX; i++) {
            int y = k1 * i + b1;
            if (y < minY || y > maxY) {
                continue;
            }
            result.append(QPoint(i, y));
        }
        double k2 = double(p1.x() - p2.x()) / (p1.y() - p2.y());
        double b2 = p2.x() - k2 * p2.y();
        for (int i = minY; i <= maxY; i++) {
            int x = k2 * i + b2;
            if (x < minX || x > maxX) {
                continue;
            }
            result.append(QPoint(x, i));
        }
    }
    return result;
}

QVector<QPoint> MainWindow::calculateInner(QVector<QPoint> polygon) {
    cv::Mat mat(GRID + 2, GRID + 2, CV_8U);
    mat.setTo(0);
    for (auto p: polygon) {
        setGridMask(mat, p.x() + 1, p.y() + 1, 1);
    }
    QList<QPoint> queue;
    queue.append(QPoint(0, 0));
    int dx[4] = { 0, 1, 0, -1 };
    int dy[4] = { 1, 0, -1, 0 };
    while (!queue.empty()) {
        QPoint current = queue.front();
        queue.pop_front();
        for (int k = 0; k < 4; k++) {
            int nx = current.x() + dx[k];
            int ny = current.y() + dy[k];
            if (nx < 0 || ny < 0 || nx >= mat.rows || ny >= mat.cols) {
                continue;
            }
            if (mat.at<uchar>(nx, ny) == 0) {
                queue.append(QPoint(nx, ny));
                mat.at<uchar>(nx, ny) = 2;
            }
        }
    }
    QVector<QPoint> result;
    for (int i = 1; i < mat.rows - 1; i++) {
        for (int j = 1; j < mat.cols - 1; j++) {
            if (mat.at<uchar>(i, j) == 0) {
                result.append(QPoint(i - 1, j - 1));
            }
        }
    }
    return result;
}

void MainWindow::updateImage() {
    if (!initialized) {
        return;
    }
    img.setTo(255);
    QVector<cv::Point3d> points;
    QVector<cv::Scalar> colors;

    // 可视化的最大最小范围
    double maxZ = ui->doubleSpinBox_max->text().toDouble();
    double minZ = ui->doubleSpinBox_min->text().toDouble();
    for (int i = 0; i < VELOGRID; i++) {
        for (int j = 0; j < VELOGRID; j++) {
            if (veloMap.at<float>(i, j) < -100) {
                continue;
            }
            int level = (veloMap.at<float>(i, j) - minZ) / (maxZ - minZ) * 255.0;
            level = std::max(0, level);
            level = std::min(255, level);
            int r = COLORMAP_R[level];
            int b = COLORMAP_B[level];
            int g = COLORMAP_G[level];
            points.append(cv::Point3d((j - veloMap.cols / 2) * VELOPIXELSIZE, (veloMap.rows - i) * VELOPIXELSIZE, veloMap.at<float>(i, j)));
            colors.append(cv::Scalar(b, g, r));
            fillGrid(img, i, j, PIXELSIZE, cv::Scalar(255 - level, 255 - level, 255 - level, 0));
        }
    }
    imgBackground = img.clone();
    int num = 0;
    QVector<cv::Point3d> points_tag;
    QVector<cv::Scalar> colors_tag;
    for (int i = 0; i < GRID; i++) {
        for (int j = 0; j < GRID; j++) {
            int tag = originMap.at<uchar>(i, j);
            if (tag) {
                num += 1;
                if (pointType == "Circle") {
                    fillCircle(img, i, j, VELOPIXELSIZE, colorTable[tag]);
                } else {
                    fillGrid(img, i, j, VELOPIXELSIZE, colorTable[tag]);
                }
                for (int ti = 0; ti < RATIO; ti++) {
                    for (int tj = 0; tj < RATIO; tj++) {
                        points_tag.append(cv::Point3d((j * RATIO + tj - veloMap.cols / 2) * VELOPIXELSIZE, (veloMap.rows - i * RATIO - ti) * VELOPIXELSIZE,
                                                      veloMap.at<float>(i * RATIO + ti, j * RATIO + tj)));
                        colors_tag.append(colorTable[tag]);
                    }
                }
            }
            int mask = maskMap.at<uchar>(i, j);
            if (mask) {
                fillGrid(img, i, j, VELOPIXELSIZE, colorTable[mask]);
            }
        }
    }
    QImage qveloimg((const uchar*)img.data, img.cols, img.rows,
                    img.step, QImage::Format_RGB888);
    ui->labelMapImage->setPixmap(QPixmap::fromImage(qveloimg.rgbSwapped()));

    if (!camImg.empty()) {
        cam = camImg.clone();
        cam_raw = camImg.clone();
        if (ui->checkBox_showLidar->isChecked()) {
            drawCalibVelodyneOnImage(points, colors, cam);
            drawCalibVelodyneOnImage(points_tag, colors_tag, cam);
        }
        cv::Rect rect(cam.cols / 2 - 100, 0, cam.cols / 2, cam.rows);
        cam = cam(rect);
        cam_raw = cam_raw(rect);
        cv::resize(cam, cam, cv::Size(), (double)img.cols / cam.cols, (double)img.cols / cam.cols);
        QImage qcameraimg((const uchar*)cam.data, cam.cols, cam.rows,
                          cam.step, QImage::Format_RGB888);
        ui->labelImage->setPixmap(QPixmap::fromImage(qcameraimg.rgbSwapped()));
    }

    ui->statusBar->showMessage(QString("%1 points are labelled").arg(num));
}

void MainWindow::rotate(X::Point3d &p, double heading, double pitch, double roll) {
    X::Point3d p1;
    p1.x = p.x * cos(heading) + p.y * sin(heading);
    p1.y = -p.x * sin(heading) + p.y * cos(heading);
    p1.z = p.z;
    p = p1;

    X::Point3d p2;
    p2.x = p.x;
    p2.y = p.y * cos(pitch) - p.z * sin(pitch);
    p2.z = p.y * sin(pitch) + p.z * cos(pitch);
    p = p2;

    X::Point3d p3;
    p3.x = p.x;
    p3.y = p.y;
    p3.z = p.z;
    p = p3;
}

void MainWindow::drawCalibVelodyneOnImage(QVector<cv::Point3d> laser, QVector<cv::Scalar> color, cv::Mat &image) {
    double longitude, latitude, x, y;

    if (!image.data) {
        return;
    }

    X::Point3d velodyne_camera_shift;
    X::Matrix velodyne_camera_rot;
    double shv_x = 0, shv_y = 0, shv_z = 0.2;
    // 和相机旋转的标定参数，可以在界面中微调
    double rot_x = 0, rot_y = 0, rot_z = ui->doubleSpinBox_rotz->text().toDouble();
    velodyne_camera_shift.x = shv_x;
    velodyne_camera_shift.y = shv_y;
    velodyne_camera_shift.z = shv_z;

    X::createRotMatrix_XYZ(velodyne_camera_rot, rot_x, rot_y, rot_z);

    int col = image.cols;
    int row = image.rows;

    X::Point3d pL;
    for (int i = 0; i < laser.size(); i++) {

        // 讲道理吧，后面 normalize 后这里的 scale 就没用了
        // 但实际上运行会有影响，很奇怪
        double scale = ui->lineEdit_param->text().toDouble();
        pL.x = laser[i].x * scale;
        pL.y = laser[i].y * scale;
        pL.z = laser[i].z * scale;

        if (pL.z < -100) {
            pL.z = -2.12;
        }

        X::rotatePoint3d(pL, velodyne_camera_rot);
        X::shiftPoint3d(pL, velodyne_camera_shift);
        X::normalPoint3d(pL);

        longitude = atan2(pL.y, pL.x);
        latitude = asin(pL.z);

        longitude = (1 - longitude / M_PI) / 2;
        latitude = 0.5 - latitude / M_PI;

        x = int(longitude * col);
        y = int(latitude * row);
        if (x < 0 || x >= col || y < 0 || y >= row) {
            continue;
        }
        cv::circle(image, cv::Point(x, y), 2, color[i], -1);
    }
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
    // 当鼠标右键和 Ctrl 键按下时，清除当前格子的属性
    if (event->button() == Qt::RightButton && QApplication::keyboardModifiers() == Qt::ControlModifier) {
        auto point = ui->labelMapImage->mapFrom(this, event->pos());
        if (point.x() < 0 || point.y() < 0 || point.x() > GRID / VELOPIXELSIZE || point.y() > GRID / VELOPIXELSIZE) {
            return;
        }
        int m = point.y() * VELOPIXELSIZE;
        int n = point.x() * VELOPIXELSIZE;
        setGridMask(originMap, m, n, 0);
    }
    if (graphType == "Point") {
        if (event->button() == Qt::LeftButton && QApplication::keyboardModifiers() == Qt::ControlModifier) {
            auto point = ui->labelMapImage->mapFrom(this, event->pos());
            int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
            int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
            point.setX(point.x() - margin_left);
            point.setY(point.y() - margin_top);
            if (point.x() < 0 || point.y() < 0 || point.x() > GRID / VELOPIXELSIZE || point.y() > GRID / VELOPIXELSIZE) {
                return;
            }
            int m = point.y() * VELOPIXELSIZE;
            int n = point.x() * VELOPIXELSIZE;
            setGridMask(originMap, m, n, currentType);
        }
    }
    if (graphType == "Line") {
        if (event->button() == Qt::LeftButton && QApplication::keyboardModifiers() == Qt::ControlModifier) {
            auto point = ui->labelMapImage->mapFrom(this, event->pos());
            int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
            int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
            point.setX(point.x() - margin_left);
            point.setY(point.y() - margin_top);
            if (point.x() < 0 || point.y() < 0 || point.x() > GRID / VELOPIXELSIZE || point.y() > GRID / VELOPIXELSIZE) {
                return;
            }
            int m = point.y() * VELOPIXELSIZE;
            int n = point.x() * VELOPIXELSIZE;
            if (pointStack.size() == 1) {
                QVector<QPoint> points = calculateLine(pointStack[0], QPoint(m, n));
                for (auto p: points) {
                    setGridMask(originMap, p.x(), p.y(), currentType);
                }
                setGridMask(originMap, pointStack[0].x(), pointStack[0].y(), currentType);
                setGridMask(originMap, m, n, currentType);
                pointStack.clear();
            } else {
                pointStack.append(QPoint(m, n));
            }
        }
    }
    if (graphType == "Polygon") {
        if (event->button() == Qt::LeftButton && QApplication::keyboardModifiers() == Qt::ControlModifier) {
            auto point = ui->labelMapImage->mapFrom(this, event->pos());
            int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
            int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
            point.setX(point.x() - margin_left);
            point.setY(point.y() - margin_top);
            if (point.x() < 0 || point.y() < 0 || point.x() > GRID / VELOPIXELSIZE || point.y() > GRID / VELOPIXELSIZE) {
                return;
            }
            int m = point.y() * VELOPIXELSIZE;
            int n = point.x() * VELOPIXELSIZE;
            if (pointStack.size() > 0 && pointStack[0] == QPoint(m, n)) {
                QVector<QPoint> points;
                for (int i = 1; i < pointStack.size(); i++) {
                    points.append(calculateLine(pointStack[i - 1], pointStack[i]));
                }
                points.append(calculateLine(pointStack[0], QPoint(m, n)));
                points.append(calculateLine(pointStack[pointStack.size() - 1], QPoint(m, n)));
                points.append(calculateInner(points));
                for (auto p: points) {
                    setGridMask(originMap, p.x(), p.y(), currentType);
                }
                for (int i = 0; i < pointStack.size(); i++) {
                    setGridMask(originMap, pointStack[i].x(), pointStack[i].y(), currentType);
                }
                setGridMask(originMap, m, n, currentType);
                pointStack.clear();
            } else {
                pointStack.append(QPoint(m, n));
            }
        }
    }
}

void MainWindow::mouseMoveEvent(QMouseEvent *event) {
    // 鼠标移动的过程中，对图像进行着色
    if (graphType == "Point") {
        auto point = ui->labelMapImage->mapFrom(this, event->pos());
        int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
        int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
        point.setX(point.x() - margin_left);
        point.setY(point.y() - margin_top);
        if (point.x() < 0 || point.y() < 0 || point.x() > GRID / VELOPIXELSIZE || point.y() > GRID / VELOPIXELSIZE) {
            return;
        }
        int m = point.y() * VELOPIXELSIZE;
        int n = point.x() * VELOPIXELSIZE;
        maskMap.setTo(0);
//        setGridMask(maskMap, m, n, 102);
    }
    if (graphType == "Line") {
        auto point = ui->labelMapImage->mapFrom(this, event->pos());
        int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
        int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
        point.setX(point.x() - margin_left);
        point.setY(point.y() - margin_top);
        if (point.x() < 0 || point.y() < 0 || point.x() > GRID / VELOPIXELSIZE || point.y() > GRID / VELOPIXELSIZE) {
            return;
        }
        int m = point.y() * VELOPIXELSIZE;
        int n = point.x() * VELOPIXELSIZE;
        maskMap.setTo(0);
        if (pointStack.size() == 1) {
            QVector<QPoint> points = calculateLine(pointStack[0], QPoint(m, n));
            for (auto p: points) {
                setGridMask(maskMap, p.x(), p.y(), 101);
            }
            setGridMask(maskMap, pointStack[0].x(), pointStack[0].y(), 102);
        }
//        setGridMask(maskMap, m, n, 102);
    }
    if (graphType == "Polygon") {
        auto point = ui->labelMapImage->mapFrom(this, event->pos());
        int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
        int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
        point.setX(point.x() - margin_left);
        point.setY(point.y() - margin_top);
        if (point.x() < 0 || point.y() < 0 || point.x() > GRID / VELOPIXELSIZE || point.y() > GRID / VELOPIXELSIZE) {
            return;
        }
        int m = point.y() * VELOPIXELSIZE;
        int n = point.x() * VELOPIXELSIZE;
        maskMap.setTo(0);
        if (pointStack.size() > 0) {
            QVector<QPoint> points;
            for (int i = 1; i < pointStack.size(); i++) {
                points.append(calculateLine(pointStack[i - 1], pointStack[i]));
            }
            points.append(calculateLine(pointStack[0], QPoint(m, n)));
            points.append(calculateLine(pointStack[pointStack.size() - 1], QPoint(m, n)));
            for (auto p: points) {
                setGridMask(maskMap, p.x(), p.y(), 101);
            }
            setGridMask(maskMap, pointStack[0].x(), pointStack[0].y(), 103);
            for (int i = 1; i < pointStack.size(); i++) {
                setGridMask(maskMap, pointStack[i].x(), pointStack[i].y(), 102);
            }
        }
//        setGridMask(maskMap, m, n, 102);
        if (pointStack.size() > 0 && pointStack[0] == QPoint(m, n)) {
            setGridMask(maskMap, m, n, 103);
        }
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    // 按下 Esc 键的时候，清除绘制了一般的标注区域
    if (event->key() == Qt::Key_Escape) {
        pointStack.clear();
    }
}
