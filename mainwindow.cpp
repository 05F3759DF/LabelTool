#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow) {
    static_assert(SCALE % VELOSCALE == 0, "");
    ui->setupUi(this);

    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::openFile);
    connect(ui->actionExport, &QAction::triggered, this, &MainWindow::exportFile);
    connect(ui->actionScan, &QAction::triggered, this, &MainWindow::scan);
    connect(ui->pushButton_set, &QPushButton::clicked, this, &MainWindow::setTime);
    connect(ui->pushButton_execute, &QPushButton::clicked, this, &MainWindow::execute);

    img.create(GRID * SCALE + 1, GRID * SCALE + 1, CV_8UC4);
    veloImg.create(GRID * SCALE + 1, GRID * SCALE + 1, CV_8UC3);
    originMap = cv::Mat(GRID, GRID, CV_8U);
    originMap.setTo(0);
    maskMap = cv::Mat(GRID, GRID, CV_8U);
    maskMap.setTo(0);
    veloMap = cv::Mat(VELOGRID, VELOGRID, CV_32F);
    veloMap.setTo(10000);

    colorTable[1] = cv::Scalar(0, 0, 0, 100);
    colorTable[2] = cv::Scalar(255, 0, 0, 100);
    colorTable[3] = cv::Scalar(0, 255, 0, 100);
    colorTable[11] = cv::Scalar(255, 255, 0, 25);
    colorTable[12] = cv::Scalar(0, 0, 255, 25);
    colorTable[13] = cv::Scalar(255, 0, 255, 25);
    colorTable[14] = cv::Scalar(255, 0, 128, 25);

    currentType = 1;
    ui->radioButton_type1->setChecked(true);
    connect(ui->radioButton_blank, &QRadioButton::clicked, [=](bool checked) {
        if (checked) {
            currentType = 0;
        }
    });
    connect(ui->radioButton_type1, &QRadioButton::clicked, [=](bool checked) {
        if (checked) {
            currentType = 1;
        }
    });
    connect(ui->radioButton_type2, &QRadioButton::clicked, [=](bool checked) {
        if (checked) {
            currentType = 2;
        }
    });
    connect(ui->radioButton_type3, &QRadioButton::clicked, [=](bool checked) {
        if (checked) {
            currentType = 3;
        }
    });

    ui->radioButton_point->setChecked(true);
    graphType = "Point";
    connect(ui->radioButton_point, &QRadioButton::clicked, [=](bool checked) {
        if (checked) {
            graphType = "Point";
            pointStack.clear();
        }
    });
    connect(ui->radioButton_line, &QRadioButton::clicked, [=](bool checked) {
        if (checked) {
            graphType = "Line";
            pointStack.clear();
        }
    });
    connect(ui->radioButton_polygon, &QRadioButton::clicked, [=](bool checked) {
        if (checked) {
            graphType = "Polygon";
            pointStack.clear();
        }
    });
    connect(&renderTimer, &QTimer::timeout, this, &MainWindow::updateImage);
    renderTimer.start(100);

    ui->labelMapImage->setMouseTracking(true);
    ui->centralWidget->setMouseTracking(true);
    setMouseTracking(true);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::openFile() {
    QString filename = QFileDialog::getOpenFileName(this, "Config", ".");
    setting = new QSettings(filename, QSettings::IniFormat, this);


    velodyneFilename = setting->value("/Sensor/velodyne", "").toString();
    gpsFilename = setting->value("/Sensor/gps", "").toString();
    calibFilename = setting->value("/Sensor/velocalib", "db32.xml").toString();
    qDebug() << velodyneFilename << gpsFilename;

    innerCalib.loadCalib(calibFilename.toStdString());
}

void MainWindow::exportFile() {
    QString filename = ui->lineEdit_gpstime->text() + ".txt";
    QVector<QPair<QPoint, int>> pointList;
    for (int i = 0; i < originMap.rows; i++) {
        for (int j = 0; j < originMap.cols; j++) {
            if (originMap.at<uchar>(i, j)) {
                pointList.append(qMakePair(QPoint(i, j), originMap.at<uchar>(i, j)));
            }
        }
    }
    FILE *f = fopen(filename.toStdString().c_str(), "w");
    fprintf(f, "%d\n", pointList.size());
    for (auto p: pointList) {
        fprintf(f, "%d %d %d\n", p.first.x(), p.first.y(), p.second);
    }
    fclose(f);
    QMessageBox::information(this, "Info", QString("Labels have been saved to %1.").arg(filename));
}

void MainWindow::scan()
{
    FILE *fgps = fopen(gpsFilename.toStdString().c_str(), "r");
    fclose(fgps);
    FILE *fvelodyne = fopen(velodyneFilename.toStdString().c_str(), "rb");
    char data[2048];
    fread(data, 1, 24, fvelodyne);
    int sec, usec, caplen, len;
    int64 count = 24;
    while (!feof(fvelodyne)) {
        fread((char*)&sec, sizeof(int), 1, fvelodyne);
        fread((char*)&usec, sizeof(int), 1, fvelodyne);
        fread((char*)&caplen, sizeof(int), 1, fvelodyne);
        fread((char*)&len, sizeof(int), 1, fvelodyne);
        count += 16;
        if (len != 1248) {
            fread(data, 1, len, fvelodyne);
            count += len;
            continue;
        }
        QDateTime dateTime = QDateTime::fromMSecsSinceEpoch((long long)sec * 1000);
        int time = (long long)dateTime.time().msecsSinceStartOfDay() + usec / 1000;
        veloTimeList.append(time);
        veloPosList.append(count);
        fread(data, 1, 1248, fvelodyne);
        count += 1248;
    }
    fclose(fvelodyne);
}

void MainWindow::setTime()
{
    if (ui->lineEdit_gpstime->text().isEmpty()) return;
    int time = ui->lineEdit_gpstime->text().toInt();
    auto pos = std::lower_bound(veloTimeList.begin(), veloTimeList.end(), time);
    if (pos == veloTimeList.end()) return;
    int index = pos - veloTimeList.begin();
    int startIndex = index - 180;
    int endIndex = index + 180;
    if (startIndex < 0) startIndex = 0;
    if (endIndex >= veloTimeList.size()) endIndex = veloTimeList.size() - 1;
    ui->lineEdit_starttime->setText(QString::number(startIndex));
    ui->lineEdit_endtime->setText(QString::number(endIndex));
}

void MainWindow::execute()
{
    originMap.setTo(0);
    maskMap.setTo(0);
    veloImg.setTo(0);
    veloMap.setTo(10000);
    FILE *fvelodyne = fopen(velodyneFilename.toStdString().c_str(), "rb");
    char data[2048];
    int startIndex = ui->lineEdit_starttime->text().toInt();
    int endIndex = ui->lineEdit_endtime->text().toInt();
    for (int index = startIndex; index <= endIndex; index++) {
        fseeko64(fvelodyne, veloPosList[index] + 42, SEEK_SET);
        fread(data, 1, 1200, fvelodyne);

        for (int i = 0; i < 12; i++) {
            int blockStart = i * 100;
            unsigned int blockID = (unsigned char)data[blockStart] + (unsigned char)data[blockStart + 1] * 0x100;
            int blockOffset = blockID == 0xEEFF ? 0 : 32;
            unsigned int rotationData = (unsigned char)data[blockStart + 2] + (unsigned char)data[blockStart + 3] * 0x100;
            double theta = rotationData / 18000.0 * 3.1415926535;
            for (int j = 0; j < 32; j++) {
                int index = j + blockOffset;
                int distanceData = (unsigned char)data[blockStart + 4 + j * 3] + (unsigned char)data[blockStart + 5 + j * 3] * 0x100;
                int intensity = (unsigned char)data[blockStart + 6 + j * 3];
                if (distanceData == 0) {
                    continue;
                }

                double distance = distanceData * innerCalib.unit + innerCalib.distCorrection[index];

                double cosVertAngle = cos(innerCalib.vertCorrection[index]);
                double sinVertAngle = sin(innerCalib.vertCorrection[index]);

                double cosRotAngle = cos(theta);
                double sinRotAngle = sin(theta);

                double xyDistance = distance * cosVertAngle;

                cv::Point3d point;
                point.x = xyDistance * sinRotAngle * 0.01;
                point.y = xyDistance * cosRotAngle * 0.01;
                point.z = distance * sinVertAngle * 0.01; // to m
                rotate(point, M_PI_2);

                int r = point.x * 10 + VELOGRID / 2;
                int c = point.y * 10 + VELOGRID;
                setGridMaskF(veloMap, r, c, point.z);
            }
        }
    }
    fclose(fvelodyne);
}

void MainWindow::drawGrid(cv::Mat mat, int m, int n, int scale, cv::Scalar color)
{
    if (m < 0 || n < 0 || m >= mat.rows / scale || n >= mat.cols / scale) return;
    cv::rectangle(mat, cv::Point(m * scale, n * scale), cv::Point((m + 1) * scale, (n + 1) * scale), color, 1);
}

void MainWindow::fillGrid(cv::Mat mat, int m, int n, int scale, cv::Scalar color, bool withoutBorder)
{
    if (m < 0 || n < 0 || m >= mat.rows / scale || n >= mat.cols / scale) return;
    cv::rectangle(mat, cv::Point(m * scale + withoutBorder, n * scale + withoutBorder),
                  cv::Point((m + 1) * scale - withoutBorder, (n + 1) * scale - withoutBorder), color, -1);
}

void MainWindow::setGridMask(cv::Mat mat, int m, int n, int type)
{
    if (m < 0 || n < 0 || m >= mat.rows || n >= mat.cols) return;
    mat.at<uchar>(m, n) = type;
}

void MainWindow::setGridMaskF(cv::Mat mat, int m, int n, int type)
{
    if (m < 0 || n < 0 || m >= mat.rows || n >= mat.cols) return;
    mat.at<float>(m, n) = type;
}

QVector<QPoint> MainWindow::calculateLine(QPoint p1, QPoint p2)
{
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
            if (y < minY || y > maxY) continue;
            result.append(QPoint(i, y));
        }
        double k2 = double(p1.x() - p2.x()) / (p1.y() - p2.y());
        double b2 = p2.x() - k2 * p2.y();
        for (int i = minY; i <= maxY; i++) {
            int x = k2 * i + b2;
            if (x < minX || x > maxX) continue;
            result.append(QPoint(x, i));
        }
    }
    return result;
}

QVector<QPoint> MainWindow::calculateInner(QVector<QPoint> polygon)
{
    cv::Mat mat(GRID + 2, GRID + 2, CV_8U);
    mat.setTo(0);
    for (auto p: polygon) {
        setGridMask(mat, p.x() + 1, p.y() + 1, 1);
    }
    QList<QPoint> queue;
    queue.append(QPoint(0, 0));
    int dx[4] = {0, 1, 0, -1};
    int dy[4] = {1, 0, -1, 0};
    while (!queue.empty()) {
        QPoint current = queue.front();
        queue.pop_front();
        for (int k = 0; k < 4; k++) {
            int nx = current.x() + dx[k];
            int ny = current.y() + dy[k];
            if (nx < 0 || ny < 0 || nx >= mat.rows || ny >= mat.cols) continue;
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

void MainWindow::updateImage()
{
    img.setTo(255);
    veloImg.setTo(255);
    for (int i = 0; i < VELOGRID; i++) {
        for (int j = 0; j < VELOGRID; j++) {
            drawGrid(veloImg, i, j, VELOSCALE, cv::Scalar(200, 200, 200));
            if (veloMap.at<float>(i, j) > 100) continue;
            double maxZ = 2.0, minZ = -3.0;
            int r = (veloMap.at<float>(i, j) - minZ) / (maxZ - minZ) * 255.0;
            r = r > 255 ? 255 : r;
            r = r < 0 ? 0 : r;
            int b = 255 - r;
            int g = 2 * (r > b ? b : r);
            fillGrid(img, i, j, VELOSCALE, cv::Scalar(b, g, r, 255), false);
        }
    }
    for (int i = 0; i < GRID; i++) {
        for (int j = 0; j < GRID; j++) {
//            drawGrid(img, i, j, SCALE);
            int tag = originMap.at<uchar>(i, j);
            if (tag) {
                fillGrid(img, i, j, SCALE, colorTable[tag]);
            }
            int mask = maskMap.at<uchar>(i, j);
            if (mask) {
                fillGrid(img, i, j, SCALE, colorTable[mask]);
                drawGrid(veloImg, i, j, SCALE);
                int startI = i * RATIO;
                int startJ = j * RATIO;
                for (int ni = startI; ni < startI + RATIO; ni++) {
                    for (int nj = startJ; nj < startJ + RATIO; nj++) {
                        fillGrid(veloImg, ni, nj, VELOSCALE, colorTable[mask]);
                    }
                }
            }
        }
    }

    QImage qimg((const uchar*)img.data, img.cols, img.rows,
                img.step, QImage::Format_RGBA8888);
    ui->labelMapImage->setPixmap(QPixmap::fromImage(qimg.rgbSwapped()));

    QImage qveloimg((const uchar*)veloImg.data, veloImg.cols, veloImg.rows,
                    veloImg.step, QImage::Format_RGB888);
    ui->labelLidar->setPixmap(QPixmap::fromImage(qveloimg.rgbSwapped()));
}

void MainWindow::rotate(cv::Point3d &point, double ang)
{
    cv::Point3d p;
    p.x = point.x * cos(ang) - point.y * sin(ang);
    p.y = point.x * sin(ang) + point.y * cos(ang);
    p.z = point.z;
    point = p;
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton && QApplication::keyboardModifiers () == Qt::ControlModifier) {
        auto point = ui->labelMapImage->mapFrom(this, event->pos());
        if (point.x() < 0 || point.y() < 0 || point.x() > GRID * SCALE || point.y() > GRID * SCALE) return;
        int m = point.x() / SCALE;
        int n = point.y() / SCALE;
        setGridMask(originMap, m, n, 0);
    }
    if (graphType == "Point") {
        if (event->button() == Qt::LeftButton && QApplication::keyboardModifiers () == Qt::ControlModifier) {
            auto point = ui->labelMapImage->mapFrom(this, event->pos());
            int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
            int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
            point.setX(point.x() - margin_left);
            point.setY(point.y() - margin_top);
            if (point.x() < 0 || point.y() < 0 || point.x() > GRID * SCALE || point.y() > GRID * SCALE) return;
            int m = point.x() / SCALE;
            int n = point.y() / SCALE;
            setGridMask(originMap, m, n, currentType);
        }
    }
    if (graphType == "Line") {
        if (event->button() == Qt::LeftButton && QApplication::keyboardModifiers () == Qt::ControlModifier) {
            auto point = ui->labelMapImage->mapFrom(this, event->pos());
            int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
            int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
            point.setX(point.x() - margin_left);
            point.setY(point.y() - margin_top);
            if (point.x() < 0 || point.y() < 0 || point.x() > GRID * SCALE || point.y() > GRID * SCALE) return;
            int m = point.x() / SCALE;
            int n = point.y() / SCALE;
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
        if (event->button() == Qt::LeftButton && QApplication::keyboardModifiers () == Qt::ControlModifier) {
            auto point = ui->labelMapImage->mapFrom(this, event->pos());
            int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
            int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
            point.setX(point.x() - margin_left);
            point.setY(point.y() - margin_top);
            if (point.x() < 0 || point.y() < 0 || point.x() > GRID * SCALE || point.y() > GRID * SCALE) return;
            int m = point.x() / SCALE;
            int n = point.y() / SCALE;
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

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (graphType == "Point") {
        auto point = ui->labelMapImage->mapFrom(this, event->pos());
        int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
        int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
        point.setX(point.x() - margin_left);
        point.setY(point.y() - margin_top);
        if (point.x() < 0 || point.y() < 0 || point.x() > GRID * SCALE || point.y() > GRID * SCALE) return;
        int m = point.x() / SCALE;
        int n = point.y() / SCALE;
        maskMap.setTo(0);
        setGridMask(maskMap, m, n, 12);
    }
    if (graphType == "Line") {
        auto point = ui->labelMapImage->mapFrom(this, event->pos());
        int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
        int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
        point.setX(point.x() - margin_left);
        point.setY(point.y() - margin_top);
        if (point.x() < 0 || point.y() < 0 || point.x() > GRID * SCALE || point.y() > GRID * SCALE) return;
        int m = point.x() / SCALE;
        int n = point.y() / SCALE;
        maskMap.setTo(0);
        if (pointStack.size() == 1) {
            QVector<QPoint> points = calculateLine(pointStack[0], QPoint(m, n));
            for (auto p: points) {
                setGridMask(maskMap, p.x(), p.y(), 11);
            }
            setGridMask(maskMap, pointStack[0].x(), pointStack[0].y(), 12);
        }
        setGridMask(maskMap, m, n, 12);
    }
    if (graphType == "Polygon") {
        auto point = ui->labelMapImage->mapFrom(this, event->pos());
        int margin_left = ui->labelMapImage->width() <= img.rows ? 0 : (ui->labelMapImage->width() - img.rows) / 2;
        int margin_top = ui->labelMapImage->height() <= img.cols ? 0 : (ui->labelMapImage->height() - img.cols) / 2;
        point.setX(point.x() - margin_left);
        point.setY(point.y() - margin_top);
        if (point.x() < 0 || point.y() < 0 || point.x() > GRID * SCALE || point.y() > GRID * SCALE) return;
        int m = point.x() / SCALE;
        int n = point.y() / SCALE;
        maskMap.setTo(0);
        if (pointStack.size() > 0) {
            QVector<QPoint> points;
            for (int i = 1; i < pointStack.size(); i++) {
                points.append(calculateLine(pointStack[i - 1], pointStack[i]));
            }
            points.append(calculateLine(pointStack[0], QPoint(m, n)));
            points.append(calculateLine(pointStack[pointStack.size() - 1], QPoint(m, n)));
            for (auto p: points) {
                setGridMask(maskMap, p.x(), p.y(), 11);
            }
            setGridMask(maskMap, pointStack[0].x(), pointStack[0].y(), 13);
            for (int i = 1; i < pointStack.size(); i++) {
                setGridMask(maskMap, pointStack[i].x(), pointStack[i].y(), 12);
            }
        }
        setGridMask(maskMap, m, n, 12);
        if (pointStack.size() > 0 && pointStack[0] == QPoint(m, n)) {
            setGridMask(maskMap, m, n, 13);
        }
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Escape) {
        pointStack.clear();
    }
}

