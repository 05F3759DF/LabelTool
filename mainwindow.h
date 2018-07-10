#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <algorithm>

#include <QMainWindow>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QDateTime>
#include <QSettings>
#include <QMouseEvent>
#include <QVector>
#include <QMap>
#include <QTimer>
#include <QDebug>

#include <opencv2/opencv.hpp>

#include "velocalib.h"


class Position {
public:
    double x;
    double y;
    double heading;
};

#define SCALE 10
#define GRID 60
#define VELOSCALE 2
#define VELOGRID (SCALE * GRID / VELOSCALE)
#define RATIO (SCALE / VELOSCALE)

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    void openFile();
    void exportFile();
    void scan();
    void setTime();
    void execute();
    cv::Mat img;
    cv::Mat originMap;
    cv::Mat maskMap;

    cv::Mat veloImg;
    cv::Mat veloMap;

    void drawGrid(cv::Mat mat, int m, int n, int scale, cv::Scalar color=cv::Scalar(100, 100, 100));
    void fillGrid(cv::Mat mat, int m, int n, int scale, cv::Scalar color=cv::Scalar(0, 0, 0), bool withoutBorder = true);
    void setGridMask(cv::Mat mat, int m, int n, int type);
    void setGridMaskF(cv::Mat mat, int m, int n, int type);
    QVector<QPoint> calculateLine(QPoint p1, QPoint p2);
    QVector<QPoint> calculateInner(QVector<QPoint> polygon);
    void updateImage();
    QSettings *setting;

    void rotate(cv::Point3d &point, double ang);

    QMap<int, cv::Scalar> colorTable;
    int currentType;
    QString graphType;
    QVector<QPoint> pointStack;

    int startTime, endTime;
    QString velodyneFilename, gpsFilename;

    QTimer renderTimer;
    QVector<int> gpsTimeList;
    QVector<Position> posList;
    QVector<int> veloTimeList;
    QVector<int64> veloPosList;

    VeloCalib innerCalib;
    QString calibFilename;
protected:
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
};

#endif // MAINWINDOW_H
