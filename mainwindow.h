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
#include "coordinateconvertion.h"
#include "colormap.h"
#include "coordinatetransform.h"

/**
 * @brief 记录位置信息
 */
class Position {
public:
    double x;
    double y;
    double z;
    double heading;
    double pitch;
    double roll;
    Position()
        : x(0), y(0), z(0), heading(0), pitch(0), roll(0) {}
    Position(double x, double y, double z, double heading, double pitch, double roll)
        : x(x), y(y), z(z), heading(heading), pitch(pitch), roll(roll) {}
};

#define PIXELSIZE 0.5
#define GRID 81
#define VELOPIXELSIZE 0.1
#define VELOGRID (PIXELSIZE * GRID / VELOPIXELSIZE)
#define RATIO (PIXELSIZE / VELOPIXELSIZE)

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    /**
     * @brief 根据配置文件打开相关文件
     */
    void openFile();

    /**
     * @brief 导出标注的栅格地图
     */
    void exportFile();

    /**
     * @brief 只导出栅格地图背景
     */
    void exportBackground();

    /**
     * @brief 导入相关文件
     *
     * 需要对应的文件在openFile中已经被正确打开
     */
    void loadFile();

    /**
     * @brief 扫描输入文件，把必要的信息存储在内存中
     */
    void scan();

    /**
     * @brief 设置需要可视化的时间范围
     */
    void setTime();

    /**
     * @brief 执行可视化
     */
    void execute();

    cv::Mat img;
    cv::Mat imgBackground;
    cv::Mat originMap;
    cv::Mat maskMap;

    cv::Mat veloImg;
    cv::Mat veloMap;
    cv::Mat camImg, cam, cam_raw;

    bool initialized;

    void fillGrid(cv::Mat mat, int m, int n, double pixelSize, cv::Scalar color = cv::Scalar(0, 0, 0), bool withoutBorder = true);
    void fillCircle(cv::Mat mat, int m, int n, double pixelSize, cv::Scalar color = cv::Scalar(0, 0, 0), bool withoutBorder = true);
    void setGridMask(cv::Mat mat, int m, int n, int value);
    void setGridMaskF(cv::Mat mat, int m, int n, float value);
    void setGridMaskF_gt(cv::Mat mat, int m, int n, int value);
    QVector<QPoint> calculateLine(QPoint p1, QPoint p2);
    QVector<QPoint> calculateInner(QVector<QPoint> polygon);
    QSettings *setting;

    CoordinateConvertion coord;
    void rotate(X::Point3d &p, double heading, double pitch = 0, double roll = 0);

    QMap<int, cv::Scalar> colorTable;
    int currentType;
    QString graphType;
    QVector<QPoint> pointStack;

    int startTime, endTime;
    QString velodyneFilename, gpsFilename, imuFilename;
    QString ladybugFilename, ladybugTimeFilename;

    cv::VideoCapture video;

    QTimer renderTimer;
    QVector<int> gpsTimeList;
    QVector<Position> posList;
    QVector<int> veloTimeList;
    QVector<int64> veloPosList;
    QVector<int> cameraTimeList;
    QVector<int> imageFrameList;
    VeloCalib innerCalib;
    QString calibFilename;

    void updateImage();

    void drawCalibVelodyneOnImage(QVector<cv::Point3d> laser, QVector<cv::Scalar> color, cv::Mat& image);

    void showSplitCloud();
protected:
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);

};

#endif // MAINWINDOW_H
