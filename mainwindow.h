#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort>
#include <QDebug>
#include <QVector>
#include <QSerialPort>
#include <iostream>
#include <iomanip>
#include <limits>
#include <QTimer>
#include <stdio.h>
#include <QMessageBox>
#include "tinyxml2.h"
#include "spline.h"
#include "GeographicLib/UTMUPS.hpp"
#include <sstream>
#include "GeographicLib/Constants.hpp"
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void FloatToByte(double dnumber, uint8_t* bOut);
    void FloatToByteArrayWithNipes(double dnumber, uint8_t* bOut);
    void DoubleToByte(double dNorth, double dEast, uint8_t* bEast, uint8_t* bNorth);
    void ConvertDoubleToByte(double dYaw,uint8_t* arr);
private slots:

    void on_btn_open_clicked();

    void on_btn_sendpidm1_clicked();

    void on_btn_sendvrefm1_clicked();

    void on_btn_sendpidm2_clicked();

    void on_btn_sendvrefm2_clicked();

    void on_btn_cleardata_clicked();

    void on_btn_stop_clicked();

    void readData();
    void on_btn_readMap_clicked();

    void on_btn_drawMap_clicked();


    void on_btn_SendMap_clicked();

    void TimeOut_Event();

    void on_btn_ClearMap_clicked();

    void on_btn_SetStanley_clicked();

    void on_btn_SetFuzzy_clicked();

    void on_btn_SendData_clicked();

private:
    Ui::MainWindow *ui;
    long m_baudrate;
    QByteArray data;
    QSerialPort *m_serial;
    bool bPortOpen = false;
    QVector<double> dVmotor1, dVmotor2,Time;
    double dTime = 0.0;
    vector<double>x ;
    vector<double>y ;
    QVector<double> dEast, dNorth;
    vector<double>dYaw;
    vector<double>xval;
    vector<double>yval;
    unsigned int unPoint = 0;
    uint32_t uPointNumber = 0;
    uint32_t uCount = 0;
    uint32_t uCurrentPoint = 0;
    uint32_t uCountYaw = 0;
    QTimer* Timer;
};
#endif // MAINWINDOW_H
