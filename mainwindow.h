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

private:
    Ui::MainWindow *ui;
    long m_baudrate;
    QByteArray data;
    QSerialPort *m_serial;
    bool bPortOpen = false;
    QVector<double> dVmotor1, dVmotor2,Time;
    double dTime = 0.0;
    QVector<double> dEast, dNorth;
};
#endif // MAINWINDOW_H
