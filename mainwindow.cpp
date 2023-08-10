#include "mainwindow.h"
#include "ui_mainwindow.h"
/*-------------Protocol------------
---- 1 byte STX --- 4 bytes Cmd --- 3 bytes option --- 50 bytes data ---
---- 1 byte SYN --- 1 byte ETX*/

// for synchronous
#define     pi  3.14
uint8_t bSTX[] =   {0x02} ;
uint8_t bSYNC[] =  {0x16} ;
uint8_t bACK[] =   {0x06} ;
uint8_t bEXT[] =   {0x03} ;
// for cmd
uint8_t bSMAP[4] = {0x53, 0x4D, 0x41, 0x50};
uint8_t bSPAR[4] = {0x53, 0x50, 0x41, 0x52};
uint8_t bVMOV[4] = {0x56, 0x4D, 0x4F, 0x56};
uint8_t bSTOP[4] = {0x53, 0x54, 0x4F, 0x50};
uint8_t bCDAT[4] = {0x43, 0x44, 0x41, 0x54};
//
uint8_t writebuffer[84] = {0};
uint8_t bOption[3] = {0};

// for motor parameters
uint8_t bPIDM1[6] = {0};
uint8_t bPIDM2[6] = {0};
uint8_t bVelM1[2] = {0};
uint8_t bVelM2[2] = {0};

// for Stanley parameters
uint8_t bKgain_Stanley[2] = {0};
uint8_t bKs_Stanley[2]    = {0};

// for Fuzzy parameters
uint8_t bKe_Fuzzy[2] = {0};
uint8_t bKe_dot_Fuzzy[2] = {0};
uint8_t bVmax_Fuzzy[2] = {0};
uint8_t bDataPoint[10] = {0};
MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent)
        , ui(new Ui::MainWindow)
    {
    ui->setupUi(this);
    uCount = 0;
    uCurrentPoint = 0;
    uCountYaw = 0;
    m_serial = new QSerialPort;
    ui->ccb_baudrate->addItem("1200");
    ui->ccb_baudrate->addItem("2400");
    ui->ccb_baudrate->addItem("4800");
    ui->ccb_baudrate->addItem("19200");
    ui->ccb_baudrate->addItem("115200");
    ui->label_portstatus->setText("Port is closed");
    //
    QList<QSerialPortInfo> com_ports = QSerialPortInfo::availablePorts();
    QSerialPortInfo port;
    ui->ccb_portname->clear();
    foreach(port, com_ports)
    {
        ui->ccb_portname->addItem(port.portName());
    }
    // Initial Value
    bPortOpen = false;
    Timer = new QTimer(this);
    //signal and slots
    connect(m_serial,&QSerialPort::readyRead,this,&MainWindow::readData);
    connect(Timer,&QTimer::timeout,this,&MainWindow::TimeOut_Event);
    // For Configurating Motor1 Chart
    ui->Motor1_Chart->addGraph();
    ui->Motor1_Chart->graph(0)->setScatterStyle(QCPScatterStyle::ssNone);
    ui->Motor1_Chart->graph(0)->setLineStyle(QCPGraph::lsLine);
    ui->Motor1_Chart->xAxis->setRange(0,2000);
    ui->Motor1_Chart->yAxis->setRange(0,2000);
    //
    ui->Motor1_Chart->legend->setVisible(true);
    ui->Motor1_Chart->graph(0)->setName("Velocity");
    ui->Motor1_Chart->legend->setBrush(QColor(255, 255, 255, 150));
    //
    ui->Motor1_Chart->xAxis->setLabel("Time(s)");
    ui->Motor1_Chart->yAxis->setLabel("m/s");
    ui->Motor1_Chart->plotLayout()->insertRow(0);
    ui->Motor1_Chart->plotLayout()->addElement(0, 0, new QCPTextElement(ui->Motor1_Chart, "Velocity of Motor 1", QFont("sans", 10, QFont::Bold)));
    ui->Motor1_Chart->graph(0)->setPen((QPen(QColor(255, 128, 0))));
    ui->Motor1_Chart->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    // for Configurating Motor2 Chart
    ui->Motor2_Chart->addGraph();
    ui->Motor2_Chart->graph(0)->setScatterStyle(QCPScatterStyle::ssNone);
    ui->Motor2_Chart->graph(0)->setLineStyle(QCPGraph::lsLine);
    ui->Motor2_Chart->xAxis->setLabel("Time(s)");
    ui->Motor2_Chart->yAxis->setLabel("m/s");
    ui->Motor2_Chart->plotLayout()->insertRow(0);
    ui->Motor2_Chart->plotLayout()->addElement(0, 0, new QCPTextElement(ui->Motor2_Chart, "Velocity of Motor 2", QFont("sans", 10, QFont::Bold)));
    ui->Motor2_Chart->graph(0)->setName("Velocity Motor 2");
    ui->Motor2_Chart->graph(0)->setPen((QPen(QColor(255, 128, 0))));
    ui->Motor2_Chart->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    // for drawing map
    ui->Map_Chart->addGraph();
    ui->Map_Chart->graph(0)->setScatterStyle(QCPScatterStyle::ssDiamond);
    ui->Map_Chart->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->Map_Chart->xAxis->setLabel("East");
    ui->Map_Chart->yAxis->setLabel("North");
    ui->Map_Chart->plotLayout()->insertRow(0);
    ui->Map_Chart->plotLayout()->addElement(0, 0, new QCPTextElement(ui->Map_Chart, "Map", QFont("sans", 10, QFont::Bold)));
    ui->Map_Chart->graph(0)->setName("Velocity Motor 2");
    ui->Map_Chart->graph(0)->setPen((QPen(QColor(255, 0, 0))));
    ui->Map_Chart->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    //
    ui->Map_Chart->addGraph();
    ui->Map_Chart->graph(1)->setScatterStyle(QCPScatterStyle::ssDiamond);
    ui->Map_Chart->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->Map_Chart->graph(1)->setName("");
    ui->Map_Chart->graph(1)->setPen((QPen(QColor(0, 0, 255))));
    //
    ui->Map_Chart->addGraph();
    ui->Map_Chart->graph(2)->setScatterStyle(QCPScatterStyle::ssDiamond);
    ui->Map_Chart->graph(2)->setLineStyle(QCPGraph::lsNone);
    ui->Map_Chart->graph(2)->setName("");
    ui->Map_Chart->graph(2)->setPen((QPen(QColor(0, 255, 0))));

    //
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::FloatToByte(double dnumber, uint8_t *bOut)
{
    uint16_t temp = dnumber*100;
    bOut[0] = (temp >> 8) & 0xFF;
    bOut[1] = temp & 0xFF;
}

void MainWindow::FloatToByteArrayWithNipes(double dnumber, uint8_t *bOut)
{
    uint16_t temp = dnumber*10000;
    bOut[0] = (temp >> 8) & 0xFF;
    bOut[1] = temp & 0xFF;
}

void MainWindow::on_btn_open_clicked()
{
    bool state = false;
    QString sPortName, sBaudRate;
    sPortName = ui->ccb_portname->currentText();
    sBaudRate = ui->ccb_baudrate->currentText();
    m_baudrate = sBaudRate.toULong(&state,10);
    if(bPortOpen == false && !(m_serial->isOpen())){
        bPortOpen = true;
        m_serial->setPortName(sPortName);
        m_serial->setBaudRate(m_baudrate);
        m_serial->setDataBits(QSerialPort::Data8);
        m_serial->setStopBits(QSerialPort::OneStop);
        m_serial->setFlowControl(QSerialPort::NoFlowControl);
        m_serial->setParity(QSerialPort::NoParity);
        m_serial->setReadBufferSize(4096);
        bool bstate = m_serial->open(QIODevice::ReadWrite);
        if(bstate == true){
            ui->btn_open->setText("close");
            ui->label_portstatus->setText("Port is opened");
            qDebug() << "Port is opened";
        }
        else {
            bPortOpen = false;
            ui->label_portstatus->setText("Cannot open port");
            qDebug() << "Cannot open port";
        }
    }
    else{
        qDebug() <<"Port is closed";
        bPortOpen = false;
        m_serial->close();
        ui->btn_open->setText("open");
        ui->label_portstatus->setText("Port is closed");
    }
}
void MainWindow::on_btn_sendpidm1_clicked()
{
    uint8_t bKpM1[2] = {0}, bKiM1[2] = {0}, bKdM1[2] = {0};
    double dKpM1 = (ui->edt_kp->text()).toDouble(nullptr);
    double dKiM1 = (ui->edt_ki->text()).toDouble(nullptr);
    double dKdM1 = (ui->edt_kd->text()).toDouble(nullptr);
    FloatToByte(dKpM1,bKpM1);
    FloatToByte(dKiM1,bKiM1);
    FloatToByteArrayWithNipes(dKdM1,bKdM1);
    uint8_t index = 0;
    memcpy(bPIDM1 + index,bKpM1,2);
    index += sizeof(bKpM1);
    memcpy(bPIDM1 + index,bKiM1,2);
    index += sizeof(bKiM1);
    memcpy(bPIDM1 + index,bKdM1,2);
}


void MainWindow::on_btn_sendvrefm1_clicked()
{
    uint8_t bVel[2] = {0};
    double dVel = (uint16_t)(ui->edt_vref->text()).toDouble(nullptr);
    int nVel = (uint8_t)dVel;
    int nDigit = (dVel - nVel)*100;
    bVel[0] = nVel  & 0xFF;
    bVel[1] = nDigit & 0xFF;
    memcpy(bVelM1,bVel,2);
}


void MainWindow::on_btn_sendpidm2_clicked()
{
    uint8_t index = 0;
    uint8_t bKpM2[2] = {0}, bKiM2[2] = {0}, bKdM2[2] = {0};
    double dKpM2 = (ui->edt_kp_2->text()).toDouble(nullptr);
    double dKiM2 = (ui->edt_ki_2->text()).toDouble(nullptr);
    double dKdM2 = (ui->edt_kd_2->text()).toDouble(nullptr);
    FloatToByte(dKpM2,bKpM2);
    FloatToByte(dKiM2,bKiM2);
    FloatToByteArrayWithNipes(dKdM2,bKdM2);
    memcpy(bPIDM2 + index,bKpM2,2);
    index += sizeof(bKpM2);
    memcpy(bPIDM2 + index,bKiM2,2);
    index += sizeof(bKiM2);
    memcpy(bPIDM2 + index,bKdM2,2);
}


void MainWindow::on_btn_sendvrefm2_clicked()
{
    uint8_t bVel[2] = {0};
    double dVel = (uint16_t)(ui->edt_vref_2->text()).toDouble(nullptr);
    int nVel = (int)dVel;
    int nDigit = (dVel - nVel)*100;
    bVel[0] = nVel  & 0xFF;
    bVel[1] = nDigit & 0xFF;
    memcpy(bVelM2,bVel,2);
}


void MainWindow::on_btn_cleardata_clicked()
{
    int nIndex = 0;
    memset(writebuffer, 0, 84);
    memcpy(writebuffer + nIndex,bSTX,1);
    nIndex += 1;
    memcpy(writebuffer + nIndex, bSTOP,sizeof(bSTOP));
    writebuffer[83] = bEXT[0];
    writebuffer[82] = bSYNC[0];
    m_serial->write((char*)writebuffer,84);
}


void MainWindow::on_btn_stop_clicked()
{
    int nIndex = 0;
    memset(writebuffer, 0, 84);
    memcpy(writebuffer + nIndex,bSTX,1);
    nIndex += 1;
    memcpy(writebuffer + nIndex, bVMOV,sizeof(bVMOV));
    writebuffer[83] = bEXT[0];
    writebuffer[82] = bSYNC[0];
    m_serial->write((char*)writebuffer,84);

}

void MainWindow::readData()
{
    QStringList hexList;
//    double dVel = 0.0;
    double dEast = 0.0;
    double dNorth = 0.0;
    double dYaw = 0.0;
    if (m_serial->bytesAvailable() == 84) {
        data = m_serial->readAll();
        qDebug() << "Size: " << data.size();
    }
    uint8_t* array = reinterpret_cast<uint8_t*>(data.data());
    if((array[0] == 0x02 && array[83] == 0x03)){
//        for(int i =0; i< 10; i++){
//            QString hexString = QString::number(array[i], 16).rightJustified(2, '0');
//            hexList.append(hexString);
//            }
//    QString string = "Received data:       " + hexList.join("    ") ;
//    data.clear();
//    ui->list_dataReceive->addItem(string);
//    dVel = (array[2] << 8) + array[3];
//    ui->Motor1_Chart->graph(0)->addData(dTime,dVel);
//    ui->Motor1_Chart->rescaleAxes();
//    ui->Motor1_Chart->replot();
//    dTime += 0.01;
        dEast  = 681000  +  ((array[5] << 8) + array[6]) + (double)( (array[7] << 16) + (array[8] << 8) + array[9])/1000000;
        dNorth = 1191000 + ((array[10] << 8) + array[11]) + (double)((array[12] << 16) + (array[13] << 8) + array[14])/1000000;
        if (array[15] == 1){
            dYaw = -(array[16] + ((double)array[17])/100);
        }
        else{
            dYaw = array[16] + ((double)array[17])/100;
        }
        std::cout << "Yaw angle: " << dYaw  << std::fixed << std::setprecision(8) << std::endl;
        std::cout << "East value: " << dEast  << std::fixed << std::setprecision(8) << std::endl;
        std::cout << "North value: " << dNorth  << std::fixed << std::setprecision(8) << std::endl;
        ui->Map_Chart->graph(2)->addData(dEast,dNorth);
        //ui->Map_Chart->rescaleAxes();
        ui->Map_Chart->replot();

    }

}




void MainWindow::on_btn_readMap_clicked()
{
    tinyxml2::XMLDocument doc;
    uint8_t uPoint = 0;
    if (!doc.LoadFile("C:/ImportantThings/Thesis/Software/robot/test.kml"))
    {
        qDebug() << "FILE KML READ SUCCESS";
    }
    else {
        qDebug() << "File KML READ FAIL";
    }
    // Truy cập phần tử <Placemark>
    tinyxml2::XMLElement* placemarkElement = doc.FirstChildElement("kml")->
                          FirstChildElement("Document")->FirstChildElement("Placemark");
    while(placemarkElement != nullptr){
        tinyxml2::XMLElement* lineStringElement = placemarkElement->FirstChildElement("LineString");
        if(lineStringElement != nullptr){
            tinyxml2::XMLElement* coordinatesElement = lineStringElement->FirstChildElement("coordinates");
            if (coordinatesElement != nullptr){
                const char* coordinatesStr = coordinatesElement->GetText();
                if (coordinatesStr != nullptr){
                    // Split the coordinate string into individual coordinates
                    std::string coordinates(coordinatesStr);
                    std::istringstream iss(coordinates);
                    std::string coordinate;
                    while (std::getline(iss, coordinate, ' ')){
                        try{
                        // Parse latitude and longitude from the coordinate string
                        double latitude, longitude;
                        std::istringstream coordinateIss(coordinate);
                        std::string lonLat;
                        std::getline(coordinateIss, lonLat,',');
                        longitude = std::stod(lonLat);
                        std::getline(coordinateIss, lonLat,',');
                        latitude = std::stod(lonLat);
                        // Convert latitude and longitude to UTM
                        int zone;
                        bool northHemisphere;
                        double easting, northing;
                        GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northHemisphere, easting, northing);
                        // Print the UTM coordinates
                        uPoint = uPoint + 1;
                        std::cout << "UTM Coordinates: Zone " << zone << " " << (northHemisphere ? "N" : "S") << " " <<
                                                         std::fixed << std::setprecision(8) << easting << "   " << northing << std::endl;
                        dEast.append(easting);
                        dNorth.append(northing);
                        ui->Map_Chart->graph(0)->addData(easting,northing);
                        ui->Map_Chart->rescaleAxes();
                        ui->Map_Chart->replot();}
                        catch(const std::invalid_argument& e){
                        //qDebug() << "There are " << uPoint << " points";;
                        }
                    }
                }
            }
        }
         placemarkElement = placemarkElement->NextSiblingElement("Placemark");
    }

}


void MainWindow::on_btn_drawMap_clicked()
{
    for(int i = 0; i < dEast.size(); i++){
        x.push_back(dEast[i]);
    }
    for(int i =0; i < dNorth.size(); i++){
        y.push_back(dNorth[i]);
    }
    double ds = 0.5;
    CubicSpline2D sp(x,y);
    std::vector<double>s;
    for(double val = 0.0; val < sp.s.back(); val += ds) {
        s.push_back(val);
    }
    for (double i_s : s) {
        pair<double, double> value = sp.calc_position(i_s, sp.s);
        xval.push_back(value.first);
        yval.push_back(value.second);
    }
    for(int i = 0; i < (int)xval.size(); i++){
        ui->Map_Chart->graph(1)->addData(xval[i],yval[i]);
        ui->Map_Chart->rescaleAxes();
        ui->Map_Chart->replot();

    }
    sp.calc_yaw(xval,yval,dYaw);
    size_t index = dYaw.size();
    for(size_t i = 0; i < index ; i++){
        ui->Motor2_Chart->graph(0)->addData(i,(dYaw[i]*360)/(2*pi));
        ui->Motor2_Chart->rescaleAxes();
        ui->Motor2_Chart->replot();
    }
    qDebug() << "There are " << xval.size() << " points";
    uPointNumber = xval.size();
    QString str = QString::number(xval.size());
    ui->labl_TotalPoint->setText(str);
}
void MainWindow::DoubleToByte(double dEast, double dNorth,uint8_t* bEast, uint8_t* bNorth){
    double North = dNorth - 1191000;
    double East  =  dEast - 681000;
    int nNorth = (int)North;
    int nEast = (int)East;
    int nNorthDigit = (North - nNorth)*1000000;
    int nEastDigit = (East - nEast)*1000000;
    bNorth[0] = (nNorth >> 8) & 0xFF;
    bNorth[1] = nNorth & 0xFF;
    bNorth[2] = (nNorthDigit >> 16) & 0xFF;
    bNorth[3] = (nNorthDigit >> 8)  & 0xFF;
    bNorth[4] = nNorthDigit & 0xFF;
    //
    bEast[0] = (nEast >> 8) & 0xFF;
    bEast[1] = nEast & 0xFF;
    bEast[2] = (nEastDigit >> 16) & 0xFF;
    bEast[3] = (nEastDigit >> 8)  & 0xFF;
    bEast[4] = nEastDigit & 0xFF;
}

void MainWindow::ConvertDoubleToByte(double dYaw, uint8_t *arr)
{
    int nYaw = 0,nDigit = 0;
    if(dYaw < 0){
        arr[0] = 1;
        dYaw = -dYaw;
        nYaw = static_cast<int>(dYaw);
        nDigit = (dYaw - nYaw)*1000000;
        arr[1] = (nYaw & 0xFF);
        arr[2] = (nDigit >> 16) & 0xFF;
        arr[3] = (nDigit >> 8) & 0xFF;
        arr[4] = nDigit & 0xFF;
    }
    else{
        arr[0] = 0;
        nYaw = static_cast<int>(dYaw);
        nDigit = (dYaw - nYaw)*1000000;
        arr[1] = nYaw & 0xFF;
        arr[2] = (nDigit >> 16) & 0xFF;
        arr[3] = (nDigit >> 8) & 0xFF;
        arr[4] =  nDigit & 0xFF;
    }

}

void MainWindow::on_btn_SendMap_clicked()
{
      writebuffer[83] = bEXT[0];
      writebuffer[82] = bSYNC[0];
      Timer->setInterval(200);
      Timer->start();
      this->uCount = (uint32_t)(uPointNumber/5);
      this->uCountYaw = (uint32_t)uPointNumber - 1;
      uCurrentPoint = 0;
}

void MainWindow::TimeOut_Event()
{
    //memset(writebuffer,0,84);
    memcpy(writebuffer,bSTX,1);
    memcpy(writebuffer + 1, bSMAP,sizeof(bSMAP));
    writebuffer[83] = bEXT[0];
    writebuffer[82] = bSYNC[0];
    uint8_t bNorth[5] =   {0};
    uint8_t bEast[5]  =   {0};
    uint8_t bYaw[5]   =   {0};
    uint8_t bCount[2] =   {0};
    double dNorth = 0,dEast = 0, dYaw = 0, dNorthValue = 0, dEastValue = 0, dYawValue = 0;
    if(this->uCount > 0){
        for(uint8_t nIndex = 0; nIndex < 5; nIndex++){
            dEast  = xval[this->uCurrentPoint];
            dNorth = yval[this->uCurrentPoint];
            DoubleToByte(dEast,dNorth,bEast,bNorth);
            dNorthValue = ((bNorth[0] << 8) + bNorth[1]) + (double)((bNorth[2] << 16) +
                           (bNorth[3] << 8) + bNorth[4])/1000000 + 1191000;
            dEastValue = ((bEast[0] << 8) + bEast[1]) + (double)((bEast[2] << 16) +
                           (bEast[3] << 8) + bEast[4])/1000000 + 681000;
            if(this->uCurrentPoint < this->uCountYaw){
                dYaw = this->dYaw[this->uCurrentPoint];
                ConvertDoubleToByte(dYaw,bYaw);
                // for debug
                if(bYaw[0] == 0){
                    dYawValue = ((bYaw[1]) + static_cast<double>(((bYaw[2] << 16) + (bYaw[3] << 8) + bYaw[4]))/1000000);
                }
                else{
                    dYawValue = -((bYaw[1]) + static_cast<double>(((bYaw[2] << 16) + (bYaw[3] << 8) + bYaw[4]))/1000000);
                }
                std::cout << std::fixed << std::setprecision(8) << "Yaw Value:    " << dYawValue << "        " << this->dYaw[uCurrentPoint] << std::endl;
            }
            else{
                memset(bYaw,0,5);
                bYaw[0] = 0x0A;
                bYaw[1] = 0x0D;
                std::cout << "End of life: " << std::endl;
            }
            // for debug
            std::cout << std::fixed << std::setprecision(8) <<"North value: " << dNorthValue << "    East value: " << dEastValue << std::endl;
            std::cout << "             " << yval[this->uCurrentPoint]<<"               " << xval[uCurrentPoint] << std::endl;
            memcpy(writebuffer +  nIndex*5 + 5,bYaw,5);
            memcpy(writebuffer + nIndex*5 + 32,bEast,5);
            memcpy(writebuffer + nIndex*5 + 57,bNorth,5);
            (this->uCurrentPoint)++;
        }
        bCount[0] = ((this->uCurrentPoint) >> 8) & 0xFF;
        bCount[1] = (this->uCurrentPoint) & 0xFF;
        memcpy(writebuffer + 30,bCount,2);
        (this->uCount)--;
        // For Debug
        std::cout << this->uCount << std::endl;
        // For sending
        m_serial->write((char*)writebuffer,84);
        //
     }
    else if(this->uPointNumber == 0){
        Timer->stop();
        qDebug() << "Let's pray";
    }
    else {
        printf("\nhello\n");
        uint8_t uIndex = (uint8_t)((this->uPointNumber) % 5);
        for(uint8_t i = 0 ; i < uIndex; i++){
            dEast  = xval[uCurrentPoint];
            dNorth = yval[uCurrentPoint];
            DoubleToByte(dEast,dNorth,bEast,bNorth);
            dNorthValue = ((bNorth[0] << 8) + bNorth[1]) + (double)((bNorth[2] << 16) +
                           (bNorth[3] << 8) + bNorth[4])/1000000 + 1191000;
            dEastValue = ((bEast[0] << 8) + bEast[1]) + (double)((bEast[2] << 16) +
                           (bEast[3] << 8) + bEast[4])/1000000 + 681000;
            if(this->uCurrentPoint < this->uCountYaw){
                dYaw = this->dYaw[this->uCurrentPoint];
                ConvertDoubleToByte(dYaw,bYaw);
                // for debug
                if(bYaw[0] == 0){
                    dYawValue = ((bYaw[1]) + static_cast<double>(((bYaw[2] << 16) + (bYaw[3] << 8) + bYaw[4]))/1000000);
                }
                else{
                    dYawValue = -((bYaw[1]) + static_cast<double>(((bYaw[2] << 16) + (bYaw[3] << 8) + bYaw[4]))/1000000);
                }
             std::cout << std::fixed << std::setprecision(8) << "Yaw Value:    " << dYawValue << "        " << this->dYaw[uCurrentPoint] << std::endl;
            }
            else{
                memset(bYaw,0,5);
                bYaw[0] = 0x0A;
                bYaw[1] = 0x0D;
                std::cout << "End of life :v" << std::endl;
            }
            memcpy(&writebuffer[5] + i*5,bYaw,5);
            memcpy(&writebuffer[32] + i*5,bEast,5);
            memcpy(&writebuffer[57] + i*5,bNorth,5);
            // for debug

            std::cout << std::fixed << std::setprecision(8) <<"North value: " << dNorthValue << "    East value: " << dEastValue << std::endl;
            std::cout << "             " << yval[uCurrentPoint]<<"               " << xval[uCurrentPoint] << std::endl;
            uCurrentPoint++;
        }
        bCount[0] = ((this->uCurrentPoint) >> 8) & 0xFF;
        bCount[1] = (this->uCurrentPoint) & 0xFF;
        memcpy(&writebuffer[30],bCount,2);
        // for Debug
        std::cout << this->uCurrentPoint  << "   say good bye" << std::endl;
        // For sending
        m_serial->write((char*)writebuffer,84);
        Timer->stop();
    }

}


void MainWindow::on_btn_ClearMap_clicked()
{}



void MainWindow::on_btn_SetStanley_clicked()
{
    uint8_t bKgain[2] = {0}, bKsoft[2] = {0};
    double dKgain = (ui->edt_Kgain->text()).toDouble(nullptr);
    double dKsoft = (ui->edt_Ksoft->text()).toDouble(nullptr);
    FloatToByte(dKgain,bKgain);
    FloatToByte(dKsoft,bKsoft);
    memcpy(bKgain_Stanley,bKgain,sizeof(bKgain));
    memcpy(bKs_Stanley,bKsoft,sizeof(bKsoft));
}


void MainWindow::on_btn_SetFuzzy_clicked()
{
    uint8_t bKe[2] = {0}, bKe_dot[2] = {0}, bVmax[2] = {0} ;
    double dKe = (ui->edt_Ke->text()).toDouble((nullptr));
    double dKe_dot = (ui->edt_Ke_dot->text()).toDouble((nullptr));
    double  dVmax     = (ui->edt_Vmax->text()).toDouble(nullptr);
    FloatToByte(dKe,bKe);
    FloatToByte(dKe_dot,bKe_dot);
    FloatToByte(dVmax,bVmax);
    memcpy(bKe_Fuzzy,bKe,sizeof(bKe));
    memcpy(bKe_dot_Fuzzy,bKe_dot,sizeof(bKe_dot));
    memcpy(bVmax_Fuzzy,bVmax,sizeof(bVmax));
}


void MainWindow::on_btn_SendData_clicked()
{
    int nIndex = 0;
    writebuffer[83] = bEXT[0];
    writebuffer[82] = bSYNC[0];
    memcpy(writebuffer,bSTX,sizeof(bSTX));
    nIndex += sizeof(bSTX);
    memcpy(writebuffer + nIndex, bSPAR, sizeof(bSPAR));
    nIndex += sizeof(bSPAR);
    memcpy(writebuffer + nIndex, bPIDM1, sizeof(bPIDM1));
    nIndex += sizeof(bPIDM1);
    memcpy(writebuffer + nIndex, bPIDM2, sizeof(bPIDM2));
    nIndex += sizeof(bPIDM2);
    memcpy(writebuffer + nIndex, bKgain_Stanley, sizeof(bKgain_Stanley));
    nIndex += sizeof(bKgain_Stanley);
    memcpy(writebuffer + nIndex, bKs_Stanley, sizeof(bKs_Stanley));
    nIndex += sizeof(bKs_Stanley);
    memcpy(writebuffer + nIndex, bKe_Fuzzy, sizeof(bKe_Fuzzy));
    nIndex += sizeof(bKe_Fuzzy);
    memcpy(writebuffer + nIndex, bKe_dot_Fuzzy, sizeof(bKe_dot_Fuzzy));
    nIndex += sizeof(bKe_dot_Fuzzy);
    memcpy(writebuffer + nIndex, bVmax_Fuzzy, sizeof(bVmax_Fuzzy));
    nIndex += sizeof(bVmax_Fuzzy);
    memcpy(writebuffer + nIndex, bSYNC, sizeof(bSYNC));
    nIndex += sizeof(bSYNC);
    memcpy(writebuffer + nIndex, bEXT, sizeof(bEXT));
    m_serial->write((char*)writebuffer,84);
}

