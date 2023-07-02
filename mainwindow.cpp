#include "mainwindow.h"
#include "ui_mainwindow.h"
uint8_t bSTX =  0x02 ;
uint8_t bSYNC =  0x16 ;
uint8_t bACK =   0x06 ;
uint8_t bEXT =  0x03 ;
uint8_t writebuffer[10] = {0};
uint8_t bSPID[1] = { 0x53};
uint8_t bMVEL[1] = { 0x4D};
uint8_t bSTOP[1] = {0x45};
uint8_t bcmd[1] = {0};
uint8_t bdata[6] = {0};
    MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent)
        , ui(new Ui::MainWindow)
    {
    ui->setupUi(this);
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

    //signal and slots
    connect(m_serial,&QSerialPort::readyRead,this,&MainWindow::readData);

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
        m_serial->setFlowControl(QSerialPort::FlowControl());
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
{   QStringList hexList;
    dTime = 0.0;
    memcpy(bcmd,bSPID,1);
    uint8_t bKpM1[2] = {0}, bKiM1[2] = {0}, bKdM1[2] = {0};
    double dKpM1 = (ui->edt_kp->text()).toDouble(nullptr);
    double dKiM1 = (ui->edt_ki->text()).toDouble(nullptr);
    double dKdM1 = (ui->edt_kd->text()).toDouble(nullptr);
    FloatToByte(dKpM1,bKpM1);
    FloatToByte(dKiM1,bKiM1);
    FloatToByteArrayWithNipes(dKdM1,bKdM1);
    uint8_t index = 0;
    memcpy(bdata + index,bKpM1,2);
    index += sizeof(bKpM1);
    memcpy(bdata + index,bKiM1,2);
    index += sizeof(bKiM1);
    memcpy(bdata + index,bKdM1,2);
    //
    memcpy(&writebuffer[0],&bSTX,1);
    memcpy(&writebuffer[1],bcmd,1);
    memcpy(&writebuffer[2],bdata,6);
    memcpy(&writebuffer[8],&bSYNC,1);
    memcpy(&writebuffer[9],&bEXT,1);
    for(int i =0; i< 10; i++){
       QString hexString = QString::number(writebuffer[i], 16).rightJustified(2, '0');
       hexList.append(hexString);
    }
    QString sdata = "Transmitted data:  " + hexList.join("    ") ;
    ui->list_dataReceive->addItem(sdata);
    m_serial->write((char*)writebuffer,10);
}


void MainWindow::on_btn_sendvrefm1_clicked()
{
    QStringList hexList;
    uint8_t bVel[2] = {0};
    memcpy(bcmd,bMVEL,1);
    uint16_t dVel = (uint16_t)(ui->edt_vref->text()).toDouble(nullptr);
    //FloatToByte(dVel,bVel);
    bVel[0] = (dVel >> 8) & 0xFF;
    bVel[1] = dVel & 0xFF;
    memcpy(bdata,bVel,2);
    //
    memcpy(&writebuffer[0],&bSTX,1);
    memcpy(&writebuffer[1],bcmd,1);
    memcpy(&writebuffer[2],bdata,6);
    memcpy(&writebuffer[8],&bSYNC,1);
    memcpy(&writebuffer[9],&bEXT,1);
    for(int i =0; i< 10; i++){
       QString hexString = QString::number(writebuffer[i], 16).rightJustified(2, '0');
       hexList.append(hexString);
    }
    QString sdata = "Transmitted data:  " + hexList.join("    ") ;
    ui->list_dataReceive->addItem(sdata);
    m_serial->write((char*)writebuffer,10);
}


void MainWindow::on_btn_sendpidm2_clicked()
{
    QStringList hexList;
    dTime = 0.0;
    memcpy(bcmd,bSPID,1);
    uint8_t bKpM2[2] = {0}, bKiM2[2] = {0}, bKdM2[2] = {0};
    uint8_t index = 0;
    double dKpM2 = (ui->edt_kp_2->text()).toDouble(nullptr);
    double dKiM2 = (ui->edt_ki_2->text()).toDouble(nullptr);
    double dKdM2 = (ui->edt_kd_2->text()).toDouble(nullptr);
    FloatToByte(dKpM2,bKpM2);
    FloatToByte(dKiM2,bKiM2);
    FloatToByteArrayWithNipes(dKdM2,bKdM2);
    memcpy(bdata + index,bKpM2,2);
    index += sizeof(bKpM2);
    memcpy(bdata + index,bKiM2,2);
    index += sizeof(bKiM2);
    memcpy(bdata + index,bKdM2,2);
    //
    memcpy(&writebuffer[0],&bSTX,1);
    memcpy(&writebuffer[1],bcmd,1);
    memcpy(&writebuffer[2],bdata,6);
    memcpy(&writebuffer[8],&bSYNC,1);
    memcpy(&writebuffer[9],&bEXT,1);
    //
    for(int i =0; i< 10; i++){
       QString hexString = QString::number(writebuffer[i], 16).rightJustified(2, '0');
       hexList.append(hexString);
    }
    QString sdata = "Transmitted data:  " + hexList.join("    ") ;
    ui->list_dataReceive->addItem(sdata);
    m_serial->write((char*)writebuffer,10);
}


void MainWindow::on_btn_sendvrefm2_clicked()
{
    QStringList hexList;
    uint8_t bVel[2] = {0};
    memcpy(bcmd,bMVEL,1);
    uint16_t dVel = (uint16_t)(ui->edt_vref_2->text()).toDouble(nullptr);
    //FloatToByte(dVel,bVel);
    bVel[0] = (dVel >> 8) & 0xFF;
    bVel[1] = dVel & 0xFF;
    memcpy(bdata,bVel,2);
    //
    memcpy(&writebuffer[0],&bSTX,1);
    memcpy(&writebuffer[1],bcmd,1);
    memcpy(&writebuffer[2],bdata,6);
    memcpy(&writebuffer[8],&bSYNC,1);
    memcpy(&writebuffer[9],&bEXT,1);
    for(int i =0; i< 10; i++){
       QString hexString = QString::number(writebuffer[i], 16).rightJustified(2, '0');
       hexList.append(hexString);
    }
    QString sdata = "Transmitted data:  " + hexList.join("    ") ;
    ui->list_dataReceive->addItem(sdata);
    m_serial->write((char*)writebuffer,10);
}


void MainWindow::on_btn_cleardata_clicked()
{
    ui->list_dataReceive->clear();
}


void MainWindow::on_btn_stop_clicked()
{
   QStringList hexList;
   memcpy(bcmd,bSTOP,1);
   memset(bdata,0,6);
   memcpy(&writebuffer[0],&bSTX,1);
   memcpy(&writebuffer[1],bcmd,1);
   memcpy(&writebuffer[2],bdata,6);
   memcpy(&writebuffer[8],&bSYNC,1);
   memcpy(&writebuffer[9],&bEXT,1);
   for(int i =0; i< 10; i++){
      QString hexString = QString::number(writebuffer[i], 16).rightJustified(2, '0');
      hexList.append(hexString);
   }
   QString sdata = "Transmitted data:  " + hexList.join("    ") ;
   ui->list_dataReceive->addItem(sdata);
   m_serial->write((char*)writebuffer,10);


}

void MainWindow::readData()
{
    QStringList hexList;
    double dVel = 0.0;
    if (m_serial->bytesAvailable() == 10) {
        data = m_serial->readAll();
        qDebug() << "Size: " << data.size();
    }
    uint8_t* array = reinterpret_cast<uint8_t*>(data.data());
    if((array[0] == 0x02 && array[9] == 0x03)){
        for(int i =0; i< 10; i++){
            QString hexString = QString::number(array[i], 16).rightJustified(2, '0');
            hexList.append(hexString);
            }
    QString string = "Received data:       " + hexList.join("    ") ;
    data.clear();
    ui->list_dataReceive->addItem(string);
    dVel = (array[2] << 8) + array[3];
    ui->Motor1_Chart->graph(0)->addData(dTime,dVel);
    ui->Motor1_Chart->rescaleAxes();
    ui->Motor1_Chart->replot();
    dTime += 0.01;
    }
}




void MainWindow::on_btn_readMap_clicked()
{
    tinyxml2::XMLDocument doc;
    uint8_t uPoint = 0;
    if (!doc.LoadFile("C:/Important Things/Thesis/Software/robot/UTM2.kml"))
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
    vector<double>xval;
    vector<double>yval;
    vector<double>x ;
    vector<double>y ;
    for(int i = 0; i < dEast.size(); i++){
        x.push_back(dEast[i]);
    }
    for(int i =0; i < dNorth.size(); i++){
        y.push_back(dNorth[i]);
    }
    CubicSpline2D sp(x, y);
    double ds = 0.5;
    std::vector<double> s;
    for (double val = 0.0; val < sp.s.back(); val += ds) {
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
}

