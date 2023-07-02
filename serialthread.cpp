#include "serialthread.h"
#include <QtCore>
#include <QMutex>
QMutex mutex;
uint8_t bSTX =  0x02 ;
uint8_t bSYNC =  0x16 ;
uint8_t bACK =   0x06 ;
uint8_t bEXT =  0x03 ;
uint8_t writebuffer[10] = {0};
void serialThread::readData()
{
    while (serial->bytesAvailable() >= 10) {
        data = serial->readAll();
    }
   uint8_t* array = reinterpret_cast<uint8_t*>(data.data());
   data.clear();
   emit ProcessData(array);
}

serialThread::serialThread()
{
    connect(serial, &QSerialPort::readyRead, this, &serialThread::readData);
}

void serialThread::run()
{
    while(1){
        if(GetportActive() && (!serial->isOpen())){
            serial->setPortName(m_serialName);
            serial->setBaudRate(m_baudrate);
            serial->setDataBits(QSerialPort::Data8);
            serial->setStopBits(QSerialPort::OneStop);
            serial->setFlowControl(QSerialPort::NoFlowControl);
            serial->setParity(QSerialPort::NoParity);
            bool bstate = serial->open(QIODevice::ReadWrite);
            if(bstate == true){
                qDebug() << "Port is openned" ;
                emit EventOpen("Close");
            }
            else{
                qDebug() << "Cannot open port";
            }
        }
        else if(!GetportActive() && serial->isOpen()){
            serial->close();
            qDebug() << "Port is closed";
            emit EventClose("Open");
        }
        if(serial->isOpen()){
          if(m_send){
              serial->write((char*)writebuffer,10);
              serial->waitForBytesWritten(1000);
              QStringList hexList;
              for(int i =0; i< 10; i++){
                 QString hexString = QString::number(writebuffer[i], 16).rightJustified(2, '0');
                 hexList.append(hexString);
              }
              QString data = "Transmitted data:  " + hexList.join("    ") ;
              m_send = false;
              emit showdataTransmitted(data);
          }
          else{}
        }
    }

}

void serialThread::PortInTransaction(const QString &portName, const QString &baudrate)
{
    bool oke = false;
    m_serialName = portName;
    m_baudrate = baudrate.toULong(&oke,10);
//    if(!isRunning()){
//        start();
//    }
}

void serialThread::DataTransaction(const uint8_t *cmd, const uint8_t *data)
{
    memcpy(&writebuffer[0],&bSTX,1);
    memcpy(&writebuffer[1],cmd,1);
    memcpy(&writebuffer[2],data,6);
    memcpy(&writebuffer[8],&bSYNC,1);
    memcpy(&writebuffer[9],&bEXT,1);
    senddata();
}

bool serialThread::closePort()
{   mutex.lock();
    m_portState = false;
    mutex.unlock();
    return false;
}

bool serialThread::openPort()
{   mutex.lock();
    m_portState = true;
    mutex.unlock();
    return true;
}
bool serialThread::GetportActive()
{
    return m_portState;
}

void serialThread::senddata()
{
    m_send = true;
}
