#ifndef SERIALTHREAD_H
#define SERIALTHREAD_H
#include <QObject>
#include <QMutex>
#include <QtCore>
#include <QSerialPort>
class serialThread : public QThread
{
    Q_OBJECT
signals:
    void ProcessData(const uint8_t* data);
    void showdataTransmitted(const QString& data);
    void EventOpen(const QString& mes);
    void EventClose(const QString& mes);
public slots:
    void readData();
    //void EventRead();
public:
    serialThread();
    void run() override;
    void PortInTransaction(const QString &portName,const QString &baudrate);
    void DataTransaction(const uint8_t* cmd,const uint8_t* data);
    bool closePort();
    bool openPort();
    bool GetportActive();
    void senddata();
public:
    QString m_serialName;
    long m_baudrate;
    bool m_portState = false;
    bool m_send = false;
    bool m_receive = false;
private:
    QByteArray data;
};

#endif // SERIALTHREAD_H
