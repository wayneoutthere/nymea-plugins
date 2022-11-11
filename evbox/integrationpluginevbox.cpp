/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* Copyright 2013 - 2022, nymea GmbH
* Contact: contact@nymea.io
*
* This file is part of nymea.
* This project including source code and documentation is protected by
* copyright law, and remains the property of nymea GmbH. All rights, including
* reproduction, publication, editing and translation, are reserved. The use of
* this project is subject to the terms of a license agreement to be concluded
* with nymea GmbH in accordance with the terms of use of nymea GmbH, available
* under https://nymea.io/license
*
* GNU Lesser General Public License Usage
* Alternatively, this project may be redistributed and/or modified under the
* terms of the GNU Lesser General Public License as published by the Free
* Software Foundation; version 3. This project is distributed in the hope that
* it will be useful, but WITHOUT ANY WARRANTY; without even the implied
* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this project. If not, see <https://www.gnu.org/licenses/>.
*
* For any further details and any questions please contact us under
* contact@nymea.io or see our FAQ/Licensing Information on
* https://nymea.io/license/faq
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include "integrationpluginevbox.h"
#include "plugininfo.h"
#include "plugintimer.h"

#include <QSerialPortInfo>
#include <QSerialPort>
#include <QDataStream>

IntegrationPluginEVBox::IntegrationPluginEVBox()
{

}

IntegrationPluginEVBox::~IntegrationPluginEVBox()
{
}

void IntegrationPluginEVBox::discoverThings(ThingDiscoveryInfo *info)
{
    // Create the list of available serial interfaces

    foreach(QSerialPortInfo port, QSerialPortInfo::availablePorts()) {

        qCDebug(dcEVBox()) << "Found serial port:" << port.portName();
        QString description = port.manufacturer() + " " + port.description();
        ThingDescriptor thingDescriptor(info->thingClassId(), port.portName(), description);
        ParamList parameters;
        foreach (Thing *existingThing, myThings()) {
            if (existingThing->paramValue(evboxThingSerialPortParamTypeId).toString() == port.portName()) {
                thingDescriptor.setThingId(existingThing->id());
                break;
            }
        }
        parameters.append(Param(evboxThingSerialPortParamTypeId, port.portName()));
        thingDescriptor.setParams(parameters);
        info->addThingDescriptor(thingDescriptor);
    }

    info->finish(Thing::ThingErrorNoError);
}

void IntegrationPluginEVBox::setupThing(ThingSetupInfo *info)
{
    Thing *thing = info->thing();
    QString interface = thing->paramValue(evboxThingSerialPortParamTypeId).toString();
    QSerialPort *serialPort = new QSerialPort(interface, this);

    serialPort->setBaudRate(QSerialPort::Baud38400);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setParity(QSerialPort::NoParity);

    connect(serialPort, &QSerialPort::readyRead, thing, [=]() {
        QByteArray data = serialPort->readAll();
        qCDebug(dcEVBox()) << "Data received from serial port:" << data;
        m_inputBuffers[thing].append(data);
        processInputBuffer(thing);
    });

    if (!serialPort->open(QSerialPort::ReadWrite)) {
        qCWarning(dcEVBox()) << "Unable to open serial port";
        info->finish(Thing::ThingErrorHardwareFailure);
        return;
    }

    m_serialPorts.insert(thing, serialPort);

    m_pendingSetups.insert(thing, info);
    connect(info, &ThingSetupInfo::finished, this, [=](){
        m_pendingSetups.remove(thing);
    });
    QTimer::singleShot(2000, info, [=](){
        qCDebug(dcEVBox()) << "Timeout during setup";
        info->finish(Thing::ThingErrorHardwareNotAvailable, QT_TR_NOOP("The EVBox is not responding."));
    });

    sendCommand(thing);
}

void IntegrationPluginEVBox::executeAction(ThingActionInfo *info)
{

    info->finish(Thing::ThingErrorThingClassNotFound);
}

void IntegrationPluginEVBox::sendCommand(Thing *thing)
{
    QByteArray commandData;

    commandData += "80"; // Dst addr
    commandData += "A0"; // Sender address
    commandData += "69"; // Command
    commandData += "00e6"; // Phase 1 max current
    commandData += "008c"; // Phase 2 max current
    commandData += "0154"; // Phase 3 max current
    commandData += "003c"; // Timeout (60 sec)
    commandData += "0028"; // Phase 1 max current after timeout
    commandData += "0050"; // Phase 2 max current after timeout
    commandData += "0046"; // Phase 3 max current after timeout

    commandData += createChecksum(commandData);

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);
    stream << static_cast<quint8>(0x02); // Start of frame
    stream.writeRawData(commandData.data(), commandData.length());
    stream << static_cast<quint8>(0x03); // End of frame

    qCDebug(dcEVBox()) << "data:" << data;
    qCDebug(dcEVBox()) << "Writing" << data.toHex();
    QSerialPort *serialPort = m_serialPorts.value(thing);
    serialPort->write(data);
}

QByteArray IntegrationPluginEVBox::createChecksum(const QByteArray &data) const
{
    QDataStream checksumStream(data);
    quint8 sum = 0;
    quint8 xOr = 0;
    while (!checksumStream.atEnd()) {
        quint8 byte;
        checksumStream >> byte;
        sum += byte;
        xOr ^= byte;
    }
    return QString("%1%2").arg(sum,2,16, QChar('0')).arg(xOr,2,16, QChar('0')).toLocal8Bit();
}

void IntegrationPluginEVBox::processInputBuffer(Thing *thing)
{
    QByteArray packet;
    QDataStream inputStream(m_inputBuffers.value(thing));
    QDataStream outputStream(&packet, QIODevice::WriteOnly);
    bool startFound = false, endFound = false;

    while (!inputStream.atEnd()) {
        quint8 byte;
        inputStream >> byte;
        if (!startFound) {
            if (byte == 0x02) {
                startFound = true;
                continue;
            } else {
                qCWarning(dcEVBox()) << "Discarding byte not matching start of frame 0x" + QString::number(byte, 16);
                continue;
            }
        }

        if (byte == 0x03) {
            endFound = true;
            break;
        }

        outputStream << byte;
    }

    if (startFound && endFound) {
        m_inputBuffers[thing].remove(0, packet.length() + 2);
    } else {
        qCDebug(dcEVBox()) << "Data seems incomplete... Waiting for more...";
        return;
    }

    if (packet.length() < 2) { // In practice it'll be longer, but let's make sure we won't crash checking the checksum
        qCDebug(dcEVBox()) << "Packet is too short";
        return;
    }

    qCDebug(dcEVBox()) << "Packet received:" << packet;

    QByteArray checksum = createChecksum(packet.left(packet.length() - 4));
    if (checksum != packet.right(4)) {
        qCWarning(dcEVBox()) << "Checksum mismatch for incoming packet:" << packet << "Given checksum:" << packet.right(4) << "Expected:" << checksum;
        return;
    }

    if (m_pendingSetups.contains(thing)) {
        qCDebug(dcEVBox()) << "Finishing setup";
        m_pendingSetups.take(thing)->finish(Thing::ThingErrorNoError);
    }

    processDataPacket(thing, packet);
}

void IntegrationPluginEVBox::processDataPacket(Thing *thing, const QByteArray &packet)
{
    qCDebug(dcEVBox()) << thing->name() << packet;

    QDataStream stream(packet);

    char buf[2];
    stream.readRawData(buf, 2); // from

    stream.readRawData(buf, 2); // to
    stream.readRawData(buf, 2); // commandId
    Command command = static_cast<Command>(QByteArray(buf, 2).toInt());

    qCDebug(dcEVBox()) << "Command received:" << command;


}

