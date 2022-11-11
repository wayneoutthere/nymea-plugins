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

    sendCommand(thing, Command69, 0);
}

void IntegrationPluginEVBox::executeAction(ThingActionInfo *info)
{
    Thing *thing = info->thing();

    if (info->action().actionTypeId() == evboxPowerActionTypeId) {
        bool power = info->action().paramValue(evboxPowerActionPowerParamTypeId).toBool();
        sendCommand(info->thing(), Command69, power ? info->thing()->stateValue(evboxMaxChargingCurrentStateTypeId).toUInt() : 0);
    } else if (info->action().actionTypeId() == evboxMaxChargingCurrentActionTypeId) {
        int maxChargingCurrent = info->action().paramValue(evboxMaxChargingCurrentActionMaxChargingCurrentParamTypeId).toInt();
        sendCommand(info->thing(), Command68, maxChargingCurrent);
    }

    m_pendingActions[thing].append(info);
    connect(info, &ThingActionInfo::finished, this, [=](){
        m_pendingActions[thing].removeAll(info);
    });

}

void IntegrationPluginEVBox::sendCommand(Thing *thing, Command command, quint16 maxChargingCurrent)
{
    QByteArray commandData;

    commandData += "80"; // Dst addr
    commandData += "A0"; // Sender address
    commandData += QString::number(command);
    commandData += QString("%1").arg(maxChargingCurrent * 10, 4, 10, QChar('0'));
    commandData += QString("%1").arg(maxChargingCurrent * 10, 4, 10, QChar('0'));
    commandData += QString("%1").arg(maxChargingCurrent * 10, 4, 10, QChar('0'));
    commandData += "003c"; // Timeout (60 sec)
    commandData += QString("%1").arg(0, 4, 10, QChar('0'));
    commandData += QString("%1").arg(0, 4, 10, QChar('0'));
    commandData += QString("%1").arg(0, 4, 10, QChar('0'));

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
    if (!m_pendingActions.value(thing).isEmpty()) {
        m_pendingActions.value(thing).first()->finish(Thing::ThingErrorNoError);
    }

    processDataPacket(thing, packet);
}

void IntegrationPluginEVBox::processDataPacket(Thing *thing, const QByteArray &packet)
{
    qCDebug(dcEVBox()) << thing->name() << packet;

    QDataStream stream(QByteArray::fromHex(packet));

    quint8 from, to, commandId, wallboxCount;
    quint16 minPollInterval, maxChargingCurrent;
    stream >> from >> to >> commandId >> minPollInterval >> maxChargingCurrent >> wallboxCount;



    // Command 69 would give a list of wallboxes (they can be chained apparently) but we only support a single one for now
//    for (int i = 0; i < wallboxCount; i++) {
    if (wallboxCount > 0) {
        quint16 minChargingCurrent, chargingCurrentL1, chargingCurrentL2, chargingCurrentL3, cosinePhiL1, cosinePhiL2, cosinePhiL3, totalEnergyConsumed;
        stream >> minChargingCurrent >> chargingCurrentL1 >> chargingCurrentL2 >> chargingCurrentL3 >> cosinePhiL1 >> cosinePhiL2 >> cosinePhiL3 >> totalEnergyConsumed;

        thing->setStateMinMaxValues(evboxMaxChargingCurrentStateTypeId, minChargingCurrent / 10, maxChargingCurrent / 10);
        thing->setStateValue(evboxMaxChargingCurrentStateTypeId, chargingCurrentL1 / 10);
        // No idea why there's L2, L3...


        qCDebug(dcEVBox()) << "Command received:" << commandId << "min time:" << minPollInterval << "min current:" << minChargingCurrent << "max current:" << maxChargingCurrent << "used current" << chargingCurrentL1 << chargingCurrentL2 << chargingCurrentL3 << "total" << totalEnergyConsumed;


    }
//    char buf[4];
//    stream.readRawData(buf, 2); // from
//    stream.readRawData(buf, 2); // to
//    stream.readRawData(buf, 2); // commandId
//    Command command = static_cast<Command>(QByteArray(buf, 2).toInt());

//    stream.readRawData(buf, 4);
//    quint16 minInterval = QByteArray(buf, 4).toUInt();

//    stream.readRawData(buf, 4);
//    quint16 maxChargingCurrent = QByteArray(buf, 4).toUInt();









}

