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
        qCDebug(dcEVBox()) << "Data received on serial port:" << serialPort->readAll().toHex();
    });

    if (!serialPort->open(QSerialPort::ReadWrite)) {
        qCWarning(dcEVBox()) << "Unable to open serial port";
        info->finish(Thing::ThingErrorHardwareFailure);
        return;
    }

    m_serialPorts.insert(thing, serialPort);

    sendCommand(thing);


    info->finish(Thing::ThingErrorNoError);
}

void IntegrationPluginEVBox::executeAction(ThingActionInfo *info)
{

    info->finish(Thing::ThingErrorThingClassNotFound);
}

void IntegrationPluginEVBox::sendCommand(Thing *thing)
{
    QByteArray commandData;
    QDataStream commandDataStream(&commandData, QIODevice::WriteOnly);

    commandDataStream << static_cast<quint8>(0x80); // Dst addr (chargepoint)
    commandDataStream << static_cast<quint8>(0xA0); // Sender address
    commandDataStream << static_cast<quint8>(0x69); // Command
    commandDataStream << static_cast<quint16>(0x00e6); // Phase 1 max current
    commandDataStream << static_cast<quint16>(0x008c); // Phase 2 max current
    commandDataStream << static_cast<quint16>(0x0154); // Phase 3 max current
    commandDataStream << static_cast<quint16>(0x003c); // Timeout (60 sec)
    commandDataStream << static_cast<quint16>(0x0028); // Phase 1 max current after timeout
    commandDataStream << static_cast<quint16>(0x0050); // Phase 2 max current after timeout
    commandDataStream << static_cast<quint16>(0x0046); // Phase 3 max current after timeout

    QDataStream checksumStream(commandData);
    quint8 sum = 0;
    quint8 xOr = 0;
    while (!checksumStream.atEnd()) {
        quint8 byte;
        checksumStream >> byte;
        sum += byte;
        xOr ^= byte;
    }

    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);
    stream << static_cast<quint8>(0x02); // Start of frame
    commandData.append(QByteArray::number(sum).toHex());
    commandData.append(QByteArray::number(xOr).toHex());
    stream.writeRawData(commandData.toHex().data(), commandData.toHex().length());
    stream << static_cast<quint8>(0x03); // End of frame

    qCDebug(dcEVBox()) << "data:" << data;
    qCDebug(dcEVBox()) << "Writing" << data.toHex();
    QSerialPort *serialPort = m_serialPorts.value(thing);
    serialPort->write(data);
}

