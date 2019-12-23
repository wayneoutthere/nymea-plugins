/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                         *
 *  Copyright (C) 2020 Bernhard Trinnes <bernhard.trinnes@nymea.io>        *
 *                                                                         *
 *  This file is part of nymea.                                            *
 *                                                                         *
 *  This library is free software; you can redistribute it and/or          *
 *  modify it under the terms of the GNU Lesser General Public             *
 *  License as published by the Free Software Foundation; either           *
 *  version 2.1 of the License, or (at your option) any later version.     *
 *                                                                         *
 *  This library is distributed in the hope that it will be useful,        *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU      *
 *  Lesser General Public License for more details.                        *
 *                                                                         *
 *  You should have received a copy of the GNU Lesser General Public       *
 *  License along with this library; If not, see                           *
 *  <http://www.gnu.org/licenses/>.                                        *
 *                                                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef NANOLEAF_H
#define NANOLEAF_H

#include <QObject>
#include <QTimer>
#include <QUuid>
#include <QHostAddress>
#include <QColor>

#include "network/networkaccessmanager.h"
#include "devices/device.h"

class Nanoleaf : public QObject
{
    Q_OBJECT
public:

    explicit Nanoleaf(NetworkAccessManager *networkManager, const QHostAddress &address, int port = 16021, QObject *parent = nullptr);
    void setIpAddress(const QHostAddress &address);
    QHostAddress ipAddress();

    void setPort(int port);
    int port();

    //AUTHORIZATION
    void addUser();
    void deleteUser();

    //GET ALL PANEL INFORMATION

    //STATES
    void getPower();
    void getHue();
    void getBrightness();
    void getSaturation();
    void getColorTemperature();
    void getColorMode();

    QUuid setPower(bool power);
    QUuid setHue(QColor color);
    QUuid setBrightness(int percentage);
    QUuid setSaturation(int percentage);
    QUuid setColorTemperature(int mired);


    //EFFECTS

    //PANEL LAYOUT

    //IDENTIFY


    //EXTERNAL CONTROL


    //RHYTHM

private:
    NetworkAccessManager *m_networkManager = nullptr;
    QString m_authToken;
    QHostAddress m_address;
    int m_port;

signals:
    void connectionChanged(bool connected);
    void authenticationStatusChanged(bool authenticated);
    void requestedExecuted(QUuid requestId, bool success);

    void powerReceived(bool power);
    void brightnessReceived(int percentage);
    void colorModeReceived();
    void hueReceived(QColor color);
    void saturationReceived(int percentage);
};

#endif // NANOLEAF_H
