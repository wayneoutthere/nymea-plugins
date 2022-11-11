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

#ifndef INTEGRATIONPLUGINEVBOX_H
#define INTEGRATIONPLUGINEVBOX_H

#include "integrations/integrationplugin.h"

class QSerialPort;

class IntegrationPluginEVBox: public IntegrationPlugin
{
    Q_OBJECT

    Q_PLUGIN_METADATA(IID "io.nymea.IntegrationPlugin" FILE "integrationpluginevbox.json")
    Q_INTERFACES(IntegrationPlugin)

public:
    enum Command {
        Command68 = 0x68,
        Command69 = 0x69
    };
    Q_ENUM(Command)

    explicit IntegrationPluginEVBox();
    ~IntegrationPluginEVBox();

    void discoverThings(ThingDiscoveryInfo *info) override;
    void setupThing(ThingSetupInfo *info) override;
    void thingRemoved(Thing *thing) override;
    void executeAction(ThingActionInfo *info) override;

private:
    void sendCommand(Thing *thing, Command command, quint16 maxChargingCurrent);

    QByteArray createChecksum(const QByteArray &data) const;

    void processInputBuffer(Thing *thing);
    void processDataPacket(Thing *thing, const QByteArray &packet);

private:
    QHash<Thing*, QSerialPort*> m_serialPorts;
    QHash<Thing*, ThingSetupInfo*> m_pendingSetups;
    QHash<Thing*, QList<ThingActionInfo*>> m_pendingActions;

    QHash<Thing*, QByteArray> m_inputBuffers;
};

#endif // INTEGRATIONPLUGINEVBOX_H
