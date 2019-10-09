/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                         *
 *  Copyright (C) 2019 Bernhard Trinnes <bernhard.trinnes@nymea.io>        *
 *                                                                         *
 *  This file is part of nymea.                                            *
 *                                                                         *
 *  Guh is free software: you can redistribute it and/or modify            *
 *  it under the terms of the GNU General Public License as published by   *
 *  the Free Software Foundation, version 2 of the License.                *
 *                                                                         *
 *  Guh is distributed in the hope that it will be useful,                 *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the           *
 *  GNU General Public License for more details.                           *
 *                                                                         *
 *  You should have received a copy of the GNU General Public License      *
 *  along with guh. If not, see <http://www.gnu.org/licenses/>.            *
 *                                                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include "deviceplugincoinmarketcap.h"
#include "network/networkaccessmanager.h"
#include "plugininfo.h"

#include <QJsonDocument>

DevicePluginCoinMarketCap::DevicePluginCoinMarketCap()
{
}

void DevicePluginCoinMarketCap::setupDevice(DeviceSetupInfo *info)
{
    Device *device = info->device();

    if (device->deviceClassId() == currentPricesDeviceClassId) {
        getPriceCall(device);

        if(!m_pluginTimer) {
            m_pluginTimer = hardwareManager()->pluginTimerManager()->registerTimer(10);
            connect(m_pluginTimer, &PluginTimer::timeout, this, &DevicePluginCoinMarketCap::onPluginTimer);
        }

        info->finish(Device::DeviceErrorNoError);
        return;
    }
    info->finish(Device::DeviceErrorSetupFailed);
    return;
}

void DevicePluginCoinMarketCap::deviceRemoved(Device *device)
{
    while (m_httpRequests.values().contains(device)) {
        QNetworkReply *reply = m_httpRequests.key(device);
        m_httpRequests.remove(reply);
        reply->deleteLater();
    }

    if (myDevices().empty()) {
        hardwareManager()->pluginTimerManager()->unregisterTimer(m_pluginTimer);
        m_pluginTimer = nullptr;
    }
}

void DevicePluginCoinMarketCap::onPluginTimer()
{
    foreach (Device *device, myDevices()) {
        if (device->deviceClassId() == currentPricesDeviceClassId) {
            getPriceCall(device);
        }
    }
}

void DevicePluginCoinMarketCap::onPriceCallFinished()
{
    QNetworkReply *reply = static_cast<QNetworkReply *>(sender());
    reply->deleteLater();

    int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

    if (!m_httpRequests.contains(reply)) {
        return;
    }

    Device *device = m_httpRequests.take(reply);

    // Check HTTP status code
    if (status != 200 || reply->error() != QNetworkReply::NoError) {
        qCWarning(dcCoinMarketCap()) << "Request error:" << status << reply->errorString();
        device->setStateValue(currentPricesConnectedStateTypeId, false);
    }

    // check JSON file
    QJsonParseError error;
    QJsonDocument jsonResponse = QJsonDocument::fromJson(reply->readAll(), &error);

    if (error.error != QJsonParseError::NoError) {
        qCWarning(dcCoinMarketCap()) << "Update reply JSON error:" << error.errorString();
        return;
    }

    QVariantList list = jsonResponse.toVariant().toList();

    foreach (QVariant element, list) {
        QVariantMap elementMap = element.toMap();
        device->setStateValue(currentPricesConnectedStateTypeId, true);
        double price;

        if (elementMap.value("id").toString() == "bitcoin") {
            price = elementMap.value(QString("price_%1").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower())).toDouble();
            qDebug(dcCoinMarketCap()) << "Bitcoin Price in" << QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower() << price;
            device->setStateValue(currentPricesBtcStateTypeId, price);

        } else if (elementMap.value("id").toString() == "ethereum") {
            price = elementMap.value(QString("price_%1").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower())).toDouble();
            qDebug(dcCoinMarketCap()) << "Etherium Price in" << QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower() << price;
            device->setStateValue(currentPricesEthStateTypeId, price);

        } else if (elementMap.value("id").toString() == "ripple") {
            price = elementMap.value(QString("price_%1").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower())).toDouble();
            qDebug(dcCoinMarketCap()) << "Ripple Price in" << QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower() << price;
            device->setStateValue(currentPricesXrpStateTypeId, price);

        } else if (elementMap.value("id").toString() == "bitcoin-cash") {
            price = elementMap.value(QString("price_%1").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower())).toDouble();
            qDebug(dcCoinMarketCap()) << "Bitcoin-cash Price in" << QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower() << price;
            device->setStateValue(currentPricesBchStateTypeId, price);

        } else if (elementMap.value("id").toString() == "litecoin") {
            price = elementMap.value(QString("price_%1").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower())).toDouble();
            qDebug(dcCoinMarketCap()) << "Litecoin Price in" << QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower() << price;
            device->setStateValue(currentPricesLtcStateTypeId, price);

        } else if (elementMap.value("id").toString() == "nem") {
            price = elementMap.value(QString("price_%1").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower())).toDouble();
            qDebug(dcCoinMarketCap()) << "Nem Price in" << QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower() << price;
            device->setStateValue(currentPricesXemStateTypeId, price);

        } else if (elementMap.value("id").toString() == "ethereum-classic") {
            price = elementMap.value(QString("price_%1").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower())).toDouble();
            qDebug(dcCoinMarketCap()) << "Ethereum Classic Price in" << QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower() << price;
            device->setStateValue(currentPricesEtcStateTypeId, price);

        } else if (elementMap.value("id").toString() == "dash") {
            price = elementMap.value(QString("price_%1").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower())).toDouble();
            device->setStateValue(currentPricesDashStateTypeId, price);

        } else if (elementMap.value("id").toString() == "iota") {
            price = elementMap.value(QString("price_%1").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower())).toDouble();
            device->setStateValue(currentPricesMiotaStateTypeId, price);

        } else if (elementMap.value("id").toString() == "neo") {
            price = elementMap.value(QString("price_%1").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower())).toDouble();
            device->setStateValue(currentPricesAnsStateTypeId, price);
        }
    }
}

void DevicePluginCoinMarketCap::getPriceCall(Device *device)
{
    QUrl url;
    url.setUrl(QString("https://api.coinmarketcap.com/v1/ticker/?convert=%1&limit=30").arg(QString(device->paramValue(currentPricesDeviceFiatParamTypeId).toString()).toLower()));
    QNetworkRequest request;
    request.setUrl(url);
    request.setRawHeader("User-Agent", "nymea 1.0");

    QNetworkReply *reply = hardwareManager()->networkManager()->get(request);
    connect(reply, &QNetworkReply::finished, this, &DevicePluginCoinMarketCap::onPriceCallFinished);

    m_httpRequests.insert(reply, device);
}
