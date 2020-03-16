/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* Copyright 2013 - 2020, nymea GmbH
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

#include "bluos.h"
#include "extern-plugininfo.h"

#include <QNetworkRequest>
#include <QNetworkReply>
#include <QUrlQuery>
#include <QXmlStreamReader>

BluOS::BluOS(NetworkAccessManager *networkmanager,  QHostAddress hostAddress, int port,  QObject *parent) :
    QObject(parent),
    m_hostAddress(hostAddress),
    m_port(port),
    m_networkManager(networkmanager)
{

}

int BluOS::port()
{
    return m_port;
}

QHostAddress BluOS::hostAddress()
{
    return m_hostAddress;
}

QUuid BluOS::getStatus()
{
    QUuid requestId = QUuid::createUuid();
    QUrl url;
    url.setScheme("http");
    url.setHost(m_hostAddress.toString());
    url.setPort(m_port);
    url.setPath("/Status");
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));
    connect(reply, &QNetworkReply::finished, this, [reply, this] {
        reply->deleteLater();
        int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

        // Check HTTP status code
        if (status != 200 || reply->error() != QNetworkReply::NoError) {
            if (reply->error() == QNetworkReply::HostNotFoundError) {
                emit connectionChanged(false);
            }
            qCWarning(dcBluOS()) << "Request error:" << status << reply->errorString();
            return;
        }
        emit connectionChanged(true);
        QXmlStreamReader xml;
        xml.addData(reply->readAll());
        if (xml.hasError()) {
            qCDebug(dcBluOS()) << "XML Error:" << xml.errorString();
        }

        StatusResponse statusResponse;
        if (xml.readNextStartElement()) {
            if (xml.name() == "status") {
                while(xml.readNextStartElement()){
                    if(xml.name() == "artist"){
                    } else if(xml.name() == "artist"){
                        statusResponse.Artist = xml.readElementText();
                    } else if(xml.name() == "album"){
                        statusResponse.Album = xml.readElementText();
                    } else if(xml.name() == "name"){
                        statusResponse.Name = xml.readElementText();
                    } else if(xml.name() == "service"){
                        statusResponse.Service = xml.readElementText();
                    } else if(xml.name() == "serviceIcon"){
                        statusResponse.ServiceIcon = xml.readElementText();
                    } else if(xml.name() == "shuffle"){
                        statusResponse.Shuffle = xml.readElementText().toInt();
                    } else if(xml.name() == "repeat"){
                        statusResponse.Shuffle = xml.readElementText().toInt();
                    } else if(xml.name() == "state"){
                        statusResponse.PlaybackState = xml.readElementText().toInt();
                    } else if(xml.name() == "volume"){
                        statusResponse.Volume = xml.readElementText().toInt();
                    } else if(xml.name() == "mute"){
                        statusResponse.Mute = xml.readElementText().toInt();
                    }  else {
                        xml.skipCurrentElement();
                    }
                }
            }
        }
        emit statusReceived(statusResponse);
    });
    return requestId;
}

QUuid BluOS::setVolume(uint volume)
{
    QUuid requestId = QUuid::createUuid();

    QUrlQuery query;
    query.addQueryItem("level", QString::number(volume));
    query.addQueryItem("tell_slaves", "off");

    QUrl url;
    url.setScheme("http");
    url.setHost(m_hostAddress.toString());
    url.setPort(m_port);
    url.setPath("/Volume");
    url.setQuery(query);
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));
    connect(reply, &QNetworkReply::finished, this, [requestId, reply, this] {
        reply->deleteLater();
        int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

        // Check HTTP status code
        if (status != 200 || reply->error() != QNetworkReply::NoError) {
            if (reply->error() == QNetworkReply::HostNotFoundError) {
                emit connectionChanged(false);
            }
            emit actionExecuted(requestId, false);
            qCWarning(dcBluOS()) << "Request error:" << status << reply->errorString();
            return;
        }
        emit connectionChanged(true);

        QXmlStreamReader xml;
        xml.addData(reply->readAll());
        if (xml.hasError()) {
            qCDebug(dcBluOS()) << "XML Error:" << xml.errorString();
        }
        int volume = 0;
        bool mute = false;
        if (xml.readNextStartElement()) {
            if (xml.name() == "volume") {
                if(xml.attributes().hasAttribute("mute")) {
                    mute = xml.attributes().value("mute").toInt();
                }
                volume = xml.readElementText().toInt();
            }
        }
        emit volumeReceived(volume, mute);
        emit actionExecuted(requestId, true);
    });
    return requestId;
}

QUuid BluOS::setMute(bool mute)
{
    QUuid requestId = QUuid::createUuid();

    QUrlQuery query;
    query.addQueryItem("mute", QString::number(mute));
    query.addQueryItem("tell_slaves", "off");

    QUrl url;
    url.setScheme("http");
    url.setHost(m_hostAddress.toString());
    url.setPort(m_port);
    url.setPath("/Volume");
    url.setQuery(query);

    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));
    connect(reply, &QNetworkReply::finished, this, [requestId, reply, this] {
        reply->deleteLater();
        int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

        // Check HTTP status code
        if (status != 200 || reply->error() != QNetworkReply::NoError) {
            if (reply->error() == QNetworkReply::HostNotFoundError) {
                emit connectionChanged(false);
            }
            emit actionExecuted(requestId, false);
            qCWarning(dcBluOS()) << "Request error:" << status << reply->errorString();
            return;
        }
        emit connectionChanged(true);
        emit actionExecuted(requestId, true);
    });

    return requestId;
}

QUuid BluOS::play()
{
    return playBackControl(PlaybackCommand::Play);
}

QUuid BluOS::pause()
{
    return playBackControl(PlaybackCommand::Pause);
}

QUuid BluOS::stop()
{
    return playBackControl(PlaybackCommand::Stop);
}

QUuid BluOS::back()
{
    return playBackControl(PlaybackCommand::Back);
}

QUuid BluOS::setShuffle(bool shuffle)
{
    Q_UNUSED(shuffle)
    QUuid requestId = QUuid::createUuid();

    QUrl url;
    url.setScheme("http");
    url.setHost(m_hostAddress.toString());
    url.setPort(m_port);
    url.setPath("/Shuffle");
    QUrlQuery query;
    query.addQueryItem("state", QString::number(shuffle));
    url.setQuery(query);
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));
    connect(reply, &QNetworkReply::finished, this, [requestId, reply, this] {
        reply->deleteLater();
        int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

        // Check HTTP status code
        if (status != 200 || reply->error() != QNetworkReply::NoError) {
            if (reply->error() == QNetworkReply::HostNotFoundError) {
                emit connectionChanged(false);
            }
            emit actionExecuted(requestId, false);
            qCWarning(dcBluOS()) << "Request error:" << status << reply->errorString();
            return;
        }
        emit connectionChanged(true);
        emit actionExecuted(requestId, true);
    });
    return requestId;
}

QUuid BluOS::setRepeat(RepeatMode repeatMode)
{
    Q_UNUSED(repeatMode)
    QUuid requestId = QUuid::createUuid();

    QUrl url;
    url.setScheme("http");
    url.setHost(m_hostAddress.toString());
    url.setPort(m_port);
    url.setPath("/Repeat");
    QUrlQuery query;
    query.addQueryItem("state", QString::number(repeatMode));
    url.setQuery(query);
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));
    connect(reply, &QNetworkReply::finished, this, [requestId, reply, this] {
        reply->deleteLater();
        int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

        // Check HTTP status code
        if (status != 200 || reply->error() != QNetworkReply::NoError) {
            if (reply->error() == QNetworkReply::HostNotFoundError) {
                emit connectionChanged(false);

            }
            emit actionExecuted(requestId, false);
            qCWarning(dcBluOS()) << "Request error:" << status << reply->errorString();
            return;
        }
        emit connectionChanged(true);
        emit actionExecuted(requestId, true);
    });
    return requestId;
}

QUuid BluOS::listPresets()
{
    QUuid requestId = QUuid::createUuid();

    QUrl url;
    url.setScheme("http");
    url.setHost(m_hostAddress.toString());
    url.setPort(m_port);
    url.setPath("/Presets");
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));
    connect(reply, &QNetworkReply::finished, this, [reply, this] {
        reply->deleteLater();
        int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

        // Check HTTP status code
        if (status != 200 || reply->error() != QNetworkReply::NoError) {
            if (reply->error() == QNetworkReply::HostNotFoundError) {
                emit connectionChanged(false);
            }

            qCWarning(dcBluOS()) << "Request error:" << status << reply->errorString();
            return;
        }
        emit connectionChanged(true);
    });

    return requestId;
}

QUuid BluOS::loadPreset(int preset)
{
    Q_UNUSED(preset)
    QUuid requestId = QUuid::createUuid();

    QUrl url;
    url.setScheme("http");
    url.setHost(m_hostAddress.toString());
    url.setPort(m_port);
    url.setPath("/Presets");
    QUrlQuery query;
    query.addQueryItem("id", QString::number(preset));
    url.setQuery(query);
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));
    connect(reply, &QNetworkReply::finished, this, [reply, this] {
        reply->deleteLater();
        int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

        // Check HTTP status code
        if (status != 200 || reply->error() != QNetworkReply::NoError) {
            if (reply->error() == QNetworkReply::HostNotFoundError) {
                emit connectionChanged(false);
            }

            qCWarning(dcBluOS()) << "Request error:" << status << reply->errorString();
            return;
        }
        emit connectionChanged(true);
    });
    return requestId;
}

QUuid BluOS::addGroupPlayer(QHostAddress address, int port)
{
    Q_UNUSED(address)
    Q_UNUSED(port)
    QUuid requestId = QUuid::createUuid();

    QUrl url;
    url.setScheme("http");
    url.setHost(m_hostAddress.toString());
    url.setPort(m_port);
    url.setPath("/AddSlave");
    QUrlQuery query;
    query.addQueryItem("slave", address.toString());
    query.addQueryItem("port", QString::number(port));
    url.setQuery(query);
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));
    connect(reply, &QNetworkReply::finished, this, [reply, this] {
        reply->deleteLater();
        int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

        // Check HTTP status code
        if (status != 200 || reply->error() != QNetworkReply::NoError) {
            if (reply->error() == QNetworkReply::HostNotFoundError) {
                emit connectionChanged(false);
            }

            qCWarning(dcBluOS()) << "Request error:" << status << reply->errorString();
            return;
        }
        emit connectionChanged(true);
    });
    return requestId;
}

QUuid BluOS::removeGroupPlayer(QHostAddress address, int port)
{
    Q_UNUSED(address)
    Q_UNUSED(port)
    QUuid requestId = QUuid::createUuid();

    QUrl url;
    url.setScheme("http");
    url.setHost(m_hostAddress.toString());
    url.setPort(m_port);
    url.setPath("/RemoveSlave");
    QUrlQuery query;
    query.addQueryItem("slave", address.toString());
    query.addQueryItem("port", QString::number(port));
    url.setQuery(query);
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));
    connect(reply, &QNetworkReply::finished, this, [reply, this] {
        reply->deleteLater();
        int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

        // Check HTTP status code
        if (status != 200 || reply->error() != QNetworkReply::NoError) {
            if (reply->error() == QNetworkReply::HostNotFoundError) {
                emit connectionChanged(false);
            }

            qCWarning(dcBluOS()) << "Request error:" << status << reply->errorString();
            return;
        }
        emit connectionChanged(true);
    });
    return requestId;
}

QUuid BluOS::skip()
{
    return playBackControl(PlaybackCommand::Skip);
}

QUuid BluOS::playBackControl(BluOS::PlaybackCommand command)
{
    QUuid requestId = QUuid::createUuid();
    QUrl url;
    url.setScheme("http");
    url.setHost(m_hostAddress.toString());
    url.setPort(m_port);
    switch (command) {
    case PlaybackCommand::Play:
        url.setPath("/Play");
        break;
    case PlaybackCommand::Pause:
        url.setPath("/Pause");
        break;
    case PlaybackCommand::Stop:
        url.setPath("/Stop");
        break;
    case PlaybackCommand::Back:
        url.setPath("/Back");
        break;
    case PlaybackCommand::Skip:
        url.setPath("/Skip");
        break;
    }
    QNetworkRequest request;
    request.setUrl(url);
    QNetworkReply *reply = m_networkManager->get(request);
    qCDebug(dcBluOS()) << "Sending request" << request.url();
    connect(reply, &QNetworkReply::finished, this, [requestId, reply, this] {
        reply->deleteLater();
        int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

        // Check HTTP status code
        if (status != 200 || reply->error() != QNetworkReply::NoError) {
            if (reply->error() == QNetworkReply::HostNotFoundError) {
                emit connectionChanged(false);
            }
            emit actionExecuted(requestId, false);
            qCWarning(dcBluOS()) << "Request error:" << status << reply->errorString();
            return;
        }
        emit connectionChanged(true);

        QXmlStreamReader xml;
        xml.addData(reply->readAll());
        if (xml.hasError()) {
            qCDebug(dcBluOS()) << "XML Error:" << xml.errorString();
        }
        emit actionExecuted(requestId, true);
    });
    return requestId;
}
