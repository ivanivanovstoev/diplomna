/****************************************************************************
**
** Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
** Copyright (C) 2012 Laszlo Papp <lpapp@kde.org>
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/
/*
 * Copyright (C) 2018  Ivan Stoev
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
*/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "console.h"
#include "settingsdialog.h"

#include <QMessageBox>
#include <QLabel>
#include <QtSerialPort/QSerialPort>
#include <QFileDialog>
#include <QFile>

//! [0]
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::MainWindow),
    cmd_(6,'\0'),
    read_cmd_(false),
    cmd_bytes_(0),
    cmd_size_(5),
    cap_in_file_(false),
    read_file_(false),
    overflow('\0'),
    has_overflow(false)
{
//! [0]
    ui_->setupUi(this);
    console_ = new Console;
    console_->setEnabled(false);
    setCentralWidget(console_);
//! [1]
    serial_ = new QSerialPort(this);
//! [1]
    settings_ = new SettingsDialog;

    ui_->actionConnect->setEnabled(true);
    ui_->actionDisconnect->setEnabled(false);
    ui_->actionQuit->setEnabled(true);
    ui_->actionConfigure->setEnabled(true);

    status_ = new QLabel;
    ui_->statusBar->addWidget(status_);

    initActionsConnections();

    connect(serial_, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
            this, &MainWindow::handleError);

//! [2]
    connect(serial_, &QSerialPort::readyRead, this, &MainWindow::readData);
//! [2]
    connect(console_, &Console::getData, this, &MainWindow::writeData);

    pp_ = new sat_protocol::protocol_parser(1024, *serial_, *console_);
    pp_->SetCallback([this](sat_protocol::SatCommandMsg& msg){handleCommand(msg);});
//! [3]
}
//! [3]

MainWindow::~MainWindow()
{
    delete settings_;
    delete ui_;
}

//! [4]
void MainWindow::openSerialPort()
{
    SettingsDialog::Settings p = settings_->settings();
    serial_->setPortName(p.name);
    serial_->setBaudRate(p.baudRate);
    serial_->setDataBits(p.dataBits);
    serial_->setParity(p.parity);
    serial_->setStopBits(p.stopBits);
    serial_->setFlowControl(p.flowControl);
    if (serial_->open(QIODevice::ReadWrite)) {
        console_->setEnabled(true);
        console_->setLocalEchoEnabled(p.localEchoEnabled);
        ui_->actionConnect->setEnabled(false);
        ui_->actionDisconnect->setEnabled(true);
        ui_->actionConfigure->setEnabled(false);
        showStatusMessage(tr("Connected to %1 : %2, %3, %4, %5, %6")
                          .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                          .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl));
    } else {
        QMessageBox::critical(this, tr("Error"), serial_->errorString());

        showStatusMessage(tr("Open error"));
    }
}
//! [4]

//! [5]
void MainWindow::closeSerialPort()
{
    if (serial_->isOpen())
        serial_->close();
    //console_->setEnabled(false);
    ui_->actionConnect->setEnabled(true);
    ui_->actionDisconnect->setEnabled(false);
    ui_->actionConfigure->setEnabled(true);
    showStatusMessage(tr("Disconnected"));
}
//! [5]

void MainWindow::about()
{
    QMessageBox::about(this, tr("About Simple Terminal"),
                       tr("The <b>Simple Terminal</b> example demonstrates how to "
                          "use the Qt Serial Port module in modern GUI applications "
                          "using Qt, with a menu bar, toolbars, and a status bar."));
}

//! [6]
void MainWindow::writeData(const QByteArray &data)
{
    if (!read_file_)
         serial_->write(data);
    else {
        QString msg = "Sending file please wait ...";
        console_->putData(msg.toUtf8());}
}
//! [6]
//static int __index = 0;
//! [7]
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
template<typename TInputIter>
std::string make_hex(TInputIter first, TInputIter last, bool use_uppercase = true, bool insert_spaces = false)
{
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    if (use_uppercase)
        oss << std::uppercase;
    while (first != last)
    {

       // std::cout << static_cast<int>(*first - 48 )*10 << "|" <<static_cast<int>(*(first+1) - 48) << "|";
        oss << std::setw(2)<< (static_cast<int>(*first - 48)*10 + static_cast<int>(*(first+1) - 48));
        first++;
        if (insert_spaces && first != last)
          oss << " ";
        if (first != last)
          first++;
    }
    std::cout << std::endl;
    return oss.str();
}
template<typename TInputIter>
std::vector<uint8_t>
make_hex_vec(TInputIter first, TInputIter last)
{
    std::vector<uint8_t> ret;
    while (first != last)
    {
        ret.push_back(static_cast<uint8_t>(*first - 48)*10 + static_cast<uint8_t>(*(first+1) - 48));
        first++;
        if (first != last)
          first++;
    }
    return ret;
}


void MainWindow::readData()
{

    char buf[1024];
    qint64 nread = 0;
    while ((nread = serial_->read(buf, 1024)) > 0){
//        data_.insert(std::end(data_), &buf[0], &buf[nread]);
//        QString msg = "Received "; msg+= std::to_string(nread).c_str();
//        msg += " bytes , msg ";
//        for (int i = 0;i < nread;++i){
//          msg += std::to_string(buf[i]).c_str(); msg += "|";
//        }
//        msg += "\nParsing... \n";
//        console_->putData(msg.toUtf8());
        pp_->AddData((unsigned char*)buf,nread);
    }
}
//! enum msg_command_type

QString GetCommandText(uint8_t command, uint8_t ins)
{
    QString ret;
    switch (command){
    case sat_protocol::CMD_ACK:
        ret = "ACK\n";
        break;
    case sat_protocol::CMD_NACK:
        ret = "NACK\n";
        break;
    case sat_protocol::CMD_DATA:
    case sat_protocol::CMD_BALISTIC:
        ret = "Position and orientation information\n";
        if (ins == sat_protocol::INS_SENSOR_INF)
            ret += "Sensors information\n";
        else if (ins == sat_protocol::INS_SENSOR_DATA)
            ret += "Orientation and speed information\n";
        else if (ins == sat_protocol::INS_GPS_DATA)
            ret += "GPS positionning information\n";
        break;
    case sat_protocol::CMD_INFO:
        ret = "DATA\n";
        break;
   }
   return ret;
}

QString& RemoveExt(QString& cap_file_name){
  if (cap_file_name.lastIndexOf('.') > 0)
    cap_file_name.truncate(cap_file_name.lastIndexOf('.'));
  return cap_file_name;
}
void MainWindow::handleCommand(sat_protocol::SatCommandMsg& m_oMsg)
{
    if ((m_oMsg.command_ == sat_protocol::CMD_DATA) && (m_oMsg.instruction_ == sat_protocol::INS_SENSOR_DATA))
    {
         QString msg;
         msg +="Device ID [";
         msg +=std::to_string(m_oMsg.deviceid_).c_str();
         msg +="]\n";
         msg += std::string((char *)m_oMsg.msg_.data(), m_oMsg.msg_.size()).c_str();
         msg += "\n\n";
         bal_infos_[m_oMsg.deviceid_] = msg;
    }
    QString msg = "Ground station softaware v1.2\n\n";
    if (bal_infos_.size() > 0) {
       msg += "Models Orientation\n";
        for (auto i = bal_infos_.begin();i != bal_infos_.end();++i)
            msg += i->second;
    }

    if (!cap_in_file_){
        if ((m_oMsg.command_ == sat_protocol::CMD_DATA) && (m_oMsg.instruction_ == sat_protocol::INS_GPS_DATA))
            main_msg_ = std::string((char *)m_oMsg.msg_.data(), m_oMsg.msg_.size()).c_str() + main_msg_;
//        QString msg = "\nPacket received!";
//        // msg +=std::to_string(m_oMsg.rssi_).c_str() ;
//        msg +="\ndeviceid_ ";
//        msg +=std::to_string(m_oMsg.deviceid_).c_str();
//        msg +="\ncommand_  and instruction_ [";
//        msg += std::to_string(m_oMsg.command_).c_str();
//        msg +="|";
//        msg += std::to_string(m_oMsg.instruction_).c_str();
//        msg +="]\npaket number ";
//        msg += std::to_string(m_oMsg.packet_number_).c_str();
//        msg +="\nerr ";
//        msg +=ErrToByte(m_oMsg.err_);
//        msg +="\nBegin payload\n";
//        msg += GetCommandText(m_oMsg.command_, m_oMsg.instruction_);
//        msg += std::string((char *)m_oMsg.msg_.data(), m_oMsg.msg_.size()).c_str();
//        msg +="\nEnd payload\n";
    }
    else{
        if ((m_oMsg.command_ == sat_protocol::CMD_DATA) && (m_oMsg.instruction_ == sat_protocol::INS_GPS_DATA))
        {
            auto f = cap_files_.find(m_oMsg.deviceid_);
            if (f == cap_files_.end())
            {
                QFile &file = cap_files_[m_oMsg.deviceid_];
                file.setFileName(RemoveExt(cap_file_name_) + std::to_string(m_oMsg.deviceid_).c_str() +".kml");
                file.open(QIODevice::Append);
            }
            cap_files_[m_oMsg.deviceid_].write(std::string((char *)m_oMsg.msg_.data(), m_oMsg.msg_.size()).c_str());
        }

      //  cap_files_->write(msg.toUtf8());
    }
    if (main_msg_.size() > 4096)
        main_msg_ = main_msg_.left(4096);
    msg += main_msg_;
    console_->clear();
    console_->putData(msg.toUtf8());
}
//! [8]
void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), serial_->errorString());
        closeSerialPort();
    }
}
//! [8]

void MainWindow::initActionsConnections()
{
    connect(ui_->actionConnect, &QAction::triggered, this, &MainWindow::openSerialPort);
    connect(ui_->actionDisconnect, &QAction::triggered, this, &MainWindow::closeSerialPort);
    connect(ui_->actionQuit, &QAction::triggered, this, &MainWindow::close);
    connect(ui_->actionConfigure, &QAction::triggered, settings_, &MainWindow::show);
    connect(ui_->actionClear, &QAction::triggered, console_, &Console::clear);
    connect(ui_->actionAbout, &QAction::triggered, this, &MainWindow::about);
    connect(ui_->actionAboutQt, &QAction::triggered, qApp, &QApplication::aboutQt);
}

void MainWindow::showStatusMessage(const QString &message)
{
    status_->setText(message);
}

bool MainWindow::check_command(const QByteArray &data)
{
    if ((data.at(1) == 'S') && (data.at(2) == 'A') && (data.at(3) == 'T'))
        return true;
    return false;
}

void MainWindow::on_actionCapture_to_File_triggered()
{
    cap_file_name_ = QFileDialog::getSaveFileName(this,
                                                    tr("Select file to capture"),
                                                    "",
                                                    "All files (*.*);;Text files (*.txt);;Capture files (*.cpr);;Jpeg files (*.jpg)");
    //cap_file_.reset(new QFile(cap_file_name_));
    //cap_file_->open(QIODevice::Append);
    cap_in_file_ = true;
}

void MainWindow::on_actionStop_Capture_triggered()
{
    if (cap_in_file_)
    {
        cap_in_file_ = false;
        for (auto it = cap_files_.begin(); it != cap_files_.end();it++){
            it->second.close();
        }
        cap_files_.clear();
    }
}

void MainWindow::on_actionSendFile_triggered()
{
    send_file_name_ = QFileDialog::getOpenFileName(this,
                                                    tr("Select file to send"),
                                                    "",
                                                    "All files (*.*);;Text files (*.txt);;Capture files (*.cpr);;Jpeg files (*.jpg)");
    send_file_.reset(new QFile(send_file_name_));
    send_file_->open(QIODevice::ReadOnly);
    read_file_ = true;
    char buf[100];
    qint64 nread = 0;
    while ((nread = send_file_->read(buf, 100)) > 0){
       serial_->write(buf, nread);
    }
    read_file_ = false;
}
