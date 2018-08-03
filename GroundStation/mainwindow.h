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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtCore/QtGlobal>

#include <QMainWindow>

#include <QtSerialPort/QSerialPort>
#include "protocol_parser.h"


QT_BEGIN_NAMESPACE

class QLabel;
class QFile;

namespace Ui {
class MainWindow;
}
namespace sat_protocol {
class SatCommandMsg;
}
QT_END_NAMESPACE

class Console;
class SettingsDialog;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void openSerialPort();
    void closeSerialPort();
    void about();
    void writeData(const QByteArray &data);
    void readData();
    void handleError(QSerialPort::SerialPortError error);
    void on_actionCapture_to_File_triggered();
    void on_actionStop_Capture_triggered();


    void on_actionSendFile_triggered();

private:
    void initActionsConnections();

private:
    void showStatusMessage(const QString &message);
    bool check_command(const QByteArray &data);
    void execute_command(const QByteArray &data);
    void handleCommand(sat_protocol::SatCommandMsg& m_oMsg);

    Ui::MainWindow *ui_;
    QLabel *status_;
    Console *console_;
    SettingsDialog *settings_;
    QSerialPort *serial_;
    QByteArray cmd_;
    QString cap_file_name_;
    std::map<uint32_t, QFile> cap_files_;
    std::map<uint32_t, QString> bal_infos_;
    QString send_file_name_;
    QScopedPointer<QFile> send_file_;
    sat_protocol::protocol_parser* pp_;
    std::vector<uint8_t> data_;
    QString main_msg_;

    bool read_cmd_;
    int cmd_bytes_;
    int cmd_size_;
    bool cap_in_file_;
    bool read_file_;
    char overflow;
    bool has_overflow;
};

#endif // MAINWINDOW_H
