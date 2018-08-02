#ifndef CAPTURETOFILE_H
#define CAPTURETOFILE_H

#include <QDialog>

namespace Ui {
class CaptureToFile;
}

class CaptureToFile : public QDialog
{
    Q_OBJECT

public:
    explicit CaptureToFile(QWidget *parent = 0);
    ~CaptureToFile();

private:
    Ui::CaptureToFile *ui;
};

#endif // CAPTURETOFILE_H
