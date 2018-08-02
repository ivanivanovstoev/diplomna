#include "capturetofile.h"
#include "ui_capturetofile.h"

CaptureToFile::CaptureToFile(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CaptureToFile)
{
    ui->setupUi(this);
}

CaptureToFile::~CaptureToFile()
{
    delete ui;
}
