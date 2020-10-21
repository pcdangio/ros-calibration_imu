#include "fmain.h"
#include "ui_fmain.h"

fmain::fmain(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::fmain)
{
    ui->setupUi(this);
}

fmain::~fmain()
{
    delete ui;
}
