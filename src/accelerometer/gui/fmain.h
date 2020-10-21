#ifndef FMAIN_H
#define FMAIN_H

#include <QMainWindow>

namespace Ui {
class fmain;
}

class fmain : public QMainWindow
{
    Q_OBJECT

public:
    explicit fmain(QWidget *parent = nullptr);
    ~fmain();

private:
    Ui::fmain *ui;
};

#endif // FMAIN_H
