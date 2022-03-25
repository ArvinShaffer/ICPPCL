#include "icppcl.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ICPPCL w;
    w.show();
    return a.exec();
}
