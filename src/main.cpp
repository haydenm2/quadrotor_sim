#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app{argc, argv};
    MainWindow w{argc, argv};
    w.show();

    a.exec();

    return 0;
    // return a.exec();
}