#include "mainwindow.h"

#include <QApplication>
#include <QMessageBox>
#include <QDebug>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // 启用统一的日志输出格式，确保 Release 模式下也能看到调试信息
    qSetMessagePattern("[%{time yyyy-MM-dd hh:mm:ss.zzz}] [%{type}] %{message}");

    qDebug() << "[Main] 正在创建主界面...";

    MainWindow w;

    qDebug() << "[Main] 主界面创建成功，正在显示...";

    w.show();

    if (!w.isVisible()) {
        qCritical() << "[Main] 错误：主界面无法显示！";
        QMessageBox::critical(nullptr, "错误", "主界面无法显示！\n\n程序可能无法正常运行。");
        return -1;
    }

    qDebug() << "[Main] 主界面已成功显示，进入事件循环...";

    return a.exec();
}
