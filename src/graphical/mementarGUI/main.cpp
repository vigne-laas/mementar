#include "include/mementar/graphical/mementarGUI/DarkStyle.h"
#include "include/mementar/graphical/mementarGUI/mementargui.h"

#include <QApplication>

#include <csignal>
#include <thread>

#include <ros/package.h>
#include <ros/ros.h>

void spinThread(bool* run)
{
  ros::Rate r(100);
  while(*run == true)
  {
    ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    a.setStyle(new DarkStyle);

    std::string path = ros::package::getPath("mementar");
    path = path + "/docs/img/logo/mementar.ico";
    QIcon icon(QString::fromStdString(path));
    a.setWindowIcon(icon);

    mementarGUI w;
    w.show();

    ros::init(argc, argv, "mementarGUI");

    ros::NodeHandle n;
    bool run = true;

    w.init(&n);
    w.wait();

    w.start();

    std::thread spin_thread(spinThread,&run);

    signal(SIGINT, SIG_DFL);
    auto a_exec = a.exec();

    run = false;
    spin_thread.join();

    return a_exec;
}
