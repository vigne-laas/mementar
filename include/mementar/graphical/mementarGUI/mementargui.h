#ifndef MEMENTAR_MEMENTARGUI_H
#define MEMENTAR_MEMENTARGUI_H

#include <QMainWindow>
#include "include/mementar/graphical/mementarGUI/QCheckBoxExtended.h"
#include "include/mementar/graphical/mementarGUI/CallBackTimer.h"
#include <QTextCursor>

#include <ros/ros.h>
#include <vector>
#include <string>

#include "std_msgs/String.h"

namespace Ui {
class mementarGUI;
}

class mementarGUI : public QMainWindow
{
    Q_OBJECT

public:
  explicit mementarGUI(QWidget *parent = 0);
  ~mementarGUI();

  void init(ros::NodeHandle* n);
  void wait();
  void start();

private:
  Ui::mementarGUI *ui;
  ros::NodeHandle* n_;

  std::map<std::string, ros::Publisher> facts_publishers_;
  std::map<std::string, ros::Publisher> actions_publishers_;
  std::map<std::string, ros::Subscriber> feeder_notifications_subs_;
  std::string feeder_notifications_;

  int time_source_;
  std::atomic<ros::Time> current_time_;
  CallBackTimer timer_;

  void displayInstancesList();
  void displayErrorInfo(const std::string& text);

  std::string vector2string(const std::vector<std::string>& vect);
  std::string vector2html(const std::vector<std::string>& vect);

  void updateTime();

public slots:
  void actionButtonHoverEnterSlot();
  void actionButtonHoverLeaveSlot();

  void actionButtonClickedSlot();

  void nameEditingFinishedSlot();
  void currentTabChangedSlot(int);

  void displayInstancesListSlot();
  void addInstanceSlot();
  void deleteInstanceSlot();
  void saveInstanceSlot();
  void drawInstanceSlot();
  void InstanceNameAddDelChangedSlot(const QString&);
  void InstanceNameChangedSlot(const QString&);
  void timesourceChangedSlot(int index);
  void currentTimeEditingFinishedSlot();

  void feederCallback(const std_msgs::String& msg);
  void feederAddSlot();
  void feederDelSlot();
  void feederCommitSlot();
  void feederCheckoutSlot();
  void createPublisher(const std::string& onto_ns);

signals:
  void feederSetHtmlSignal(QString);
  void setTimeSignal(QString);
};

#endif // MEMENTAR_MEMENTARGUI_H
