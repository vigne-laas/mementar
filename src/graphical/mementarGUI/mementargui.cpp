#include "include/mementar/graphical/mementarGUI/mementargui.h"
#include "include/mementar/graphical/mementarGUI/QLineEditExtended.h"
#include "include/mementar/graphical/mementarGUI/QPushButtonExtended.h"
#include "ui_mementargui.h"

#include <QScrollBar>

#include "mementar/MementarService.h"
#include "mementar/StampedString.h"
#include "mementar/MementarAction.h"
#include "std_msgs/String.h"

#include <regex>

#define QUEU_SIZE 1000

mementarGUI::mementarGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::mementarGUI)
{
    ui->setupUi(this);

    QObject::connect(ui->manager_refresh_button, SIGNAL(clicked()),this, SLOT(displayInstancesListSlot()));
    QObject::connect(ui->manager_add_instance_button, SIGNAL(clicked()),this, SLOT(addInstanceSlot()));
    QObject::connect(ui->manager_delete_instance_button, SIGNAL(clicked()),this, SLOT(deleteInstanceSlot()));
    QObject::connect(ui->manager_save_button, SIGNAL(clicked()),this, SLOT(saveInstanceSlot()));
    QObject::connect(ui->manager_draw_button, SIGNAL(clicked()),this, SLOT(drawInstanceSlot()));

    QObject::connect(ui->feeder_add_start_button, SIGNAL(clicked()),this, SLOT(feederAddSlot()));
    QObject::connect(ui->feeder_remove_end_button, SIGNAL(clicked()),this, SLOT(feederDelSlot()));
    QObject::connect(ui->feeder_commit_button, SIGNAL(clicked()),this, SLOT(feederCommitSlot()));
    QObject::connect(ui->feeder_checkout_button, SIGNAL(clicked()),this, SLOT(feederCheckoutSlot()));

    QObject::connect(ui->manager_instance_name_editline, SIGNAL(textChanged(const QString&)), this, SLOT(InstanceNameAddDelChangedSlot(const QString&)));
    QObject::connect(ui->static_instance_name_editline, SIGNAL(textChanged(const QString&)), this, SLOT(InstanceNameChangedSlot(const QString&)));
    QObject::connect(ui->static_instance_name_editline, SIGNAL(editingFinished()),this, SLOT(nameEditingFinishedSlot()));
    QObject::connect(ui->static_tab_widget, SIGNAL(currentChanged(int)),this, SLOT(currentTabChangedSlot(int)));

    QObject::connect( this, SIGNAL( feederSetHtmlSignal(QString) ), ui->feeder_info_edittext, SLOT( setHtml(QString) ) ,Qt::BlockingQueuedConnection);
}

mementarGUI::~mementarGUI()
{
    delete ui;
}

void mementarGUI::init(ros::NodeHandle* n)
{
  n_ = n;
  facts_publishers_["_"] = n_->advertise<mementar::StampedString>("/mementar/insert_fact_stamped", QUEU_SIZE);
  actions_publishers_["_"] = n_->advertise<mementar::MementarAction>("/mementar/insert_action", QUEU_SIZE);
  feeder_notifications_subs_["_"] = n_->subscribe("mementar/feeder_notifications", QUEU_SIZE, &mementarGUI::feederCallback, this);
}

void mementarGUI::wait()
{
  QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                  "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                  "p, li { white-space: pre-wrap; }"
                  "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                  "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">Wainting for </span><span style=\" font-size:12pt; font-weight:600; color:#a40000;\">mementar</span></p></body></html>";
  ui->static_info_area->setHtml(html);
}

void mementarGUI::start()
{
  QString html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                  "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                  "p, li { white-space: pre-wrap; }"
                  "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                  "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600; color:#4e9a06;\">Mementar</span><span style=\" font-size:12pt; color:#4e9a06;\"> detected</span></p></body></html>";
  ui->static_info_area->setHtml(html);
}

void mementarGUI::nameEditingFinishedSlot()
{

}

void mementarGUI::displayErrorInfo(const std::string& text)
{
  std::string html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                  "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
                  "p, li { white-space: pre-wrap; }"
                  "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
                  "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">" + text + "</span></p></body></html>";
  ui->static_info_area->setHtml(QString::fromStdString(html));
}

void mementarGUI::displayInstancesList()
{
  ros::ServiceClient client = n_->serviceClient<mementar::MementarService>("mementar/manage");

  mementar::MementarService srv;
  srv.request.action = "list";

  std::string html;
  if(!client.call(srv))
  {
    html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
            "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
            "p, li { white-space: pre-wrap; }"
            "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
            "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; color:#a40000;\">mementar is not running in multi mode.</span></p></body></html>";
  }
  else
  {
    std::string text = vector2html(srv.response.values);
    html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
            "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
            "p, li { white-space: pre-wrap; }"
            "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
            "<p align=\"left\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; \">" + text + "</span></p></body></html>";
  }

  ui->manager_instances_list_edittext->setHtml(QString::fromStdString(html));
}

void mementarGUI::displayInstancesListSlot()
{
  displayInstancesList();
}

std::string mementarGUI::vector2string(const std::vector<std::string>& vect)
{
  std::string res;
  for(const auto& v : vect)
    res += v + "\n";
  return res;
}

std::string mementarGUI::vector2html(const std::vector<std::string>& vect)
{
  std::string res;
  for(const auto& v : vect)
    res += " - " + v + "<br>";
  return res;
}

void mementarGUI::currentTabChangedSlot(int index)
{
  if(index == 1)
    displayInstancesList();
}

void mementarGUI::addInstanceSlot()
{
  ros::ServiceClient client = n_->serviceClient<mementar::MementarService>("mementar/manage");

  mementar::MementarService srv;
  srv.request.action = "add";
  srv.request.param = ui->manager_instance_name_editline->text().toStdString();

  std::regex base_regex("(.*)=(.*)");
  std::smatch base_match;
  if (std::regex_match(srv.request.param, base_match, base_regex))
  {
    if (base_match.size() == 3)
    {
      srv.request.action = "copy";
      if(facts_publishers_.find(base_match[1].str()) == facts_publishers_.end())
      {
        facts_publishers_[base_match[1].str()] = n_->advertise<mementar::StampedString>("mementar/insert_fact_stamped/" + base_match[1].str(), QUEU_SIZE);
        actions_publishers_[base_match[1].str()] = n_->advertise<mementar::MementarAction>("mementar/insert_action/" + base_match[1].str(), QUEU_SIZE);
      }

      if(feeder_notifications_subs_.find(base_match[1].str()) == feeder_notifications_subs_.end())
        feeder_notifications_subs_[base_match[1].str()] = n_->subscribe("mementar/feeder_notifications", QUEU_SIZE, &mementarGUI::feederCallback, this);
    }
  }
  else
  {
    if(facts_publishers_.find(srv.request.param) == facts_publishers_.end())
    {
      facts_publishers_[srv.request.param] = n_->advertise<mementar::StampedString>("mementar/insert_fact_stamped/" + srv.request.param, QUEU_SIZE);
      actions_publishers_[srv.request.param] = n_->advertise<mementar::MementarAction>("mementar/insert_action/" + srv.request.param, QUEU_SIZE);
    }

    if(feeder_notifications_subs_.find(srv.request.param) == feeder_notifications_subs_.end())
      feeder_notifications_subs_[srv.request.param] = n_->subscribe("mementar/feeder_notifications", QUEU_SIZE, &mementarGUI::feederCallback, this);
  }

  if(!client.call(srv))
    displayErrorInfo("mementar/manage client call failed");
  else
  {
    start();
    if(srv.response.code == 4)
      ui->static_result_editext->setText(QString::fromStdString(srv.request.param + " already created"));
    else if(srv.response.code == 1)
      ui->static_result_editext->setText(QString::fromStdString("fail to stop " + srv.request.param + " : please retry"));
    else
      ui->static_result_editext->setText(QString::fromStdString(""));
    displayInstancesList();
  }
}

void mementarGUI::deleteInstanceSlot()
{
  ros::ServiceClient client = n_->serviceClient<mementar::MementarService>("mementar/manage");

  mementar::MementarService srv;
  srv.request.action = "delete";
  srv.request.param = ui->manager_instance_name_editline->text().toStdString();

  if(!client.call(srv))
    displayErrorInfo("mementar/manage client call failed");
  else
  {
    start();
    if(srv.response.code == 4)
      ui->static_result_editext->setText(QString::fromStdString("Instance \'" + srv.request.param + "\' don't exist"));
    else
      ui->static_result_editext->setText(QString::fromStdString(""));
    displayInstancesList();
  }
}

void mementarGUI::saveInstanceSlot()
{
  std::string service_name = (ui->manager_instance_name_editline->text().toStdString() == "") ? "mementar/actions" : "mementar/actions/" + ui->manager_instance_name_editline->text().toStdString();
  ros::ServiceClient client = n_->serviceClient<mementar::MementarService>(service_name);

  mementar::MementarService srv;
  srv.request.action = "save";
  srv.request.param = ui->manager_save_path_editline->text().toStdString();

  if(!client.call(srv))
    displayErrorInfo("mementar/manage client call failed");
  else
  {
    if(srv.response.code == 4)
      ui->static_result_editext->setText(QString::fromStdString("path \'" + srv.request.param + "\' don't exist"));
    else
      ui->static_result_editext->setText(QString::fromStdString(""));
  }
}

void mementarGUI::drawInstanceSlot()
{
  std::string service_name = (ui->manager_instance_name_editline->text().toStdString() == "") ? "mementar/actions" : "mementar/actions/" + ui->manager_instance_name_editline->text().toStdString();
  ros::ServiceClient client = n_->serviceClient<mementar::MementarService>(service_name);

  mementar::MementarService srv;
  srv.request.action = "draw";
  srv.request.param = ui->manager_save_path_editline->text().toStdString();

  if(!client.call(srv))
    displayErrorInfo("mementar/manage client call failed");
  else
  {
    if(srv.response.code == 4)
      ui->static_result_editext->setText(QString::fromStdString("path \'" + srv.request.param + "\' don't exist"));
    else
      ui->static_result_editext->setText(QString::fromStdString(""));
  }
}

void mementarGUI::InstanceNameAddDelChangedSlot(const QString& text)
{
  if(ui->static_instance_name_editline->text() != text)
    ui->static_instance_name_editline->setText(text);
}

void mementarGUI::InstanceNameChangedSlot(const QString& text)
{
  if(ui->manager_instance_name_editline->text() != text)
    ui->manager_instance_name_editline->setText(text);
}

void mementarGUI::feederCallback(const std_msgs::String& msg)
{
  feeder_notifications_ += "<p>-" + msg.data + "</p>";

  std::string html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
          "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">"
          "p, li { whicommitte-space: pre-wrap; }"
          "</style></head><body style=\" font-family:'Noto Sans'; font-size:9pt; font-weight:400; font-style:normal;\">"
          "<p align=\"left\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; \">" + feeder_notifications_ + "<br></span></p></body></html>";

  ui->feeder_info_edittext->moveCursor(QTextCursor::End);
  feederSetHtmlSignal(QString::fromStdString(html));
  ui->feeder_info_edittext->ensureCursorVisible();
}

void mementarGUI::feederAddSlot()
{
  std::string subject = ui->feeder_subject_editline->text().toStdString();
  std::string predicat = ui->feeder_property_editline->text().toStdString();
  std::string object = ui->feeder_object_editline->text().toStdString();

  if((subject == "") && (predicat == "") && (object == ""))
  {
    return;
  }
  else
  {
    std::string instance_ns = ui->static_instance_name_editline->text().toStdString();
    if(instance_ns == "")
      instance_ns = "_";
    createPublisher(instance_ns);
    if((predicat == "") && (object == ""))
    {
      mementar::MementarAction msg;
      msg.name = subject;
      msg.start_stamp = ros::Time::now();
      msg.end_stamp = ros::Time(0);
      actions_publishers_[instance_ns].publish(msg);
    }
    else
    {
      mementar::StampedString msg;
      msg.data = "[ADD]" + subject + "|" + predicat + "|" + object;
      msg.stamp = ros::Time::now();
      facts_publishers_[instance_ns].publish(msg);
    }
  }
}

void mementarGUI::feederDelSlot()
{
  std::string subject = ui->feeder_subject_editline->text().toStdString();
  std::string predicat = ui->feeder_property_editline->text().toStdString();
  std::string object = ui->feeder_object_editline->text().toStdString();

  if((subject == "") && (predicat == "") && (object == ""))
  {
    return;
  }
  else
  {
    std::string instance_ns = ui->static_instance_name_editline->text().toStdString();
    if(instance_ns == "")
      instance_ns = "_";
    createPublisher(instance_ns);
    if((predicat == "") && (object == ""))
    {
      mementar::MementarAction msg;
      msg.name = subject;
      msg.start_stamp = ros::Time(0);
      msg.end_stamp = ros::Time::now();
      actions_publishers_[instance_ns].publish(msg);
    }
    else
    {
      mementar::StampedString msg;
      msg.data = "[DEL]" + subject + "|" + predicat + "|" + object;
      msg.stamp = ros::Time::now();
      facts_publishers_[instance_ns].publish(msg);
    }
  }
}

void mementarGUI::feederCommitSlot()
{
  mementar::StampedString msg;
  msg.data = "[commit]" + ui->feeder_commit_name_editline->text().toStdString() + "|";
  std::string instance_ns = ui->static_instance_name_editline->text().toStdString();
  if(instance_ns == "")
    instance_ns = "_";
  createPublisher(instance_ns);
  facts_publishers_[instance_ns].publish(msg);
}

void mementarGUI::feederCheckoutSlot()
{
  mementar::StampedString msg;
  msg.data = "[checkout]" + ui->feeder_commit_name_editline->text().toStdString() + "|";
  std::string instance_ns = ui->static_instance_name_editline->text().toStdString();
  if(instance_ns == "")
    instance_ns = "_";
  createPublisher(instance_ns);
  facts_publishers_[instance_ns].publish(msg);
}

void mementarGUI::createPublisher(const std::string& instance_ns)
{
  if(facts_publishers_.find(instance_ns) == facts_publishers_.end())
  {
    facts_publishers_[instance_ns] = n_->advertise<mementar::StampedString>("mementar/insert_fact_stamped/" + instance_ns, QUEU_SIZE);
    while(ros::ok() && (facts_publishers_[instance_ns].getNumSubscribers() == 0))
      ros::spinOnce();
  }

  if(actions_publishers_.find(instance_ns) == actions_publishers_.end())
  {
    actions_publishers_[instance_ns] = n_->advertise<mementar::MementarAction>("mementar/insert_action/" + instance_ns, QUEU_SIZE);
    while(ros::ok() && (actions_publishers_[instance_ns].getNumSubscribers() == 0))
      ros::spinOnce();
  }
}
