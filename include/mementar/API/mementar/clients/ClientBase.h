#ifndef MEMENTAR_CLIENTBASE_H
#define MEMENTAR_CLIENTBASE_H

#include <vector>
#include <string>

#include <ros/ros.h>

#include "mementar/MementarService.h"

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED     "\x1B[0;91m"
#endif
#ifndef COLOR_ORANGE
#define COLOR_ORANGE  "\x1B[1;33m"
#endif
#ifndef COLOR_GREEN
#define COLOR_GREEN   "\x1B[1;92m"
#endif

namespace mementar
{

class ClientBase
{
public:
  ClientBase(ros::NodeHandle* n, std::string name) : client(n->serviceClient<mementar::MementarService>("mementar/" + name, true))
  {
    n_ = n;
    name_ = name;
  }

  size_t nb() {return cpt;}
  void resetNb() {cpt = 0;}
  static void verbose(bool verbose) { verbose_ = verbose; }

protected:
  ros::ServiceClient client;

  inline std::vector<std::string> call(mementar::MementarService& srv)
  {
    std::vector<std::string> res;
    cpt++;

    if(client.call(srv))
      return srv.response.values;
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call mementar/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<mementar::MementarService>("mementar/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored mementar/" << name_ << COLOR_OFF << std::endl;
        return srv.response.values;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        res.push_back("ERR:SERVICE_FAIL");
        return res;
      }
    }
  }

  inline std::string callStr(mementar::MementarService& srv)
  {
    std::string res = "";
    cpt++;

    if(client.call(srv))
    {
      if(srv.response.values.size())
        return srv.response.values[0];
      else
        return res;
    }
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call mementar/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<mementar::MementarService>("mementar/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored mementar/" << name_ << COLOR_OFF << std::endl;
        if(srv.response.values.size())
          return srv.response.values[0];
        else
          return res;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        res = "ERR:SERVICE_FAIL";
        return res;
      }
    }
  }

  inline ros::Time callStamp(mementar::MementarService& srv)
  {
    ros::Time res;
    cpt++;

    if(client.call(srv))
    {
      if(srv.response.values.size())
        return srv.response.time_value;
      else
        return res;
    }
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call mementar/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<mementar::MementarService>("mementar/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored mementar/" << name_ << COLOR_OFF << std::endl;
        if(srv.response.values.size())
          return srv.response.time_value;
        else
          return res;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        return res;
      }
    }
  }

  inline bool callNR(mementar::MementarService& srv)
  {
    cpt++;

    if(client.call(srv))
      return true;
    else
    {
      if(verbose_)
        std::cout << COLOR_ORANGE << "Failure to call mementar/" << name_ << COLOR_OFF << std::endl;
      client = n_->serviceClient<mementar::MementarService>("mementar/" + name_, true);
      if(client.call(srv))
      {
        if(verbose_)
          std::cout << COLOR_GREEN << "Restored mementar/" << name_ << COLOR_OFF << std::endl;
        return true;
      }
      else
      {
        if(verbose_)
          std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
        return false;
      }
    }
  }

private:
    std::string name_;
    ros::NodeHandle* n_;
    static size_t cpt;
    static bool verbose_;
};

} // namespace mementar

#endif // MEMENTAR_CLIENTBASE_H
