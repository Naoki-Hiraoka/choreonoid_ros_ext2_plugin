#ifndef CNOIDROSEXTPLUGIN_OBJECTSTATEPUBLISHER_ITEM_H
#define CNOIDROSEXTPLUGIN_OBJECTSTATEPUBLISHER_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/Camera>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

namespace cnoid {

  class ObjectStatePublisherItem : public ControllerItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    ObjectStatePublisherItem();

    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;

    virtual double timeStep() const override { return timeStep_;};
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override {}
    virtual void stop() override {}

    virtual void onPositionChanged() override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

  protected:
    void setupROS(); bool setupROSDone_ = false;

    ros::Publisher odometryPub_;
    ros::Publisher jointStatePub_;

    std::string bodyName_;
    std::string odometryTopicName_;
    std::string frameId_;
    std::string childFrameId_;
    double poseCovariance_ = 0.0;
    double twistCovariance_ = 0.0;
    std::string jointStateTopicName_;
    double publishRate_ = 100.0;

    cnoid::ControllerIO* io_;
    double timeStep_;

    double time_ = 0.0;

     // input()で取得し、control()で使用される
    nav_msgs::Odometry odometry_;
    sensor_msgs::JointState jointState_;
  };

  typedef ref_ptr<ObjectStatePublisherItem> ObjectStatePublisherItemPtr;
}

#endif
