#ifndef CNOIDROSEXTPLUGIN_STEERING_ITEM_H
#define CNOIDROSEXTPLUGIN_STEERING_ITEM_H

#include <cnoid/Item>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <cnoid/BodyItem>
#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <cnoid/ToolBar>
#include <geometry_msgs/Twist.h>

namespace cnoid {

  class PositionSteeringItem : public Item
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    PositionSteeringItem();

  protected:
    virtual void onPositionChanged() override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    void setupROS(); bool setupROSDone_ = false;

    void onSimulationAboutToStart(SimulatorItem* simulatorItem);
    void onSimulationStarted();
    void onSimulationStep();
    void onCmdVelSub(const geometry_msgs::Twist& msg);

    cnoid::ToolBar* toolBar_;
    cnoid::ToolButton* button_;

    BodyItemPtr bodyItem_;
    SimulatorItem* currentSimulatorItem_;
    ScopedConnectionSet currentSimulatorItemConnections_;
    ros::Subscriber cmdVelSub_;
    ros::CallbackQueue callbackQueue_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;
    cnoid::Vector3 refp_;
    cnoid::Matrix3 refR_;
    cnoid::Vector3 refv_;
    cnoid::Vector3 refw_;
    cnoid::Vector3 preverror_;
    cnoid::Vector3 preverrorR_;

    std::string linkName_;
    cnoid::Vector3 localPos_;
    double pgain_;
    double dgain_;
    double pgainR_;
    double dgainR_;
  };

  typedef ref_ptr<PositionSteeringItem> PositionSteeringItemPtr;
}

#endif
