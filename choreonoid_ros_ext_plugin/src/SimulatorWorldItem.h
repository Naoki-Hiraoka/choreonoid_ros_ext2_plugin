#ifndef CNOIDROSEXTPLUGIN_SIMULATORWORLDITEM_H
#define CNOIDROSEXTPLUGIN_SIMULATORWORLDITEM_H

#include <cnoid/Item>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <cnoid/BodyItem>
#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <std_srvs/Trigger.h>
#include <gazebo_msgs/SetModelState.h>

namespace cnoid {

  class SimulatorWorldItem : public Item
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    SimulatorWorldItem();

  protected:
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    void setupROS(); bool setupROSDone_ = false;

    void onSimulationAboutToStart(SimulatorItem* simulatorItem);
    void onSimulationStarted();
    void onSimulationStep();

    bool onResetSrv(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool onSetModelStateSrv(gazebo_msgs::SetModelState::Request& req, gazebo_msgs::SetModelState::Response& res);

    SimulatorItem* currentSimulatorItem_;
    ScopedConnectionSet currentSimulatorItemConnections_;
    ros::ServiceServer resetSrv_;
    ros::ServiceServer setModelStateSrv_;
    ros::CallbackQueue callbackQueue_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    int resetStep_=0;
    int setModelStateStep_=0;
  };

  typedef ref_ptr<SimulatorWorldItem> SimulatorWorldItemPtr;
}

#endif
