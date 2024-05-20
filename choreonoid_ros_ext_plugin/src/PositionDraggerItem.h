#ifndef CNOIDROSEXTPLUGIN_POSITIONDRAGGER_ITEM_H
#define CNOIDROSEXTPLUGIN_POSITIONDRAGGER_ITEM_H

#include <cnoid/Item>

#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/PositionDragger>
#include <cnoid/BodyItem>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <cnoid/ToolBar>

namespace cnoid {

  class PositionDraggerItem : public cnoid::Item
  {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext);

    PositionDraggerItem();

  protected:
    virtual void onPositionChanged() override;
    virtual bool store(cnoid::Archive& archive) override;
    virtual bool restore(const cnoid::Archive& archive) override;

    void onSimulationAboutToStart(cnoid::SimulatorItem* simulatorItem);
    void onSimulationStarted();
    void onSimulationStep();
    void onButtonToggled(bool on);
    void onDraggerDragged();

    enum state {ENABLED, DISABLED} state_ = DISABLED;
    cnoid::BodyItemPtr bodyItem_;
    cnoid::SimulatorItem* currentSimulatorItem_;
    cnoid::ScopedConnectionSet currentSimulatorItemConnections_;
    cnoid::Isometry3 targetT_;
    cnoid::Vector6 prevError_;

    std::string linkName_;
    cnoid::Isometry3 localT_;
    double pgain_;
    double dgain_;
    double pgainR_;
    double dgainR_;

    cnoid::PositionDraggerPtr positionDragger_;
    cnoid::ToolBar* toolBar_;
    cnoid::ToolButton* button_;
  };

  typedef ref_ptr<PositionDraggerItem> PositionDraggerItemPtr;
}

#endif
