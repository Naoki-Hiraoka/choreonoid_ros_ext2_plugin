#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

#include "ClockPublisherItem.h"
#include "ClockShmItem.h"
#include "CraneItem.h"
#include "CameraPublisherItem.h"
#include "DepthCameraPublisherItem.h"
#include "OdometryCameraPublisherItem.h"
#include "OdometryPublisherItem.h"
#include "SimulatorWorldResetItem.h"
#include "PositionDraggerItem.h"
#include "PositionSteeringItem.h"
#include "ObjectStatePublisherItem.h"

using namespace cnoid;

class ROSExtPlugin : public Plugin
{
public:

    ROSExtPlugin() : Plugin("ROSExt")
    {
      require("Body");
    }

    virtual bool initialize() override
    {
      ClockPublisherItem::initializeClass(this);
      ClockShmItem::initializeClass(this);
      CraneItem::initializeClass(this);
      CameraPublisherItem::initializeClass(this);
      DepthCameraPublisherItem::initializeClass(this);
      OdometryCameraPublisherItem::initializeClass(this);
      OdometryPublisherItem::initializeClass(this);
      SimulatorWorldResetItem::initializeClass(this);
      PositionDraggerItem::initializeClass(this);
      PositionSteeringItem::initializeClass(this);
      ObjectStatePublisherItem::initializeClass(this);
      return true;
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ROSExtPlugin)
