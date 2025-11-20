#include "SimulatorWorldResetItem.h"
#include <QCoreApplication>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/EigenArchive>
#include <cnoid/EigenUtil>
#include <eigen_conversions/eigen_msg.h>

namespace cnoid {

  void SimulatorWorldResetItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<SimulatorWorldResetItem>("SimulatorWorldResetItem");
  }

  SimulatorWorldResetItem::SimulatorWorldResetItem() {
    if(!ros::isInitialized()){
      QStringList argv_list = QCoreApplication::arguments();
      int argc = argv_list.size();
      char* argv[argc];
      //なぜかわからないがargv_list.at(i).toUtf8().data()のポインタをそのままargvに入れるとros::initがうまく解釈してくれない.
      for(size_t i=0;i<argv_list.size();i++){
        char* data = argv_list.at(i).toUtf8().data();
        size_t dataSize = 0;
        for(size_t j=0;;j++){
          if(data[j] == '\0'){
            dataSize = j;
            break;
          }
        }
        argv[i] = (char *)malloc(sizeof(char) * dataSize+1);
        for(size_t j=0;j<dataSize;j++){
          argv[i][j] = data[j];
        }
        argv[i][dataSize] = '\0';
      }
      ros::init(argc,argv,"choreonoid", ros::init_options::NoSigintHandler/*ctrl-Cで終了するように*/);
      for(size_t i=0;i<argc;i++){
        free(argv[i]);
      }
    }

    SimulationBar::instance()->sigSimulationAboutToStart().connect([&](SimulatorItem* simulatorItem){onSimulationAboutToStart(simulatorItem);});
  }

  void SimulatorWorldResetItem::setupROS() {
    if(this->setupROSDone_) return;
    this->setupROSDone_ = true;

    // コンストラクタ内だとthis->name()が設定されていない
    ros::NodeHandle nh;
    nh.setCallbackQueue(&(this->callbackQueue_));
    this->spinner_ = std::make_shared<ros::AsyncSpinner>(1,&(this->callbackQueue_));
    this->resetSrv_ = nh.advertiseService(this->name()+"/Reset",&SimulatorWorldResetItem::onResetSrv,this);
    this->setModelStateSrv_ = nh.advertiseService(this->name()+"/SetModelState",&SimulatorWorldResetItem::onSetModelStateSrv,this);
    this->spinner_->start();
  }

  bool SimulatorWorldResetItem::store(Archive& archive) {
    return true;
  }

  bool SimulatorWorldResetItem::restore(const Archive& archive) {
    return true;
  }

  void SimulatorWorldResetItem::onSimulationAboutToStart(SimulatorItem* simulatorItem)
  {
    this->currentSimulatorItem_ = simulatorItem;

    this->currentSimulatorItemConnections_.add(
        simulatorItem->sigSimulationStarted().connect(
            [&](){ onSimulationStarted(); }));

    setupROS(); // コンストラクタやcallLaterだとname()やrestore()が未完了
  }

  void SimulatorWorldResetItem::onSimulationStarted()
  {
    this->resetStep_ = 0;
    this->currentSimulatorItem_->addPostDynamicsFunction([&](){ onSimulationStep(); });
  }

  void SimulatorWorldResetItem::onSimulationStep()
  {
    if(this->currentSimulatorItem_ && this->resetStep_>0){
      this->resetStep_--;

      // 動かない TODO
      const std::vector<SimulationBody*>& bodies = this->currentSimulatorItem_->simulationBodies();
      for(int i=0;i<bodies.size();i++){
        //bodies[i]->body()->initializePosition();
        bodies[i]->bodyItem()->restoreInitialState(false);
        bodies[i]->body()->rootLink()->T() = bodies[i]->bodyItem()->body()->rootLink()->T();
        for(size_t j=0;j<bodies[i]->body()->numAllJoints();j++){
          bodies[i]->body()->joint(j)->q() = bodies[i]->bodyItem()->body()->joint(j)->q();
        }
        bodies[i]->body()->initializeState();
      }
    }

    if(this->currentSimulatorItem_ && this->setModelStateStep_>0){
      this->currentSimulatorItem_->clearForcedPositions();
      this->setModelStateStep_--;
    }
  }


  bool SimulatorWorldResetItem::onResetSrv(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
    this->resetStep_ = 1;
    res.success = true;
    return true;
  }

  bool SimulatorWorldResetItem::onSetModelStateSrv(gazebo_msgs::SetModelState::Request& req, gazebo_msgs::SetModelState::Response& res) {
    std::string bodyName = req.model_state.model_name;
    cnoid::Isometry3 pose;
    tf::poseMsgToEigen(req.model_state.pose, pose);

    SimulationBody* body = (this->currentSimulatorItem_) ? this->currentSimulatorItem_->findSimulationBody(bodyName) : nullptr;
    if(body){
      this->currentSimulatorItem_->setForcedPosition(body->bodyItem(), pose);
      setModelStateStep_= 1;
      res.success = true;
      return true;
    }else{
      res.success = false;
      return true;
    }
  }

}

