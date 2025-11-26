#include "SimulatorWorldItem.h"
#include <QCoreApplication>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/EigenArchive>
#include <cnoid/EigenUtil>
#include <cnoid/DyBody>
#include <eigen_conversions/eigen_msg.h>

namespace cnoid {

  void SimulatorWorldItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<SimulatorWorldItem>("SimulatorWorldItem");
  }

  SimulatorWorldItem::SimulatorWorldItem() {
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

  void SimulatorWorldItem::setupROS() {
    if(this->setupROSDone_) return;
    this->setupROSDone_ = true;

    // コンストラクタ内だとthis->name()が設定されていない
    ros::NodeHandle nh;
    nh.setCallbackQueue(&(this->callbackQueue_));
    this->spinner_ = std::make_shared<ros::AsyncSpinner>(1,&(this->callbackQueue_));
    this->resetSrv_ = nh.advertiseService(this->name()+"/Reset",&SimulatorWorldItem::onResetSrv,this);
    this->setModelStateSrv_ = nh.advertiseService(this->name()+"/SetModelState",&SimulatorWorldItem::onSetModelStateSrv,this);
    this->spinner_->start();
  }

  bool SimulatorWorldItem::store(Archive& archive) {
    return true;
  }

  bool SimulatorWorldItem::restore(const Archive& archive) {
    return true;
  }

  void SimulatorWorldItem::onSimulationAboutToStart(SimulatorItem* simulatorItem)
  {
    this->currentSimulatorItem_ = simulatorItem;

    this->currentSimulatorItemConnections_.add(
        simulatorItem->sigSimulationStarted().connect(
            [&](){ onSimulationStarted(); }));

    setupROS(); // コンストラクタやcallLaterだとname()やrestore()が未完了
  }

  void SimulatorWorldItem::onSimulationStarted()
  {
    this->resetStep_ = 0;
    this->currentSimulatorItem_->addPostDynamicsFunction([&](){ onSimulationStep(); });
  }

  void SimulatorWorldItem::onSimulationStep()
  {
    if(this->currentSimulatorItem_ && this->resetStep_>0){
      this->resetStep_--;

      const std::vector<SimulationBody*>& bodies = this->currentSimulatorItem_->simulationBodies();
      for(int i=0;i<bodies.size();i++){
        //bodies[i]->body()->initializePosition();
        bodies[i]->bodyItem()->restoreInitialState(false);
        bodies[i]->body()->rootLink()->T() = bodies[i]->bodyItem()->body()->rootLink()->T();
        for(size_t j=0;j<bodies[i]->body()->numAllJoints();j++){
          bodies[i]->body()->joint(j)->q() = bodies[i]->bodyItem()->body()->joint(j)->q();
        }
        bodies[i]->body()->initializeState();

        // AISTSimulatorのForwardDynamicsABMは、calcABMFirstHalf()を前回周期の値を用いて既に行っているため、body()->initializeState()した状態で残りのcalcABMLastHalf()を行うと整合性がとれずnanになったり吹っ飛んだりする. dd()に大きな値をセットするととりあえずまともに動く.
        for(int l=0;l<bodies[i]->body()->numLinks();l++){
          cnoid::LinkPtr link = bodies[i]->body()->link(l);
          cnoid::DyLinkPtr dyLink = cnoid::dynamic_pointer_cast<cnoid::DyLink>(link);
          if(dyLink) dyLink->dd() = 1e10;
        }
      }
    }

    if(this->currentSimulatorItem_ && this->setModelStateStep_>0){
      this->setModelStateStep_--;
      if(this->setModelStateStep_ == 0){
        this->currentSimulatorItem_->clearForcedPositions();
      }
    }
  }


  bool SimulatorWorldItem::onResetSrv(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
    this->resetStep_ = 1;
    res.success = true;
    return true;
  }

  bool SimulatorWorldItem::onSetModelStateSrv(gazebo_msgs::SetModelState::Request& req, gazebo_msgs::SetModelState::Response& res) {
    std::string bodyName = req.model_state.model_name;
    cnoid::Isometry3 pose;
    tf::poseMsgToEigen(req.model_state.pose, pose);

    SimulationBody* body = (this->currentSimulatorItem_) ? this->currentSimulatorItem_->findSimulationBody(bodyName) : nullptr;
    if(body){
      this->currentSimulatorItem_->setForcedPosition(body->bodyItem(), pose);
      setModelStateStep_= 2; // postDynamicsの直前に呼ばれた場合に機能しないので2loop回す
      res.success = true;
      return true;
    }else{
      res.success = false;
      return true;
    }
  }

}

