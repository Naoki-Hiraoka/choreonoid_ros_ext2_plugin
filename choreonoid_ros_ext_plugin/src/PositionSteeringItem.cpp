#include "PositionSteeringItem.h"
#include <QCoreApplication>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/EigenArchive>
#include <cnoid/EigenUtil>
#include <cnoid/LazyCaller>
#include <cnoid/MainWindow>

namespace cnoid {

  void PositionSteeringItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<PositionSteeringItem>("PositionSteeringItem");
  }

  PositionSteeringItem::PositionSteeringItem() {
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

  void PositionSteeringItem::setupROS() {
    if(this->setupROSDone_) return;
    this->setupROSDone_ = true;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&(this->callbackQueue_));
    this->spinner_ = std::make_shared<ros::AsyncSpinner>(1,&(this->callbackQueue_));
    this->cmdVelSub_ = nh.subscribe(this->name()+"/cmd_vel",1,&PositionSteeringItem::onCmdVelSub,this);
    this->spinner_->start();
  }

  void PositionSteeringItem::onPositionChanged(){
    BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem){
      if(ownerBodyItem->body()->link(this->linkName_)){
        this->bodyItem_ = ownerBodyItem;
        cnoid::LinkPtr link = ownerBodyItem->body()->link(this->linkName_);
        this->refp_ = link->p() + link->R() * this->localPos_;
        this->refR_ = link->R();
        this->refv_.setZero();
        this->refw_.setZero();
        this->preverror_.setZero();
        this->preverrorR_.setZero();
      }else{
        MessageView::instance()->putln(this->linkName_+" not found.",
                                       MessageView::ERROR);
        this->bodyItem_ = nullptr;
      }
    } else {
      this->bodyItem_ = nullptr;
    }
  }

  bool PositionSteeringItem::store(Archive& archive) {
    archive.write("linkName", this->linkName_);
    write(archive,"localPos", this->localPos_);
    archive.write("pgain", this->pgain_);
    archive.write("dgain", this->dgain_);
    archive.write("pgainR", this->pgainR_);
    archive.write("dgainR", this->dgainR_);
    return true;
  }

  bool PositionSteeringItem::restore(const Archive& archive) {
    archive.read("linkName", this->linkName_);
    read(archive,"localPos", this->localPos_);
    archive.read("pgain", this->pgain_);
    archive.read("dgain", this->dgain_);
    archive.read("pgainR", this->pgainR_);
    archive.read("dgainR", this->dgainR_);
    return true;
  }

  void PositionSteeringItem::onSimulationAboutToStart(SimulatorItem* simulatorItem)
  {
    this->currentSimulatorItem_ = simulatorItem;
    this->currentSimulatorItemConnections_.add(
        simulatorItem->sigSimulationStarted().connect(
            [&](){ onSimulationStarted(); }));

    setupROS(); // コンストラクタやcallLaterだとname()やrestore()が未完了
  }

  void PositionSteeringItem::onSimulationStarted()
  {
    if(this->bodyItem_){
      cnoid::LinkPtr link = this->bodyItem_->body()->link(this->linkName_);
      if(link){
        this->refp_ = link->p() + link->R() * this->localPos_;
        this->refR_ = link->R();
      }
      this->refv_.setZero();
      this->refw_.setZero();
      this->preverror_.setZero();
      this->preverrorR_.setZero();
    }
    this->currentSimulatorItem_->addPreDynamicsFunction([&](){ onSimulationStep(); });
  }

  void PositionSteeringItem::onSimulationStep()
  {
    double dt = this->currentSimulatorItem_->worldTimeStep();

    if(!this->bodyItem_) return;
    SimulationBodyPtr simBody = this->currentSimulatorItem_->findSimulationBody(this->bodyItem_);
    if(!simBody) return;
    cnoid::LinkPtr link = simBody->body()->link(this->linkName_);
    if(!link) return;
    cnoid::Vector3 p = link->p() + link->R()*this->localPos_;
    cnoid::Matrix3 R = link->R();

    this->refp_ += this->refR_ * (this->refv_ * dt);
    if(this->refw_.norm() > 0){
      this->refR_ = cnoid::Matrix3(cnoid::AngleAxisd(this->refR_) * cnoid::AngleAxisd(this->refw_.norm() * dt, this->refw_.normalized()));
    }

    cnoid::Vector3 error = this->refp_ - p;
    cnoid::Vector3 derror = (error - this->preverror_)/dt;
    this->preverror_ = error;

    cnoid::AngleAxisd tmp_errorR = cnoid::AngleAxisd(this->refR_ * R.transpose());
    cnoid::Vector3 errorR = tmp_errorR.angle() * tmp_errorR.axis();
    cnoid::Vector3 derrorR = (errorR - this->preverrorR_)/dt;
    this->preverrorR_ = errorR;

    cnoid::Vector3 f = error * this->pgain_ + derror * this->dgain_;
    cnoid::Vector3 m = errorR * this->pgainR_ + derrorR * this->dgainR_;

    link->addExternalForceAtLocalPosition(f, this->localPos_);
    link->tau_ext() += m;
  }

  void PositionSteeringItem::onCmdVelSub(const geometry_msgs::Twist& msg){
    this->refv_ << msg.linear.x, msg.linear.y, msg.linear.z;
    this->refw_ << msg.angular.x, msg.angular.y, msg.angular.z;
    return;
  }


}

