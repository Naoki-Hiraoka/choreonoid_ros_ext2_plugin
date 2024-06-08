#include "CraneItem.h"
#include <QCoreApplication>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/EigenArchive>
#include <cnoid/EigenUtil>
#include <cnoid/LazyCaller>
#include <cnoid/MainWindow>

namespace cnoid {

  void CraneItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<CraneItem>("CraneItem");
  }

  CraneItem::CraneItem() {
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
      ros::init(argc,argv,"choreonoid");
      for(size_t i=0;i<argc;i++){
        free(argv[i]);
      }
    }

    cnoid::callLater([&](){ // コンストラクタ内だとthis->name()が設定されていない
        this->toolBar_ = new ToolBar((this->name()+"Bar").c_str());
        this->toolBar_->setVisibleByDefault(true);
        MainWindow::instance()->addToolBar(this->toolBar_);
        this->button_ = this->toolBar_->addToggleButton(QIcon(":/Base/icon/adjustsize.svg"));
        this->button_->setToolTip((this->linkName_+" Crane").c_str());
        this->button_->sigToggled().connect([&](bool on){ onButtonToggled(on); });
      });

    SimulationBar::instance()->sigSimulationAboutToStart().connect([&](SimulatorItem* simulatorItem){onSimulationAboutToStart(simulatorItem);});

  }

  void CraneItem::setupROS() {
    if(this->setupROSDone_) return;
    this->setupROSDone_ = true;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&(this->callbackQueue_));
    this->spinner_ = std::make_shared<ros::AsyncSpinner>(1,&(this->callbackQueue_));
    this->LiftSrv_ = nh.advertiseService(this->name()+"/lift",&CraneItem::onLiftSrv,this);
    this->spinner_->start();
  }

  void CraneItem::onPositionChanged(){
    BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem){
      if(ownerBodyItem->body()->link(this->linkName_)){
        this->bodyItem_ = ownerBodyItem;
        cnoid::LinkPtr link = ownerBodyItem->body()->link(this->linkName_);
        this->targetHeight_ = (link->p() + link->R() * this->localPos_)[2];
        this->prevp_ = link->p() + link->R() * this->localPos_;
        this->prevR_ = link->R();
        this->prevError_ = 0.0;
      }else{
        MessageView::instance()->putln(this->linkName_+" not found.",
                                       MessageView::ERROR);
        this->bodyItem_ = nullptr;
      }
    } else {
      this->bodyItem_ = nullptr;
    }
  }

  bool CraneItem::store(Archive& archive) {
    archive.write("linkName", this->linkName_);
    write(archive,"localPos", this->localPos_);
    archive.write("maxHeight", this->maxHeight_);
    archive.write("minHeight", this->minHeight_);
    archive.write("upVelocity", this->upVelocity_);
    archive.write("downVelocity", this->downVelocity_);
    archive.write("pgain", this->pgain_);
    archive.write("dgain", this->dgain_);
    archive.write("dgainR", this->dgainR_);
    archive.write("liftStart", this->liftStart_);
    return true;
  }

  bool CraneItem::restore(const Archive& archive) {
    archive.read("linkName", this->linkName_);
    read(archive,"localPos", this->localPos_);
    archive.read("maxHeight", this->maxHeight_);
    archive.read("minHeight", this->minHeight_);
    archive.read("upVelocity", this->upVelocity_);
    archive.read("downVelocity", this->downVelocity_);
    archive.read("pgain", this->pgain_);
    archive.read("dgain", this->dgain_);
    archive.read("dgainR", this->dgainR_);
    archive.read("liftStart", this->liftStart_);
    return true;
  }

  void CraneItem::onSimulationAboutToStart(SimulatorItem* simulatorItem)
  {
    this->currentSimulatorItem_ = simulatorItem;
    this->currentSimulatorItemConnections_.add(
        simulatorItem->sigSimulationStarted().connect(
            [&](){ onSimulationStarted(); }));

    setupROS(); // コンストラクタやcallLaterだとname()やrestore()が未完了
  }

  void CraneItem::onSimulationStarted()
  {
    if(this->bodyItem_){
      cnoid::LinkPtr link = this->bodyItem_->body()->link(this->linkName_);
      if(link){
        this->prevp_ = link->p() + link->R() * this->localPos_;
        this->prevR_ = link->R();
        this->targetHeight_ = this->prevp_[2];
      }
      this->prevError_ = 0.0;
    }
    this->button_->setChecked(this->liftStart_);
    this->currentSimulatorItem_->addPreDynamicsFunction([&](){ onSimulationStep(); });
  }

  void CraneItem::onSimulationStep()
  {
    this->frame_++;
    double dt = this->currentSimulatorItem_->worldTimeStep();

    if(!this->bodyItem_) return;
    SimulationBodyPtr simBody = this->currentSimulatorItem_->findSimulationBody(this->bodyItem_);
    if(!simBody) return;
    cnoid::LinkPtr link = simBody->body()->link(this->linkName_);
    if(!link) return;
    cnoid::Vector3 point = link->p() + link->R()*this->localPos_;

    if(this->state_ != DISABLED){
      switch(this->state_){
      case UP:
        this->targetHeight_ += this->upVelocity_ * dt;
        this->targetHeight_ = std::min(this->targetHeight_, this->maxHeight_);
        break;
      case DOWN:
        this->targetHeight_ -= this->downVelocity_ * dt;
        if(this->targetHeight_< this->minHeight_) this->state_ = DISABLED;
        break;
      default:
        break;
      }
      double error = this->targetHeight_ - point[2];
      double derror = (error - this->prevError_)/dt;
      this->prevError_ = error;
      cnoid::Vector3 f = cnoid::Vector3::Zero();
      f[2] = error * this->pgain_ + derror * this->dgain_;
      f.head<2>() = (- (point-this->prevp_)/dt * this->dgain_).head<2>();
      cnoid::AngleAxis angleAxis = cnoid::AngleAxis(link->R() * this->prevR_.transpose());
      cnoid::Vector3 m = - angleAxis.angle()*angleAxis.axis()/dt * this->dgainR_;

      if(f[2]>0){
        link->addExternalForceAtLocalPosition(f, this->localPos_);

        link->addExternalForceAtLocalPosition( m[0]/2*cnoid::Vector3::UnitZ(), this->localPos_+link->R().transpose()*cnoid::Vector3::UnitY());
        link->addExternalForceAtLocalPosition(-m[0]/2*cnoid::Vector3::UnitZ(), this->localPos_-link->R().transpose()*cnoid::Vector3::UnitY());
        link->addExternalForceAtLocalPosition( m[1]/2*cnoid::Vector3::UnitX(), this->localPos_+link->R().transpose()*cnoid::Vector3::UnitZ());
        link->addExternalForceAtLocalPosition(-m[1]/2*cnoid::Vector3::UnitX(), this->localPos_-link->R().transpose()*cnoid::Vector3::UnitZ());
        link->addExternalForceAtLocalPosition( m[2]/2*cnoid::Vector3::UnitY(), this->localPos_+link->R().transpose()*cnoid::Vector3::UnitX());
        link->addExternalForceAtLocalPosition(-m[2]/2*cnoid::Vector3::UnitY(), this->localPos_-link->R().transpose()*cnoid::Vector3::UnitX());

      }
    }else{
      this->prevError_ = 0.0;
    }

    this->prevp_ = point;
    this->prevR_ = link->R();
  }

  bool CraneItem::onLiftSrv(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res){
    this->button_->setChecked(req.data);
    res.success = true;
    return true;
  }

  void CraneItem::onButtonToggled(bool on){
    if(on) {
      switch(this->state_){
      case DISABLED:
        this->state_ = UP;
        this->targetHeight_ = this->prevp_[2];
        break;
      case DOWN:
        this->state_ = UP;
        break;
      default:
        break;
      }
    } else {
      switch(this->state_){
      case UP:
        this->state_ = DOWN;
        break;
      default:
        break;
      }
    }
  }

}

