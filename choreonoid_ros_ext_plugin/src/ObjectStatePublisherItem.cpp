#include "ObjectStatePublisherItem.h"
#include <QCoreApplication>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/SceneGraph>

namespace cnoid {

  void ObjectStatePublisherItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<ObjectStatePublisherItem>("ObjectStatePublisherItem");
  }

  ObjectStatePublisherItem::ObjectStatePublisherItem(){
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
  }

  void ObjectStatePublisherItem::setupROS() {
    if(this->setupROSDone_) return;
    this->setupROSDone_ = true;

    ros::NodeHandle nh;

    std::string topicName;
    if(this->odometryTopicName_!="") topicName = this->odometryTopicName_;
    else topicName = this->bodyName_+"/odom";
    this->odometryPub_ = nh.advertise<nav_msgs::Odometry>(topicName, 1);

    if(this->jointStateTopicName_!="") topicName = this->jointStateTopicName_;
    else topicName = this->bodyName_+"/joint_states";
    this->jointStatePub_ = nh.advertise<sensor_msgs::JointState>(topicName, 1);
  }

  void ObjectStatePublisherItem::onPositionChanged(){
    BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem){
      this->bodyName_ = ownerBodyItem->body()->name();
    }
    if(this->bodyName_ == ""){
      this->bodyName_ = this->name();
    }
  }

  bool ObjectStatePublisherItem::initialize(ControllerIO* io) {
    this->io_ = io;
    this->timeStep_ = io->worldTimeStep();

    setupROS(); // コンストラクタやcallLaterだとname()やrestore()が未完了

    return true;
  }

  bool ObjectStatePublisherItem::start() {
    if(!this->io_->body()) return false;

    if(this->frameId_ != "") this->odometry_.header.frame_id = this->frameId_;
    else this->odometry_.header.frame_id = "map";

    if(this->childFrameId_ != "") {
      this->odometry_.child_frame_id = this->childFrameId_;
    } else {
      cnoid::SgGroup* shape = this->io_->body()->rootLink()->shape();
      if(shape && shape->numChildObjects() > 0 && shape->child(0)->name().size()!=0){
        this->odometry_.child_frame_id = shape->child(0)->name();
      }else {
        this->odometry_.child_frame_id = this->io_->body()->rootLink()->name();
      }
    }

    for(int i=0;i<6;i++){
      for(int j=0;j<6;j++){
        if(i==j) this->odometry_.pose.covariance[i*6+j] = this->poseCovariance_;
        else this->odometry_.pose.covariance[i*6+j] = 0.0;
      }
    }
    for(int i=0;i<6;i++){
      for(int j=0;j<6;j++){
        if(i==j) this->odometry_.twist.covariance[i*6+j] = this->twistCovariance_;
        else this->odometry_.twist.covariance[i*6+j] = 0.0;
      }
    }

    this->jointState_.name.resize(this->io_->body()->numJoints());
    this->jointState_.position.resize(this->io_->body()->numJoints());
    this->jointState_.velocity.resize(this->io_->body()->numJoints());
    for(int i=0;i<this->io_->body()->numJoints();i++){
      this->jointState_.name[i] = this->io_->body()->joint(i)->name();
    }

    return true;
  }

  void ObjectStatePublisherItem::input() {
    if(!this->io_->body()) return;

    cnoid::Isometry3 pose = this->io_->body()->rootLink()->T();
    this->odometry_.pose.pose.position.x = pose.translation()[0];
    this->odometry_.pose.pose.position.y = pose.translation()[1];
    this->odometry_.pose.pose.position.z = pose.translation()[2];
    cnoid::Quaternion quat = cnoid::Quaternion(pose.linear());
    this->odometry_.pose.pose.orientation.x = quat.x();
    this->odometry_.pose.pose.orientation.y = quat.y();
    this->odometry_.pose.pose.orientation.z = quat.z();
    this->odometry_.pose.pose.orientation.w = quat.w();

    cnoid::Vector6 twist;
    twist.head<3>() = this->io_->body()->rootLink()->v();
    twist.tail<3>() = this->io_->body()->rootLink()->w();
    this->odometry_.twist.twist.linear.x = twist[0];
    this->odometry_.twist.twist.linear.y = twist[1];
    this->odometry_.twist.twist.linear.z = twist[2];
    this->odometry_.twist.twist.angular.x = twist[3];
    this->odometry_.twist.twist.angular.y = twist[4];
    this->odometry_.twist.twist.angular.z = twist[5];

    for(int i=0;i<this->io_->body()->numJoints();i++){
      this->jointState_.position[i] = this->io_->body()->joint(i)->q();
      this->jointState_.velocity[i] = this->io_->body()->joint(i)->dq();
    }
  }

  // The body oject given in the initalized function() must not be accessed
  // in this function. The access should be done in input() and output().
  bool ObjectStatePublisherItem::control() {
    if(!this->io_->body()) return false;

    if(this->timeStep_ < 1.0 / this->publishRate_){
      this->time_ += this->timeStep_;
      if(this->time_ < 1.0 / this->publishRate_) return true;
      this->time_ -= 1.0 / this->publishRate_;
    }

    this->odometry_.header.stamp.fromSec(this->io_->currentTime());
    this->odometryPub_.publish(this->odometry_);

    this->jointState_.header.stamp.fromSec(this->io_->currentTime());
    this->jointStatePub_.publish(this->jointState_);

    return true;
  }

  bool ObjectStatePublisherItem::store(Archive& archive) {
    archive.write("odometryTopicName", this->odometryTopicName_);
    archive.write("frameId", this->frameId_);
    archive.write("childFrameId", this->childFrameId_);
    archive.write("poseCovariance", this->poseCovariance_);
    archive.write("twistCovariance", this->twistCovariance_);
    archive.write("jointStateTopicName", this->jointStateTopicName_);
    archive.write("publishRate", this->publishRate_);
    return true;
  }

  bool ObjectStatePublisherItem::restore(const Archive& archive) {
    archive.read("odometryTopicName", this->odometryTopicName_);
    archive.read("frameId", this->frameId_);
    archive.read("childFrameId", this->childFrameId_);
    archive.read("poseCovariance", this->poseCovariance_);
    archive.read("twistCovariance", this->twistCovariance_);
    archive.read("jointStateTopicName", this->jointStateTopicName_);
    archive.read("publishRate", this->publishRate_);
    return true;
  }

}

