ClockPublisherItem: /clockをpublish. choreonoid1.8がreleaseされたらhttps://github.com/choreonoid/choreonoid_ros/blob/master/src/plugin/WorldROSItem.h を使いたい

CameraPublisherItem: camera画像をpublish. https://github.com/choreonoid/choreonoid_ros/blob/master/src/plugin/BodyROSItem.h との差異は、欲しいcamera画像だけをpublishする点と、camera_infoもpublishする点.

DepthCameraPublisherItem: depth_cameraのimage,depth,pointcloudをpublish. https://github.com/choreonoid/choreonoid_ros/blob/master/src/plugin/BodyROSItem.h との差異は、欲しいcameraだけをpublishする点と、camera_infoとdepth画像もpublishする点.

CraneItem: ロボットを吊るすクレーン. <name>/Liftにstd_srvsサービスを送るとクレーンをオン, オフできる.

SpringDamplerController: 関節をSpringDamper化するSimplerController