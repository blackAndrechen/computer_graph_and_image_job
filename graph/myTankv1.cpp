//键盘操控坦克
// Created by czpchen on 2020/6/8.
//
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/Node>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <iostream>
#include <math.h>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIActionAdapter>


/*  节点移动状态 */
class nodeMoveState{
public:
   nodeMoveState(){
       up=false;
       down=false;
       left=false;
       right=false;
   }
   bool up,down,left,right;
};

/* 节点位置朝向数据 */
class dataType : public osg::Referenced
{
public:
   dataType(osg::MatrixTransform *n,nodeMoveState *tids){
       pat=n;
       keyboard=tids;
       stride=0.5;
       pos.set(0.0,0.0,0.0);
       rotation=0.0;
   };
   void updatePosition();
   osg::Vec3d getPos(){return pos;};
   double getRotation(){return rotation;};
protected:
   osg::ref_ptr<osg::MatrixTransform> pat;
   double stride;
   nodeMoveState *keyboard;
   osg::Vec3d pos;
   double rotation;
};
void dataType::updatePosition(){
   if (keyboard->up) {
       pos.set(sin(-M_PI*rotation/180)*stride+pos.x(),pos.y()+cos(M_PI*rotation/180)*stride,pos.z());
       pat->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(rotation),0.0,0.0,1.0)*osg::Matrix::translate(pos.x(),pos.y(),pos.z()));
   }
   if (keyboard->down){
       pos.set(sin(+M_PI*rotation/180)*stride+pos.x(),pos.y()-cos(M_PI*rotation/180)*stride,pos.z());
       pat->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(rotation),0.0,0.0,1.0)*osg::Matrix::translate(pos.x(),pos.y(),pos.z()));
   }
   if(keyboard->left){
       rotation+=1;
       if(rotation>360) rotation-=360;
       pat->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(rotation),0.0,0.0,1.0)*osg::Matrix::translate(pos.x(),pos.y(),pos.z()));
   }
   if(keyboard->right){
       rotation-=1;
       if(rotation<0) rotation+=360;
       pat->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(rotation),0.0,0.0,1.0)*osg::Matrix::translate(pos.x(),pos.y(),pos.z()));
   }
}

/* 按键回调策略 */
class UserEventHandler : public osgGA::GUIEventHandler{
public:
   UserEventHandler(nodeMoveState *tids){
       nms=tids;
   }
   virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa) {
       osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
       if (!viewer) return false;
       switch (ea.getEventType()) {
           case osgGA::GUIEventAdapter::KEYDOWN: {
               if (ea.getKey() == 'w') {
                   nms->up=true;
               }
               if (ea.getKey() == 's') {
                   nms->down=true;
               }
               if (ea.getKey() == 'a') {
                   nms->left=true;
               }
               if (ea.getKey() == 'd') {
                   nms->right=true;
               }
               break;
           }
           case osgGA::GUIEventAdapter::KEYUP: {
               if (ea.getKey() == 'w') {
                   nms->up=false;
               }
               if (ea.getKey() == 's') {
                   nms->down=false;
               }
               if (ea.getKey() == 'a') {
                   nms->left=false;
               }
               if (ea.getKey() == 'd') {
                   nms->right=false;
               }
               break;
           }
           default:
               break;
       }
       return false;
   }
protected:
   nodeMoveState *nms;
};

/* 回调函数入口 */
class updateTankPosCallback:public osg::NodeCallback{
public:
   updateTankPosCallback(nodeMoveState *tids){
       nms=tids;
   }
   virtual void operator()(osg::Node* node,osg::NodeVisitor* nv)
   {
       osg::ref_ptr<dataType> tankData =dynamic_cast<dataType*> (node->getUserData() );
       tankData->updatePosition();
       traverse(node, nv);
   }
protected:
   nodeMoveState *nms;
};


int main(){

   osgViewer::Viewer viewer;
   osg::ref_ptr<osg::Group> root=new osg::Group();

   /*地形*/
   osg::ref_ptr<osg::Node> osgdirt=osgDB::readNodeFile("../NPS_Data/Models/JoeDirt/JoeDirt.flt");
   osg::ref_ptr<osg::MatrixTransform> dirt=new osg::MatrixTransform;
   dirt->setMatrix(osg::Matrix::translate(0.0,0.0,-20.0));
   dirt->addChild(osgdirt.get());

   /*坦克*/
   osg::ref_ptr<osg::Node> osgtank=osgDB::readNodeFile("../NPS_Data/Models/t72-tank/t72-tank_des.flt");
   osg::ref_ptr<osg::MatrixTransform> tank=new osg::MatrixTransform;
//    tank->setMatrix(osg::Matrix::translate(0.0,0.0,0.0));
   tank->addChild(osgtank.get());


   root->addChild(dirt.get());
   root->addChild(tank.get());


   nodeMoveState *tids=new nodeMoveState();
   dataType *transData=new dataType(tank,tids);
   tank->setUserData(transData);
   tank->setUpdateCallback(new updateTankPosCallback(tids));
   viewer.addEventHandler(new UserEventHandler(tids));

   viewer.setSceneData(root.get());
   viewer.realize();
   viewer.run();


   return 0;
}