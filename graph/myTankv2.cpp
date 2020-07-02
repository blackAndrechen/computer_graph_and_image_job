//新增旋转炮台
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

double getTheta(double x,double y,double z,double w){
   double theta=atan2(2*(z*y+w*x),1-2*(z*z+w*w));
   return theta;
}
void printMatrix(osg::Matrix mat){
   osg::Vec3 pos=mat.getTrans();
   osg::Vec4 rota=mat.getRotate().asVec4();

   std::cout<<"position x:"<< pos[0] <<" y:"<<pos[1]<<" z:"<<pos[2]<<"    ";
   std::cout<<"rotation :"<<getTheta(rota[1],rota[2],rota[3],rota[0])<<std::endl;
}

/*bld-坦克*/
class bldTank{
public:
   bldTank(){
       trans=new osg::MatrixTransform;
       tanktrans=new osg::MatrixTransform;
       guntrans=new osg::MatrixTransform;
       turrettrans=new osg::MatrixTransform;

       osg::ref_ptr<osg::Node> tankbld=osgDB::readNodeFile("../NPS_Data/Models/tankbld.flt");
       osg::ref_ptr<osg::Node> gunbld=osgDB::readNodeFile("../NPS_Data/Models/gunbld.flt");
       osg::ref_ptr<osg::Node> turretbld=osgDB::readNodeFile("../NPS_Data/Models/turretbld.flt");


       tanktrans->addChild(tankbld.get());
       guntrans->addChild(gunbld.get());
       turrettrans->addChild(turretbld.get());

       trans->addChild(tanktrans);
       trans->addChild(guntrans);
       trans->addChild(turrettrans);
   }
   osg::ref_ptr<osg::MatrixTransform> trans;
   osg::ref_ptr<osg::MatrixTransform> tanktrans;
   osg::ref_ptr<osg::MatrixTransform> guntrans;
   osg::ref_ptr<osg::MatrixTransform> turrettrans;
};

/*  节点移动状态 */
class nodeMoveState{
public:
   nodeMoveState(){
       up=false;down=false;left=false;right=false;
       gunup=false;gundown=false;
       turretl=false;turretr=false;
   }
   bool up,down,left,right,gunup,gundown,turretl,turretr;
};

/* 节点位置数据 */
class dataType : public osg::Referenced
{
public:
   dataType(bldTank *n,nodeMoveState *tids){
       keyboard=tids;
       myTank=n;
       pos.set(0.0,0.0,0.0);
       tankrota.set(0.0,0.0,0.0);
   };
   void updatePosition(){
       if(keyboard->up){
           pos.set(pos.x()-sin(M_PI*tankrota[0]/180),pos.y()+cos(M_PI*tankrota[0]/180),pos.z());

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));
       }
       if(keyboard->down){
           pos.set(pos.x()+sin(M_PI*tankrota[0]/180),pos.y()-cos(M_PI*tankrota[0]/180),pos.z());

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));
       }
       if(keyboard->left){
           tankrota[0]+=1;
           if(tankrota[0]>360) tankrota[0]-=360;

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));
       }
       if(keyboard->right){
           tankrota[0]-=1;
           if(tankrota[0]<0) tankrota[0]+=360;

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));
       }
       if(keyboard->gunup && tankrota[2]<45){
           tankrota[2]+=1;
           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));
       }
       if(keyboard->gundown && tankrota[2]>-10){
           tankrota[2]-=1;
           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));
       }
       if(keyboard->turretl){
           tankrota[1]+=1;
           if(tankrota[1]>360) tankrota[1]-=360;

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));
       }
       if(keyboard->turretr){
           tankrota[1]-=1;
           if(tankrota[1]<0) tankrota[1]+=360;

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));
       }
   };

protected:
   nodeMoveState *keyboard;
   bldTank *myTank;
   osg::Vec3d pos;
   osg::Vec3d tankrota;//0整个车的旋转角度，1炮台的旋转角度。2炮的旋转角度
};

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
               if (ea.getKey() == 'z') {
                   nms->gunup=true;
               }
               if (ea.getKey() == 'x') {
                   nms->gundown=true;
               }
               if (ea.getKey() == 'c') {
                   nms->turretl=true;
               }
               if (ea.getKey() == 'v') {
                   nms->turretr=true;
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
               if (ea.getKey() == 'z') {
                   nms->gunup=false;
               }
               if (ea.getKey() == 'x') {
                   nms->gundown=false;
               }
               if (ea.getKey() == 'c') {
                   nms->turretl=false;
               }
               if (ea.getKey() == 'v') {
                   nms->turretr=false;
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

//    /*地形*/
//    osg::ref_ptr<osg::Node> osgdirt=osgDB::readNodeFile("../NPS_Data/Models/JoeDirt/JoeDirt.flt");
//    osg::ref_ptr<osg::MatrixTransform> dirt=new osg::MatrixTransform;
//    dirt->setMatrix(osg::Matrix::translate(0.0,0.0,-20.0));
//    dirt->addChild(osgdirt.get());
//
   /*坦克*/
   bldTank* mytank=new bldTank();
   root->addChild(mytank->trans.get());


//    printMatrix(tank->getMatrix());
//    tank->setMatrix(tank->getMatrix()*osg::Matrix::rotate(osg::DegreesToRadians(180.0),0.0,0.0,1.0));
//    printMatrix(tank->getMatrix());


   osg::ref_ptr<osg::Node> osgtank_tmp=osgDB::readNodeFile("glider.osg");
   osg::ref_ptr<osg::MatrixTransform> tank_tmp=new osg::MatrixTransform;
   tank_tmp->addChild(osgtank_tmp.get());
   root->addChild(tank_tmp.get());

   nodeMoveState *tids=new nodeMoveState();
   dataType *transData=new dataType(mytank,tids);
   mytank->trans->setUserData(transData);
   mytank->trans->setUpdateCallback(new updateTankPosCallback(tids));
   viewer.addEventHandler(new UserEventHandler(tids));

   viewer.setSceneData(root.get());
   viewer.realize();
   viewer.run();


   return 0;
}