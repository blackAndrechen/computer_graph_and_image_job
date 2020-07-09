//新增扬尘
// Created by czpchen on 2020/6/13.
//
#include <limits>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/Node>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <iostream>
#include <math.h>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIActionAdapter>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgUtil/IntersectVisitor>
#include <osg/Texture2D>
#include <osg/BlendFunc>
#include <osgParticle/Particle>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/ModularEmitter>
#include <osgParticle/ModularProgram>
#include <osgParticle/RandomRateCounter>
#include <osgParticle/SectorPlacer>
#include <osgParticle/RadialShooter>
#include <osgParticle/AccelOperator>
#include <osgParticle/FluidFrictionOperator>
#include <osgParticle/MultiSegmentPlacer>


double MAXDOUBLE=(std::numeric_limits<double>::max)();

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
double getHight(double x,double y,osg::ref_ptr<osg::MatrixTransform> dirt){
   osg::LineSegment* tankLocationSegment = new osg::LineSegment();
   tankLocationSegment->set(
           osg::Vec3(x, y, 999) ,
           osg::Vec3(x, y, -999) );
   osgUtil::IntersectVisitor findTankElevationVisitor;
   findTankElevationVisitor.addLineSegment(tankLocationSegment);
   dirt->accept(findTankElevationVisitor);
   osgUtil::IntersectVisitor::HitList tankElevationLocatorHits;
   tankElevationLocatorHits=findTankElevationVisitor.getHitList(tankLocationSegment);
   osgUtil::Hit heightTestResults;
   if(tankElevationLocatorHits.size()>0) {
       heightTestResults = tankElevationLocatorHits.front();
       osg::Vec3d terrainHeight = heightTestResults.getWorldIntersectPoint();
       return terrainHeight[2];
   }
   else{
       return MAXDOUBLE;
   }
}

/*bld-坦克*/
class bldTank{
public:
   bldTank(osg::ref_ptr<osg::MatrixTransform> dirt){
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

       /*初始化坦克位置*/
       double z=getHight(0,0,dirt);
       tanktrans->setMatrix(osg::Matrix::translate(osg::Vec3d(0.0,0.0,z)));
       guntrans->setMatrix(osg::Matrix::translate(osg::Vec3d(0.0,0.0,z)));
       turrettrans->setMatrix(osg::Matrix::translate(osg::Vec3d(0.0,0.0,z)));

       trans->addChild(tanktrans);
       trans->addChild(guntrans);
       trans->addChild(turrettrans);
   }
   osg::ref_ptr<osg::MatrixTransform> trans;
   osg::ref_ptr<osg::MatrixTransform> tanktrans;
   osg::ref_ptr<osg::MatrixTransform> guntrans;
   osg::ref_ptr<osg::MatrixTransform> turrettrans;
};

/*坦克移动产生的扬尘*/
osg::ref_ptr<osg::MatrixTransform> createParticle(){
   osg::ref_ptr<osg::MatrixTransform> partcileTrans=new osg::MatrixTransform;
   osgParticle::Particle ptemplate;
   ptemplate.setLifeTime(2);
   ptemplate.setSizeRange(osgParticle::rangef(0.01f, 1.0f));
   ptemplate.setAlphaRange(osgParticle::rangef(0.0f, 1.0f));
   ptemplate.setColorRange(osgParticle::rangev4(osg::Vec4(189/255.0,183/255.0,107/255.0,1), osg::Vec4(189/255.0,183/255.0,107/255.0,0.5)));
   ptemplate.setRadius(0.05f);
   ptemplate.setMass(0.05f);

   osg::ref_ptr<osgParticle::ParticleSystem> ps = new osgParticle::ParticleSystem();
   ps->setDefaultAttributes("Images/smoke.rgb", false, false);
   ps->setDefaultParticleTemplate(ptemplate);
   osg::ref_ptr<osgParticle::ModularEmitter> emitter = new osgParticle::ModularEmitter();
   emitter->setParticleSystem(ps.get());
   osg::ref_ptr<osgParticle::RandomRateCounter> counter = new osgParticle::RandomRateCounter();
   counter->setRateRange(5.0f, 5.0f);
   emitter->setCounter(counter.get());
   osg::ref_ptr<osgParticle::MultiSegmentPlacer> placer = new osgParticle::MultiSegmentPlacer();

   placer->addVertex(-1.5,-1.5,0);
   placer->addVertex(1.5,-1.5,0);
   emitter->setPlacer(placer.get());
   osg::ref_ptr<osgParticle::RadialShooter> shooter = new osgParticle::RadialShooter();
   shooter->setInitialSpeedRange(0, 3);
   shooter->setThetaRange(-M_PI/2,-M_PI/2);
   shooter->setPhiRange(M_PI/2,M_PI/2);
   emitter->setShooter(shooter.get());
   partcileTrans->addChild(emitter.get());

   osg::ref_ptr<osgParticle::ModularProgram> program = new osgParticle::ModularProgram();
   program->setParticleSystem(ps.get());
   osg::ref_ptr<osgParticle::AccelOperator> ap = new osgParticle::AccelOperator();
   ap->setToGravity(0.0);
   program->addOperator(ap.get());
   osg::ref_ptr<osgParticle::FluidFrictionOperator> ffo = new osgParticle::FluidFrictionOperator();
   ffo->setFluidToAir();
   program->addOperator(ffo.get());
   partcileTrans->addChild(program.get());
   osg::ref_ptr<osgParticle::ParticleSystemUpdater> psu = new osgParticle::ParticleSystemUpdater();
   psu->addParticleSystem(ps.get());

   osg::ref_ptr<osg::Geode> geode = new osg::Geode;
   geode->addDrawable(ps.get());

   partcileTrans->addChild(geode.get());
   partcileTrans->addChild(psu.get());
   return partcileTrans;
}

/*  节点移动状态 */
class nodeMoveState{
public:
   nodeMoveState(){
       up=false;down=false;left=false;right=false;
       gunup=false;gundown=false;
       turretl=false;turretr=false;
       havePartcile=true;
   }
   bool up,down,left,right,gunup,gundown,turretl,turretr,havePartcile;
};

/* 节点位置数据 */
class dataType : public osg::Referenced
{
public:
   dataType(osg::ref_ptr<osg::Group> root,bldTank *n,nodeMoveState *tids,osg::ref_ptr<osg::MatrixTransform> terrain,osg::ref_ptr<osg::MatrixTransform> partcile){
       myRoot=root;
       myPartcile=partcile;
       keyboard=tids;
       myTank=n;
       dirt=terrain;
       pos.set(0.0,0.0,getHight(0,0,dirt));
       tankrota.set(0.0,0.0,0.0);
   };
   void updatePosition(){
       if(keyboard->up){
           double x=pos.x()-sin(M_PI*tankrota[0]/180);
           double y=pos.y()+cos(M_PI*tankrota[0]/180);
           double z=getHight(x,y,dirt);
           if(z==MAXDOUBLE) return;
           pos.set(x,y,z);

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myPartcile->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos+osg::Vec3d(0,0,1)));
           if(!keyboard->havePartcile) {
               myRoot->addChild(myPartcile);
               keyboard->havePartcile=true;
           }
       }
       if(keyboard->down){
           double x=pos.x()+sin(M_PI*tankrota[0]/180);
           double y=pos.y()-cos(M_PI*tankrota[0]/180);
           double z=getHight(x,y,dirt);
           if(z==MAXDOUBLE) return;
           pos.set(x,y,z);

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]+tankrota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos));

           myPartcile->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(tankrota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(pos+osg::Vec3d(0,0,1)));
           if(!keyboard->havePartcile){
               myRoot->addChild(myPartcile);
               keyboard->havePartcile=true;
           }
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
       if(!keyboard->up && !keyboard->down && keyboard->havePartcile){
           myRoot->removeChild(myPartcile);
           keyboard->havePartcile=false;
       }
   };

public:
   osg::ref_ptr<osg::Group> myRoot;
   osg::ref_ptr<osg::MatrixTransform> myPartcile;
   nodeMoveState *keyboard;
   bldTank *myTank;
   osg::Vec3d pos;
   osg::ref_ptr<osg::MatrixTransform> dirt;
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


/*相机*/
class Follow:public osgGA::KeySwitchMatrixManipulator
{
public:
   Follow(osgViewer::Viewer *viewerParam,dataType *tankData){
       transData=tankData;
       m_vPosition = osg::Vec3(0.0, -100.0, 13.5+transData->pos[2]);
       m_vRotation = osg::Vec3(80.0, 0.0f, 0.0f);
       viewer = viewerParam;
   }
   virtual void setByMatrix(){}
   virtual void setByInverseMatrix(){}
   virtual osg::Matrixd getMatrix() const{
       osg::Matrixd mat1;
       mat1=osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[0]),1.0,0.0,0.0)*osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[2]+transData->tankrota[0]+transData->tankrota[1]),0.0,0.0,1.0)*osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[1]),0.0,1.0,0.0);
       return mat1*osg::Matrix::translate(transData->pos)*osg::Matrixd::translate(m_vPosition.x()-m_vPosition.y()*sin(M_PI*(transData->tankrota[0]+transData->tankrota[1])/180),m_vPosition.y()*cos(M_PI*(transData->tankrota[0]+transData->tankrota[1])/180),m_vPosition.z());
   }
   virtual osg::Matrixd getInverseMatrix() const{
       osg::Matrixd mat1;
       mat1=osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[0]),1.0,0.0,0.0)*osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[2]+transData->tankrota[0]+transData->tankrota[1]),0.0,0.0,1.0)*osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[1]),0.0,1.0,0.0);
       return osg::Matrixd::inverse(mat1*osg::Matrix::translate(transData->pos)*osg::Matrixd::translate(m_vPosition.x()-m_vPosition.y()*sin(M_PI*(transData->tankrota[0]+transData->tankrota[1])/180),m_vPosition.y()*cos(M_PI*(transData->tankrota[0]+transData->tankrota[1])/180),m_vPosition.z()));
   }
   virtual float getFusionDistanceValue() const {
       return viewer->getFusionDistanceValue();
   }
   virtual osgUtil::SceneView::FusionDistanceMode getFusionDistanceMode() const{
       return viewer->getFusionDistanceMode();
   }
   bool handle(const osgGA::GUIEventAdapter& gea,osgGA::GUIActionAdapter& gaa){
       return false;
   }
private:
   osg::Vec3 m_vPosition;
   osg::Vec3 m_vRotation;
   osgUtil::SceneView::FusionDistanceMode model1;
   osgViewer::Viewer *viewer;
   dataType* transData;
};

int main(){

   osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
   osg::ref_ptr<osg::Group> root=new osg::Group();

   /*地形*/
   osg::ref_ptr<osg::Node> osgdirt=osgDB::readNodeFile("../NPS_Data/Models/terrain.ive");
   osg::ref_ptr<osg::MatrixTransform> dirt=new osg::MatrixTransform;
   dirt->setMatrix(osg::Matrix::translate(0.0,0.0,0.0));
   dirt->addChild(osgdirt.get());
   root->addChild(dirt.get());

   /*坦克*/
   bldTank* mytank=new bldTank(dirt);
   root->addChild(mytank->trans.get());

   osg::ref_ptr<osg::Node> osgtank_tmp=osgDB::readNodeFile("glider.osg");
   osg::ref_ptr<osg::MatrixTransform> tank_tmp=new osg::MatrixTransform;
   tank_tmp->addChild(osgtank_tmp.get());
   root->addChild(tank_tmp.get());

   /*扬尘*/
   osg::ref_ptr<osg::MatrixTransform> mypartcile=createParticle();
   root->addChild(mypartcile);


   nodeMoveState *tids=new nodeMoveState();
   dataType *transData=new dataType(root,mytank,tids,dirt,mypartcile);
   mytank->trans->setUserData(transData);
   mytank->trans->setUpdateCallback(new updateTankPosCallback(tids));
   viewer->addEventHandler(new UserEventHandler(tids));

   viewer->setCameraManipulator(new Follow(viewer,transData));

   viewer->setSceneData(root.get());
   viewer->realize();
   viewer->run();


   return 0;
}

