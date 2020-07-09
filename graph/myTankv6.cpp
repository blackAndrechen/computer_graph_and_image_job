//新增子弹功能
//将datatype的一些状态移动到nodeMoveState类内
// Created by czpchen on 2020/6/15.
//
#include <iostream>
#include <math.h>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/Node>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
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
#include <osgParticle/RadialShooter>
#include <osgParticle/AccelOperator>
#include <osgParticle/FluidFrictionOperator>
#include <osgParticle/MultiSegmentPlacer>
#include <osg/ShapeDrawable>
#include <osgParticle/FireEffect>
#include <osgParticle/ExplosionEffect>
#include <osgParticle/ExplosionDebrisEffect>


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
void printVec3(osg::Vec3d pos){
   std::cout<<"position x:"<< pos.x() <<" y:"<<pos.y()<<" z:"<<pos.z()<<std::endl;

}

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
   if (tankElevationLocatorHits.size()>0) {
       heightTestResults = tankElevationLocatorHits.front();
       osg::Vec3d terrainHeight = heightTestResults.getWorldIntersectPoint();
       return terrainHeight[2];
   }
   else{
       return 9999.0;
   }
}

/*  节点移动状态 */
class nodeMoveState{
public:
   nodeMoveState(osg::ref_ptr<osg::MatrixTransform> dirt,osg::Vec3d pos){
       initPos.set(pos);
       myDirt=dirt;
       up=false;down=false;left=false;right=false;
       gunup=false;gundown=false;
       turretl=false;turretr=false;
       havePartcile=false;
       bullet=false,bulletswitch=false,boom=false;

       double z=getHight(initPos.x(),initPos.y(),myDirt);

       bulletBoomPos.set(0.0,0.0,0.0);
       gunHead.set(0.0,0.0,0.0);
       tankPos.set(initPos.x(),initPos.y(),z);
       tankRota.set(0.0,0.0,0.0);
       myPartcile=createParticle();
   }
   bool up,down,left,right,gunup,gundown,\
   turretl,turretr,havePartcile,bullet,bulletswitch,boom;
   osg::Vec3d initPos;
   osg::Vec3d bulletBoomPos;
   osg::Vec3d gunHead;
   osg::Vec3d tankPos;
   osg::Vec3d tankRota;//0整个车的旋转角度，1炮台的旋转角度。2炮的旋转角度
   osg::ref_ptr<osg::MatrixTransform> myDirt;
   osg::ref_ptr<osg::MatrixTransform> myPartcile;
};

/*bld-坦克*/
class bldTank{
public:
   bldTank(nodeMoveState *tids){
       nms=tids;
       trans=new osg::MatrixTransform;
       tanktrans=new osg::MatrixTransform;
       guntrans=new osg::MatrixTransform;
       turrettrans=new osg::MatrixTransform;
       bullettrans=new osg::MatrixTransform;

       osg::ref_ptr<osg::Node> tankbld=osgDB::readNodeFile("../NPS_Data/Models/tankbld.flt");
       osg::ref_ptr<osg::Node> gunbld=osgDB::readNodeFile("../NPS_Data/Models/gunbld.flt");
       osg::ref_ptr<osg::Node> turretbld=osgDB::readNodeFile("../NPS_Data/Models/turretbld.flt");

       osg::ref_ptr<osg::Geode> bullet=new osg::Geode();
       osg::ref_ptr<osg::Sphere> sphere=new osg::Sphere(osg::Vec3d(0.0,0.0,0.0),0.2f);
       osg::ref_ptr<osg::ShapeDrawable> unit=new osg::ShapeDrawable(sphere);
       bullet->addChild(unit);

       tanktrans->addChild(tankbld.get());
       guntrans->addChild(gunbld.get());
       turrettrans->addChild(turretbld.get());
       bullettrans->addChild(bullet);

       /*初始化坦克位置*/
       tanktrans->setMatrix(osg::Matrix::translate(nms->tankPos));
       guntrans->setMatrix(osg::Matrix::translate(nms->tankPos));
       turrettrans->setMatrix(osg::Matrix::translate(nms->tankPos));
       bullettrans->setMatrix(osg::Matrix::translate(0,2.28,0.04)*osg::Matrix::translate(0.0+nms->tankPos.x(),1.22+nms->tankPos.y(),2.59+nms->tankPos.z()));

       trans->addChild(tanktrans);
       trans->addChild(guntrans);
       trans->addChild(turrettrans);
   }
   osg::ref_ptr<osg::MatrixTransform> trans;
   osg::ref_ptr<osg::MatrixTransform> tanktrans;
   osg::ref_ptr<osg::MatrixTransform> guntrans;
   osg::ref_ptr<osg::MatrixTransform> turrettrans;
   osg::ref_ptr<osg::MatrixTransform> bullettrans;
   nodeMoveState *nms;
};

/* 节点位置数据 */
class dataType : public osg::Referenced
{
public:
   dataType(bldTank *n){
       nms=n->nms;
       myTank=n;

       osg::Matrix mat_tmp=osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos);
       nms->gunHead.set(mat_tmp.getTrans());
       nms->bulletBoomPos.set(9999.0,9999.0,9999.0);

   };
   void updatePosition(){
       if(nms->bulletswitch && !nms->bullet){//一次只允许存在一枚炮弹，按下‘j’后发射一枚炮弹，设置初始位置为炮口处
           nms->bullet=true;
           myTank->trans->addChild(myTank->bullettrans);
           //init position
           myTank->bullettrans->setMatrix(osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));
           //init speed
           bulletV.set(cos(M_PI*nms->tankRota[2]/180)*cos(M_PI*(nms->tankRota[0]+90+nms->tankRota[1])/180),\
           cos(M_PI*nms->tankRota[2]/180)*sin(M_PI*(nms->tankRota[0]+90+nms->tankRota[1])/180),\
           sin(M_PI*nms->tankRota[2]/180));
       }
       else if(nms->bullet){//如果有的话，更新炮弹位置，方向根据炮塔+坦身的角度，炮身的仰角
           double scale=4;//炸弹运行的步长
           double gravity=0.02;//模拟重力，用来更新z轴速度

           osg::Vec3d curPos=myTank->bullettrans->getMatrix().getTrans();
           bulletV.set(bulletV.x(),bulletV.y(),bulletV.z()-gravity);
           curPos.set(curPos.x()+scale*bulletV.x(),curPos.y()+scale*bulletV.y(),curPos.z()+scale*bulletV.z());
           myTank->bullettrans->setMatrix(osg::Matrix::translate(curPos));
           double z=getHight(curPos.x(),curPos.y(),nms->myDirt);
           if (curPos.z()<z){//炸弹落地

               nms->bullet=false;
               nms->boom=true;
               myTank->trans->removeChild(myTank->bullettrans);
               nms->bulletBoomPos.set(curPos.x(),curPos.y(),z);
           }
       }

       if(nms->up){
           double x=nms->tankPos.x()-sin(M_PI*nms->tankRota[0]/180);
           double y=nms->tankPos.y()+cos(M_PI*nms->tankRota[0]/180);
           double z=getHight(x,y,nms->myDirt);
           if(z>9990.0) return;
           printVec3(nms->tankPos);
           nms->tankPos.set(nms->tankPos);
           nms->tankPos.set(x,y,z);

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           osg::Matrix mat_tmp=osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos);
           nms->gunHead.set(mat_tmp.getTrans());
       }
       else if(nms->down){
           double x=nms->tankPos.x()+sin(M_PI*nms->tankRota[0]/180);
           double y=nms->tankPos.y()-cos(M_PI*nms->tankRota[0]/180);
           double z=getHight(x,y,nms->myDirt);
           if(z==9990.0) return;
           nms->tankPos.set(nms->tankPos);
           nms->tankPos.set(x,y,z);

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           osg::Matrix mat_tmp=osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos);
           nms->gunHead.set(mat_tmp.getTrans());
       }

       if(nms->left){
           nms->tankRota[0]+=1;
           if(nms->tankRota[0]>360) nms->tankRota[0]-=360;

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           osg::Matrix mat_tmp=osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos);
           nms->gunHead.set(mat_tmp.getTrans());
       }
       else if(nms->right){
           nms->tankRota[0]-=1;
           if(nms->tankRota[0]<0) nms->tankRota[0]+=360;

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           osg::Matrix mat_tmp=osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos);
           nms->gunHead.set(mat_tmp.getTrans());
       }

       if(nms->gunup && nms->tankRota[2]<45){
           nms->tankRota[2]+=1;
           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           osg::Matrix mat_tmp=osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos);
           nms->gunHead.set(mat_tmp.getTrans());
       }
       else if(nms->gundown && nms->tankRota[2]>-10){
           nms->tankRota[2]-=1;
           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           osg::Matrix mat_tmp=osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos);
           nms->gunHead.set(mat_tmp.getTrans());
       }

       if(nms->turretl){
           nms->tankRota[1]+=1;
           if(nms->tankRota[1]>360) nms->tankRota[1]-=360;

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           osg::Matrix mat_tmp=osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos);
           nms->gunHead.set(mat_tmp.getTrans());
       }
       else if(nms->turretr){
           nms->tankRota[1]-=1;
           if(nms->tankRota[1]<0) nms->tankRota[1]+=360;

           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           osg::Matrix mat_tmp=osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos);
           nms->gunHead.set(mat_tmp.getTrans());
       }
   };

public:
   nodeMoveState *nms;
   bldTank *myTank;
   osg::Vec3d bulletV;//炮弹的速度，0为x,1为y,2为z
};

/* 按键回调策略 */
class UserEventHandler : public osgGA::GUIEventHandler{
public:
   UserEventHandler(nodeMoveState *tids){
       nms=tids;
   }
   void createGunFire(osg::Group* root){
       osg::Vec3d position=nms->gunHead;
       osg::Vec3d wind(0,0,-1.2);
       position.set(position);

       osg::ref_ptr<osg::MatrixTransform> particleTrans=new osg::MatrixTransform;
       double scale = 0.5;
       double dur=0.1;
       double inten=10;

       osgParticle::FireEffect *fire = new osgParticle::FireEffect(position, scale);
       fire->setIntensity(inten);
       fire->setEmitterDuration(dur);
       fire->setWind(wind);

       particleTrans->addChild(fire);
       root->addChild(particleTrans);

   }
   void createBoom(osg::Group* root){
       osg::Vec3d position=nms->bulletBoomPos;

       osg::ref_ptr<osg::MatrixTransform> particleTrans=new osg::MatrixTransform;
       osg::Vec3 wind(1.0f, 0.0f, 0.0f);
       double scale = 6.0;

       osgParticle::ExplosionEffect *explosion = new osgParticle::ExplosionEffect(position, scale);
       osgParticle::ExplosionDebrisEffect *explosionDebri = new osgParticle::ExplosionDebrisEffect(position, scale);
       osgParticle::FireEffect *fire = new osgParticle::FireEffect(position, scale);

       explosion->setWind(wind);
       explosionDebri->setWind(wind);
       fire->setWind(wind);

       particleTrans->addChild(explosion);
       particleTrans->addChild(explosionDebri);
       particleTrans->addChild(fire);
       root->addChild(particleTrans);

       nms->bulletBoomPos.set(1.0,1.0,1.0);
   }
   virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa) {
       osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
       osg::Group* root = dynamic_cast<osg::Group*>(viewer->getSceneData());
       if (!root) return false;
       if(nms->boom==true){
           nms->boom=false;
           createBoom(root);
       }
       if(nms->up or nms->down){
           nms->myPartcile->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos+osg::Vec3d(3*sin(M_PI*nms->tankRota[0]/180),-3*cos(M_PI*nms->tankRota[0]/180),1)));
           if(!nms->havePartcile){
               root->addChild(nms->myPartcile);
               nms->havePartcile=true;
           }
       }else if(nms->havePartcile){
           root->removeChild(nms->myPartcile);
           nms->havePartcile=false;
       }
       switch (ea.getEventType()) {
           case osgGA::GUIEventAdapter::KEYDOWN: {
               if (ea.getKey() == 'w') {
                   nms->up=true;
               }
               else if (ea.getKey() == 's') {
                   nms->down=true;
               }
               if (ea.getKey() == 'a') {
                   nms->left=true;
               }
               else if (ea.getKey() == 'd') {
                   nms->right=true;
               }
               if (ea.getKey() == 'z') {
                   nms->gunup=true;
               }
               else if (ea.getKey() == 'x') {
                   nms->gundown=true;
               }
               else if (ea.getKey() == 'c') {
                   nms->turretl=true;
               }
               else if (ea.getKey() == 'v') {
                   nms->turretr=true;
               }
               if (ea.getKey() == 'j') {
                   if(nms->bullet==false){
                       createGunFire(root);
                   }
                   nms->bulletswitch=true;
               }
               break;
           }
           case osgGA::GUIEventAdapter::KEYUP: {
               if (ea.getKey() == 'w') {
                   nms->up=false;
               }
               else if (ea.getKey() == 's') {
                   nms->down=false;
               }
               if (ea.getKey() == 'a') {
                   nms->left=false;
               }
               else if (ea.getKey() == 'd') {
                   nms->right=false;
               }
               if (ea.getKey() == 'z') {
                   nms->gunup=false;
               }
               else if (ea.getKey() == 'x') {
                   nms->gundown=false;
               }
               if (ea.getKey() == 'c') {
                   nms->turretl=false;
               }
               else if (ea.getKey() == 'v') {
                   nms->turretr=false;
               }
               if (ea.getKey() == 'j') {
                   nms->bulletswitch=false;
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
   Follow(osgViewer::Viewer *viewerParam,nodeMoveState *tids){
       nms=tids;
       m_vPosition = osg::Vec3(0.0, -50.0, 10.5+nms->tankPos[2]);
       m_vRotation = osg::Vec3(85.0, 0.0f, 0.0f);
       viewer = viewerParam;
   }
   virtual void setByMatrix(){}
   virtual void setByInverseMatrix(){}
   virtual osg::Matrixd getMatrix() const{
       osg::Matrixd mat1;
       mat1=osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[0]),1.0,0.0,0.0)*osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[2]+nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[1]),0.0,1.0,0.0);
       return mat1*osg::Matrix::translate(nms->tankPos)*osg::Matrixd::translate(m_vPosition.x()-m_vPosition.y()*sin(M_PI*(nms->tankRota[0]+nms->tankRota[1])/180),m_vPosition.y()*cos(M_PI*(nms->tankRota[0]+nms->tankRota[1])/180),m_vPosition.z());
   }
   virtual osg::Matrixd getInverseMatrix() const{
       osg::Matrixd mat1;
       mat1=osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[0]),1.0,0.0,0.0)*osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[2]+nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*osg::Matrix::rotate(osg::DegreesToRadians(m_vRotation[1]),0.0,1.0,0.0);
       return osg::Matrixd::inverse(mat1*osg::Matrix::translate(nms->tankPos)*osg::Matrixd::translate(m_vPosition.x()-m_vPosition.y()*sin(M_PI*(nms->tankRota[0]+nms->tankRota[1])/180),m_vPosition.y()*cos(M_PI*(nms->tankRota[0]+nms->tankRota[1])/180),m_vPosition.z()));
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
   nodeMoveState* nms;
};



int main(){

   osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
   osg::ref_ptr<osg::Group> root=new osg::Group();

   /*地形*/
   osg::ref_ptr<osg::Node> osgdirt=osgDB::readNodeFile("../NPS_Data/Models/terrain.ive");
   osg::ref_ptr<osg::MatrixTransform> dirt=new osg::MatrixTransform;
   dirt->addChild(osgdirt.get());
   root->addChild(dirt.get());

   /*初始化坦克各种状态*/
   nodeMoveState *tids=new nodeMoveState(dirt,osg::Vec3d(277.761,-1826.73,0));
   bldTank* mytank=new bldTank(tids);
   root->addChild(mytank->trans.get());

   /*按键响应及回调策略*/
   dataType *transData=new dataType(mytank);
   mytank->trans->setUserData(transData);
   mytank->trans->setUpdateCallback(new updateTankPosCallback(tids));
   viewer->addEventHandler(new UserEventHandler(tids));

   /*相机跟随*/
   viewer->setCameraManipulator(new Follow(viewer,tids));

   viewer->setSceneData(root.get());
   viewer->realize();
   viewer->run();
   return 0;
}

