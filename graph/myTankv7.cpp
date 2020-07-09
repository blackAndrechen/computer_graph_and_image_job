//新增敌军
// Created by czpchen on 2020/6/17.
//
#include <vector>
#include <random>
#include <ctime>
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


void printMatrix(osg::Matrix mat){
   osg::Vec3d pos=mat.getTrans();
   osg::Vec4d rota=mat.getRotate().asVec4();

   std::cout<<"position x:"<< pos[0] <<" y:"<<pos[1]<<" z:"<<pos[2]<<"    ";
   std::cout<<"rotation x:"<<rota[0]<<" y:"<<rota[1] <<" z:"<<rota[2]<<" w:"<<rota[2]<<std::endl;
}
void printVec3(osg::Vec3d pos){
   std::cout<<"position x:"<< pos.x() <<" y:"<<pos.y()<<" z:"<<pos.z()<<std::endl;

}
double getDistance(osg::Vec3d pos1,osg::Vec3d pos2){
   return sqrt(pow(pos1[0]-pos2[0],2)+pow(pos1[1]-pos2[1],2)+pow(pos1[2]-pos2[2],2));
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

/*获取指定坐标的地面高度*/
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
       up=false;down=false;left=false;right=false;//上下左右
       gunup=false;gundown=false;//炮台以及大炮
       turretl=false;turretr=false;
       havePartcile=false;//是否产生扬尘
       bullet=false,bulletswitch=false,boom=false;//是否存在炸弹，炸弹开关，是否爆炸

       double z=getHight(initPos.x(),initPos.y(),myDirt);

       bulletBoomPos.set(0.0,0.0,0.0);
       gunHead.set(0.0,0.0,0.0);
       tankPos.set(initPos.x(),initPos.y(),z);
       tankRota.set(0.0,0.0,0.0);
       myPartcile=createParticle();
   }
   bool up,down,left,right,gunup,gundown,\
   turretl,turretr,havePartcile,bullet,bulletswitch,boom;
   osg::Vec3d initPos;//初始位置
   osg::Vec3d bulletBoomPos;//子弹触发爆炸的位置
   osg::Vec3d gunHead;//枪口位置
   osg::Vec3d tankPos;//坦克的当前位置
   osg::Vec3d tankRota;//0整个车的旋转角度，1炮台的旋转角度。2炮的旋转角度
   osg::ref_ptr<osg::MatrixTransform> myDirt;//地面
   osg::ref_ptr<osg::MatrixTransform> myPartcile;//扬尘
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
   osg::ref_ptr<osg::MatrixTransform> trans;//总容器
   osg::ref_ptr<osg::MatrixTransform> tanktrans;//坦克主体容器
   osg::ref_ptr<osg::MatrixTransform> guntrans;//坦克炮体
   osg::ref_ptr<osg::MatrixTransform> turrettrans;//坦克的炮台
   osg::ref_ptr<osg::MatrixTransform> bullettrans;//炸弹，一次只允许有一个炸弹发射
   nodeMoveState *nms;
};

/*敌人*/
class Enemy{
public:
   //坦克的初始位置，敌人数量，敌人出现的区域，宽和长
   Enemy(osg::Vec3d mypos,int enemyNums,int width,int depth,osg::ref_ptr<osg::Group> root,osg::ref_ptr<osg::MatrixTransform> dirt){
       myDirt=dirt;
       myRoot=root;
       t80=osgDB::readNodeFile("../NPS_Data/Models/t80.flt");
       t72=osgDB::readNodeFile("../NPS_Data/Models/t72-tank/t72-tank_des.flt");
       machinegun=osgDB::readNodeFile("../NPS_Data/Models/machinegun.flt");
       m1=new osg::MatrixTransform;
       m1->addChild(osgDB::readNodeFile("../NPS_Data/Models/tankm1.flt"));
       m1->addChild(osgDB::readNodeFile("../NPS_Data/Models/turretm1.flt"));
       m1->addChild(osgDB::readNodeFile("../NPS_Data/Models/gunm1.flt"));

       generateRandomEnemy(mypos,enemyNums,width,depth);
   }
public:
   std::vector<osg::ref_ptr<osg::MatrixTransform> > staticEnemyTrans;//存放静止的敌人
   std::vector<osg::Vec3d > staticEnemyPos;//静止敌人的位置
   std::vector<osg::ref_ptr<osg::MatrixTransform> > moveEnemyTrans;//存放移动的敌人
   std::vector<osg::Vec3d > moveEnemyPos;//移动敌人的当前位置
   std::vector<osg::Vec3d > moveEnemyInitPos;//移动敌人的初始位置，记录的目的是为了计算敌人的偏移量，偏离一定距离后掉头
   std::vector<double > moveEnemyRota;//移动敌人的前进方向
   bool isMoveLive[100];//记录移动敌人是否活着
   bool isStaticLive[1000];//记录静止敌人是否活着
private:
   osg::ref_ptr<osg::MatrixTransform> myDirt;
   osg::ref_ptr<osg::Group> myRoot;
   osg::ref_ptr<osg::Node> t80;
   osg::ref_ptr<osg::Node> t72;
   osg::ref_ptr<osg::Node> machinegun;
   osg::ref_ptr<osg::MatrixTransform> m1;

/*生成敌人的函数*/
   osg::ref_ptr<osg::MatrixTransform> createEnemy(osg::Vec4d pos){//传入位置和角度
       osg::ref_ptr<osg::MatrixTransform> enemytrans=new osg::MatrixTransform;

       enemytrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(pos[3]),0.0,0.0,1.0)\
       *osg::Matrix::translate(osg::Vec3d(pos[0],pos[1],pos[2])));
       enemytrans->addChild(t80.get());
       return enemytrans;
   }
   osg::ref_ptr<osg::MatrixTransform> createEnemy(osg::Vec3d pos,int type){//传入位置和坦克类型
       osg::ref_ptr<osg::MatrixTransform> enemytrans=new osg::MatrixTransform;

       enemytrans->setMatrix(osg::Matrix::translate(pos));
       if(type==1){
           enemytrans->addChild(t72.get());
       }else if(type==2){
           enemytrans->addChild(machinegun.get());
       }else{
           enemytrans->addChild(m1.get());
       }

       return enemytrans;
   }
   void generateRandomEnemy(osg::Vec3d mypos,int enemyNums,int width,int depth){
       std::default_random_engine rand;//利用随机数生成坐标
       rand.seed(time(0));
       std::uniform_real_distribution<double> randWidth(mypos.x()-width, mypos.x()+width);
       std::uniform_real_distribution<double> randDepth(mypos.y()-depth, mypos.y()+depth);

       /*在某些位置生成一些会移动的目标*/
       std::vector<osg::Vec4d> moveInitPos{osg::Vec4d(277,-1588,0,90),\
       osg::Vec4d(155,-1682,0,0),osg::Vec4d(2,-1600,0,90),\
       osg::Vec4d(232,-1626,0,180),osg::Vec4d(452,-1811,0,110),\
       osg::Vec4d(433,-1628,0,180),osg::Vec4d(306,-1979,0,20)};
       osg::ref_ptr<osg::MatrixTransform> tmptrans;
       for(int i=0;i<moveInitPos.size();i++){
           tmptrans=createEnemy(moveInitPos[i]);

           myRoot->addChild(tmptrans);
           moveEnemyTrans.push_back(tmptrans);
           moveEnemyPos.push_back(osg::Vec3d(moveInitPos[i][0],moveInitPos[i][1],moveInitPos[i][2]));
           moveEnemyInitPos.push_back(osg::Vec3d(moveInitPos[i][0],moveInitPos[i][1],moveInitPos[i][2]));
           moveEnemyRota.push_back(moveInitPos[i][3]);
           isMoveLive[i]=true;
       }

       /*生成一些随机位置的静止目标*/
       for (int i = 0; i < enemyNums; i++) {
           double tmpx=randWidth(rand);
           double tmpy=randDepth(rand);
           double z=getHight(tmpx,tmpy,myDirt);
           if(i<enemyNums/3){
               tmptrans=createEnemy(osg::Vec3d(tmpx,tmpy,z),1);
           }else if(i<2*enemyNums/3){
               tmptrans=createEnemy(osg::Vec3d(tmpx,tmpy,z),2);
           }else{
               tmptrans=createEnemy(osg::Vec3d(tmpx,tmpy,z),3);
           }

           myRoot->addChild(tmptrans);
           staticEnemyTrans.push_back(tmptrans);
           staticEnemyPos.push_back(osg::Vec3d(tmpx,tmpy,z));
           isStaticLive[i]=true;
       }
   }
};

/* 节点位置数据 */
class dataType : public osg::Referenced
{
public:
   nodeMoveState *nms;
   bldTank *myTank;
   Enemy *myEnemy;
   osg::Vec3d bulletV;//炮弹的速度，0为x,1为y,2为z

   dataType(bldTank *n,Enemy *enemy){
       nms=n->nms;
       myTank=n;
       myEnemy=enemy;

       osg::Matrix mat_tmp=osg::Matrix::translate(0,2.28,0.04)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos);
       nms->gunHead.set(mat_tmp.getTrans());
       nms->bulletBoomPos.set(9999.0,9999.0,9999.0);

   };
   void updatePosition(){
       if(myEnemy->moveEnemyPos.size()>0){//更新移动敌人的位置
           double scale=0.3;

           for(int i=0;i<myEnemy->moveEnemyInitPos.size();i++){
               if(myEnemy->isMoveLive[i]==false){
                   continue;
               }
               osg::Vec3d newPos=myEnemy->moveEnemyPos[i]+osg::Vec3d(-sin(M_PI*myEnemy->moveEnemyRota[i]/180)*scale,cos(M_PI*myEnemy->moveEnemyRota[i]/180)*scale,0);
               double predis=getDistance(myEnemy->moveEnemyPos[i],myEnemy->moveEnemyInitPos[i]);
               double curdis=getDistance(newPos,myEnemy->moveEnemyInitPos[i]);

               if(curdis>predis && curdis>100){
                   myEnemy->moveEnemyRota[i]+=180;
                   if(myEnemy->moveEnemyRota[i]>360) myEnemy->moveEnemyRota[i]-=360;
               }

               myEnemy->moveEnemyPos[i].set(newPos);
               myEnemy->moveEnemyTrans[i]->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(myEnemy->moveEnemyRota[i]),0.0,0.0,1.0)\
           *osg::Matrix::translate(newPos));
           }
       }
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

           bool flag=false;
           for(int i=0;i<myEnemy->moveEnemyInitPos.size();i++){//炸弹飞行过程中是否击中运动中的敌人
               if(myEnemy->isMoveLive[i]==false) continue;
               double bullet2enemy=getDistance(curPos,myEnemy->moveEnemyPos[i]);
               if(bullet2enemy<4){
                   flag=true;
                   break;
               }
           }

           for (int j = 0; j < myEnemy->staticEnemyPos.size() && !flag; j++) {
               if(myEnemy->isStaticLive[j]==false) continue;
               double bullet2enemy=getDistance(curPos,myEnemy->staticEnemyPos[j]);
               if(bullet2enemy<4){
                   flag=true;
                   break;
               }
           }

           if (curPos.z()<z or flag){//炸弹碰到了敌人或者落地，立刻爆炸
               nms->bullet=false;
               nms->boom=true;
               myTank->trans->removeChild(myTank->bullettrans);
               nms->bulletBoomPos.set(curPos.x(),curPos.y(),z);
           }
       }

       if(nms->up){//上下左右炮塔炮体等逻辑
           double x=nms->tankPos.x()-sin(M_PI*nms->tankRota[0]/180);
           double y=nms->tankPos.y()+cos(M_PI*nms->tankRota[0]/180);
           double z=getHight(x,y,nms->myDirt);
           if(abs(z-nms->tankPos.z())>5.0) return;//两次位移的z轴差不能超过5
           printVec3(nms->tankPos);//输出当前位置
           nms->tankPos.set(nms->tankPos);
           nms->tankPos.set(x,y,z);

           //更新坦克三个部件的位置
           myTank->guntrans->setMatrix(osg::Matrix::translate(0.0,-1.22,-2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[2]),1.0,0.0,0.0)*\
           osg::Matrix::translate(0.0,1.22,2.59)*\
           osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->turrettrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]+nms->tankRota[1]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           myTank->tanktrans->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos));

           //为了攻击后炸弹能从枪口射出，随时记录枪口位置
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
           if(abs(z-nms->tankPos.z())>5.0) return;
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
};

/* 按键回调策略 */
class UserEventHandler : public osgGA::GUIEventHandler{
public:
   UserEventHandler(nodeMoveState *tids,Enemy *enemy){
       myEnemy=enemy;
       nms=tids;
   }
   void createGunFire(osg::Group* root){//产生枪口火焰
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
   void createBoom(osg::Group* root){//产生爆炸
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

       for(int i=0;i<myEnemy->moveEnemyInitPos.size();i++){
           if(myEnemy->isMoveLive[i]==false) continue;
           double dis=getDistance(position,myEnemy->moveEnemyPos[i]);
           if(dis<4){
               root->removeChild(myEnemy->moveEnemyTrans[i]);
               myEnemy->isMoveLive[i]=false;
               break;
           }
       }

       for(int j=0;j<myEnemy->staticEnemyPos.size();j++){
           if(myEnemy->isStaticLive[j]==false) continue;
           double dis=getDistance(position,myEnemy->staticEnemyPos[j]);
           if(dis<4){
               root->removeChild(myEnemy->staticEnemyTrans[j]);
               myEnemy->isStaticLive[j]=false;
               break;
           }
       }


       nms->bulletBoomPos.set(1.0,1.0,1.0);
   }
   virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa) {
       osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
       osg::Group* root = dynamic_cast<osg::Group*>(viewer->getSceneData());
       if (!root) return false;
       if(nms->boom==true){//爆炸特效
           nms->boom=false;
           createBoom(root);
       }
       if(nms->up or nms->down){//移动产生扬尘特效
           nms->myPartcile->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(nms->tankRota[0]),0.0,0.0,1.0)*\
           osg::Matrix::translate(nms->tankPos+osg::Vec3d(3*sin(M_PI*nms->tankRota[0]/180),-3*cos(M_PI*nms->tankRota[0]/180),1)));
           if(!nms->havePartcile){
               root->addChild(nms->myPartcile);
               nms->havePartcile=true;
           }
       }else if(nms->havePartcile){//静止移除扬尘
           root->removeChild(nms->myPartcile);
           nms->havePartcile=false;
       }
       switch (ea.getEventType()) {
           case osgGA::GUIEventAdapter::KEYDOWN: {//各种按键响应
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
   Enemy *myEnemy;
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
       m_vPosition = osg::Vec3(0.0, -50.0, 10.5+nms->tankPos[2]);//相机位于坦克的后上放
       m_vRotation = osg::Vec3(85.0, 0.0f, 0.0f);//相机角度
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
   osg::Vec3d initPos(277.761,-1826.73,0);
   nodeMoveState *tids=new nodeMoveState(dirt,initPos);
   bldTank* mytank=new bldTank(tids);
   root->addChild(mytank->trans.get());

   /*初始化敌军*/
   Enemy* myEnemy=new Enemy(initPos,50,500,500,root,dirt);

   /*按键响应及回调策略*/
   dataType *transData=new dataType(mytank,myEnemy);
   mytank->trans->setUserData(transData);
   mytank->trans->setUpdateCallback(new updateTankPosCallback(tids));
   viewer->addEventHandler(new UserEventHandler(tids,myEnemy));

   /*相机跟随*/
   viewer->setCameraManipulator(new Follow(viewer,tids));

   viewer->setSceneData(root.get());
   viewer->realize();
   viewer->run();
   return 0;
}