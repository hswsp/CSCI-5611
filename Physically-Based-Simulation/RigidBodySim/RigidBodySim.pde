import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.CountDownLatch;

import queasycam.*;
QueasyCam cam;

//import peasy.*;
//PeasyCam cam;
PVector g=new PVector(0,9.8,0);
PImage img;
float radius = 100;
PVector pball1;
float floor;
float fsizex;
float fsizey;
float up,right,left,forward,back; //range of activation
RigidBox[] rigidBodies;//
RigidCylinder[] rigidCylinders;
RigidCone[] rigidCons;
Matrix Word2ViewMatrix;
int KeyboardControl=-1;

PVector testball=new PVector(0,0,0);
PVector teste1=new PVector(0,0,0);
PVector teste2=new PVector(0,0,0);
void setup() 
{
  //Create Window
  //3D Rotational Dynamics
  size(1000, 750, P3D);
  surface.setTitle("Cloth simulation");
  surface.setLocation(displayWidth/4-width/2,displayHeight/4-height/2);
  pixelDensity(displayDensity());
  hint(ENABLE_DEPTH_TEST);
  
  pball1= new PVector(width/2,3*height/4,-100); 
  //set cam
  cam = new QueasyCam(this);
  perspective(PI/3, (float)width/height, 0.01, 10000);
  cam.speed = 1;              // default is 3
  cam.sensitivity = 0.5;      // default is 2
  cam.controllable = true;
  cam.pan = -PI/2;
  cam.position = new PVector(pball1.x,pball1.y-100,pball1.z+500);
  double[][]A=
  {
    {1,0,0},
    {0,0,-1},
    {0,-1,0}
  };
  Word2ViewMatrix=new Matrix(A);
  
  fsizex = 5*width;
  fsizey = 5*height;
  floor = pball1.y+radius;
  
  up = floor - 2*height;
  left =  pball1.x- 800;
  right =  pball1.x + 800;
  forward = pball1.z -800;
  back = pball1.z + 800;
  InitialRigidBody();
 
}
void InitialRigidBody()
{
   //Initila box
  rigidBodies=new RigidBox[5];
  for(int i=0;i<5;++i)
  {
    rigidBodies[i]=new RigidBox(100+10*i,100-5*i,60-5*i,50);
  }
  PVector StickIniPos=PVector.add(pball1,new PVector(-2*radius,0.9*radius-rigidBodies[0].shape.zlen,1.5*radius));// (50,50,-100)
  rigidBodies[0].pos=View2Word(StickIniPos);
  float theta=PI/3;//elevation
  float alpha=-PI/4;
  rigidBodies[0].pVel.set(PVector.mult(new PVector(cos(alpha)*cos(theta),sin(alpha)*cos(theta),sin(theta)),50));
  rigidBodies[0].L.set(0,0,100000);//15000,100000
  rigidBodies[0].AddForceAndTorque(PVector.mult(View2Word(g),rigidBodies[0].shape.mass),new PVector(0,0,0));
  
  StickIniPos.set(PVector.add(pball1,new PVector(2*radius,0.9*radius-rigidBodies[1].shape.zlen,1.5*radius)));// (50,50,-100)
  //rigidBodies[1].shape.mass=10000;
  rigidBodies[1].pos=View2Word(StickIniPos);
  theta=PI/3;//elevation
  alpha=-3*PI/4;
  rigidBodies[1].pVel.set(PVector.mult(new PVector(cos(alpha)*cos(theta),sin(alpha)*cos(theta),sin(theta)),50));
  rigidBodies[1].L.set(0,0,100000);//
  rigidBodies[1].AddForceAndTorque(PVector.mult(View2Word(g),rigidBodies[1].shape.mass),new PVector(0,0,0));
  
  StickIniPos.set(PVector.add(pball1,new PVector(-2*radius,0.9*radius-rigidBodies[2].shape.zlen,-1.5*radius)));// (50,50,-100)
  rigidBodies[2].pos=View2Word(StickIniPos);
  theta=PI/3;//elevation
  alpha=PI/4;
  rigidBodies[2].pVel.set(PVector.mult(new PVector(cos(alpha)*cos(theta),sin(alpha)*cos(theta),sin(theta)),50));
  rigidBodies[2].L.set(0,-15000,-100000);//
  rigidBodies[2].AddForceAndTorque(PVector.mult(View2Word(g),rigidBodies[2].shape.mass),new PVector(0,0,0));
  
  StickIniPos.set(PVector.add(pball1,new PVector(2*radius,0.9*radius-rigidBodies[3].shape.zlen,-1.5*radius)));// (50,50,-100)
  rigidBodies[3].pos=View2Word(StickIniPos);
  theta=PI/3;//elevation
  alpha=3*PI/4;
  rigidBodies[3].pVel.set(PVector.mult(new PVector(cos(alpha)*cos(theta),sin(alpha)*cos(theta),sin(theta)),50));
  rigidBodies[3].L.set(0,-15000,-100000);//
  rigidBodies[3].AddForceAndTorque(PVector.mult(View2Word(g),rigidBodies[3].shape.mass),new PVector(0,0,0));
  
  
  //initial Cone
  rigidCons=new RigidCone[5];
  rigidCons[0] = new RigidCone(50, 100 ,2);
  PVector ConeIniPos= PVector.add(pball1,new PVector(0,radius-50,0));//(2*radius,radius-50,-1.5*radius)
  rigidCons[0].pos=View2Word(ConeIniPos);
  theta=PI/3;
  alpha=-3*PI/4;
  rigidCons[0].pVel.set(PVector.mult(new PVector(cos(alpha)*cos(theta),sin(alpha)*cos(theta),sin(theta)),50));
  //initial impulse
  PVector f=new PVector(50,0,10);//1000
  PVector arm=new PVector(0,-3*rigidCons[0].shape.height/4,0);
  rigidCons[0].AddForceAndTorque(f,arm);
  rigidCons[0].AddForceAndTorque(PVector.mult(View2Word(g),rigidCons[0].shape.mass),new PVector(0,0,0));
  
  rigidCons[0].L.set(0,0,500);//initial angular momentum
  
  rigidCons[0].Updata(0.1);
  rigidCons[0].ResetForceAndTorque();
  rigidCons[0].AddForceAndTorque(PVector.mult(View2Word(g),rigidCons[0].shape.mass),new PVector(0,0,0));
  
  
  //initial Cylinder
  rigidCylinders=new RigidCylinder[5];
  rigidCylinders[0] = new RigidCylinder(50, 100 ,5);
  PVector CylinderIniPos= PVector.add(pball1,new PVector(-radius,-radius-50,1.5*radius));
  rigidCylinders[0].pos=View2Word(CylinderIniPos);
  theta=PI/6;
  alpha=-PI/4;
  rigidCylinders[0].pVel.set(PVector.mult(new PVector(cos(alpha)*cos(theta),sin(alpha)*cos(theta),sin(theta)),50));
  //initial impulse
  f.set(0,500,-500);
  arm.set(rigidCylinders[0].shape.height/2,0.0,0.0);//PVector.add(rigidBodies[0].pos,)
  rigidCylinders[0].AddForceAndTorque(f,arm);
  rigidCylinders[0].AddForceAndTorque(PVector.mult(View2Word(g),rigidCylinders[0].shape.mass),new PVector(0,0,0));
  rigidCylinders[0].Updata(0.1);
  rigidCylinders[0].ResetForceAndTorque();
  rigidCylinders[0].AddForceAndTorque(PVector.mult(View2Word(g),rigidCylinders[0].shape.mass),new PVector(0,0,0));
}

void update(float dt) 
{
  rigidCylinders[0].Updata(dt);
  rigidCons[0].Updata(dt);
  for(int i=0;i<4;++i)
  {
    rigidBodies[i].Updata(dt);
    rigidBodies[i].BoxWallCollision();
    for(int j=0;j<4;++j)
    {
      if(i!=j)
      {
        rigidBodies[i].BoxbetweenCollision(rigidBodies[j],dt);
      }
    }
  }
  switch(KeyboardControl)
  {
    case 1: 
    pball1.x+=5;
    break;
   case 2:
    pball1.x-=5;
    break;
    case 3:
    pball1.y-=5;
    break;
    case 4:
    pball1.y+=5;
    if(pball1.y+radius>floor){
     pball1.y=floor-radius;
    }
    break;
  }
}
//Allow the user to push the mass with the left and right keys
void keyPressed() {
  if (keyCode == RIGHT) 
  {
    KeyboardControl=1;
    //pball1.x+=5;
  }
  if (keyCode == LEFT) 
  {
    KeyboardControl=2;
   // pball1.x-=5;
  }
  if (keyCode == UP) {
    KeyboardControl=3;
   //pball1.y-=5;
  }
  if (keyCode == DOWN) {
    KeyboardControl=4;
   //pball1.y+=5;
  }
}
void keyReleased()
{
  KeyboardControl=-1;
}
void mouseDragged() 
{
  
}

void draw() 
{
  float dt=0.01;
  background(200);
  ambientLight(102, 102, 102);
  directionalLight(51, 102, 126, 1, 1, 0);
  spotLight(51, 102, 126, 0, -20, 0, 0, 1, 0, PI/2, 1);
  
  update(dt); 
  //drawline();  //debug only
  //drawsphere();
  drawstick();
  drawcylinder();
  drawcone();
  drawFloor();
  CheckeredFloor();
  textSize(32);
  stroke(0, 200, 250, 255);
  fill(0, 200, 250, 255);
  text(frameRate, width - 400, 300);
}

void CheckeredFloor() 
{
  float boxw=100;
  float boxz=100;
  int numx=20;
  int numz=20;
  noStroke();
  for (int i = 0; i < numx; i = i+1) {
    for (int j = 0; j < numz; j = j+1) {
      // % is modulo, meaning rest of division 
      if (i%2 == 0) { 
        if (j%2 == 0) { 
          fill (255, 0, 0);
        }
        else
        {
          fill (182, 155, 76 );
        }
      }  
      else {
        if (j%2 == 0) { 
          fill (182, 155, 76);
        }
        else
        {
           fill (255, 0, 0);
          
        }
      } // if
      pushMatrix();
      translate ( boxw*(i-numx/3), floor, boxz*j-1110);
      box ( boxw, 7, boxz);  // one cell / tile 
      popMatrix();
    } // for
  } // for
} // function 

void drawFloor()
{
  fill(153,51,255);
  noStroke();
  pushMatrix();
  translate(left,pball1.y-fsizey/8+50,0);
  rotateY(PI/2);
  rectMode(CENTER);
  rect(0,0,fsizex/2,fsizey/2);
  popMatrix();
  
  pushMatrix();
  translate(right,pball1.y-fsizey/8+100,0);
  rotateY(PI/2);
  rectMode(CENTER);
  rect(0,0,fsizex/2,fsizey/2);
  popMatrix();
  
  pushMatrix();
  translate(pball1.x,pball1.y-fsizey/8+100,forward);
  rectMode(CENTER);
  rect(0,0,fsizex/2,fsizey/2);
  popMatrix();
  pushMatrix();
  translate(pball1.x,pball1.y-fsizey/8+50,back);
  rectMode(CENTER);
  rect(0,0,fsizex/2,fsizey/2);
  popMatrix();
  
  pushMatrix();
  translate(pball1.x,up,pball1.z-fsizey/8);
  rotateX(PI/2);
  rectMode(CENTER);
  rect(0,0,fsizex,fsizey);
  popMatrix();
  
}
void drawline()
{
  PVector Xcm=Word2View(rigidBodies[0].pos);
  PVector e1=Word2View(testball);
  PVector e2=Word2View(teste1);
  PVector e3=Word2View(teste2);

  pushMatrix();
  fill(255);
  strokeWeight(5);
  line(Xcm.x, Xcm.y, Xcm.z, e1.x, e1.y, e1.z);
  line(Xcm.x, Xcm.y, Xcm.z, e2.x, e2.y, e2.z);
  line(Xcm.x, Xcm.y, Xcm.z, e3.x, e3.y, e3.z);
  popMatrix();
}
void drawsphere()
{
  pushMatrix();
  translate(pball1.x,pball1.y,pball1.z);
  noStroke();
  fill(255);
  sphere(radius);
  popMatrix();
  
  //only for debug
  pushMatrix();
  PVector O=Word2View(testball);
  translate(O.x,O.y,O.z);
  noStroke();
  fill(255);
  sphere(10);
  popMatrix();
  
  pushMatrix();
  PVector O1=Word2View(teste1);
  translate(O1.x,O1.y,O1.z);
  noStroke();
  fill(255);
  sphere(10);
  popMatrix();
  
  pushMatrix();
  PVector O2=Word2View(teste2);
  translate(O2.x,O2.y,O2.z);
  noStroke();
  fill(255);
  sphere(10);
  popMatrix();
}

void drawstick()
{
  for(int i=0;i<4;++i)
  {
    pushMatrix();
    noStroke();
    fill((200+50*i)%256,(100+10*i)%256,(0+50*i)%256);
    PVector Xcm=Word2View(rigidBodies[i].pos);
    PVector Ang=Word2View(rigidBodies[i].angle);
    float angle=Ang.mag();
    Ang.normalize();
    translate(Xcm.x,Xcm.y,Xcm.z); 
    rotate(angle, Ang.x, Ang.y, Ang.z);
    box(rigidBodies[i].shape.width,rigidBodies[i].shape.zlen,rigidBodies[i].shape.height);
    popMatrix();
  }
}
void drawcylinder()
{
  PVector Xcm=Word2View(rigidCylinders[0].pos);
  PVector Ang=Word2View(rigidCylinders[0].angle);
  float angle=Ang.mag();
  Ang.normalize();
  fill(0,255 , 128);
  pushMatrix();    
  translate(Xcm.x,Xcm.y,Xcm.z); 
  rotate(angle, Ang.x, Ang.y, Ang.z);
  drawCylinder( 30,  rigidCylinders[0].shape.radius,rigidCylinders[0].shape.height );
  popMatrix();
}

void drawcone()
{
  fill(255, 0, 128);
  PVector Xcm=Word2View(rigidCons[0].pos);
  PVector Ang=Word2View(rigidCons[0].angle);
  float angle=Ang.mag();
  Ang.normalize();
  pushMatrix();
  translate(Xcm.x,Xcm.y,Xcm.z); 
  rotate(angle, Ang.x, Ang.y, Ang.z);
  drawCone( 30, rigidCons[0].shape.radius, 0,rigidCons[0].shape.height );
  translate( 120, 330, 0 );
  popMatrix();
}
PVector Word2View(PVector Word)
{
  double [][]wt=
    {{Word.x},
    {Word.y},
    {Word.z}
    };
  Matrix Wt=new Matrix(wt);
  Matrix view=Word2ViewMatrix.inverse().times(Wt);
  return new PVector((float)view.get(0,0),(float)view.get(1,0),(float)view.get(2,0));
}
PVector View2Word(PVector Word)
{
  double [][]wt=
    {{Word.x},
    {Word.y},
    {Word.z}
    };
  Matrix Wt=new Matrix(wt);
  Matrix view=Word2ViewMatrix.times(Wt);
  return new PVector((float)view.get(0,0),(float)view.get(1,0),(float)view.get(2,0));
}
