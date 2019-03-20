import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.CountDownLatch;

import queasycam.*;

QueasyCam cam;

PVector g=new PVector(0,9.8,0);
PImage img;
float radius = 100;
PVector pball1;
float floor;
float fsizex;
float fsizey;
Cloth cloth;
Matrix Word2ViewMatrix;

int KeyboardControl=-1;
void setup() 
{
  //Create Window
  //3D Rotational Dynamics
  size(1000, 750, P3D);
  surface.setTitle("Cloth simulation");
  surface.setLocation(displayWidth/4-width/2,displayHeight/4-height/2);
  pixelDensity(displayDensity());
  hint(ENABLE_DEPTH_TEST);
  
  //set cam
  cam = new QueasyCam(this);
  cam.speed = 3;              // default is 3
  cam.sensitivity = 0.5;      // default is 2
  cam.controllable = true;
  cam.position = new PVector(562,432,400);
  cam.pan = -PI/2;
  //cam.position = new PVector(100,500,0);//(100,500,0)
  perspective(PI/3, (float)width/height, 0.01, 10000);
  //set texture //<>//
  img = loadImage("cloth_texture.jpg");
 
  pball1= new PVector(width/2,3*height/4,-40); //<>//
  floor = pball1.y+radius;
  heightOffset = pball1.y-2*radius;
  widthOffset = pball1.x-1.2*radius;
  ZOffset = 50;
  fsizex = 5*width; //<>//
  fsizey = 5*height;
  cloth=new Cloth();
  cloth.FindaNeighbors(); //<>//
  
  
}
void update(float dt) 
{
  cloth.UpdatePhysics(dt);
  cloth.Collision();
  
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
  
  if (keyCode == ENTER){
    cam.controllable = true;
  }
  
  if (keyCode == BACKSPACE){
    cam.controllable = false;
  }
}
void keyReleased()
{
  KeyboardControl=-1;
}
void mouseDragged() 
{  
  float mZ = map(mouseX,0,width,cloth.ClothParticle[29][29].pos.z,cloth.ClothParticle[0][0].pos.z);
  float mY = map(mouseY,0,height,cloth.ClothParticle[0][0].pos.y,cloth.ClothParticle[29][29].pos.y);
  float mpZ = map(pmouseX,0,width,cloth.ClothParticle[29][29].pos.z,cloth.ClothParticle[0][0].pos.z);
  float mpY = map(pmouseY,0,height,cloth.ClothParticle[0][0].pos.y,cloth.ClothParticle[29][29].pos.y);

    for (int i=0;i<cloth.width;++i) 
    {
      for (int j=0;j<cloth.height;++j) 
      {
        Particle p=cloth.ClothParticle[i][j];
        if (Math.abs(mZ - p.pos.z) < 10 && Math.abs(mY - p.pos.y) < 10) {
          if (mouseButton == LEFT){
            PVector delta_V= new PVector(0, 5*(mY-mpY), 5*(mZ - mpZ));
            p.vel.add(delta_V);
            if (delta_V.mag() > 30){
              cloth.ClothParticle[i][j].flag = false;
              p.neighbors.clear();
            }
          }
          else
            p.neighbors.clear();
            
        }
      }
    }
}


void draw() 
{
  float dt=0.03;
  background(200);
  ambientLight(102, 102, 102);
  directionalLight(51, 102, 126, 1, 1, 0);
  spotLight(51, 102, 126, 0, -20, 0, 0, 1, 0, PI/2, 1);

  //println(cam.position);
  update(dt); 
  drawsphere();
  drawCloth(); //<>//
  CheckeredFloor();
  textSize(32);
  stroke(0, 200, 250, 255);
  fill(0, 200, 250, 255);
  text(frameRate, width - 400, 300);
}
void drawCloth()
{
  pushMatrix();
  //draw cloth
  textureMode(NORMAL);
  for(int j=0;j<cloth.height-1;j++)
  {
    beginShape(TRIANGLE_STRIP);//triangles
    texture(img);
    noStroke();   
    noFill();  
    for(int i=0;i<cloth.width-1;++i)
    {
      if (!cloth.ClothParticle[i][j].flag){
        continue;
      }
      else{
        float u = map(i, 0, cloth.width-1, 0, 1);
        float v1 = map(j, 0, cloth.height-1, 0, 1);
        float v2 = map(j+1, 0, cloth.height-1, 0, 1);
        
        float x1 = cloth.ClothParticle[i][j].pos.x;
        float y1 = cloth.ClothParticle[i][j].pos.y;
        float z1 = cloth.ClothParticle[i][j].pos.z; //<>//
        vertex(x1,y1,z1,u,v1);
        
        float x2 = cloth.ClothParticle[i][j+1].pos.x;
        float y2 = cloth.ClothParticle[i][j+1].pos.y;
        float z2 = cloth.ClothParticle[i][j+1].pos.z;
        vertex(x2,y2,z2,u,v2);
      }
      
    }
      endShape(CLOSE);//
  }

  //for(int j=0;j<cloth.height-1;j++)
  //{
  //  PVector p=cloth.ClothParticle[cloth.width-1][j].pos;
  //  PVector p1=cloth.ClothParticle[cloth.width-1][j+1].pos;
  //  line(p.x,p.y,p.z, p1.x,p1.y,p1.z);
  //}
  //for(int i=0;i<cloth.width-1;++i)
  //{ //<>//
  //  PVector p=cloth.ClothParticle[i][cloth.height-1].pos;
  //  PVector p1=cloth.ClothParticle[i+1][cloth.height-1].pos;
  //  line(p.x,p.y,p.z, p1.x,p1.y,p1.z);
  //}
  popMatrix();
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
        } //<>//
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
  fill(110,200,0);
  noStroke();
  pushMatrix();
  translate(fsizex/2,floor,0);
  rotateX(PI/2);
  rectMode(CENTER);
  rect(0,0,fsizex,fsizey);
  popMatrix();
}

void drawsphere()
{
   pushMatrix();
  //draw ball
  translate(pball1.x,pball1.y,pball1.z);
  noStroke();
  fill(255);
  sphere(radius);
  popMatrix();
}
