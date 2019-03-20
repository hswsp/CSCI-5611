
//Initila check floor
boolean boolUseCheckeredFloor = true; 
//Simulation Parameters
float floor = 500;
PVector gravity = new PVector(0,9.8,0);
float radius = 20;
PVector stringTop=new PVector(600/2,50,0);
float restLen = 40;
float mass = 30;
float k = 20; //1 1000
float kv = 20;

int flag = 0;//mouse interaction

//camera
import queasycam.*;

QueasyCam cam;

//Inital positions and velocities of masses
PVector pball1;
PVector vball1;

PVector pball2;
PVector vball2;

PVector pball3;
PVector vball3;


void setup() 
{
  pball1 = new PVector(200,200,0);
  pball2= new PVector(300,250,0);
  pball3= new PVector(400,300,0);
  vball1= new PVector(0,0,0);
  vball2 = new PVector(0,0,0);
  vball3 = new PVector(0,0,0);
  //Create Window
//3D Rotational Dynamics
  size(600, 600, P3D);
  pixelDensity(displayDensity());
  surface.setTitle("Ball on Spring!");
  //light
  lights();
  //camera
  //cam = new QueasyCam(this);
  //cam.speed = 3;              // default is 3
  //cam.sensitivity = 1;      // default is 2
  
}

void update(float dt) {
   //Compute (damped) Hooke's law for each spring 
  PVector l1= PVector.sub(pball1,stringTop);
  float l1norm=l1.mag();
  l1.normalize();
  
  PVector l2= PVector.sub(pball2,pball1);
  float l2norm=l2.mag();
  l2.normalize();
  
  PVector l3= PVector.sub(pball3,pball2);
  float l3norm=l3.mag();
  l3.normalize();
  
  PVector v2= PVector.sub(vball2,vball1);
  PVector v3= PVector.sub(vball3,vball2);

  
  PVector stringF1 = PVector.mult(l1,-k*(l1norm - restLen));
  PVector dampF1 = PVector.mult(vball1,-kv);
  PVector forceY1 = PVector.add(stringF1,dampF1);
  
  PVector stringF2 = PVector.mult(l2,-k*(l2norm - restLen));
  PVector dampF2 =   PVector.mult(v2,-kv);
  PVector forceY2 = PVector.add(stringF2,dampF2);
  
  PVector stringF3 = PVector.mult(l3,-k*(l3norm - restLen));
  PVector dampF3 = PVector.mult(v3,-kv);
  PVector forceY3 = PVector.add(stringF3,dampF3);
 
  //If are are doing this right, the top spring should be much longer than the bottom
  println("l1:",pball1.y - stringTop.y, " l2:",pball2.y - pball1.y, " l3:",pball3.y-pball2.y);
  
  //Eulerian integration
  PVector accY1 = PVector.add(PVector.add(gravity,PVector.mult(forceY1,.5/mass)), PVector.mult(forceY2,- .5/mass)); 
  pball1.add(PVector.add(PVector.mult(vball1,dt),PVector.mult(gravity,0.5*dt*dt)));
  vball1.add(PVector.mult(accY1,dt));
  
  PVector accY2= PVector.add(PVector.add(gravity,PVector.mult(forceY2,.5/mass)), PVector.mult(forceY3,- .5/mass)); 
  pball2.add(PVector.add(PVector.mult(vball2,dt),PVector.mult(gravity,0.5*dt*dt)));
  vball2.add(PVector.mult(accY2,dt));
  
  PVector accY3 = PVector.add(gravity,PVector.mult(forceY3,.5/mass)); 
  pball3.add(PVector.add(PVector.mult(vball3,dt),PVector.mult(gravity,0.5*dt*dt)));
  vball3.add(PVector.mult(accY3,dt));
  
  
  //Collision detection and response
  if (pball1.y+radius > floor)
  {
    vball1.y*= -.9;
    pball1.y = floor - radius;
  }
  if (pball2.y+radius > floor)
  {
    vball2.y *= -.9;
    pball2.y = floor - radius;
  }
  if (pball3.y+radius > floor)
  {
    vball3.y *= -.9;
    pball3.y = floor - radius;
  }
  
}

//Allow the user to push the mass with the left and right keys
void keyPressed() {
  if (keyCode == RIGHT) 
  {
    vball3.x+=30;
   
  }
  if (keyCode == LEFT) {
    vball3.x-=30;
   
  }
  if (keyCode == UP) {
    vball3.z-=50;
   
  }
  if (keyCode == DOWN) {
    vball3.z+=50;
   
  }
}


//Draw the scene: one sphere for the mass, and one line connecting it to the anchor
void draw() 
{
  background(255,255,255);
  CheckeredFloor() ;
  //offSetZ++;
  update(.04); //We're using a fixed, large dt -- this is a bad idea!!
  pushMatrix();
  fill(0,0,0);
  stroke(5);
  line(stringTop.x,stringTop.y,pball1.x,pball1.y);
  //draw ball
  translate(pball1.x,pball1.y);
  noStroke();
  fill(0, 200, 10);
  sphere(radius);
  popMatrix();
  
  pushMatrix();
  fill(0,0,0);
  stroke(5);
  line(pball1.x,pball1.y,pball2.x,pball2.y);
  translate(pball2.x,pball2.y);
  noStroke();
  fill(0, 200, 10);
  sphere(radius);
  popMatrix();
  
  pushMatrix();
  fill(0,0,0);
  stroke(5);
  line(pball2.x,pball2.y,pball3.x,pball3.y);
  translate(pball3.x,pball3.y);
  noStroke();
  fill(0, 200, 10);
  sphere(radius);
  popMatrix();
  
  //drag the lowest ball
  if (mousePressed){
    float mouse_X = mouseX;
    float mouse_Y = mouseY;
    if (flag == 0){
      pball3.x = mouseX;
      pball3.y = mouseY;
      flag = 1;
    }
    if (flag == 1){
      pball3.x = mouse_X;
      pball3.y = mouse_Y;
      flag = 0;
    }
  }
  
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
