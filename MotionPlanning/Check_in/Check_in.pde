import queasycam.*;

QueasyCam cam;

void setup() 
{
  //Create Window
  //3D Rotational Dynamics
  size(1000, 750, P3D);
  surface.setTitle("Motion planning");
  surface.setLocation(displayWidth/4-width/2,displayHeight/4-height/2);
  pixelDensity(displayDensity());
  hint(ENABLE_DEPTH_TEST);
  
  //set cam
  cam = new QueasyCam(this);
  cam.speed = 2;              // default is 3
  cam.sensitivity = 0.5;      // default is 2
  cam.controllable = true;
  cam.position = new PVector(0,0,1000);//
  cam.pan = -PI/2;
  perspective(PI/3, (float)width/height, 0.01, 10000);
  
  agentP.set(start.x,start.y,agentH/2).mult(mag);
  pball.set(0,0,ballR).mult(mag);
}
void update(float dt) 
{
 
}
//Allow the user to push the mass with the left and right keys
void keyPressed() {
  if (keyCode == RIGHT) 
  {
    
  }
  if (keyCode == LEFT) 
  {
   
  }
  if (keyCode == UP) {
    
  }
  if (keyCode == DOWN) {
   
  }
  
  if (keyCode == ENTER){
   
  }
  
  if (keyCode == BACKSPACE){
    
  }
}
void keyReleased()
{
  
}
void mouseDragged() 
{  
  
}


void draw() 
{
  background(255,255,255);
  //ambientLight(102, 102, 102);
  //directionalLight(51, 102, 126, 0, 0, -1);
  //spotLight(51, 102, 126, 0, 0, 20, 0, 0, -1, PI/2, 1);
  drawcylinder();
  drawsphere();
  CheckeredFloor();
  
}
void CheckeredFloor() 
{
  float boxw=mag;
  float boxh=mag;
  float boxd=7;
  noStroke();
  for (int i = 0; i < roomw; i = i+1) {
    for (int j = 0; j < roomh; j = j+1) {
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
      translate ( boxw*(i-roomw/2), boxh*(j-roomh/2) , Floorz);
      box ( boxw, boxh, boxd);  // one cell / tile 
      popMatrix();
    } // for
  } // for
} // function 

void drawcylinder()
{
  float R=agentR*mag;
  float H=agentH*mag;
  fill(130, 82, 1);//wood brown
  noStroke();
  pushMatrix();    
  translate(agentP.x,agentP.y,agentP.z); 
  drawCylinder( 30, R , H );
  popMatrix();
}
void drawsphere()
{
  float radius=ballR*mag;
  fill(226,56,34);
  noStroke();
  pushMatrix();
  translate(pball.x,pball.y,pball.z);
  sphere(radius);
  popMatrix();
}
