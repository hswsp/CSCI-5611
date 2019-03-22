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
  cam.position = new PVector(0,0,800);//
  cam.pan = -PI/2;
  perspective(PI/3, (float)width/height, 0.01, 10000);
  
  agentP.set(start.x,start.y,agentH/2).mult(mag);
  pball.set(0,0,0).mult(mag);
  
  PRM();
  AStarSearch();
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
  drawRoadmap();
  CheckeredFloor();
  
  drawPath();
}
void star(float x, float y, float radius1, float radius2, int npoints) 
{
  float angle = TWO_PI / npoints;
  float halfAngle = angle/2.0;
  beginShape();
  for (float a = 0; a < TWO_PI; a += angle) {
    float sx = x + cos(a) * radius2;
    float sy = y + sin(a) * radius2;
    vertex(sx, sy);
    sx = x + cos(a+halfAngle) * radius1;
    sy = y + sin(a+halfAngle) * radius1;
    vertex(sx, sy);
  }
  endShape(CLOSE);
}
void drawRoadmap()
{
  int dimension=samples.size();
  pushMatrix();
  fill(0,0,255);
  translate(mag*start.x, mag*start.y,Z);
  rotate(frameCount / -100.0);
  star(0, 0, 30, 70, 5); 
  popMatrix();
  
  pushMatrix();
  fill(0,255,0);
  translate(mag*goal.x, mag*goal.y, Z);
  rotate(frameCount / -100.0);
  star(0, 0, 30, 70, 5); 
  popMatrix();
  
  pushMatrix();
  stroke(255,255,255);
  strokeWeight(5);
  for(int i=0;i<dimension;++i)
  {
    for(int j=0;j<dimension;++j)
    {
      if(weightmap[i][j]<Float.MAX_VALUE) 
      {
        PVector P1=samples.get(i);
        PVector P2=samples.get(j);
        line(P1.x,P1.y,Z,P2.x,P2.y,Z);
      }
    }
  }
  popMatrix();
}
void drawPath()
{
  pushMatrix();
  stroke(255,0,255);
  strokeWeight(5);
  Iterator<Node> iter = CLOSED.iterator(); 
  int i=0;
  int j=0;
  Node cur=iter.next();
  Node pre=cur.parent;
  while(iter.hasNext())
  {
    pre=cur;
    cur=iter.next();
  }

  while(pre!=null)
  {
    i=cur.Index;
    j=pre.Index;
    PVector P1=samples.get(i);
    PVector P2=samples.get(j);
    //println("P1=(",P1.x,P1.y,")","P2=(",P2.x,P2.y,")");
    line(P1.x,P1.y,Z,P2.x,P2.y,Z); //<>//
    cur=pre;
    pre=pre.parent;
  }
  popMatrix();
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
  fill(236, 229, 255);
  noStroke();
  pushMatrix();
  translate(pball.x,pball.y,ballR*mag);
  sphere(radius);
  popMatrix();
}
