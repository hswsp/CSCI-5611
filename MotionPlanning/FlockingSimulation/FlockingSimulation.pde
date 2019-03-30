import queasycam.*;
import java.awt.event.KeyEvent;
import java.lang.reflect.Method;
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
  cam.position = new PVector(0,0,850);//
  cam.pan = -PI/2;
  perspective(PI/3, (float)width/height, 0.01, 10000);
  InitialObstacles(pball,ballR);
  InitAgents();
 //<>//
}
void InitAgents()
{
  agents=new RRTAgent[AgentNum];
  //Magents = new PRMAgent[AgentNum];
  for(int i=0;i<AgentNum;++i)
  {
    agents[i] = new RRTAgent();
    //Magents[i] = new PRMAgent();
  }
  agents[1].start=new PVector(-9,9,0);
  agents[1].goal=new PVector(9,-9,0);
  
  //Magents[1].start=new PVector(-9,9,0);
  //Magents[1].goal=new PVector(9,-9,0);
 
  for(int i=0;i<AgentNum;++i)
  {
    agents[i].P.set(agents[i].start.x,agents[i].start.y,agents[i].H/2).mult(mag);
    agents[i].RRT_Road();
    agents[i].targetItr=agents[i].Path.iterator();
    if(agents[i].targetItr.hasNext())
    {
      agents[i].NexttarId=agents[i].CurtarId = agents[i].targetItr.next();
    }
    
    
    //Magents[i].P.set(Magents[i].start.x,Magents[i].start.y,Magents[i].H/2).mult(mag);
    //Magents[i].PRM_Road();
  }
  
}


void update(float dt) 
{
  if(IsStartAnimation)
  {
    UpdateAgent(dt);
  }
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
  if (keyCode == KeyEvent.VK_SPACE ){//SAPCE
    IsStartAnimation=true;
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
  float dt=0.01;
  background(255,255,255);
  update(dt);
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
void drawRoadmap()//draw PRM Road map
{
  for(int l=0;l<AgentNum;++l)
  {
    Vector<PVector> samples= agents[l].samples;
    PVector start=agents[l].start;
    PVector goal=agents[l].goal;
    Float weightmap[][] = agents[l].weightmap;
    
    //Vector<PVector> samples= Magents[l].samples;
    //PVector start=Magents[l].start;
    //PVector goal=Magents[l].goal;
    //Float weightmap[][] = Magents[l].weightmap;
    
    int dimension=samples.size();
    pushMatrix();
    fill(0,0,(255+50*l)%256);
    translate(mag*start.x, mag*start.y,Z);
    rotate(frameCount / -100.0);
    star(0, 0, 30, 70, 5); 
    popMatrix();
    
    pushMatrix();
    fill(0,(255+50*l)%256,0);
    translate(mag*goal.x, mag*goal.y, Z);
    rotate(frameCount / -100.0);
    star(0, 0, 30, 70, 5); 
    popMatrix();
    
    pushMatrix();
    for(int i=0;i<dimension;++i)
    {
      for(int j=0;j<dimension;++j)
      {
        if(weightmap[i][j]<Float.MAX_VALUE) 
        {
          PVector P1=samples.get(i);
          PVector P2=samples.get(j);
          //show edges
          stroke((255+50*l)%256,(255+50*l)%256,(255+50*l)%256);
          strokeWeight(2);
          line(P1.x,P1.y,Z,P2.x,P2.y,Z);
        }
      }
    }
    popMatrix();
  }
}
void drawPath()
{
  for(int i=0;i<AgentNum;++i)
  {
    Vector<PVector> samples= agents[i].samples;
    ArrayList<Integer>  Path=agents[i].Path;
    
    //Vector<PVector> samples= Magents[i].samples;
    //ArrayList<Integer>  Path=Magents[i].Path;
    
    pushMatrix();
    strokeWeight(5);
    for(int k=0;k<Path.size()-1;++k)
    {
      PVector P1=samples.get(Path.get(k));
      PVector P2=samples.get(Path.get(k+1));
      stroke((255+25*i)%256,(0+88*i)%256,(255+34*i)%256);
      line(P1.x,P1.y,Z,P2.x,P2.y,Z);
      // show milestones
      pushMatrix();
      noStroke();
      fill((186+25*i)%256,(85+88*i)%256,(211+34*i)%256);
      translate(P1.x,P1.y,Z);
      rotate(frameCount / -100.0);
      star(0, 0, 10, 23, 6); 
      popMatrix();
    }
    popMatrix();
  }
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
  for(int i=0;i<AgentNum;++i)
  {
    PVector agentP=agents[i].P;
    float R=agents[i].R*mag;
    float H=agents[i].H*mag;
    
    //PVector agentP=Magents[i].P;
    //float R=Magents[i].R*mag;
    //float H=Magents[i].H*mag;
    fill((130+25*i)%256, (82+55*i)%256, (1+88*i)%256);//wood brown
    noStroke();
    pushMatrix();    
    translate(agentP.x,agentP.y,agentP.z); 
    drawCylinder( 30, R , H );
    popMatrix();
  }
}
void drawsphere()
{
  for(int i=0;i<ObsNumber;++i)
  {
    float radius=ballR[i]*mag;
    fill(236, 229, 255);
    noStroke();
    pushMatrix();
    translate(pball[i].x,pball[i].y,ballR[i]*mag);
    sphere(radius);
    popMatrix();
  }
}

boolean Arrival(PVector destination,PVector agentP)
{
  float threshold=.1;
  if(PVector.sub(destination,agentP).mag()<threshold)
  {
    println("arrival!");
    return true;
  }
  else
  {
    return false;
  }
}
void UpdateAgent(float dt)
{
  for(int i=0;i<AgentNum;++i)
  {
    Vector<PVector> samples= agents[i].samples;
    PVector agentP=agents[i].P;
    PVector CurTarget=samples.get(agents[i].CurtarId);
    PVector NextTarget=samples.get(agents[i].NexttarId);
    PVector forward=new PVector();
    if(Arrival(CurTarget,agentP))
    {
      if(agents[i].targetItr.hasNext())
      {
        agents[i].CurtarId = agents[i].targetItr.next();
        CurTarget=samples.get(agents[i].CurtarId);
        agents[i].NexttarId = agents[i].targetItr.next();
        NextTarget=samples.get(agents[i].NexttarId);
      }
      else
      {
        return;
      }
    }
    if(intersection(agentP,NextTarget))
    {
      forward=PVector.sub(CurTarget,agentP).normalize();
    }
    else
    {
      forward=PVector.sub(NextTarget,agentP).normalize();
      agents[i].CurtarId = agents[i].NexttarId;
      if(agents[i].targetItr.hasNext())
      {
        agents[i].NexttarId = agents[i].targetItr.next();
      }
    }
    
    agentP.add(PVector.mult(forward,mag*dt));
  }
}
