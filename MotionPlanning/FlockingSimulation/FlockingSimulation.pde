import queasycam.*;
import java.awt.event.KeyEvent;
import java.lang.reflect.Method;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import com.manyangled.gibbous.optim.convex.*;
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
  perspective(PI/4, (float)width/height, 0.01, 10000);
  cam.speed = 2;              // default is 3
  cam.sensitivity = 0.5;      // default is 2
  cam.controllable = true;
  
  cam.pan = -PI/2;
  cam.position = new PVector(0,0,1200);//
  
  InitialObstacles(pball,ballR);
  InitAgents();
  

}
void InitAgents()
{
  multiAgents=new MultiAgents("RRT",agentnumber);
  multiAgents.init();
  
}


void update(float dt) 
{
  if(IsStartAnimation)
  {
    
    //UpdateAgentSmooth(dt);
    UpdateAgentLinearly(dt);
  }
}
//Allow the user to push the mass with the left and right keys
void keyPressed() {
  if (keyCode == RIGHT) 
  {
    Agent0goal=true;  
  }
  if (keyCode == LEFT) 
  {
    Agent0Start=true;
  }
  if (keyCode == UP) {
    
  }
  if (keyCode == DOWN) {
   
  }
  if (keyCode == SHIFT) {
   AddObstacle=true;
  }
  if (keyCode == ENTER){
   cam.controllable = false;
  }
  
  if (keyCode == BACKSPACE){
    
  }
  if (keyCode == KeyEvent.VK_SPACE ){//SAPCE
    IsStartAnimation=true;
  }
  
}
void keyReleased()
{
  if (keyCode == RIGHT) 
  {
    Agent0goal=false;  
  }
  if (keyCode == LEFT) 
  {
    Agent0Start=false;
  }
  if (keyCode == SHIFT) {
    AddObstacle=false;
  }
  if (keyCode == KeyEvent.VK_SPACE ){//SAPCE
    IsStartAnimation=false;
  }
}
void mouseClicked()
{
  float mX = map(mouseX,0,width,(-roomw/2-2.5)*mag,(roomw/2+2.5)*mag);
  float mY = map(mouseY,0,height,(-roomh/2)*mag,(roomh/2)*mag);
  if( AddObstacle)
  {
    AddObstacle(mX,mY);
    multiAgents.update();
  }
  if(Agent0Start)
  {
    ChangeStart(mX,mY);
    multiAgents.update();
  }
  if(Agent0goal)
  {
    ChnageGoal(mX,mY);
    multiAgents.update();
  }
}
void mouseDragged() 
{ 
  
  float mX = map(mouseX,0,width,(-roomw/2-2.5)*mag,(roomw/2+2.5)*mag);
  float mY = map(mouseY,0,height,(-roomh/2)*mag,(roomh/2)*mag);
  float mpX = map(pmouseX,0,width,(-roomw/2-2.5)*mag,(roomw/2+2.5)*mag);
  float mpY = map(pmouseY,0,height,(-roomh/2)*mag,(roomh/2)*mag);
  int ID=findObs(mpX,mpY);
  if(ID!=-1){
    pball[ID].x=mX;
     pball[ID].y=mY;
  }
  multiAgents.update();
}

int findObs(float mX,float mY)
{
  float Epsilon=5E-1*mag;
  for(int i=0;i<ObsNumber;++i)
  {
    if(abs(mX-pball[i].x)<Epsilon&&abs(mY-pball[i].y)<Epsilon){
      return i;
    }
  }
  return -1;
}
void AddObstacle(float mX, float mY)
{
  if(findObs(mX,mY)==-1)
  {
    ObsNumber+=1;
    PVector[] pballnew=new PVector[ObsNumber];
    float[] ballRnew = new float[ObsNumber];
    for(int i=0;i<ObsNumber-1;++i)
    {
      pballnew[i]=pball[i];
      ballRnew[i]=ballR[i];
    }
    pballnew[ObsNumber-1]=new PVector(mX,mY,0);
    ballRnew[ObsNumber-1]= ballR[0];
    pball=pballnew;
    ballR=ballRnew;
  }
}

void ChangeStart(float mX, float mY)
{
  if(multiAgents.agents!=null)
  {
    multiAgents.agents[0].P.x=mX;
    multiAgents.agents[0].P.y=mY;
  }
}
void ChnageGoal(float mX, float mY)
{
  if(multiAgents.agents!=null)
  {
    multiAgents.agents[0].goal.x=mX/mag;
    multiAgents.agents[0].goal.y=mY/mag;
  }
}
void draw() 
{
  //cam.controllable = false;
  //float dt=0.01;
  background(255,255,255);
  update(dt);
  //ambientLight(102, 102, 102);
  //directionalLight(51, 102, 126, 0, 0, -1);
  //spotLight(51, 102, 126, 0, 0, 20, 0, 0, -1, PI/2, 1);
  drawcylinder();
  drawsphere();
  //drawRoadmap();
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
  for(int l=0;l<multiAgents.AgentNum;++l)
  {
    Vector<PVector> samples= multiAgents.agents[l].samples;
    PVector start=multiAgents.agents[l].start;
    PVector goal=multiAgents.agents[l].goal;
    Float weightmap[][] = multiAgents.agents[l].weightmap;
    int dimension=samples.size();
    //draw start
    pushMatrix();
    noStroke();
    fill(0,0,(255+50*l)%256);
    translate(mag*start.x, mag*start.y,Z);
    rotate(frameCount / -100.0);
    star(0, 0, 30, 70, 5); 
    popMatrix();
    
    //draw goal
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
  for(int i=0;i<multiAgents.AgentNum;++i)
  {
    Vector<PVector> samples= multiAgents.agents[i].samples;
    ArrayList<Integer>  Path=multiAgents.agents[i].Path;
    pushMatrix();
    strokeWeight(2);
    for(int k=0;k<Path.size()-1;++k)
    {
      PVector P1=samples.get(Path.get(k));
      PVector P2=samples.get(Path.get(k+1));
      stroke((255+25*i)%256,(0+88*i)%256,(255+34*i)%256);
      line(P1.x,P1.y,Z,P2.x,P2.y,Z);
      
      //// show milestones
      //pushMatrix();
      //noStroke();
      //fill((186+25*i)%256,(85+88*i)%256,(211+34*i)%256);
      //translate(P1.x,P1.y,Z);
      //rotate(frameCount / -100.0);
      //star(0, 0, 10, 23, 6); 
      //popMatrix();
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
          fill(128,128,128) ;//(255, 0, 0)
        }
        else
        {
          fill(0) ;//(182, 155, 76 )
        }
      }  
      else {
        if (j%2 == 0) { 
          fill(0) ;//(182, 155, 76)
        }
        else
        {
           fill(128) ; //(255, 0, 0)
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
  for(int i=0;i<multiAgents.AgentNum;++i)
  {
    PVector agentP=multiAgents.agents[i].P;
    float R=multiAgents.agents[i].R*mag;
    float H=multiAgents.agents[i].H*mag;
    fill((130+25*i)%256, (82+55*i)%256, (1+88*i)%256);//wood brown
    noStroke();
    pushMatrix();    
    translate(agentP.x,agentP.y,multiAgents.agents[i].H/2*mag); //agentP.z
    drawCylinder( 30, R , H );
    popMatrix();
  }

  stroke(255,0,0);
  strokeWeight(5);
  for(int k=0;k<2;++k){
    if(bestline[k]!=null)
    {
      line(multiAgents.agents[1].P.x, multiAgents.agents[1].P.y, multiAgents.agents[1].H/2*mag,
      multiAgents.agents[1].P.x+5*mag*bestline[k].direction.x, multiAgents.agents[1].P.y+5*mag*bestline[k].direction.y, 
      multiAgents.agents[1].H/2*mag+5*mag*bestline[k].direction.z);
    }
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
  float threshold=.5*mag;
  if(PVector.sub(destination,agentP).mag()<threshold)
  { 
    return true;
  }
  else
  {
    return false;
  }
}

void UpdateAgentLinearly(float dt)
{
  RVO Local=new RVO();
  for(int i=0;i<multiAgents.AgentNum;++i)
  {
    Vector<PVector> samples= multiAgents.agents[i].samples;
    PVector agentP=multiAgents.agents[i].P;
    PVector CurTarget=samples.get(multiAgents.agents[i].CurtarId);
    PVector forward=new PVector();
    if(Arrival(CurTarget,agentP))
    {
      if(multiAgents.agents[i].targetItr.hasNext())
      {
        multiAgents.agents[i].CurtarId = multiAgents.agents[i].targetItr.next();
        CurTarget=samples.get(multiAgents.agents[i].CurtarId);
      }
      else
      {
        continue;
      }
    }
    forward=PVector.sub(CurTarget,agentP).normalize();
    forward.z=0;
    multiAgents.agents[i].forward.set(PVector.mult(forward,multiAgents.agents[i].Vel)); 
    multiAgents.boids.get(i).velocity=forward;
  }
  bestline=new Line[2];
  ArrayList<PVector> Vel=Local.LocalIntersection(multiAgents.agents,multiAgents.AgentNum);
  for(int i=0;i<multiAgents.AgentNum;++i)
  {
    if(!Arrival(multiAgents.agents[i].P,PVector.mult(multiAgents.agents[i].goal,mag)))
    {
      /*************************RVO******************************/
      if(Vel!=null){
      multiAgents.agents[i].P.add(PVector.mult(Vel.get(i),dt));
      }
      else{
        multiAgents.agents[i].P.add(PVector.mult( multiAgents.agents[i].forward,dt));
      }
      /*************************Boids*******************************/
      //multiAgents.boids.get(i).flock(multiAgents.boids);
      //multiAgents.boids.get(i).update();
    }
    
  }
  
}

/*Implement  path  smoothing*/
void UpdateAgentSmooth(float dt)
{
  for(int i=0;i<multiAgents.AgentNum;++i)
  {
    Vector<PVector> samples= multiAgents.agents[i].samples;
    PVector agentP=multiAgents.agents[i].P;
    PVector CurTarget=samples.get(multiAgents.agents[i].CurtarId);
    PVector NextTarget=samples.get(multiAgents.agents[i].NexttarId);
    PVector forward=new PVector();
    if(Arrival(CurTarget,agentP))
    {
      if(multiAgents.agents[i].targetItr.hasNext())
      {
        multiAgents.agents[i].CurtarId = multiAgents.agents[i].targetItr.next();
        CurTarget=samples.get(multiAgents.agents[i].CurtarId);
        multiAgents.agents[i].NexttarId = multiAgents.agents[i].targetItr.next();
        NextTarget=samples.get(multiAgents.agents[i].NexttarId);
      }
      else
      {
        continue;
      }
    }
    if(intersection(agentP,0,NextTarget,0,multiAgents.agents[i].space))
    {
      forward=PVector.sub(CurTarget,agentP).normalize();
    }
    else
    {
      forward=PVector.sub(NextTarget,agentP).normalize();
      multiAgents.agents[i].CurtarId = multiAgents.agents[i].NexttarId;
      if(multiAgents.agents[i].targetItr.hasNext())
      {
        multiAgents.agents[i].NexttarId = multiAgents.agents[i].targetItr.next();
      }
    }
    agentP.add(PVector.mult(forward,multiAgents.agents[i].Vel*dt));
  }
}
