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
  cam.speed = 2;              // default is 3
  cam.sensitivity = 0.5;      // default is 2
  cam.controllable = true;
  cam.position = new PVector(0,0,850);//
  cam.pan = -PI/2;
  perspective(PI/3, (float)width/height, 0.01, 10000);
  InitialObstacles(pball,ballR);
  InitAgents();
  //QP sol=new QP();
  //double [][] A=new double[][] { { -1.0, 0.0 },{0.0,-1.0} }; // constraint x > 1,y>1
  //double []b= new double[] { -1.0,-1.0};
  //sol.InitQ(new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } },new double[] { -2.0, -2.0 },0.0);
  //sol.InitLI(A,b);
  //sol.solver(new double[] { 2.0, 2.0 } );
  //println("xmin",sol.xmin[0],sol.xmin[1]);
  //QuadraticFunction q = new QuadraticFunction(
  //  new double[][] { { 1.0, 0.0 }, { 0.0, 1.0 } },
  //  new double[] { 0.0, 0.0 },
  //  0.0);
  //// optimize function q with an inequality constraint and an equality constraint,
  //// using the barrier method
  //BarrierOptimizer barrier = new BarrierOptimizer();
  //PointValuePair pvp = barrier.optimize(
  //    new ObjectiveFunction(q),
  //    new LinearInequalityConstraint(
  //        new double[][] { { -1.0, 0.0 },{0.0,-1.0} }, // constraint x > 1,y>1
  //        new double[] { -1.0,-1.0}),
  //    //new LinearEqualityConstraint(
  //        //new double[][] { { 1.0, 0.0 } },  // constraint y = 1,
  //        //new double[] { 1.0 }),
  //        null,
  //    new InitialGuess(new double[] { 2.0, 2.0 }));
  
  //double[] xmin = pvp.getFirst();  // { 1.0, 1.0 }
  //double vmin = pvp.getSecond();   // 1.0
  //println("xmin",sol.xmin[0],sol.xmin[1]);

}
void InitAgents()
{
  multiAgents=new MultiAgents("RRT",4);
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
  //float dt=0.01;
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
  //if(bestline!=null)
  //{
  //  stroke(126);
  //  strokeWeight(5);
  //  line(multiAgents.agents[1].P.x, multiAgents.agents[1].P.y, multiAgents.agents[1].H/2*mag,
  //  multiAgents.agents[1].P.x+5*mag*bestline.direction.x, multiAgents.agents[1].P.y+5*mag*bestline.direction.y, 
  //  multiAgents.agents[1].H/2*mag+5*mag*bestline.direction.z);
  //}
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
  float threshold=.1*mag;
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
