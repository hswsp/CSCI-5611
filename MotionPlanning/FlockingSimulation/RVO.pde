class RVO
{
  private  boolean IsLocalIntersection(Agent[] agents,int ID,int num)
  {
    Agent A=agents[ID];
    for(int i=0;i<num;++i)
    {
      if(i==ID)
       continue;
      Agent B=agents[i];
      float SphereR =(A.R+B.R)*mag+A.Vel*dt*3;
      float distance=PVector.sub(A.P,B.P).mag();
      if(distance<=SphereR)
      {
        return true;
      }
    }
     return false;
  }
  private Line VerticalLine(PVector SO, float ObstacleR, float AR, float AVel)//Agent A,Agent Obstacle
  {
    /*set S to origin
    SO=S-O
    */
    PVector PinLine=PVector.add(SO,PVector.mult(SO.normalize(),-((ObstacleR+AR)*mag+AVel*dt)));
    Line line=new Line(SO,PVector.mult(SO,-1).normalize());
    return line.Perpendicular(PinLine);
  }
  private PVector Findclosed(PVector PB, PVector VA, PVector VB, float Ra, float Rb,ArrayList<ArrayList<Double>> A, ArrayList<Double>b)
  {
    PointCircleTangent CircleT= new PointCircleTangent();
    PVector minIntersaction=null;
    Line RVOLine=new Line();
    float minLen=Float.MAX_VALUE;
    PVector Reciprocal=PVector.add(PVector.mult(VA,0.5),PVector.mult(VB,0.5));
    // distance constraints
    if(PB.mag()>(Ra+Rb)*mag+VA.mag()*dt)
      return null;
    ArrayList<PVector> points=CircleT.pointCircleTangentPoints(PB,(Ra+Rb)*mag,new PVector(0,0,0));
    if(points.size()==2) 
    { 
      PVector rel=PVector.sub(VA,Reciprocal);
      PVector relcrossTan=new PVector();
      PVector relcrossTan1=new PVector();
      if(PVector.cross(points.get(0),rel,relcrossTan).dot(PVector.cross(points.get(1),rel,relcrossTan1))>0){
       return null; //outside cons, return null
      }
    }
    Line []VAtoTan = new Line[2];
    for(int i=0;i<points.size();++i)// add tangent
    {
      PVector dir=new PVector(points.get(i).x,points.get(i).y,points.get(i).z).normalize();// tangent line orientation
      if(dir.mag()==0)
      {
        dir.set(1,0,0);
      }
      Line tangent=new Line(Reciprocal,dir); // apex lies at (Va+Vb)/2
      PVector norm=PVector.sub(points.get(i),PB).normalize() ; //outer norm 
      VAtoTan[i] = new Line(VA,norm); 
      PVector Intersection=tangent.Intersection(tangent,VAtoTan[i]);
      float distance =PVector.dist(Intersection,VA);
      if(abs(distance-minLen)<1E-3) // the same for both tangent
      {
        PVector org=new PVector();
        PVector.cross(tangent.direction,PB,org);
        if(org.dot(new PVector(0,0,1))>0)
        {
          minLen=distance;
          RVOLine=tangent;
          minIntersaction=Intersection;
        }
      }
      else if(distance<minLen)
      {
        minLen=distance;
        RVOLine=tangent;
        minIntersaction=Intersection;
      }    
    }
 
    // add verticle constraints
    Line Vline=VerticalLine(PB,Rb, Ra, VA.mag()); 
    Vline.point.add(Reciprocal);
    Line VAtoVl = new Line(VA,new PVector(-Vline.direction.y,Vline.direction.x,0));
    PVector Intersection=Vline.Intersection(Vline,VAtoVl);
    float Vdistance=PVector.dist(Intersection,VA);
    if(Vdistance<minLen)
    {
      minLen=Vdistance;
      RVOLine=Vline;
      minIntersaction=Intersection;
    }
    
    /******************add the optimal line to LP ***************/
    if(minIntersaction!=null)
    {
      ArrayList<Double> lines = new  ArrayList<Double>();
      lines.add((double)RVOLine.direction.y);
      lines.add(-1*((double)RVOLine.direction.x));
      double btemp=(double)RVOLine.direction.y*RVOLine.point.x-(double)RVOLine.direction.x*RVOLine.point.y;
      if(lines.get(0)*(PB.x+Reciprocal.x)+lines.get(1)*(PB.y+Reciprocal.y)-btemp<0) // the center of b should be outside the constraints
      {
        for(int l=0;l<lines.size();++l)
        {
           lines.set(l,-1*lines.get(l));
        }
        btemp=-btemp;
      }
      A.add(lines);// store the outer normal of the line
      b.add(btemp);
    }
    return minIntersaction;
  }
  private PVector ObstacleCones(Agent[] agents,int ID,int num, ArrayList<ArrayList<Double>> A, ArrayList<Double>b) // one agnet VS all
  {
    //output:A,b
    PointCircleTangent CircleT= new PointCircleTangent();
    PVector PA=agents[ID].P;
    float R=agents[ID].R; // current agent R
    PVector VA=agents[ID].forward;
    float minLen=Float.MAX_VALUE;
    PVector minIntersaction=null;
    int n=0;
    for(int j=0;j<num;++j)
    {
      if(j==ID)
       continue;
      PVector Intersaction=Findclosed(PVector.sub(agents[j].P,PA), VA, agents[j].forward, R, agents[j].R,A,b);
      if(Intersaction!=null&&PVector.dist(Intersaction,VA)<minLen)
      {
        minIntersaction=Intersaction;
        //bestline[n]=new Line();
        //bestline[n].direction=new PVector((float)(double)(A.get(n).get(1)),(float)-A.get(n).get(0),0);
        //n++;
      }
    //<>//
    }
    /*check static obstacle*/
    for(int j=0;j<ObsNumber;++j)
    {
      Line RVOLine=new Line();
      minLen=Float.MAX_VALUE;
      PVector PB=PVector.sub(pball[j],PA);//Velocity obstacle center
      PVector Reciprocal=new PVector(0,0,0);//PVector.mult(VA,0.5);
      // distance constraints
      if(PB.mag()>(ballR[j]+R)*mag+VA.mag()*dt*3)
         continue;
         
      ArrayList<PVector> points=CircleT.pointCircleTangentPoints(PB,(ballR[j]+R)*mag,new PVector(0,0,0));
      Line VAtoTan[] = new Line[2];
      if(points.size()==2) 
      { 
        PVector rel=PVector.sub(VA,Reciprocal);
        PVector relcrossTan=new PVector();
        PVector relcrossTan1=new PVector();
        if(PVector.cross(points.get(0),rel,relcrossTan).dot(PVector.cross(points.get(1),rel,relcrossTan1))>0){
         continue; //outside cons, return null
        }
      }
      for(int i=0;i<points.size();++i)// add tangent
      {
        PVector dir=new PVector(points.get(i).x,points.get(i).y,points.get(i).z).normalize();// tangent line orientation
        if(dir.mag()==0)
        {
          dir.set(1,0,0);
        }
        Line tangent =new Line(Reciprocal,dir);   
        PVector norm=PVector.sub(points.get(i),PB).normalize() ; //outer norm 
        VAtoTan[i] = new Line(VA,norm); 
        PVector Intersection=tangent.Intersection(tangent,VAtoTan[i]);
        float distance =PVector.dist(Intersection,VA);
        if(abs(distance-minLen)<1E-3) // the same for both tangent
        {
          PVector org=new PVector();
          PVector.cross(tangent.direction,PB,org);
          //println(tangent.direction,PB,org);
          if(org.dot(new PVector(0,0,1))>0)
          {
            minLen=distance;
            RVOLine=tangent;
            minIntersaction=Intersection;
          }
        }
        else if(distance<minLen)
        {
          minLen=distance;
          RVOLine=tangent;
          minIntersaction=Intersection;
        }
      }
      // add verticle constraints
      Line Vline=VerticalLine(PB,ballR[j], R, VA.mag()); 
      Vline.point.add(Reciprocal);
      Line VAtoVl = new Line(VA,new PVector(-Vline.direction.y,Vline.direction.x,0));
      PVector Intersection=Vline.Intersection(Vline,VAtoVl);
      float Vdistance=PVector.dist(Intersection,VA);
      if(Vdistance<minLen)
      {
        minLen=Vdistance;
        RVOLine=Vline;
        minIntersaction=Intersection;
      }
      //add the optimal line to LP 
      ArrayList<Double> lines = new  ArrayList<Double>();
      lines.add((double)RVOLine.direction.y);
      lines.add(-1*((double)RVOLine.direction.x));
      double btemp=(double)RVOLine.direction.y*RVOLine.point.x-(double)RVOLine.direction.x*RVOLine.point.y;
      if(lines.get(0)*(PB.x+Reciprocal.x)+lines.get(1)*(PB.y+Reciprocal.y)-btemp<0) // the center of b should be outside the constraints
      {
        for(int l=0;l<lines.size();++l)
        {
           lines.set(l,-1*lines.get(l));
        }
        btemp=-btemp;
      }
      if(lines.get(0)*VA.x+lines.get(1)*VA.y-btemp>0)
      {
        A.add(lines);// store the outer normal of the line
        b.add(btemp);
      }
    }
      return minIntersaction;
  }
  
  public ArrayList<PVector> LocalIntersection(Agent[] agents,int num)
  {
    ArrayList<PVector> Vel=new ArrayList<PVector>();
    for(int i=0;i<num; ++i){
      PVector VA=agents[i].forward;
      if(!IsLocalIntersection(agents,i,num)&& feasible(agents[i].P,0, new CylinderBall(agents[i].R+agents[i].Vel/mag*dt*3)))
      {
        Vel.add(VA);
        continue;
      }
      ArrayList<ArrayList<Double>> At=new ArrayList<ArrayList<Double>>();
      ArrayList<Double>bt=new ArrayList<Double>();
      PVector minIntersaction =ObstacleCones(agents,i,num,At,bt);
      if(At.size()==0)//no constraints, add origin
      {
        Vel.add(VA);
        continue;
      }
      //initial A,b
      double[][] A=new double [At.size()][2];
      double [] b=new double [bt.size()];
      for(int k=0;k<At.size();++k)
      {
       for(int j=0;j<At.get(k).size();++j)
         {
           A[k][j]=At.get(k).get(j);
         }
       b[k]=bt.get(k);
      }
      QP sol=new QP();
      sol.InitQ(new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } },new double[] { -2*VA.x, -2*VA.y },VA.x*VA.x+VA.y*VA.y);
      sol.InitLI(A,b);
      sol.solver();
      //if(!sol.solver()){
      //  Vel.add(PVector.mult(VA,-1));
      //  continue;
      //}
      Vel.add(new PVector((float)sol.xmin[0],(float)sol.xmin[1],0));
    }
    return Vel;
  }
}
