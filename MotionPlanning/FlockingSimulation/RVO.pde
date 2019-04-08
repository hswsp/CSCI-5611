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
  private Line VerticalLine(Agent A,Agent Obstacle)//
  {
    //set S to origin
    PVector O=Obstacle.P;
    PVector S=A.P;
    PVector PinLine=PVector.add(O,PVector.mult(PVector.sub(S,O).normalize(),(Obstacle.R+A.R)*mag+A.Vel*dt*3));
    Line line=new Line(O,PVector.sub(S,O).normalize());
    return line.Perpendicular(PVector.sub(PinLine,S));
  }
  private PVector ObstacleCones(Agent[] agents,int ID,int num, ArrayList<ArrayList<Double>> A, ArrayList<Double>b) // one agnet VS all
  {
    //output:A,b
    PointCircleTangent CircleT= new PointCircleTangent();
    PVector PA=agents[ID].P;
    float R=agents[ID].R; // current agent R
    PVector VA=agents[ID].forward;
    PVector minIntersaction=null;
    for(int j=0;j<num;++j)
    {
      if(j==ID)
       continue;
      Line RVOLine=new Line();
      float minLen=Float.MAX_VALUE;
      PVector PB=PVector.sub(agents[j].P,PA);//Velocity obstacle center
      PVector Reciprocal=PVector.mult(PVector.add(VA,agents[j].forward),0.5);
      ArrayList<PVector> points=CircleT.pointCircleTangentPoints(PB,(agents[j].R+R)*mag,new PVector(0,0,0));
      Line VAtoTan[] = new Line[2];
      for(int i=0;i<points.size();++i)// add tangent
      {
        PVector dir=points.get(i).normalize();// tangent line orientation
        if(dir.mag()==0)
        {
          dir.set(1,0,0);
        }
        Line tangent=new Line(Reciprocal,dir); // apex lies at (Va+Vb)/2     
        VAtoTan[i] = new Line(VA,new PVector(-tangent.direction.y,tangent.direction.x,0)); 
       
      }
      if(PVector.dot(VAtoTan[0].direction,VAtoTan[1].direction)>0) // outside cons
      {
        continue;
      }
      for(int i=0;i<points.size();++i)
      {
        PVector dir=points.get(i).normalize();// tangent line orientation
        if(dir.mag()==0)
        {
          dir.set(1,0,0);
        }
        Line tangent =new Line(Reciprocal,dir);
        PVector Intersection=tangent.Intersection(tangent,VAtoTan[i]);
        float distance=PVector.sub(Intersection,VA).mag();
        if(distance<minLen)
        {
          minLen=distance;
          RVOLine=tangent;
          minIntersaction=Intersection;
        }
      }
      //// add verticle constraints
      //Line Vline=VerticalLine(agents[ID],agents[j]); 
      //Vline.point.add(Reciprocal);
      //Line VAtoTan = new Line(VA,new PVector(-Vline.direction.y,Vline.direction.x,0));
      //PVector Intersection=Vline.Intersection(Vline,VAtoTan);
      //float distance=PVector.sub(Intersection,VA).mag();
      //if(distance<minLen)
      //{
      //  minLen=distance;
      //  RVOLine=Vline;
      //  minIntersaction=Intersection;
      //}
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
      A.add(lines);// store the outer normal of the line
      b.add(btemp);
    }
      
    /*check static obstacle*/
    for(int j=0;j<ObsNumber;++j)
    {
      Line RVOLine=new Line();
      float minLen=Float.MAX_VALUE;
      PVector PB=PVector.sub(pball[j],PA);//Velocity obstacle center
      PVector Reciprocal=new PVector(0,0,0);//PVector.mult(VA,0.5);
      ArrayList<PVector> points=CircleT.pointCircleTangentPoints(PB,(ballR[j]+R)*mag,new PVector(0,0,0));
      Line VAtoTan[] = new Line[2];
      for(int i=0;i<points.size();++i)// add tangent
      {
        PVector dir=points.get(i).normalize();// tangent line orientation
        if(dir.mag()==0)
        {
          dir.set(1,0,0);
        }
        Line tangent =new Line(Reciprocal,dir);   
        VAtoTan[i] = new Line(VA,new PVector(-tangent.direction.y,tangent.direction.x,0)); 
      }
      if(PVector.dot(VAtoTan[0].direction,VAtoTan[1].direction)>0) // outside cons
      {
        continue;
      }
      for(int i=0;i<points.size();++i)
      {
        PVector dir=points.get(i).normalize();// tangent line orientation
        if(dir.mag()==0)
        {
          dir.set(1,0,0);
        }
        Line tangent =new Line(Reciprocal,dir);   
        PVector Intersection=tangent.Intersection(tangent,VAtoTan[i]);
        float distance=PVector.sub(Intersection,VA).mag();
        if(distance<minLen)
        {
          minLen=distance;
          RVOLine=tangent;
          minIntersaction=Intersection;
          //bestline=RVOLine;
        }
      }
      //add the optimal line to LP 
      ArrayList<Double> lines = new  ArrayList<Double>();
      //println("bestLine is ",bestline.point,bestline.direction);
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
      if(lines.get(0)*(PB.x+Reciprocal.x)+lines.get(1)*(PB.y+Reciprocal.y)-btemp<0)
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
      if(minIntersaction==null)//no constraints, add origin
      {
        Vel.add(VA);
        continue;
      }
      //initial A,b
      double[][] A=new double [At.size()][2];
      double [] b=new double [bt.size()];
      double []InitialG=new double[] {minIntersaction.x,minIntersaction.y};
      for(int k=0;k<At.size();++k)
      {
       for(int j=0;j<At.get(k).size();++j)
         {
           A[k][j]=At.get(k).get(j);
           InitialG[j]-=mag*A[k][j];
         }
       b[k]=bt.get(k);
      }
      
      QP sol=new QP();
      //println("VA is ",VA);
      sol.InitQ(new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } },new double[] { -2*VA.x, -2*VA.y },VA.x*VA.x+VA.y*VA.y);
      sol.InitLI(A,b);
      sol.solver(InitialG);//new double[] {agents[j].forward.x-mag*A[0][0], agents[j].forward.y-mag*A[0][1]} 
      Vel.add(new PVector((float)sol.xmin[0],(float)sol.xmin[1],0));
      //println("the best solution for" ,i, "is ",sol.xmin[0],sol.xmin[1]);
    }
    return Vel;
  }
}
