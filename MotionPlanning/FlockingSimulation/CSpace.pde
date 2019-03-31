import java.lang.reflect.Method;
void InitialObstacles(PVector pball[],float ballR[])
 {
   for(int i=0;i<ObsNumber;++i)
   {
     ballR[i]=2;
     pball[i]=new PVector(0,0,0).mult(mag);
   }
   pball[0]=new PVector(0,0,0).mult(mag);
   pball[1]=new PVector(4,4,0).mult(mag);
   pball[2]=new PVector(-4,4,0).mult(mag);
   pball[3]=new PVector(-4,-4,0).mult(mag);
   pball[4]=new PVector(4,-4,0).mult(mag);
 }

interface CSpace
{
  public boolean inCspace(PVector x,PVector pball, float ballR);
}
class CylinderBall implements CSpace //Cylinder-ball configuration space
{
  float R;
  CylinderBall(float CylinderR)
  {
    this.R=CylinderR;
  }
  public boolean inCspace(PVector x,PVector pball, float ballR)
  {
    float SphereR =(R+ballR)*mag;
    float distance=PVector.sub(x,pball).mag();
    return distance<SphereR;
  }
 };

//boolean CylinderBall(PVector x,PVector pball, float ballR,float agentR)
//{
//  float SphereR =(agentR+ballR)*mag;
//  float distance=PVector.sub(x,pball).mag();
//  return distance<SphereR;
//}
boolean feasible(PVector x,CSpace cspace)
{//returns true if the vector x is in the feasible space
  for(int i=0;i<ObsNumber;++i)
  {
    if(cspace.inCspace(x,pball[i],ballR[i]))
    {
      return false;
    }
  } 
  return true;
}

boolean intersection(PVector s,PVector g,CSpace cspace)//Is segment sg intersect with configration space
{
  float Maxiter=5E3;
  for(int i=0;i<=(int)Maxiter;++i)
  {
    float t=(float)i/Maxiter;
    PVector p=PVector.mult(g,t).add(PVector.mult(s,(1-t)));
    if(!feasible(p,cspace))
    {
      return true;
    }
  }
  return false;
}
