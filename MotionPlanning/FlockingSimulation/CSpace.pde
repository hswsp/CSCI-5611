import java.lang.reflect.Method;
void InitialObstacles(PVector pball[],float ballR[])
 {
   for(int i=0;i<ObsNumber;++i)
   {
     ballR[i]=1;
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
  public boolean inCspace(PVector x,float t,PVector[] pball, float[] ballR);
}
class CylinderBall implements CSpace //Cylinder-ball configuration space
{
  float R;
  CylinderBall(){}
  CylinderBall(float r)
  {
    this.R=r;
  }
  public boolean inCspace(PVector x,float t,PVector[] pball, float[] ballR)
  {
    for(int i=0;i<ObsNumber;++i)
    {
      float SphereR =(R+ballR[i])*mag;
      float distance=PVector.sub(x,pball[i]).mag();
      if(distance<SphereR)
      {
        return false;
      }
    }
    return true;
  }
 };

boolean feasible(PVector x,float t,CSpace cspace)
{
  //returns true if the vector x is in the feasible space
  if(cspace.inCspace(x,t,pball,ballR))
  {
    return true;
  }
  return false;
}

boolean intersection(PVector s,float t0,PVector g,float t1,CSpace cspace)//Is segment sg intersect with configration space
{
  float Maxiter=5E3;
  for(int i=0;i<=(int)Maxiter;++i)
  {
    float t=(float)i/Maxiter;
    PVector p=PVector.mult(g,t).add(PVector.mult(s,(1-t)));
    float time=t1*t+t0*(1-t);
    if(!feasible(p,time,cspace))
    {
      return true;
    }
  }
  return false;
}
