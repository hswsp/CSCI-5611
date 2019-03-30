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
boolean CylinderBall(PVector x,PVector pball, float ballR)//Cylinder-ball configuration space
{
  float SphereR =(0.5+ballR)*mag;//agentR
  float distance=PVector.sub(x,pball).mag();
  return distance<SphereR;
}
boolean feasible(PVector x)
{//returns true if the vector x is in the feasible space
  for(int i=0;i<ObsNumber;++i)
  {
    if(CylinderBall(x,pball[i],ballR[i]))
    {
      return false;
    }
  }
  return true;
}

boolean intersection(PVector s,PVector g)//Is segment sg intersect with configration space
{
  float Maxiter=5E3;
  for(int i=0;i<=(int)Maxiter;++i)
  {
    float t=(float)i/Maxiter;
    PVector p=PVector.mult(g,t).add(PVector.mult(s,(1-t)));
    if(!feasible(p))
    {
      return true;
    }
  }
  return false;
}
