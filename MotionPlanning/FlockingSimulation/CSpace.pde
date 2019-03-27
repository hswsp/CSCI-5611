boolean CylinderBall(PVector x)//Cylinder-ball configuration space
{
  float SphereR =(agentR+ballR)*mag;
  float distance=PVector.sub(x,pball).mag();
  return distance<SphereR;
}
boolean feasible(PVector x)
{//returns true if the vector x is in the feasible space
    if(CylinderBall(x))
    {
      return false;
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
