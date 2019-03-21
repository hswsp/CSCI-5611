boolean CylinderBall(PVector x)
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
