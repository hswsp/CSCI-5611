 boolean Facecloth(Cloth cloth)
{
  //get all camera relative vector
  PVector campos=cam.position;
  PVector camforward=cam.getForward();
  PVector camright=cam.getRight();
  PVector camup=new PVector();
  camright.cross(camforward,camup);
  
  // cloth plane
  PVector clothNorm=new PVector(1,0,0);
  PVector P0=new PVector(widthOffset,heightOffset+cloth.width/2*cloth.l0,ZOffset - cloth.height/2* cloth.l0);
  PVector dis=PVector.sub(campos,P0);
  float camP0distance=PVector.dot(dis,clothNorm);
  float cosangle=camP0distance/dis.mag();
  //range of steady state of cloth
  float clothz1,clothz2,clothy1,clothy2;
  float offset=50;
  float disthreshold=100;
  clothy1=heightOffset -offset;
  clothy2=heightOffset +cloth.width*cloth.l0  +offset;
  clothz1=ZOffset - cloth.height* cloth.l0 -offset;
  clothz2=ZOffset +offset;
  
  if(clothy1<campos.y&&campos.y<clothy2&&clothz1<campos.z&&campos.z<clothz2&& camP0distance<=disthreshold && cosangle>0.8)
  {
    return true;
  }
  else
  {
    return false;
  }
}
