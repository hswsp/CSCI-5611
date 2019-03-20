
class Box extends Shape
{
  public float width;//x length
  public float height; //y length
  public float zlen;//z lengh
  Box(float w, float h, float c,float m)
  {
    super(m);
    this.width=w;
    this.height=h;
    this.zlen = c;
    double[][]Inertia=
    {
      {m * (h * h+ zlen * zlen) / 12,0,0},
      {0,m * (zlen * zlen + w * w) / 12,0},
      {0,0,m * (w * w + h * h) / 12}
    };
    ComputeRotInertia(new Matrix(Inertia));
  }
}

class RigidBox extends RigidBody
{
  Box shape;
  float e_floor=0.2;//coeff of collission
  RigidBox(float w, float h,  float c, float m)
  {
    super(new Box(w,  h,  c, m));
    this.shape=(Box)super.shape;
  }
 public Matrix get_E()
 {
   return super.shape.E;
 }
 private PVector GeneralCollision(PVector n, PVector ra,PVector Vc, PVector Vn, PVector rb,Shape B, PVector Lb, PVector La)
 {
   /*
   n: normal;
   ra, rb: two box's r vector
   Vc: current box barycenter velocity buffer
   Vn: velocity of impact of other box
   return value:
   Lb: change of Geometry B
   La: change of this Geometry
   return: manipulation of velocities
   */
   //compute Vrel
   PVector Va=new PVector();
   this.omega.cross(ra,Va);
   Va.add(Vc);
   float Vrel= PVector.dot(PVector.sub(Va,Vn),n);
   Vrel= Vrel*(1+e_floor);
   
   //change angleMomentum
   PVector RacrossN=new PVector();
   PVector RbcrossN=new PVector();
   ra.cross(n,RacrossN);
   rb.cross(n,RbcrossN);
   PVector tempa=new PVector();
   PVector tempb=new PVector();
   tempa=Mat2Vector(super.shape.RotInertia.times(Vector2Mat(RacrossN)));
   tempb=Mat2Vector(B.RotInertia.times(Vector2Mat(RbcrossN)));
   float j= Vrel/(1/shape.mass+1/B.mass+PVector.dot(n,tempa.cross(ra))+PVector.dot(n,tempb.cross(rb)));
   PVector impulse=PVector.mult(n,j);
   PVector dL=new PVector();
   PVector dLb=new PVector();
   ra.cross(impulse,dL);
   La.add(dL);
   rb.cross(PVector.mult(impulse,-1),dLb);
   Lb.add(dLb);
   return impulse;
 }
 
 private void CollisionStateChange(PVector n,PVector vn,PVector newpos,PVector TotalAngmomentumChange)
 {
   /*
   n: normal of face
   vn: relative v of collision point
   newpos: collision point
   TotalAngmomentumChange : change of angluar momentum
   */
  //linear
   PVector bounce=PVector.mult(n,PVector.dot(vn,n));
   PVector dv=PVector.mult(bounce,1+e_floor);
   this.pVel.sub(dv);
   this.pos.set(newpos);
   //angle
   this.L.add(TotalAngmomentumChange);
 }
 void Get8vertices(PVector []p)
 {
   PVector a1=new PVector();
   PVector a2=new PVector();
   PVector a3=new PVector();
   super.GetCord(a1,a2,a3);
   p[0]=PVector.add(this.pos,PVector.add(PVector.add(PVector.mult(a1,this.shape.width/2),PVector.mult(a2,-1*this.shape.height/2)),
   PVector.mult(a3,1*this.shape.zlen/2)));
   p[1]=PVector.add(this.pos,PVector.add(PVector.add(PVector.mult(a1,this.shape.width/2),PVector.mult(a2,1*this.shape.height/2)),
   PVector.mult(a3,1*this.shape.zlen/2)));
   p[2]=PVector.add(this.pos,PVector.add(PVector.add(PVector.mult(a1,-1*this.shape.width/2),PVector.mult(a2,1*this.shape.height/2)),
   PVector.mult(a3,1*this.shape.zlen/2)));
   p[3]=PVector.add(this.pos,PVector.add(PVector.add(PVector.mult(a1,-1*this.shape.width/2),PVector.mult(a2,-1*this.shape.height/2)),
   PVector.mult(a3,1*this.shape.zlen/2)));
   p[4]=PVector.add(this.pos,PVector.add(PVector.add(PVector.mult(a1,this.shape.width/2),PVector.mult(a2,-1*this.shape.height/2)),
   PVector.mult(a3,-1*this.shape.zlen/2)));
   p[5]=PVector.add(this.pos,PVector.add(PVector.add(PVector.mult(a1,this.shape.width/2),PVector.mult(a2,1*this.shape.height/2)),
   PVector.mult(a3,-1*this.shape.zlen/2)));
   p[6]=PVector.add(this.pos,PVector.add(PVector.add(PVector.mult(a1,-1*this.shape.width/2),PVector.mult(a2,1*this.shape.height/2)),
   PVector.mult(a3,-1*this.shape.zlen/2)));
   p[7]=PVector.add(this.pos,PVector.add(PVector.add(PVector.mult(a1,-1*this.shape.width/2),PVector.mult(a2,-1*this.shape.height/2)),
   PVector.mult(a3,-1*this.shape.zlen/2)));
 }
 
 private PVector PointFaceCollision(PVector n,PVector r,PVector vn) 
 {
   //n: face normal facing box, r: barycenter to collision point, vn: v of barycenter
   
   //compute Vrel
   PVector Vr=new PVector();
   this.omega.cross(r,Vr);
   PVector vp=PVector.add(vn,Vr);
   PVector bounce=PVector.mult(n,PVector.dot(vp,n));
   PVector dv=PVector.mult(bounce,1+e_floor);
   
   //change angleMomentum
   PVector RcrossN=new PVector();
   r.cross(n,RcrossN);
   PVector temp=new PVector();
   temp=Mat2Vector(super.shape.RotInertia.times(Vector2Mat(RcrossN)));
   float j=dv.mag()/(1/shape.mass+PVector.dot(n,temp.cross(r)));
   PVector impulse=PVector.mult(n,j);
   PVector dL=new PVector();
   r.cross(impulse,dL);
   return dL;
 }
 
 public void BoxWallCollision()
 {
   PVector []p=new PVector[8];//Position of 8 Vertices
   for(int i=0;i<8;++i)
   {
     p[i]=new PVector();
   } 
   Get8vertices(p);
   //vector buffer
   PVector vn=new PVector();
   vn.set(this.pVel);
   
   PVector TotalAngmomentumChange=new PVector(0,0,0);
   float d=0;
   int downNum=0;
   int upNum=0;
   int leftNum=0;
   int rightNum=0;
   int forwardNum=0;
   int backNum=0;
   float floorz=-floor;
   for(int i=0;i<8;++i)
   {
     if(p[i].z<=floorz)
     {
       PVector n=new PVector(0,0,1);
       float dl=floorz-p[i].z;
       if(Math.abs(dl)>Math.abs(d))
       {
         d=dl;   //find the max
       }
       PVector R=PVector.sub(p[i],this.pos);
       TotalAngmomentumChange.add(PointFaceCollision(n,R,vn));
       ++downNum;
     }
     else if(p[i].z>=-up)
     {
       float dl=-up-p[i].z;
       if(Math.abs(dl)>Math.abs(d))
       {
         d=dl;   //find the max
       }
       PVector R=PVector.sub(p[i],this.pos);
       PVector n=new PVector(0,0,-1);
       TotalAngmomentumChange.add(PointFaceCollision(n,R,vn));
       ++upNum;
     }
     else if(p[i].x<=left)
     {
       float dl=left-p[i].x;
       if(Math.abs(dl)>Math.abs(d))
       {
         d=dl;   //find the max
       }
       PVector R=PVector.sub(p[i],this.pos);
       PVector n=new PVector(1,0,0);
       TotalAngmomentumChange.add(PointFaceCollision(n,R,vn));
       ++leftNum;
     }
     else if(p[i].x>=right)
     {
       
       float dl=right-p[i].x;
       if(Math.abs(dl)>Math.abs(d))
       {
         d=dl;   //find the max
       }
       PVector R=PVector.sub(p[i],this.pos);
       PVector n=new PVector(-1,0,0);
       TotalAngmomentumChange.add(PointFaceCollision(n,R,vn));
       ++rightNum;
     }
     else if(p[i].y>=-forward)
     {
       
       float dl=-forward-p[i].y;
       if(Math.abs(dl)>Math.abs(d))
       {
         d=dl;   //find the max
       }
       PVector R=PVector.sub(p[i],this.pos);
       PVector n=new PVector(0,-1,0);
       TotalAngmomentumChange.add(PointFaceCollision(n,R,vn));
       ++forwardNum;
     }
     else if(p[i].y<=-back)
     {
       
       float dl=-back-p[i].y;
       if(Math.abs(dl)>Math.abs(d))
       {
         d=dl;   //find the max
       }
       PVector R=PVector.sub(p[i],this.pos);
       PVector n=new PVector(0,1,0);
       TotalAngmomentumChange.add(PointFaceCollision(n,R,vn));
       ++backNum;
     }
   }
   if(downNum>0)
   {
     PVector n=new PVector(0,0,1);
     CollisionStateChange(n,vn,this.pos.add(new PVector(0,0,d)),PVector.div(TotalAngmomentumChange,downNum));
   }
   else if(upNum>0)
   {
     //linear
     PVector n=new PVector(0,0,-1);
     CollisionStateChange(n,vn,this.pos.add(new PVector(0,0,d)),PVector.div(TotalAngmomentumChange,upNum));
   }
    else if(leftNum>0)
   {
     //linear
     PVector n=new PVector(1,0,0);
     CollisionStateChange(n,vn,this.pos.add(new PVector(d,0,0)),PVector.div(TotalAngmomentumChange,leftNum));
   }
   else if(rightNum>0)
   {
     //linear
     PVector n=new PVector(-1,0,0);
     CollisionStateChange(n,vn,this.pos.add(new PVector(d,0,0)),PVector.div(TotalAngmomentumChange,rightNum));
   }
   else if(forwardNum>0)
   {
     //linear
     PVector n=new PVector(0,-1,0);
     CollisionStateChange(n,vn,this.pos.add(new PVector(0,d,0)),PVector.div(TotalAngmomentumChange,forwardNum));
   }
   else if(backNum>0)
   {
     //linear
     PVector n=new PVector(0,1,0);
     CollisionStateChange(n,vn,this.pos.add(new PVector(0,d,0)),PVector.div(TotalAngmomentumChange,backNum));
   }
 }
 public void BoxbetweenCollision(RigidBox box,float dt)
 {
   //PVector []p=new PVector[8];//quadrant :1,2,3,4,5,6,7,8
   //PVector []pbox= new PVector[8];
   //for(int i=0;i<8;++i)
   //{
   //  p[i]=new PVector();
   //  pbox[i]=new PVector();
   //} 
   //Get8vertices(p);
   //PVector a1=new PVector();
   //PVector a2=new PVector();
   //PVector a3=new PVector();
   //GetCord(a1,a2,a3);
   //testball.set(PVector.add(this.pos,PVector.mult(a1,this.shape.width/2+30)));
   //teste1.set(PVector.add(this.pos,PVector.mult(a2,this.shape.height/2+30)));
   //teste2.set(PVector.add(this.pos,PVector.mult(a3,this.shape.zlen/2+30)));
   
   if(PVector.sub(this.pos,box.pos).mag()<=Math.sqrt(this.shape.width*this.shape.width+this.shape.height*this.shape.height+this.shape.zlen*this.shape.zlen)/2
   +Math.sqrt(box.shape.width*box.shape.width+box.shape.height*box.shape.height+box.shape.zlen*box.shape.zlen)/2)//sphere boundary
   {
     PVector []p=new PVector[8];//quadrant :1,2,3,4,5,6,7,8
     PVector []pbox= new PVector[8];
     for(int i=0;i<8;++i)
     {
       p[i]=new PVector();
       pbox[i]=new PVector();
     } 
     Get8vertices(p);
     
     //vector buffer
     PVector Vc=new PVector();
     Vc.set(this.pVel);
     PVector TotalAngmomentumChange=new PVector(0,0,0);
     PVector BoxAngmomentumChange=new PVector(0,0,0);
     PVector TotalVelchange=new PVector(0,0,0);
     PVector DeletXcm=new PVector(0,0,0);
     int NumofCollision =0;
     for(PVector p0:p)//PVector p0:p
     {      
       if(PointInBox(p0,box))//vertex in other box
       {    
         //compute Vp0 of this box
         PVector V0=new PVector();
         PVector r = PVector.sub(p0,this.pos);
         this.omega.cross(r,V0);
         V0.add(Vc);
         PVector V1=new PVector();
         PVector r1 = PVector.sub(p0,box.pos);
         box.omega.cross(r1,V1);
         V1.add(box.pVel);
         PVector V=PVector.sub(V1,V0).normalize();
         PVector b1=new PVector();
         PVector b2=new PVector();
         PVector b3=new PVector();
         box.GetCord(b1,b2,b3);
         box.Get8vertices(pbox);
         float t=1E8;
         float temp_t;
         int NIndex=-1;
         PVector[]norm=new PVector[6];
         PVector[]Pf=new PVector[6];
         for(int i=0;i<6;++i)
         {
           norm[i]=new PVector();
           Pf[i]=new PVector();
         }
         norm[0].set(PVector.mult(b1,1));//right face
         norm[1].set(PVector.mult(b2,-1));//back
         norm[2].set(PVector.mult(b3,1));//up
         norm[3].set(PVector.mult(b2,1));//foward
         norm[4].set(PVector.mult(b1,-1)); //left
         norm[5].set(PVector.mult(b3,-1));//down
         for(int i=0;i<3;++i)
         {
           Pf[i].set(pbox[0]);//right face- up
         }
         for(int i=3;i<6;++i)
         {
           Pf[i].set(pbox[2]); //foward - down
         }
         for(int i=0;i<6;++i)
         {
           if((temp_t=RayPlaneIntersect( norm[i],Pf[i],p0,V))>0&&temp_t<t)
           {
             t=temp_t;
             NIndex=i;
           }
         }
         if(t<1E8)//have found the collision point
         {
           NumofCollision+=1;
           PVector Bp=GetPointOnLine(p0,V,t);
           PVector rb = PVector.sub(Bp,box.pos);
           //compute Vn of that box
           PVector Vn=new PVector();
           box.omega.cross(rb,Vn);
           Vn.add(box.pVel);
           //compute the change of v and angular momentum
           PVector J=GeneralCollision(norm[NIndex], r,Vc, Vn, rb, box.shape,BoxAngmomentumChange,TotalAngmomentumChange);
           TotalVelchange.add(PVector.div(J,this.shape.mass));
           DeletXcm.add(PVector.sub(Bp,r));
         }
       } 
     }
     if(NumofCollision>0)
     {
       //linear
       float damp=100;
       this.pVel.add(PVector.div(TotalVelchange,NumofCollision*damp));
       box.pVel.add(PVector.div(TotalVelchange,-1*(box.shape.mass/this.shape.mass)*NumofCollision*damp));
       this.pos.add(PVector.div(PVector.sub(PVector.div(DeletXcm,NumofCollision),this.pos),2));
       //angle
       this.L.add(PVector.div(TotalAngmomentumChange,NumofCollision*damp));
       box.L.add(PVector.div(BoxAngmomentumChange,NumofCollision*damp));
     }
   }
   
 }
 
 private boolean PointInBox(PVector P, RigidBox box)
 {
   
   PVector PinBoxCord=Word2View(PVector.sub(P,box.pos), box.get_E());
   if(Math.abs(PinBoxCord.x)<= box.shape.width/2 && Math.abs(PinBoxCord.y)<= box.shape.height/2 && Math.abs(PinBoxCord.z)<= box.shape.zlen/2 )
   { 
     
     return true;
   }
   else
   {
     return false;
   }
 }
}

float RayPlaneIntersect(PVector N,PVector C0, PVector P0,  PVector V)
{
    /*
    N is the normal
    V is the direction
    C0 : one point in plan
    all should be normalized
    */
    return -(PVector.dot(P0,N)-PVector.dot(C0,N))/PVector.dot(V,N);
}
PVector GetPointOnLine(PVector P0, PVector V, float t)
{
  return PVector.add(P0,PVector.mult(V,t));
}

// Linear transform
/*
(e1',e2',e3')=(e1,e2,e3)ViewMatrix
-->
(x',y',z')T=ViewMatrix-1*(x,y,z)T
*/
PVector Word2View(PVector Word, Matrix ViewMatrix)
{
  double [][]wt=
    {{Word.x},
    {Word.y},
    {Word.z}
    };
  Matrix Wt=new Matrix(wt);
  Matrix view=ViewMatrix.inverse().times(Wt);
  return new PVector((float)view.get(0,0),(float)view.get(1,0),(float)view.get(2,0));
}
