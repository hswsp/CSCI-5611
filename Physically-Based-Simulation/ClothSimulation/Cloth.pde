import java.util.Vector;
float heightOffset;
float ZOffset;
float widthOffset;

PVector Vair=new PVector(0,0,0);
class Particle {
  public PVector pos;
  public PVector vel;
  public PVector acc;//record total acceleration
  public float mass;
  public Vector<Particle> neighbors;
  PVector vn;//previous vel
  PVector acc_old;
  public boolean flag;
  Particle(PVector pos, PVector vel, PVector accel, float m) {
    this.pos = pos;
    this.vel = vel;
    this.acc = accel;
    this.mass = m;
    this.vn=vel;
    this.acc_old=accel;
    this.flag = true;
    neighbors = new Vector<Particle>(2);
  }
}

class Cloth
{
  int width;
  int height;
  int TREADNUM;
  float l0;//rest length
  float maxlenRatio;
  float ks ; //spring
  float kd ;//damping
  Particle ClothParticle[][];
  
  float e_ball;
  float e_floor;
  float rho;//density of fluid
  float cd;
  boolean flag;
  public Cloth()
  {
    TREADNUM=4;
    ks = 100;  //600
    kd = 40;   //0
    Vair=new PVector(0,0,0);
    rho= 1.293E-4;
    cd = 1E-4;
    e_ball=.001;
    e_floor=.2;
    l0=6;
    maxlenRatio=1.5;
    width=30;
    height=30;
    ClothParticle =new Particle[width][];
    for(int i=0;i<width;++i)
    {
      ClothParticle[i]=new Particle[height];
      for(int j=0;j<height;j++)
      {
        PVector inipos=new PVector(widthOffset + i*l0,heightOffset, ZOffset-j* l0);
        PVector inivel=new PVector(0,0,0);
        ClothParticle[i][j]=new Particle(inipos,inivel,g,1.5);//pos,  vel, accel, mass
      }
    }
  }
  
  void updatevn()
  {
    for(int i=0;i<width;++i)
    {
      for(int j=0;j<height;++j)
      {
       this.ClothParticle[i][j].vn.set(this.ClothParticle[i][j].vel);
       this.ClothParticle[i][j].acc_old.set(this.ClothParticle[i][j].acc);
      }
    }
  }
  
  void fixtop()
  {
    for(int j=0;j<height;++j)//fix top
    {
      this.ClothParticle[0][j].vel.set(0,0,0);
      //update pos 
      this.ClothParticle[0][j].pos.set(widthOffset ,heightOffset, ZOffset-j* l0);  
    }
  }
  PVector Aerodynamic(Particle p0)
  {
    PVector f_aero=new PVector(0,0,0);
    
    if(p0.neighbors.size()>1)
    {
      Particle p1=p0.neighbors.get(0);
      Particle p2=p0.neighbors.get(1);
      PVector v=PVector.sub(PVector.div(PVector.add(PVector.add(p0.vn,p1.vn),p2.vn),3.0),Vair);
      PVector r1=PVector.sub(p1.pos,p0.pos);
      PVector r2=PVector.sub(p2.pos,p0.pos);
      PVector norm=new PVector(0,0,0);
      PVector.cross(r2,r1,norm);
     
      float coef=v.mag()*PVector.dot(v,norm)/(2*(norm.mag()+1E-12));
      f_aero=PVector.mult(norm,-0.5*rho*coef*cd);
      //println(f_aero);
      
    }
    return f_aero;
  }
  public void Collision()
  {
    for(int i=0;i<width;++i)
      for(int j=0;j<height;++j)
        {

          if(this.ClothParticle[i][j].pos.y>floor)
          {
            PVector n=new PVector(0,-1,0);
            PVector bounce=PVector.mult(n,PVector.dot(ClothParticle[i][j].vel,n));
            this.ClothParticle[i][j].pos.y=floor;
            this.ClothParticle[i][j].vel.sub(PVector.mult(bounce,1+e_floor));
            
          }
          float d=PVector.sub(pball1,ClothParticle[i][j].pos).mag();
          //ball collision
          if(d<radius+.1)
          {
            PVector n=PVector.mult(PVector.sub(pball1,ClothParticle[i][j].pos),-1);
            n.normalize();
            PVector bounce=PVector.mult(n,PVector.dot(ClothParticle[i][j].vel,n));
            this.ClothParticle[i][j].vel.sub(PVector.mult(bounce,1+e_ball));
            this.ClothParticle[i][j].pos.add(PVector.mult(n,.1+radius-d));// move out
          }
          
        }
  }
  
  public void FindaNeighbors()
  {
    Particle p;
    Particle p2;
    for (int i = 0; i < width; i++) 
    {
      for (int j = 0; j < height; j++) 
      {
        p = ClothParticle[i][j];
        if(i!=width-1)
        {
          p2 = ClothParticle[i+1][j];
          p.neighbors.add(p2);
        }
        if(j!=height-1)
        {
          p2 = ClothParticle[i][j+1];
          p.neighbors.add(p2); 
        } 
      }
    }

  }
  
  public void UpdatePhysics(float dt)
  {
    Particle p0;
    PVector DeltaAcc=new PVector(0,0,0);
    updatevn();
   
   //final CountDownLatch latch = new CountDownLatch(TREADNUM);
   //final CountDownLatch latch1 = new CountDownLatch(TREADNUM);
   //ExecutorService fixedThreadPool = Executors.newFixedThreadPool(TREADNUM);
   // for (int thread = 0; thread < TREADNUM; thread++) //horiz
   // {
   //   final int index = thread;
   //   final float t=dt;
   //   fixedThreadPool.execute(new Runnable() 
   //   {
   //     @Override
   //     public void run() 
   //     {
          //int start=(int)((float)index/(float)TREADNUM*width);
          //int end = (int)((float)(index+1)/(float)TREADNUM*width);
          //if(index==TREADNUM-1)
          //{
          //  end-=1;
          //}
    for(int i=0;i<width;++i)
    {
      for(int j=0;j<height;++j)
      {
        
        p0 = ClothParticle[i][j];
        float m0=p0.mass;
        if (p0.flag){
          //mid point
          p0.pos.add(PVector.mult(p0.vn,dt));//each one only add once v0*dt
          p0.pos.add(PVector.mult(g,0.5*dt*dt));//pos+=1/2*a*t^2
          p0.vel.add(PVector.mult(g,dt));
          
          ////Eulerian
          //p0.pos.add(PVector.mult(p0.vn,dt));
          //p0.vel.add(PVector.mult(g,dt));
          
         //add aerodynamic force
          PVector f_aero=Aerodynamic(p0);
          for(Particle p1:p0.neighbors)
          {
            PVector e=PVector.sub(p1.pos,p0.pos);
            float l=(float)Math.sqrt(e.dot(e));
            e.div(l);
            
            float v1= e.dot(p0.vn);
            float v2= e.dot(p1.vn);
            float fs=-ks*(l0-l);
            if(l>maxlenRatio*l0)
            {
              
            }
            float fd=-kd*(v1-v2);  
            float m1=p1.mass;
            //mid point
            DeltaAcc.set(PVector.mult(e,(fs+fd)/m0));
            //update pos
            p0.pos.add(PVector.mult(DeltaAcc,0.5*dt*dt));
            //update v  
            p0.vel.add(PVector.mult(DeltaAcc,dt));
           
           //add aerodynamic Acc
            DeltaAcc.set(PVector.mult(e,(fs+fd)/m1));
            p1.pos.sub(PVector.mult(DeltaAcc,0.5*dt*dt));
            p1.vel.sub(PVector.mult(DeltaAcc,dt));
            
            ////implicit Euler
            //float a0=e.dot(p0.acc_old);
            //float a1=e.dot(p1.acc_old);
            //DeltaAcc.set(PVector.mult(e,(fs+fd)/m0));
            //p0.acc.add(DeltaAcc);
            //DeltaAcc.set(PVector.mult(e,(fs+fd)/m1));
            //p1.acc.sub(DeltaAcc);
            //float dv1=dt*fd/m0/(1+dt*kd/m0);
            //float dv2 =-dt*fd/m1/(1+dt*kd/m1);
            //p0.vel.add(PVector.mult(e,dv1));
            //p1.vel.add(PVector.mult(e,dv2));  
            //dv1=dt*fs/(m0*(1-dt*v1/a0));
            //dv2=-dt*fs/(m1*(1-dt*v2/a1));
            //p0.vel.add(PVector.mult(e,dv1));
            //p1.vel.add(PVector.mult(e,dv2)); 
            
           
            DeltaAcc = PVector.div(f_aero,3*m1);
            p1.acc.add(DeltaAcc);
            p1.pos.add(PVector.mult(DeltaAcc,0.5*dt*dt));
            p1.vel.add(PVector.mult(DeltaAcc,dt));
            
          }
            DeltaAcc = PVector.div(f_aero,3*m0);
            p0.acc.add(DeltaAcc);
            p0.pos.add(PVector.mult(DeltaAcc,0.5*dt*dt));
            p0.vel.add(PVector.mult(DeltaAcc,dt));
        }
        else{
          continue;
        }
        
          
      }
    }
    //      latch.countDown();
    //    }
    //  });
    //}
    //try {
    //        latch.await();
    //    } catch (InterruptedException e) {
    //        e.printStackTrace();
    //    }
    
    

   fixtop();
    //      latch1.countDown();
    //    }
    //  });
    //}
    //try {
    //        latch1.await();
    //    } catch (InterruptedException e) {
    //        e.printStackTrace();
    //    }

  }
}
