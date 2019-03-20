import Jama.Matrix;
/*
all computed in barycentric coordinate. and static coord is :
right, forward and up direction
*/
class Shape
{
  float mass;
  Matrix E;//record the orientation of the 
  Matrix A;//rotate matrix
  Matrix RotInertia;  //Iniertial inverse
  Shape(float m)
  {
    mass=m; 
    double[][]e=
    {
      {1,0,0},
      {0,1,0},
      {0,0,1}
    };
    E=new Matrix(e);
  }
  void ComputeRotInertia(Matrix momentOfInertia)
  {
    RotInertia=momentOfInertia.eig().getD();
    A=momentOfInertia.eig().getV();
    this.RotInertia=this.RotInertia.inverse();
  }
}



class RigidBody
{
  PVector pos;//barycenter
  PVector angle;
  PVector pVel;
  PVector omega;
  PVector L; //AngularMomentum
  PVector force;
  PVector torque;
  public Shape shape;
  RigidBody(Shape shape)
  {
    pos=new PVector(0,0,0);
    angle=new PVector(0,0,0);
    //angle=0;
    pVel=new PVector(0,0,0);
    omega=new PVector(0,0,0);
    L = new PVector(0,0,0);
    force=new PVector(0,0,0);
    torque=new PVector(0,0,0); 
    this.shape=shape;
  }
  Matrix cross2dot(PVector w)
  {
    double [][]w_={
      {0,-w.z,w.y},
      {w.z,0,-w.x},
      {-w.y,w.x,0}
    };
    Matrix w_hat = new Matrix(w_);
    return w_hat;
  }
  void AddForceAndTorque(PVector f,PVector arm) // force and arm vector
  {
    this.force.add(f);
    PVector dtorque=new PVector();
    //println("arm = ",arm,", f=",f);
    arm.cross(f,dtorque);
    this.torque.add(dtorque);
 }
 void ResetForceAndTorque()
 {
   this.force.set(0,0,0);
   this.torque.set(0,0,0);
 }
 Matrix orthonormalization(Matrix At)
 {
   //Gram-Schmidt orthonormalization
    int[]r={0,1,2};
    int[]c={0};
    Matrix b1=At.getMatrix(r,c);
    c[0]=1;
    Matrix b2=At.getMatrix(r,c);
    c[0]=2;
    Matrix b3=At.getMatrix(r,c);
    
    b1=b1.times(1/b1.normF());
    
    b2=b2.minus(b1.times(b1.transpose().times(b2)));
    b2=b2.times(1/b2.normF());
    
    b3=b3.minus(b1.times(b1.transpose().times(b3))).minus(b2.times(b2.transpose().times(b3)));
    b3=b3.times(1/b3.normF());
    double [][]a1=b1.getArray();
    double [][]a2=b2.getArray();
    double [][]a3=b3.getArray();
    double [][]a={
      {a1[0][0],a2[0][0],a3[0][0]},
      {a1[1][0],a2[1][0],a3[1][0]},
      {a1[2][0],a2[2][0],a3[2][0]}
    };
    return new Matrix(a);
 }
 void GetCord(PVector a1,PVector a2,PVector a3)
 {
   int[]r={0,1,2};
   int[]c={0};
   Matrix b1=shape.E.getMatrix(r,c);
   c[0]=1;
   Matrix b2=shape.E.getMatrix(r,c);
   c[0]=2;
   Matrix b3=shape.E.getMatrix(r,c);
   a1.set((float)b1.get(0,0),(float)b1.get(1,0),(float)b1.get(2,0));
   a2.set((float)b2.get(0,0),(float)b2.get(1,0),(float)b2.get(2,0));
   a3.set((float)b3.get(0,0),(float)b3.get(1,0),(float)b3.get(2,0));
 }
 void Updata(float dt)
 {
   //postion
    pos.add(PVector.mult(pVel,dt));
    pos.add(PVector.mult(force,0.5*dt*dt/shape.mass));
    pVel.add(PVector.mult(force,dt/shape.mass));
    
    //angle
    Matrix w=cross2dot(omega);
    Matrix At=shape.A;
    Matrix Et=shape.E;
    L.add(PVector.mult(torque,dt));
    //println("the angler momentum is:",L,"\n");
    Et=Et.plus(w.times(Et).times(dt));
    shape.E=orthonormalization(Et);
    Matrix I=At.times(shape.RotInertia).times(At.transpose());
    double [][]lt=
    {{L.x},
    {L.y},
    {L.z}
    };
    Matrix Lt=new Matrix(lt);
    w=I.times(Lt);
    this.omega.set((float)w.get(0,0),(float)w.get(1,0),(float)w.get(2,0));
    //this.angle = this.omega.mag()*dt;
    this.angle.add(PVector.mult(this.omega,dt));
 }
}

Matrix Vector2Mat(PVector L)
{
  double [][]lt=
    {{L.x},
    {L.y},
    {L.z}
    };
    return new Matrix(lt);
}

PVector Mat2Vector(Matrix w)
{
  return new PVector((float)w.get(0,0),(float)w.get(1,0),(float)w.get(2,0));
}
