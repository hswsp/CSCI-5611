import processing.opengl.*;

class Cone extends Shape//Zc=1/4*h
{
  float radius;//bottom raius
  float height; //length
  Cone(float a, float l, float m)// a is raius and l is height
  {
    super(m);
    this.radius=a;
    this.height=l;
    double[][]Inertia=
    {
      {3 * m * (4*a*a+l*l) / 80,0,0},
      {0,3 * m * (a*a) / 10,0},
      {0,0,3 * m * (4*a*a+l*l) / 80}
    };
    ComputeRotInertia(new Matrix(Inertia));
  }
}

class RigidCone extends RigidBody
{
  Cylinder shape;
  float e_floor=0.9;//coeff of collission
  RigidCone(float a, float l, float m)
  {
    super(new Cylinder(a,  l, m));
    this.shape=new Cylinder( a,  l,  m);
  }
}


void drawCone( int sides, float r1, float r2, float h)
{
    float angle = 360 / sides;
    float halfHeight = h / 2;

    // draw top of the tube
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r1;
        float y = sin( radians( i * angle ) ) * r1;
        vertex( x, y, -halfHeight);
    }
    endShape(CLOSE);

    // draw bottom of the tube
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r2;
        float y = sin( radians( i * angle ) ) * r2;
        vertex( x, y, halfHeight);
    }
    endShape(CLOSE);
    
    // draw sides
    beginShape(TRIANGLE_STRIP);
    for (int i = 0; i < sides + 1; i++) {
        float x1 = cos( radians( i * angle ) ) * r1;
        float y1 = sin( radians( i * angle ) ) * r1;
        float x2 = cos( radians( i * angle ) ) * r2;
        float y2 = sin( radians( i * angle ) ) * r2;
        vertex( x1, y1, -halfHeight);
        vertex( x2, y2, halfHeight);    
    }
    endShape(CLOSE);

}
