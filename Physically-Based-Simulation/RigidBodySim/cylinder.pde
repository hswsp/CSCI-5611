import processing.opengl.*;


class Cylinder extends Shape
{
  float radius;//raius
  float height; //length
  Cylinder(float a, float l, float m)// a is raius and l is height
  {
    super(m);
    this.radius=a;
    this.height=l;
    double[][]Inertia=
    {
      {m * (a*a) / 12,0,0},
      {0,m * (3*a*a+l*l) / 12,0},
      {0,0,m * (3*a*a+l*l) / 12}
    };
    ComputeRotInertia(new Matrix(Inertia));
  }
}

class RigidCylinder extends RigidBody
{
  Cylinder shape;
  float e_floor=0.9;//coeff of collission
  RigidCylinder(float a, float l, float m)
  {
    super(new Cylinder(a,  l, m));
    this.shape=new Cylinder( a,  l,  m);
  }
}

void drawCylinder( int sides, float r, float h)
{
    float angle = 360 / sides;
    float halfHeight = h / 2;

    // draw top of the tube
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float y = sin( radians( i * angle ) ) * r;
        vertex( x, y, -halfHeight);
    }
    endShape(CLOSE);

    // draw bottom of the tube
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float y = sin( radians( i * angle ) ) * r;
        vertex( x, y, halfHeight);
    }
    endShape(CLOSE);
    
    // draw sides
    beginShape(TRIANGLE_STRIP);
    for (int i = 0; i < sides + 1; i++) {
        float x = cos( radians( i * angle ) ) * r;
        float y = sin( radians( i * angle ) ) * r;
        vertex( x, y, halfHeight);
        vertex( x, y, -halfHeight);    
    }
    endShape(CLOSE);

}
