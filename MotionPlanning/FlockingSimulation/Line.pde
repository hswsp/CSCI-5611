

/**
 * Defines a directed line.
 */
public class Line 
{
    /**
     * The direction of this directed line.
     */
    public PVector direction = new PVector();

    /**
     * A point on this directed line.
     */
    public PVector point = new PVector();

    /**
     * Constructs and initializes a directed line.
     */
    public Line() {
    }

    /**
     * Constructs and initializes a directed line with the specified point and
     * direction.
     *
     * @param point     A point on the directed line.
     * @param direction The direction of the directed line.
     */
    public Line(PVector point, PVector direction) {
        this.direction = direction.normalize();
        this.point = point;
    }
    
    public Line Perpendicular(PVector point)
    {
      PVector Pdirection=new PVector();
      Pdirection.y=this.direction.x;
      Pdirection.x=-this.direction.y;
      Pdirection.normalize();
      return new Line(point,Pdirection);
    }
    public PVector Intersection(Line line1,Line line2)
    {
      /*
       return : intersaction point
       
      */
      if(line1.direction==line2.direction ||line1.direction==PVector.mult(line2.direction,-1))
        return null;
      else
      {
        Double [][]A=new Double[2][2];
        Double []B=new Double[2];
        A[0][0]=(double)line1.direction.x;
        A[0][1]=-(double)line2.direction.x;
        A[1][0]=(double)line1.direction.y;
        A[1][1]=-(double)line2.direction.y;
        B[0]=(double)line2.point.x-(double)line1.point.x;
        B[1]=(double)line2.point.y-(double)line1.point.y;
        Cramer solver=new Cramer(A,B);
        Double [] t=solver.solve ();
        double t1=t[0];
        double t2=t[1];
        return  PVector.add(line1.point,PVector.mult(line1.direction,(float)t1));
      }
       
    }
}
