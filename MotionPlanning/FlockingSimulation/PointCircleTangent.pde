import static java.lang.Math.*;
import java.awt.geom.Point2D;

public class PointCircleTangent {
  
  // A very same epsilon value used as a threshold 
  public static final double EPS = 0.0000001;

  // Due to double rounding precision the value passed into the asin
  // function may be outside its domain of [-1, +1] which would return
  // the value Double.NaN which we do not want.
  public  float arcsinSafe(float x) {
    if (x <= -1.0) return -PI/2.0;
    if (x >= +1.0) return +PI/2.0;
    return asin(x);
  }

  // Finds the point(s) of intersection of the lines tangent to the circle centered
  // at 'center' from the point 'point'.
  public  ArrayList<PVector> pointCircleTangentPoints(PVector center, float radius, PVector pt) {
    ArrayList<PVector> points=new ArrayList<PVector>();
    float px = pt.x, py = pt.y;
    float cx = center.x, cy = center.y;

    // Compute the distance to the circle center
    float dx = cx - px;
    float dy = cy - py;
    float dist = sqrt(dx*dx+dy*dy);
    
    // Point is strictly contained within the circle
    if (dist <= radius) return points;

    float angle, angle1, angle2;
    
    angle1 = arcsinSafe(radius/dist);
    angle2 = atan2(dy, dx);

    angle = angle2 - angle1;
    PVector p1 = new PVector(cx + radius * sin(angle), cy + radius * -cos(angle),0);

    angle = angle1 + angle2;
    PVector p2 = new PVector(cx + radius * -sin(angle), cy + radius * cos(angle),0);

    // Points are sufficiently close to be considered the same point 
    // (i.e the original point is on the circle circumference) 
    if (PVector.sub(p1,p2).mag() < EPS)
    {
      points.add(pt);
    }
    else
    {
      points.add(p1);
      points.add(p2);
    }
    return points;
  }
}
