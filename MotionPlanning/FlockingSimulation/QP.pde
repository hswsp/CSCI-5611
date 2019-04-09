import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import com.manyangled.gibbous.optim.convex.*;
/**
 * Given A, b, c, implements 0.5*(x^T)A(x) + b.x + c;
 * The gradient is A(x) + b, and the Hessian is A
 minize objective function with constraints expressed as 
        Ax <= b
        Ax = b.
 */
 class QP
 {
   QuadraticFunction q;
   PointValuePair pvp;
   LinearInequalityConstraint LI;
   LinearEqualityConstraint LE;
   double[] xmin ;
   double vmin;
   QP()
   {
     q=null;
     pvp=null;
     LI=null;
     LE=null;
   }
   void InitQ(double[][] Q,double[] b, double c)
   {
     q = new QuadraticFunction(Q,b,c);
   }
   void InitLI(double[][] A,double[] b)
   {
     LI = new LinearInequalityConstraint(A,b); //Ax<=b
   }
   void InitLE(double[][] A,double[] b)
   {
     LE = new LinearEqualityConstraint(A,b); 
   }
   void solver() //Initial must in feasible region
   {
     // solve for a feasible point that satisfies the constraints
      PointValuePair fpvp = ConvexOptimizer.feasiblePoint(this.LI, this.LE);
      // if not < 0, there is no feasible point
      assert fpvp.getSecond() < 0.0;
      double[] ig = fpvp.getFirst();
     BarrierOptimizer barrier = new BarrierOptimizer();
     PointValuePair pvp = barrier.optimize(new ObjectiveFunction(q),LI,LE,new InitialGuess(ig));
     xmin = pvp.getFirst();
     vmin = pvp.getSecond();
   }
 }
//// create a convex objective function
//QuadraticFunction q = new QuadraticFunction(
//    new double[][] { { 1.0, 0.0 }, { 0.0, 1.0 } },
//    new double[] { 0.0, 0.0 },
//    0.0);

//// optimize function q with an inequality constraint and an equality constraint,
//// using the barrier method
//BarrierOptimizer barrier = new BarrierOptimizer();
//PointValuePair pvp = barrier.optimize(
//    new ObjectiveFunction(q),
//    new LinearInequalityConstraint(
//        new double[][] { { -1.0, 0.0 } }, // constraint x > 1,
//        new double[] { -1.0 }),
//    new LinearEqualityConstraint(
//        new double[][] { { 0.0, 1.0 } },  // constraint y = 1,
//        new double[] { 1.0 }),
//    new InitialGuess(new double[] { 10.0, 10.0 }));

//double[] xmin = pvp.getFirst();  // { 1.0, 1.0 }
//double vmin = pvp.getSecond();   // 1.0
