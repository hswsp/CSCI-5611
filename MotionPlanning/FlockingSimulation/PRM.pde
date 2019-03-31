import java.util.Random;
import java.util.Arrays;
import java.util.Collections;
import java.util.Map;
import java.util.List;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Stack;

class PRMAgent extends Agent
{
  Float weightmap[][];
  Vector<PVector> samples;
  ArrayList<Integer>  Path;
  /*animate agent*/
  Iterator<Integer> targetItr;//point to Path
  Integer CurtarId;
  Integer NexttarId;
  PRMAgent()
  {
    super();
    samples=new Vector(roomw*roomh);
  }
  /*---------------------------------------------PRM-----------------------------------------------------*/
  void PRM()
  {
    
    int [][]room=new int [roomw][roomh];
    int Maxiter=50;
    for(int[] row:room)
    {
      for(int p:row)
      {
        p=0;//0 mean not be choosen
      }
    }
    
    /*Generate milestones*/
    int iter=0;
    samples.add(PVector.mult(start,mag));//start is the first one
    room[(int)start.x+roomw/2][(int)start.y+roomh/2]=1;
    Random r=new Random();
    while(iter<Maxiter)
    {
      int row=r.nextInt(roomw-1);
      int col=r.nextInt(roomh-1);
      if(room[row][col]==0)
      {
        room[row][col]=1;
        PVector p=new PVector(row-roomw/2,col-roomh/2,0);
        p.mult(mag);
        if(feasible(p, new CylinderBall(R)))
        {
          samples.add(p);
          iter++;
        }
      }
    }
    samples.add(PVector.mult(goal,mag));//goal is the last one
    room[(int)goal.x+roomw/2][(int)goal.y+roomh/2]=1;
    
    /*connect neiboring milestones*/
    //initial
    int dimension=samples.size();
    weightmap =new Float[dimension][];
    for(int i=0;i<dimension;++i)
    {
      weightmap[i]=new Float[dimension];
      for(int j=0;j<dimension;++j)
      {
        weightmap[i][j]=Float.MAX_VALUE;  //initial as infinit
      }
    }
    //compute distance
    for(int i=0;i<dimension;++i)
    {
      Map<Integer, Float> Distance = new HashMap<Integer, Float>();
      for(int j=0;j<dimension;++j)
      {
         Distance.put(j,weightmap[i][j]);
         PVector p1=samples.get(i);
         PVector p2=samples.get(j);
         if(i!=j && !intersection(p1,p2,new CylinderBall(R)))
         {
           Distance.put(j,PVector.sub(p2,p1).mag());
         }
      }
      //sort ascending
      List<Map.Entry<Integer,Float>> list = new ArrayList<Map.Entry<Integer,Float>>(Distance.entrySet());
      Collections.sort(list,new Comparator<Map.Entry<Integer,Float>>() 
      {
        //sort ascending
        public int compare(Map.Entry<Integer, Float> o1,Map.Entry<Integer, Float> o2) 
        {
          return o1.getValue().compareTo(o2.getValue());
        }        
      });
      
      //store K-NN distance
      int k=0;
      boolean FoundNeighbor=false;
      for(Map.Entry<Integer,Float> mapping:list)
      { 
        float value=mapping.getValue();
        if(value!=Float.MAX_VALUE)
        {
          weightmap[i][mapping.getKey()]=value;
          FoundNeighbor=true;
        }
       
         if(++k>K&&FoundNeighbor)
         {
           break;
         }
      } 
      
    }//end for
    
  }//function
  
  /*-----------------------------------------Search PRM------------------------------------------*/
  void generateSuccessor(Node node)
    {
      int dimension=samples.size();
      for(int j=0;j<dimension;++j)
      {
        if(weightmap[node.Index][j]<Float.MAX_VALUE)
        {
          node.Addsuccessor(j);
        }
      }
    }
    
  void AStarSearch(ArrayList<Node>  CLOSED)
  {
    int dimension=samples.size();
    Queue<Node> OPEN = new PriorityQueue<Node>(dimension,idComparator);
    Node Start=new Node(0);
    Start.g=0;
    Start.h=PVector.sub(samples.get(dimension-1),samples.get(Start.Index)).mag();
    Start.f=Start.g+Start.h;
    OPEN.add(Start);//put the starting node on the open list 
    while(!OPEN.isEmpty())
    {
      Node q=OPEN.poll();
      generateSuccessor(q);
      for(Node successor:q.successors)
      {
        if(successor.Index==dimension-1)// successor is the goal
        {
          println("found goal!");
          CLOSED.add(q);
          CLOSED.add(successor);
          return;
        }
        successor.g = q.g + weightmap[successor.Index][q.Index];
        successor.h = PVector.sub(samples.get(dimension-1),samples.get(successor.Index)).mag();//Euclidean Heuristics
        successor.f = successor.g + successor.h;
        /*if a node with the same position as 
          successor is in the OPEN list which has a 
          lower f than successor, skip this successor*/
        Iterator<Node> itrO = OPEN.iterator();
        boolean isInList=false;
        float tempf=Float.MAX_VALUE;
        while (itrO.hasNext()) 
        {
          Node p=itrO.next();
          if(p.Index==successor.Index)
          {
            isInList=true;
            tempf=p.f;
            break;
          }
        }
        if(isInList && tempf<successor.f)
        {
          continue;
        }
        /*if a node with the same position as 
          successor  is in the CLOSED list which has
          a lower f than successor, skip this successor
          otherwise, add  the node to the open list*/
        Iterator<Node> itrC = CLOSED.iterator();
        isInList=false;
        tempf=Float.MAX_VALUE;
        while (itrC.hasNext()) 
        {
          Node p=itrC.next();
          if(p.Index==successor.Index)
          {
            isInList=true;
            tempf=p.f;
            break;
          }
        }
        if( isInList && tempf<successor.f)
        {
          continue;
        }
        else
        {
          OPEN.add(successor);
        }
      }//for
    CLOSED.add(q); 
    }//while
  }//function
  
  
 
  void SearchRoad()
  {
    int dimension=samples.size();
    ArrayList<Node>  CLOSED= new ArrayList<Node> (dimension);
    Path= new ArrayList<Integer> (dimension);
    Stack<Integer> stack = new Stack();
    AStarSearch(CLOSED);
    Iterator<Node> iter = CLOSED.iterator(); 
    Integer value=0;
    Node cur=iter.next();
    while(iter.hasNext())
    {
      cur=iter.next();
    }
    while(cur!=null)
    {
      value=cur.Index;
      stack.push(value);
      cur=cur.parent;
    }
    //store the index of the path node from start to goal
    int totalNumber=stack.size();
    for(int i=0; i<totalNumber; i++) 
    { 
      Path.add(stack.pop());//stack.get(i)
    }
  }
  
  void PRM_Road()
  {
    PRM();
    SearchRoad();
  }
}

 //Comparator
public static Comparator<Node> idComparator = new Comparator<Node>()//Small top pile, only ensure the top is the smallest
{
  @Override
  public int compare(Node c1, Node c2) 
  {
      return (int) (c1.f - c2.f);
  }
};
  
