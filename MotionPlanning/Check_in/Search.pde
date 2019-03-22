import java.util.PriorityQueue;
import java.util.Queue;
import java.util.ArrayList;
import java.util.Iterator;
class Node
{
  int Index;//current node index in samples
  float g;
  float h;
  float f;
  Vector<Node> successors;
  Node parent;
  Node(int index)
  {
    this.Index=index;
    successors =new Vector<Node>();
    f=g=h=0;
  }
  Node(Node node)
  {
    this.Index=node.Index;
    this.successors =node.successors;
    this.f=node.f;
    this.g=node.g;
    this.h=node.h;
    this.parent=node.parent;
  }
  private void Addsuccessor(int index)
  {
    Node successor=new Node(index);
    successor.parent=new Node(this);
    this.successors.add(successor);
  }
  void generateSuccessor()
  {
    int dimension=samples.size();
    for(int j=0;j<dimension;++j)
    {
      if(weightmap[this.Index][j]<Float.MAX_VALUE)
      {
        this.Addsuccessor(j);
      }
    }
  }
 
}//class


void AStarSearch()
{
  int dimension=samples.size();
  Queue<Node> OPEN = new PriorityQueue<Node>(dimension,idComparator);
  CLOSED= new ArrayList<Node> (dimension);
  Node Start=new Node(0);
  Start.g=0;
  Start.h=PVector.sub(samples.get(dimension-1),samples.get(Start.Index)).mag();
  Start.f=Start.g+Start.h;
  OPEN.add(Start);//put the starting node on the open list 
  while(!OPEN.isEmpty())
  {
    Node q=OPEN.poll();
    q.generateSuccessor();
    
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


//Comparator
public static Comparator<Node> idComparator = new Comparator<Node>()//Small top pile, only ensure the top is the smallest
{
  @Override
  public int compare(Node c1, Node c2) 
  {
      return (int) (c1.f - c2.f);
  }
};
