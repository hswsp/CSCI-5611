import java.util.Random;
import java.util.Queue;
import java.util.LinkedList;
class SearchTree
{
  Node root;
  Node goal;
  Integer size;
}
class RRTAgent extends Agent
{
  /*---------------------------------------------RRT-----------------------------------------------------*/
  // get collision free neighbors
  ArrayList<Node> get_nearby_vertices(PVector curnode,SearchTree Tree)
  {
    ArrayList<Node>  neighbor= new ArrayList<Node>();
    //compute distance
    
    Map<Node, Float> Distance = new HashMap<Node, Float>();
    levelTraverse(Tree,Distance, curnode);
    //sort ascending
    List<Map.Entry<Node,Float>> list = new ArrayList<Map.Entry<Node,Float>>(Distance.entrySet());
    Collections.sort(list,new Comparator<Map.Entry<Node,Float>>() 
    {
      //sort ascending
      public int compare(Map.Entry<Node, Float> o1,Map.Entry<Node, Float> o2) 
      {
        return o1.getValue().compareTo(o2.getValue());
      }        
    });
    
    //store K-NN distance in order
    int k=0;
    for(Map.Entry<Node,Float> mapping:list)
    { 
      if(++k>K)
      {
        break;
      }
      Node p1=mapping.getKey();
      neighbor.add(p1);
    } 
     return neighbor;
  }
  
  void levelTraverse(SearchTree Tree,Map<Node, Float> Distance,PVector curnode)
  {
    // generate all nodes that collision_free to current node
    Node root=Tree.root;
    if(root == null)
        return;
    Queue<Node> queue = new LinkedList<Node>();
    queue.offer(root);
    while(!queue.isEmpty())
    {
      Node node = queue.poll();
      PVector p=samples.get(node.Index);
      if(!intersection(p,0,curnode,0,space))
      {
        float distance=PVector.sub(curnode,p).mag();
        Distance.put(node,distance);
      }
      //next node
      Iterator<Node> succIter=node.successors.iterator();
      while(succIter.hasNext())
      {
        Node child=succIter.next();
        queue.offer(child);
      }
    }
  }
  boolean sample_random_state(Random r,int [][]room,PVector p)
  {
    //sample in CSpace
    int Maxiter=500;
    int iter=0;
    while(iter++<Maxiter)
    {
      int row=r.nextInt(roomw-1);
      int col=r.nextInt(roomh-1);
      if(room[row][col]==0)
      {
        room[row][col]=1;
        p.set(row-roomw/2,col-roomh/2,0).mult(mag);
        if(feasible(p,0,space))
        {
          samples.add(p);
          return true;
        }
      }
    }
    return false;
  }
  
  boolean rewire(SearchTree rrt,Node Xnew)
  {
    //rewire Xnew
    PVector Xrand=samples.get(Xnew.Index);
    ArrayList<Node> neighbor=get_nearby_vertices(Xrand,rrt);
    Node Xbest=null; //best fit neighbor
    float Cmin=Float.MAX_VALUE; // minimun cost
    if(neighbor.isEmpty())
    {
      return false;
    }
    for(Node Xnearst : neighbor)
    {
      PVector Xneighbors= samples.get(Xnearst.Index);
      float cost=Xnearst.g+PVector.sub(Xneighbors,Xrand).mag(); 
      if(cost<Cmin)
      {
        Xbest=Xnearst;
        Cmin=cost;
      }
    }
    if(Xbest!=null)
    {
     Xnew.parent=Xbest;
     Xnew.g=Cmin;
     Xbest.Addsuccessor(Xnew);
    }
    //rewire Xneighbors
    for(Node Xnear : neighbor)
    {
      if(Xnear==Xbest)
      {
        continue;
      }
      PVector Xneighbors= samples.get(Xnear.Index);
      float cost=Xnew.g+PVector.sub(Xneighbors,Xrand).mag();
      if(cost<Xnear.g)
      {
        Xnear.parent.successors.remove(Xnear);
        Xnear.parent=Xnew;
        Xnew.successors.add(Xnear);
        Xnear.g=cost;
      }
    }
   return true;  
  }
  SearchTree RRT() throws ReachException
  {
    SearchTree rrt= new SearchTree();
    float epsilon=3*mag;
    Random r=new Random();
    int [][]room=new int [roomw][roomh];;
    for(int[] row:room)
    {
      for(int p:row)
      {
        p=0;//0 mean not be choosen
      }
    }
    PVector s=PVector.mult(start,mag);
    samples.add(s);//start is the first one
    Node Start=new Node(samples.indexOf(s));
    rrt.root=Start;
    room[(int)start.x+roomw/2][(int)start.y+roomh/2]=1;
    int Maxiter=500;
    int iter=0;
    while(iter++<Maxiter)
    {
      PVector Xrand=new PVector();
      if(!sample_random_state(r,room,Xrand))
      {
        println("Error!Cannot find new sample!");
        continue;//next sample
      }
      Node Xnew=new Node(samples.indexOf(Xrand));
      if(!rewire(rrt,Xnew))
      {
        continue;
      }
      if(Xrand==PVector.mult(goal,mag))
      {
        continue;
      }
      else if((iter%5)==0&&(!intersection(PVector.mult(goal,mag),0,Xrand,0,space)))//&& PVector.sub(PVector.mult(goal,mag),Xrand).mag()<epsilon 
      {
        PVector g=PVector.mult(goal,mag);
        samples.add(g);
        Node END=new Node(samples.indexOf(g));
        rewire(rrt,END);
        rrt.goal=END;
       // println("Reach Goal!");
        break;
      }
      else if(iter>=Maxiter)
      {
        rrt.goal=Xnew;
        throw new ReachException("RRT Cannot Reach Goal!");
      }
      
    }
    return rrt;
  }
  
  void Gen_Road()
  {
    //long startTime=System.currentTimeMillis(); 
    SearchTree rrt;
    try
    {
      rrt=RRT();
     }
    catch(ReachException e)
    {
       RuntimeException exception = new RuntimeException(e);
       throw  exception;
    }
    //long endTime=System.currentTimeMillis(); 
    //System.out.println("time for RRT "+(endTime-startTime)+"ms");
   
    Node cur=rrt.goal;
    Stack<Integer> stack = new Stack();
    Path= new ArrayList<Integer> (samples.size());
    while(cur!=null)
    {
      Integer value=cur.Index;
      stack.push(value);
      cur=cur.parent;
    }
    //store the index of the path node from start to goal
    int totalNumber=stack.size();
    for(int i=0; i<totalNumber; i++) 
    { 
      Path.add(stack.pop());
    }
    
    // show map
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
    levelTraverse(rrt,weightmap);
   
  }
  
  void levelTraverse(SearchTree Tree,Float[][]weightmap)
  {
    // generate all nodes that collision_free to current node
    Node root=Tree.root;
    if(root == null)
        return;
    Queue<Node> queue = new LinkedList<Node>();
    queue.offer(root);
    while(!queue.isEmpty())
    {
      Node node = queue.poll();    
      //next node
      Iterator<Node> succIter=node.successors.iterator();
      while(succIter.hasNext())
      {
        Node child=succIter.next();
        queue.offer(child);
        weightmap[node.Index][child.Index]=child.g;
      }
    }
  }
}
