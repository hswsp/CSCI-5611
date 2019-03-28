import java.util.Vector;
import java.util.Iterator;

class Node
{
  int Index;//current node index in samples
  float g;
  float h;
  float f;
  Vector<Node> successors;
  Node parent;
  Node(Node node)
  {
    this.Index=node.Index;
    this.f=node.f;
    this.g=node.g;
    this.h=node.h;
    this.parent=node.parent;
    this.successors =new Vector<Node>(node.successors);
  }
  Node(int index)
  {
    this.Index=index;
    this.f= this.g= this.h=0;
    successors =new Vector<Node>();
    this.parent=null;
  }

  void Addsuccessor(int index)
  {
    Node successor=new Node(index);
    successor.parent=this;//new Node(this)
    this.successors.add(successor);
  }
}//class

/*draw picture*/
static float mag=50;
static float Z=.2*mag;
/*global about floor*/
static float Floorz=0;
static int roomw=21;
static int roomh=21;
/*global about agent*/
static float agentR=0.5;
static float agentH=3*agentR;
static PVector agentP=new PVector();
/*ball obstacle*/
static PVector pball=new PVector();
static float ballR=2;

static PVector goal=new PVector(9,9,0);
static PVector start = new PVector(-9,-9,0);

/*animate agent*/
Iterator<Integer> targetItr;//point to Path
Integer CurtarId;
Integer NexttarId;

/*road map*/
static Vector<PVector> samples=new Vector(roomw*roomh);//store milestone: each element is a point(PVector)
static Float weightmap[][];//store the wight between each milestones(i->j)
static int K=5;//K-NN


/*Search*/
ArrayList<Integer>  Path;

/*keyboard Interaction*/
static boolean IsStartAnimation=false;
