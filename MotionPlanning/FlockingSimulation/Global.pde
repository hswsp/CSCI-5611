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
}//Node

class Agent
{
  float R;
  float H;
  PVector P;
  PVector goal;
  PVector start;
  Agent()
  {
    goal=new PVector(9,9,0);
    start = new PVector(-9,-9,0);
    R=0.5;
    H=3*R;
    P=new PVector();
  }
}

/*draw picture*/
static float mag=50;
static float Z=.2*mag;

/*global about floor*/
static float Floorz=0;
static int roomw=21;
static int roomh=21;

/*global about agent*/
Integer AgentNum=2;
RRTAgent []agents;
PRMAgent []Magents;
/*ball obstacle*/
static int ObsNumber=5;
static PVector[] pball=new PVector[ObsNumber];
static float[] ballR = new float[ObsNumber];

/*road map*/
static int K=5;//K-NN

/*keyboard Interaction*/
static boolean IsStartAnimation=false;
