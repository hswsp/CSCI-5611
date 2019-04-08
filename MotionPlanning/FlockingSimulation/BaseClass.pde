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
    successor.parent=this;
    this.successors.add(successor);
  }
  
  void Addsuccessor(Node node)
  {
    Node successor=new Node(node);
    successor.parent=this;
    this.successors.add(successor);
  }
}//Node

abstract class Agent
{
  float R;
  float H;
  PVector P;
  PVector forward;
  PVector goal;
  PVector start;
  float Vel;
  CSpace space;
  /*road map*/
  Float weightmap[][];
  Vector<PVector> samples;
  /*Search*/
  ArrayList<Integer>  Path;
  /*animate agent*/
  Iterator<Integer> targetItr;//point to Path
  Integer CurtarId;
  Integer NexttarId;
  abstract void Gen_Road();
  Agent()
  {
    goal=new PVector(9,9,0);
    start = new PVector(-9,-9,0);
    forward = new PVector();
    R=0.5;
    H=3*R;
    P=new PVector(start.x,start.y,start.z);
    Vel=3*mag;
    samples=new Vector(roomw*roomh);
  } 
}
public class ReachException extends Exception 
{
  public ReachException(String message)
  {
    super(message);
  }
}
