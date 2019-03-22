import java.util.Vector;
static float mag=50;
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

/*road map*/
static Vector<PVector> samples=new Vector(roomw*roomh);//store milestone: each element is a point(PVector)
static Float weightmap[][];//store the wight between each milestones(i->j)
static int K=5;//K-NN
/*Search*/
ArrayList<Node>  CLOSED;
/*draw picture*/
static float Z=.2*mag;
