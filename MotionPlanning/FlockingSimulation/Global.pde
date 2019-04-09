import java.util.Vector;
import java.util.Iterator;


/*draw picture*/
static float mag=10;
static float Z=.5*mag;

/*global about floor*/
static float Floorz=0;
static int roomw=101;//21
static int roomh=101;// 21

MultiAgents multiAgents;
/*ball obstacle*/
int ObsNumber=5;
PVector[] pball=new PVector[ObsNumber];
float[] ballR = new float[ObsNumber];

/*road map*/
static int K=5;//K-NN

/*keyboard Interaction*/
static boolean IsStartAnimation=false;

/*animation*/
float dt=0.01;
boolean AddObstacle=false;
boolean Agent0Start=false;
boolean Agent0goal=false;
Line[] bestline=new Line[2];

static int agentnumber=20;
static float AgentsR=0.5;
