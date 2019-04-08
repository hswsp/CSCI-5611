import java.util.Vector;
import java.util.Iterator;


/*draw picture*/
static float mag=50;
static float Z=.2*mag;

/*global about floor*/
static float Floorz=0;
static int roomw=21;
static int roomh=21;

MultiAgents multiAgents;
/*ball obstacle*/
static int ObsNumber=5;
static PVector[] pball=new PVector[ObsNumber];
static float[] ballR = new float[ObsNumber];

/*road map*/
static int K=5;//K-NN

/*keyboard Interaction*/
static boolean IsStartAnimation=false;

/*animation*/
float dt=0.01;

//Line bestline;
