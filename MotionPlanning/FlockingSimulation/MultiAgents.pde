class MultiAgents
{
  Integer AgentNum;
  Agent[] agents;
  ArrayList<Float> []milestones;
  ArrayList<PVector> AgentsP;
  ArrayList<Boid> boids; // treat it as boids
  MultiAgents(String type,int num)
  {
    AgentNum=num;
    milestones =new ArrayList[AgentNum];
    AgentsP = new ArrayList<PVector>(AgentNum);
    boids = new ArrayList<Boid>(AgentNum);
    if(type=="PRM")
    {
      this.agents=new PRMAgent[AgentNum];
    }
    else
    {
      this.agents=new RRTAgent[AgentNum];
    }
    HashSet<Integer> rowbias = new HashSet<Integer>();
    HashSet<Integer> colbias = new HashSet<Integer>();
    int biasnum=5;
    RandomNumbers.randomSet(-biasnum,biasnum,2*biasnum,rowbias);
    RandomNumbers.randomSet(-biasnum,biasnum,2*biasnum,colbias);
    int [] row=new int[2*biasnum];
    int [] col=new int[2*biasnum];
    int m=0;
    for (Integer s:rowbias) {
      row[m++]=s;
    }
    m=0;
     for (Integer s:colbias) {
      col[m++]=s;
    }
    for(int i=0;i<AgentNum;++i)
    {
      
      if(type=="PRM")
      {
        this.agents[i] = new PRMAgent();
      }
      else
      {
        this.agents[i] = new RRTAgent();
      }
      this.agents[i].start=new PVector(pow(-1,(i/2)%2+1)*(roomw/2.5),pow(-1,((i+1)/2)%2+1)*(roomh/2.5),0);
      this.agents[i].goal=new PVector(pow(-1,(i/2)%2)*(roomw/2.5),pow(-1,((i+1)/2)%2)*(roomh/2.5),0);
      this.agents[i].start.x+=row[(i%(2*biasnum)+i/(2*biasnum))%(2*biasnum)];
      this.agents[i].start.y+=col[i%(2*biasnum)];
      boids.add(new Boid(agents[i]));
    }
    //for test
    if(testmode) 
    {
      this.agents[0].start=new PVector(-5,-5,0);
      this.agents[0].goal=new PVector(5,5,0); 
      this.agents[1].start=new PVector(-5,5,0);
      this.agents[1].goal=new PVector(5,-5,0);
      this.agents[2].start=new PVector(5,5,0);
      this.agents[2].goal=new PVector(-5,-5,0);
      this.agents[3].start=new PVector(5,-5,0);
      this.agents[3].goal=new PVector(-5,5,0);
      this.agents[4].start=new PVector(-7,0,0);
      this.agents[4].goal=new PVector(7,0,0);
      this.agents[5].start=new PVector(7,0,0);
      this.agents[5].goal=new PVector(-7,0,0);
      //this.agents[6].start=new PVector(-7,3,0);
      //this.agents[6].goal=new PVector(7,-3,0);
      //this.agents[7].start=new PVector(7,-3,0);
      //this.agents[7].goal=new PVector(-7,3,0);
    }
    
    
  }
  private ArrayList<Float> genmilestonetime(Agent agent)
  {
    ArrayList<Float> milestones=new ArrayList<Float>();
    float t=0;
    float v=agent.Vel;
    if(agent.Path==null||agent.Path.size()==0)
    {
      return null;
    }
    int dimension=agent.Path.size();
    for(int i=0;i<dimension-1;++i)
    {
      milestones.add(t);
      t=t+PVector.sub(agent.samples.get(agent.Path.get(i+1)),agent.samples.get(agent.Path.get(i))).mag()/v;
    }
    milestones.add(t);
    return milestones;
  }
   private void calculateAgentsP(float t)
   {
    for(int i=0;i<AgentNum;++i)
    {
      if(agents[i].Path==null)
      {
        break;
      }
      PVector s=new PVector();
      float t0=0;
      int k=0;
      if(t>=milestones[i].get(milestones[i].size()-1))
      {
        AgentsP.set(AgentNum,PVector.mult(agents[i].goal,mag));
      }
      else
      {
        while(t0<=t)
        {
          t0=milestones[i].get(k);
          s=agents[i].samples.get(agents[i].Path.get(k));
          ++k;//point to the next
        }
        PVector dir=PVector.sub(agents[i].samples.get(agents[i].Path.get(k)),s).normalize();
        PVector arri=PVector.add(s,PVector.mult(dir,agents[i].Vel*(t-t0)));
        AgentsP.set(AgentNum,arri);
      }
    }
   }
   
  class AgentsInteraction implements CSpace //Priority method
  {
    int CurrentAgent;
    AgentsInteraction(){}
    AgentsInteraction(int AgenNumber)
    {
      CurrentAgent=AgenNumber;
    }
    public boolean inCspace(PVector x,float t,PVector[] pball, float[] ballR)
    {
      calculateAgentsP(t);
      for(int i=0;i<ObsNumber;++i)
      {
        float SphereR =(agents[CurrentAgent].R+ballR[i])*mag;
        float distance=PVector.sub(x,pball[i]).mag();
        if(distance<SphereR)
        {
          return false;
        }
      }
      for(int i=0;i<AgentNum;++i)
      {
        if(i==CurrentAgent||agents[i].Path==null)
        {
          break;
        }
        else
        {
          float SphereR =(agents[CurrentAgent].R+agents[i].R)*mag;
          float distance=PVector.sub(x,AgentsP.get(i)).mag();
          if(distance<SphereR)
          {
            return false;
          }
        }
      }
      return true;
    }
   };
   
  void init()
  {
    for(int i=0;i<AgentNum;++i)
    {
      this.agents[i].P.set(agents[i].start.x,agents[i].start.y,0).mult(mag);
      agents[i].space = new CylinderBall(agents[i].R);
    }
    update();
  }
   void update()
   {
    for(int i=0;i<AgentNum;++i)
    {
      this.agents[i].start.set(agents[i].P.x/mag,agents[i].P.y/mag,0);
      agents[i].Gen_Road();
      milestones[i]=genmilestonetime(agents[i]);
      //for draw smooth path
      agents[i].targetItr=agents[i].Path.iterator();
      if(agents[i].targetItr.hasNext())
      {
        agents[i].NexttarId=agents[i].CurtarId = agents[i].targetItr.next();
      }  
    }  
   }
  
  
}
