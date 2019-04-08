class MultiAgents
{
  Integer AgentNum;
  Agent[] agents;
  ArrayList<Float> []milestones;
  ArrayList<PVector> AgentsP;
  MultiAgents(String type,int num)
  {
    AgentNum=num;
    milestones =new ArrayList[AgentNum];
    AgentsP = new ArrayList<PVector>(AgentNum);
    if(type=="PRM")
    {
      this.agents=new PRMAgent[AgentNum];
    }
    else
    {
      this.agents=new RRTAgent[AgentNum];
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
      //milestones[i]=new ArrayList<Float>();
    }
    if(AgentNum>1)
    {
      //this.agents[0].goal=new PVector(-9,9,0);
      
      this.agents[1].start=new PVector(-9,9,0);
      this.agents[1].goal=new PVector(-9,-9,0);//test (-9,-9,0)
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
      agents[i].P.set(agents[i].start.x,agents[i].start.y,0).mult(mag);
      agents[i].space = new CylinderBall(agents[i].R);
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
