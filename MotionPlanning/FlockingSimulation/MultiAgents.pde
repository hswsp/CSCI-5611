class MultiAgents
{
  Integer AgentNum;
  Agent[] agents;
  MultiAgents(String type,int num)
  {
    AgentNum=num;
    if(type=="PRM")
    {
     agents=new PRMAgent[AgentNum];
     for(int i=0;i<AgentNum;++i)
      {
        agents[i] = new PRMAgent();
      }
    }
    else
    {
      agents=new RRTAgent[AgentNum];
      for(int i=0;i<AgentNum;++i)
      {
        agents[i] = new RRTAgent();
      }
    }
    if(AgentNum>1)
    {
      agents[1].start=new PVector(-9,9,0);
      agents[1].goal=new PVector(9,-9,0);
    }
    
  }
  void init()
  {
    for(int i=0;i<AgentNum;++i)
    {
      agents[i].P.set(agents[i].start.x,agents[i].start.y,agents[i].H/2).mult(mag);
      agents[i].space = new CylinderBall(agents[i].R);
      agents[i].Gen_Road();
      agents[i].targetItr=agents[i].Path.iterator();
      if(agents[i].targetItr.hasNext())
      {
        agents[i].NexttarId=agents[i].CurtarId = agents[i].targetItr.next();
      }
    }
  }
   
  
  
}
