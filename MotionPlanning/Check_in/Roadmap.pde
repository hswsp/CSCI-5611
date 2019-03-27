import java.util.Random;
import java.util.Arrays;
import java.util.Collections;
import java.util.Map;
import java.util.List;
import java.util.Comparator;
import java.lang.Float;
void PRM()
{
  int [][]room=new int [roomw][roomh];
  int Maxiter=50;
  for(int[] row:room)
  {
    for(int p:row)
    {
      p=0;//0 mean not be choosen
    }
  }
  
  /*Generate milestones*/
  int iter=0;
  samples.add(PVector.mult(start,mag));//start is the first one
  room[(int)start.x+roomw/2][(int)start.y+roomh/2]=1;
  Random r=new Random();
  while(iter<Maxiter)
  {
    int row=r.nextInt(roomw-1);
    int col=r.nextInt(roomh-1);
    if(room[row][col]==0)
    {
      room[row][col]=1;
      PVector p=new PVector(row-roomw/2,col-roomh/2,0);
      p.mult(mag);
      if(feasible(p))
      {
        samples.add(p);
        iter++;
      }
    }
  }
  samples.add(PVector.mult(goal,mag));//goal is the last one
  room[(int)goal.x+roomw/2][(int)goal.y+roomh/2]=1;
  
  /*connect neiboring milestones*/
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
  
  //compute distance
  for(int i=0;i<dimension;++i)
  {
    Map<Integer, Float> Distance = new HashMap<Integer, Float>();
    for(int j=0;j<dimension;++j)
    {
       Distance.put(j,weightmap[i][j]);//Initial
       PVector p1=samples.get(i);
       PVector p2=samples.get(j);
       if(i!=j && !intersection(p1,p2))
       {
         Distance.put(j,PVector.sub(p2,p1).mag());
       }
    }
    //sort ascending
    List<Map.Entry<Integer,Float>> list = new ArrayList<Map.Entry<Integer,Float>>(Distance.entrySet());
    Collections.sort(list,new Comparator<Map.Entry<Integer,Float>>() 
    {
      //sort ascending
      public int compare(Map.Entry<Integer, Float> o1,Map.Entry<Integer, Float> o2) 
      {
        return o1.getValue().compareTo(o2.getValue());
      }        
    });
    
    //store K-NN distance
    int k=0;
    boolean FoundNeighbor=false;
    for(Map.Entry<Integer,Float> mapping:list)
    { 
      float value=mapping.getValue();
      if(value!=Float.MAX_VALUE)
      {
        weightmap[i][mapping.getKey()]= weightmap[mapping.getKey()][i]=value;
        FoundNeighbor=true;
      }
      // System.out.println(mapping.getKey()+":"+mapping.getValue()); 
       if(++k>K&&FoundNeighbor)
       {
         break;
       }
    } 
    
  }//end for
  
}//function

 
