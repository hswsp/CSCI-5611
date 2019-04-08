class Boid {

  PVector position;
  PVector velocity;
  PVector acceleration;
  float r;
  float maxCohforce;    // Maximum steering force
  float maxsepforce;
  float maxspeed;    // Maximum speed
    Boid(Agent agent) {
    acceleration = new PVector(0, 0);
    this.velocity = agent.forward;
    this.position = agent.P;
    r = agent.R;
    maxspeed = agent.Vel;
    maxsepforce = 0.05*mag;
    maxCohforce = 0.01*mag;
  }

  void applyForce(PVector force) {
    acceleration.add(force);
  }

  // We accumulate a new acceleration each time based on three rules
  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids);   // Separation
    PVector ali = align(boids);      // Alignment
    PVector coh = cohesion(boids);   // Cohesion
    // Arbitrarily weight these forces
    sep.mult(1.5);
    ali.mult(1.0);
    coh.mult(1.0);
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
  }

  // Method to update position
  void update() {
    // Update velocity
    velocity.add(acceleration);
    // Limit speed
    velocity.limit(maxspeed);
    position.add(velocity);//PVector.mult(velocity,dt)
    // Reset accelertion to 0 each cycle
    acceleration.mult(0);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position);  // A vector pointing from the position to the target
    // Scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);
    // Steering = Desired - Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxCohforce);  // Limit to maximum steering force
    return steer;
  }

  // Separation
  // Method checks for nearby boids and steers away
  PVector separate (ArrayList<Boid> boids) {
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Boid other : boids) {
      float minseparation=(this.r+other.r)*mag;
      float desiredseparation = 1.1*minseparation;
      float d = PVector.dist(position, other.position);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) { //>0 exempt itself
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(position, other.position);
        diff.normalize();
         if(d<=minseparation)
        {
          diff.mult(maxsepforce);
        }
        else{
         diff.div(d-minseparation);  // Weight by distance
        }        
        steer.add(diff);
        count++;            // Keep track of how many
      }
    }
    // seperate from static obstacles
    for (int i=0;i<ObsNumber;++i) {
      float minseparation=(this.r+ballR[i])*mag;
      float desiredseparation = 1.1*minseparation;
      float d = PVector.dist(position, pball[i]);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(position, pball[i]);
        diff.normalize();
        if(d<=minseparation)
        {
          diff.mult(maxsepforce);
        }
        else{
         diff.div(d-minseparation);
        }
        steer.add(diff);
        count++;      
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }
    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxsepforce);
    }
    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  PVector align (ArrayList<Boid> boids) {
    PVector sum = new PVector(0, 0);
    int count = 0;
    for (Boid other : boids) {
      float neighbordist = (2*this.r+other.r)*mag;
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div((float)count);//average
      // Implement Reynolds: Steering = Desired - Velocity
      sum.normalize();
      sum.mult(maxspeed);
      PVector steer = PVector.sub(sum, velocity);
      steer.limit(maxCohforce);
      return steer;
    } 
    else {
      return new PVector(0, 0);
    }
  }

  // Cohesion
  // For the average position of all nearby boids, calculate steering vector towards that position
  PVector cohesion (ArrayList<Boid> boids) {
    PVector sum = new PVector(0, 0);   // Start with empty vector to accumulate all positions
    int count = 0;
    for (Boid other : boids) {
      float neighbordist = (2*this.r+other.r)*mag;
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.position); // Add position
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return seek(sum);  // Steer towards the position
    } 
    else {
      return new PVector(0, 0);
    }
  }
}
