#ifndef TIER3ADVISOR_H
#define TIER3ADVISOR_H
          
/*
 * Tier3Advisor.h
 * Created:Jun. 27, 2013
 * Last Modified: April 14, 2014
 *
 * brief Implementation of my idea of tier three advisors.
 * Note: I think this is important fact that I keep forgetting from time to time so I would like to keep it here:
 * advisors should implement their own check whether they are commenting or not. I already have advisor_active member variable
 * that I use when initializing them. This is global flag (for the entire run), while advisor_commenting flag will be set or 
 * reset depending on the environment state, only for active robots.
 *
 * author Slavisa Djukic <sdjukic@hunter.cuny.edu>
 *
 */

# include <string>
# include <vector>
# include <set>
# include <map>

# include "Beliefs.h"
# include "FORRAction.h"

class Tier3Advisor { 

 public:
  // All we need to initialize the advisor is:
  // possible actions it can take, its name, description and flag that indicates
  // whether advisor is active or not
  // NOTE: variable st indicates max step robot can take, not sure if this is 
  // important, check and correct.
  // Interesting thing is that compiler is making me initialize member values
  // in the same order they are declared
  // NOTE: how I am calling Advisor constructor in this call
  Tier3Advisor(Beliefs *beliefs, string name, string description, double weight, double *parameters, bool isActive = true); 

    // Dummy default constructor
    Tier3Advisor() {};

    // Need destructor too, has to be virtual, since individual advisors in the Controller.cpp are casted as generic
    // tier3 advisors so this destructor will be called on them, but they are created with their own constructors
    // which should be called when terminating them
    virtual ~Tier3Advisor();

    /*
     * Function that will produce comment for proposed action
     * It will be overloaded by individual advisor i.e. whether it is greedouy
     * or obstacleAvoider
     * This will work fine because target can be defined differently for different
     * advisors. Greedy will consider where target is and for obstacleAvoider
     * target will be wall
     */
    virtual double actionComment(FORRAction action) = 0;

    // This is factory method that will create concrete instance of Tier3Advisor
    static Tier3Advisor* makeAdvisor(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool isActive);

    // This function will return advices on all proposed actions
    std::map <FORRAction, double> allAdvice();

    // method that returns advisor's name; it is used to print it out in the log
    // to see which advisor gave what advice strenght to which action
    string get_name(){ return name;}
    
    // method that returns the flag of whether advisor is active or not
    bool is_active(){ return active; }
    // accessor to advisor_active variable
    void set_active(bool value) { active = value; }   

    double get_weight() { return weight;}  
    void set_weight(double weight) {weight = weight;} 
    
    // setter and getter methods for isCommenting flag (whether advisor should comment current action)
    bool is_commenting() { return advisor_commenting; }
    // every active advisor for the run will call this function before it is asked for comments
    // if this function sets advisor_commenting variable to false then this advisor will not
    // produce comments this iteration.
    virtual void set_commenting() = 0;
    void normalize(map <FORRAction, double> *);
    void rank(map <FORRAction, double> *);
    void standardize(map <FORRAction, double> *);
    
    // Cannot use private since I am subclassing this class and I need all these member variables
 protected:

    // this is array in which we store magic numbers for our advisors
    double auxiliary_constants[4];
    string name;
    string description;
    double weight;
    Beliefs *beliefs;
    /* some advisers will not be active all the time this flag is showing whether adviser is active or not
     * this flag should be checked in the RobotController so we do not waste function call on it and also confuse the
     * return value from such (unnecessary) call !!!
     * But we have to cycle through all the active advisors for the run and make them check whether they should comment
     * or not.
     */
    bool advisor_commenting;
    // this flag gets set by the configuration file and enables/disables advisor for the whole run
    bool active;
};

class Tier3Greedy : public Tier3Advisor{
  public:
  // this is constructor for greedy advisor
  Tier3Greedy (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3Greedy();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3Greedy() {};
  // Tier3Greedy should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3GreedyRotation : public Tier3Advisor{
  public:
  // this is constructor for greedy advisor
  Tier3GreedyRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3GreedyRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3GreedyRotation() {};
  // Tier3Greedy should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ExitFinderLinear : public Tier3Advisor{
  public:
  // this is constructor for exitfinder advisor
  Tier3ExitFinderLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3ExitFinderLinear();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3ExitFinderLinear() {};
  // TierExitFinder should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3ExitFinderRotation : public Tier3Advisor{
  public:
  // this is constructor for exitfinder advisor
  Tier3ExitFinderRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3ExitFinderRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3ExitFinderRotation() {};
  // Tier3Greedy should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ExitFinderFieldLinear : public Tier3Advisor{
  public:
  // this is constructor for exitfinderfield advisor
  Tier3ExitFinderFieldLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3ExitFinderFieldLinear();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3ExitFinderFieldLinear() {};
  // TierExitFinderField should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3ExitFinderFieldRotation : public Tier3Advisor{
  public:
  // this is constructor for exitfinderfield advisor
  Tier3ExitFinderFieldRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3ExitFinderFieldRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3ExitFinderFieldRotation() {};
  // Tier3ExitFinderField should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3RegionLeaverLinear : public Tier3Advisor{
  public:
  // this is constructor for RegionLeaver advisor
  Tier3RegionLeaverLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3RegionLeaverLinear();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3RegionLeaverLinear() {};
  // TierRegionLeaver should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3RegionLeaverRotation : public Tier3Advisor{
  public:
  // this is constructor for RegionLeaver advisor
  Tier3RegionLeaverRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3RegionLeaverRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3RegionLeaverRotation() {};
  // Tier3RegionLeaver should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3RegionFinderLinear : public Tier3Advisor{
  public:
  // this is constructor for regionfinder advisor
  Tier3RegionFinderLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3RegionFinderLinear();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3RegionFinderLinear() {};
  // TierRegionFinder should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3RegionFinderRotation : public Tier3Advisor{
  public:
  // this is constructor for rotationfinder advisor
  Tier3RegionFinderRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3RegionFinderRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3RegionFinderRotation() {};
  // Tier3RegionFinderRotation should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3AvoidRobot : public Tier3Advisor{
  public:
  // this is constructor for greedy advisor
  Tier3AvoidRobot (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3AvoidRobot();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3AvoidRobot() {};
  // Tier3AvoidRobot should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3AvoidRobotRotation : public Tier3Advisor{
  public:
  // this is constructor for greedy advisor
  Tier3AvoidRobotRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3AvoidRobotRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3AvoidRobotRotation() {};
  // Tier3Greedy should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3AvoidObstacle : public Tier3Advisor{
 public:
  Tier3AvoidObstacle(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Default constructor
  Tier3AvoidObstacle();
  // Same as for Tier3Greedy, need to define destructor
  virtual ~Tier3AvoidObstacle () {};
  // This type of advisor will avoid obstacles by implementing Gompertz function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3AvoidObstacleRotation : public Tier3Advisor{
 public:
  Tier3AvoidObstacleRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Default constructor
  Tier3AvoidObstacleRotation();
  // Same as for Tier3Greedy, need to define destructor
  virtual ~Tier3AvoidObstacleRotation () {};
  // This type of advisor will avoid obstacles by implementing Gompertz function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};
/*
class Tier3CloseIn : public Tier3Advisor{
 public:
  Tier3CloseIn(Beliefs *beliefs, string name, string description, double weight, double * magic_init, bool is_active = true); 
  // Default constructor
  Tier3CloseIn();

  virtual ~Tier3CloseIn () {};
  // This type of advisor will avoid obstacles by implementing Gompertz function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3CloseInRotation : public Tier3Advisor{
 public:
  Tier3CloseInRotation(Beliefs *beliefs, string name, string description, double weight, double * magic_init, bool is_active = true); 
  // Default constructor
  Tier3CloseInRotation();

  virtual ~Tier3CloseInRotation() {};
  // This type of advisor will avoid obstacles by implementing Gompertz function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};
*/
// this is advisor that favors big steps
class Tier3BigStep : public Tier3Advisor{
 public:
  Tier3BigStep(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3BigStep();
  
  virtual ~Tier3BigStep() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3BigStepRotation : public Tier3Advisor{
 public:
  Tier3BigStepRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3BigStepRotation();
  
  virtual ~Tier3BigStepRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3GoAroundRotation : public Tier3Advisor{
 public:
  Tier3GoAroundRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3GoAroundRotation();
  
  virtual ~Tier3GoAroundRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};



// Idea for this advisor
// since it is fuzzy it should allow robot to go away from the target if it
// is close to it (assume going around the wall)
// But the further away from the target the more paranoid it should become

/* advisor added Jan. 13, 2014
 * To strengthen the moves in the direction of the target I added this
 * advisor that gives positive comment to all the moves that are 45 degrees
 * from the line that connects robot and target-> then we consider that we
 * are generally getting closer to the target with those steps.
 * Moves that are between 45 and 90 degrees from that line, in both 
 * directions are considered possible for this advisor so it gives them 0.
 * And this advisor does not like moves that are away from the target so
 * it gives -5 to those actions.
 */ 
class Tier3AvoidLeaf : public Tier3Advisor{
 public:
  Tier3AvoidLeaf(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3AvoidLeaf();
  
  virtual ~Tier3AvoidLeaf() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3AvoidLeafRotation : public Tier3Advisor{
 public:
  Tier3AvoidLeafRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3AvoidLeafRotation();
  
  virtual ~Tier3AvoidLeafRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3AvoidLeafField : public Tier3Advisor{
 public:
  Tier3AvoidLeafField(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3AvoidLeafField();
  
  virtual ~Tier3AvoidLeafField() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3AvoidLeafFieldRotation : public Tier3Advisor{
 public:
  Tier3AvoidLeafFieldRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3AvoidLeafFieldRotation();
  
  virtual ~Tier3AvoidLeafFieldRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3Explorer : public Tier3Advisor{
 public:
  Tier3Explorer(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3Explorer();
  
  virtual ~Tier3Explorer() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ExplorerRotation : public Tier3Advisor{
 public:
  Tier3ExplorerRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3ExplorerRotation();
  
  virtual ~Tier3ExplorerRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3BaseLine : public Tier3Advisor{
 public:
  Tier3BaseLine(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3BaseLine();
  
  virtual ~Tier3BaseLine() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3BaseLineRotation : public Tier3Advisor{
 public:
  Tier3BaseLineRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3BaseLineRotation();
  
  virtual ~Tier3BaseLineRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};



class Tier3WaypointFinderLinear : public Tier3Advisor{
 public: 
  Tier3WaypointFinderLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3WaypointFinderLinear();
  
  virtual ~Tier3WaypointFinderLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};



class Tier3WaypointFinderRotation : public Tier3Advisor{
 public: 
  Tier3WaypointFinderRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3WaypointFinderRotation();
  
  virtual ~Tier3WaypointFinderRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};



class Tier3TrailFinderLinear : public Tier3Advisor{
 public: 
  Tier3TrailFinderLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3TrailFinderLinear();
  
  virtual ~Tier3TrailFinderLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};



class Tier3TrailFinderRotation : public Tier3Advisor{
 public: 
  Tier3TrailFinderRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3TrailFinderRotation();
  
  virtual ~Tier3TrailFinderRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3EnterDoorLinear : public Tier3Advisor{
 public: 
  Tier3EnterDoorLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3EnterDoorLinear();
  
  virtual ~Tier3EnterDoorLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3EnterDoorRotation : public Tier3Advisor{
 public: 
  Tier3EnterDoorRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3EnterDoorRotation();
  
  virtual ~Tier3EnterDoorRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3ExitDoorLinear : public Tier3Advisor{
 public: 
  Tier3ExitDoorLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3ExitDoorLinear();
  
  virtual ~Tier3ExitDoorLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3ExitDoorRotation : public Tier3Advisor{
 public: 
  Tier3ExitDoorRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3ExitDoorRotation();
  
  virtual ~Tier3ExitDoorRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3DoorAttractorLinear : public Tier3Advisor{
 public: 
  Tier3DoorAttractorLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3DoorAttractorLinear();
  
  virtual ~Tier3DoorAttractorLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3DoorAttractorRotation : public Tier3Advisor{
 public: 
  Tier3DoorAttractorRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3DoorAttractorRotation();
  
  virtual ~Tier3DoorAttractorRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3NeighborDoorLinear : public Tier3Advisor{
 public: 
  Tier3NeighborDoorLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3NeighborDoorLinear();
  
  virtual ~Tier3NeighborDoorLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3NeighborDoorRotation : public Tier3Advisor{
 public: 
  Tier3NeighborDoorRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3NeighborDoorRotation();
  
  virtual ~Tier3NeighborDoorRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


#endif



