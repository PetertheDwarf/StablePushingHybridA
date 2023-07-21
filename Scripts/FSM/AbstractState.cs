using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class AbstractState
{
    protected Host robot_Host;
    
    public AbstractState(Host _robot_Host){
        robot_Host = _robot_Host;
    }

    public virtual void EnterBehavior(){

    }

    public virtual void UpdateBehavior(){

    }

    public virtual void ExitBehavior(){

    }

}
