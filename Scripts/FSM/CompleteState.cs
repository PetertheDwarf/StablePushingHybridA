using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CompleteState : AbstractState
{
    public CompleteState(Host _robot_Host) : base(_robot_Host){}
    // Start is called before the first frame update
    public override void EnterBehavior()
    {
        Debug.Log("State: Complete");
        
        // 
        robot_Host.StartComplete();

    }
    public override void ExitBehavior()
    {
        robot_Host.InitController();
        robot_Host.SetState(new RestState(robot_Host));
    }

    public override void UpdateBehavior()
    {
        if(robot_Host.IsGoal()){
            ExitBehavior();
        }
    }
}
