using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PushingState : AbstractState
{
    public PushingState(Host _robot_Host) : base(_robot_Host){}
    public override void EnterBehavior()
    {
        Debug.Log("State: Pushing");
        
        // 
        robot_Host.StartPushing();

    }

    public override void ExitBehavior()
    {
        robot_Host.InitController();
        robot_Host.SetState(new CompleteState(robot_Host));
    }

    public override void UpdateBehavior()
    {
        if(robot_Host.IsGoal()){
            ExitBehavior();
        }
    }
}
