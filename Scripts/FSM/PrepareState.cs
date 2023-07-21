using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PrepareState : AbstractState
{
    public PrepareState(Host _robot_Host) : base(_robot_Host){}
    public override void EnterBehavior()
    {
        Debug.Log("State: Prepare");
        robot_Host.StartPrepare();
    }

    public override void ExitBehavior()
    {
        robot_Host.InitController();
        robot_Host.SetState(new ContactState(robot_Host));
    }

    public override void UpdateBehavior()
    {
        if(robot_Host.IsGoal()){
            Debug.Log("Arrive PreParePoint");
            ExitBehavior();
        }
    }



}
