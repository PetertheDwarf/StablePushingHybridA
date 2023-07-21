using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ContactState : AbstractState
{
    public ContactState(Host _robot_Host) : base(_robot_Host){}
    public override void EnterBehavior()
    {
        Debug.Log("State: Contact");
        robot_Host.StartContact();
    }

    public override void ExitBehavior()
    {
        robot_Host.InitController();
        robot_Host.SetState(new PushingState(robot_Host));
    }

    public override void UpdateBehavior()
    {
        if(robot_Host.IsGoal()){
            ExitBehavior();
        }
    }



}

