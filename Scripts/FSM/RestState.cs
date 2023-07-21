using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RestState : AbstractState
{
    public RestState(Host _robot_Host) : base(_robot_Host){}
    public override void EnterBehavior()
    {
        Debug.Log("State: Rest");
    }

    public override void UpdateBehavior()
    {
        CheckTask();
    }

    public override void ExitBehavior()
    {
        robot_Host.InitController();
        robot_Host.SetState(new PrepareState(robot_Host));
    }

    void CheckTask(){
        if(robot_Host.task_Paths != null && robot_Host.task_Paths.Count > 0){
            bool shouldUpdate = robot_Host.UpdateTaskPath();
            if(shouldUpdate){
                //Debug.Log("Get Mission~");
                ExitBehavior();
            }
        }
    }
}
