using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Host : MonoBehaviour
{
    #region For Test
    public Transform _targetTrans;
    #endregion

    public Agent_Setting robot1;
    public A_Star_PathFinding a_Star_Planner;
    AbstractState current_State;
    public Controller _controller;

    public Box_Info task_Box;

    // 0: init, no task
    // 1: first task
    // 2: second taks
    // ...
    [HideInInspector] public int task_Index = 0;
    [HideInInspector] public List<robot_WayPoint[]> task_Paths = new List<robot_WayPoint[]>();
    [HideInInspector] public List<int> task_Indexs = new List<int>();
    [HideInInspector] public robot_WayPoint[] cur_Path;
    //[HideInInspector] public bool isContact = false;

    float robot_Length;
    int prepare_Time = 10;
    Transform _trans;

    void Start()
    {
        _trans = transform;
        SetState(new RestState(this));
        robot_Length = robot1.agent_Size.x;
        task_Index = 0;
    }

    void Update()
    {
        current_State.UpdateBehavior();
        _controller.Controller_Update();
        // if(Input.GetKeyDown("t")){
        //     _controller.UpdatePushingConstrain(0.6f);
        //     _controller.test();
        // }
    }


    public void A_Star_PathFinding(robot_WayPoint _tar){
        a_Star_Planner.StartFindPath(
            _startPos: Tools.Vec3ToVec2(_trans.localPosition),
            _targetPos: _tar.pos,
            _useBoxLayer: true);
    }

    public void StartComplete(){
        robot_WayPoint complete_Point = GetCompletePoint();
        robot_WayPoint[] complete_Path = new robot_WayPoint[1]{complete_Point};
        _controller.isGoal = false;
        _controller.UpdateRobotPath(complete_Path);
    }
    public void StartPushing(){
        _controller.isGoal = false;
        _controller.UpdateRobotPath(cur_Path);
    }

    public void StartContact(){
        robot_WayPoint[] contact_Path = new robot_WayPoint[1]{cur_Path[0]};
        _controller.isGoal = false;
        _controller.UpdateRobotPath(contact_Path);
    }

#region Complete Point
robot_WayPoint GetCompletePoint(){
    float dist = 0.8f * robot_Length;
    Vector2 heading = new Vector2(_trans.forward.x, _trans.forward.z);
    Vector2 curPos = Tools.Vec3ToVec2(_trans.localPosition);
    Vector2 temp = new Vector2(curPos.x - dist * heading.x, curPos.y - dist * heading.y);
    robot_WayPoint completePoint = new robot_WayPoint(temp, _trans.eulerAngles.y, -3);
    return completePoint;
}
#endregion 


#region Prepare Point
    public void StartPrepare(){
        StartCoroutine("GoToPreparePoint");
    }
    IEnumerator GoToPreparePoint(){
        robot_WayPoint preparePoint = GetPreparePoint(cur_Path);
        A_Star_PathFinding(preparePoint);
        yield return new WaitUntil(() => a_Star_Planner.path.Length > 0);

        robot_WayPoint[] preparePath = GetAStarPathToWayPoint(preparePoint);
        _controller.isGoal = false;
        _controller.UpdateRobotPath(preparePath);
        //_controller.UpdatePushingConstrain();
        yield return prepare_Time;
    }

    public robot_WayPoint[] GetAStarPathToWayPoint(robot_WayPoint _prepare){
        Vector2[] path = a_Star_Planner.path;
        //  Debug.Log("A star Path: " + path.Length);
        robot_WayPoint[] temp_Waypoints = new robot_WayPoint[path.Length];
        for(int i = 0; i < path.Length; i++){
            if(i == path.Length - 1){
                temp_Waypoints[i] = _prepare; 
            }else{
                temp_Waypoints[i] = new robot_WayPoint(path[i], 0, -1);
            }
        }
        return temp_Waypoints;
    }

    public robot_WayPoint GetPreparePoint(robot_WayPoint[] _cur_Path){
        robot_WayPoint first_Point = _cur_Path[0];
        float new_X = -robot_Length * Mathf.Sin(first_Point.theta * Mathf.Deg2Rad);
        float new_Y = -robot_Length * Mathf.Cos(first_Point.theta * Mathf.Deg2Rad);

        Vector2 preparePos = new Vector2(first_Point.pos.x + new_X, first_Point.pos.y + new_Y);
        robot_WayPoint prepare_Point = new robot_WayPoint(preparePos, first_Point.theta, -2);
        return prepare_Point;
    }

#endregion

    public bool IsGoal(){
        if(_controller.isGoal){
            return true;
        }else{
            return false;
        }
    }

    public void InitController(){
        _controller.isGoal = false;
    }



    public bool UpdateTaskPath(){
        if(task_Index < task_Paths.Count){
            cur_Path = task_Paths[task_Index];
            task_Index++;
            return true;
        }
        else{
            return false;
        }
    }

    public void AllocateNewTask(List<robot_WayPoint[]> _paths, List<int> _indexs){
        task_Paths = _paths;
        task_Indexs = _indexs;

        task_Index = 0;
    }


    public void SetState(AbstractState state){
        current_State = state;
        state.EnterBehavior();
    }


    void OnDrawGizmos(){
        float r = 0.05f;
        float dist = 0.5f;
       
    }
}
