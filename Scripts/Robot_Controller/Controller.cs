using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Controller : MonoBehaviour
{
    
    public Agent_Setting robot;
    public bool showRobot = false;
    public bool useStablePushing = false;
    
    const float local_Update_Dist = 0.15f;

    // 注意!!! 這是 For 機器人的，不是 For Hybrid A 的，所以這個可以靠近一點
    const float pos_Accuracy = 0.02f;
    const float pushing_Pos_Accuracy = 0.05f;
    const int heading_Accuracy = 2;
    const int pushing_heading_Accuracy = 2;
    float cos_Heading_Accuracy = Mathf.Cos(heading_Accuracy * Mathf.Deg2Rad);
    float robot_Length;
    float max_Omega;
    float sensor_Radius;

    #region State
    public bool isGoal;
    bool start_Move = false;
    #endregion

    public float velocity;
    public float omega;

    Transform _trans;
    Rigidbody _rb;
    float _fixedDeltaTime;
    
    #region Planner
    A_Star_PathFinding A_Planner;

    DWA_Planner DWA;
    DWA_Pushing DWA_Pushing;
    #endregion

    #region For Test
    public Transform _targetTrans;
    #endregion

    Vector2 target_Pos;
    //float target_Dir;
    Vector2[] path = null;
    robot_WayPoint[] robot_Path = null;

    #region 計時用
    float timer = 0;
    float findpath_Timer = 0;
    #endregion

    void Start()
    {
        _rb = GetComponent<Rigidbody>();
        _trans = transform;
        _fixedDeltaTime = Time.fixedDeltaTime;

        robot_Length = robot.agent_Size.x;
        max_Omega = robot.max_Omega;
        sensor_Radius = robot.sensor_Radius;
        velocity = 0;
        omega = 0;

        A_Planner = GameObject.FindGameObjectWithTag(robot.agent_Name).GetComponent<A_Star_PathFinding>();
        DWA = GetComponent<DWA_Planner>();
        DWA_Pushing = GetComponent<DWA_Pushing>();
    }

#region 機器人移動的部分
    void FixedUpdate()
    {
        Move();
    }
#endregion

    public void UpdatePushingConstrain(float _constraint){
        //DWA.pushingConstraint = _constraint;
        DWA_Pushing.pushingConstraint = _constraint;
    }

    public void UpdateRobotPath(robot_WayPoint[] _path){
        robot_Path = _path;
        robot_Waypoint_Index = 0;
        //DWA_Pushing.InitIndex();
        start_Move = true;
    }

    //  這是 controller 的 update，掛在主控的 update 下。
    //  若 startplan 為真，則開始路徑以及運動規劃。

    // mode 1 ：是採用傳統 A* + DWA 的方式，並需要考量 Box Layer，以及轉向
    // mode 2 ：是採用 Hybrid A* 生成的路徑 + DWA，不考慮 Box Layer 及轉向
    public void Controller_Update()
    {
        Vector2 cur_Pos = Tools.Vec3ToVec2(_trans.localPosition);
        float curDir = Tools.AngleToWorld(_trans.localEulerAngles.y);
        if(robot_Path != null){
            int final_Index = 0;
            if(robot_Path.Length == 1){
                final_Index = 0;
            }else{
                final_Index = robot_Path.Length - 1;
            }
            Vector2 final_Pos = robot_Path[final_Index].pos;
            float final_Dir =  Tools.AngleToWorld(robot_Path[final_Index].theta);
            int final_Mode = robot_Path[final_Index].mode;
            float dist_ToGoal = Vector2.Distance(cur_Pos, final_Pos);
            float dir_Diff = Mathf.Abs(curDir - final_Dir);
            
            CheckIfIsGoal(dist_ToGoal, final_Dir, final_Mode);
            // 先判斷最終點的類型為何
            // 再判斷是否已抵達
            // 否 的話就直行 locomotion_Controller
            if(start_Move){
                timer += _fixedDeltaTime;
                if(robot_Path.Length > 0){
                    if(timer > 0.1f){
                        Locomotion_Controller(cur_Pos, curDir, final_Pos, final_Dir, final_Mode, robot_Path);
                        timer = 0;
                    }
                }
            }else{
                velocity = 0;
                omega = 0;
            }
        }
    }

    float old_Dist = float.MaxValue;
    void CheckIfIsGoal(float _dist_ToGoal, float _final_Dir, int _final_Mode){
        if(_final_Mode < 0){
            if(_dist_ToGoal <= pos_Accuracy){
                object[] stop_Params = new object[3]{_dist_ToGoal, _final_Dir, _final_Mode};
                StopCoroutine("SlowStop");
                StartCoroutine("SlowStop", stop_Params);
            }
        }else{
            if(_dist_ToGoal <= pushing_Pos_Accuracy){
                object[] stop_Params = new object[1]{_dist_ToGoal};
                StopCoroutine("SlowStop_Pushing");
                StartCoroutine("SlowStop_Pushing", stop_Params);   
            }
        }

    }

    
    IEnumerator SlowStop(object[] _params){
        float dist = (float)_params[0];
        float _final_Dir = (float)_params[1];
        int mode = (int)_params[2];
        velocity = 0.4f * velocity;
        omega = 0.4f * omega;
        Debug.Log("SlowStop!" + " Mode: " + mode);
        if(old_Dist >= dist){
            Debug.Log("If");
            old_Dist = dist;
            if(dist <= 0.4f * pos_Accuracy){
                velocity = 0;
                omega = 0;
                if(mode == -2){
                    Vector2 temp_Tar_Dir_Vec = new Vector2(Mathf.Cos(_final_Dir * Mathf.Deg2Rad), Mathf.Sin(_final_Dir * Mathf.Deg2Rad));
                    Vector2 temp_Cur_Dir_Vec = Tools.Vec3ToVec2(_trans.forward);
                    float cross_Dir = temp_Tar_Dir_Vec.x * temp_Cur_Dir_Vec.y - temp_Tar_Dir_Vec.y * temp_Cur_Dir_Vec.x;
                    float dot_Dir = temp_Tar_Dir_Vec.x * temp_Cur_Dir_Vec.x + temp_Tar_Dir_Vec.y * temp_Cur_Dir_Vec.y;
                    if(dot_Dir >= cos_Heading_Accuracy){
                        Debug.Log("Normal: Goal!!");
                        isGoal = true;
                        start_Move = false;
                        old_Dist = float.MaxValue;
                        yield return new WaitForSeconds(2);
                    }else{
                        Debug.Log("Turn");
                        TurnDir(cross_Dir, dot_Dir);
                    }
                }else{
                    Debug.Log("Weird");
                    isGoal = true;
                    start_Move = false;
                    old_Dist = float.MaxValue;
                    yield break;
                }
                yield break;
            }
        }else if(old_Dist < dist){
            Debug.Log("WTF");
            velocity = 0;
            omega = 0;
            if(mode == -2){
                Vector2 temp_Tar_Dir_Vec = new Vector2(Mathf.Cos(_final_Dir * Mathf.Deg2Rad), Mathf.Sin(_final_Dir * Mathf.Deg2Rad));
                Vector2 temp_Cur_Dir_Vec = Tools.Vec3ToVec2(_trans.forward);
                float cross_Dir = temp_Tar_Dir_Vec.x * temp_Cur_Dir_Vec.y - temp_Tar_Dir_Vec.y * temp_Cur_Dir_Vec.x;
                float dot_Dir = temp_Tar_Dir_Vec.x * temp_Cur_Dir_Vec.x + temp_Tar_Dir_Vec.y * temp_Cur_Dir_Vec.y;
                if(dot_Dir >= cos_Heading_Accuracy){
                    Debug.Log("Normal: Goal!!");
                    isGoal = true;
                    start_Move = false;
                    old_Dist = float.MaxValue;
                    yield return new WaitForSeconds(2);
                }else{
                    Debug.Log("Turn");
                    TurnDir(cross_Dir, dot_Dir);
                }
            }else{
                Debug.Log("Weird");
                isGoal = true;
                start_Move = false;
                old_Dist = float.MaxValue;
                yield return new WaitForSeconds(2);
            }
            yield break;
        }
    }
   IEnumerator SlowStop_Pushing(object[] _params){
        float dist = (float)_params[0];
        velocity = 0.8f * velocity;
        omega = 0.8f * omega;
        Debug.Log("SlowStop!");
        if(old_Dist >= dist){
            old_Dist = dist;
            if(dist <= 0.4f * pushing_Pos_Accuracy){
                velocity = 0;
                omega = 0;
                isGoal = true;
                start_Move = false;
                old_Dist = float.MaxValue;
                yield return new WaitForSeconds(2);
            }
        }else if(old_Dist < dist){
            velocity = 0;
            omega = 0;
            isGoal = true;
            start_Move = false;
            old_Dist = float.MaxValue;
            yield return new WaitForSeconds(2);
        }
        yield break;
    }

    public void Locomotion_Controller(Vector2 _cur_Pos, float _cur_Dir, Vector2 _final_Pos, float _final_Dir, int _final_Mode, robot_WayPoint[] _path)
    {
        // 用來丟進 DWA 的
        Vector2 twist = new Vector2(velocity, omega);
        // 得到 local target
        robot_WayPoint[] temp_Path = GetFarPointPath(_path);
        Vector2 new_End_Point = temp_Path[temp_Path.Length - 1].pos;
        //robot_WayPoint local_WayPoint = Simple_Local(temp_Path, cur_Pos, final_Mode);
        robot_WayPoint local_WayPoint = Simple_Local(_path, _cur_Pos, _final_Mode);
        int wayPoint_Mode = local_WayPoint.mode;
        show_Local = local_WayPoint.pos;    // for debug
        //Debug.Log("wayPoint_Mode:" + wayPoint_Mode);

        if(_final_Mode < 0){
            if(_final_Mode != -3){
                //DWA_Pushing.Call_DWA(cur_Pos, curDir, local_WayPoint.pos, final_Pos, final_Dir, twist, false);
                DWA_Pushing.Call_DWA_Normal(_cur_Pos, _cur_Dir, local_WayPoint.pos, _final_Pos, _final_Dir, twist, true);
                UpdateTwist(_path.Length);
            }else{
                DWA_Pushing.Call_DWA_Normal(_cur_Pos, _cur_Dir, local_WayPoint.pos, _final_Pos, _final_Dir, twist, false);
                UpdateTwist(_path.Length);
            }
        }    
        else{
            // 同上，當小於多少範圍的時候 執行一個 corountine 稍微等個幾秒 如果沒有越來越近就停下 反之則超過一定的時間就停下
            // if(dist_ToGoal <= pushing_Pos_Accuracy && dir_Diff <= pushing_heading_Accuracy){
            // //if(dist_ToGoal <= pushing_Pos_Accuracy && ){
            //     velocity = 0;
            //     omega = 0;
            //     Debug.Log("Pushing: Goal!!");
            //     isGoal = true;
            //     start_Move = false;
            // }else{
                //TODO: 
                // 把整條 path 傳進去 然後 follow heaading
    
                // if(wayPoint_Mode == 2){
                    //DWA_Pushing.Call_DWA(cur_Pos, curDir, local_WayPoint.pos, final_Pos, final_Dir, twist, true);
                    //DWA_Pushing.Call_DWA_Pushing(cur_Pos, curDir, temp_Path, new_End_Point, final_Dir, twist, false);
                    DWA_Pushing.Call_DWA_Pushing(_cur_Pos, _cur_Dir, local_WayPoint.pos, new_End_Point, _final_Dir, twist, false);
                    UpdateTwist(_path.Length);
                // }else{
                    //DWA_Pushing.Call_DWA(cur_Pos, curDir, local_WayPoint.pos, final_Pos, final_Dir, twist, true);
                //     DWA_Pushing.Call_DWA(cur_Pos, curDir, local_WayPoint.pos, final_Pos, final_Dir, twist, true);
                //     UpdateTwist(_path.Length);
                // }
            
        }

    }

    void UpdateTwist(int _pathLength){
        if(!isGoal){
            float weight = 1;
            int dist_far = _pathLength - robot_Waypoint_Index;
            if(dist_far <= 10){
                weight = 0.8f + 0.02f * dist_far;
            }
            velocity =  weight * DWA_Pushing.velocity;
            omega = weight * DWA_Pushing.omega;
        }
    }
    void TurnDir(float _cross_Dir, float _dot_Dir){
        if(_cross_Dir > 0){
            omega = -Dir_Omega(_dot_Dir);
        }else{
            omega = Dir_Omega(_dot_Dir);
        }
    }

    float Dir_Omega(float _dot_Dir){
        if(_dot_Dir < 0){
            return max_Omega * 0.6f;
        }else if(_dot_Dir > 0 && _dot_Dir < 0.6f){
            return max_Omega * 0.5f;
        }else{
            return max_Omega * 0.4f;
        }
    }

    int robot_Waypoint_Index = 0;
    robot_WayPoint Simple_Local(robot_WayPoint[] _path, Vector2 _curPos, int _mode){
        float change_Dist = 0.22f;
        if(_mode < 0){
            change_Dist = 0.32f;
        }
        if(_path.Length == 1){
            return _path[0];
        }else{
            if(robot_Waypoint_Index == 0){
                robot_Waypoint_Index = 2;
                return _path[1];
            }
            if(robot_Waypoint_Index < _path.Length - 1){
                float dist = Vector2.Distance(_curPos, _path[robot_Waypoint_Index].pos);
                if(dist < change_Dist){
                //if(dist < 0.4f){
                    robot_Waypoint_Index++;
                    return _path[robot_Waypoint_Index];
                }
            }else{
                robot_Waypoint_Index = _path.Length - 1;
                return _path[robot_Waypoint_Index];
            }
            return _path[robot_Waypoint_Index];
        }
    }


    robot_WayPoint[] GetFarPointPath(robot_WayPoint[] _path){
        if(_path[_path.Length-1].mode < 0){
            return _path;
        }
        if(_path.Length == 1){
            return _path;
        }else{
            robot_WayPoint[] new_Path = new robot_WayPoint[_path.Length + 1];
            for(int i = 0; i < _path.Length; i++){
                new_Path[i] = _path[i];
                if(i == _path.Length - 1){
                    new_Path[i + 1] = FarPoint(_path[_path.Length - 1]);
                    // new_Path[i] = _path[i];
                }
            }
            return new_Path;
        }
    }

    robot_WayPoint FarPoint(robot_WayPoint _finalPoint){
        float dist = 1.414f * 0.1f * 1.2f ;
        float new_X = dist * Mathf.Sin(_finalPoint.theta * Mathf.Deg2Rad);
        float new_Y = dist * Mathf.Cos(_finalPoint.theta * Mathf.Deg2Rad);
        Vector2 new_Pos = new Vector2(_finalPoint.pos.x + new_X, _finalPoint.pos.y + new_Y);
        robot_WayPoint advancePoint = new robot_WayPoint(new_Pos, _finalPoint.theta, 0);
        return advancePoint;
    }

    void Move(){
        //WriteString(velocity,omega);
        float curDir = Tools.AngleToWorld(_trans.localEulerAngles.y);

        float new_X = _trans.localPosition.x;
        float new_Y = _trans.localPosition.z;
        float new_Dir = curDir;

        float temp_Vel = Mathf.Abs((float)Math.Round(velocity, 4));
        float temp_Omega = Mathf.Abs((float)Math.Round(omega, 4));

        if(temp_Omega == 0 && temp_Vel != 0 ){
            new_X = new_X + velocity * Mathf.Cos(curDir * Mathf.Deg2Rad) * _fixedDeltaTime;
            new_Y = new_Y + velocity * Mathf.Sin(curDir * Mathf.Deg2Rad) * _fixedDeltaTime; 
        }
        else if(temp_Vel == 0 && temp_Omega != 0){  
            new_Dir = new_Dir + _fixedDeltaTime * omega * Mathf.Rad2Deg;
        }
        else if(temp_Vel == 0 && temp_Omega == 0){
            new_X = _trans.localPosition.x;
            new_Y = _trans.localPosition.z;
            new_Dir = curDir;
        }else{
            new_X = new_X + (velocity/omega) * (Mathf.Sin((omega * _fixedDeltaTime) + curDir * Mathf.Deg2Rad) - Mathf.Sin(curDir * Mathf.Deg2Rad));
            new_Y = new_Y - (velocity/omega) * (Mathf.Cos((omega * _fixedDeltaTime) + curDir * Mathf.Deg2Rad) - Mathf.Cos(curDir * Mathf.Deg2Rad));
            new_Dir = curDir + _fixedDeltaTime * omega * Mathf.Rad2Deg;
        }
        Quaternion eulerToQ = Quaternion.Euler(_trans.localEulerAngles.x, Tools.AngleToUnity(new_Dir), _trans.localEulerAngles.z);
        Vector3 temp_Pos = new Vector3(new_X, _trans.localPosition.y, new_Y);
        _rb.MovePosition(temp_Pos);
        _rb.MoveRotation(eulerToQ);
    }


    Vector2 show_Local; 
    void OnDrawGizmos(){
        const float dist = 1.414f * 0.1f * 1.6f * 0.5f;
        Gizmos.color = Color.red;
        Vector3 local = Tools.Vec2ToVec3(show_Local);
        Gizmos.DrawSphere(local, 0.1f);
        // Gizmos.color = Color.black;
        // for(int i = 0; i < path.Length; i++){
        //     Gizmos.DrawSphere(path[i]- Vector3.up, 0.05f);
        // }
    
        if(showRobot){
            if(robot_Path != null && robot_Path.Length > 0){
                foreach(robot_WayPoint p in robot_Path){
                    Vector3 dir = new Vector3(p.pos.x + dist * Mathf.Sin((p.theta) * Mathf.Deg2Rad) , 0, p.pos.y + dist * Mathf.Cos((p.theta) * Mathf.Deg2Rad));
                    if(p.mode == 2){
                        Gizmos.color = Color.yellow;
                    }else if(p.mode == 1){
                        Gizmos.color = Color.cyan;
                    }else if(p.mode == 3){
                        Gizmos.color = Color.green;
                    }else{
                        Gizmos.color = new Color(085f, 0.325f, 0.098f, 0.4f);
                    }
                    Gizmos.DrawLine(Tools.Vec2ToVec3(p.pos), dir);
                    Gizmos.DrawSphere(Tools.Vec2ToVec3(p.pos), 0.1f);
                }
            }
        }
    }



    // robot_WayPoint Local_Waypoint(robot_WayPoint[] _path)
    // {
    //     Vector2 _path_Goal = _path[_path.Length-1].pos;
    //     Vector2 cur_Pos = Tools.Vec3ToVec2(_trans.localPosition);
    //     float dist = Vector2.Distance(_path_Goal, cur_Pos);

    //     if(robot_Waypoint_Index < _path.Length){
    //         if(robot_Waypoint_Index == 0){
    //             for(int i = 0; i < _path.Length; i++){
    //                 float cur_To_Local = Vector2.Distance(cur_Pos, _path[i].pos);
    //                 if(cur_To_Local > sensor_Radius * 0.25f){
    //                     robot_Waypoint_Index = i;
    //                     return _path[robot_Waypoint_Index];
    //                 }
    //             }
    //         }else{
    //             float old_Dist = Vector2.Distance(cur_Pos, _path[robot_Waypoint_Index].pos);
    //             if(old_Dist < 0.24f * sensor_Radius){
    //                 if(robot_Waypoint_Index < _path.Length - 1){
    //                     robot_Waypoint_Index++;
    //                 }else{
    //                     robot_Waypoint_Index = _path.Length - 1;
    //                 }
    //             }
    //             return _path[robot_Waypoint_Index];
    //         }
    //     }
    //     return _path[_path.Length-1];
    // }
}
