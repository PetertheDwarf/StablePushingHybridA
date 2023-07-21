using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DWA_Pushing : MonoBehaviour
{
    [Header("Agent Setting")]
    public Agent_Setting robot;

    [Header("Display Setting")]
    public bool drawTrajectory = false;
    LineRenderer lineRenderer;

    [Header("Map Info")]
    bool noObstacle;
    public GridMap grid;
    public Voronoi_Map voronoi_Map;

    [Header("Twist")]
    public float velocity;
    public float omega;
    public float pushingConstraint = 0; 
    //public int index = 0;
    Transform _trans;
    float rob_Length;
    float rob_Acc;
    float rob_Ang_Acc;
    float rob_Max_Vel;
    float rob_Max_Omega;
    float pushing_Max_Vel;
    float pushing_Max_Omega;
    float senosr_Radius;

    [Header("DWA Weight")]
    public float heading_Weight = 1;
    public float dist_Weight = 0.25f;
    public float vel_Weight = 0.6f;
    public float goal_Weight = 0.2f;

    [Header("DWA_Pushing Weight")]
    public float pushing_Heading_Weight = 1;
    public float pushing_Dist_Weight = 0.25f;
    public float pushing_Vel_Weight = 0.6f;
    public float pushing_Goal_Weight = 0.2f;
    public float pushing_GoalHeading_Weight = 0.2f;

    const float dt = 0.05f;
    const float tau = 0.5f;
    const float pushing_Tau = 1f;
    const float deltaVel = 0.02f;
    const float deltaOmega = 0.02f;
    const float pos_Accuracy = 0.01f;
    //const float next_Point = 0.005f;
    const float pushing_Accuracy = 0.08f;
    const float pushing_heading_Accuracy = 2;
    const float stable_Vel = 0.32f;


#region For DWA Process
    Vector2[] path_Points;
    List<float> dwa_Heading = new List<float>();
    //List<float> dwa_Heading_Push = new List<float>();
    List<float> dwa_Dist = new List<float>();
    List<Vector2> dwa_Twist = new List<Vector2>();
    List<int> dwa_Goal = new List<int>();
    List<float> dwa_HeadingGoal = new List<float>();
    float heading_Max = float.MinValue;
    float heading_Min = float.MaxValue;
    //float heading_Push_Max = float.MinValue;
    //float heading_Push_Min = float.MaxValue;
    float dist_Max = float.MinValue;
    float dist_Min = float.MaxValue;
    float vel_Max = float.MinValue;
    float vel_Min = float.MaxValue;
#endregion

    void Awake(){
        LineInit();
    }
    void LineInit(){
        float lineWidth = 0.02f;
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.startColor = Color.green;;
        lineRenderer.endColor  = Color.green;;
        lineRenderer.startWidth = lineWidth;
        lineRenderer.endWidth = lineWidth;
        lineRenderer.enabled = false;
    }

    void Start(){
        _trans = transform;
        // Agent Setting
        rob_Length = robot.agent_Size.x;
        rob_Acc = robot.Acceleration;
        rob_Ang_Acc = robot.Angular_Acc;
        rob_Max_Vel = robot.max_Vel;
        rob_Max_Omega = robot.max_Omega;
        senosr_Radius = robot.sensor_Radius;

        pushing_Max_Vel = robot.pushing_Max_Vel;
        pushing_Max_Omega = robot.pushing_Max_Omega;

        // Map
        noObstacle = StaticMap.noObstacle;
    }

    public void Call_DWA_Normal(Vector2 _cur_Pos, float _cur_Dir, Vector2 _tar_Pos, Vector2 _final_Pos, float _final_Dir, Vector2 _cur_Twist, bool _checkBox){
        //Debug.Log("Normal DWA");
        StopCoroutine("DWA_ForPushing");
        StopCoroutine("DWA_Noraml");
        object[] normal_Params = new object[5]{_cur_Pos, _cur_Dir, _tar_Pos, _cur_Twist, _checkBox};
        StartCoroutine("DWA_Normal", normal_Params);
        //StartCoroutine(DWA_Normal(_cur_Pos, _cur_Dir, _tar_Pos, _cur_Twist));
    }

    // public void Call_DWA_Pushing(Vector2 _cur_Pos, float _cur_Dir, robot_WayPoint[] _path, Vector2 _final_Pos, float _final_Dir, Vector2 _cur_Twist, bool _checkBox){
    //         StopCoroutine("DWA_Normal");
    //         StopCoroutine("DWA_ForPushing");
    //         object[] pushing_Params = new object[6]{_cur_Pos, _cur_Dir, _path, _final_Pos, _final_Dir, _cur_Twist};
    //         StartCoroutine("DWA_ForPushing", pushing_Params);
    // }
    public void Call_DWA_Pushing(Vector2 _cur_Pos, float _cur_Dir, Vector2 _tar_Pos, Vector2 _final_Pos, float _final_Dir, Vector2 _cur_Twist, bool _checkBox){
            StopCoroutine("DWA_Normal");
            StopCoroutine("DWA_ForPushing");
            object[] pushing_Params = new object[6]{_cur_Pos, _cur_Dir, _tar_Pos, _final_Pos, _final_Dir, _cur_Twist};
            StartCoroutine("DWA_ForPushing", pushing_Params);
    }


#region For Pushing Use
    //IEnumerator DWA_ForPushing(Vector2 _cur_Pos, float _cur_Dir, Vector2 _tar_Pos, Vector2 _final_Pos, float _final_Dir, Vector2 _cur_Twist)
    IEnumerator DWA_ForPushing(object[] _pushing_Params)
    {
        Vector2 _cur_Pos = (Vector2)_pushing_Params[0];
        float _cur_Dir = (float)_pushing_Params[1];
        //robot_WayPoint[] _path = (robot_WayPoint[])_pushing_Params[2];
        Vector2 _tar_Pos = (Vector2)_pushing_Params[2];
        Vector2 _final_Pos = (Vector2)_pushing_Params[3];
        float _final_Dir = (float)_pushing_Params[4];
        Vector2 _cur_Twist = (Vector2)_pushing_Params[5];

        // robot_WayPoint cur_Point = _path[GetCurPoint(_cur_Pos, _path)];
        // Vector2 _tar_Pos = cur_Point.pos;
        // float _curTar_Dir = Tools.AngleToWorld(cur_Point.theta); 

        InitDWA();
        int Count = 0;
        bool isCollide = false;
        Vector2 choose_Twist = Vector2.zero;
        
        // 根據當下的速度展開的可行範圍視窗
        float accept_Vel_Max = _cur_Twist.x + rob_Acc * pushing_Tau;
        float accept_Vel_Min = _cur_Twist.x - rob_Acc * pushing_Tau;
        float accept_Omega_Max = _cur_Twist.y + rob_Ang_Acc * pushing_Tau;
        float accept_Omega_Min = _cur_Twist.y - rob_Ang_Acc * pushing_Tau;

        // 根據自身條件限縮視窗範圍
        // 因為正在推箱子，所以把後退的速度鎖住了
        if(accept_Vel_Max >= pushing_Max_Vel){
            accept_Vel_Max = pushing_Max_Vel;
        }
        if(accept_Vel_Max <= 0){
            accept_Vel_Max = 0;
        }
        if(accept_Vel_Min <= 0){
            accept_Vel_Min = 0;
        }
        if(accept_Vel_Min >= robot.pushing_Max_Vel){
            accept_Vel_Min = robot.pushing_Max_Vel;
        }
        
        if(accept_Omega_Max >= pushing_Max_Omega){
            accept_Omega_Max = pushing_Max_Omega;
        }
        if(accept_Omega_Min <= -pushing_Max_Omega){
            accept_Omega_Min = -pushing_Max_Omega;
        }
        
        // 主體
        // 搜索視窗中的各個點作為可接受的速度與角速度
        for(float opt_Vel = 0; opt_Vel < accept_Vel_Max; opt_Vel += deltaVel){
            for(float opt_Omega = -rob_Max_Omega; opt_Omega < accept_Omega_Max; opt_Omega += deltaOmega){
                if(opt_Vel < accept_Vel_Min){
                    continue;
                }
                if(opt_Omega < accept_Omega_Min){
                    continue;
                }
                Count++;
                // 當機算量很大的時候，不要一幀內算完，減輕計算量
                if(Count % 500 == 0){
                    yield return null;
                }
                RobotData predict_Robot = Trajectory(opt_Vel, opt_Omega, _cur_Pos, _cur_Dir, pushing_Tau);
                IntVector2 predict_GridPos = StaticMap.CellPosFromWorld(predict_Robot.Pos);
                if(!StaticMap.isInGrid(predict_GridPos)){
                    Debug.Log("Out of Range");
                    continue;
                }
                float shortDist = ObstacleDistance(predict_GridPos);
                float min_Dist = 1.2f * (rob_Length/2);
                if(shortDist > min_Dist){
                    float dist_Max_Vel = 1.2f * Mathf.Sqrt(2 * rob_Acc * shortDist);
                    if(opt_Vel <= dist_Max_Vel){
                        // Stable Pushing Constraints
                        float radius = Mathf.Abs(opt_Vel / opt_Omega);
                        if(radius > 100){
                            radius = 9999;
                        }
                        //Debug.Log(radius);
                        if(radius < pushingConstraint){
                            //Debug.Log("Out");
                            continue;
                        }else if(radius > pushingConstraint * 1.8f){
                            if(radius != 9999){
                                //Debug.Log("Out");
                                continue;
                            }
                            //Debug.Log("Infinity");
                        }
                        float dist_ToFinal = Vector2.Distance(predict_Robot.Pos, _final_Pos);
                        HeadingCostTotal_Pushing(predict_Robot, dist_ToFinal, _tar_Pos);
                        DistCostTotal_Pushing(predict_GridPos);
                        VelCostTotal(opt_Vel, opt_Omega);
                        GoalCost_Pushing(predict_Robot, dist_ToFinal, _final_Pos);
                        //EveryHeadingCost_(predict_Robot.Dir, _curTar_Dir);
                        GoalHeadingCost(predict_Robot.Dir, dist_ToFinal, _final_Dir);
                    }else{
                        Debug.Log("Will Collide!");
                        continue;
                    }
                }else{
                    Debug.Log("Is Collide");
                    isCollide = true;
                    break;
                }
            }
        }
        //Debug.Log("Count: " + Count);
        if(!isCollide){
            choose_Twist = CostFunction_Push();
        }
        velocity = choose_Twist.x;
        omega = choose_Twist.y;
        if(drawTrajectory){
            Trajectory(velocity, omega, _cur_Pos, _cur_Dir, pushing_Tau);
            DrawTrajectory();
        }
        yield break;
    }

    // int GetCurPoint(Vector2 _cur_Pos, robot_WayPoint[] _path){
    //     if(_path.Length == 1){
    //         return index;
    //     }else{
    //         if(index == 0){
    //             if(_path.Length >= 3){
    //                 index = 2;
    //             }else{
    //                 index = _path.Length - 1;
    //             }
    //         }else{
    //             float dist = Vector2.Distance(_cur_Pos, _path[index].pos);
    //             if(dist < next_Point){
    //                 if(index < _path.Length - 1){
    //                     index++;
    //                 }else{
    //                     index = _path.Length - 1;
    //                 }
    //             }
    //         }
    //     }
    //     return index;
    // }

    // public void InitIndex(){
    //     index = 0;
    // }

    //For Pushing Use
    Vector2 CostFunction_Push(){
        Vector2 final_Twist = Vector2.zero;
        float cost_Value = float.MinValue;

        for(int i = 0; i < dwa_Twist.Count; i++){
            //float heading_Cost = (dwa_Heading[i] - heading_Min) / (heading_Max - heading_Min);
            float heading_Cost = dwa_Heading[i];
            //Debug.Log("Heading: " + heading_Cost);
            
            float dist_Cost = 0;
            if(!noObstacle){
                if(dist_Max != dist_Min){
                    dist_Cost = (dwa_Dist[i] - dist_Min) / (dist_Max - dist_Min);
                }else{
                    dist_Cost = 0;
                }
                if(dist_Cost > 1){
                    dist_Cost = 1;
                }
            }
            //Debug.Log("Dist: " + dist_Cost);

            float vel_Cost = (stable_Vel - Mathf.Abs(dwa_Twist[i].x - stable_Vel)) / (stable_Vel);
            //Debug.Log("vel_Cost: " + vel_Cost + "  dwa_Twist: " + dwa_Twist[i].x );
            float goal_Cost = dwa_Goal[i];
            float goalHeading_Cost = dwa_HeadingGoal[i];
            //Debug.Log(goalHeading_Cost);
            float cost_Sum = pushing_Heading_Weight * heading_Cost + pushing_Dist_Weight * dist_Cost + pushing_Vel_Weight * vel_Cost + pushing_Goal_Weight * goal_Cost + pushing_GoalHeading_Weight * goalHeading_Cost;
            //float cost_Sum = pushing_Dist_Weight * dist_Cost + pushing_Vel_Weight * vel_Cost + pushing_Goal_Weight * goal_Cost + pushing_GoalHeading_Weight * goalHeading_Cost;
            //Debug.Log("Heading: " + heading_Cost + "  Dist: " + dist_Cost + "  Vel: " + vel_Cost + "  Goal: " + goal_Cost + " vel: " + dwa_Twist[i].x + "  Omega: " + dwa_Twist[i].y) ;
            if(cost_Sum > cost_Value){
                cost_Value = cost_Sum;
                final_Twist = dwa_Twist[i];
            }
        }
        //Debug.Log("twist: " + dwa_Twist[index].x);
        return final_Twist;
    }
    // void EveryHeadingCost_(float _predict_Headings, float _cur_Dir){
    //     //Debug.Log("final_Dir: " + _final_Dir);
    //     float goalHeading_Cost = 0;
    //     // if(_final_Dist < 0.3f){
    //         Vector2 predict_Heading = new Vector2(Mathf.Cos(_predict_Headings * Mathf.Deg2Rad), Mathf.Sin(_predict_Headings * Mathf.Deg2Rad)).normalized;
    //         Vector2 _final_Vec = new Vector2(Mathf.Cos(_cur_Dir * Mathf.Deg2Rad), Mathf.Sin(_cur_Dir * Mathf.Deg2Rad)).normalized;
            
    //         //float heading_Diff = Mathf.Acos(_final_Vec.x * predict_Heading.x + _final_Vec.y * predict_Heading.y);
    //         float dot_Value = _final_Vec.x * predict_Heading.x + _final_Vec.y * predict_Heading.y;
    //         goalHeading_Cost = dot_Value;
    //         // if(heading_Diff < pushing_heading_Accuracy){
    //         //     goalHeading_Cost = 1;
    //         // }else{
    //         //     goalHeading_Cost = 0;
    //         // }
    //     //}

    //     dwa_HeadingGoal.Add(goalHeading_Cost);
    // }

    void GoalHeadingCost(float _predict_Headings, float _final_Dist, float _final_Dir){
        //Debug.Log("final_Dir: " + _final_Dir);
        float goalHeading_Cost = 0;
        if(_final_Dist < 0.3f){
            Vector2 predict_Heading = new Vector2(Mathf.Cos(_predict_Headings * Mathf.Deg2Rad), Mathf.Sin(_predict_Headings * Mathf.Deg2Rad)).normalized;
            Vector2 _final_Vec = new Vector2(Mathf.Cos(_final_Dir * Mathf.Deg2Rad), Mathf.Sin(_final_Dir * Mathf.Deg2Rad)).normalized;
            
            //float heading_Diff = Mathf.Acos(_final_Vec.x * predict_Heading.x + _final_Vec.y * predict_Heading.y);
            float dot_Value = _final_Vec.x * predict_Heading.x + _final_Vec.y * predict_Heading.y;
            goalHeading_Cost = dot_Value;
            // if(heading_Diff < pushing_heading_Accuracy){
            //     goalHeading_Cost = 1;
            // }else{
            //     goalHeading_Cost = 0;
            // }
        }

        dwa_HeadingGoal.Add(goalHeading_Cost);
    }
    

    void HeadingCostTotal_Pushing(RobotData _predict_Rob, float _final_Dist, Vector2 _goal){

        Vector2 predict_Heading = new Vector2(Mathf.Cos(_predict_Rob.Dir * Mathf.Deg2Rad), Mathf.Sin(_predict_Rob.Dir * Mathf.Deg2Rad)).normalized;
        Vector2 dirToTarget = new Vector2(_goal.x - _predict_Rob.Pos.x, _goal.y - _predict_Rob.Pos.y).normalized;
        float dot_Value = dirToTarget.x * predict_Heading.x + dirToTarget.y * predict_Heading.y;
        float cost = dot_Value;
        
        if(_final_Dist < 0.3f){
            cost = 0;
        }
        dwa_Heading.Add(cost);

        if(cost > heading_Max){
            heading_Max = cost;
        }
        if(cost < heading_Min){
            heading_Min = cost;
        }
    }

    void DistCostTotal_Pushing(IntVector2 _gridPos){
        if(!noObstacle){
            // float penalty = grid.grid[_gridPos.x, _gridPos.z].inflationLayer + grid.grid[_gridPos.x, _gridPos.z].boxLayer;
            float dist_Cost = voronoi_Map.Voronoi_grid[_gridPos.x, _gridPos.z].distanceToClosestObstacle;

            if(dist_Cost < 2 * rob_Length){
                dwa_Dist.Add(dist_Cost);
            }else{
                dist_Cost = 5 * rob_Length;
                dwa_Dist.Add(dist_Cost); 
            }
            
            if(dist_Cost > dist_Max){
                dist_Max = dist_Cost;
            }
            if(dist_Cost < dist_Min){
                dist_Min = dist_Cost;
            }
        }else{
            float dist_Cost = 999;
            dwa_Dist.Add(dist_Cost);    
        }
    }


// TODO:
// 要丟入所有pushing 的點朝向，通過以後才換下一個
// 確保所有的點都盡可能符合規劃好地朝向
    void GoalCost_Pushing(RobotData _predict_Pos, float _final_Dist, Vector2 _tar){
        float dist_ToGoal = Vector2.Distance(_predict_Pos.Pos, _tar);
        int goal_Cost = 0;
        if(_final_Dist < 0.4f){
            if(dist_ToGoal < pushing_Accuracy){
                goal_Cost = 1;
            }else{
                goal_Cost = 0;
            }
        }
        dwa_Goal.Add(goal_Cost);
    }
#endregion



#region Normal Use
    //IEnumerator DWA_Normal(Vector2 _cur_Pos, float _cur_Dir, Vector2 _tar_Pos, Vector2 _cur_Twist)
    IEnumerator DWA_Normal(object[] _normal_Params)
    {
        Vector2 _cur_Pos = (Vector2)_normal_Params[0];
        float _cur_Dir = (float)_normal_Params[1];
        Vector2 _tar_Pos = (Vector2)_normal_Params[2];
        Vector2 _cur_Twist = (Vector2)_normal_Params[3];
        bool _checkBox = (bool)_normal_Params[4];

        InitDWA();
        int Count = 0;
        bool isCollide = false;
        Vector2 choose_Twist = Vector2.zero;
        
        // 根據當下的速度展開的可行範圍視窗
        float accept_Vel_Max = _cur_Twist.x + rob_Acc * tau;
        float accept_Vel_Min = _cur_Twist.x - rob_Acc * tau;
        float accept_Omega_Max = _cur_Twist.y + rob_Ang_Acc * tau;
        float accept_Omega_Min = _cur_Twist.y - rob_Ang_Acc * tau;

        // 根據自身條件限縮視窗範圍
        if(accept_Vel_Max >= rob_Max_Vel){
            accept_Vel_Max = rob_Max_Vel;
        }
        if(accept_Vel_Max <= -rob_Max_Vel){
            accept_Vel_Max = -rob_Max_Vel;
        }
        if(accept_Vel_Min <= -rob_Max_Vel){
            accept_Vel_Min = -rob_Max_Vel;
        }
        if(accept_Vel_Min >= rob_Max_Vel){
            accept_Vel_Min = rob_Max_Vel;
        }
        
        if(accept_Omega_Max >= rob_Max_Omega){
            accept_Omega_Max = rob_Max_Omega;
        }
        if(accept_Omega_Min <= -rob_Max_Omega){
            accept_Omega_Min = -rob_Max_Omega;
        }
        
        // 主體
        // 搜索視窗中的各個點作為可接受的速度與角速度
        for(float opt_Vel = -rob_Max_Vel; opt_Vel < accept_Vel_Max; opt_Vel += deltaVel){
            for(float opt_Omega = -rob_Max_Omega; opt_Omega < accept_Omega_Max; opt_Omega += deltaOmega){
                if(opt_Vel < accept_Vel_Min){
                    continue;
                }
                if(opt_Omega < accept_Omega_Min){
                    continue;
                }
                Count++;
                // 當機算量很大的時候，不要一幀內算完
                // 減輕計算量
                if(Count % 500 == 0){
                    yield return null;
                }
                RobotData predict_Robot = Trajectory(opt_Vel, opt_Omega, _cur_Pos, _cur_Dir, tau);
                IntVector2 predict_GridPos = StaticMap.CellPosFromWorld(predict_Robot.Pos);
                if(!StaticMap.isInGrid(predict_GridPos)){
                    Debug.Log("Out of Range");
                    continue;
                }
                float shortDist = ObstacleDistance(predict_GridPos);
                float min_Dist = 1.2f * (rob_Length/2);
                if(shortDist > min_Dist){
                    float dist_Max_Vel = 1.2f * Mathf.Sqrt(2 * rob_Acc * shortDist);
                    if(opt_Vel <= dist_Max_Vel){
                        HeadingCostTotal(_tar_Pos, predict_Robot, opt_Vel);
                        DistCostTotal(predict_GridPos, _checkBox);
                        VelCostTotal(opt_Vel, opt_Omega);
                        GoalCost(predict_Robot.Pos, _tar_Pos);

                    }else{
                        Debug.Log("Will Collide!");
                        continue;
                    }
                }else{
                    Debug.Log("Is Collide");
                    isCollide = true;
                    break;
                }
            }
        }

        if(!isCollide){
            choose_Twist = CostFunction();
        }
        velocity = choose_Twist.x;
        omega = choose_Twist.y;
        if(drawTrajectory){
            Trajectory(velocity, omega, _cur_Pos, _cur_Dir, tau);
            DrawTrajectory();
        }
        yield break;
    }

    Vector2 CostFunction(){
        Vector2 final_Twist = Vector2.zero;
        float cost_Value = float.MinValue;

        for(int i = 0; i < dwa_Twist.Count; i++){
            //float heading_Cost = (dwa_Heading[i] - heading_Min) / (heading_Max - heading_Min);
            float heading_Cost = dwa_Heading[i];
            float dist_Cost = 0;
            if(!noObstacle){
                if(dist_Max != dist_Min){
                    dist_Cost = (dwa_Dist[i] - dist_Min) / (dist_Max - dist_Min);
                }else{
                    dist_Cost = 1;
                }
                if(dist_Cost > 1){
                    dist_Cost = 1;
                }
            }
            float vel_Cost = (stable_Vel - Mathf.Abs(Mathf.Abs(dwa_Twist[i].x) - stable_Vel)) / stable_Vel;
            float goal_Cost = dwa_Goal[i];
            
            float cost_Sum = heading_Weight * heading_Cost + dist_Weight * dist_Cost + vel_Weight * vel_Cost + goal_Weight * goal_Cost;   
            if(cost_Sum > cost_Value){
                cost_Value = cost_Sum;
                final_Twist = dwa_Twist[i];
            }
        }
        return final_Twist;
    }

    void GoalCost(Vector2 _predict_Pos, Vector2 _tar){
        float dist_ToGoal = Vector2.Distance(_predict_Pos, _tar);
        int goal_Cost = 0;
        if(dist_ToGoal < pos_Accuracy){
            goal_Cost = 1;
            dist_ToGoal = 0;
        }else{
            goal_Cost = 0;
        }
        dwa_Goal.Add(goal_Cost);
    }

    void VelCostTotal(float _vel, float _omega){
        Vector2 twist = new Vector2(_vel, _omega);
        dwa_Twist.Add(twist);

        float vel_Abs = Mathf.Abs(twist.x);
        if(vel_Abs > vel_Max){
            vel_Max = vel_Abs;
        }
        if(vel_Abs < vel_Min){
            vel_Min = vel_Abs;
        }
    }

    void DistCostTotal(IntVector2 _gridPos, bool _checkBox){
        if(!noObstacle){
            float penalty = 0;
            float weight = 1;
            if(_checkBox){
                //penalty = 0.6f * grid.grid[_gridPos.x, _gridPos.z].inflationLayer + grid.grid[_gridPos.x, _gridPos.z].boxLayer;
                penalty = 0.2f * grid.grid[_gridPos.x, _gridPos.z].inflationLayer + grid.grid[_gridPos.x, _gridPos.z].boxLayer;
                weight = 1f;
            }
            float dist_Cost = weight * voronoi_Map.Voronoi_grid[_gridPos.x, _gridPos.z].distanceToClosestObstacle - 0.006f * penalty;
            if(dist_Cost < 0){
                dist_Cost = 0;
            }
            //Debug.Log("Penalty: " + penalty);

            if(dist_Cost < 2 * rob_Length){
                dwa_Dist.Add(dist_Cost);
            }else{
                dist_Cost = 5 * rob_Length;
                dwa_Dist.Add(dist_Cost); 
            }
            
            if(dist_Cost > dist_Max){
                dist_Max = dist_Cost;
            }
            if(dist_Cost < dist_Min){
                dist_Min = dist_Cost;
            }
        }else{
            float dist_Cost = 999;
            dwa_Dist.Add(dist_Cost);    
        }
    }

    // For Normal Use
    void HeadingCostTotal(Vector2 _goal, RobotData _predict_Rob, float _vel){
        Vector2 dirToTarget = new Vector2(_goal.x - _predict_Rob.Pos.x, _goal.y - _predict_Rob.Pos.y).normalized;
        Vector2 predict_Heading = new Vector2(Mathf.Cos(_predict_Rob.Dir * Mathf.Deg2Rad), Mathf.Sin(_predict_Rob.Dir * Mathf.Deg2Rad)).normalized;
        if(_vel < 0){
            predict_Heading = -predict_Heading;
        }
        float dot_Value = dirToTarget.x * predict_Heading.x + dirToTarget.y * predict_Heading.y;
        float cost = dot_Value;

        dwa_Heading.Add(cost);
        if(cost > heading_Max){
            heading_Max = cost;
        }
        if(cost < heading_Min){
            heading_Min = cost;
        }
    }

    float ObstacleDistance(IntVector2 _gridPos){
        if(!noObstacle){
            return voronoi_Map.Voronoi_grid[_gridPos.x, _gridPos.z].distanceToClosestObstacle;
        }else{
            return 999;
        }
    }
#endregion


    public RobotData Trajectory(float _vel, float _omega, Vector2 _curPos, float _curDir, float _tau){
        float new_X, new_Y;
        RobotData new_RobotPos;
        Vector2[] temp_Points;

        float temp_Vel = (float)Math.Round(_vel, 4);
        float temp_Omega = (float)Math.Round(_omega, 4);
        float temp_Heading = _curDir * Mathf.Deg2Rad;

        if(temp_Omega == 0 && temp_Vel != 0){
            new_X = _curPos.x + _vel * Mathf.Cos(temp_Heading) * _tau;
            new_Y = _curPos.y + _vel * Mathf.Sin(temp_Heading) * _tau; 
            new_RobotPos.Pos = new Vector2(new_X, new_Y);
            new_RobotPos.Dir = _curDir;
            
            temp_Points = new Vector2[2]{_curPos, new_RobotPos.Pos};      
        }
        else if(temp_Vel == 0 && temp_Omega != 0){
            new_RobotPos.Pos = _curPos;
            new_RobotPos.Dir = _curDir + _omega * Mathf.Rad2Deg * _tau;

            temp_Points = new Vector2[2]{_curPos, new_RobotPos.Pos};
        }
        else if(temp_Vel == 0 && temp_Omega == 0){
            new_RobotPos.Pos = _curPos;
            new_RobotPos.Dir = _curDir;

            temp_Points = new Vector2[1]{_curPos};
        }
        else{
            float r = _vel / _omega;
            new_X = _curPos.x + r * (Mathf.Sin((_omega * _tau) + temp_Heading) - Mathf.Sin(temp_Heading));
            new_Y = _curPos.y - r * (Mathf.Cos((_omega * _tau) + temp_Heading) - Mathf.Cos(temp_Heading));
            new_RobotPos.Pos = new Vector2(new_X, new_Y);
            new_RobotPos.Dir = _curDir + _omega * Mathf.Rad2Deg * _tau;

            int points_Length = (int)(_tau/dt);
            temp_Points = new Vector2[points_Length];
            for(int i = 0; i < points_Length; i++){
                float cur_X = _curPos.x + r * (Mathf.Sin((_omega * (dt * i)) + temp_Heading) - Mathf.Sin(temp_Heading));
                float cur_Y = _curPos.y - r * (Mathf.Cos((_omega * (dt * i)) + temp_Heading) - Mathf.Cos(temp_Heading));
                temp_Points[i] = new Vector2(cur_X, cur_Y);
            }
        }
        path_Points = temp_Points;
        return new_RobotPos;
    }
    void DrawTrajectory(){
        lineRenderer.enabled  = true;
        lineRenderer.positionCount = path_Points.Length;
        Vector3[] points = new Vector3[path_Points.Length];
        for(int i = 0; i < path_Points.Length; i++){
            points[i] = new Vector3(path_Points[i].x, _trans.localPosition.y, path_Points[i].y);
        }
        lineRenderer.SetPositions(points);
    }
    void InitDWA(){
        dwa_Heading.Clear();
        dwa_Dist.Clear();
        dwa_Twist.Clear();
        dwa_Goal.Clear();
        //dwa_Heading_Push.Clear();
        dwa_HeadingGoal.Clear();
        heading_Max = float.MinValue;
        heading_Min = float.MaxValue;
        //heading_Push_Max = float.MinValue;
        //heading_Push_Min = float.MaxValue;
        dist_Max = float.MinValue;
        dist_Min = float.MaxValue;
        vel_Max = float.MinValue;
        vel_Min = float.MaxValue;
    }
    public struct RobotData{
        public Vector2 Pos;
        public float Dir;
        public RobotData(Vector2 _Pos, float _Dir){
            Pos = _Pos;
            Dir = _Dir;
        }
    }


}
