using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DWA_Planner : MonoBehaviour
{
    [Header("Agent Setting")]
    public Agent_Setting robot;
    [Header("Display Setting")]
    public bool drawTrajectory;
    LineRenderer lineRenderer;
    [Header("Map Info")]
    public GridMap grid;
    bool noObstacle;
    public Voronoi_Map voronoi_Map;
    Voronoi_Node[,] obstacle_Dist_Map;
    // public LayerMask obstacleLayer;
    [Header("Twists")]
    public float velocity;
    public float omega;

#region StablePushing
    bool useStablePushing;
    [Header("PushingState")]
    //  我在想這部分也許可以放到 scriptable 裡面
    public float pushingConstraint;
    public bool withBox = false;
#endregion

#region Robot Property
    Transform _transform;
    float rob_Length;
    float rob_Acc;
    float rob_Ang_Acc; 
    float rob_MAX_VEL;
    float rob_MAX_OMEGA;
    float sensor_Radius;
#endregion

#region DWA Weight
    [Header("DWA Weight")]
    public float heading_Weight = 1;
    public float dist_Weight = 1.2f;
    public float vel_Weight = 0.8f;
#endregion


#region DWA Setting
    float dt = 0.05f;
    float tau = 0.5f;  // 20 * dt
    float deltaVel = 0.02f;
    float deltaOmega = 0.02f;
#endregion


#region DWA Process
    Vector2[] path_Points;
    List<float> dwa_Heading = new List<float>();
    List<float> dwa_Dist = new List<float>();
    List<Vector2> dwa_Twist = new List<Vector2>();

    float heading_Max = -1000;
    float heading_Min = 1000;
    float dist_Max = -1000;
    float dist_Min = 1000;
    float dist_Sum;
    float vel_Max = -1000;
    float vel_Min = 1000;
#endregion

    void Awake(){
        LineInit();
    }
    void Start(){
        _transform = transform;
        //  Agent Setting
        rob_Length = robot.agent_Size.x;
        rob_Acc = robot.Acceleration;
        rob_Ang_Acc = robot.Angular_Acc;
        rob_MAX_VEL = robot.max_Vel;
        rob_MAX_OMEGA = robot.max_Omega;

        sensor_Radius = robot.sensor_Radius;
        //  If Static Map dont have any Obstacle
        noObstacle = StaticMap.noObstacle;
        if(!noObstacle){
            if(voronoi_Map.Voronoi_grid != null){
                obstacle_Dist_Map = voronoi_Map.Voronoi_grid;
            }   
        }else{ 
            obstacle_Dist_Map = null;
        }
    }

    public void Call_DWA(Vector2 rob_Pos, float rob_Dir, Vector2 tar_Point, Vector2 _twist, float _safety_Dist, bool _useStablePushing){
        StartCoroutine(DWA(rob_Pos, rob_Dir, tar_Point, _twist, _safety_Dist, _useStablePushing));
    }

    IEnumerator DWA(Vector2 rob_Pos, float rob_Dir, Vector2 tar_Point, Vector2 _twist, float _safety_Dist, bool _useStablePushing)
    {
        useStablePushing = _useStablePushing;
        dwa_Dist.Clear();
        dwa_Heading.Clear();
        dwa_Twist.Clear();
        heading_Max = -1000;
        heading_Min = 1000;
        dist_Max = -1000;
        dist_Min = 1000;
        vel_Max = -1000;
        vel_Min = 1000;
        dist_Sum = 0;
        
        int count = 0;
        bool isCollide = false;
        Vector2 choose_Twist = new Vector2(-999, -999);

        float velocityMax = _twist.x + rob_Acc * tau;
        float velocityMin = _twist.x - rob_Acc * tau;
        float omegaMax = _twist.y + rob_Ang_Acc * tau;
        float omegaMin = _twist.y - rob_Ang_Acc * tau;

        if(velocityMax >= rob_MAX_VEL){
            velocityMax = rob_MAX_VEL;
        }
        if(velocityMax <= -rob_MAX_VEL){
            velocityMax = -rob_MAX_VEL;
        }
        if(velocityMin <= -rob_MAX_VEL){
            velocityMin = -rob_MAX_VEL;
        }
        if(velocityMin >= rob_MAX_VEL){
            velocityMin = rob_MAX_VEL;
        }
        
        if(omegaMax >= rob_MAX_OMEGA){
            omegaMax = rob_MAX_OMEGA;
        }
        if(omegaMin <= -rob_MAX_OMEGA){
            omegaMin = -rob_MAX_OMEGA;
        }
       
        //  Dynamic Windows: Vd
        //  Generate the window base on the current Twists.
        for(float opt_Vel = velocityMin; opt_Vel < velocityMax; opt_Vel += deltaVel){
            for(float opt_Omega = omegaMin; opt_Omega < omegaMax; opt_Omega += deltaOmega){
                count++;
                if(count % 1200 == 0){
                    yield return null;
                }
               
                // Predict the new state.
                RobotData predict_Robot = Trajectory(opt_Vel, opt_Omega, rob_Pos, rob_Dir);
                //  Admissible Velocity: Va
                //  Robot current state, to calculate the admissible velocity. 
                IntVector2 predict_GridPos = StaticMap.CellPosFromWorld(predict_Robot.Pos);
                float shortDistance = ObstacleDistance(predict_GridPos);

                //  Check if the robot hit the obstacle
                //float min_Dist = rob_Length/2;  //!!!!!!!!!!!!!!!!!!!!!!!!!這裡這個要能夠變
                float min_Dist = _safety_Dist; 
                if(shortDistance > min_Dist){  
                    // 為什麼這裡要用 2?
                    float min_Vel = 2 * Mathf.Sqrt(2 * rob_Acc * shortDistance);
                    if(opt_Vel < min_Vel){

                        # region StablePushing Constraint
                        if(useStablePushing){
                            float radius = opt_Vel/opt_Omega;
                            if(Math.Round(opt_Omega, 5) == 0){
                                radius = 9999;
                            }
                            if(radius < 0){
                                if(radius > -pushingConstraint){
                                    continue;
                                }
                            }else{
                                if(radius < pushingConstraint){
                                    continue;
                                }
                            }
                        }
                        #endregion

                        //  Calculate the Heading cost;
                        HeadingCostTotal(tar_Point, predict_Robot, opt_Vel);
                        
                        //  Calculate the distance between prediction & obstacle
                        //  Grid Penalty + Inflation Layer
                        if(!noObstacle){
                            int penalty = grid.grid[predict_GridPos.x, predict_GridPos.z].penalty + grid.grid[predict_GridPos.x, predict_GridPos.z].inflationLayer;
                            //float dist_Cost = voronoi_Map.Voronoi_grid[predict_GridPos.x, predict_GridPos.z].distanceToClosestObstacle - 0.03f * penalty;
                            float dist_Cost = voronoi_Map.Voronoi_grid[predict_GridPos.x, predict_GridPos.z].distanceToClosestObstacle;
                            dwa_Dist.Add(dist_Cost);
                            if(dist_Cost > dist_Max){
                                dist_Max = dist_Cost;
                            }
                            if(dist_Cost < dist_Min){
                                dist_Min = dist_Cost;
                            }
                            dist_Sum += dist_Cost;
                        }else{
                            dist_Sum = 0;
                        }
                        //Debug.Log("dist_Cost: " + dist_Cost + "dist_Sum: " + dist_Sum);
                        
                        //  Store all possible Twists
                        Vector2 twist = Vector2.right * opt_Vel + Vector2.up * opt_Omega;
                        dwa_Twist.Add(twist);
                        if(Mathf.Abs(twist.x) > vel_Max){
                            vel_Max = Mathf.Abs(twist.x);
                        }
                        if(Mathf.Abs(twist.x) < vel_Min){
                            vel_Min = Mathf.Abs(twist.x);
                        }
                    }else{
                        Debug.Log("will Collided");  
                        continue;
                    }
                }else{
                    Debug.Log("is collided");
                    isCollide = true;
                }
            }
        }

        if(isCollide != true){
            choose_Twist = CostFunction();
        }else{
            Debug.Log("is collided");
            choose_Twist = Vector2.zero;
        }
        velocity = choose_Twist.x;
        omega = choose_Twist.y;
        if(drawTrajectory){
            Trajectory(velocity, omega, rob_Pos, rob_Dir);
            DrawTrajectory();
        }
        //Debug.Log(count);
        yield break;
    }

    void DrawTrajectory(){
        lineRenderer.enabled  = true;
        lineRenderer.positionCount = path_Points.Length;
        Vector3[] points = new Vector3[path_Points.Length];
        for(int i = 0; i < path_Points.Length; i++){
            points[i] = new Vector3(path_Points[i].x, _transform.localPosition.y, path_Points[i].y);
        }
        lineRenderer.SetPositions(points);
    }

    float ObstacleDistance(IntVector2 _gridPos){
        //float shortDist = sensor_Radius;
        if(!noObstacle)
            return voronoi_Map.Voronoi_grid[_gridPos.x, _gridPos.z].distanceToClosestObstacle;
        else
            return 999;
        #region  old_Version
        // //  Calculate the shortest distance from obstacle to the robot.
        // //  Find the colliders
        // Vector3 pos = Tools.Vec2ToVec3(_curPos);
        // Collider[] colliders = Physics.OverlapSphere(pos, sensor_Radius, obstacleLayer);
        // //  Nothing Collide in sensor area!! == SAFE
        // if(colliders.Length == 0){
        //     shortDist = sensor_Radius;
        //     return shortDist;
        // }else{
        //     //  Something is going to collide in sensor area!! == check the distance
        //     //  This 0.1f is the Radius of the robot.
        //     bool isCollide;
        //     if(withBox){
        //         float box_Radius = 10;
        //         isCollide = Physics.CheckSphere(pos, box_Radius, obstacleLayer);
        //     }else{
        //         isCollide = Physics.CheckSphere(pos, rob_Length/2, obstacleLayer); //  The Second should change!!!! if robot is with the box!!!!!!!!!!!!!!!!
        //     }
           

        //     if(isCollide == true){
        //         shortDist = 0;
        //         return shortDist;
        //     }else{
        //         for(int i = 0; i < colliders.Length; i++){
        //             Vector3 closetPoint = colliders[i].ClosestPoint(_curPos);
        //             Vector2 closetPoint_Vec2 = new Vector2(closetPoint.x, closetPoint.z);
        //             float distance = Vector2.Distance(closetPoint_Vec2, _curPos);
        //             if(shortDist >= distance){
        //                 shortDist = distance;
        //             }
        //         }
        //         return shortDist;
        //     }
        // }
        #endregion
    }

    public struct RobotData{
        public Vector2 Pos;
        public float Dir;
        public RobotData(Vector2 _Pos, float _Dir){
            Pos = _Pos;
            Dir = _Dir;
        }
    }

    public RobotData Trajectory(float tar_velocity, float tar_omega, Vector2 curPosition, float curDir){
        float new_X;
        float new_Y;
        RobotData newPosition;
        Vector2[] temp_Points;

        float temp_Vel = Mathf.Abs((float)Math.Round(tar_velocity, 4));
        float temp_Omega = Mathf.Abs((float)Math.Round(tar_omega, 4));

        if(temp_Omega == 0 && temp_Vel != 0){
            new_X = curPosition.x + tar_velocity * Mathf.Cos(curDir * Mathf.Deg2Rad) * tau;
            new_Y = curPosition.y + tar_velocity * Mathf.Sin(curDir * Mathf.Deg2Rad) * tau; 
            //  New Posistion & Heading
            newPosition.Pos = new Vector2(new_X, new_Y);
            newPosition.Dir = curDir;

            //  Draw the Curve
            temp_Points = new Vector2[2]{
                curPosition,
                newPosition.Pos
            };
            path_Points = temp_Points;
            return newPosition;
        }
        else if(temp_Vel == 0 && temp_Omega != 0){  
            //  New Posistion & Heading
            newPosition.Pos = new Vector2(_transform.localPosition.x, _transform.localPosition.z);
            newPosition.Dir = curDir + tau * tar_omega * Mathf.Rad2Deg;
            //  Draw the Curve
            temp_Points = new Vector2[2]{
                curPosition,
                newPosition.Pos
            };
            path_Points = temp_Points;
            return newPosition;
        }else if(temp_Vel == 0 && temp_Omega == 0){
            //  New Posistion & Heading
            newPosition.Pos = new Vector2(_transform.localPosition.x, _transform.localPosition.z);
            newPosition.Dir = curDir;
            return newPosition;
        }else{
            new_X = curPosition.x + (tar_velocity/tar_omega) * (Mathf.Sin((tar_omega * tau) + curDir * Mathf.Deg2Rad) - Mathf.Sin(curDir * Mathf.Deg2Rad));
            new_Y = curPosition.y - (tar_velocity/tar_omega) * (Mathf.Cos((tar_omega * tau) + curDir * Mathf.Deg2Rad) - Mathf.Cos(curDir * Mathf.Deg2Rad));
            //  New Posistion & Heading
            newPosition.Pos = new Vector2(new_X, new_Y);
            newPosition.Dir = curDir + tau * tar_omega * Mathf.Rad2Deg;
            //  Draw the Curves
            
            int points_Length = (int)(tau/dt);
            temp_Points = new Vector2[points_Length];
            for(int i = 0; i < points_Length; i++){
                // Debug.Log(tau/dt + "  i: " + i);
                float cur_X = curPosition.x + (tar_velocity/tar_omega) * (Mathf.Sin((tar_omega * (dt * i)) + curDir * Mathf.Deg2Rad) - Mathf.Sin(curDir * Mathf.Deg2Rad));
                float cur_Y = curPosition.y - (tar_velocity/tar_omega) * (Mathf.Cos((tar_omega * (dt * i)) + curDir * Mathf.Deg2Rad) - Mathf.Cos(curDir * Mathf.Deg2Rad));
                temp_Points[i] = new Vector2(cur_X, cur_Y);
            }
            path_Points = temp_Points;
            return newPosition;
        }
    }


    void HeadingCostTotal(Vector2 goal, RobotData predict_Rob, float vel){
        Vector2 dirToTarget = (goal - predict_Rob.Pos).normalized;
        Vector2 thetaToVector = new Vector2(Mathf.Cos(predict_Rob.Dir * Mathf.Deg2Rad), Mathf.Sin(predict_Rob.Dir * Mathf.Deg2Rad));
        //Vector3 thetaToVector_R = Vector3.right * Mathf.Cos((predict_Rob.Dir + 180) * Mathf.Deg2Rad) + Vector3.forward * Mathf.Sin((predict_Rob.Dir + 180) * Mathf.Deg2Rad);
        Vector2 predict_Heading = (thetaToVector).normalized;
        //Vector3 predict_Heading_R = (thetaToVector_R).normalized;
        
        float angleDiff = Vector2.Angle(dirToTarget, predict_Heading);
        float angleDiff_R = Vector2.Angle(dirToTarget, -predict_Heading);
        float cost = 0;
        if(vel >= 0){
            cost = 180 - angleDiff;
            dwa_Heading.Add(cost);
        }else{
            cost = 180 - angleDiff_R;
            dwa_Heading.Add(cost);
        }    
        if(cost > heading_Max){
            heading_Max = cost;
        }
        if(cost < heading_Min){
            heading_Min = cost;
        }
    }

    Vector2 CostFunction(){
        Vector2 tar_Twist = Vector2.one;
        float costValue = -1000;
   
        for(int j = 0; j < dwa_Twist.Count; j++){
            //  Heading
            float heading_Cost = (dwa_Heading[j] - heading_Min) / (heading_Max - heading_Min);
            //  Distance
            float dist_Cost = 0;
            if(!noObstacle){
                dist_Cost = dwa_Dist[j];
            }
            
            if(dist_Sum == 0){
                dist_Cost = 0;
            }else{
                if(dist_Max == dist_Min){
                    dist_Cost =  dwa_Dist[j] / dist_Sum;
                }else{
                    dist_Cost = (dwa_Dist[j] - dist_Min) / (dist_Max - dist_Min);
                }
            }
            //Debug.Log(dist_Cost);

            //  Velocity
            float vel_Cost = (Mathf.Abs(dwa_Twist[j].x) - vel_Min) / (vel_Max - vel_Min);  
            //  Cost Function
            float cost_Sum = heading_Weight * heading_Cost + dist_Weight * dist_Cost + vel_Weight * vel_Cost;
            if(cost_Sum > costValue){
                costValue = cost_Sum;
                tar_Twist = dwa_Twist[j];   
            }
        }
        return tar_Twist;
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
}

