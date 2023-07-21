using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestTrajectory : MonoBehaviour
{
    public GridMap grid;
    public Voronoi_Map voronoi_Map;
    LineRenderer lineRenderer;
    Transform _trans;
    public Transform _target;
    const float dt = 0.05f;
    const float tau = 0.8f;
    Vector2[] path_Points;
    // Start is called before the first frame update
    void Start()
    {
        _trans = this.transform;
        lineRenderer = GetComponent<LineRenderer>();
        
    }
    [Range(-3, 3)]
    public float vel = 0;
    [Range(-1, 1)]
    public float omega = 0;
    // Update is called once per frame
    void Update()
    {
        Vector2 tarPos = Tools.Vec3ToVec2(_target.localPosition);
        Vector2 curPos = Tools.Vec3ToVec2(_trans.localPosition);
        float curDir = Tools.AngleToWorld(_trans.localEulerAngles.y);
        RobotData cur_Robot = new RobotData(curPos, curDir);
        RobotData test = Trajectory(vel, omega, curPos, curDir);
        //Debug.Log("testPos: " + test.Pos.x + ", " + test.Pos.y);
        // //HeadingCostTotal(tarPos, cur_Robot, vel); 
        //DrawTrajectory();
        //HeadingCostTotal(tarPos, cur_Robot, 1f);
        IntVector2 test_Dist = StaticMap.CellPosFromWorld(curPos);
        DistCostTotal(test_Dist);
    }
    void HeadingCostTotal(Vector2 _goal, RobotData _predict_Rob, float _vel){
        Vector2 dirToTarget = new Vector2(_goal.x - _predict_Rob.Pos.x, _goal.y - _predict_Rob.Pos.y).normalized;
        Vector2 predict_Heading = new Vector2(Mathf.Cos(_predict_Rob.Dir * Mathf.Deg2Rad), Mathf.Sin(_predict_Rob.Dir * Mathf.Deg2Rad)).normalized;
        
        //float dot_Value = predict_Heading.x * dirToTarget.x + predict_Heading.y * dirToTarget.y;
        // float angleDiff = Vector2.Angle(dirToTarget, predict_Heading);
        // float angleDiff_R = Vector2.Angle(dirToTarget, -predict_Heading);
        
        
        float dot_Value_Abs = Mathf.Abs(dirToTarget.x * predict_Heading.x + dirToTarget.y * predict_Heading.y);
        float cost = dot_Value_Abs;
        
        Debug.Log("cost: " + cost);
        // dwa_Heading.Add(cost);
        // // if(_vel >= 0){
        // //     cost = 180 - angleDiff;
        // //     dwa_Heading.Add(cost);
        // // }else{
        // //     cost = 180 - angleDiff_R;
        // //     dwa_Heading.Add(cost);
        // // }
        // if(cost > heading_Max){
        //     heading_Max = cost;
        // }
        // if(cost < heading_Min){
        //     heading_Min = cost;
        // }
    }
    void DistCostTotal(IntVector2 _gridPos){

        int penalty = grid.grid[_gridPos.x, _gridPos.z].penalty + grid.grid[_gridPos.x, _gridPos.z].inflationLayer;
            //float dist_Cost = obstacle_Dist_Map[predict_GridPos.x, predict_GridPos.z].distanceToClosestObstacle - 0.03f * penalty;
        float dist_Cost = voronoi_Map.Voronoi_grid[_gridPos.x, _gridPos.z].distanceToClosestObstacle;
            //dwa_Dist.Add(dist_Cost);
            // if(dist_Cost > dist_Max){
            //     dist_Max = dist_Cost;
            // }
            // if(dist_Cost < dist_Min){
            //     dist_Min = dist_Cost;
            // }
        Debug.Log("Dist Cost: " + dist_Cost);
    }
    public struct RobotData{
        public Vector2 Pos;
        public float Dir;
        public RobotData(Vector2 _Pos, float _Dir){
            Pos = _Pos;
            Dir = _Dir;
        }
    }
    public RobotData Trajectory(float _vel, float _omega, Vector2 _curPos, float _curDir){
        float new_X, new_Y;
        RobotData new_RobotPos;
        Vector2[] temp_Points;

        float temp_Vel = (float)Math.Round(_vel, 4);
        float temp_Omega = (float)Math.Round(_omega, 4);
        float temp_Heading = _curDir * Mathf.Deg2Rad;

        if(temp_Omega == 0 && temp_Vel != 0){
            new_X = _curPos.x + _vel * Mathf.Cos(temp_Heading) * tau;
            new_Y = _curPos.y + _vel * Mathf.Sin(temp_Heading) * tau; 
            new_RobotPos.Pos = new Vector2(new_X, new_Y);
            new_RobotPos.Dir = _curDir;
            
            temp_Points = new Vector2[2]{_curPos, new_RobotPos.Pos};      
        }
        else if(temp_Vel == 0 && temp_Omega != 0){
            new_RobotPos.Pos = _curPos;
            new_RobotPos.Dir = _curDir + _omega * Mathf.Rad2Deg * tau;

            temp_Points = new Vector2[2]{_curPos, new_RobotPos.Pos};
        }
        else if(temp_Vel == 0 && temp_Omega == 0){
            new_RobotPos.Pos = _curPos;
            new_RobotPos.Dir = _curDir;

            temp_Points = new Vector2[1]{_curPos};
        }
        else{
            float r = _vel / _omega;
            new_X = _curPos.x + r * (Mathf.Sin((_omega * tau) + temp_Heading) - Mathf.Sin(temp_Heading));
            new_Y = _curPos.y - r * (Mathf.Cos((_omega * tau) + temp_Heading) - Mathf.Cos(temp_Heading));
            new_RobotPos.Pos = new Vector2(new_X, new_Y);
            new_RobotPos.Dir = _curDir + _omega * Mathf.Rad2Deg * tau;

            int points_Length = (int)(tau/dt);
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
}
