using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestTools : MonoBehaviour
{
    public Voronoi_Map voronoi_Map;
    public Transform cubeA;
    float scale_X, scale_Z, robot_Size;
    float min_Obstacle_Dist = 0.16f;
    public Transform cubeB;

    Vector3[] dubinPoint;

    Vector2[] corners = null;
    robot_WayPoint test_Side = null;
    Voronoi_Node[,] obstacleCost_Map;


    DubinsGeneratePaths dubinsGenerator;
    // Start is called before the first frame update
    void Start()
    {
        scale_X = cubeA.localScale.x;
        scale_Z = cubeA.localScale.z;
        robot_Size = 0.2f;
        dubinsGenerator = new DubinsGeneratePaths(1);
        
    }

    // Update is called once per frame
    void Update()
    {
        // float vec_X = Mathf.Sin(cubeA.eulerAngles.y * Mathf.Deg2Rad);
        // float vec_Y = Mathf.Cos(cubeA.eulerAngles.y * Mathf.Deg2Rad);
        // Vector2 a_forward = new Vector2(vec_X, vec_Y);
        // Vector2 a_To_B = Tools.Vec3ToVec2(cubeB.position - cubeA.position);
        // float angle = Tools.AngleBetweenVector2(a_forward, a_To_B);
        // Debug.Log(angle);


//         float cur_Heading = Tools.Angle360(cubeA.transform.eulerAngles.y);
// //        float headingDiff_ToGoal = Mathf.Abs(cubeB.transform.eulerAngles.y - cur_Heading);
//         float headingDiff_ToGoal = (cubeB.transform.eulerAngles.y - cur_Heading);
//         Debug.Log("heading Diff: " + headingDiff_ToGoal);
//         float dot_Value = Mathf.Abs(Tools.DotToCos(cubeA.transform.eulerAngles.y, cubeB.transform.eulerAngles.y));
//         Debug.Log("dot: " + dot_Value);
        // float dubins_Cost = 0;
        // OneDubinsPath[] dubinsToGoal = dubinsGenerator.GetAllDubinsPaths(
        //     startPos: cubeA.localPosition,
        //     startHeading: cubeA.eulerAngles.y * Mathf.Deg2Rad,
        //     goalPos: cubeB.localPosition,
        //     goalHeading: cubeB.transform.eulerAngles.y * Mathf.Deg2Rad
        // );
        // if(dubinsToGoal != null){
        //     List<OneDubinsPath> list_Dubins = new List<OneDubinsPath>();
        //     for(int i = 0; i < dubinsToGoal.Length; i++){
        //         list_Dubins.Add(dubinsToGoal[i]);
        //     }
        //     dubins_Cost = dubinsGenerator.FindShortestPathLength(list_Dubins);
        //     OneDubinsPath test_Dubins = dubinsGenerator.FindShortestPath(list_Dubins);
        //     dubins_Cost = test_Dubins.totalLength;
        //     dubinPoint = new Vector3[test_Dubins.pathCoordinates.Count];
        //     for(int i = 0; i < test_Dubins.pathCoordinates.Count; i++){
        //         dubinPoint[i] = test_Dubins.pathCoordinates[i]; 
        //     }
            
        // }
            
        // float dist = Vector3.Distance(cubeA.localPosition, cubeB.localPosition);
        // Debug.Log("Dubins: " + dubins_Cost + " Real_Dist: " + dist);

        if(Input.GetKeyDown("q")){
            obstacleCost_Map = voronoi_Map.Voronoi_grid;
            corners = new Vector2[4];
            corners = GetCornerPoints(Tools.Vec3ToVec2(cubeA.localPosition), cubeA.transform.localEulerAngles.y);
            bool collide = CheckCollision_Corners(corners);
            Node3D temp_Node = new Node3D(Tools.Vec3ToVec2(cubeA.localPosition), cubeA.transform.localEulerAngles.y);
            temp_Node.parent = temp_Node;
            temp_Node.dir_Index = 1;
            test_Side = GetChangeSidePoint(temp_Node);
            bool robot_Collide = CheckCollision_Robot(test_Side.pos, true);
        }

    }

    Vector2[] GetCornerPoints(Vector2 _cen_Point, float _heading){
        float l = scale_X/2;
        float w = scale_Z/2;
        Vector2[] corner_Points = new Vector2[4]{
            new Vector2(l, w), 
            new Vector2(l, -w), 
            new Vector2(-l, -w), 
            new Vector2(-l, w)
        };
        
        Vector2[] corner_Points_Rot = new Vector2[4];
        for(int i = 0; i < corner_Points.Length; i++){
            float new_X = corner_Points[i].x * Mathf.Cos(_heading * Mathf.Deg2Rad) + corner_Points[i].y * Mathf.Sin(_heading * Mathf.Deg2Rad);
            float new_Y = - corner_Points[i].x * Mathf.Sin(_heading * Mathf.Deg2Rad) + corner_Points[i].y * Mathf.Cos(_heading * Mathf.Deg2Rad);
            corner_Points_Rot[i] = new Vector2(_cen_Point.x + new_X, _cen_Point.y + new_Y);
            Debug.Log("Corners Points: " + corner_Points_Rot[i]);
            
        }
        return corner_Points_Rot;
    }

    bool CheckCollision_Corners(Vector2[] corner_Points){
        for(int i = 0; i < corner_Points.Length; i++){
            IntVector2 corner_GridPos = StaticMap.CellPosFromWorld(corner_Points[i]);
            if(!StaticMap.isInGrid(corner_GridPos)){
                //Debug.Log("work");
                return true;
            }
            //Debug.Log("GridPos: " + corner_GridPos.x + " , " + corner_GridPos.z);
            float obstacle_Dist = obstacleCost_Map[corner_GridPos.x, corner_GridPos.z].distanceToClosestObstacle;
           //Debug.Log("i: " + i + "  obstacle_Dist: " + obstacle_Dist);
            if(obstacle_Dist < min_Obstacle_Dist){
                return true;
            }
        }
        return false;
    }

    robot_WayPoint GetChangeSidePoint(Node3D _afterChange_Node){
        Node3D parent_Node = _afterChange_Node.parent;
        int cur_Node_Dir = _afterChange_Node.dir_Index;
        float cur_Heading = _afterChange_Node.theta;
        int parent_Dir = parent_Node.dir_Index;
        if(Mathf.Abs(cur_Node_Dir) == 10 || Mathf.Abs(cur_Node_Dir) == 20){
            // 找 parent 
            // 推側邊
            float new_X = (scale_X/2 + robot_Size/2) * Mathf.Cos(cur_Heading * Mathf.Deg2Rad);
            float new_Y = (scale_X/2 + robot_Size/2) * -Mathf.Sin(cur_Heading * Mathf.Deg2Rad);
            Vector2 side_Vec = new Vector2(new_X, new_Y);
            
            Vector2 side_Point = parent_Node.worldPosition - side_Vec;
            if(cur_Node_Dir > 0){
                side_Point = parent_Node.worldPosition + side_Vec;
            }
            return new robot_WayPoint(side_Point, cur_Heading, 2);
        }else{
            // 找 parent 
            // 推正面 或背面
            float new_X = (scale_Z/2 + robot_Size/2) * Mathf.Sin(cur_Heading * Mathf.Deg2Rad);
            float new_Y = (scale_Z/2 + robot_Size/2) * Mathf.Cos(cur_Heading * Mathf.Deg2Rad);
            Vector2 nor_Vec = new Vector2(new_X, new_Y);
            // Vector2 nor_Point = parent_Node.worldPosition + nor_Vec;
            // float theta = -parent_Node.theta;
            
            Vector2 nor_Point = parent_Node.worldPosition - nor_Vec;
            float theta = parent_Node.theta;
            
            return new robot_WayPoint(nor_Point, theta, 2);
        }
    }

    bool CheckCollision_Robot(Vector2 _robot_Pos, bool _changeSide){
        IntVector2 robot_GridPos = StaticMap.CellPosFromWorld(_robot_Pos);
        if(!StaticMap.isInGrid(robot_GridPos)){
            return true;
        }
        float weight = 2f;
        if(_changeSide){
            weight = 5f;
        }
        float obstacle_Dist = obstacleCost_Map[robot_GridPos.x, robot_GridPos.z].distanceToClosestObstacle;
        float dist = weight * robot_Size;
        Debug.Log("Robot obstacle_Dist: " + obstacle_Dist + " dist: " + dist);
        if(obstacle_Dist < dist){
            Debug.Log("true");
            return true;
        }else{
            Debug.Log("false");
            return false;
        }
    }

    void OnDrawGizmos(){
        if(dubinPoint != null){
            for(int i = 0; i < dubinPoint.Length; i++){
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(dubinPoint[i], 0.02f);
                //Vector3 pos = dubinPoint[i];
                //float dist = driveDistance * 0.5f;
                //Debug.Log("theta: " + dubinTheta.Count + "   point: " + dubinPoint.Count);
                ///float theta = Tools.AngleToWorld(dubinTheta[i]);
                //Vector3 dir = new Vector3(pos.x + dist * Mathf.Cos(theta * Mathf.Deg2Rad), pos.y, pos.z + dist * Mathf.Sin(theta * Mathf.Deg2Rad)); 
                //Gizmos.DrawLine(pos, dir);
            }
        }
        if(corners != null){
            for(int i = 0; i < corners.Length; i++){
                Gizmos.color = Color.yellow;
                Vector3 point = Tools.Vec2ToVec3(corners[i]);
                Gizmos.DrawSphere(point, 0.1f);  
            }
            if(test_Side != null){
                Gizmos.color = Color.cyan;
                Gizmos.DrawSphere(Tools.Vec2ToVec3(test_Side.pos), 0.1f); 
            }
        }
        
    }


}
