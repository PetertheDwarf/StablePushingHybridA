using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestMapInGrid : MonoBehaviour
{
    public Transform testCube;
    float scale_X, scale_Z;
    public Voronoi_Map voronoi_Map;
    Voronoi_Node[,] obstacleCost_Map;
    float min_Obstacle_Dist = 0.15f;

    Vector2[] corners = null; 
    // Start is called before the first frame update
    void Start()
    {
        scale_X = transform.localScale.x;
        scale_Z = transform.localScale.z;


        
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetKeyDown("q")){
            obstacleCost_Map = voronoi_Map.Voronoi_grid;
        }
        float heading = transform.eulerAngles.y;
        corners = GetCornerPoints(Tools.Vec3ToVec2(testCube.position), heading);
        bool isCollide = CheckCollision_Corners(corners);
        Debug.Log(isCollide);
        // IntVector2 gridPos = StaticMap.CellPosFromWorld(Tools.Vec3ToVec2(testCube.position));
        // Debug.Log("gridPos: " + gridPos.x + ", " + gridPos.z);
        // if(StaticMap.isInGrid(gridPos)){
        //     Debug.Log("Is In Grid");
        // }
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
        }
        return corner_Points_Rot;
    }
    
    // True means collide
    bool CheckCollision_Corners(Vector2[] corner_Points){
        for(int i = 0; i < corner_Points.Length; i++){
            IntVector2 corner_GridPos = StaticMap.CellPosFromWorld(corner_Points[i]);
            if(!StaticMap.isInGrid(corner_GridPos)){
                //Debug.Log("work");
                return true;
            }
            //Debug.Log("GridPos: " + corner_GridPos.x + " , " + corner_GridPos.z);
            float obstacle_Dist = obstacleCost_Map[corner_GridPos.x, corner_GridPos.z].distanceToClosestObstacle;
            if(obstacle_Dist < min_Obstacle_Dist){
                return true;
            }
        }
        return false;
    }

    void OnDrawGizmos(){
        if(corners != null){
            foreach(Vector2 c in corners){
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(Tools.Vec2ToVec3(c), 0.05f);
            }

        }
    }
}
