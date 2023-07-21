using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// 靜態地圖 Grid Map
// 這裡假設所有障礙物是靜止不動的（靜態障礙物）
// 因此這些地圖資訊並不需要重覆計算，是固定的。
// 最主要包含地圖的靜態障礙物，以及地圖本身的大小。
// 以 Cell 為單位，並生成一個固定的地圖，作為後續所有計算的基底，而不用重複生成地圖，或是分層地圖間的計算間相互干擾。

public class StaticMap : MonoBehaviour
{
    //地圖設定（固定的，不過因為要讓別人讀，所以這裡設成 public ）
    [HideInInspector] public static readonly IntVector2 gridWorldSize = new IntVector2(10, 10);
    [HideInInspector] public static readonly float nodeRadius = 0.04f;
    [HideInInspector] public static Vector2 worldLeftBottom;
    [HideInInspector] public static int gridSizeX;
    [HideInInspector] public static int gridSizeY;
    [HideInInspector] public static Cell[,] base_Map;
    [HideInInspector] public static bool noObstacle;


    [Header("Display Setting")]
    public bool showGrid = false;

    [Header("MapSetting")]
    public LayerMask unWalkableMask;


    void Awake(){
        transform.position = Vector3.zero;

        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / (2*nodeRadius));
        gridSizeY = Mathf.RoundToInt(gridWorldSize.z / (2*nodeRadius));
        worldLeftBottom = new Vector2(-(gridWorldSize.x / 2), -(gridWorldSize.z / 2));

        CreateMap();
    }

    // 最重要的建構初始地圖
    void CreateMap()
    {
        base_Map = new Cell[gridSizeX, gridSizeY];
        int unwalkable_Count = 0;

        for(int x = 0; x < gridSizeX; x++){
            for(int y = 0; y < gridSizeY; y++){
                Vector2 worldPoint = worldLeftBottom + Vector2.right * (x * 2 * nodeRadius + nodeRadius) + Vector2.up * (y * 2 * nodeRadius + nodeRadius);
                Vector3 worldPoint_vec3 = Tools.Vec2ToVec3(worldPoint);

                bool walkable = !(Physics.CheckSphere(worldPoint_vec3, nodeRadius, unWalkableMask));
                if(!walkable){
                    unwalkable_Count++;
                }

                base_Map[x,y] = new Cell(
                    _walkable: walkable,
                    _worldPosition: worldPoint,
                    _grid_X: x,
                    _grid_Y: y
                );
            }
        }
        if(unwalkable_Count == 0){
            noObstacle = true;
        }
        //Debug.Log(noObstacle);
    }

    public int MaxSize{
        get{
            return gridSizeX * gridSizeY;
        }
    }


    public static IntVector2 CellPosFromWorld(Vector2 _worldPosition){
        float x_Dist = _worldPosition.x - worldLeftBottom.x;
        float y_Dist = _worldPosition.y - worldLeftBottom.y;
        int x_Num = (int)(x_Dist / (2 * nodeRadius));
        int y_Num = (int)(y_Dist / (2 * nodeRadius));
        if(x_Dist < 0){
            x_Num = -1;
        }
        if(y_Dist < 0){
            y_Num = -1;
        }
        return new IntVector2(x_Num, y_Num);
    }

    public static bool isInGrid(IntVector2 _input){
        if(_input.x < 0 || _input.x >= gridSizeX){
            return false;
        }
        if(_input.z < 0 || _input.z >= gridSizeY){
            return false;
        }
        return true;
    }

    void OnDrawGizmos(){
        if(base_Map != null && showGrid){
            foreach(Cell c in base_Map){
                if(c.walkable){
                    Gizmos.color = Color.white;
                }else{  
                    Gizmos.color = Color.black;
                }
                Vector3 pos_Vec3 = Tools.Vec2ToVec3(c.worldPosition) - Vector3.up;
                Gizmos.DrawCube(pos_Vec3, Vector3.one * (2 * nodeRadius) * 0.9f);
            }
        }
    }
}
