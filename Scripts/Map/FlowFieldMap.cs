using System.Collections;
using System.Collections.Generic;
using UnityEngine;



// 這邊事實上好像可以用繼承來寫，反正大致上 地圖的屬性都是相同的（這之後有機會再來改）
// FlowField 可能會用到很多次
// 有一個 FlowField 是用來生成 voronoi 的， 有一個是用來找 target 用在 Hybrid 的

public class FlowFieldMap : MonoBehaviour
{
    [Header ("Display Setting")]
    public bool showGrid = false;
    public bool showNumber = false;
    
    #region 地圖設定
    [HideInInspector] public FlowFieldNode[,] flowfield_Map;
    public GridMap gridMap;
    Cell[,] map;
    int length;
    float nodeRadius;
    Vector3 worldLeftBottom;
    #endregion


    private void Start() {
        transform.position = Vector3.zero;    

        worldLeftBottom = StaticMap.worldLeftBottom;
        nodeRadius = StaticMap.nodeRadius;
        map = StaticMap.base_Map;
        length = map.GetLength(0);

        CreateGrid();
    }

    public void CreateGrid(){
        flowfield_Map = new FlowFieldNode[length, length];
        for(int x = 0; x < length; x++){
            for(int y = 0; y < length; y++){
                flowfield_Map[x, y] = new FlowFieldNode(map[x, y]);
                flowfield_Map[x, y].penalty = gridMap.grid[x, y].penalty;
                flowfield_Map[x, y].inflationLayer = gridMap.grid[x, y].inflationLayer;
            }
        }
    }

    public HashSet<FlowFieldNode> GetNeighbours(FlowFieldNode node, bool _includeCorners){
        HashSet<FlowFieldNode> neighbours = new HashSet<FlowFieldNode>();
        for(int x = -1; x <= 1; x++){
            for(int y = -1; y <= 1; y++){
                if(x == 0 && y == 0){
                    continue;
                }
                if(!_includeCorners){
                    if(x == -1 && y == -1){ continue;}
                    if(x == -1 && y == 1){ continue;}
                    if(x == 1 && y == -1){ continue;}
                    if(x == 1 && y == 1){ continue;}
                }
                int checkX = node.grid_X + x;
                int checkY = node.grid_Y + y;
                if(checkX >= 0 && checkX < length && checkY >= 0 && checkY < length){
                    neighbours.Add(flowfield_Map[checkX, checkY]);
                }
            }
        }
        return neighbours;
    }

    // public void FindMaxValue(){
    //     foreach(FlowFieldNode n in flowfield_Map){
    //         if(n.totalCost > max_Value){
    //             max_Value = n.totalCost;
    //         }
    //     }
    // }

    void OnDrawGizmos(){
        if(flowfield_Map != null && showGrid){
            foreach(FlowFieldNode n in flowfield_Map){
                Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
                Gizmos.color = Color.Lerp(Color.green, Color.red, Mathf.InverseLerp(0, 999, n.totalCost));
                Gizmos.DrawCube(vec3 - Vector3.up * 1, Vector3.one * (2 * StaticMap.nodeRadius * 0.9f));
            }
        }
    }
}
