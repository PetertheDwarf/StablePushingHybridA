using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestMask : MonoBehaviour
{
    public LayerMask obstacle;
    public bool showTest;

    Cell[,] test_Map;
    int gridSizeX;
    int gridSizeY;
    Vector2 worldLeftBottom;
    float nodeRadius;

    // Start is called before the first frame update
    void Start()
    {
        test_Map = StaticMap.base_Map;
        gridSizeX = StaticMap.gridSizeX;
        gridSizeY = StaticMap.gridSizeY;
        worldLeftBottom = StaticMap.worldLeftBottom;
        nodeRadius = StaticMap.nodeRadius;
    }

    // Update is called once per frame
    void Update()
    {
        for(int x = 0; x < gridSizeX; x++){
            for(int y = 0; y < gridSizeY; y++){
                Vector2 worldPoint = worldLeftBottom + Vector2.right * (x * 2 * nodeRadius + nodeRadius) + Vector2.up * (y * 2 * nodeRadius + nodeRadius);
                Vector3 worldPoint_vec3 = Tools.Vec2ToVec3(worldPoint);

                bool walkable = !(Physics.CheckSphere(worldPoint_vec3, nodeRadius, obstacle));
                if(!walkable){
                    test_Map[x, y].walkable = false;
                }else{
                    test_Map[x, y].walkable = true;
                }

            }
        }
    }
    void OnDrawGizmos(){
        if(test_Map != null && showTest){
            foreach(Cell n in test_Map){
                Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
                if(!n.walkable){
                    Gizmos.color = Color.black;
                }else{
                    Gizmos.color = Color.white;
                }
                Gizmos.DrawCube(vec3 - Vector3.up * 1, Vector3.one * (0.09f));
            }
        }
    }
}
