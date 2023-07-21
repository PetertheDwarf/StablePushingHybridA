using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(Voronoi_Map))]
public class ShowVoronoi : Editor
{
    void OnSceneGUI()
    {
        Voronoi_Map vornoi_Map = target as Voronoi_Map;
        if(vornoi_Map == null || vornoi_Map.Voronoi_grid == null){
            return;
        }
        if(!vornoi_Map.showObstacleDist && !vornoi_Map.showEdgeDist){
            return;
        }
        //Create Text on each checkpoint //
        
        if(vornoi_Map.showEdgeDist){
            foreach(Voronoi_Node n in vornoi_Map.Voronoi_grid){
                Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
                if(n.distanceToClosestEdge > 999){
                    UnityEditor.Handles.Label(vec3 + Vector3.up, "Max");
                }else{
                    float value = (float)Math.Round(n.distanceToClosestEdge, 2);
                    UnityEditor.Handles.Label(vec3 + Vector3.up, value.ToString());
                }
                
                
            }
        }
        if(vornoi_Map.showObstacleDist){
            foreach(Voronoi_Node n in vornoi_Map.Voronoi_grid){
                Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
                float value = (float)Math.Round(n.distanceToClosestObstacle, 2);
                UnityEditor.Handles.Label(vec3 + Vector3.up, value.ToString());
            }
        }

    }
}
