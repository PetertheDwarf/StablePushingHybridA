using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(GridMap))]
public class ShowValue : Editor
{
    void OnSceneGUI()
    {
        GridMap map = target as GridMap;
        if(map == null || map.grid == null){
            return;
        }
        if(map.showFCost || map.showGCost || map.showHCost){
            //Create Text on each checkpoint //
            foreach(Node n in map.grid){
                Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
                if(map.showFCost){
                    UnityEditor.Handles.Label(vec3 + Vector3.up, n.fCost.ToString());
                }
                else if(map.showGCost){
                    UnityEditor.Handles.Label(vec3 + Vector3.up, n.gCost.ToString());
                }else{
                    UnityEditor.Handles.Label(vec3 + Vector3.up, n.hCost.ToString());
                }
                
            }    
        }else{
            return;
        }
    }
}
