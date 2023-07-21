using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;


[CustomEditor(typeof(FlowFieldMap))]
public class ShowFlowField : Editor
{

    void OnSceneGUI()
    {
        FlowFieldMap map = target as FlowFieldMap;
        if(map == null || map.flowfield_Map == null){
            return;
        }
        if(!map.showNumber){
            return;
        }
        //Create Text on each checkpoint //
        foreach(FlowFieldNode n in map.flowfield_Map){
            Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
            UnityEditor.Handles.Label(vec3 + Vector3.up, n.totalCost.ToString());
        }
    }
}