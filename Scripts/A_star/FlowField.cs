using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class FlowField : MonoBehaviour
{
    public FlowFieldMap flowfield_Map;
    private int length;
    
    //  int 0 = withoutObstacle; 1 = check Obstacle without layer; 2 = with layer;
    public void Generate_FlowField2(List<Vector2> _startPoints, bool _includeCorners, int _checkObstacle, int[,] _region)
    {
        #region Initialize
        flowfield_Map.CreateGrid();
        length = flowfield_Map.flowfield_Map.GetLength(0);

        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                if(_checkObstacle == 0){
                    flowfield_Map.flowfield_Map[i, j].walkable = true;
                    flowfield_Map.flowfield_Map[i, j].region = _region[i, j];
                }
                flowfield_Map.flowfield_Map[i, j].Reset();
            }
        }
        #endregion

        Heap<FlowFieldNode> openSet = new Heap<FlowFieldNode>( length * length );
        for(int i = 0; i < _startPoints.Count; i++){
            IntVector2 _gridPos = StaticMap.CellPosFromWorld(_startPoints[i]);
            FlowFieldNode start_Node = flowfield_Map.flowfield_Map[_gridPos.x, _gridPos.z];
            openSet.Add(start_Node);

            start_Node.totalCost = 0;
            start_Node.isClosed = false;
            start_Node.closestNodes.Add(_gridPos);
        }

        int count = 0;
        while(openSet.Count > 0){
            count++;
            FlowFieldNode cur_Node = openSet.RemoveFirst();
            IntVector2 cur_GridPos = new IntVector2(cur_Node.grid_X, cur_Node.grid_Y);
            cur_Node.isClosed = true;

            foreach(FlowFieldNode neighbor in flowfield_Map.GetNeighbours(cur_Node, _includeCorners)){
                IntVector2 neighbour_GridPos = new IntVector2(neighbor.grid_X, neighbor.grid_Y);
                if(!neighbor.walkable || neighbor.isClosed){
                    continue;
                }
                int cost = 0;



                if(_checkObstacle == 2){
                    cost = cur_Node.totalCost + Tools.GetDistance(cur_GridPos, neighbour_GridPos) + neighbor.penalty + neighbor.inflationLayer;
                }else{
                    cost = cur_Node.totalCost + Tools.GetDistance(cur_GridPos, neighbour_GridPos);
                }
                    //cost = cur_Node.totalCost + GetDistance(cur_Node, neighbor) + neighbor.penalty + neighbor.inflationLayer;
                if(cost < neighbor.totalCost){
                    neighbor.totalCost = cost;
                    neighbor.parent = cur_Node;
                    
                    neighbor.region = flowfield_Map.flowfield_Map[cur_Node.grid_X, cur_Node.grid_Y].region;
                                    
                    foreach(IntVector2 v in cur_Node.closestNodes){
                        neighbor.closestNodes.Add(v);
                    }

                    if(!openSet.Contains(neighbor)){
                        openSet.Add(neighbor);
                    }
                    else{
                        openSet.UpdateItem(neighbor);
                    }
                }
            }
        }
    }
}