using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
public class A_Star_PathFinding : MonoBehaviour
{
    [Header("Display Setting")]
    public bool showPath = false;

    int obstacle_Weight = 2;
    GridMap grid;
    Node[,] grid_Map;
    public Voronoi_Map voronoi_Map;
    [HideInInspector] public Vector2[] path;

    void Awake(){
        grid = GetComponent<GridMap>();
    }

    public void StartFindPath(Vector2 _startPos, Vector2 _targetPos, bool _useBoxLayer){
        StartCoroutine(Findpath(_startPos, _targetPos, _useBoxLayer));
    }

    IEnumerator Findpath(Vector2 _startPos, Vector2 _targetPos, bool _useBoxLayer)
    {
        grid.ClearCost();
        if(_useBoxLayer){
            grid.UpdateBoxLayer(grid.grid);
        }
        grid_Map = grid.grid;

        bool pathSuccess = false;

        IntVector2 start_GridPos = StaticMap.CellPosFromWorld(_startPos);
        IntVector2 target_GridPos = StaticMap.CellPosFromWorld(_targetPos);
        if(!StaticMap.isInGrid(start_GridPos) && StaticMap.isInGrid(target_GridPos)){
            Debug.Log("Target or Start Position Error");
            yield break;
        }
        Node startNode = grid_Map[start_GridPos.x, start_GridPos.z];
        Node tarNode = grid_Map[target_GridPos.x, target_GridPos.z];

        if(startNode.walkable && tarNode.walkable){
            Heap<Node> openSet = new Heap<Node>(200000);
            HashSet<Node> closeSet = new HashSet<Node>();
            openSet.Add(startNode);

            while(openSet.Count > 0){
                Node curNode = openSet.RemoveFirst();
                closeSet.Add(curNode);//    Visited

                // stop condition.
                if(curNode == tarNode){
                    pathSuccess = true;
                    Debug.Log("A* Path Found!!");
                    break;
                }

                int count = 0;
                foreach(Node neighbour in curNode.neighbours){
                    count++;
                    if(count % 1500 == 0){
                        yield return null;
                    }
                    if(!neighbour.walkable || closeSet.Contains(neighbour)){
                        continue;
                    }
                    int curToNei;
                    //int obstacle_Cost =  (int)(obstacle_Weight * voronoi_Map.Voronoi_grid[neighbour.grid_X, neighbour.grid_Y].distanceToClosestObstacle);
                    float obstacle_Dist = voronoi_Map.Voronoi_grid[neighbour.grid_X, neighbour.grid_Y].distanceToClosestObstacle;
                    int obstacle_Cost = 0;
                    if(obstacle_Dist < 1f){
                        obstacle_Cost = obstacle_Weight * (int)((1 - obstacle_Dist) * 10);
                    }
                    if(_useBoxLayer){
                        //Debug.Log(obstacle_Cost);
                        curToNei = curNode.gCost + GetDistance(curNode, neighbour) + obstacle_Cost + neighbour.inflationLayer + neighbour.boxLayer;
                    }else{
                        curToNei = curNode.gCost + GetDistance(curNode, neighbour) + obstacle_Cost + neighbour.inflationLayer;
                    }
                    //curToNei = curNode.gCost + GetDistance(curNode, neighbour);
                    if(curToNei < neighbour.gCost || !openSet.Contains(neighbour)){
                        neighbour.gCost = curToNei;
                        neighbour.hCost = GetDistance(neighbour, tarNode);    // A star
                        neighbour.parent = curNode;

                        if(!openSet.Contains(neighbour)){
                            openSet.Add(neighbour);
                        }
                        else{
                            openSet.UpdateItem(neighbour);
                        }
                    }
                }
            }    
        }
        yield return null;
        if(pathSuccess){
            path = RetracePath(startNode, tarNode);
        }
    }
    Vector2[] RetracePath(Node startNode, Node endNode){
        List<Node> temp_path = new List<Node>();
        Node currentNode = endNode;

        while(currentNode != startNode){
            temp_path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        
        Vector2[] waypoints = new Vector2[temp_path.Count];
        for(int i = 0; i < temp_path.Count; i++){
            waypoints[i] = temp_path[i].worldPosition;

        }
        Array.Reverse(waypoints);
        return waypoints;
    }

    int GetDistance(Node nodeA, Node nodeB){
        int distX = Mathf.Abs(nodeA.grid_X - nodeB.grid_X);
        int distY = Mathf.Abs(nodeA.grid_Y - nodeB.grid_Y);

        if(distX > distY){
            return 14 * distY + 10 * (distX - distY);
        }
        else{
            return 14 * distX + 10 * (distY - distX);
        }
    }

    void OnDrawGizmos(){
        if(showPath){
            if(path != null){
                foreach(Vector2 point in path){
                    Gizmos.color = Color.black;
                    Vector3 pos = Tools.Vec2ToVec3(point);
                    Gizmos.DrawSphere(pos, 0.03f);
                }
            }
        }
    }
}

