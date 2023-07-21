using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Voronoi_Node
{
    public bool walkable;
    public Vector2 worldPosition;
    public int grid_X;
    public int grid_Y;

    public float voronoi_Cost;

    public int region;
    public bool isVoronoiEdge;


    public float distanceToClosestObstacle;
    public float distanceToClosestEdge;

    public HashSet<IntVector2> closestObstacleNodes;
    public HashSet<IntVector2> closestEdgeNodes;


    public Voronoi_Node(Cell _input){
        walkable = _input.walkable;
        worldPosition = _input.worldPosition;
        grid_X = _input.grid_X;
        grid_Y = _input.grid_Y;
    }


    public Vector2 ClosestObstaclePos(Vector2 _pos, Voronoi_Node[,] grid){
        Vector2 closest = Vector2.one * -1;
        float closestDist = Mathf.Infinity;

        foreach(IntVector2 n in closestObstacleNodes){
            float sqr_Dist = (_pos - grid[(int)n.x, (int)n.z].worldPosition).sqrMagnitude;
            if(sqr_Dist < closestDist){
                closestDist = sqr_Dist;
                closest = grid[(int)n.x, (int)n.z].worldPosition;
            }
        }

        return closest;
    }

    public Vector2 ClosestEdgePos(Vector2 _pos, Voronoi_Node[,] grid){
        Vector2 closest = Vector2.one * -1;
        float closestDist = Mathf.Infinity;

        foreach(IntVector2 n in closestEdgeNodes){
            float sqr_Dist = (_pos - grid[(int)n.x, (int)n.z].worldPosition).sqrMagnitude;
            if(sqr_Dist < closestDist){
                closestDist = sqr_Dist;
                closest = grid[(int)n.x, (int)n.z].worldPosition;
            }
        }

        return closest;
    }

}
