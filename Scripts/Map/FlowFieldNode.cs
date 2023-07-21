using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FlowFieldNode : IHeapItem<FlowFieldNode>
{
    public bool walkable;
    public Vector3 worldPosition;

    public int grid_X;
    public int grid_Y;
    public int region = -1;

    public FlowFieldNode parent;
    public HashSet<FlowFieldNode> neighborNodes;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    public HashSet<IntVector2> closestNodes;

    public int totalCost;
    public int penalty; //  For obstacle or road penalty
    public int inflationLayer;  //  For inflation Layer
    public int boxLayer;    //  For box

    int heapIndex;

    public Vector2 flow_Direction;

    public bool isClosed;


    public FlowFieldNode(Cell _input){
        walkable = _input.walkable;
        worldPosition = _input.worldPosition;
        grid_X = _input.grid_X;
        grid_Y = _input.grid_Y;

        closestNodes = new HashSet<IntVector2>();
    }

    public FlowFieldNode(bool _walkable, Vector3 _worldPos, int _grid_X, int _grid_Y, int _penalty, int _inflationLayer, int _box_Layer)
    {
        walkable = _walkable;
        worldPosition = _worldPos;
        grid_X = _grid_X;
        grid_Y = _grid_Y;

        penalty = _penalty;
        inflationLayer = _inflationLayer;
        boxLayer = _box_Layer;
        closestNodes = new HashSet<IntVector2>();
    }

    public void Reset()
    {
        parent = null;
        isClosed = false;
        //Reset cost of movement (total cost) to this node to something large
        totalCost = int.MaxValue;
        //The cost to move to this node
        penalty = 0;        
    }



    //Add a neighbor to this node
    public void AddNeighbor(FlowFieldNode neighbor)
    {
        if (neighborNodes == null)
        {
            neighborNodes = new HashSet<FlowFieldNode>();
        }

        neighborNodes.Add(neighbor);
    }

    public int HeapIndex{
        get{
            return heapIndex;
        }
        set{
            heapIndex = value;
        }
    }

    public int CompareTo(FlowFieldNode nodeToCompare){
        int compare = totalCost.CompareTo(nodeToCompare.totalCost);
        if(compare == 0){
            compare = totalCost.CompareTo(nodeToCompare.totalCost);
        }
        return -compare;
    }

}
