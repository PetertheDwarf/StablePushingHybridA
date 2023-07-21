using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node : IHeapItem<Node>
{
    public bool walkable;
    public Vector2 worldPosition;

    public int grid_X;
    public int grid_Y;
    public int penalty; //  For obstacle or road penalty
    public int inflationLayer;  //  For inflation Layer
    public int boxLayer;    //  For box

    public Node parent;
    public HashSet<Node> neighbours;

    public int gCost;
    public int hCost;

    int heapIndex;

    public int fCost{
        get{
            return gCost + hCost;
        }
    }
    public void AddCost(int _penalty, int _inflationLayer, int _boxLayer){
        penalty = _penalty;
        inflationLayer = _inflationLayer;
        boxLayer = _boxLayer;
    }
    public Node(Cell _input){
        walkable = _input.walkable;
        worldPosition = _input.worldPosition;
        grid_X = _input.grid_X;
        grid_Y = _input.grid_Y;
    }
    public int HeapIndex{
        get{
            return heapIndex;
        }
        set{
            heapIndex = value;
        }
    }
    public int CompareTo(Node nodeToCompare){
        int compare = fCost.CompareTo(nodeToCompare.fCost);
        if(compare == 0){
            compare = hCost.CompareTo(nodeToCompare.hCost);
        }
        return -compare;
    }
}
    