using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node3D : IHeapItem<Node3D>
{
    public Vector2 worldPosition;

    public float pos_X{
        get{
            return worldPosition.x;
        }
    }
    public float pos_Y{
        get{
            return worldPosition.y;
        }
    }

    // heading in degree
    public float theta;

    public Node3D parent;

    // 0: start; 
    // 1: straight; 2: left-curve; 3: right-curve  forward
    // -1: backward; -2: right-back; -3: left-back
    // 10: left(由右邊往左推); 
    // -10: right(由左邊往右推);
    // 100: Dubin
    // 20: Orthogonal (由右邊往左推);
    // -20: Orthogonal (由左邊往右推);
    // 25: Orthogonal 往前推
    public int dir_Index = 0;
    public bool afterChange = false;

    // for display
    public float gCost;
    public float hCost;
    public float fCost{
        get{
            return gCost + hCost;
        }
    }

    public Node3D(Vector2 _worldPos, float _theta){
        worldPosition = _worldPos;
        theta = _theta;
    }

    public Node3D(Vector2 _worldPos, float _theta, Node3D _parentNode, int _dir_Index){
        worldPosition = _worldPos;
        theta = _theta;
        parent = _parentNode;
        dir_Index = _dir_Index;
    }

    public void AddCost(float _gCost, float _hCost){
        gCost = _gCost;
        hCost = _hCost;
    }

    int heapIndex;
    public int HeapIndex{
        get{
            return heapIndex;
        }
        set{
            heapIndex = value;
        }
    }

    public void CopyDataToNode(Node3D _other){
        _other.gCost = gCost;
        _other.hCost = hCost;
        _other.theta = theta;
        _other.parent = parent;
        _other.dir_Index = dir_Index;
    }


    public int CompareTo(Node3D nodeToCompare){
        int compare = fCost.CompareTo(nodeToCompare.fCost);
        if(compare == 0){
            compare = hCost.CompareTo(nodeToCompare.hCost);
        }
        return -compare;
    }
}
