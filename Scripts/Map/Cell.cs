using System.Collections;
using UnityEngine;


// 靜態地圖 Grid Map 的單位格點
// 這裡假設一些固有的障礙物是靜止不動的
// 因此這些地圖資訊並不需要重覆計算，是固定的。
// 最主要包含地圖的靜態障礙物，以及地圖本身的大小。
// 將這些格點資訊儲存下來，以便後續的使用，像是各種 path finding 演算法、運動規劃、或是碰撞檢測。

public class Cell
{
    public bool walkable;
    public Vector2 worldPosition;

    public int grid_X;
    public int grid_Y;


    public Cell(bool _walkable, Vector2 _worldPosition, int _grid_X, int _grid_Y){
        walkable = _walkable;
        worldPosition = _worldPosition;
        grid_X = _grid_X;
        grid_Y = _grid_Y;
    }
}
