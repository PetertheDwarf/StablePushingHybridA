using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class robot_WayPoint
{
    public Vector2 pos;
    public float theta;
    
    // 3: 終點
    // 2: 轉換點 需考慮 box_Layer
    // 1: 末端點
    // 0: 常態點

    // -1: 普通點 需考慮 box_Layer 及更新為 robot 的膨脹層 以及 朝向
    // -2: 普通的終點 需考慮 box_Layer 及更新為 robot 的膨脹層 以及 朝向，同時也是轉換點的準備點
    // -3: 普通點 但不考慮 box_Layer 但實際上好像不影響？
    public int mode;

    public robot_WayPoint(Vector2 _pos, float _theta, int _mode){
        pos = _pos;
        theta = _theta;
        mode = _mode;
    }
    
}
