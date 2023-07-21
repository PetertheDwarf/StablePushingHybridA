using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Box_Info : MonoBehaviour
{

    [Header("Display Settting")] 
    public bool showGizmos; 
    [Header("Target Setting")]
    public Host robot1; // For Testing
    public Transform target;
    public Hybrid_A_Star hybrid_Planner;
    public GridMap grid_Map;
    const float goal_tolerance = 0.05f; //  這可以統一

    //  store the child paths.
    [HideInInspector] public robot_WayPoint[] ori_Path;
    [HideInInspector] public List<robot_WayPoint[]> sub_Paths = new List<robot_WayPoint[]>();
    [HideInInspector] public int task_Index = 0;

    [Header("Task State")]
    public bool toAuction;
    public bool isComplete;

    [HideInInspector] public Transform _trans;
    Transform _target_Trans;
    Vector3 _target_Pos_Vec3;
    float box_Width;
    float box_Length;

    Vector2[] MidPoints = new Vector2[4];


    [Header("StablePushing")]    
    public float verticalConstraint; 
    public float peshkinConstraint;
    public constrainLine[] PeshkinBound;
    public constrainLine[] VerticalBound;

    const float friction_Coefficient = 0.6f;    // 這也可以
    const int interval = 5;   // 這或許也可以

    
    [Header("ShadowBox")]
    public bool show_Shadow = false;
    public shadowBox shadow;
    int shadow_Index = 0;



    void Start(){
        _trans = transform;
        _target_Trans = target;
        _target_Pos_Vec3 = _target_Trans.position;
        toAuction = !IsGoal();

        box_Length = _trans.localScale.x;
        box_Width = _trans.localScale.z;

        ori_Path = null;
    }

    void Update(){        
        if(Time.frameCount % interval == 0){
            bool isGoal = IsGoal();
            if(isGoal){
                isComplete = true;
            }else{
                isComplete = false;

            }
        }
        
        
        // if(Input.GetKeyDown("a")){
        //     GetPushingTask();
        //     robot1.task_Paths = sub_Paths;
        //     for(int i = 0; i < sub_Paths.Count; i++){
        //         robot1.task_Indexs.Add(i+1);
        //     }
            
        // }
        if(Input.GetKeyDown("a")){
            PathPlanning(0.8f);
            
        }
        // //Test CuttingPath Function 
        // if(Input.GetKeyDown("s")){
        //     Queue<robot_WayPoint[]> test = hybrid_Planner.CutPathToTask(ori_Path);
        //     TestCuttingFunction = test.Dequeue();
        //     TestCuttingFunction = test.Dequeue();
        //     TestCuttingFunction = test.Dequeue();
        // }

        if(show_Shadow){
            if(Input.GetKeyDown(KeyCode.UpArrow)){
                if(shadow_Index < hybrid_Planner.path.Count){
                    shadow_Index++;
                    ShowShadowBox();
                }
            }else if(Input.GetKeyDown(KeyCode.DownArrow)){
                if(shadow_Index > 0){
                    shadow_Index--;
                    ShowShadowBox();
                }
            }
            if(Input.GetKeyDown(KeyCode.Space)){
                shadow_Index = hybrid_Planner.path.Count;
                ShowShadowBox();
            }
        }else{
            shadow.TurnOff();
        }
    }

    public List<robot_WayPoint[]> GetPushingTask(){
        Get_4Side_MidPoints();
        Vector2 robot_Length = new Vector2(0.2f, 0.2f);
        GetVerticalBound(robot_Length);
        GetPeshkinBound(robot_Length);
        float r = GetStablePushingConstraint(verticalConstraint, peshkinConstraint);
        Debug.Log("Constraint: " + r);
        robot1._controller.UpdatePushingConstrain(1.5f * r);
        ori_Path = PathPlanning(1.5f * r); 
        List<robot_WayPoint[]> temp = hybrid_Planner.CutPathToTask(ori_Path);
        sub_Paths = temp;
        return temp;
    }


    void ShowShadowBox(){
            if(hybrid_Planner.path.Count > 0){
                shadow.showShadowBox(hybrid_Planner.path, shadow_Index);
            }
    }

    bool IsGoal(){
        Vector2 cur_pos = Tools.Vec3ToVec2(_trans.localPosition);
        Vector2 target_Pos = Tools.Vec3ToVec2(_target_Pos_Vec3);
        float dist = Vector2.Distance(cur_pos, target_Pos);
        return dist <= goal_tolerance ? true : false; 
    }

    robot_WayPoint[] PathPlanning(float _rotation_R)
    {
        Vector2 box_Scale = new Vector2(1.5f * box_Width/2, 1.5f * box_Width);
        grid_Map.UpdateInflationLayer(box_Scale);

        Vector2 start = Tools.Vec3ToVec2(_trans.localPosition);
        Vector2 target = Tools.Vec3ToVec2(_target_Pos_Vec3);
        float start_Heading = _trans.localEulerAngles.y;
        float target_Heading = _target_Trans.localEulerAngles.y;
        Pos_Info startPos = new Pos_Info(start, start_Heading);
        Pos_Info targetPos = new Pos_Info(target, target_Heading);

        hybrid_Planner.FindPath(startPos, targetPos, box_Length, box_Width, _rotation_R, 0.2f);
        robot_WayPoint[] path = hybrid_Planner.FinalPath();

        return path;
    }

    //  0: forward, 1: right, 2: backward, 3: left
    void Get_4Side_MidPoints(){
        MidPoints[0] = Tools.Vec3ToVec2(_trans.localPosition + _trans.forward * (box_Width/2));
        MidPoints[1] = Tools.Vec3ToVec2(_trans.localPosition + _trans.right * (box_Length/2));
        MidPoints[2] = Tools.Vec3ToVec2(_trans.localPosition - _trans.forward * (box_Width/2));
        MidPoints[3] = Tools.Vec3ToVec2(_trans.localPosition - _trans.right * (box_Length/2));
    }

    //  0: right_Top, 1: right_Bottom, 2: left_Bottom, 3: left_Top
    Vector2[] GetCorner(){
        Vector2[] _corner = new Vector2[4];
        _corner[0] = Tools.Vec3ToVec2(_trans.localPosition + _trans.forward * (box_Width/2) + _trans.right * (box_Length/2));
        _corner[1] = Tools.Vec3ToVec2(_trans.localPosition - _trans.forward * (box_Width/2) + _trans.right * (box_Length/2));
        _corner[2] = Tools.Vec3ToVec2(_trans.localPosition - _trans.forward * (box_Width/2) - _trans.right * (box_Length/2));
        _corner[3] = Tools.Vec3ToVec2(_trans.localPosition + _trans.forward * (box_Width/2) - _trans.right * (box_Length/2));
        return _corner;
    }

    //  給定路徑點，決定起始位置。
    public Vector2[] Get_PreparePoint(Vector2 _robot_Length, Vector2 _point, float _safety_Factor){
        Vector2[] _prepareState = new Vector2[2];   //  point & direction
        //float safety_Factor = _safety_Factor;
        
        Vector2 corner_2_Cen = Tools.Vec3ToVec2(_trans.forward * (box_Width/2) + _trans.right * (box_Length/2));
        float corner_Dot_Forward_ = VectorDot(_trans.forward, corner_2_Cen);
        Debug.Log("Box: " + corner_Dot_Forward_);
        
        Vector2 point_2_Cen = _point - Tools.Vec3ToVec2(_trans.localPosition);
        float Point_Dot_Forward = VectorDot(Tools.Vec3ToVec2(_trans.forward), point_2_Cen);
        Debug.Log(Point_Dot_Forward);

        //  short = right or left; else = forward or backward
        if(Mathf.Abs(Point_Dot_Forward) < corner_Dot_Forward_){
            float Point_Dot_Right = VectorDot(Tools.Vec3ToVec2(_trans.right), point_2_Cen);
            if(Point_Dot_Right > 0){
                _prepareState[0] = _point + Tools.Vec3ToVec2(_trans.right * _robot_Length.y/2 * _safety_Factor); 
                _prepareState[1] = Tools.Vec3ToVec2(-_trans.right);
            }else{
                _prepareState[0] = _point - Tools.Vec3ToVec2(_trans.right * _robot_Length.y/2 * _safety_Factor); 
                _prepareState[1] = Tools.Vec3ToVec2(_trans.right);
            }
        }else{
            if(Point_Dot_Forward > 0){
                _prepareState[0] = _point + Tools.Vec3ToVec2(_trans.forward * _robot_Length.y/2 * _safety_Factor); 
                _prepareState[1] = - Tools.Vec3ToVec2(_trans.forward);
            }else{
                _prepareState[0] = _point - Tools.Vec3ToVec2(_trans.forward * _robot_Length.y/2 * _safety_Factor); 
                _prepareState[1] = Tools.Vec3ToVec2(_trans.forward);
            }
        }
        return _prepareState;
    }

    void GetVerticalBound(Vector2 _robot_Length){
        float y = -box_Width/2 - _robot_Length.y/2;
        Vector2[] corner = GetCorner();

        //  0: L1   1: L2    2: L3   3: L4
        VerticalBound = new constrainLine[4];
        VerticalBound[0] = new constrainLine(-friction_Coefficient, corner[2]);
        VerticalBound[1] = new constrainLine(-friction_Coefficient, corner[0]);
        VerticalBound[2] = new constrainLine(friction_Coefficient, corner[3]);
        VerticalBound[3] = new constrainLine(friction_Coefficient, corner[1]);
    
        //  VerticalConstraint()
        // Debug.Log("Y:" + y);
        float x = (y - box_Width/2)/(-friction_Coefficient) + box_Length/2 ;
        verticalConstraint = x;
        // Debug.Log("X: " + x);
        //Debug.Log("Vertical: " + verticalConstraint);
    }

    void GetPeshkinBound(Vector2 _robot_Length)
    {
        float y = - box_Width/2 - _robot_Length.y/2;
        Vector2[] corner = GetCorner();
        Vector2 point_OnLine1;
        Vector2 point_OnLine4;
        float slope;
        if(_robot_Length.x > box_Length){
            slope = -1 / (box_Width/box_Length);
            point_OnLine1 = corner[2] + (Tools.Vec3ToVec2(_trans.localPosition) - corner[2])/2;
            point_OnLine4 = corner[1] + (Tools.Vec3ToVec2(_trans.localPosition) - corner[1])/2;
        }else{
            slope = -1 / (box_Width/_robot_Length.x);
            point_OnLine1 = Tools.Vec3ToVec2(_trans.localPosition) - (_robot_Length.x/4) * Vector2.right - (_trans.localPosition.z/2)/2 * Vector2.up;
            point_OnLine4 = Tools.Vec3ToVec2(_trans.localPosition) + (_robot_Length.x/4) * Vector2.right - (_trans.localPosition.z/2)/2 * Vector2.up;
        }

        //  0: L1   1: L2    2: L3   3: L4
        PeshkinBound = new constrainLine[4];
        PeshkinBound[0] = new constrainLine(slope, point_OnLine1);
        PeshkinBound[1] = new constrainLine(slope, corner[0]);
        PeshkinBound[2] = new constrainLine(-slope, corner[3]);
        PeshkinBound[3] = new constrainLine(-slope, point_OnLine4);        



        float x = ((y - box_Width/2) / slope) + box_Length/2;
        peshkinConstraint = x;
        //Debug.Log("Peshkin: " + peshkinConstraint);

    }

    float GetStablePushingConstraint(float _verticalBound, float _peshkinBound){
        if(_peshkinBound > _verticalBound){
            return _peshkinBound;
        }else{
            return _verticalBound;
        }
    }


#region Tool & Struct
public class constrainLine{
    public float slope;
    public Vector2 point;
    public constrainLine(float _slope, Vector2 _point){
        slope = _slope;
        point = _point;
    }
}   


public class Task{
    public enum state{ unAssigned, Assigned, Complete, Fail, None}
    public Vector3[] child_Path;
    public Vector3[] prepare_State; //  point & dir
    public Vector3[] target_State;  // point & dir
    public bool target_Check_Dir;

    public Task(Vector3[] _path, Vector3[] _prepare_State, Vector3[] _target_State, bool _Check_Dir){
        child_Path = _path;
        prepare_State = _prepare_State;
        target_State = _target_State;
        target_Check_Dir = _Check_Dir;
    }


} 
float VectorDot(Vector2 _a, Vector2 _b){
    float dot_Value = _a.x * _b.x + _a.y * _b.y;
    return dot_Value;
}


void OnDrawGizmos(){
    float r = 0.05f;

    if(showGizmos){
        // if(test != null && test.Length != 0){
        //     Gizmos.color = Color.red;
        //     // for(int i = 0; i < 4; i++){
        //     //     Gizmos.DrawSphere(test[i], r);
        //     // }
        //     Gizmos.DrawSphere(test[0],r);
        
        // }
        // if(MidPoints!= null){
        //     Gizmos.color = Color.red;
        //     for(int i = 0; i < 4; i++){
        //         Vector3 p = Tools.Vec2ToVec3(MidPoints[i]);
        //         Gizmos.DrawSphere(p, r);
        //     }

        // }
        // if(ori_Path != null){
        //     Gizmos.color = Color.black;
        //     for(int i = 0; i < ori_Path.Length; i++){
        //         Vector3 p = Tools.Vec2ToVec3(ori_Path[i].pos);
        //         Gizmos.DrawSphere(p, r);
        //     } 
        // }
        // if(TestCuttingFunction != null){
        //     Gizmos.color = Color.black;
        //     for(int i = 0; i < TestCuttingFunction.Length; i++){
        //         Vector3 p = Tools.Vec2ToVec3(TestCuttingFunction[i].pos);
        //         Gizmos.DrawSphere(p, r);
        //     }
        // }
    }
}
#endregion
}
