using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Tools
{
    public static Vector3 Vec2ToVec3(Vector2 _input){
        Vector3 vec3 = new Vector3(_input.x, 0, _input.y);
        return vec3;
    }

    public static Vector2 Vec3ToVec2(Vector3 _input){
        Vector2 vec2 = new Vector2(_input.x, _input.z);
        return vec2;
    }

    public static int GetDistance(IntVector2 _A, IntVector2 _B){
        int distX = Mathf.Abs(_A.x - _B.x);
        int distY = Mathf.Abs(_A.z - _B.z);
        if(distX > distY){
            return 14 * distY + 10 * (distX - distY);
        }
        else{
            return 14 * distX + 10 * (distY - distX);
        }
    }

    public static float AngleToWorld(float u_Degree){
        float w_Degree;
        return w_Degree = 90 - u_Degree;
    }
    
    public static float AngleToUnity(float w_Degree){
        float u_Degree;
        return u_Degree = 90 - w_Degree;
    }

    public static float Angle360(float _input){
        while(_input > 360){
            _input = _input - 360;
        }
        while(_input < 0){
            _input = _input + 360;
        }
        return _input;
    }

    // 用 sin 檢查是否相同吧
    public static float AngleBetweenVector2(Vector2 _A, Vector2 _B){
        float dot_Value = _A.x * _B.x + _A.y * _B.y;
        float A_Magnitude = _A.magnitude;
        float B_Magnitude = _B.magnitude;
        float angle = Mathf.Rad2Deg * (Mathf.Acos(dot_Value / (A_Magnitude * B_Magnitude)));
        return angle;
    }

    public static Vector2[] CutLineToPoints(Vector2 _curPos, Vector2 _target, float _dist){
        float forward_Dist = Vector2.Distance(_curPos, _target);
        int count = (int)(forward_Dist / _dist);
        float lastPoint_Dist = forward_Dist - count * _dist; 
        Vector2 nor_Cur_Tar = _dist * new Vector2(_target.x - _curPos.x, _target.y - _curPos.y).normalized;
        if(lastPoint_Dist < _dist * 0.4f){
            Vector2[] points = new Vector2[count];
            for(int i = 0; i < count; i++){
                points[i] = new Vector2(_curPos.x + nor_Cur_Tar.x * (i+1), _curPos.y + nor_Cur_Tar.y * (i + 1));
                if(i == count - 1){
                    points[count - 1] = _target;
                }
            }
            return points;
        }else{
            Vector2[] points = new Vector2[count + 1];
            for(int i = 0; i < count + 1; i++){
                points[i] = new Vector2(_curPos.x + nor_Cur_Tar.x * (i+1), _curPos.y + nor_Cur_Tar.y * (i + 1));
                if(i == count){
                    points[count] = _target;
                }
            }
            return points;
        }
    }

    public static List<Node3D> PointsToNode_Orthogonal(Vector2[] _forward_Points, Vector2[] _side_Points, Node3D _curNode, bool _turnRight){
        List<Node3D> orthogonalNodes = new List<Node3D>();
        Node3D parent = _curNode;
        float theta = _curNode.theta;
    
        if(_forward_Points.Length > 0 && _side_Points.Length > 0){
            for(int i = 0; i < _forward_Points.Length; i++){
                Node3D temp_Node = new Node3D(_forward_Points[i], theta, parent, 25);
                parent = temp_Node;
                orthogonalNodes.Add(temp_Node);
            }
            for(int j = 0; j < _side_Points.Length; j++){
                int temp_Dir = 20;
                if(_turnRight){
                    temp_Dir = -20;
                }
                Node3D temp_Node = new Node3D(_side_Points[j], theta, parent, temp_Dir);
                parent = temp_Node;
                if(j == 0){
                    temp_Node.afterChange = true;
                }
                orthogonalNodes.Add(temp_Node);

            }
        }
        return orthogonalNodes;
    }

    public static float DotToCos(float _from, float _To){
        float _Ax = Mathf.Sin(_from * Mathf.Deg2Rad);
        float _Ay = Mathf.Cos(_from * Mathf.Deg2Rad);
        //Vector2 A_Vec = new Vector2(_Ax, _Ay);

        float _Bx = Mathf.Sin(_To * Mathf.Deg2Rad);
        float _By = Mathf.Cos(_To * Mathf.Deg2Rad);
        //Vector2 B_Vec = new Vector2(_Bx, _By);

        float dot_Value = _Ax * _Bx + _Ay * _By;
        return dot_Value;
    }
}
