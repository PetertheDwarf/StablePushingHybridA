using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Pos_Info
{
    public Vector2 pos_2D;

    // heading degree
    public float dir;

    public Pos_Info(Vector2 _pos, float _dir){
        pos_2D = _pos;
        dir = _dir;
    }

    // Vector2 vec3To2(Vector3 _input){
    //     Vector2 vec2 = new Vector2(_input.x, _input.z);
    //     return vec2;
    // }
}
