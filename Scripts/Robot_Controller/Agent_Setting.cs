using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "New Agent", menuName = "Agent")]
public class Agent_Setting : ScriptableObject
{
    public string agent_Name;
    [Header("Agent Property")]
    public Vector2 agent_Size; 
    public float Acceleration;
    public float Angular_Acc;
    public float max_Vel;
    public float max_Omega;

    public float pushing_Max_Vel;
    public float pushing_Max_Omega;

    public float sensor_Radius;

    public bool isPushing = false;

}
