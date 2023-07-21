using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Target_Detect : MonoBehaviour
{
    Transform _trans;
    Renderer _rend;
    Color _color;

    void Start(){
        _trans = transform;
        _rend = GetComponent<Renderer>();
        _color = new Color(0.9290f, 0.7940f, 0.1250f, 0.4f);
    }
    private void OnTriggerStay(Collider other)
    {
        // if(other.CompareTag("Robot")){
        //     bool isGoal = other.GetComponent<Controller>().isGoal;       
        //     if(isGoal){
        //         _rend.material.color = _color;
        //     }
        // }
    }
}
