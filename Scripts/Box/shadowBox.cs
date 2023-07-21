using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class shadowBox : MonoBehaviour
{
    int pool_Size = 10;
    public Transform mission_Box;
    public Material translucent;
    Vector3 box_Scale;


    // Start is called before the first frame update
    void Start()
    {
        box_Scale = mission_Box.localScale;
        AddObjectInPool();
    }

    public void showShadowBox(List<Node3D> _path, int _index){
        TurnOff();
        int box_Num = transform.childCount;
        while(box_Num < _path.Count){
            AddObjectInPool();
            box_Num = transform.childCount;
        }
        
        for(int i = 0; i < _index; i++){
            transform.GetChild(i).transform.localScale = box_Scale;
            transform.GetChild(i).transform.position = new Vector3(_path[i].worldPosition.x, transform.GetChild(i).transform.position.y, _path[i].worldPosition.y);
            transform.GetChild(i).transform.eulerAngles = new Vector3(0, _path[i].theta, 0);
            transform.GetChild(i).gameObject.SetActive(true);
        }
    }

    void AddObjectInPool(){
        for(int i = 0; i < pool_Size; i++){
            GameObject shadow_Box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            shadow_Box.GetComponent<BoxCollider>().enabled = false;
            shadow_Box.GetComponent<Renderer>().material = translucent;
            shadow_Box.transform.SetParent(transform);
            shadow_Box.transform.position = mission_Box.localPosition;
            shadow_Box.transform.localScale = box_Scale;
            shadow_Box.SetActive(false);
        }
    }

    public void TurnOff(){
        for(int i = 0; i < transform.childCount; i++){
            transform.GetChild(i).gameObject.SetActive(false);
        }    
    }
}
