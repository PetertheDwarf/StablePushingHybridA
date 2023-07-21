using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Voronoi_Map : MonoBehaviour
{
    [Header ("Display Setting")]
    public bool showGizmos = false;
    public bool showEdge = true;
    public bool showObstacleDist = false;
    public bool showEdgeDist = false;

    int region_Count = 0;
    Color[] region_Color;

    [HideInInspector] public Voronoi_Node[,] Voronoi_grid;
    Cell[,] map;
    FlowField flowField;
    int length;
    bool createMap = false;

    void Start(){
        flowField = GetComponent<FlowField>();
        map = StaticMap.base_Map;
        length = StaticMap.gridSizeX;
    }

    void Update(){
        if(!createMap){
            ColorSet(100);
            Voronoi_grid = Generate_VoronoiField();
            createMap = true;
        }
    }

    void ColorSet(int _colorNum){
        region_Color = new Color[_colorNum];
        for(int i = 0; i < _colorNum; i++){
            region_Color[i] = Random.ColorHSV(0f, 1f, 0f, 1f, 0.5f, 1f);
        }
    }
 

    public Voronoi_Node[,] Generate_VoronoiField(){
        Voronoi_Node[,] voronoi_Field = new Voronoi_Node[length, length];

        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                voronoi_Field[i, j] = new Voronoi_Node(map[i, j]);
            }
        }
        FindObstacleRegion(voronoi_Field);
        CreateVoronoiRegion(voronoi_Field);
        FindVoronoiEdge(voronoi_Field);
        FindDistanceToEdge(voronoi_Field);
        return voronoi_Field;
    }

    void FindObstacleRegion(Voronoi_Node[,] _voronoi_Field){
        for(int i = 0; i < length; i ++){
            for(int j = 0; j < length; j++){
                _voronoi_Field[i, j].region = -1;
            }
        }

        int region_Number = 1;
        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                Voronoi_Node cur_Node = _voronoi_Field[i, j];
                if(cur_Node.walkable){ continue;}
                if(cur_Node.region != -1){ continue;}
                FloodFillObstacle(i, j, region_Number, _voronoi_Field);
                region_Number++;
            }
        }
        region_Count = region_Number + 1;
    }

    void FloodFillObstacle(int _grid_X, int _grid_Y, int _region_Number, Voronoi_Node[,] _voronoi_Field){
        //int count = 0;
        Queue<IntVector2> obstacleSet = new Queue<IntVector2>();
        IntVector2 start_Pos = new IntVector2(_grid_X, _grid_Y);
        obstacleSet.Enqueue(start_Pos);

        while(obstacleSet.Count > 0){
            //count++;
            IntVector2 cur_Pos = obstacleSet.Dequeue();
            Voronoi_Node cur_Node = _voronoi_Field[(int)cur_Pos.x, (int)cur_Pos.z];
            cur_Node.region = _region_Number;

            for(int x = -1; x <= 1; x++){
                for(int y = -1; y <= 1; y++){
                    if(x == -1 && y == -1){ 
                        continue;
                    }
                    if(x == -1 && y == 1){
                        continue;
                    }
                    if(x == 0 && y == 0){
                        continue;
                    }
                    if(x == 1 && y == -1){
                        continue;
                    }
                    if(x == 1 && y == 1){
                        continue;
                    }
                    

                    int checkX = (int)cur_Pos.x + x;
                    int checkY = (int)cur_Pos.z + y;
                
                    if(checkX >= 0 && checkX < length && checkY >= 0 && checkY < length){
                        Voronoi_Node neighbor = _voronoi_Field[checkX, checkY];
                        IntVector2 neighbot_Pos = new IntVector2(checkX, checkY);
                        if(!neighbor.walkable){
                            if(neighbor.region == -1){
                                if(!obstacleSet.Contains(neighbot_Pos)){
                                    obstacleSet.Enqueue(neighbot_Pos);
                                }
                            }
                        }
                    }
                }
            }
        }
    }


    void CreateVoronoiRegion(Voronoi_Node[,] _voronoi_Field){    
        int[,] region = new int[length, length];

        List<Vector2> startNodes = new List<Vector2>();
        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                if(!_voronoi_Field[i, j].walkable){
                    startNodes.Add(_voronoi_Field[i, j].worldPosition);
                    region[i, j] = _voronoi_Field[i, j].region;
                }
            }
        }

        flowField.Generate_FlowField2(
            _startPoints: startNodes,
            _includeCorners: true,
            _checkObstacle: 0, 
            _region: region);

        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                _voronoi_Field[i, j].distanceToClosestObstacle = flowField.flowfield_Map.flowfield_Map[i, j].totalCost;
                _voronoi_Field[i, j].region = flowField.flowfield_Map.flowfield_Map[i, j].region;
                _voronoi_Field[i, j].closestObstacleNodes = flowField.flowfield_Map.flowfield_Map[i, j].closestNodes;
            
                // 這個是將原本的 Cost 轉換成絕對距離
                HashSet<IntVector2> closest = _voronoi_Field[i, j].closestObstacleNodes;
                if(closest != null && closest.Count > 0){
                    foreach(IntVector2 vec in closest){
                        float dist = (_voronoi_Field[(int)vec.x, (int)vec.z].worldPosition - _voronoi_Field[i, j].worldPosition).magnitude;
                        _voronoi_Field[i, j].distanceToClosestObstacle = dist;
                        break;
                    }
                }
            }
        }
    }

    void FindVoronoiEdge(Voronoi_Node[,] _voronoi_Field){
        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                int current_Region = _voronoi_Field[i, j].region;

                for(int x = -1; x <= 1; x++){
                    for(int y = -1; y <= 1; y++){
                        if(x == -1 && y == -1){ 
                            continue;
                        }
                        if(x == -1 && y == 1){
                            continue;
                        }
                        if(x == 0 && y == 0){
                            continue;
                        }
                        if(x == 1 && y == -1){
                            continue;
                        }
                        if(x == 1 && y == 1){
                            continue;
                        }
                        int checkX = i + x;
                        int checkY = j + y;

                        if(checkX >= 0 && checkX < length && checkY >= 0 && checkY < length){
                            Voronoi_Node neighbor = _voronoi_Field[checkX, checkY];

                            if(!neighbor.isVoronoiEdge){
                                if(neighbor.region != current_Region){
                                    _voronoi_Field[i, j].isVoronoiEdge = true;
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }
                
            }
        }

    }

    void FindDistanceToEdge(Voronoi_Node[,] _voronoi_Field){
        List<Vector2> start_Pos = new List<Vector2>();
        int[,] _region = new int[length, length];
        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                _region[i, j] = -1;
                if(_voronoi_Field[i, j].isVoronoiEdge){
                    start_Pos.Add(_voronoi_Field[i, j].worldPosition);
                }
            }
        }

        flowField.Generate_FlowField2(
            _startPoints: start_Pos,
            _includeCorners: true,
            _checkObstacle: 1,
            _region: _region);

        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                _voronoi_Field[i, j].distanceToClosestEdge = flowField.flowfield_Map.flowfield_Map[i, j].totalCost;
                _voronoi_Field[i, j].closestEdgeNodes = flowField.flowfield_Map.flowfield_Map[i, j].closestNodes; 
            
                HashSet<IntVector2> closest = _voronoi_Field[i, j].closestEdgeNodes;
                // 這個是將原本的 Cost 轉換成絕對距離
                if(closest != null && closest.Count > 0){
                    foreach(IntVector2 vec in closest){
                        float dist = (_voronoi_Field[(int)vec.x, (int)vec.z].worldPosition - _voronoi_Field[i, j].worldPosition).magnitude;
                        _voronoi_Field[i, j].distanceToClosestEdge = dist;
                        break;
                    }
                }
            }
        }
    }

    void OnDrawGizmos(){
        if(showGizmos){
            if(Voronoi_grid != null){
                foreach(Voronoi_Node n in Voronoi_grid){
                    Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
                    if(n.region != -1){
                        Gizmos.color = region_Color[n.region];
                    }else{
                        Gizmos.color = Color.grey;
                    }
                    Gizmos.DrawCube(vec3 - Vector3.up * 1, Vector3.one * (0.1f));
                }
                if(showEdge){
                    foreach(Voronoi_Node n in Voronoi_grid){
                        Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
                        if(n.isVoronoiEdge){
                            Gizmos.color = Color.black;
                            Gizmos.DrawCube(vec3 - Vector3.up * 1, Vector3.one * (0.1f));
                        }
                    }
                }
            }
        }
    }
}
