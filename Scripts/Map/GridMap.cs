using System.Collections;
using System.Collections.Generic;
using UnityEngine;


// GridMap
// 名子取得不太好，但這是基於基底的靜態地圖，長出的其中一層地圖
// 主要用在【計算膨脹層】那些的，給【傳統 A* 】所使用
// 膨脹層可能會隨著有沒有箱子而更新；penalty 也會跟著 box 而需要更新
// boxlayer 會隨著箱子移動而改變

// 因此，該 Map 可分為兩大屬性： 一個是靜態的 penalty 跟 blur，而另一個部分則是可能會改變的 膨脹層 跟 box layer

public class GridMap : MonoBehaviour
{
    public Agent_Setting robot;
    Vector2 AgentSize;
    
    [Header ("Display Setting")]
    public bool showBlur = false;
    public bool showInflation = false;
    public bool showBox = false;

    [Header ("Display Cost Setting")]
    public bool showFCost = false;
    public bool showGCost = false;
    public bool showHCost = false;

    [Header ("Map Setting")]
    public LayerMask unWalkableMask;
    public LayerMask boxMask;
    public int obstacleProximityPenalty = 5;
    public IntVector2 penalty_Range = new IntVector2(20 , 1000);
    int boxBlur_Size = 1;

    [HideInInspector] public Node[,] grid;
    Cell[,] map;
    int length;
    Vector2 worldLeftBottom;
    float nodeRadius;
    

    private void Start()
    {
        AgentSize = robot.agent_Size;
        transform.position = Vector3.zero;    

        map = StaticMap.base_Map;
        length = map.GetLength(0);
        worldLeftBottom = StaticMap.worldLeftBottom;
        nodeRadius = StaticMap.nodeRadius;
        
        CreateGrid();
        Vector2 inflation_R = new Vector2(1.2f * AgentSize.x/2, 1.2f * AgentSize.x);
        UpdateInflationLayer(inflation_R);
    }

    void Update(){
        if(Input.GetKeyDown("q")){
            CreateGrid();
            Vector2 inflation_R = new Vector2(1.2f * AgentSize.x/2, 1.2f * AgentSize.x);
            UpdateInflationLayer(inflation_R);
        }
    }
    // private void Update(){
    //     if(Input.GetKeyDown("q")){
    //         Vector2 inflation_R = new Vector2(0.2f, 0.4f);
    //         UpdateInflationLayer(inflation_R, grid);
    //         UpdateBoxLayer(0.2f, grid);
    //     }
    // }

    public int MaxSize{
        get{
            return length * length;
        }
    }
    public void ClearCost()
    {
        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                grid[i,j].gCost = 0;
                grid[i,j].hCost = 0;
            }
        }
    }
    
    public void CreateGrid()
    {
        grid = new Node[length, length];

        for(int x = 0; x < length; x++){
            for(int y = 0; y < length; y++){
                grid[x, y] = new Node(map[x, y]);
                bool walkable = grid[x, y].walkable;
                Vector2 worldPos = grid[x, y].worldPosition;
                IntVector2 gridPos = new IntVector2(x, y);

                #region Add Penalty
                //  terrainPenalty 這部分不會改變（靜態障礙物）
                int terrainPenalty = 0;
                if(!walkable){
                    terrainPenalty += obstacleProximityPenalty;
                }
                grid[x, y].penalty = terrainPenalty;
                #endregion
            }
        }
        for(int x = 0; x < length; x++){
            for(int y = 0; y < length; y++){
                IntVector2 gridPos = new IntVector2(x, y);
                grid[x, y].neighbours = GetNeighbours(gridPos);
                //Debug.Log(grid[x, y].neighbours.Count);
            }
        }
        BoxBlur(boxBlur_Size, grid);
    }

    //  Inflation Layer
    // 把這部分拉出來，根據不同的 box 以及狀態 給予膨脹層
    // 膨脹層以短的那邊為主，因為要確保短的一邊可走，至於碰撞的部分，就交給碰撞檢測。

    // inflation_R: 近 / 遠
    public void UpdateInflationLayer(Vector2 _infaltion_R)
    {
        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                Vector2 worldPos = grid[i, j].worldPosition;
                int inflation_Layer_Penalty = InflationLayer(_infaltion_R, worldPos);

                grid[i, j].inflationLayer = inflation_Layer_Penalty;
            }
        }
    }

    int InflationLayer(Vector2 inflation_R, Vector2 _worldPoint){
        Vector3 pos = Tools.Vec2ToVec3(_worldPoint);

        bool checkCol_Out = Physics.CheckSphere(pos, inflation_R.y, unWalkableMask);
        if(checkCol_Out){
            bool checkCol_In = Physics.CheckSphere(pos, inflation_R.x, unWalkableMask);
            if(checkCol_In){
                return penalty_Range.z;
            }else{
                return penalty_Range.x;
            }
        }else{
            return 0;
        }
    }

    public void UpdateBoxLayer(Node[,] _grid){
        for(int i = 0; i < length; i++){
            for(int j = 0; j < length; j++){
                _grid[i, j].boxLayer = 0;

                Vector2 worldPos = grid[i, j].worldPosition;
                Vector3 pos = Tools.Vec2ToVec3(worldPos);
                float box_Inflation_R = 1f * (AgentSize.x/2);

                bool checkBox = Physics.CheckSphere(pos, box_Inflation_R, boxMask);
                int box_Cost = checkBox ? (11 * penalty_Range.x) : 0;
                
                _grid[i, j].boxLayer = box_Cost;
            }
        } 
        BoxBlur_ForBox(3, _grid);
    }

    Vector2 AgentSizeToRadius(){
        float short_Side;
        float long_Side;
        if(AgentSize.x <= AgentSize.y){
            short_Side = AgentSize.x;
            long_Side = AgentSize.y;
        }
        else{
            short_Side = AgentSize.y;
            long_Side = AgentSize.x;
        }
        float in_R = (short_Side / 2) * 1.2f;
        float out_R = (Mathf.Sqrt((long_Side / 2) * (long_Side / 2) + (short_Side / 2) * (short_Side / 2))) * 1.2f;
        return new Vector2 (in_R, out_R);
    }



    //  Show On Gizmos: Gray Scale
    int penalty_Min = int.MaxValue;
    int penalty_Max = int.MinValue;
    // BoxBlur 只影響 penalty 而已
    // 對膨脹層沒有影響
    void BoxBlur(int blurSize, Node[,] _grid){
		int kernelSize = blurSize * 2 + 1;
		int kernelExtents = (kernelSize - 1) / 2;

		int[,] penaltiesHorizontalPass = new int[length,length];
		int[,] penaltiesVerticalPass = new int[length,length];

		for (int y = 0; y < length; y++) {
			for (int x = -kernelExtents; x <= kernelExtents; x++) {
				int sampleX = Mathf.Clamp (x, 0, kernelExtents);
				penaltiesHorizontalPass [0, y] += _grid[sampleX, y].penalty;
			}

			for (int x = 1; x < length; x++) {
				int removeIndex = Mathf.Clamp(x - kernelExtents - 1, 0, length);
				int addIndex = Mathf.Clamp(x + kernelExtents, 0, length-1);

				penaltiesHorizontalPass [x, y] = penaltiesHorizontalPass [x - 1, y] - _grid[removeIndex, y].penalty + _grid[addIndex, y].penalty;
			}
		}
		for (int x = 0; x < length; x++) {
			for (int y = -kernelExtents; y <= kernelExtents; y++) {
				int sampleY = Mathf.Clamp(y, 0, kernelExtents);
				penaltiesVerticalPass [x, 0] += penaltiesHorizontalPass [x, sampleY];
			}

			int blurredPenalty = Mathf.RoundToInt((float)penaltiesVerticalPass [x, 0] / (kernelSize * kernelSize));
			//grid [x, 0].penalty = blurredPenalty;
            _grid[x, 0].penalty = blurredPenalty;

			for (int y = 1; y < length; y++) {
				int removeIndex = Mathf.Clamp(y - kernelExtents - 1, 0, length);
				int addIndex = Mathf.Clamp(y + kernelExtents, 0, length-1);

				penaltiesVerticalPass [x, y] = penaltiesVerticalPass [x, y-1] - penaltiesHorizontalPass [x,removeIndex] + penaltiesHorizontalPass [x, addIndex];
				blurredPenalty = Mathf.RoundToInt((float)penaltiesVerticalPass [x, y] / (kernelSize * kernelSize));
				//grid [x, y].penalty = blurredPenalty;
                _grid[x, y].penalty = blurredPenalty;
                // Show on Gizmos: Gray Scale
				if (blurredPenalty > penalty_Max) {
					penalty_Max = blurredPenalty;
				}
				if (blurredPenalty < penalty_Min) {
					penalty_Min = blurredPenalty;
				}
			}
		}
	}

    int box_penalty_Min = int.MaxValue;
    int box_penalty_Max = int.MinValue;

    void BoxBlur_ForBox(int blurSize, Node[,] _grid){
		int kernelSize = blurSize * 2 + 1;
		int kernelExtents = (kernelSize - 1) / 2;

		int[,] penaltiesHorizontalPass = new int[length,length];
		int[,] penaltiesVerticalPass = new int[length,length];

		for (int y = 0; y < length; y++) {
			for (int x = -kernelExtents; x <= kernelExtents; x++) {
				int sampleX = Mathf.Clamp (x, 0, kernelExtents);
				penaltiesHorizontalPass [0, y] += _grid[sampleX, y].boxLayer;
			}

			for (int x = 1; x < length; x++) {
				int removeIndex = Mathf.Clamp(x - kernelExtents - 1, 0, length);
				int addIndex = Mathf.Clamp(x + kernelExtents, 0, length-1);

				penaltiesHorizontalPass [x, y] = penaltiesHorizontalPass [x - 1, y] - _grid[removeIndex, y].boxLayer + _grid[addIndex, y].boxLayer;
			}
		}
		for (int x = 0; x < length; x++) {
			for (int y = -kernelExtents; y <= kernelExtents; y++) {
				int sampleY = Mathf.Clamp(y, 0, kernelExtents);
				penaltiesVerticalPass [x, 0] += penaltiesHorizontalPass [x, sampleY];
			}

			int blurredPenalty = Mathf.RoundToInt((float)penaltiesVerticalPass [x, 0] / (kernelSize * kernelSize));
			//grid [x, 0].penalty = blurredPenalty;
            _grid[x, 0].boxLayer = blurredPenalty;

			for (int y = 1; y < length; y++) {
				int removeIndex = Mathf.Clamp(y - kernelExtents - 1, 0, length);
				int addIndex = Mathf.Clamp(y + kernelExtents, 0, length-1);

				penaltiesVerticalPass [x, y] = penaltiesVerticalPass [x, y-1] - penaltiesHorizontalPass [x,removeIndex] + penaltiesHorizontalPass [x, addIndex];
				blurredPenalty = Mathf.RoundToInt((float)penaltiesVerticalPass [x, y] / (kernelSize * kernelSize));
				//grid [x, y].penalty = blurredPenalty;
                _grid[x, y].boxLayer = blurredPenalty;
                // Show on Gizmos: Gray Scale
				if (blurredPenalty > box_penalty_Max) {
					box_penalty_Max = blurredPenalty;
				}
				if (blurredPenalty < box_penalty_Min) {
					box_penalty_Min = blurredPenalty;
				}
			}
		}
	}
    HashSet<Node> GetNeighbours(IntVector2 _input){
        HashSet<Node> neighbours = new HashSet<Node>();
        for(int x = -1; x <= 1; x++){
            for(int y = -1; y <= 1; y++){
                if(x == 0 && y == 0){
                    continue;
                }
                //Debug.Log("x: " + x + " y: " + y);
                int checkX = _input.x + x;
                int checkY = _input.z + y;
                // Debug.Log("X: " + checkX + "  Y: " + checkY);
                if(checkX >= 0 && checkX < length){
                    if(checkY >= 0 && checkY < length){
                        neighbours.Add(grid[checkX, checkY]);
                    }
                }
            }
        }

        return neighbours;
    }

    // public Node NodeFromWorld(Vector3 worldPosition){
    //     float x_Dist = worldPosition.x - worldLeftBottom.x;
    //     float y_Dist = worldPosition.z - worldLeftBottom.y;

    //     int x_Num = (int)(x_Dist / (2 * nodeRadius));
    //     int y_Num = (int)(y_Dist / (2 * nodeRadius));

    //     if(x_Num > length || x_Num < 0){
    //         x_Num = -999;
    //     }
    //     if(y_Num > length || y_Num < 0){
    //         y_Num = -999;
    //     }
    //     return grid[x_Num, y_Num]; 
    // }




    void OnDrawGizmos(){
        if(grid != null && showBlur){
            foreach(Node n in grid){
                Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
                Gizmos.color = Color.Lerp(Color.white, Color.black, Mathf.InverseLerp(penalty_Min, penalty_Max, n.penalty));
                Gizmos.DrawCube(vec3 - Vector3.up * 1,  Vector3.one * (2 * nodeRadius) * 0.9f);
            }
        }
        
        //  Show Inflation Layer;
        if(grid != null && showInflation){
            foreach(Node n in grid){
                Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
                if(n.inflationLayer == 0)
                    Gizmos.color = Color.white; 
                else if(n.inflationLayer == 20){
                    //Gizmos.color = new Color (0.9f, 0.7f, 0.7f, 1);
                    Gizmos.color = Color.cyan;
                }
                else{
                    Gizmos.color = Color.red;
                }
                // if(n.boxLayer > 0){
                //     Gizmos.color = Color.red; 
                // }else{ Gizmos.color = Color.white; }
                Gizmos.DrawCube(vec3 - Vector3.up * 1,  Vector3.one * (2 * nodeRadius) * 0.9f);
            }
        }
        if(grid != null && showBox){
            foreach(Node n in grid){
                if(n.boxLayer != 0){
                    Vector3 vec3 = Tools.Vec2ToVec3(n.worldPosition);
                    //Gizmos.color = Color.red; 
                    Gizmos.color = Color.Lerp(Color.white, Color.red, Mathf.InverseLerp(box_penalty_Min, box_penalty_Max, n.boxLayer));
                    Gizmos.DrawCube(vec3 - Vector3.up * 1,  Vector3.one * (2 * nodeRadius) * 0.9f);
                }
            }
        }
    }
}
