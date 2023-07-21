// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// public class FlowFieldMap_gCost : MonoBehaviour
// {
//     Vector2 AgentSize = Robot1_Setting.ROBOT_LENGTH;
    
//     [Header ("Display Setting")]
//     // 這裡需要整理
//     public bool showNumber = false;
//     public bool showGrid = false;
//     public bool displayInflation = false;

//     [Header ("Map Setting")]
//     public LayerMask unWalkableMask;
//     public LayerMask boxMask;
//     public int obstacleProximityPenalty = 5;
//     public Vector2 inflationLayerPenalty = new Vector2(5 , 100);
//     public int boxBlur_Size = 1;

//     [HideInInspector] public FlowFieldNode[,] flowfield_Map;
//     int gridSizeX, gridSizeY;
//     Vector3 worldLeftBottom;
//     float nodeRadius;
//     Cell[,] map;

//     private void Start() {
//         transform.position = Vector3.zero;    
//         gridSizeX = StaticMap.gridSizeX;
//         gridSizeY = StaticMap.gridSizeY;
//         worldLeftBottom = StaticMap.worldLeftBottom;
//         nodeRadius = StaticMap.nodeRadius;
//         map = StaticMap.base_Map;

//         CreateGrid();
//     }
//     public int MaxSize{
//         get{
//             return gridSizeX * gridSizeY;
//         }
//     }

//     public void CreateGrid(){
//         flowfield_Map = new FlowFieldNode[gridSizeX, gridSizeY];
//         for(int x = 0; x < gridSizeX; x++){
//             for(int y = 0; y < gridSizeY; y++){
//                 flowfield_Map[x, y] = new FlowFieldNode(map[x, y]);
//                 bool walkable = flowfield_Map[x, y].walkable;
//                 Vector2 worldPos = flowfield_Map[x, y].worldPosition;
//                 IntVector2 gridPos = new IntVector2(x, y);
                
//                 //  Inflation Layer
//                 Vector2 inflation_R = AgentSizeToRadius();
//                 int inflationLayer =  InflationLayer(inflation_R, worldPoint);
                
//                 //  Penalty
//                 int movementPenalty = 0;
//                 if(!walkable){
//                     movementPenalty += obstacleProximityPenalty;
//                 }
//                 int boxPenalty = BoxLayer(AgentSize.x, worldPoint);

//                 grid[x, y] = new FlowFieldNode(walkable, worldPoint, x, y, movementPenalty, inflationLayer, boxPenalty);
//             }
//         }
//         BoxBlur(boxBlur_Size);
//     }

//     int BoxLayer(float _radius, Vector3 _worldPoint){
//         bool checkCol_Out = Physics.CheckSphere(_worldPoint, _radius/2, boxMask);
//         if(checkCol_Out){
//             return 100;
//         }else{ 
//             return 0;
//         }
//     }

//     int InflationLayer(Vector2 inflation_R, Vector3 _worldPoint){
//         bool checkCol_Out = Physics.CheckSphere(_worldPoint, inflation_R.y, unWalkableMask);
//         if(checkCol_Out){
//             bool checkCol_In = Physics.CheckSphere(_worldPoint, inflation_R.x, unWalkableMask);
//             if(checkCol_In){
//                 return (int)inflationLayerPenalty.y;
//             }else{
//                 return (int)inflationLayerPenalty.x;
//             }
//         }else{
//             return 0;
//         }
//     }

//     Vector2 AgentSizeToRadius(){
//         float short_Side;
//         float long_Side;
//         if(AgentSize.x <= AgentSize.y){
//             short_Side = AgentSize.x;
//             long_Side = AgentSize.y;
//         }
//         else{
//             short_Side = AgentSize.y;
//             long_Side = AgentSize.x;
//         }
//         float in_R = (short_Side / 2) * 1.2f;
//         float out_R = (Mathf.Sqrt((long_Side / 2) * (long_Side / 2) + (short_Side / 2) * (short_Side / 2))) * 1.2f;
//         return new Vector2 (in_R, out_R);
//     }

//     //  Show On Gizmos: Gray Scale
//     int penalty_Min = int.MaxValue;
//     int penalty_Max = int.MinValue;
//     void BoxBlur(int blurSize){
// 		int kernelSize = blurSize * 2 + 1;
// 		int kernelExtents = (kernelSize - 1) / 2;

// 		int[,] penaltiesHorizontalPass = new int[gridSizeX,gridSizeY];
// 		int[,] penaltiesVerticalPass = new int[gridSizeX,gridSizeY];

// 		for (int y = 0; y < gridSizeY; y++) {
// 			for (int x = -kernelExtents; x <= kernelExtents; x++) {
// 				int sampleX = Mathf.Clamp (x, 0, kernelExtents);
// 				penaltiesHorizontalPass [0, y] += grid[sampleX, y].penalty;
// 			}

// 			for (int x = 1; x < gridSizeX; x++) {
// 				int removeIndex = Mathf.Clamp(x - kernelExtents - 1, 0, gridSizeX);
// 				int addIndex = Mathf.Clamp(x + kernelExtents, 0, gridSizeX-1);

// 				penaltiesHorizontalPass [x, y] = penaltiesHorizontalPass [x - 1, y] - grid[removeIndex, y].penalty + grid[addIndex, y].penalty;
// 			}
// 		}
// 		for (int x = 0; x < gridSizeX; x++) {
// 			for (int y = -kernelExtents; y <= kernelExtents; y++) {
// 				int sampleY = Mathf.Clamp (y, 0, kernelExtents);
// 				penaltiesVerticalPass [x, 0] += penaltiesHorizontalPass [x, sampleY];
// 			}

// 			int blurredPenalty = Mathf.RoundToInt((float)penaltiesVerticalPass [x, 0] / (kernelSize * kernelSize));
// 			//grid [x, 0].penalty = blurredPenalty;
//             grid[x, 0].penalty = blurredPenalty;

// 			for (int y = 1; y < gridSizeY; y++) {
// 				int removeIndex = Mathf.Clamp(y - kernelExtents - 1, 0, gridSizeY);
// 				int addIndex = Mathf.Clamp(y + kernelExtents, 0, gridSizeY-1);

// 				penaltiesVerticalPass [x, y] = penaltiesVerticalPass [x, y-1] - penaltiesHorizontalPass [x,removeIndex] + penaltiesHorizontalPass [x, addIndex];
// 				blurredPenalty = Mathf.RoundToInt((float)penaltiesVerticalPass [x, y] / (kernelSize * kernelSize));
// 				//grid [x, y].penalty = blurredPenalty;
//                 grid[x, y].penalty = blurredPenalty;
//                 // Show on Gizmos: Gray Scale
// 				if (blurredPenalty > penalty_Max) {
// 					penalty_Max = blurredPenalty;
// 				}
// 				if (blurredPenalty < penalty_Min) {
// 					penalty_Min = blurredPenalty;
// 				}
// 			}
// 		}
// 	}
//     public HashSet<FlowFieldNode> GetNeighbours(FlowFieldNode node, bool _includeCorners){
//         HashSet<FlowFieldNode> neighbours = new HashSet<FlowFieldNode>();
//         for(int x = -1; x <= 1; x++){
//             for(int y = -1; y <= 1; y++){
//                 if(x == 0 && y == 0){
//                     continue;
//                 }
//                 if(!_includeCorners){
//                     if(x == -1 && y == -1){ continue;}
//                     if(x == -1 && y == 1){ continue;}
//                     if(x == 1 && y == -1){ continue;}
//                     if(x == 1 && y == 1){ continue;}
//                 }
//                 int checkX = node.grid_X + x;
//                 int checkY = node.grid_Y + y;
//                 if(checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY){
//                     neighbours.Add(grid[checkX, checkY]);
//                 }
//             }
//         }
//         return neighbours;
//     }
//     public FlowFieldNode NodeFromWorld(Vector3 worldPosition){
//         Vector3 worldLeftBottom = transform.position - Vector3.right * (gridWorldSize.x / 2) - Vector3.forward * (gridWorldSize.y / 2);
//         float x_Dist = worldPosition.x - worldLeftBottom.x;
//         float y_Dist = worldPosition.z - worldLeftBottom.z;

//         if(x_Dist > gridWorldSize.x){
//             x_Dist = gridWorldSize.x;
//         }
//         else if(x_Dist < 0){
//             x_Dist = 0;
//         }

//         if(y_Dist > gridWorldSize.y){
//             y_Dist = gridWorldSize.y;
//         }
//         else if(y_Dist < 0){
//             y_Dist = 0;
//         }
//         int x_Num = (int)(x_Dist / (2 * nodeRadius));
//         int y_Num = (int)(y_Dist / (2 * nodeRadius));

//         return grid[x_Num, y_Num]; 
//     }

//     void OnDrawGizmos(){
//         if(grid != null && displayInflation){
//             foreach(FlowFieldNode n in grid){
//                 if(n.inflationLayer == 0)
//                     Gizmos.color = Color.white; 
//                 else if(n.inflationLayer == 5){
//                     Gizmos.color = new Color (0.9f, 0.7f, 0.7f, 1);
//                 }
//                 else{
//                     Gizmos.color = Color.red;
//                 }
//                 // if(n.boxLayer > 0){
//                 //     Gizmos.color = Color.red; 
//                 // }else{ Gizmos.color = Color.white; }
//                 Gizmos.DrawCube(n.worldPosition - Vector3.up * 1, Vector3.one * (0.09f));
//             }
//         }
//     }
// }
