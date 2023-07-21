using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Hybrid_A_Star : MonoBehaviour
{
    [Header("Display Setting")]
    public bool showPath = false;
    public bool showRobot = false;
    [Range(0, 200000)]
    public int showIteration = 8;
    public int maxIteration = 120000;
    
    [Header("Use Shot")]
    public bool useDubinsShot = false;
    public bool useOrthogonalShot = false;

    [Header("Voronoi Map")]
    public Voronoi_Map voronoi_Map;
    [HideInInspector] public float rot_Radius = 1;
    [HideInInspector] public float step_theta;
    
    //GridMap map;
    bool noObstacle;
    float node_Radius;
    int length;
    Cell[,] map;
    FlowField flowField;
    FlowFieldNode[,] hCost_Map;
    Voronoi_Node[,] obstacleCost_Map;
    const float voronoi_Weight = 0.1f;
    DubinsGeneratePaths dubinsGenerator;

    const int resolution = 15;
    const float driveDistance = 1.414f * 0.1f * 1.2f;
    const float pos_Accuracy = 0.05f;
    const int heading_Accuracy = 12;
    float cos_Heading_Accuracy = Mathf.Cos(heading_Accuracy * Mathf.Deg2Rad);
    const float startDubin_Dist = 1.5f;
    float min_Obstacle_Dist = 0.16f;

    Vector2[] subpath;
    float[] subpath_Heading;
    int[] dir_Indexs;


    List<Node3D> allExpand_Nodes = new List<Node3D>();  // for Debug
    [HideInInspector] public List<Node3D> path = new List<Node3D>();
    [HideInInspector] public robot_WayPoint[] robot_Path = null;
    Node3D theClosestNode = null;
    bool closestPath = false;

    
    float robot_Size = 0.2f;
    float scale_X, scale_Z;

    void Start(){
        map = StaticMap.base_Map;
        node_Radius = StaticMap.nodeRadius;
        noObstacle = StaticMap.noObstacle;
        length = map.GetLength(0);
        
        flowField = GetComponent<FlowField>();

        //dir_Indexs = new int[8]{1, 2, 3, 10, -10, -1, -2, -3};
        dir_Indexs = new int[6]{1, 2, 3, -1, -2, -3};
    }

    void UpdateAgentSize(float _agentSize){
        robot_Size = _agentSize;
    }
    void UpdateScale(float _scale_X, float _scale_Z){
        scale_X = _scale_X;
        scale_Z = _scale_Z;
    }

    void UpdateRadius(float _new_Radius){
        rot_Radius = _new_Radius;
        step_theta = Mathf.Rad2Deg * (Mathf.Acos(1 - (driveDistance / rot_Radius) *  (driveDistance / rot_Radius) / 2));
        
        float cos_X = rot_Radius * (1 - Mathf.Cos(step_theta * Mathf.Deg2Rad));
        float sin_X = rot_Radius * (Mathf.Sin(step_theta * Mathf.Deg2Rad));

        // subpath = new Vector2[8]{
        //     new Vector2(0, driveDistance),
        //     new Vector2(-cos_X, sin_X),
        //     new Vector2(cos_X, sin_X),

        //     new Vector2(-driveDistance, 0),
        //     new Vector2(driveDistance, 0),

        //     new Vector2(0, -driveDistance),
        //     new Vector2(cos_X, -sin_X),
        //     new Vector2(-cos_X, -sin_X)
        // };
        subpath = new Vector2[6]{
            new Vector2(0, driveDistance),
            new Vector2(-cos_X, sin_X),
            new Vector2(cos_X, sin_X),

            // new Vector2(-driveDistance, 0),
            // new Vector2(driveDistance, 0),

            new Vector2(0, -driveDistance),
            new Vector2(cos_X, -sin_X),
            new Vector2(-cos_X, -sin_X)
        };
        //subpath_Heading = new float[8]{0, -step_theta, step_theta, 0, 0, 180, -step_theta + 180, step_theta + 180};
        subpath_Heading = new float[6]{0, -step_theta, step_theta, 0, -step_theta, step_theta};
    }

    public void FindPath(Pos_Info startPos, Pos_Info targetPos, float _scale_X, float _scale_Z, float _rotation_R, float _robot_Length)
    {
        UpdateAgentSize(_robot_Length);
        UpdateScale(_scale_X, _scale_Z);
        UpdateRadius(_rotation_R);

        robot_Path = null;
        allExpand_Nodes.Clear();
        path.Clear();

        // 障礙物的 voronoi 地圖，是靜態的，因此先抓。
        obstacleCost_Map = voronoi_Map.Voronoi_grid;
        // Flowfield, 一旦有了 target，只需要 run 一次就好
        hCost_Map = GenerateFlowField(targetPos);

        dubinsGenerator = new DubinsGeneratePaths(rot_Radius);

        // 初始化起點與終點
        IntVector2 startGrid = StaticMap.CellPosFromWorld(startPos.pos_2D);
        IntVector2 targetGrid = StaticMap.CellPosFromWorld(targetPos.pos_2D);
        Cell checkStart = map[startGrid.x, startGrid.z];
        Cell checkTarget = map[targetGrid.x, targetGrid.z];

        // 確認起點與終點都在可走的範圍
        if(checkStart.walkable && checkTarget.walkable){
            Node3D startNode = new Node3D(startPos.pos_2D, startPos.dir, null, 0);
            Node3D endNode = null;

            //Debug.Log("Start Hybrid, Is walkable");
            const int max_HeapNodes = 200000;
            Heap<Node3D> openSet = new Heap<Node3D>(max_HeapNodes);

            // 中止條件 stop condition
            bool found = false;
            bool resign = false;
            int iterations = 0;

            //  注意這裡生成的是 "二維" 的串列，也就是說一坨 list
            //  為什麼要這麼做呢？因為要檢查該 Node 是否是 closed 的
            //  而 Hybrid 的方法並不像是傳統格 A* 使用格點中心，因此無法依照網格的位置判斷
            //  在 Hybrid A* 中，會出現同一網格，但多個 Node 的狀況出現，因此這裡要做的就是如何判斷是否是同一節點
            //  並且是否重複出現，簡而言之，就是判斷該 Node 是否為 closeSet 的狀態。
            HashSet<int>[,] closedCells = new HashSet<int>[length, length];
            Dictionary<int, Node3D>[,] NodesCost = new Dictionary<int, Node3D>[length, length];
            //  initial
            for(int x = 0; x < length; x++){
                for(int y = 0; y < length; y++){
                    closedCells[x, y] = new HashSet<int>();
                    NodesCost[x, y] = new Dictionary<int, Node3D>();
                }
            }

            //  和傳統 A* 相同，第一步都是把 startNode 放入 openSet 裡面。
            openSet.Add(startNode);
            
            //  接著便是主體的部分 
            while(openSet.Count > 0){
                if(found){
                    Debug.Log("Found the path");
                    break;
                }
                if(iterations > maxIteration){
                    resign = true;
                    break;
                }
                iterations += 1; 

                //  將具有最小 F Cost 的 Node 取出
                Node3D curNode = openSet.RemoveFirst();
                //  並取得其 "網格" 位置 以及 與目標的距離與角度差距（以 cos theta 來看，因為 +- ）
                IntVector2 curNode_GridPos = StaticMap.CellPosFromWorld(curNode.worldPosition);
                int dir_Index_Abs = Mathf.Abs(curNode.dir_Index);
                float dist_ToGoal = Vector2.Distance(curNode.worldPosition, targetPos.pos_2D);
                float dot_Value_Abs = Mathf.Abs(Tools.DotToCos(curNode.theta, targetPos.dir));

                //  checkHeadingInThisCell，是將被存在這個網格中的各個角度取出來
                //  並檢查是否該網格已經有 解析後的角度，若沒有就加進去（類似於放進去 CloseSet)
                //  若是已經存在，就代表是在 "該網格中的該角度已經在 CloseSet 內"，因此可以跳過。
                int roundHeading = RoundValue(curNode.theta, resolution); 
                HashSet<int> checkHeadingInThisCell = closedCells[curNode_GridPos.x, curNode_GridPos.z];
                bool isClosed = false;

                if(!checkHeadingInThisCell.Contains(roundHeading)){
                    closedCells[curNode_GridPos.x, curNode_GridPos.z].Add(roundHeading);
                }else{
                    isClosed = true;
                }
                if(isClosed){
                    iterations -= 1;
                    continue;
                }

                //  返回最接近的節點
                if(theClosestNode == null){
                    theClosestNode = curNode;
                }else{
                    float heading_Weight = 0.05f;
                    if(dist_ToGoal < 0.8f){
                        heading_Weight = 0.1f;
                    }
                    float closestNode_Dist = Vector2.Distance(theClosestNode.worldPosition, targetPos.pos_2D);
                    float dot_Value_Closest_Abs = Mathf.Abs(Tools.DotToCos(theClosestNode.theta, targetPos.dir));
                    float closestCost = closestNode_Dist + heading_Weight * (1 - dot_Value_Closest_Abs);
                    float cur_Cost = dist_ToGoal + heading_Weight * (1 - dot_Value_Abs);
                    if(cur_Cost < closestCost){
                        theClosestNode = curNode;
                    }
                }

                #region Shot
                // 前面這部分是 orthogonal Shot
                bool orthogonal_Fail = true;
                float orthogonal_TotalLength = 0;
                Node3D orthogonal_EndNode = null;
                // 後面部分是 DubinsShot
                bool dubins_Fail = true;
                float dubins_TotalLength = 0;
                Node3D dubins_EndNode = null;

                if(dir_Index_Abs != 10 && dist_ToGoal < 5){
                    float probability = UnityEngine.Random.Range(0f, 1f);
                    if((dist_ToGoal > 1f && probability < 0.2f) || (dist_ToGoal < 1f && probability < 0.5f)){
                        if(useOrthogonalShot){
                            if(dot_Value_Abs >= cos_Heading_Accuracy){
                                Vector2 cur_ToGoal_Vec = new Vector2(targetPos.pos_2D.x - curNode.worldPosition.x, targetPos.pos_2D.y - curNode.worldPosition.y);
                                float vec_X = Mathf.Sin(curNode.theta * Mathf.Deg2Rad);
                                float vec_Y = Mathf.Cos(curNode.theta * Mathf.Deg2Rad);
                                Vector2 cur_Forward_Vec = new Vector2(vec_X, vec_Y);
                                float angle = Tools.AngleBetweenVector2(cur_Forward_Vec, cur_ToGoal_Vec);
                                float cos_Angle =  Mathf.Cos(angle * Mathf.Deg2Rad);
                                
                                bool turnRight = false;
                                float cross_Value = cur_Forward_Vec.x * cur_ToGoal_Vec.y - cur_Forward_Vec.y * cur_ToGoal_Vec.x;
                                if(cross_Value < 0){
                                    turnRight = true;
                                }
                                if(cos_Angle < 0){
                                    orthogonal_Fail = true;
                                    continue;
                                }
                                Vector2 forward_Vec = dist_ToGoal * cos_Angle * cur_Forward_Vec;
                                Vector2 forward_Point = new Vector2(forward_Vec.x + curNode.worldPosition.x, forward_Vec.y + curNode.worldPosition.y);

                                // 切兩段
                                // 一段是直行，一段是橫行，且過程中始終保持 theta 的朝向。
                                Vector2[] forward_Points = Tools.CutLineToPoints(curNode.worldPosition, forward_Point, driveDistance);
                                Vector2[] side_Points = Tools.CutLineToPoints(forward_Point, targetPos.pos_2D, driveDistance);
                                List<Node3D> orthogonalNodes = Tools.PointsToNode_Orthogonal(forward_Points, side_Points, curNode, turnRight);
                                orthogonal_TotalLength = Vector2.Distance(curNode.worldPosition, forward_Point) + Vector2.Distance(forward_Point, targetPos.pos_2D);

                                for(int i = 0; i < orthogonalNodes.Count; i++){
                                    IntVector2 cur_GridPos = StaticMap.CellPosFromWorld(orthogonalNodes[i].worldPosition);
                                    if(!StaticMap.isInGrid(cur_GridPos)){
                                        orthogonal_Fail = true;
                                        break;
                                    }
                                    if(!noObstacle){
                                        float dist_ToObstacle = obstacleCost_Map[cur_GridPos.x, cur_GridPos.z].distanceToClosestObstacle;
                                        if(dist_ToObstacle < 1.5f * (scale_X + robot_Size) / 2){
                                            bool center_Collide = CheckCollision_Center(cur_GridPos);
                                            if(center_Collide){
                                                orthogonal_Fail = true;
                                                break;
                                            }
                                        }
                                        Vector2[] corner_Points = GetCornerPoints(orthogonalNodes[i].worldPosition, orthogonalNodes[i].theta);
                                        bool corner_Collide = CheckCollision_Corners(corner_Points);
                                        if(corner_Collide){
                                            orthogonal_Fail = true;
                                            break;
                                        }
                                        robot_WayPoint robot_Point = GetRobotPos(orthogonalNodes[i].worldPosition, orthogonalNodes[i].theta, orthogonalNodes[i].dir_Index);
                                        bool robot_Collide = CheckCollision_Robot(robot_Point.pos, false);
                                        if(robot_Collide){
                                            orthogonal_Fail = true;
                                            break;
                                        }
                                        if(orthogonalNodes[i].afterChange){
                                            robot_WayPoint side_Point = GetChangeSidePoint(orthogonalNodes[i]);
                                            bool side_Collide = CheckCollision_Robot(side_Point.pos, true);
                                            //Debug.Log("side_Point: " + side_Point.pos.x + side_Point.pos.y + "  Collide: " + side_Collide);
                                            if(side_Collide){
                                                orthogonal_Fail = true;
                                                break;
                                            }
                                        }
                                    }
                                    orthogonal_Fail = false;
                                }
                                
                                if(!orthogonal_Fail){
                                    //Debug.Log("orthogonalShot Success!");
                                    orthogonal_EndNode = orthogonalNodes[orthogonalNodes.Count - 1];
                                }
                            }
                        }
                        if(useDubinsShot){
                            Vector3 curPos = Tools.Vec2ToVec3(curNode.worldPosition);
                            Vector3 tarPos = Tools.Vec2ToVec3(targetPos.pos_2D);
                            float startHeading = curNode.theta * Mathf.Deg2Rad;
                            float goalHeading = targetPos.dir * Mathf.Deg2Rad;
                            float goalHeading_180 = (targetPos.dir + 180) * Mathf.Deg2Rad;

                            List<OneDubinsPath> dubinsToGoal = new List<OneDubinsPath>();
                            OneDubinsPath[] dubinsToGoal_Forward = dubinsGenerator.GetAllDubinsPaths(
                                startPos: curPos,
                                startHeading: startHeading,
                                goalPos: tarPos,
                                goalHeading: goalHeading
                            );
                            for(int i = 0; i < dubinsToGoal_Forward.Length; i++){
                                dubinsToGoal.Add(dubinsToGoal_Forward[i]);
                            }

                            OneDubinsPath[] dubinsToGoal_Back = dubinsGenerator.GetAllDubinsPaths(
                                startPos: curPos,
                                startHeading: startHeading,
                                goalPos: tarPos,
                                goalHeading: goalHeading_180
                            );
                            for(int i = 0; i < dubinsToGoal_Back.Length; i++){
                                dubinsToGoal.Add(dubinsToGoal_Back[i]);
                            }

                            if(dubinsToGoal.Count > 0){
                                OneDubinsPath shortestPath = dubinsGenerator.FindShortestPath(dubinsToGoal);
                                dubins_TotalLength = shortestPath.totalLength;
                                shortestPath = dubinsGenerator.GetLessPoint(shortestPath, 40);
                                List<Node3D> dubinShotPath = dubinsGenerator.PointToNode(shortestPath, curNode);
                                
                                for(int j = 0; j < dubinShotPath.Count; j++){
                                    IntVector2 cur_GridPos = StaticMap.CellPosFromWorld(dubinShotPath[j].worldPosition);
                                    if(!StaticMap.isInGrid(cur_GridPos)){
                                        dubins_Fail = true;
                                        break;
                                    }
                                    if(!noObstacle){
                                        float dist_ToObstacle = obstacleCost_Map[cur_GridPos.x, cur_GridPos.z].distanceToClosestObstacle;
                                        if(dist_ToObstacle < 1.5f * (scale_X + robot_Size)/ 2){
                                            bool center_Collide = CheckCollision_Center(cur_GridPos);
                                            if(center_Collide){
                                                dubins_Fail = true;
                                                break;
                                            }
                                            Vector2[] corner_Points = GetCornerPoints(dubinShotPath[j].worldPosition, dubinShotPath[j].theta);
                                            bool corner_Collide = CheckCollision_Corners(corner_Points);
                                            if(corner_Collide){
                                                dubins_Fail = true;
                                                break;
                                            }
                                            robot_WayPoint robot_Point = GetRobotPos(dubinShotPath[j].worldPosition, dubinShotPath[j].theta, dubinShotPath[j].dir_Index);
                                            bool robot_Collide = CheckCollision_Robot(robot_Point.pos, false);
                                            if(robot_Collide){
                                                dubins_Fail = true;
                                                break;
                                            }
                                            if(dubinShotPath[j].afterChange){
                                                robot_WayPoint side_Point = GetChangeSidePoint(dubinShotPath[j]);
                                                bool side_Collide = CheckCollision_Robot(side_Point.pos, true);
                                                if(side_Collide){
                                                    dubins_Fail = true;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    dubins_Fail = false;
                                }
                                if(!dubins_Fail){
                                    //Debug.Log("dubinsShot Success!");
                                    dubins_EndNode = dubinShotPath[dubinShotPath.Count - 1];
                                }
                            }
                        }
                    }
                }

                if(!orthogonal_Fail && !dubins_Fail){
                    if(dubins_TotalLength > 1.4f * orthogonal_TotalLength){
                        endNode = orthogonal_EndNode;
                        found = true;
                        Debug.Log("Orthogonal Found the Path! " + " iterations: " + iterations);
                        break;
                    }else{
                        endNode = dubins_EndNode;
                        found = true;
                        Debug.Log("Dubin Shot Found the Path! " + " iterations: " + iterations);
                        break;
                    }
                }else if(!orthogonal_Fail){
                    endNode = orthogonal_EndNode;
                    found = true;
                    Debug.Log("Orthogonal Shot Found the Path! " + " iterations: " + iterations);
                    break;
                }else if(!dubins_Fail){
                    endNode = dubins_EndNode;
                    found = true;
                    Debug.Log("Dubin Shot Found the Path! " + " iterations: " + iterations);
                    break;
                }
                #endregion


                // 這是 Debug 用的
                allExpand_Nodes.Add(curNode);
                // 如果在可接受範圍內，就將 該 Node 設為 endNode
                if(dist_ToGoal <= pos_Accuracy && dot_Value_Abs >= cos_Heading_Accuracy){  
                    found = true;
                    endNode = curNode;
                    Debug.Log("Found the Path!" + "  iterations: " + iterations);
                }else{
                    //Debug.Log("Finding, iterations: " + iterations);
                    bool startDubin = false;
                    if(dist_ToGoal < startDubin_Dist || noObstacle){
                        startDubin = true;
                    }
                    if(dist_ToGoal > 0.1f){
                        startDubin = false;
                    }
                    // 先長出所有的 "子節點"，各個方向以及 shot 產生的節點，並包含所有的 Cost 值。
                    List<Node3D> neighbours = GetNeighbours(curNode, dir_Index_Abs, targetPos, startDubin);
                    
                    // 檢查所有的子節點
                    for(int i = 0; i < neighbours.Count; i++){
                        Vector2 pos = neighbours[i].worldPosition;
                        IntVector2 grid_Pos = StaticMap.CellPosFromWorld(pos);
                        float heading = neighbours[i].theta;
                        int rounded_neighbourHeading = RoundValue(neighbours[i].theta, resolution);

                        if(closedCells[grid_Pos.x, grid_Pos.z].Contains(rounded_neighbourHeading)){
                            continue;
                        }

                        float new_GCost = neighbours[i].gCost;
                        Dictionary<int, Node3D> nodeCostDatas = NodesCost[grid_Pos.x, grid_Pos.z];

                        if(nodeCostDatas.ContainsKey(rounded_neighbourHeading)){
                            if(new_GCost < nodeCostDatas[rounded_neighbourHeading].gCost){
                                Node3D oldNode = nodeCostDatas[rounded_neighbourHeading];
                                neighbours[i].CopyDataToNode(oldNode);
                                openSet.UpdateItem(oldNode);
                            }
                        }

                        // 最後檢查是否碰撞
                        // 障礙物的值要隨著物體而改變
                        // 這裡也許可以放在前面，最低分的才要算，其他的可以跳過。
                        // 感覺會快一點點
                        if(!StaticMap.noObstacle){
                            float dist_ToObstacle = obstacleCost_Map[grid_Pos.x, grid_Pos.z].distanceToClosestObstacle;
                            if(dist_ToObstacle < 1.5f * (scale_X + robot_Size)/2){
                                bool center_Collide = CheckCollision_Center(grid_Pos);
                                if(center_Collide){
                                    //Debug.Log("center Collide");
                                    continue;
                                }
                                Vector2[] corner_Points = GetCornerPoints(pos, heading);
                                bool corner_Collide = CheckCollision_Corners(corner_Points);
                                if(corner_Collide){
                                    //Debug.Log("corner Collide");
                                    continue;
                                }
                                robot_WayPoint robot_Pos = GetRobotPos(pos, heading, neighbours[i].dir_Index);
                                bool robot_Collide = CheckCollision_Robot(robot_Pos.pos, false);
                                if(robot_Collide){
                                    continue;
                                }
                                if(neighbours[i].afterChange){
                                    robot_WayPoint changeSide_Point = GetChangeSidePoint(neighbours[i]);
                                    bool changeSide_Collide = CheckCollision_Robot(changeSide_Point.pos, true); 
                                    if(changeSide_Collide){
                                        continue;
                                    }
                                    //Debug.Log("Neigh_side_Point: " + changeSide_Point.pos.x + changeSide_Point.pos.y + "  Collide: " + changeSide_Collide);
                                }
                            }
                        }
                        openSet.Add(neighbours[i]);
                    }
                }

            }

            if(resign){
                Debug.Log("Cannot find the path to the goal. This is the closed one. Iterations: " + iterations);
                endNode = theClosestNode;
                closestPath = true;
            }
            if(openSet.Count == 0 && !found){
                endNode = theClosestNode;
                closestPath = true;
                Debug.Log("Cannot find the path");
            }

            path = GeneratePath(endNode);
            // robot_path 可以寫在外面 以便 後續旋轉用
            robot_Path = FinalPath();
        }else{
            Debug.Log("Not walkable situation!!");
        }
    }

    FlowFieldNode[,] GenerateFlowField(Pos_Info targetPos){
        List<Vector2> target_Point = new List<Vector2>();
        target_Point.Add(targetPos.pos_2D);
        flowField.Generate_FlowField2(
            _startPoints: target_Point,
            _includeCorners: true,
            _checkObstacle: 2,
            _region: null);
        return flowField.flowfield_Map.flowfield_Map;
    }


    // 負責找出 該 Node 的所有子節點
    List<Node3D> GetNeighbours(Node3D _curNode, int _cur_Dir_Index_Abs, Pos_Info _targetPos, bool _startDubin){
        List<Node3D> neighbours = new List<Node3D>();
        
        int dir_Type = 6;
        // if(_cur_Dir_Index_Abs == 0){
        //     dir_Type = 8;
        // }else if(_cur_Dir_Index_Abs == 10){
        //     dir_Type = 8;
        // }

        //  計算各個方向產生子節點的位置
        for(int i = 0; i < dir_Type; i++){
            float angle = _curNode.theta;
            float rot_X = subpath[i].x * Mathf.Cos(angle * Mathf.Deg2Rad) + subpath[i].y * Mathf.Sin(angle * Mathf.Deg2Rad);
            float rot_Y = -subpath[i].x * Mathf.Sin(angle * Mathf.Deg2Rad) + subpath[i].y * Mathf.Cos(angle * Mathf.Deg2Rad);

            Vector2 new_Pos = new Vector2(rot_X + _curNode.pos_X, rot_Y + _curNode.pos_Y);
            float new_Heading = _curNode.theta + subpath_Heading[i];
            int new_dir = dir_Indexs[i];

            // 先簡單判斷中心節點，是否還在網格中，若否，則跳過。
            IntVector2 new_GridPos = StaticMap.CellPosFromWorld(new_Pos);
            if(!StaticMap.isInGrid(new_GridPos)){
                continue;
            }else{
                Node3D neighbourNode = new Node3D(new_Pos, new_Heading, _curNode, new_dir);
                if(Mathf.Abs(new_dir) == 10 || _cur_Dir_Index_Abs == 10){
                    if(new_dir != _curNode.dir_Index){
                        neighbourNode.afterChange = true;
                    }
                }
                // 注意 Cost 的單位，這裡要做單位轉換。
                // 生出 "子節點" 時，同時就計算其 Cost。
                float g_Cost = GetGCost(neighbourNode, new_GridPos); 
                float h_Cost = GetHCost(neighbourNode, new_GridPos, _targetPos, _startDubin);
                neighbourNode.AddCost(g_Cost, h_Cost);

                neighbours.Add(neighbourNode);
            }
        }
        return neighbours;
    }


    // 這裡要注意的是 pushing box 的 heading 和原本使用的會不太一樣。
    // 車輛的 heading 不會因為倒車而改變車頭方向
    // 但 pushing 在換邊推的時候，前進方向是會直接改變的，所以這個就要做一下調整!!!
    float GetGCost(Node3D _neighbourNode, IntVector2 _nei_GridPos){
        Node3D parent_Node = _neighbourNode.parent;
        // 這裡判斷是否與前一個 Node 的方向相同或是有沒有轉換方向之類的，以此給予不同的 penalty
        // 我不用特別標示出 載具 isreversing 的原因是： 我並不是真的讓載具後退，而是標誌出要換位置的 pos 以此切割地圖。
        float penalty = 0;
        float obstacle_Cost = 0;
        float dir_Weight = 1;

        if(!noObstacle){
            float cost = obstacleCost_Map[_nei_GridPos.x, _nei_GridPos.z].distanceToClosestObstacle;
            if(cost >= 1){
                cost = 1;
            }
            obstacle_Cost = - voronoi_Weight * cost;
        }

        int dir = _neighbourNode.dir_Index;
        int parent_dir = parent_Node.dir_Index;
        if(parent_dir == 0){
            dir_Weight = 1;
            if(Mathf.Abs(dir) == 10){
                dir_Weight = 15;
            }
        }else{
            if(dir != parent_dir){
                penalty = 0.2f * driveDistance; 
                if(Mathf.Abs(dir) == 10){
                    dir_Weight = 8;
                }else if(Mathf.Abs(parent_dir) == 10){
                    dir_Weight = 8;
                }
            }
            // if(dir == 1){
            //     //penalty -= 0.8f * driveDistance;
            //     penalty -= 0.2f * driveDistance;
            // }
        }
        //float g_Cost = parent_Node.gCost + dir_Weight * driveDistance + obstacle_Cost + penalty;
        float g_Cost = parent_Node.gCost + dir_Weight * driveDistance + penalty;
        return g_Cost;
    }



    //
    float GetHCost(Node3D _curNode, IntVector2 _nei_GridPos, Pos_Info _targetPos, bool _startDubin){
        // 設定好什麼時候要用 shot 吧？
        // 別人是在一定範圍內開始 且 給定一定的隨機值 shot 
        // 不過這邊可以考慮看看用其他放式採取 shot 的條件。
        // 不過這裡也一樣先用一定範圍吧。
                
        // FlowField
        // 這個單位是什麼？ /10 * 0.05
        //Debug.Log(" x: " + cur_GridPos.x + "  z: " + cur_GridPos.z);
        float hcost_2D = hCost_Map[_nei_GridPos.x , _nei_GridPos.z].totalCost;
        float h_Cost = 1f * (hcost_2D / 10) * (2 * node_Radius);

        // Dubins Path
        if(_startDubin){
            float dubins_Cost = 0;
            Vector3 target_Pos = Tools.Vec2ToVec3(_targetPos.pos_2D);
            Vector3 cur_Pos = Tools.Vec2ToVec3(_curNode.worldPosition);
            float target_heading = _targetPos.dir * Mathf.Deg2Rad;
            float target_heading_180 = (_targetPos.dir + 180) * Mathf.Deg2Rad;
            float cur_Heading = _curNode.theta * Mathf.Deg2Rad; 
            float cur_Heading_180 = (_curNode.theta + 180) * Mathf.Deg2Rad; 

            List<OneDubinsPath> list_DubinsToGoal = new List<OneDubinsPath>();
            OneDubinsPath[] dubinsToGoal_Forward = dubinsGenerator.GetAllDubinsPaths(
                startPos: cur_Pos,
                startHeading: cur_Heading,
                goalPos: target_Pos,
                goalHeading: target_heading
            );
            for(int i = 0; i < dubinsToGoal_Forward.Length; i++){
                list_DubinsToGoal.Add(dubinsToGoal_Forward[i]);
            }
            // OneDubinsPath[] dubinsToGoal_Back= dubinsGenerator.GetAllDubinsPaths(
            //     startPos: cur_Pos,
            //     startHeading: cur_Heading_180,
            //     goalPos: target_Pos,
            //     goalHeading: target_heading_180
            // );
            // for(int i = 0; i < dubinsToGoal_Back.Length; i++){
            //     list_DubinsToGoal.Add(dubinsToGoal_Back[i]);
            // }

            if(list_DubinsToGoal.Count > 0){
                dubins_Cost = dubinsGenerator.FindShortestPathLength(list_DubinsToGoal);
            }
            
            if(dubins_Cost > 1.2f * hcost_2D){
                h_Cost = 1f * dubins_Cost;
            }
        }
        return h_Cost;
    }

    // True means collide
    bool CheckCollision_Center(IntVector2 cen_Point){
        float obstacle_Dist = obstacleCost_Map[cen_Point.x, cen_Point.z].distanceToClosestObstacle;
        if(obstacle_Dist < min_Obstacle_Dist){
            return true;
        }else{
            return false;
        }
    }

    Vector2[] GetCornerPoints(Vector2 _cen_Point, float _heading){
        float l = scale_X/2;
        float w = scale_Z/2;
        Vector2[] corner_Points = new Vector2[4]{
            new Vector2(l, w), 
            new Vector2(l, -w), 
            new Vector2(-l, -w), 
            new Vector2(-l, w)
        };
        
        Vector2[] corner_Points_Rot = new Vector2[4];
        for(int i = 0; i < corner_Points.Length; i++){
            float new_X = corner_Points[i].x * Mathf.Cos(_heading * Mathf.Deg2Rad) + corner_Points[i].y * Mathf.Sin(_heading * Mathf.Deg2Rad);
            float new_Y = - corner_Points[i].x * Mathf.Sin(_heading * Mathf.Deg2Rad) + corner_Points[i].y * Mathf.Cos(_heading * Mathf.Deg2Rad);
            corner_Points_Rot[i] = new Vector2(_cen_Point.x + new_X, _cen_Point.y + new_Y);
        }
        return corner_Points_Rot;
    }
    
    // True means collide
    bool CheckCollision_Corners(Vector2[] corner_Points){
        for(int i = 0; i < corner_Points.Length; i++){
            IntVector2 corner_GridPos = StaticMap.CellPosFromWorld(corner_Points[i]);
            if(!StaticMap.isInGrid(corner_GridPos)){
                //Debug.Log("work");
                return true;
            }
            //Debug.Log("GridPos: " + corner_GridPos.x + " , " + corner_GridPos.z);
            float obstacle_Dist = obstacleCost_Map[corner_GridPos.x, corner_GridPos.z].distanceToClosestObstacle;
            if(obstacle_Dist < min_Obstacle_Dist){
                return true;
            }
        }
        return false;
    }

    robot_WayPoint GetChangeSidePoint(Node3D _afterChange_Node){
        Node3D parent_Node = _afterChange_Node.parent;
        int cur_Node_Dir = _afterChange_Node.dir_Index;
        float cur_Heading = _afterChange_Node.theta;
        int parent_Dir = parent_Node.dir_Index;
        if(Mathf.Abs(cur_Node_Dir) == 10 || Mathf.Abs(cur_Node_Dir) == 20){
            // 找 parent 
            // 推側邊
            float new_X = (scale_X/2 + robot_Size/2) * Mathf.Cos(cur_Heading * Mathf.Deg2Rad);
            float new_Y = (scale_X/2 + robot_Size/2) * -Mathf.Sin(cur_Heading * Mathf.Deg2Rad);
            Vector2 side_Vec = new Vector2(new_X, new_Y);
            
            float new_Headind = cur_Heading + 90;
            Vector2 side_Point = parent_Node.worldPosition - side_Vec;
            
            if(cur_Node_Dir > 0){
                side_Point = parent_Node.worldPosition + side_Vec;
                new_Headind = cur_Heading - 90;

            }
            return new robot_WayPoint(side_Point, new_Headind, 2);
        }else{
            // 找 parent 
            // 推正面 或背面
            float new_X = (scale_Z/2 + robot_Size/2) * Mathf.Sin(cur_Heading * Mathf.Deg2Rad);
            float new_Y = (scale_Z/2 + robot_Size/2) * Mathf.Cos(cur_Heading * Mathf.Deg2Rad);
            Vector2 nor_Vec = new Vector2(new_X, new_Y);
            // Vector2 nor_Point = parent_Node.worldPosition + nor_Vec;
            // float theta = -parent_Node.theta;
            
            Vector2 nor_Point = parent_Node.worldPosition - nor_Vec;
            float theta = parent_Node.theta;
            
            return new robot_WayPoint(nor_Point, theta, 2);
        }
    }


    
    robot_WayPoint GetRobotPos(Vector2 _cen_Point, float _heading, int _dir_Index)
    {
        Vector2 robot_Pos = _cen_Point;
        int abs_Index = Mathf.Abs(_dir_Index);
        
        if(abs_Index == 10 || abs_Index == 20){
            // 推測邊
            Vector2 pos = new Vector2(scale_X/2 + robot_Size/2, 0);
            if(_dir_Index < 0){
                pos = -pos;
            }
            float robot_X = pos.x * Mathf.Cos(_heading * Mathf.Deg2Rad) + pos.y * Mathf.Sin(_heading * Mathf.Deg2Rad);
            float robot_Y = - pos.x * Mathf.Sin(_heading * Mathf.Deg2Rad) + pos.y * Mathf.Cos(_heading * Mathf.Deg2Rad);
            robot_Pos = new Vector2(_cen_Point.x + robot_X, _cen_Point.y + robot_Y);
            return new robot_WayPoint(robot_Pos, _heading, _dir_Index);
        }else{
            // 推正面
            Vector2 pos = new Vector2(0, - scale_Z/2 - robot_Size/2);
            // if(_dir_Index < 0){
            //     pos = -pos;
            // }
            float robot_X =  pos.x * Mathf.Cos(_heading * Mathf.Deg2Rad) + pos.y * Mathf.Sin(_heading * Mathf.Deg2Rad);
            float robot_Y = - pos.x * Mathf.Sin(_heading * Mathf.Deg2Rad) + pos.y * Mathf.Cos(_heading * Mathf.Deg2Rad);
            robot_Pos = new Vector2(_cen_Point.x + robot_X, _cen_Point.y + robot_Y);
            return new robot_WayPoint(robot_Pos, _heading, _dir_Index);
        }
    }

    //
    List<Vector2> robotPoints = new List<Vector2>();
    bool CheckCollision_Robot(Vector2 _robot_Pos, bool _changeSide){
        robotPoints.Add(_robot_Pos);
        IntVector2 robot_GridPos = StaticMap.CellPosFromWorld(_robot_Pos);
        if(!StaticMap.isInGrid(robot_GridPos)){
            return true;
        }
        float weight = 2f;
        if(_changeSide){
            weight = 6f;
        }
        float obstacle_Dist = obstacleCost_Map[robot_GridPos.x, robot_GridPos.z].distanceToClosestObstacle;
        float dist = weight * robot_Size;
        //Debug.Log("obstacle_DistL: " + obstacle_Dist + " dist: " + dist);
        if(obstacle_Dist < dist){
            return true;
        }else{
            return false;
        }
    }


    int RoundValue(float _value, float _resolution){
        _value = Tools.Angle360(_value);
        int value = (int)(Mathf.Round(_value / _resolution) * _resolution);
        return value;
    }
    List<Node3D> GeneratePath(Node3D _end)
    {
        List<Node3D> finalPath = new List<Node3D>();

        Node3D curNode = _end;

        while(curNode != null)
        {
            finalPath.Add(curNode);
            curNode = curNode.parent;
        }

        if(finalPath.Count > 1){
            finalPath.Reverse();
        }
        return finalPath;
    }

    public robot_WayPoint[] FinalPath()
    {
        List<robot_WayPoint> temp_Path = new List<robot_WayPoint>();
        int list_Count = 0;
        for(int i = 0; i < path.Count; i++){
            if(path[i].afterChange){
                temp_Path[list_Count-1].mode = 1;
                if(path[i-1].dir_Index == 10){
                    temp_Path[list_Count-1].theta = temp_Path[list_Count-1].theta - 90;
                }else if(path[i-1].dir_Index == -10){
                    temp_Path[list_Count-1].theta = temp_Path[list_Count-1].theta + 90;
                }
                robot_WayPoint side_Point = GetChangeSidePoint(path[i]);
                temp_Path.Add(side_Point);
                list_Count++;
            }
            if(i == 1){
                if(path[1].dir_Index < 0){
                    robot_WayPoint first_Point = GetRobotPos(path[0].worldPosition, path[0].theta + 180, path[0].dir_Index);
                    first_Point.mode = 2;
                    temp_Path[0] = first_Point;
                }
            }
            robot_WayPoint cur_Point = GetRobotPos(path[i].worldPosition, path[i].theta, path[i].dir_Index);
            //Debug.Log("0: " + path[0].theta + " " + path[0].dir_Index);
            //Debug.Log("1: " + path[1].theta + " " + path[1].dir_Index);
            cur_Point.mode = 0;
            if(i == path.Count - 1){
                cur_Point.mode = 3;
            }
            temp_Path.Add(cur_Point);
            if(i == 0){
             cur_Point.mode = 2;   
            }

            list_Count++;
        }

        robot_WayPoint[] final_Path = new robot_WayPoint[temp_Path.Count];
        for(int i = 0; i < temp_Path.Count; i++){
            final_Path[i] = temp_Path[i];
        }
        
        return final_Path;
    }

    public List<robot_WayPoint[]> CutPathToTask(robot_WayPoint[] _ori_Path){
        List<int> path_Count = new List<int>();
        int temp_Count = 0;
        for(int i = 0; i < _ori_Path.Length; i++){
            if(_ori_Path[i].mode == 1 || _ori_Path[i].mode == 3){
                path_Count.Add(temp_Count+1);
                temp_Count = 0;
            }else{
                temp_Count++;
            }
        }
        //Debug.Log("Path.Length: " + _ori_Path.Length + "  path 2: " + path_Count[1] + "  path Count: " + path_Count.Count);
        
        List<robot_WayPoint[]> pushing_Tasks = new List<robot_WayPoint[]>();
        int pre_Count = 0; 
        for(int i = 0; i < path_Count.Count; i++){
            int count = path_Count[i];
            robot_WayPoint[] temp_Path = new robot_WayPoint[count];
            for(int j = 0; j < count; j++){
                temp_Path[j] = _ori_Path[j + pre_Count];
            }
            pre_Count += count;
            pushing_Tasks.Add(temp_Path);
        }
        return pushing_Tasks;
    }



#region 
    void OnDrawGizmos(){
        float dist = driveDistance * 0.5f;
        if(showPath){
            if(allExpand_Nodes != null){
                int count = 0;
                foreach(Node3D n in allExpand_Nodes){
                    if(count > showIteration){
                        break;
                    }
                    
                    Gizmos.color = new Color(1, 1, 1, 0.4f);
                    Vector3 pos = Tools.Vec2ToVec3(n.worldPosition);
                    Gizmos.DrawSphere(pos, 0.005f);
                    float theta = Tools.AngleToWorld(n.theta);
                    //Debug.Log(count + ": " + n.theta);
                    Vector3 dir = new Vector3(pos.x + dist * Mathf.Cos(theta * Mathf.Deg2Rad), pos.y, pos.z + dist * Mathf.Sin(theta * Mathf.Deg2Rad)); 
                    Gizmos.DrawLine(pos, dir);
                    count++;
                }
                if(path != null){
                    foreach(Node3D p in path){
                        if(!closestPath){
                            Gizmos.color = Color.green;
                            if(p.dir_Index == 100){
                                Gizmos.color = Color.green;
                            }else if(Mathf.Abs(p.dir_Index) == 10){
                                Gizmos.color = Color.magenta;
                            }else if(Mathf.Abs(p.dir_Index) == 20 || p.dir_Index == 25){
                                Gizmos.color = Color.cyan;
                            }
                        }else{
                            Gizmos.color = Color.blue;
                        }
                        
                        
                        Vector3 pos = Tools.Vec2ToVec3(p.worldPosition);
                        
                        Gizmos.DrawSphere(pos, 0.02f);
                        //float theta = Tools.AngleToWorld(p.theta);
                        float theta = p.theta;
                        Vector3 dir = new Vector3(pos.x + 1.5f * dist * Mathf.Sin(theta * Mathf.Deg2Rad), pos.y, pos.z + 1.5f * dist * Mathf.Cos(theta * Mathf.Deg2Rad)); 
                        if(p.dir_Index == 10){
                            dir = new Vector3(pos.x + dist * Mathf.Sin((theta+90) * Mathf.Deg2Rad) , pos.y, pos.z + dist * Mathf.Cos((theta+90) * Mathf.Deg2Rad));
                        }else if(p.dir_Index == -10){
                            dir = new Vector3(pos.x + dist * Mathf.Sin((theta-90) * Mathf.Deg2Rad) , pos.y, pos.z + dist * Mathf.Cos((theta-90) * Mathf.Deg2Rad));
                        }else if(p.dir_Index == 20){
                            dir = new Vector3(pos.x + dist * Mathf.Sin((theta+90) * Mathf.Deg2Rad) , pos.y, pos.z + dist * Mathf.Cos((theta+90) * Mathf.Deg2Rad));
                        }else if(p.dir_Index == -20){
                            dir = new Vector3(pos.x + dist * Mathf.Sin((theta-90) * Mathf.Deg2Rad) , pos.y, pos.z + dist * Mathf.Cos((theta-90) * Mathf.Deg2Rad));
                        }
                        Gizmos.DrawLine(pos, dir);
                        
                        if(p.afterChange){
                            Gizmos.color = Color.cyan;
                            Gizmos.DrawSphere(pos, 0.05f);
                        }
                        // if(showRobot){
                        //     robot_WayPoint robot_Pos = GetRobotPos(p.worldPosition, p.theta, p.dir_Index);
                        //     Gizmos.color = new Color(085f, 0.325f, 0.098f, 0.4f);
                        //     Gizmos.DrawSphere(Tools.Vec2ToVec3(robot_Pos.pos), 0.1f);
                        //     if(p.afterChange){
                        //         robot_WayPoint transition_Point = GetChangeSidePoint(p);
                        //         //Gizmos.color = new Color(085f, 0.325f, 0.098f, 0.4f);
                        //         Gizmos.color = Color.yellow;
                        //         Gizmos.DrawSphere(Tools.Vec2ToVec3(transition_Point.pos), 0.1f);
                        //     }else{
                        //         continue;
                        //     }
                        // }
                        // if(robotPoints.Count > 0){
                        //     Gizmos.color = Color.black;
                        //     foreach(Vector2 point in robotPoints){
                        //         Vector3 ppos = Tools.Vec2ToVec3(point);
                        //         Gizmos.DrawSphere(ppos, 0.1f);
                        //     }
                        // }
                    }
                }
            }
        }
        // if(showDubins){
        //     if(dubinPoint != null){
        //         for(int i = 0; i < dubinPoint.Count; i++){
        //             Gizmos.color = Color.red;
        //             Gizmos.DrawSphere(dubinPoint[i], 0.02f);
        //             Vector3 pos = dubinPoint[i];
        //             float dist = driveDistance * 0.5f;
        //             //Debug.Log("theta: " + dubinTheta.Count + "   point: " + dubinPoint.Count);
        //             float theta = Tools.AngleToWorld(dubinTheta[i]);
        //             Vector3 dir = new Vector3(pos.x + dist * Mathf.Cos(theta * Mathf.Deg2Rad), pos.y, pos.z + dist * Mathf.Sin(theta * Mathf.Deg2Rad)); 
        //             Gizmos.DrawLine(pos, dir);
        //         }
        //     }
        // }

        
        
    }
#endregion

}