using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DubinsGeneratePaths
{

        //The 4 different circles we have that sits to the left/right of the start/goal
        //Public so we can position the circle objects for debugging
        public Vector3 startLeftCircle;
        public Vector3 startRightCircle;
        public Vector3 goalLeftCircle;
        public Vector3 goalRightCircle;

        //To generate paths we need the position and rotation (heading) of the cars
        Vector3 startPos;
        Vector3 goalPos;
        //Heading is in radians
        float startHeading;
        float goalHeading;

        //Where we store all path data so we can sort and find the shortest path
        List<OneDubinsPath> pathDataList = new List<OneDubinsPath>();

        float turningRadius;

        public DubinsGeneratePaths(float _turningRadius){
            turningRadius = _turningRadius;
        }

        public void ChangeTurningRadius(float _turningRadius){
            turningRadius = _turningRadius;
        }

        //Get all valid Dubins paths sorted from shortest to longest
        public OneDubinsPath[] GetAllDubinsPaths(Vector3 startPos, float startHeading, Vector3 goalPos, float goalHeading)
        {
            this.startPos = startPos;
            this.goalPos = goalPos;
            this.startHeading = startHeading;
            this.goalHeading = goalHeading;

            //Reset the list with all Dubins paths
            pathDataList.Clear();

            //Position the circles that are to the left/right of the cars
            PositionLeftRightCircles();

            //Find the length of each path with tangent coordinates
            CalculateDubinsPathsLengths();

            //If we have paths
            if (pathDataList.Count > 0)
            {
                //Sort the list with paths so the shortest path is first
                pathDataList.Sort((x, y) => x.totalLength.CompareTo(y.totalLength));

                //Generate the final coordinates of the path from tangent points and segment lengths
                GeneratePathCoordinates();
                OneDubinsPath[] allPath = new OneDubinsPath[pathDataList.Count];
                for(int i = 0; i < pathDataList.Count; i++){
                    allPath[i] = pathDataList[i];
                }
                return allPath;
            }

            //No paths could be found
            return null;
        }


        //Position the left and right circles that are to the left/right of the target and the car
        void PositionLeftRightCircles()
        {
            //Start pos
            startRightCircle = DubinsMath.GetRightCircleCenterPos(startPos, startHeading, turningRadius);
            startLeftCircle = DubinsMath.GetLeftCircleCenterPos(startPos, startHeading, turningRadius);

                        //Goal pos
            goalRightCircle = DubinsMath.GetRightCircleCenterPos(goalPos, goalHeading, turningRadius);
            goalLeftCircle = DubinsMath.GetLeftCircleCenterPos(goalPos, goalHeading, turningRadius);
        }


        //
        //Calculate the path lengths of all Dubins paths by using tangent points
        //
        void CalculateDubinsPathsLengths()
        {
            //RSR and LSL is only working if the circles don't have the same position
            
            //RSR
            if (startRightCircle.x != goalRightCircle.x && startRightCircle.z != goalRightCircle.z)
            {
                Get_RSR_Length();
            }
            
            //LSL
            if (startLeftCircle.x != goalLeftCircle.x && startLeftCircle.z != goalLeftCircle.z)
            {
                Get_LSL_Length();
            }


            //RSL and LSR is only working of the circles don't intersect
            float comparisonSqr = turningRadius * 2f * turningRadius * 2f;

            //RSL
            if ((startRightCircle - goalLeftCircle).sqrMagnitude > comparisonSqr)
            {
                Get_RSL_Length();
            }

            //LSR
            if ((startLeftCircle - goalRightCircle).sqrMagnitude > comparisonSqr)
            {
                Get_LSR_Length();
            }


            //With the LRL and RLR paths, the distance between the circles have to be less than 4 * r
            comparisonSqr = 4f * turningRadius * 4f * turningRadius;

            //RLR        
            if ((startRightCircle - goalRightCircle).sqrMagnitude < comparisonSqr)
            {
                Get_RLR_Length();
            }

            //LRL
            if ((startLeftCircle - goalLeftCircle).sqrMagnitude < comparisonSqr)
            {
                Get_LRL_Length();
            }
        }


        //RSR
        void Get_RSR_Length()
        {
            //Find both tangent positons
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;

            DubinsMath.LSLorRSR(startRightCircle, goalRightCircle, turningRadius, false, out startTangent, out goalTangent);

            //Calculate lengths
            float length1 = DubinsMath.GetArcLength(startRightCircle, startPos, startTangent, turningRadius, false);

            float length2 = (startTangent - goalTangent).magnitude;

            float length3 = DubinsMath.GetArcLength(goalRightCircle, goalTangent, goalPos, turningRadius, false);

            //Save the data
            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RSR);

            //We also need this data to simplify when generating the final path'
            // 不太理解這部分
            pathData.segment2Turning = false;

            //RSR
            pathData.SetIfTurningRight(true, false, true);

            //Add the path to the collection of all paths
            pathDataList.Add(pathData);
        }


        //LSL
        void Get_LSL_Length()
        {
            //Find both tangent positions
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;

            DubinsMath.LSLorRSR(startLeftCircle, goalLeftCircle, turningRadius, true, out startTangent, out goalTangent);

            //Calculate lengths
            float length1 = DubinsMath.GetArcLength(startLeftCircle, startPos, startTangent, turningRadius, true);

            float length2 = (startTangent - goalTangent).magnitude;

            float length3 = DubinsMath.GetArcLength(goalLeftCircle, goalTangent, goalPos, turningRadius, true);

            //Save the data
            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LSL);

            //We also need this data to simplify when generating the final path
            pathData.segment2Turning = false;

            //LSL
            pathData.SetIfTurningRight(false, false, false);

            //Add the path to the collection of all paths
            pathDataList.Add(pathData);
        }


        //RSL
        void Get_RSL_Length()
        {
            //Find both tangent positions
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;

            DubinsMath.RSLorLSR(startRightCircle, goalLeftCircle, turningRadius, false, out startTangent, out goalTangent);

            //Calculate lengths
            float length1 = DubinsMath.GetArcLength(startRightCircle, startPos, startTangent, turningRadius, false);

            float length2 = (startTangent - goalTangent).magnitude;

            float length3 = DubinsMath.GetArcLength(goalLeftCircle, goalTangent, goalPos, turningRadius, true);

            //Save the data
            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RSL);

            //We also need this data to simplify when generating the final path
            pathData.segment2Turning = false;

            //RSL
            pathData.SetIfTurningRight(true, false, false);

            //Add the path to the collection of all paths
            pathDataList.Add(pathData);
        }


        //LSR
        void Get_LSR_Length()
        {
            //Find both tangent positions
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;

            DubinsMath.RSLorLSR(startLeftCircle, goalRightCircle, turningRadius, true, out startTangent, out goalTangent);

            //Calculate lengths
            float length1 = DubinsMath.GetArcLength(startLeftCircle, startPos, startTangent, turningRadius, true);

            float length2 = (startTangent - goalTangent).magnitude;

            float length3 = DubinsMath.GetArcLength(goalRightCircle, goalTangent, goalPos, turningRadius, false);

            //Save the data
            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LSR);

            //We also need this data to simplify when generating the final path
            pathData.segment2Turning = false;

            //LSR
            pathData.SetIfTurningRight(false, false, true);

            //Add the path to the collection of all paths
            pathDataList.Add(pathData);
        }


        //RLR
        void Get_RLR_Length()
        {
            //Find both tangent positions and the position of the 3rd circle
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;
            //Center of the 3rd circle
            Vector3 middleCircle = Vector3.zero;

            DubinsMath.GetRLRorLRLTangents(
                startRightCircle,
                goalRightCircle,
                turningRadius,
                false,
                out startTangent,
                out goalTangent,
                out middleCircle);

            //Calculate lengths
            float length1 = DubinsMath.GetArcLength(startRightCircle, startPos, startTangent, turningRadius, false);

            float length2 = DubinsMath.GetArcLength(middleCircle, startTangent, goalTangent, turningRadius, true);

            float length3 = DubinsMath.GetArcLength(goalRightCircle, goalTangent, goalPos, turningRadius, false);

            //Save the data
            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RLR);

            //We also need this data to simplify when generating the final path
            pathData.segment2Turning = true;

            //RLR
            pathData.SetIfTurningRight(true, false, true);

            //Add the path to the collection of all paths
            pathDataList.Add(pathData);
        }


        //LRL
        void Get_LRL_Length()
        {
            //Find both tangent positions and the position of the 3rd circle
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;
            //Center of the 3rd circle
            Vector3 middleCircle = Vector3.zero;

            DubinsMath.GetRLRorLRLTangents(
                startLeftCircle,
                goalLeftCircle,
                turningRadius,
                true,
                out startTangent,
                out goalTangent,
                out middleCircle);

            //Calculate the total length of this path
            float length1 = DubinsMath.GetArcLength(startLeftCircle, startPos, startTangent, turningRadius, true);

            float length2 = DubinsMath.GetArcLength(middleCircle, startTangent, goalTangent, turningRadius, false);

            float length3 = DubinsMath.GetArcLength(goalLeftCircle, goalTangent, goalPos, turningRadius, true);

            //Save the data
            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LRL);

            //We also need this data to simplify when generating the final path
            pathData.segment2Turning = true;

            //LRL
            pathData.SetIfTurningRight(false, true, false);

            //Add the path to the collection of all paths
            pathDataList.Add(pathData);
        }


        //
        // Generate the final path from the tangent points
        //

        //When we have found the tangent points and lengths of each path we need to get the individual coordinates
        //of the entire path so we can travel along the path
        void GeneratePathCoordinates()
        {
            for (int i = 0; i < pathDataList.Count; i++)
            {
                GetTotalPath(pathDataList[i]);
            }
        }

        public OneDubinsPath FindShortestPath(List<OneDubinsPath> _pathData){
            float shortest_Length = float.MaxValue;
            OneDubinsPath shortest_Path = _pathData[0]; 
            for(int x = 0; x < _pathData.Count; x++){
                float total_Length = _pathData[x].length1 + _pathData[x].length2 + _pathData[x].length3;
                if(shortest_Length > total_Length){
                    shortest_Length = total_Length;
                    shortest_Path = _pathData[x];
                }
            }
            return shortest_Path;
        }

        public float FindShortestPathLength(List<OneDubinsPath> _pathData){
            float shortest_Length = float.MaxValue; 
            for(int x = 0; x < _pathData.Count; x++){
                float total_Length = _pathData[x].length1 + _pathData[x].length2 + _pathData[x].length3;
                if(shortest_Length > total_Length){
                    shortest_Length = total_Length;
                }
            }
            return shortest_Length;
        }

        //Find the coordinates of the entire path from the 2 tangents and length of each segment
        void GetTotalPath(OneDubinsPath pathData)
        {
            //Store the waypoints of the final path here
            List<Vector3> finalPath = new List<Vector3>();
            List<float> thetaPath = new List<float>();

            //Start position of the car
            Vector3 currentPos = startPos;
            //Start heading of the car
            float theta = startHeading;

            //We always have to add the first position manually = the position of the car
            finalPath.Add(currentPos);
            thetaPath.Add(startHeading * Mathf.Rad2Deg);
            

            //How many line segments can we fit into this part of the path
            int segments = 0;

            //First
            segments = Mathf.FloorToInt(pathData.length1 / DubinsMath.driveDistance);

            DubinsMath.AddCoordinatesToPath(
                ref currentPos,
                ref theta,
                finalPath,
                thetaPath,
                turningRadius,
                segments,
                true,
                pathData.segment1TurningRight);

            //Second
            segments = Mathf.FloorToInt(pathData.length2 / DubinsMath.driveDistance);

            DubinsMath.AddCoordinatesToPath(
                ref currentPos,
                ref theta,
                finalPath,
                thetaPath,
                turningRadius,
                segments,
                pathData.segment2Turning,
                pathData.segment2TurningRight);

            //Third
            segments = Mathf.FloorToInt(pathData.length3 / DubinsMath.driveDistance);

            DubinsMath.AddCoordinatesToPath(
                ref currentPos,
                ref theta,
                finalPath,
                thetaPath,
                turningRadius,
                segments,
                true,
                pathData.segment3TurningRight);

            //Add the final goal coordinate
            finalPath.Add(new Vector3(goalPos.x, currentPos.y, goalPos.z));
            thetaPath.Add(goalHeading * Mathf.Rad2Deg);
            //Save the final path in the path data
            pathData.pathCoordinates = finalPath;
            pathData.pathTheta = thetaPath;
        }

        public OneDubinsPath GetLessPoint(OneDubinsPath pathData, int _step){
            int count = 0;

            List<Vector3> temp_Point = new List<Vector3>();
            List<float> temp_Theta = new List<float>();
            for(int i = 0; i < pathData.pathCoordinates.Count; i++){
                if(count == pathData.pathCoordinates.Count - 1){
                    temp_Point.Add(pathData.pathCoordinates[i]);
                    temp_Theta.Add(pathData.pathTheta[i]);
                    continue;
                }
                if(count % _step == 0){
                    temp_Point.Add(pathData.pathCoordinates[i]);
                    temp_Theta.Add(pathData.pathTheta[i]);
                    count++;
                }else{
                    count++;
                    continue;
                }
            }

            pathData.pathCoordinates = temp_Point;
            pathData.pathTheta = temp_Theta;

            return pathData;    
        }

        public List<Node3D> PointToNode(OneDubinsPath pathData, Node3D _curNode){
            List<Node3D> dubinNodes = new List<Node3D>();
            // dubinNodes.Add(_curNode);
            Node3D parent = _curNode;

            if(pathData.pathCoordinates.Count > 0){
                for(int i = 0; i < pathData.pathCoordinates.Count; i++){
                    if(i == 0){ continue;}
                    Vector2 worldPos = new Vector2(pathData.pathCoordinates[i].x, pathData.pathCoordinates[i].z);
                    float theta = pathData.pathTheta[i];

                    Node3D temp_Node = new Node3D(worldPos, theta, parent, 100);
                    parent = temp_Node;
                    dubinNodes.Add(temp_Node);
                }
            }
            return dubinNodes;    
        }
}
