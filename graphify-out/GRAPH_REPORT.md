# Graph Report - robotX  (2026-09-29)

## Corpus Check
- 27 files · ~35,012 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 952 nodes · 1550 edges · 42 communities (32 shown, 10 thin omitted)
- Extraction: 91% EXTRACTED · 9% INFERRED · 0% AMBIGUOUS · INFERRED: 132 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `3ea7311b`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- DroneController
- utils/control_.cpp
- VisionGeoNode
- VisionArTagNode
- VisionVideoNode
- VisionKRTINode
- VisionTagNode
- drone_controller.cpp
- vision_geo_.cpp
- ObjectDetectionCam
- drone_controller_.hpp
- geo_.cpp
- SharedPtr
- Request
- vision_video_.cpp
- .processFrame
- .streamFrame
- CameraViewer
- waitForService
- fuseecam.cpp
- VisionGeoNode::VisionGeoNode
- .VisionArTagNode
- VisionVideoNode::processFrame
- VisionGeoNode::processFrame
- VisionVideoNode::VisionVideoNode
- string
- .detection
- .imageCallback
- ConstSharedPtr
- main
- VisionVideoNode::imageCallback
- Float64
- DroneController::publishSetpointRawLocal
- launch_vision_geo.sh
- DroneController::getCurrentAccel
- DroneController::publishSetpointRawGlobal
- DroneController::getCurrentGPSState
- DroneController::getCurrentLaserScan
- DroneController::getCurrentEuler

## God Nodes (most connected - your core abstractions)
1. `DroneController` - 219 edges
2. `VisionGeoNode` - 99 edges
3. `VisionArTagNode` - 84 edges
4. `VisionVideoNode` - 76 edges
5. `VisionTagNode` - 66 edges
6. `VisionKRTINode` - 55 edges
7. `ObjectDetectionCam` - 50 edges
8. `centeringPayload()` - 21 edges
9. `centeringPayload()` - 21 edges
10. `moveToPoint()` - 15 edges

## Surprising Connections (you probably didn't know these)
- `centeringPayload()` --calls--> `point_rotation_by_quaternion()`  [INFERRED]
  refrence/control_.cpp → src/control/utils/geo_.cpp
- `LocalMove()` --references--> `DroneController`  [EXTRACTED]
  refrence/control_.cpp → src/control/include/drone_controller_.hpp
- `LocalMove()` --calls--> `getHeading()`  [INFERRED]
  refrence/control_.cpp → src/control/utils/math_.cpp
- `LocalMove()` --calls--> `setHeading()`  [INFERRED]
  refrence/control_.cpp → src/control/utils/math_.cpp
- `centeringPayload()` --references--> `DroneController`  [EXTRACTED]
  refrence/control_.cpp → src/control/include/drone_controller_.hpp

## Import Cycles
- None detected.

## Communities (42 total, 10 thin omitted)

### Community 0 - "DroneController"
Cohesion: 0.01
Nodes (162): DroneController, accel_cb, accel_sub, alt_cb, alt_sub, arm_, arming_client, artag_pose (+154 more)

### Community 1 - "utils/control_.cpp"
Cohesion: 0.07
Nodes (86): centeringPayload(), PoseStamped, Rate, shared_ptr, string, LocalMove(), hasReplied(), shared_ptr (+78 more)

### Community 2 - "VisionGeoNode"
Cohesion: 0.03
Nodes (74): ActiveTrack, BuoyMapPoint, Mat, MavrosState, Net, Node, SharedPtr, size_t (+66 more)

### Community 3 - "VisionArTagNode"
Cohesion: 0.03
Nodes (61): DetectorParameters, Dictionary, mutex, Node, Point3f, Ptr, time_point, VideoCapture (+53 more)

### Community 4 - "VisionVideoNode"
Cohesion: 0.04
Nodes (57): Mat, mutex, Net, Node, Point3f, SharedPtr, size_t, string (+49 more)

### Community 5 - "VisionKRTINode"
Cohesion: 0.05
Nodes (42): DictionaryInfo, DetectorParameters, Mat, mutex, Node, Ptr, SharedPtr, string (+34 more)

### Community 6 - "VisionTagNode"
Cohesion: 0.04
Nodes (48): DetectorParameters, Dictionary, mutex, Node, Ptr, time_point, VideoCapture, VideoWriter (+40 more)

### Community 7 - "drone_controller.cpp"
Cohesion: 0.07
Nodes (26): PoseStamped, DroneController::getCurrentGatePose(), DroneController::getCurrentLocalPose(), DroneController::getCurrentPoseArTag(), DroneController::getCurrentPoseBunder(), DroneController::getCurrentPoseEmber(), DroneController::getCurrentPosePayload(), DroneController::getDroneNonZeroPoseArTag() (+18 more)

### Community 8 - "vision_geo_.cpp"
Cohesion: 0.08
Nodes (38): GeoPoint, addObservationToBuoy, classColor, drawCrosshair, drawDetections, processFrame, quaternionToMat, rpy2mat (+30 more)

### Community 9 - "ObjectDetectionCam"
Cohesion: 0.06
Nodes (32): Node, SharedPtr, string, VideoWriter, ObjectDetectionCam, camera_path, closed_, current_range (+24 more)

### Community 10 - "drone_controller_.hpp"
Cohesion: 0.14
Nodes (15): Bool, Image, Pose, Float64, GPSRAW, Point, State, vector (+7 more)

### Community 11 - "geo_.cpp"
Cohesion: 0.13
Nodes (24): LatLonAlt, alt, lat, lon, XYZ, x, y, z (+16 more)

### Community 12 - "SharedPtr"
Cohesion: 0.07
Nodes (23): NavSatFix, Range, SharedPtr, State, TwistStamped, DroneController::accel_cb(), DroneController::alt_cb(), DroneController::artag_pose_cb() (+15 more)

### Community 13 - "Request"
Cohesion: 0.19
Nodes (21): Request, SharedFuture, waitForArmingService, waitForCloseCamService, waitForLandService, waitForOpenCamService, waitForServoService, waitForSetModeService (+13 more)

### Community 14 - "vision_video_.cpp"
Cohesion: 0.18
Nodes (14): drawCrosshair, drawDetections, ensureStreamWriter, Detection, Mat, vector, VisionVideoNode::displayFrame(), VisionVideoNode::drawCrosshair() (+6 more)

### Community 15 - ".processFrame"
Cohesion: 0.25
Nodes (5): Mat, Point2f, Size, TagDetection, vector

### Community 16 - ".streamFrame"
Cohesion: 0.29
Nodes (5): Mat, Point2f, Size, TagDetection, vector

### Community 17 - "CameraViewer"
Cohesion: 0.21
Nodes (7): CameraViewer, main(), Node, Save the current frame as a timestamped PNG to self._save_dir., Initialize VideoWriter and start recording., Finalize and release the VideoWriter., Convert ROS Image message to OpenCV image and display it.

### Community 18 - "waitForService"
Cohesion: 0.17
Nodes (12): T, waitForService(), DroneController::waitForArmingService(), DroneController::waitForClearMissionService(), DroneController::waitForCloseCamService(), DroneController::waitForLandService(), DroneController::waitForOpenCamService(), DroneController::waitForPushMissionService() (+4 more)

### Community 19 - "fuseecam.cpp"
Cohesion: 0.18
Nodes (10): detectNet, objDimentions, depth, height, width, offset, x, y (+2 more)

### Community 20 - "VisionGeoNode::VisionGeoNode"
Cohesion: 0.22
Nodes (9): cameraInfoCallback, gpsCallback, imageCallback, openInputSource, poseCallback, relAltCallback, setupCameraIntrinsics, setupCamToBodyRotation (+1 more)

### Community 22 - "VisionVideoNode::processFrame"
Cohesion: 0.22
Nodes (9): displayFrame, estimatePose, infer, publishLegacyPose, publishLostPose, publishPose, streamFrame, updateFps (+1 more)

### Community 23 - "VisionGeoNode::processFrame"
Cohesion: 0.25
Nodes (8): displayFrame, infer, projectPixelToGPS, publishDetections, publishMarkers, updateFps, updateMarkers, VisionGeoNode::processFrame()

### Community 25 - "VisionVideoNode::VisionVideoNode"
Cohesion: 0.29
Nodes (7): cameraInfoCallback, imageCallback, initializeOpenVINO, openInputSource, setupCameraIntrinsics, setupObjectPoints, VisionVideoNode::VisionVideoNode()

### Community 26 - "string"
Cohesion: 0.33
Nodes (6): ov_status_e, startsWith, string, VisionVideoNode::getOpenVINOError(), VisionVideoNode::openInputSource(), VisionVideoNode::startsWith()

### Community 27 - ".detection"
Cohesion: 0.47
Nodes (4): convertCUDAtoBGR(), Mat, PoseStamped, uchar3

### Community 29 - "ConstSharedPtr"
Cohesion: 0.50
Nodes (4): ConstSharedPtr, DroneController::pose_cb(), DroneController::state_cb(), DroneController::vel_cb()

### Community 31 - "VisionVideoNode::imageCallback"
Cohesion: 0.50
Nodes (4): processFrame, SharedPtr, VisionVideoNode::cameraInfoCallback(), VisionVideoNode::imageCallback()

### Community 32 - "Float64"
Cohesion: 0.67
Nodes (3): Float64, DroneController::getCurrentCompassHdg(), DroneController::getCurrentRelAlt()

## Knowledge Gaps
- **415 isolated node(s):** `x`, `y`, `width`, `height`, `depth` (+410 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **10 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `DroneController` connect `DroneController` to `utils/control_.cpp`, `drone_controller_.hpp`, `fuseecam.cpp`, `Request`?**
  _High betweenness centrality (0.497) - this node is a cross-community bridge._
- **Why does `VisionGeoNode` connect `VisionGeoNode` to `vision_geo_.cpp`, `drone_controller_.hpp`, `VisionGeoNode::VisionGeoNode`, `VisionGeoNode::processFrame`?**
  _High betweenness centrality (0.232) - this node is a cross-community bridge._
- **Why does `VisionVideoNode` connect `VisionVideoNode` to `drone_controller_.hpp`, `vision_video_.cpp`, `VisionVideoNode::processFrame`, `VisionVideoNode::VisionVideoNode`, `string`, `VisionVideoNode::imageCallback`?**
  _High betweenness centrality (0.202) - this node is a cross-community bridge._
- **What connects `x`, `y`, `width` to the rest of the system?**
  _415 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `DroneController` be split into smaller, more focused modules?**
  _Cohesion score 0.012345679012345678 - nodes in this community are weakly interconnected._
- **Should `utils/control_.cpp` be split into smaller, more focused modules?**
  _Cohesion score 0.07303370786516854 - nodes in this community are weakly interconnected._
- **Should `VisionGeoNode` be split into smaller, more focused modules?**
  _Cohesion score 0.02702702702702703 - nodes in this community are weakly interconnected._