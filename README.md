# RobotX 2026 — UAV Control 
***CLAUDE ❤️ El Jausyan***

ROS 2 (Humble) workspace for the RobotX UAV, flown on **ArduPilot** through MAVROS.
Two packages do the work:

| Package | What it does |
|---|---|
| `vision_geo` | The drone's eyes. Runs YOLO on the downward camera, maps Task 1 buoys to GPS, and publishes one centering topic per object type. |
| `control` | The drone's pilot. Runs the mission sequence (`src/control/src/mission/control.cpp`): take off, fly, wait for orders, center, drop. |

The two programs never call each other directly — they only talk through ROS topics.
Vehicles (UAV, USV, UUV) talk to each other through one shared topic: **`/mission/order`**.

---

## Quick start

```bash
# build
source /opt/ros/humble/setup.bash
colcon build --packages-select vision_msgs vision_geo control
source install/setup.bash

# 1. start vision (on the ground — it stays idle until "UAV-GO")
./script/launch_vision_geo.sh          # real drone
./script/launch_vision_geo.sh sim      # Gazebo

# 2. start the mission
./script/launch_mission.sh
```

| Config | Used by | Rebuild after editing? |
|---|---|---|
| `config/config.yaml` | control (mission values, centering, servo) | **No** — `launch_mission.sh` reads it directly |
| `src/vision_geo/config/vision_geo.yaml` / `vision_geo_sim.yaml` | vision_geo | **Yes** — the launch script reads the installed copy |

---

## `/mission/order` — command reference

Type: `std_msgs/msg/String`. Everyone publishes and listens on this one topic.

| Command | Sent by | Meaning |
|---|---|---|
| `UAV-GO` | UAV control (`pubCommand`) | "Drone is at survey altitude" → **starts vision_geo** (Task 1) |
| `MISSION-DONE` | USV | Task 1 transit finished → UAV stops watching the buoys and goes to the USV |
| `UAV-GO:<TIN>:<CIRCLE>` | UUV (Task 2) / USV (Task 3) | "Deliver the `<TIN>` tin onto the `<CIRCLE>` circle" |
| `UAV-HOLD` | anyone | Ignored by the UAV (logged as unexpected) — it just keeps holding |

Colors: `RED`, `GREEN`, `BLUE`.

- **1st color = which tin to drop** (picks the servo)
- **2nd color = which circle to center on** (picks the vision topic)

Rules:
- Case does not matter (`uav-go:red:green` works).
- A message matches when it **starts with** the expected command, so `UAV-GO:RED:GREEN` counts as `UAV-GO`.
- The UAV only listens while it is waiting (`waitCommand()`); messages sent at other times are not seen.

Examples:

| Command | Tin dropped | Circle centered on | Centering topic |
|---|---|---|---|
| `UAV-GO:RED:GREEN` | red (`servo.channel_red`) | green | `/vision_geo/target/circle_green` |
| `UAV-GO:BLUE:RED` | blue (`servo.channel_blue`) | red | `/vision_geo/target/circle_red` |
| `UAV-GO:GREEN:GREEN` | green (`servo.channel_green`) | green | `/vision_geo/target/circle_green` |

Test from a terminal:

```bash
ros2 topic pub --once /mission/order std_msgs/msg/String "{data: 'UAV-GO'}"
ros2 topic pub --once /mission/order std_msgs/msg/String "{data: 'MISSION-DONE'}"
ros2 topic pub --once /mission/order std_msgs/msg/String "{data: 'UAV-GO:RED:GREEN'}"
ros2 topic echo /mission/order
```

---

## All topics

| Topic | Type | From → To | Content |
|---|---|---|---|
| `/mission/order` | `std_msgs/String` | all vehicles ↔ all vehicles | commands above |
| `/USV/global_position/global` | `sensor_msgs/NavSatFix` | USV → UAV control | USV GPS, read **once** by `goToVehicle()` |
| `/vision_geo/map/buoy_red` | `vision_msgs/DetectedObjectArray` | vision_geo → USV | buoys currently **red** (pass on starboard) |
| `/vision_geo/map/buoy_green` | same | vision_geo → USV | buoys currently **green** (pass on port) |
| `/vision_geo/map/buoy_entry` | same | vision_geo → USV | **flashing blue** = ENTRY |
| `/vision_geo/map/buoy_exit` | same | vision_geo → USV | **solid blue** = EXIT |
| `/vision_geo/map/buoy_off` | same | vision_geo → USV | unlit buoys |
| `/vision_geo/target/<class>` | `geometry_msgs/PoseStamped` | vision_geo → `centering_red()` | position of the **closest** object of that class, in meters |
| `/vision_geo/detections` | `vision_msgs/DetectedObjectArray` | vision_geo → debug | raw detections of the current frame |
| `/vision_geo/markers` | `visualization_msgs/MarkerArray` | vision_geo → RViz | buoy map spheres (color = state) |

**Buoy map object fields** (`/vision_geo/map/buoy_*`):
`latitude`, `longitude` (use these), `label` = `"buoy_<state>"`, `class_id` = buoy ID (stays the same for the same buoy),
`confidence`, `north_offset_m` / `east_offset_m` (meters from the UAV's home — UAV only).
The whole map is re-sent every camera frame, so the latest message always has every buoy.

**Target pose fields** (`/vision_geo/target/<class>`, camera frame, from solvePnP):
`x` = meters right of image center, `y` = meters down, `z` = distance to the object.
**All zeros = not detected.** If 2+ are visible, the closest one is sent.

---

## Mission flow (big picture)

```
 ground: vision_geo starts idle ──────────────── waits for "UAV-GO"
 ground: control starts → takeoff (asks "takeoff 1 if yes ?")
   │
 TASK 1  AUTO → survey point → GUIDED → climb to survey_alt
         pubCommand("UAV-GO")  ─────────────────► vision_geo starts detecting
         hover + map buoys     ─────────────────► /vision_geo/map/buoy_*  → USV
         waitCommand("MISSION-DONE")  ◄────────── USV
         goToVehicle(): read USV GPS once → AUTO → fly above USV → GUIDED
   │
 TASK 2  waitCommand("UAV-GO")  ◄──────────────── UUV: "UAV-GO:RED:GREEN"
         parseCommand → tin=red, circle=green
         centering_red(/vision_geo/target/circle_green) → descend → center again
         drop red tin (servo) → climb
   │
 TASK 3  waitCommand("UAV-GO")  ◄──────────────── USV: "UAV-GO:<TIN>:<CIRCLE>"
         AUTO → Task 3 platform → GUIDED
         center on circle → descend → center again → drop → climb
   │
         RTL (return home)
```

Every wait holds the drone still (it keeps sending its current position), so nothing drifts while waiting.

---

## Task 1 — Safe Passage (map the buoys for the USV)

**Goal:** the safe route through the buoy field is visible only from above. The UAV finds each buoy,
reads its light, and gives the USV the GPS of every buoy with its meaning.

**Step by step**
1. Take off, then fly to the survey point (`mission.survey_lat/lon/alt`) with a one-waypoint AUTO mission.
2. Switch to GUIDED and settle at `survey_alt` (15–20 m).
3. Publish `UAV-GO` on `/mission/order` → vision_geo wakes up and starts detecting.
4. Hover and watch. vision_geo turns every buoy into a GPS point and works out its light:

   | Light seen from above | State | Topic | USV meaning |
   |---|---|---|---|
   | flashing red | `red` | `/vision_geo/map/buoy_red` | pass on starboard |
   | flashing green | `green` | `/vision_geo/map/buoy_green` | pass on port |
   | flashing blue | `entry` | `/vision_geo/map/buoy_entry` | enter here |
   | solid blue | `exit` | `/vision_geo/map/buoy_exit` | leave here |
   | off | `off` | `/vision_geo/map/buoy_off` | obstacle |

5. Keep watching until the USV sends `MISSION-DONE` (timeout 300 s). If a buoy changes during the transit
   (Disruptive tier), it moves to its new state topic within a few seconds.
6. Fly above the USV (`goToVehicle()`), ready for Task 2.

**How the buoy map works (simple version)**
- Each detection's pixel is projected onto the water using the drone's altitude and tilt → latitude/longitude.
- One map point = one real buoy. Any buoy detection within `map_merge_radius_m` (2 m) is the same buoy,
  even if its color changes (a flashing light looks like "red, off, red, off…").
- The position is the average of every sighting, so it gets more accurate the longer the drone watches.
- The light state comes from the last `state_window_s` (6 s) of sightings:
  seen off between lit frames (or a ~1 s gap) → **flashing**; lit steadily for `solid_min_s` (3 s) → **solid**.

---

## Task 2 — Infrastructure Survey & Repair (deliver for the UUV)

**Goal:** the UUV repairs the pipeline, reads the two-color light flash, and asks the UAV to drop a tin on a circle.

**Step by step**
1. The UAV is holding above the USV, waiting (`waitCommand("UAV-GO")`).
2. The UUV publishes e.g. `UAV-GO:RED:GREEN` on `/mission/order`.
3. The UAV reads it and splits it: tin = `red`, circle = `green`.
4. It centers on the green circle using `/vision_geo/target/circle_green`.
5. It descends to `drop_alt`, centers again for precision.
6. If centered → opens the **red** tin's servo. If not centered → skips the drop.
7. Climbs to `approach_alt`, then waits for Task 3.

**From receiving the topic to centering — what happens inside**

```
"UAV-GO:RED:GREEN"                     arrives on /mission/order
   │  waitCommand()    uppercase + trim, check it starts with "UAV-GO"  → accepted
   │                   command = "UAV-GO:RED:GREEN"
   ▼
parseCommand()         split on ':' and lowercase → ["uav-go", "red", "green"]
   │                   order.tin = "red"   order.circle = "green"
   ▼
target = "circle_" + order.circle              → "circle_green"
topic  = "/vision_geo/target/" + target        → "/vision_geo/target/circle_green"
   │
   ▼
centering_red(..., topic)      subscribes to /vision_geo/target/circle_green
   │                           vision_geo publishes that topic because "circle_green"
   │                           is in its class_names (topic = prefix + class name)
   ▼
x/y offset → velocity commands → centered when within centering_tolerance
   │
   ▼
order.tin == "red" → servo.channel_red → controlServoRepeated(...)
```

The only rule: **the color word in the command must match the color in the vision class name**
(`GREEN` → `circle_green`).

> Note: Task 2 centers from where the drone is (above the USV). The circle must already be in the camera view.

---

## Task 3 — Coordinated Logistics (deliver for the USV)

**Goal:** the USV docks, puts out the fire, reads the two-color flash, and asks the UAV to deliver a tin
to a circle on the **other** platform.

**Step by step**
1. The UAV waits for `UAV-GO:<TIN>:<CIRCLE>` from the USV (`command_timeout`, default 600 s).
   No order in time → the UAV returns home (RTL) and the program stops.
2. Split the command exactly as in Task 2.
3. Fly to the Task 3 platform (`mission.task3_lat/lon`) at `approach_alt` with a one-waypoint AUTO mission, then GUIDED.
4. Center on `/vision_geo/target/circle_<CIRCLE>` → descend to `drop_alt` → center again.
5. Centered → open the `<TIN>` servo. Not centered → skip the drop.
6. Climb back up, then **RTL** — mission complete.

---

## Main functions

**control** (`src/control/utils/control_.cpp`, declared in `include/control_.hpp`)

| Function | What it does |
|---|---|
| `takeoff()` | GUIDED → asks for confirmation → arm → takeoff command → wait for altitude |
| `setMode()` | switch flight mode (`GUIDED`, `AUTO`, `RTL`, `LAND`, `LOITER`) |
| `clearMission()` + `pushMission()` + `setMode("AUTO")` + `waitForWP()` | fly to a GPS point with a one-waypoint mission |
| `holdPosition()` | stay at the current position for N seconds |
| `fix_alt()` | climb/descend to an altitude |
| `pubCommand()` | publish a command on `/mission/order` |
| `waitCommand()` | hold position until a command arrives (returns `false` on timeout) |
| `parseCommand()` | `"UAV-GO:RED:GREEN"` → `tin="red"`, `circle="green"` |
| `goToVehicle()` | read the USV GPS once, fly above it, back to GUIDED |
| `centering_red()` | follow a `PoseStamped` target topic until centered |
| `controlServoRepeated()` | send a servo PWM command several times |

**vision_geo** (`src/vision_geo/`)

| Part | What it does |
|---|---|
| `src/vision_geo.cpp` → `main()` | builds the node, calls `waitCommand("UAV-GO")`, then starts detecting |
| `publishTargets()` | per class: solvePnP on the closest detection → `/vision_geo/target/<class>` |
| `projectPixelToGPS()` + `updateMarkers()` | Task 1 buoys → GPS map points |
| `updateBuoyState()` | light history → `red` / `green` / `entry` / `exit` / `off` |
| `publishBuoyMap()` | per state → `/vision_geo/map/buoy_<state>` |

---

## Important config values

`config/config.yaml` (control) — keep the `.0` on decimal values or the node will not start.

| Key | Meaning |
|---|---|
| `mission.takeoff_altitude` | takeoff height, also the height used above the USV |
| `mission.survey_lat/lon/alt` | Task 1 survey point |
| `mission.approach_alt` / `drop_alt` | first centering height / final centering + drop height |
| `mission.task3_lat/lon` | Task 3 platform |
| `communication.order_topic` | `/mission/order` |
| `communication.command_timeout` | how long to wait for an order |
| `centering_red.*` | centering speed, gains, tolerance |
| `servo.channel_red/green/blue`, `servo_buka`, `signal_repeat` | dropper per tin color (**placeholders — not mapped yet**) |

`vision_geo.yaml` (vision)

| Key | Meaning |
|---|---|
| `class_names` | YOLO classes, **same order as the model** |
| `object_size_m` | real size per class (solvePnP uses it for the distance) |
| `geo_classes` | classes that get GPS mapping — **the buoys only** |
| `order_topic`, `wait_for_order`, `order_timeout` | start trigger |
| `state_window_s`, `flash_gap_min_s`, `flash_gap_max_s`, `solid_min_s` | flashing/solid detection |
| `track_exit_frames` | keep above ~1 s of frames so a flashing light keeps its track |

---

## Not done yet

- Reporting to RoboCommand.
- Task 2 start point: finding the GREEN indicator buoy and sending it to the UUV.
- The YOLO model must be trained with the classes in `class_names` (current `.onnx` files are not).
- Servo channels/PWM are placeholders.
- Tins are pre-loaded (one servo per color); the drone does not pick tins up.
- `pubCommand()` sends once — if vision_geo misses the first `UAV-GO`, publish it again.
