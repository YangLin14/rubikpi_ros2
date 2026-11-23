# CSE 276A Homework 4: Path Planning

**Fong-Yu Lin · TSN 200057839**

<!-- VIDEOS -->

<!-- STATUS -->

## 1. Setup and ground-truth map

I planned two ways to drive a rover between diagonally opposite points in a 10 ft square with a box in the middle. The distance planner tries to make the trip short. The safety planner gives the rover more room to absorb steering and localization error. Both use the same measured geometry, start, goal, and controller.

<!-- SETUP -->

The household objects from HW2 and the AprilTags (36h11) from HW3 provide localization observations. Object locations refer to the centers of their bases, and tag locations to the centers of the printed squares. As in the assignment figure, the tags on the four faces of the box reuse the ids of the edge tags, so the Class column gives the detector label and the Landmark column names each physical instance. I use the camera frame and turning-center convention from HW2. The camera is 12.5 cm ahead of the turning center with a measured yaw offset of about 7° to the left. Heading is positive counterclockwise from +x in the world frame.

![Arena, free grid samples and planned routes](planned.svg)

*Figure 1. The two plans in the same world frame. The dark rectangle is the obstacle. The light gray area excludes rover centers that would violate the required footprint and tracking margin. Dots show the free grid samples.*

## 2. Architecture

The measured arena goes to the offline planner and provides the known object positions to the EKF. The planner writes waypoints in meters and radians. On the rover, the camera feeds the HW2 YOLO detector with its range-and-bearing estimator, and the HW3 AprilTag detector, which gets range and bearing from solvePnP. Landmark observations, the IMU yaw rate, and commanded wheel speeds feed the EKF. The waypoint controller uses the pose to turn, drive, stop, and collect new measurements. The pose and observation CSV files record what the estimator and controller did on each run.

**Measured arena → planner → waypoints → waypoint controller → motor board**  
**Camera → YOLO objects and AprilTags → range/bearing → data association → EKF pose → waypoint controller**  
**Known landmark positions, IMU and wheel commands → EKF pose**

I reused known-map localization from HW2 because the landmark positions are available before the run. The planner can therefore compute both routes in the same coordinate frame as the controller. Because two tags share each id, every sighting is matched to the instance of its class with the smallest normalized innovation squared (NIS). If a second instance also passes the chi-square gate (13.8, 2 degrees of freedom), the sighting is skipped instead of guessed. The two instances of an id are more than a meter apart, so with the known start pose the gate separates them. YOLO detections that fall on a recently seen tag are dropped, since YOLO can label a tag's black square as the cereal box. A detection matched to a landmark that, according to the EKF, lies outside the camera's field of view (55° from the camera axis plus a 10° margin) must be a false positive. In the trial run YOLO labeled clutter outside the arena as the cereal box while the real box was behind the rover. Such detections are skipped and do not count toward the consecutive rejections that stop a run. The HW4 navigator stops a run if, according to the EKF, it misses a waypoint by more than the tracking margin, which is the error the planned clearance allows for.

## 3. Representation and search

I represented the floor as an eight-connected grid with extra grid lines through the exact start and goal. Obstacles are axis-aligned rectangles. A circle centered on the rover turning center bounds the rover during a turn. For each edge, the planner computes the smallest distance from the whole line segment to a box or the arena boundary, then subtracts the rover radius. An edge is usable when that clearance meets the tracking margin. Checking the whole segment also catches diagonal steps crossing a corner.

### 3.1 Shortest distance

I used A* with each edge's length in meters as its cost and the straight-line distance to the goal as the heuristic. Straight-line distance lower-bounds the cost of any remaining path in the graph, so the first goal popped by A* gives the shortest graph route. Distance is the planning objective. The rover's elapsed time also depends on its stops, turns, and localization updates.

### 3.2 Maximum safety

I maximized the route's smallest clearance (a maximin, or widest-path, problem). The start and goal are given by the task, and here the start sits close to the arena edge. Judged by plain clearance, the start itself would be the bottleneck of every route, and the safety route would collapse onto the shortest one. I therefore judge each point by its clearance only where it is worse than the start or goal forces: a point at distance d from an endpoint may be as close as that endpoint's clearance plus 0.5 d. A route leaving a corner diagonally gains clearance at 1/√2 per meter, so this rate is always achievable, while a route that keeps hugging a wall soon falls below it. Points within this allowance do not limit the route; elsewhere a point counts with its clearance, sampled every centimeter along each edge.

Each edge gets the smallest such clearance along it, and a max-priority search finds the route whose worst edge is best. A candidate path inherits the smaller of its current value and the next edge's value, so the first goal popped gives the largest attainable value in the graph. I then ran the length-based search again using only edges meeting that value. This chooses the shortest graph route among equally safe routes. All edges of both routes also keep the tracking margin.

For both routes, I reduced the number of stops by joining vertices along the original path with straight segments. Each new segment passes the same continuous clearance check and meets the respective planner threshold. A shortest-distance tie break selects among waypoint sequences with the same number of legs. The final figures and table refer to the exported waypoint polylines.

## 4. Results

| Plan | Distance (m) | Minimum clearance (m) | Obstacle clearance (m) | Drive legs |
| --- | ---: | ---: | ---: | ---: |
<!-- PLAN_TABLE -->

Clearance is measured from the edge of the modeled circular footprint. Minimum clearance includes the arena boundary, which is set by the start and goal; obstacle clearance counts the box only. In the executed comparison I use the EKF center samples to estimate distance, clearance and cross-track error. The command span runs between the first and last logged nonzero wheel commands, including any intervening stops. Tape measurements at the final stop and observations from both videos are used to assess physical performance.

<!-- RUN_RESULTS -->

## 5. Discussion

The two objectives answer different questions. Length-based search trades clearance for a shorter path until it reaches the chosen margin. Maximin search values the tightest point on the route and can add distance or turns to give that point more room. The clearance computation assumes stationary obstacles and a measured arena. The controller relies on its pose estimate, so I also inspect localization updates and the physical stop position when evaluating a run.
