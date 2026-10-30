import math
import heapq

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray

from robot_hardware.utils.yaml_loader import load_node_params
from robot_hardware.utils.timeout import is_data_fresh


class PathPlannerNode(Node):

    # Initialize node
    def __init__(self):
        super().__init__("path_planner_node")

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.ball_x = -1.0
        self.ball_y = -1.0
        self.obstacles = []
        self.obstacles_changed = True
        self.path = []
        self.last_robot_x = 0.0
        self.last_robot_y = 0.0
        self.last_ball_x = -1.0
        self.last_ball_y = -1.0
        self.last_tag_seen_time = None
        self.last_ball_msg_time = None
        self.last_robot_msg_time = None
        self.replan_counter = 0

        self._load_config()

        self.create_subscription(Pose2D, "/vision/robot_pose", self.robot_callback, 10)
        self.create_subscription(Pose2D, "/vision/ball_position", self.ball_callback, 10)
        self.create_subscription(Float32MultiArray, "/vision/obstacles", self.obstacle_callback, 10)

        self.pub_planned_path = self.create_publisher(Float32MultiArray, "/planned_path", 10)

        self.create_timer(0.05, self.planning_loop)

        self.get_logger().info("PathPlannerNode started")

    # Load config
    def _load_config(self):
        params = load_node_params(self)
        if params is not None:
            for k, v in params.items():
                setattr(self, k, v)

    # Robot callback
    def robot_callback(self, msg):
        now = self.get_clock().now()
        if msg.x != 0.0 or msg.y != 0.0:
            self.robot_x = msg.x
            self.robot_y = msg.y
            self.last_tag_seen_time = now
            self.last_robot_msg_time = now

    # Ball callback
    def ball_callback(self, msg):
        self.last_ball_msg_time = self.get_clock().now()
        self.ball_x = msg.x
        self.ball_y = msg.y

    # Obstacle callback
    def obstacle_callback(self, msg):
        data = msg.data
        if len(data) % 4 != 0:
            self.get_logger().warn(f"[OBS] malformed obstacle data length={len(data)}")
            return
        new_obs = []
        for i in range(0, len(data), 4):
            ox, oy, ow, oh = data[i], data[i + 1], data[i + 2], data[i + 3]
            if not (math.isfinite(ox) and math.isfinite(oy) and math.isfinite(ow) and math.isfinite(oh)):
                continue
            if ow <= 0 or oh <= 0:
                continue
            new_obs.append((ox, oy, ow, oh))
        self.obstacles_changed = (new_obs != self.obstacles)
        self.obstacles = new_obs

    # Planning loop
    def planning_loop(self):
        now = self.get_clock().now()
        robot_fresh = is_data_fresh(now, self.last_robot_msg_time, self.vision_timeout)
        ball_fresh = is_data_fresh(now, self.last_ball_msg_time, self.vision_timeout)
        tag_is_fresh = is_data_fresh(now, self.last_tag_seen_time, self.vision_timeout)
        has_robot = (self.robot_x != 0.0 or self.robot_y != 0.0) and (tag_is_fresh or robot_fresh)
        has_ball = self.ball_x >= 0.0 and self.ball_y >= 0.0 and ball_fresh

        use_path = self.obstacle_avoidance and has_ball and has_robot
        if not use_path:
            return

        self.replan_counter += 1
        should_replan = self.obstacles_changed
        should_replan = should_replan or not self.path
        should_replan = should_replan or abs(self.robot_x - self.last_robot_x) > self.replan_threshold
        should_replan = should_replan or abs(self.robot_y - self.last_robot_y) > self.replan_threshold
        should_replan = should_replan or abs(self.ball_x - self.last_ball_x) > self.replan_threshold
        should_replan = should_replan or abs(self.ball_y - self.last_ball_y) > self.replan_threshold
        should_replan = should_replan or (self.replan_counter % self.path_replan_interval == 0)

        if should_replan:
            self.get_logger().info("[PLAN] planning path")
            self.plan_path()
            self.last_robot_x = self.robot_x
            self.last_robot_y = self.robot_y
            self.last_ball_x = self.ball_x
            self.last_ball_y = self.ball_y
            self.obstacles_changed = False

    # Build grid
    def build_grid(self, width=640, height=480):
        cs = self.astar_cell_size
        gw = width // cs
        gh = height // cs
        grid = [[0] * gw for _ in range(gh)]
        r_px = self.astar_obstacle_radius * cs
        clearance = max(1, int(getattr(self, "astar_clearance", 1)))
        for ox, oy, ow, oh in self.obstacles:
            if ox < 0 or oy < 0:
                continue
            gx_min = max(0, int((ox - ow / 2 - r_px) / cs) - clearance)
            gx_max = min(gw - 1, int((ox + ow / 2 + r_px) / cs) + clearance)
            gy_min = max(0, int((oy - oh / 2 - r_px) / cs) - clearance)
            gy_max = min(gh - 1, int((oy + oh / 2 + r_px) / cs) + clearance)
            for gy in range(gy_min, gy_max + 1):
                for gx in range(gx_min, gx_max + 1):
                    grid[gy][gx] = 1
        for gy in range(gh):
            for gx in range(gw):
                px = gx * cs + cs // 2
                py = gy * cs + cs // 2
                if px < self.field_left or px > self.field_right \
                   or py < self.field_top or py > self.field_bottom:
                    grid[gy][gx] = 1
        return grid

    # Heuristic
    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    # A* search
    def astar(self, grid, start, goal):
        h = len(grid)
        w = len(grid[0])
        if not (0 <= start[0] < w and 0 <= start[1] < h):
            return None
        if not (0 <= goal[0] < w and 0 <= goal[1] < h):
            return None
        if grid[start[1]][start[0]] == 1:
            return None
        if grid[goal[1]][goal[0]] == 1:
            return None
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
                     (0, 1), (1, -1), (1, 0), (1, 1)]
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                if grid[ny][nx] == 1:
                    continue
                if dx != 0 and dy != 0:
                    if grid[current[1]][nx] == 1 or grid[ny][current[0]] == 1:
                        continue
                neighbor = (nx, ny)
                step = math.sqrt(dx * dx + dy * dy)
                tentative_g = g_score[current] + step
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
        return None

    # Grid to pixel
    def grid_to_pixel(self, gx, gy):
        cs = self.astar_cell_size
        return gx * cs + cs // 2, gy * cs + cs // 2

    # Line of sight
    def line_of_sight(self, a, b, grid):
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            if not (0 <= y0 < len(grid) and 0 <= x0 < len(grid[0])):
                return False
            if grid[y0][x0] != 0:
                return False
            if x0 == x1 and y0 == y1:
                return True
            prev_x, prev_y = x0, y0
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
            if x0 != prev_x and y0 != prev_y:
                if grid[prev_y][x0] != 0 or grid[y0][prev_x] != 0:
                    return False

    # Smooth path
    def smooth_path(self, path, grid):
        if len(path) <= 2:
            return path
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            for j in range(len(path) - 1, i, -1):
                if self.line_of_sight(path[i], path[j], grid):
                    smoothed.append(path[j])
                    i = j
                    break
        return smoothed

    # Plan path
    def plan_path(self):
        if not is_data_fresh(self.get_clock().now(), self.last_tag_seen_time, self.vision_timeout):
            self.get_logger().warn("[PLAN] skip replan, robot pose stale")
            return
        grid = self.build_grid()
        gw = len(grid[0])
        gh = len(grid)
        start_gx = max(0, min(gw - 1, int(round(self.robot_x / self.astar_cell_size))))
        start_gy = max(0, min(gh - 1, int(round(self.robot_y / self.astar_cell_size))))
        goal_gx = max(0, min(gw - 1, int(round(self.ball_x / self.astar_cell_size))))
        goal_gy = max(0, min(gh - 1, int(round(self.ball_y / self.astar_cell_size))))
        blocked_start = grid[start_gy][start_gx] == 1
        blocked_goal = grid[goal_gy][goal_gx] == 1
        result = self.astar(grid, (start_gx, start_gy), (goal_gx, goal_gy))
        if result is None:
            self.get_logger().warn(
                f"[PLAN] No path! Start=({start_gx},{start_gy}) Goal=({goal_gx},{goal_gy}) "
                f"Obs={len(self.obstacles)} start_blocked={blocked_start} goal_blocked={blocked_goal}"
            )
            return
        smoothed = self.smooth_path(result, grid)
        safe = all(
            self.line_of_sight(smoothed[i], smoothed[i + 1], grid)
            for i in range(len(smoothed) - 1)
        )
        if not safe:
            self.get_logger().warn(
                f"[PLAN] Smoothing produced blocked edge ({len(smoothed)} wp), "
                f"using raw A* ({len(result)} wp)"
            )
            smoothed = result
        self.path = [self.grid_to_pixel(gx, gy) for gx, gy in smoothed]
        self._publish_planned_path()
        self.get_logger().info(
            f"[PLAN] Path {len(result)}→{len(smoothed)} wp | "
            f"Start=({start_gx},{start_gy}) Goal=({goal_gx},{goal_gy}) | "
            f"Obs={len(self.obstacles)}",
            throttle_duration_sec=1.0
        )

    # Publish path
    def _publish_planned_path(self):
        msg = Float32MultiArray()
        flat = []
        for px, py in self.path:
            flat.extend([float(px), float(py)])
        msg.data = flat
        self.pub_planned_path.publish(msg)


# Main entry
def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()