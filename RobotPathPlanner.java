import java.util.*;

public class RobotPathPlanner {

    // Grid cell representation
    static class Cell {
        int x, y;
        Cell(int x, int y) { this.x = x; this.y = y; }

        @Override public boolean equals(Object o) {
            if (!(o instanceof Cell)) return false;
            Cell c = (Cell) o;
            return x == c.x && y == c.y;
        }

        @Override public int hashCode() { return Objects.hash(x, y); }
    }

    // Robot with start and goal positions
    static class Robot {
        Cell start, goal;
        Robot(Cell s, Cell g) { start = s; goal = g; }
    }

    // Timed cell for space-time planning
    static class TimedCell {
        Cell cell;
        int time;
        TimedCell parent; // for path reconstruction
        TimedCell(Cell c, int t, TimedCell p) { cell = c; time = t; parent = p; }
    }

    int rows, cols;
    boolean[][] obstacles;
    List<Robot> robots;
    Map<Integer, List<TimedCell>> plannedPaths = new HashMap<>();

    public RobotPathPlanner(int rows, int cols, boolean[][] obstacles, List<Robot> robots) {
        this.rows = rows;
        this.cols = cols;
        this.obstacles = obstacles;
        this.robots = robots;
    }

    // A* planning with space-time collision check
    public List<TimedCell> planAStarWithPriority(int robotIndex) {
        Robot r = robots.get(robotIndex);
        PriorityQueue<TimedCell> pq = new PriorityQueue<>(Comparator.comparingInt(a -> a.time));
        Map<String, Boolean> visited = new HashMap<>();

        pq.add(new TimedCell(r.start, 0, null));
        int[] dx = {1,-1,0,0,0}; // allow wait
        int[] dy = {0,0,1,-1,0};

        while (!pq.isEmpty()) {
            TimedCell curr = pq.poll();
            String key = curr.cell.x + "," + curr.cell.y + "," + curr.time;
            if (visited.containsKey(key)) continue;
            visited.put(key, true);

            if (curr.cell.equals(r.goal)) {
                return reconstructPath(curr);
            }

            // explore neighbors
            for (int i = 0; i < 5; i++) {
                int nx = curr.cell.x + dx[i];
                int ny = curr.cell.y + dy[i];
                Cell next = new Cell(nx, ny);

                if (!isFree(next)) continue;
                if (collides(robotIndex, curr.cell, next, curr.time + 1)) continue;

                pq.add(new TimedCell(next, curr.time + 1, curr));
            }
        }
        return null; // no path found
    }

    // reconstruct full path from goal to start
    private List<TimedCell> reconstructPath(TimedCell goal) {
        List<TimedCell> path = new ArrayList<>();
        TimedCell current = goal;
        while (current != null) {
            path.add(current);
            current = current.parent;
        }
        Collections.reverse(path);
        return path;
    }

    // Check if cell is inside grid and not obstacle
    boolean isFree(Cell c) {
        return c.x >= 0 && c.x < rows &&
               c.y >= 0 && c.y < cols &&
               !obstacles[c.x][c.y];
    }

    // Collision check for current robot
    boolean collides(int robotIndex, Cell from, Cell to, int time) {
        for (int i = 0; i < robotIndex; i++) {
            List<TimedCell> path = plannedPaths.get(i);
            if (path == null) continue;

            // vertex collision
            for (TimedCell tc : path) {
                if (tc.time == time && tc.cell.equals(to)) return true;
            }

            // edge collision (swap)
            for (int t = 0; t < path.size() - 1; t++) {
                if (t + 1 != time) continue;
                Cell a1 = path.get(t).cell;
                Cell a2 = path.get(t + 1).cell;
                if (from.equals(a2) && to.equals(a1)) return true;
            }
        }
        return false;
    }

    // Plan all robots sequentially (prioritized planning)
    public void planAll() {
        for (int i = 0; i < robots.size(); i++) {
            List<TimedCell> path = planAStarWithPriority(i);
            plannedPaths.put(i, path);
        }
    }

    // Print full paths with time steps
    public void printPaths() {
        for (int i = 0; i < robots.size(); i++) {
            System.out.println("Robot " + i + " path:");
            List<TimedCell> path = plannedPaths.get(i);
            if (path == null) {
                System.out.println("  No path found!");
            } else {
                for (TimedCell tc : path) {
                    System.out.println("  (" + tc.cell.x + "," + tc.cell.y + ") at t=" + tc.time);
                }
            }
        }
    }

    public static void main(String[] args) {
        // Example: 5x5 grid with no obstacles
        boolean[][] grid = new boolean[5][5]; // false = free cell

        // Define robots (start, goal)
        List<Robot> robots = new ArrayList<>();
        robots.add(new Robot(new Cell(0,0), new Cell(4,4)));
        robots.add(new Robot(new Cell(4,0), new Cell(0,4)));

        RobotPathPlanner planner = new RobotPathPlanner(5, 5, grid, robots);
        planner.planAll();
        planner.printPaths();
    }
}

