import java.util.*;
import java.io.*;

public class Robot {

    private static String CSV_SPLIT_BY = ",";

    private static class Point {
        public static enum State {
            OBSTACLE,
            START,
            END,
            EMPTY
        };

        public int column;
        public int row;
        public State state;
        public int visitCount;

        public boolean canVisitPoint() {
            return state != State.OBSTACLE;
        }

        public double getHeuristic() {
            if (mEndPoint == null) {
                throw new IllegalStateException("End goal not set yet");
            }
            return Math.abs(mEndPoint.column - column) + Math.abs(mEndPoint.row - row);
        }

        public static Point createFromValue(String value, int rowIndex, int colIndex) {
            Point point = new Point();
            point.row = rowIndex;
            point.column = colIndex;

            if (value.equals("A")) {
                point.state = State.START;
            } else if (value.equals("B")) {
                point.state = State.END;
            } else if (value.equals("X")) {
                point.state = State.OBSTACLE;
            } else {
                point.state = State.EMPTY;
            }
            return point;
        }

        public String getVisitCountValue() {
            switch (state) {
                case OBSTACLE:
                    return "X";
                case START:
                    return "A";
                case END:
                    return "B";
                default:
                    return Integer.toString(visitCount);
            }
        }

        public String getValue(boolean isSolution) {
            switch (state) {
                case OBSTACLE:
                    return "X";
                case START:
                    return "A";
                case END:
                    return "B";
                default:
                    if (isSolution) {
                        return "P";
                    } else {
                       return "0";
                    }
            }
        }
    }

    private static class Path implements Comparable {
        public Point point;
        public Path parent;
        public double cost;
        public double costWithHeuristic;

        public Path(Path parent, Point to) {
            this.parent = parent;
            this.point = to;
            this.cost = parent == null ? 0 : parent.cost + 1; 
            this.costWithHeuristic = this.cost + to.getHeuristic();
        }

        public int compareTo(Object o) {
            Path p = (Path) o;
            return (int)(costWithHeuristic - p.costWithHeuristic);
        }
    }

    private static Point[][] mNeighborGrid;
    private static Point mStartPoint;
    private static Point mEndPoint;

    private static PriorityQueue<Path> mPaths;
    private static HashMap<Point, Double> mDistanceMap;

    public static void main(String[] args) {
        mPaths = new PriorityQueue<Path>();
        mDistanceMap = new HashMap<Point, Double>();

        if (args.length == 0) {
            System.out.println("You must provide a csv filaname");
            return;
        }

        try {
            processInput(args[0]);
        } catch (IllegalArgumentException e) {
            System.out.println("Error processing input: " + e.getMessage());
            return;
        }

        System.out.println("Original Grid:");
        printGrid(null);

        System.out.println("------------------------");
        System.out.println("Solution:");
        List<Point> solution = computePath();
        if (solution == null) {
            System.out.println("No solution found");
        } else {
            printGrid(solution);
        }

        System.out.println("------------------------");
        System.out.println("Visit counts:");
        printVisitCountGrid();
    }

    private static void printVisitCountGrid() {
        if (mNeighborGrid == null) {
            throw new IllegalStateException("Grid not initialized");
        }

        int numOfVisitedPoints = 0;
        int numOfNonVisitedPoints = 0;

        for (int i = 0; i < mNeighborGrid.length; ++i) {
            for (int j = 0; j < mNeighborGrid.length; ++j) {
                Point p = mNeighborGrid[i][j];
                if (p.visitCount > 0) {
                    numOfVisitedPoints++;
                } else {
                    numOfNonVisitedPoints++;
                }
                System.out.print(p.getVisitCountValue() + "\t");
            }
            System.out.print("\n");
        }

        System.out.println("Number of non-visited points = " + numOfNonVisitedPoints);
        System.out.println("Number of visited points = " + numOfVisitedPoints);
    }

    private static void printGrid(List<Point> solution) {
        if (mNeighborGrid == null) {
            throw new IllegalStateException("Grid not initialized");
        }

        HashSet<Point> solutionPoints = new HashSet<Point>();
        if (solution != null) {
            for (Point p : solution) {
                solutionPoints.add(p);
            }
        }

        for (int i = 0; i < mNeighborGrid.length; ++i) {
            for (int j = 0; j < mNeighborGrid.length; ++j) {
                Point p = mNeighborGrid[i][j];
                System.out.print(p.getValue(solutionPoints.contains(p)) + "\t");
            }
            System.out.print("\n");
        }
        if (solution != null) {
            System.out.println("Solution path count = " + solution.size());
        }
    }

    private static ArrayList<Point> computePath() {
        Path startPath = new Path(null, mStartPoint);
        mPaths.offer(startPath);

        Path path;
        while (true) {
            path = mPaths.poll();

            // No paths found
            if (path == null) {
                return null;
            }

            // Reached the goal
            if (path.point.equals(mEndPoint)) {
                ArrayList<Point> solutionPath = new ArrayList<Point>();
                solutionPath.add(path.point);
                while (path.parent != null) {
                    solutionPath.add(path.parent.point);
                    path = path.parent;
                }
                return solutionPath;
            }

            expandPath(path);
        }
    }

    private  static void expandPath(Path path) {
        Point point = path.point;
        point.visitCount++;

        Double min = mDistanceMap.get(point);

        // Check to see if this point has been visited with a lower cost.
        // If so, do not expand again.
        if (min == null || min.doubleValue() > path.costWithHeuristic) {
            mDistanceMap.put(point, path.costWithHeuristic);
        } else {
            return;
        }

        List<Point> neighbors = getNeighbors(point);
        for (Point p : neighbors) {
            Path neighborPath = new Path(path, p);
            mPaths.offer(neighborPath);
        }

    }

    private static List<Point> getNeighbors(Point point) {
        List<Point> neighborPoints = new ArrayList<Point>();
        
        if (point.row > 0) {
            Point neighbor = mNeighborGrid[point.row-1][point.column];
            if (neighbor.canVisitPoint()) {
                neighborPoints.add(neighbor);
            }
        } 

        if (point.row < mNeighborGrid.length - 1) {
            Point neighbor = mNeighborGrid[point.row+1][point.column];
            if (neighbor.canVisitPoint()) {
                neighborPoints.add(neighbor);
            }
        }

        if (point.column > 0) {
            Point neighbor = mNeighborGrid[point.row][point.column-1];
            if (neighbor.canVisitPoint()) {
                neighborPoints.add(neighbor);
            }
        }


        if (point.column < mNeighborGrid.length - 1) {
            Point neighbor = mNeighborGrid[point.row][point.column+1];
            if (neighbor.canVisitPoint()) {
                neighborPoints.add(neighbor);
            }
        }

        return neighborPoints;
    }

    public static void processInput(String fileName) {
        BufferedReader br = null;
        String line;
        int rowIndex = 0;
        int colIndex = 0;

        try {
            br = new BufferedReader(new FileReader(fileName));
            while ((line = br.readLine()) != null) {
                colIndex = 0;
                String[] input = line.split(CSV_SPLIT_BY);

                if (mNeighborGrid == null) {
                    mNeighborGrid = new Point[input.length][input.length];
                }

                for (String s : input) {
                    if (colIndex >= mNeighborGrid.length) {
                        throw new IllegalArgumentException("Too many columns found");
                    }

                    Point point = Point.createFromValue(s, rowIndex, colIndex);

                    if (point.state == Point.State.START) {
                        if (mStartPoint != null) {
                            throw new IllegalArgumentException("Too many start points found");
                        }

                        mStartPoint = point;
                    } else if (point.state == Point.State.END) {
                        if (mEndPoint != null) {
                            throw new IllegalArgumentException("Too many end points found");
                        }

                        mEndPoint = point;
                    }

                    mNeighborGrid[rowIndex][colIndex] = point;
                    colIndex++;
                }

                rowIndex++;
                if (colIndex < mNeighborGrid.length) {
                    throw new IllegalArgumentException("Too few columns found");
                } else if (rowIndex > mNeighborGrid.length) {
                    throw new IllegalArgumentException("Too many rows found");
                }
            }
        } catch (FileNotFoundException e) {
            throw new IllegalArgumentException(e.getMessage());
        } catch (IOException e) {
            throw new IllegalArgumentException(e.getMessage());
        } finally {
            if (br != null) {
                try {
                    br.close();
                } catch (IOException e) {
                    throw new IllegalArgumentException(e.getMessage());
                }
            }
        }

        if (rowIndex < mNeighborGrid.length) {
            throw new IllegalArgumentException("Too few rows found");
        }

        if (mStartPoint == null) {
            throw new IllegalArgumentException("No start point found");
        }

        if (mEndPoint == null) {
            throw new IllegalArgumentException("No end point found");
        }
    }
}