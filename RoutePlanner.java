import java.util.*;

/**
 * Route Planner: shortest path using Dijkstra and A* on a weighted road graph.
 * Single file: save as RoutePlanner.java, then run it in VS Code (Run button) or:
 *   javac RoutePlanner.java
 *   java RoutePlanner
 */
public class RoutePlanner {

    // ======================= MODEL =======================

    enum CostMode { DISTANCE, TIME }

    static class Node {
        final int id;
        final String name;
        final double lat, lon;

        Node(int id, String name, double lat, double lon) {
            this.id = id;
            this.name = name;
            this.lat = lat;
            this.lon = lon;
        }
    }

    static class Edge {
        final int to;
        final double distanceKm, speedKmph;

        Edge(int to, double distanceKm, double speedKmph) {
            this.to = to;
            this.distanceKm = distanceKm;
            this.speedKmph = speedKmph;
        }

        double timeMin() {
            return distanceKm / speedKmph * 60.0;
        }

        double cost(CostMode mode) {
            return mode == CostMode.DISTANCE ? distanceKm : timeMin();
        }
    }

    /** Road network as an adjacency list. Node ids are 0..n-1. */
    static class Graph {
        final List<Node> nodes = new ArrayList<>();
        final List<List<Edge>> adj = new ArrayList<>();
        final Map<String, Integer> nameToId = new HashMap<>();
        final Set<Long> blocked = new HashSet<>();
        double maxSpeed = 0;
        int edgeCount = 0;

        int addNode(String name, double lat, double lon) {
            String key = name.trim().toLowerCase();
            if (nameToId.containsKey(key)) throw new IllegalArgumentException("Duplicate node: " + name);
            int id = nodes.size();
            nodes.add(new Node(id, name.trim(), lat, lon));
            adj.add(new ArrayList<>());
            nameToId.put(key, id);
            return id;
        }

        void addDirected(int from, int to, double distKm, double speed) {
            if (distKm <= 0 || speed <= 0) throw new IllegalArgumentException("Distance and speed must be positive");
            // Keep the A* heuristic valid: a road can't be shorter than the straight line
            distKm = Math.max(distKm, haversine(nodes.get(from), nodes.get(to)) + 1e-9);
            adj.get(from).add(new Edge(to, distKm, speed));
            maxSpeed = Math.max(maxSpeed, speed);
            edgeCount++;
        }

        void addRoad(int a, int b, double distKm, double speed) {
            addDirected(a, b, distKm, speed);
            addDirected(b, a, distKm, speed);
        }

        void addRoad(String a, String b, double distKm, double speed) {
            addRoad(findId(a), findId(b), distKm, speed);
        }

        int findId(String name) {
            Integer id = nameToId.get(name.trim().toLowerCase());
            return id == null ? -1 : id;
        }

        int size() { return nodes.size(); }

        void blockRoad(int a, int b)   { blocked.add(key(a, b)); blocked.add(key(b, a)); }
        void unblockRoad(int a, int b) { blocked.remove(key(a, b)); blocked.remove(key(b, a)); }
        boolean isBlocked(int a, int b) { return !blocked.isEmpty() && blocked.contains(key(a, b)); }

        private static long key(int a, int b) { return ((long) a << 32) | (b & 0xffffffffL); }
    }

    static class Route {
        final boolean found;
        final List<Integer> path;
        final double totalCost;
        final int explored;
        final long nanos;

        Route(boolean found, List<Integer> path, double totalCost, int explored, long nanos) {
            this.found = found;
            this.path = path;
            this.totalCost = totalCost;
            this.explored = explored;
            this.nanos = nanos;
        }

        double distanceKm(Graph g) {
            double sum = 0;
            for (int i = 0; i + 1 < path.size(); i++) sum += bestEdge(g, path.get(i), path.get(i + 1), true).distanceKm;
            return sum;
        }

        double timeMin(Graph g) {
            double sum = 0;
            for (int i = 0; i + 1 < path.size(); i++) sum += bestEdge(g, path.get(i), path.get(i + 1), false).timeMin();
            return sum;
        }

        private static Edge bestEdge(Graph g, int from, int to, boolean byDistance) {
            Edge best = null;
            for (Edge e : g.adj.get(from)) {
                if (e.to != to) continue;
                if (best == null || (byDistance ? e.distanceKm < best.distanceKm : e.timeMin() < best.timeMin())) best = e;
            }
            return best;
        }
    }

    // ======================= ALGORITHMS =======================

    interface PathFinder {
        String name();
        Route findPath(Graph g, int source, int target, CostMode mode);
    }

    /** Priority queue entry: node with its priority (distance for Dijkstra, f = g + h for A*). */
    static class State implements Comparable<State> {
        final int node;
        final double priority;

        State(int node, double priority) {
            this.node = node;
            this.priority = priority;
        }

        public int compareTo(State o) { return Double.compare(priority, o.priority); }
    }

    /** Dijkstra with a binary heap and lazy deletion. O((V + E) log V). */
    static class Dijkstra implements PathFinder {
        public String name() { return "Dijkstra"; }

        public Route findPath(Graph g, int source, int target, CostMode mode) {
            long start = System.nanoTime();
            int n = g.size();
            double[] dist = new double[n];
            int[] prev = new int[n];
            boolean[] done = new boolean[n];
            Arrays.fill(dist, Double.POSITIVE_INFINITY);
            Arrays.fill(prev, -1);

            PriorityQueue<State> pq = new PriorityQueue<>();
            dist[source] = 0;
            pq.add(new State(source, 0));
            int explored = 0;

            while (!pq.isEmpty()) {
                int u = pq.poll().node;
                if (done[u]) continue;                 // stale entry
                done[u] = true;
                explored++;
                if (u == target) return new Route(true, buildPath(prev, target), dist[target], explored, System.nanoTime() - start);

                for (Edge e : g.adj.get(u)) {
                    int v = e.to;
                    if (done[v] || g.isBlocked(u, v)) continue;
                    double nd = dist[u] + e.cost(mode);
                    if (nd < dist[v]) {
                        dist[v] = nd;
                        prev[v] = u;
                        pq.add(new State(v, nd));
                    }
                }
            }
            return new Route(false, Collections.emptyList(), Double.POSITIVE_INFINITY, explored, System.nanoTime() - start);
        }
    }

    /**
     * A* search: priority = g + h, h = straight-line (Haversine) estimate to the target.
     * TIME mode uses h = km / fastest speed in graph, so h never overestimates.
     */
    static class AStar implements PathFinder {
        public String name() { return "A*"; }

        public Route findPath(Graph g, int source, int target, CostMode mode) {
            long start = System.nanoTime();
            int n = g.size();
            double[] cost = new double[n];
            int[] prev = new int[n];
            boolean[] closed = new boolean[n];
            Arrays.fill(cost, Double.POSITIVE_INFINITY);
            Arrays.fill(prev, -1);

            Node targetNode = g.nodes.get(target);
            PriorityQueue<State> open = new PriorityQueue<>();
            cost[source] = 0;
            open.add(new State(source, heuristic(g, g.nodes.get(source), targetNode, mode)));
            int explored = 0;

            while (!open.isEmpty()) {
                int u = open.poll().node;
                if (closed[u]) continue;
                closed[u] = true;
                explored++;
                if (u == target) return new Route(true, buildPath(prev, target), cost[target], explored, System.nanoTime() - start);

                for (Edge e : g.adj.get(u)) {
                    int v = e.to;
                    if (closed[v] || g.isBlocked(u, v)) continue;
                    double ng = cost[u] + e.cost(mode);
                    if (ng < cost[v]) {
                        cost[v] = ng;
                        prev[v] = u;
                        open.add(new State(v, ng + heuristic(g, g.nodes.get(v), targetNode, mode)));
                    }
                }
            }
            return new Route(false, Collections.emptyList(), Double.POSITIVE_INFINITY, explored, System.nanoTime() - start);
        }

        private static double heuristic(Graph g, Node from, Node to, CostMode mode) {
            double km = haversine(from, to);
            return mode == CostMode.DISTANCE ? km : km / g.maxSpeed * 60.0;
        }
    }

    static List<Integer> buildPath(int[] prev, int target) {
        List<Integer> path = new ArrayList<>();
        for (int at = target; at != -1; at = prev[at]) path.add(at);
        Collections.reverse(path);
        return path;
    }

    // ======================= UTILITIES =======================

    /** Great-circle distance in km. */
    static double haversine(Node a, Node b) {
        double dLat = Math.toRadians(b.lat - a.lat);
        double dLon = Math.toRadians(b.lon - a.lon);
        double h = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                + Math.cos(Math.toRadians(a.lat)) * Math.cos(Math.toRadians(b.lat))
                * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        return 2 * 6371.0088 * Math.asin(Math.min(1.0, Math.sqrt(h)));
    }

    /** Small sample map with made-up places. */
    static Graph buildSampleGraph() {
        Graph g = new Graph();
        g.addNode("Station", 20.000, 78.000);
        g.addNode("Market", 20.010, 78.005);
        g.addNode("College", 20.020, 78.010);
        g.addNode("Hospital", 20.015, 78.025);
        g.addNode("Airport", 20.050, 78.040);
        g.addNode("Mall", 20.030, 78.020);
        g.addNode("Park", 20.005, 78.020);
        g.addNode("Stadium", 20.040, 78.015);
        g.addNode("Library", 20.025, 78.000);
        g.addNode("TechPark", 20.045, 78.030);

        //            from        to          km    km/h
        g.addRoad("Station",  "Market",   1.4, 30);
        g.addRoad("Station",  "Park",     2.5, 40);
        g.addRoad("Station",  "Library",  3.2, 40);
        g.addRoad("Market",   "College",  1.5, 30);
        g.addRoad("Market",   "Park",     2.0, 30);
        g.addRoad("College",  "Mall",     1.8, 40);
        g.addRoad("College",  "Hospital", 2.0, 30);
        g.addRoad("College",  "Library",  1.5, 30);
        g.addRoad("Park",     "Hospital", 1.5, 40);
        g.addRoad("Park",     "Mall",     3.0, 50);
        g.addRoad("Mall",     "Hospital", 2.0, 40);
        g.addRoad("Mall",     "Stadium",  1.5, 40);
        g.addRoad("Mall",     "TechPark", 2.3, 50);
        g.addRoad("Stadium",  "Library",  2.6, 30);
        g.addRoad("Stadium",  "TechPark", 2.0, 40);
        g.addRoad("TechPark", "Airport",  1.4, 80);
        g.addRoad("Stadium",  "Airport",  3.3, 60);
        g.addRoad("Hospital", "TechPark", 3.8, 60);
        return g;
    }

    /** Random rows x cols grid road network, used for benchmarking. */
    static Graph buildGridGraph(int rows, int cols, long seed) {
        Random rnd = new Random(seed);
        double[] speeds = {30, 40, 60, 80};
        Graph g = new Graph();
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
                g.addNode("n" + r + "_" + c, 20.0 + r * 0.002, 78.0 + c * 0.002);
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int id = r * cols + c;
                if (c + 1 < cols) addRandomRoad(g, id, id + 1, rnd, speeds);
                if (r + 1 < rows) addRandomRoad(g, id, id + cols, rnd, speeds);
            }
        }
        return g;
    }

    private static void addRandomRoad(Graph g, int a, int b, Random rnd, double[] speeds) {
        double straight = haversine(g.nodes.get(a), g.nodes.get(b));
        g.addRoad(a, b, straight * (1.0 + rnd.nextDouble() * 0.5), speeds[rnd.nextInt(speeds.length)]);
    }

    // ======================= BENCHMARK =======================

    static void runBenchmark(int rows, int cols, int queries, long seed) {
        Graph g = buildGridGraph(rows, cols, seed);
        System.out.printf("Grid graph: %d nodes, %d directed edges, %d random queries%n", g.size(), g.edgeCount, queries);
        PathFinder dijkstra = new Dijkstra();
        PathFinder astar = new AStar();

        for (CostMode mode : CostMode.values()) {
            Random warm = new Random(seed + 7);
            for (int i = 0; i < 20; i++) {                       // warm-up for fair timing
                int s = warm.nextInt(g.size()), t = warm.nextInt(g.size());
                dijkstra.findPath(g, s, t, mode);
                astar.findPath(g, s, t, mode);
            }

            Random rnd = new Random(seed + 1);
            long dNodes = 0, aNodes = 0, dNanos = 0, aNanos = 0;
            int mismatches = 0;
            for (int i = 0; i < queries; i++) {
                int s = rnd.nextInt(g.size()), t = rnd.nextInt(g.size());
                Route d = dijkstra.findPath(g, s, t, mode);
                Route a = astar.findPath(g, s, t, mode);
                dNodes += d.explored;  aNodes += a.explored;
                dNanos += d.nanos;     aNanos += a.nanos;
                if (Math.abs(d.totalCost - a.totalCost) > 1e-6) mismatches++;
            }

            System.out.println("\nMode: " + mode);
            System.out.printf("  %-9s avg nodes explored: %10.1f   avg time: %8.3f ms%n", dijkstra.name(), dNodes / (double) queries, dNanos / 1e6 / queries);
            System.out.printf("  %-9s avg nodes explored: %10.1f   avg time: %8.3f ms%n", astar.name(), aNodes / (double) queries, aNanos / 1e6 / queries);
            if (dNodes > 0) System.out.printf("  A* explored %.1f%% fewer nodes%n", 100.0 * (dNodes - aNodes) / dNodes);
            System.out.println("  Cost mismatches (should be 0): " + mismatches);
        }
    }

    // ======================= CONSOLE MENU =======================

    public static void main(String[] args) {
        Graph graph = buildSampleGraph();
        PathFinder dijkstra = new Dijkstra();
        PathFinder astar = new AStar();
        Scanner in = new Scanner(System.in);
        System.out.printf("Loaded %d places and %d road segments.%n", graph.size(), graph.edgeCount);

        while (true) {
            System.out.println("\n==== Route Planner ====");
            System.out.println("1. List places");
            System.out.println("2. Find route");
            System.out.println("3. Compare Dijkstra vs A*");
            System.out.println("4. Block a road");
            System.out.println("5. Unblock a road");
            System.out.println("6. Run benchmark on a generated grid");
            System.out.println("0. Exit");
            System.out.print("Choice: ");
            if (!in.hasNextLine()) return;

            String choice = in.nextLine().trim();
            switch (choice) {
                case "1":
                    for (Node n : graph.nodes) System.out.printf("  %-10s (%.3f, %.3f)%n", n.name, n.lat, n.lon);
                    break;
                case "2": {
                    int s = askPlace(graph, in, "From: ");
                    int t = askPlace(graph, in, "To: ");
                    if (s < 0 || t < 0) break;
                    CostMode mode = askMode(in);
                    System.out.print("Algorithm (1 = Dijkstra, 2 = A*): ");
                    PathFinder pf = in.nextLine().trim().equals("2") ? astar : dijkstra;
                    printRoute(graph, pf, pf.findPath(graph, s, t, mode), mode);
                    break;
                }
                case "3": {
                    int s = askPlace(graph, in, "From: ");
                    int t = askPlace(graph, in, "To: ");
                    if (s < 0 || t < 0) break;
                    CostMode mode = askMode(in);
                    printRoute(graph, dijkstra, dijkstra.findPath(graph, s, t, mode), mode);
                    printRoute(graph, astar, astar.findPath(graph, s, t, mode), mode);
                    break;
                }
                case "4":
                case "5": {
                    boolean block = choice.equals("4");
                    int a = askPlace(graph, in, "Road from: ");
                    int b = askPlace(graph, in, "Road to: ");
                    if (a < 0 || b < 0) break;
                    if (block) graph.blockRoad(a, b); else graph.unblockRoad(a, b);
                    System.out.println("Road between " + graph.nodes.get(a).name + " and " + graph.nodes.get(b).name
                            + (block ? " is now blocked." : " is open again."));
                    break;
                }
                case "6":
                    runBenchmark(150, 150, 200, 42L);
                    break;
                case "0":
                    return;
                default:
                    System.out.println("Invalid choice.");
            }
        }
    }

    private static int askPlace(Graph g, Scanner in, String prompt) {
        System.out.print(prompt);
        String name = in.nextLine().trim();
        int id = g.findId(name);
        if (id < 0) System.out.println("Unknown place: " + name);
        return id;
    }

    private static CostMode askMode(Scanner in) {
        System.out.print("Optimise for (1 = distance, 2 = time): ");
        return in.nextLine().trim().equals("2") ? CostMode.TIME : CostMode.DISTANCE;
    }

    private static void printRoute(Graph g, PathFinder pf, Route r, CostMode mode) {
        System.out.println("\n[" + pf.name() + "]");
        if (!r.found) {
            System.out.printf("  No route found (explored %d nodes).%n", r.explored);
            return;
        }
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < r.path.size(); i++) {
            if (i > 0) sb.append(" -> ");
            sb.append(g.nodes.get(r.path.get(i)).name);
        }
        System.out.println("  Path     : " + sb);
        System.out.printf("  Distance : %.2f km%n", r.distanceKm(g));
        System.out.printf("  Time     : %.1f min%n", r.timeMin(g));
        System.out.printf("  Explored : %d nodes (optimised for %s)%n", r.explored, mode);
        System.out.printf("  Runtime  : %.3f ms%n", r.nanos / 1e6);
    }
}