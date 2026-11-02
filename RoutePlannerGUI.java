import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.function.IntConsumer;

/**
 * Swing front end for the Route Planner (pure Java, no other language or markup).
 * Keep this file in the SAME folder as RoutePlanner.java (the backend).
 * It extends RoutePlanner only so Graph, Dijkstra, AStar, etc. can be used without a prefix.
 *
 *   javac RoutePlanner.java RoutePlannerGUI.java
 *   java RoutePlannerGUI
 */
public class RoutePlannerGUI extends RoutePlanner {

    // ----- colour palette -----
    private static final Color INK = new Color(15, 23, 42);
    private static final Color MUTED = new Color(100, 116, 139);
    private static final Color LINE = new Color(226, 232, 240);
    private static final Color BLUE = new Color(37, 99, 235);
    private static final Color GREEN = new Color(22, 163, 74);
    private static final Color RED = new Color(220, 38, 38);

    private final Graph graph = buildSampleGraph();
    private final PathFinder dijkstra = new Dijkstra();
    private final PathFinder astar = new AStar();

    private MapPanel mapPanel;
    private JComboBox<String> fromBox, toBox, modeBox, algoBox;
    private ScrollPanel resultView;
    private JScrollPane resultScroll;
    private boolean nextClickSetsFrom = true;

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new RoutePlannerGUI().createAndShow());
    }

    // ======================= WINDOW =======================

    private void createAndShow() {
        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (Exception ignored) { }

        String[] names = new String[graph.size()];
        for (int i = 0; i < names.length; i++) names[i] = graph.nodes.get(i).name;

        fromBox = styledCombo(names);
        toBox = styledCombo(names);
        toBox.setSelectedIndex(names.length - 1);
        modeBox = styledCombo(new String[]{"Shortest distance", "Fastest time"});
        algoBox = styledCombo(new String[]{"Dijkstra", "A*", "Compare both"});

        mapPanel = new MapPanel(graph);
        mapPanel.setEndpoints(fromBox.getSelectedIndex(), toBox.getSelectedIndex());
        mapPanel.setOnNodeClick(id -> {
            if (nextClickSetsFrom) fromBox.setSelectedIndex(id); else toBox.setSelectedIndex(id);
            nextClickSetsFrom = !nextClickSetsFrom;
        });
        fromBox.addActionListener(e -> endpointsChanged());
        toBox.addActionListener(e -> endpointsChanged());

        StyledButton findBtn = new StyledButton("Find route", StyledButton.Kind.PRIMARY);
        StyledButton swapBtn = new StyledButton("Swap", StyledButton.Kind.SECONDARY);
        StyledButton blockBtn = new StyledButton("Block road", StyledButton.Kind.DANGER);
        StyledButton unblockBtn = new StyledButton("Unblock road", StyledButton.Kind.SECONDARY);
        StyledButton benchBtn = new StyledButton("Run benchmark", StyledButton.Kind.SECONDARY);
        findBtn.addActionListener(e -> findRoute());
        swapBtn.addActionListener(e -> swapEndpoints());
        blockBtn.addActionListener(e -> changeBlock(true));
        unblockBtn.addActionListener(e -> changeBlock(false));
        benchBtn.addActionListener(e -> runBenchmarkAsync(benchBtn));

        // ----- results area (plain Swing components) -----
        resultView = new ScrollPanel();
        resultView.setLayout(new BoxLayout(resultView, BoxLayout.Y_AXIS));
        resultView.setBackground(Color.WHITE);
        resultView.setBorder(BorderFactory.createEmptyBorder(12, 14, 12, 14));
        resultScroll = new JScrollPane(resultView);
        resultScroll.setBorder(BorderFactory.createMatteBorder(1, 0, 0, 0, LINE));
        resultScroll.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
        resultScroll.getVerticalScrollBar().setUnitIncrement(16);
        showWelcome();

        // ----- controls -----
        JPanel controls = new JPanel();
        controls.setOpaque(false);
        controls.setLayout(new BoxLayout(controls, BoxLayout.Y_AXIS));
        controls.setBorder(BorderFactory.createEmptyBorder(16, 16, 6, 16));
        controls.add(sectionLabel("PLAN YOUR TRIP"));
        controls.add(Box.createVerticalStrut(8));
        controls.add(twoColumns(labeled("From", fromBox), labeled("To", toBox)));
        controls.add(twoColumns(labeled("Optimise for", modeBox), labeled("Algorithm", algoBox)));
        controls.add(Box.createVerticalStrut(4));
        controls.add(fullWidth(findBtn));
        controls.add(Box.createVerticalStrut(8));

        JPanel row = new JPanel(new GridLayout(1, 3, 8, 0));
        row.setOpaque(false);
        row.add(swapBtn);
        row.add(blockBtn);
        row.add(unblockBtn);
        controls.add(fullWidth(row));
        controls.add(Box.createVerticalStrut(8));
        controls.add(fullWidth(benchBtn));

        JPanel resultHeader = new JPanel(new BorderLayout());
        resultHeader.setOpaque(false);
        resultHeader.setBorder(BorderFactory.createEmptyBorder(8, 16, 6, 16));
        resultHeader.add(sectionLabel("RESULTS"), BorderLayout.WEST);

        JPanel resultArea = new JPanel(new BorderLayout());
        resultArea.setOpaque(false);
        resultArea.add(resultHeader, BorderLayout.NORTH);
        resultArea.add(resultScroll, BorderLayout.CENTER);

        JPanel side = new JPanel(new BorderLayout());
        side.setBackground(Color.WHITE);
        side.setBorder(BorderFactory.createMatteBorder(0, 1, 0, 0, LINE));
        side.setPreferredSize(new Dimension(380, 640));
        side.add(controls, BorderLayout.NORTH);
        side.add(resultArea, BorderLayout.CENTER);

        JFrame frame = new JFrame("Route Planner - Dijkstra and A*");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(buildHeader(), BorderLayout.NORTH);
        frame.add(mapPanel, BorderLayout.CENTER);
        frame.add(side, BorderLayout.EAST);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }

    private JPanel buildHeader() {
        JPanel header = new JPanel(new BorderLayout()) {
            @Override
            protected void paintComponent(Graphics g) {
                Graphics2D g2 = (Graphics2D) g.create();
                g2.setPaint(new GradientPaint(0, 0, new Color(15, 23, 42), getWidth(), 0, new Color(30, 64, 175)));
                g2.fillRect(0, 0, getWidth(), getHeight());
                g2.dispose();
            }
        };
        header.setBorder(BorderFactory.createEmptyBorder(14, 22, 14, 22));

        JLabel title = new JLabel("Route Planner");
        title.setFont(new Font("SansSerif", Font.BOLD, 22));
        title.setForeground(Color.WHITE);
        JLabel subtitle = new JLabel("Shortest paths with Dijkstra and A*");
        subtitle.setFont(new Font("SansSerif", Font.PLAIN, 13));
        subtitle.setForeground(new Color(191, 219, 254));

        JPanel left = new JPanel();
        left.setOpaque(false);
        left.setLayout(new BoxLayout(left, BoxLayout.Y_AXIS));
        left.add(title);
        left.add(Box.createVerticalStrut(2));
        left.add(subtitle);

        JLabel hint = new JLabel("Tip: click two places on the map to set From and To");
        hint.setFont(new Font("SansSerif", Font.PLAIN, 12));
        hint.setForeground(new Color(147, 197, 253));

        header.add(left, BorderLayout.WEST);
        header.add(hint, BorderLayout.EAST);
        return header;
    }

    private static JComboBox<String> styledCombo(String[] items) {
        JComboBox<String> box = new JComboBox<>(items);
        box.setFont(new Font("SansSerif", Font.PLAIN, 13));
        box.setBackground(Color.WHITE);
        box.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        return box;
    }

    private static JLabel sectionLabel(String text) {
        JLabel l = new JLabel(text);
        l.setFont(new Font("SansSerif", Font.BOLD, 11));
        l.setForeground(MUTED);
        l.setAlignmentX(Component.LEFT_ALIGNMENT);
        return l;
    }

    private static JPanel labeled(String text, JComponent c) {
        JPanel p = new JPanel(new BorderLayout(0, 3));
        p.setOpaque(false);
        JLabel l = new JLabel(text);
        l.setFont(new Font("SansSerif", Font.PLAIN, 12));
        l.setForeground(new Color(71, 85, 105));
        p.add(l, BorderLayout.NORTH);
        p.add(c, BorderLayout.CENTER);
        p.setBorder(BorderFactory.createEmptyBorder(0, 0, 9, 0));
        p.setAlignmentX(Component.LEFT_ALIGNMENT);
        p.setMaximumSize(new Dimension(Integer.MAX_VALUE, p.getPreferredSize().height));
        return p;
    }

    private static JPanel twoColumns(JComponent left, JComponent right) {
        JPanel p = new JPanel(new GridLayout(1, 2, 10, 0));
        p.setOpaque(false);
        p.add(left);
        p.add(right);
        p.setAlignmentX(Component.LEFT_ALIGNMENT);
        p.setMaximumSize(new Dimension(Integer.MAX_VALUE, p.getPreferredSize().height));
        return p;
    }

    private static JComponent fullWidth(JComponent c) {
        c.setMaximumSize(new Dimension(Integer.MAX_VALUE, c.getPreferredSize().height));
        c.setAlignmentX(Component.LEFT_ALIGNMENT);
        return c;
    }

    // ======================= ACTIONS =======================

    private void endpointsChanged() {
        mapPanel.setEndpoints(fromBox.getSelectedIndex(), toBox.getSelectedIndex());
    }

    private void swapEndpoints() {
        int a = fromBox.getSelectedIndex();
        int b = toBox.getSelectedIndex();
        fromBox.setSelectedIndex(b);
        toBox.setSelectedIndex(a);
    }

    private CostMode selectedMode() {
        return modeBox.getSelectedIndex() == 0 ? CostMode.DISTANCE : CostMode.TIME;
    }

    private void findRoute() {
        int s = fromBox.getSelectedIndex();
        int t = toBox.getSelectedIndex();
        if (s == t) {
            show(callout("Choose two different places", new Color(0xFEF3C7), new Color(0x92400E),
                    "From and To are the same place."));
            return;
        }
        CostMode mode = selectedMode();
        String modeName = mode == CostMode.DISTANCE ? "Shortest distance" : "Fastest time";
        int algo = algoBox.getSelectedIndex();      // 0 = Dijkstra, 1 = A*, 2 = both

        Route d = algo != 1 ? dijkstra.findPath(graph, s, t, mode) : null;
        Route a = algo != 0 ? astar.findPath(graph, s, t, mode) : null;
        Route shown = a != null ? a : d;
        mapPanel.setRoute(shown.found ? shown.path : new ArrayList<Integer>());

        if (!shown.found) {
            show(titleBlock(s, t, modeName),
                    callout("No route found", new Color(0xFEE2E2), new Color(0x991B1B),
                            "These places are not connected.",
                            graph.blocked.isEmpty() ? " " : "A blocked road may be the cause."));
            return;
        }

        List<JComponent> parts = new ArrayList<>();
        if (algo == 2) {
            parts.add(titleBlock(s, t, modeName + " \u00b7 comparing both algorithms"));
            parts.add(compareTable(d, a));
            if (d.explored > 0 && a.explored < d.explored) {
                parts.add(callout("A* wins on effort", new Color(0xDCFCE7), new Color(0x166534),
                        String.format("It explored %.0f%% fewer nodes, same cost.",
                                100.0 * (d.explored - a.explored) / d.explored)));
            } else {
                parts.add(callout("Same effort", new Color(0xF1F5F9), new Color(0x334155),
                        "Both explored the same number of nodes."));
            }
            parts.add(smallLabel("ROUTE SHOWN ON MAP (A*)"));
        } else {
            parts.add(titleBlock(s, t, modeName + " \u00b7 " + (algo == 0 ? "Dijkstra" : "A*")));
            Block tiles = new Block(new GridLayout(1, 3, 8, 0));
            tiles.add(tile("DISTANCE", String.format("%.1f km", shown.distanceKm(graph)), new Color(0xEFF6FF), new Color(0x1D4ED8)));
            tiles.add(tile("TIME", String.format("%.0f min", shown.timeMin(graph)), new Color(0xF0FDF4), new Color(0x15803D)));
            tiles.add(tile("EXPLORED", shown.explored + " nodes", new Color(0xFFF7ED), new Color(0xC2410C)));
            parts.add(tiles);
            parts.add(text(String.format("Search took %.3f ms", shown.nanos / 1e6), Font.PLAIN, 12, MUTED));
            parts.add(smallLabel("ROUTE"));
        }
        parts.add(stepsPanel(shown, mode));
        show(parts.toArray(new JComponent[0]));
    }

    /** Blocks or unblocks the direct road between the selected From and To places. */
    private void changeBlock(boolean block) {
        int a = fromBox.getSelectedIndex();
        int b = toBox.getSelectedIndex();
        String an = graph.nodes.get(a).name;
        String bn = graph.nodes.get(b).name;
        boolean direct = false;
        for (Edge e : graph.adj.get(a)) {
            if (e.to == b) direct = true;
        }
        if (a == b || !direct) {
            show(callout("No direct road", new Color(0xFEF3C7), new Color(0x92400E),
                    "There is no direct road between",
                    an + " and " + bn + ".",
                    "Pick two places joined by a line."));
            return;
        }
        if (block) graph.blockRoad(a, b); else graph.unblockRoad(a, b);
        mapPanel.setRoute(new ArrayList<Integer>());
        show(callout(block ? "Road blocked" : "Road reopened",
                block ? new Color(0xFEE2E2) : new Color(0xDCFCE7),
                block ? new Color(0x991B1B) : new Color(0x166534),
                an + " \u2194 " + bn + " is now " + (block ? "closed." : "open."),
                "Press Find route to plan again."));
    }

    private void runBenchmarkAsync(JButton button) {
        button.setEnabled(false);
        show(callout("Running benchmark", new Color(0xEFF6FF), new Color(0x1E40AF),
                "Comparing both algorithms on a", "100 x 100 grid. Please wait..."));
        new SwingWorker<BenchData, Void>() {
            @Override
            protected BenchData doInBackground() {
                return runBench(100, 100, 100, 42L);
            }

            @Override
            protected void done() {
                try {
                    showBench(get());
                } catch (Exception ex) {
                    show(callout("Benchmark failed", new Color(0xFEE2E2), new Color(0x991B1B),
                            String.valueOf(ex.getMessage())));
                }
                button.setEnabled(true);
            }
        }.execute();
    }

    // ======================= RESULT PANEL (plain Swing) =======================

    /** Panel that is as wide as the scroll pane's viewport, so content never needs a horizontal scrollbar. */
    static class ScrollPanel extends JPanel implements Scrollable {
        public Dimension getPreferredScrollableViewportSize() { return getPreferredSize(); }
        public int getScrollableUnitIncrement(Rectangle r, int o, int d) { return 16; }
        public int getScrollableBlockIncrement(Rectangle r, int o, int d) { return 64; }
        public boolean getScrollableTracksViewportWidth() { return true; }
        public boolean getScrollableTracksViewportHeight() { return false; }
    }

    /** Transparent panel that is only as tall as its content (so BoxLayout does not stretch it). */
    static class Block extends JPanel {
        Block(LayoutManager layout) {
            super(layout);
            setOpaque(false);
            setAlignmentX(Component.LEFT_ALIGNMENT);
        }

        @Override
        public Dimension getMaximumSize() {
            return new Dimension(Integer.MAX_VALUE, getPreferredSize().height);
        }
    }

    /** Rounded coloured card that stacks its children vertically. */
    static class Card extends Block {
        private final Color bg;

        Card(Color bg, int padV, int padH) {
            super(null);
            this.bg = bg;
            setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
            setBorder(BorderFactory.createEmptyBorder(padV, padH, padV, padH));
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = (Graphics2D) g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g2.setColor(bg);
            g2.fillRoundRect(0, 0, getWidth(), getHeight(), 14, 14);
            g2.dispose();
            super.paintComponent(g);
        }
    }

    /** Vertical list of stops with a dot for each place and distance / time between them. */
    static class StepsPanel extends Block {
        private static final int ROW = 26, LEG = 24;
        private final List<String> names;
        private final List<String> legs;

        StepsPanel(List<String> names, List<String> legs) {
            super(null);
            this.names = names;
            this.legs = legs;
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(260, names.size() * ROW + (names.size() - 1) * LEG + 6);
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = (Graphics2D) g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
            int last = names.size() - 1;
            int y = 3;
            for (int i = 0; i <= last; i++) {
                int cy = y + ROW / 2;
                g2.setColor(i == 0 ? GREEN : i == last ? RED : BLUE);
                g2.fillOval(3, cy - 7, 14, 14);
                g2.setColor(INK);
                g2.setFont(new Font("SansSerif", Font.BOLD, 14));
                g2.drawString(names.get(i), 30, cy + 5);
                if (i < last) {
                    g2.setColor(new Color(0xCBD5E1));
                    g2.setStroke(new BasicStroke(2f));
                    g2.drawLine(10, cy + 9, 10, cy + ROW + LEG - 9);
                    g2.setColor(MUTED);
                    g2.setFont(new Font("SansSerif", Font.PLAIN, 12));
                    g2.drawString(legs.get(i), 30, cy + ROW / 2 + LEG / 2 + 4);
                }
                y += ROW + LEG;
            }
            g2.dispose();
        }
    }

    /** One horizontal bar: name, a bar scaled to 0..1, and a value on the right. */
    static class BarRow extends Block {
        private final String name, value;
        private final double fraction;
        private final Color color;

        BarRow(String name, String value, double fraction, Color color) {
            super(null);
            this.name = name;
            this.value = value;
            this.fraction = Math.max(0.02, Math.min(1.0, fraction));
            this.color = color;
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(260, 26);
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = (Graphics2D) g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
            int w = getWidth(), h = getHeight();
            int nameW = 64, valueW = 112;
            int barX = nameW, barMax = Math.max(10, w - nameW - valueW - 6);

            g2.setFont(new Font("SansSerif", Font.BOLD, 12));
            g2.setColor(INK);
            g2.drawString(name, 2, h / 2 + 4);

            g2.setColor(new Color(0xF1F5F9));
            g2.fillRoundRect(barX, h / 2 - 7, barMax, 14, 8, 8);
            g2.setColor(color);
            g2.fillRoundRect(barX, h / 2 - 7, (int) (barMax * fraction), 14, 8, 8);

            g2.setFont(new Font("SansSerif", Font.PLAIN, 11));
            g2.setColor(MUTED);
            FontMetrics fm = g2.getFontMetrics();
            g2.drawString(value, w - fm.stringWidth(value) - 2, h / 2 + 4);
            g2.dispose();
        }
    }

    private static JLabel text(String s, int style, int size, Color color) {
        JLabel l = new JLabel(s);
        l.setFont(new Font("SansSerif", style, size));
        l.setForeground(color);
        l.setAlignmentX(Component.LEFT_ALIGNMENT);
        return l;
    }

    private static JLabel smallLabel(String s) {
        return text(s, Font.BOLD, 11, MUTED);
    }

    private static Block column() {
        Block b = new Block(null);
        b.setLayout(new BoxLayout(b, BoxLayout.Y_AXIS));
        return b;
    }

    private static Card callout(String title, Color bg, Color fg, String... lines) {
        Card c = new Card(bg, 10, 12);
        c.add(text(title, Font.BOLD, 13, fg));
        for (String line : lines) c.add(text(line, Font.PLAIN, 12, fg));
        return c;
    }

    private static Card tile(String label, String value, Color bg, Color fg) {
        Card c = new Card(bg, 9, 6);
        JLabel a = text(label, Font.PLAIN, 11, MUTED);
        JLabel b = text(value, Font.BOLD, 16, fg);
        a.setAlignmentX(Component.CENTER_ALIGNMENT);
        b.setAlignmentX(Component.CENTER_ALIGNMENT);
        c.add(a);
        c.add(b);
        return c;
    }

    private Block titleBlock(int s, int t, String sub) {
        Block b = column();
        b.add(text(graph.nodes.get(s).name + "  \u2192  " + graph.nodes.get(t).name, Font.BOLD, 19, INK));
        b.add(text(sub, Font.PLAIN, 12, MUTED));
        return b;
    }

    /** Replaces everything in the results area with the given parts. */
    private void show(JComponent... parts) {
        resultView.removeAll();
        for (JComponent c : parts) {
            c.setAlignmentX(Component.LEFT_ALIGNMENT);
            resultView.add(c);
            resultView.add(Box.createVerticalStrut(10));
        }
        resultView.revalidate();
        resultView.repaint();
        SwingUtilities.invokeLater(() -> resultScroll.getVerticalScrollBar().setValue(0));
    }

    private void showWelcome() {
        Block steps = column();
        String[][] items = {
                {"1", "Click a place on the map to set From,", "then another place to set To."},
                {"2", "Choose distance or time, and the algorithm."},
                {"3", "Press Find route to see path and details."},
                {"4", "Use Block road to close a road and detour."}
        };
        for (String[] item : items) {
            Card c = new Card(item[0].equals("1") || item[0].equals("3") ? new Color(0xEFF6FF) : new Color(0xF8FAFC), 8, 12);
            c.add(text("Step " + item[0], Font.BOLD, 11, BLUE));
            for (int i = 1; i < item.length; i++) c.add(text(item[i], Font.PLAIN, 12, INK));
            steps.add(c);
            steps.add(Box.createVerticalStrut(6));
        }
        Block head = column();
        head.add(text("Welcome", Font.BOLD, 19, INK));
        head.add(text("Find the best route between places", Font.PLAIN, 12, MUTED));
        show(head, steps);
    }

    private Edge legEdge(int a, int b, CostMode mode) {
        Edge best = null;
        for (Edge e : graph.adj.get(a)) {
            if (e.to == b && (best == null || e.cost(mode) < best.cost(mode))) best = e;
        }
        return best;
    }

    private StepsPanel stepsPanel(Route r, CostMode mode) {
        List<String> names = new ArrayList<>();
        List<String> legs = new ArrayList<>();
        for (int i = 0; i < r.path.size(); i++) {
            names.add(graph.nodes.get(r.path.get(i)).name);
            if (i + 1 < r.path.size()) {
                Edge e = legEdge(r.path.get(i), r.path.get(i + 1), mode);
                legs.add(String.format("%.1f km \u00b7 %.1f min", e.distanceKm, e.timeMin()));
            }
        }
        return new StepsPanel(names, legs);
    }

    private static JLabel cell(String text, Color bg, Color fg, int style, int align) {
        JLabel l = new JLabel(text, align);
        l.setOpaque(true);
        l.setBackground(bg);
        l.setForeground(fg);
        l.setFont(new Font("SansSerif", style, 12));
        l.setBorder(BorderFactory.createEmptyBorder(7, 8, 7, 8));
        return l;
    }

    private Block compareTable(Route d, Route a) {
        Color good = new Color(0xDCFCE7), plain = new Color(0xF8FAFC), head = new Color(0xF1F5F9);
        int winner = d.explored == a.explored ? 0 : (a.explored < d.explored ? 2 : 1);
        Block t = new Block(new GridLayout(5, 3, 2, 2));
        t.add(cell("Metric", INK, Color.WHITE, Font.BOLD, SwingConstants.LEFT));
        t.add(cell("Dijkstra", INK, Color.WHITE, Font.BOLD, SwingConstants.CENTER));
        t.add(cell("A*", INK, Color.WHITE, Font.BOLD, SwingConstants.CENTER));
        addRow(t, "Distance", String.format("%.2f km", d.distanceKm(graph)), String.format("%.2f km", a.distanceKm(graph)), 0, good, plain, head);
        addRow(t, "Time", String.format("%.1f min", d.timeMin(graph)), String.format("%.1f min", a.timeMin(graph)), 0, good, plain, head);
        addRow(t, "Nodes explored", String.valueOf(d.explored), String.valueOf(a.explored), winner, good, plain, head);
        addRow(t, "Search time", String.format("%.3f ms", d.nanos / 1e6), String.format("%.3f ms", a.nanos / 1e6), 0, good, plain, head);
        return t;
    }

    private static void addRow(Block t, String name, String dv, String av, int winner, Color good, Color plain, Color head) {
        t.add(cell(name, head, INK, Font.BOLD, SwingConstants.LEFT));
        t.add(cell(dv, winner == 1 ? good : plain, INK, Font.PLAIN, SwingConstants.CENTER));
        t.add(cell(av, winner == 2 ? good : plain, INK, Font.PLAIN, SwingConstants.CENTER));
    }

    // ======================= BENCHMARK =======================

    static class BenchRow {
        CostMode mode;
        double dAvg, aAvg, dMs, aMs;
        int mismatches;
    }

    static class BenchData {
        int places, edges, queries;
        final List<BenchRow> rows = new ArrayList<>();
    }

    /** Runs Dijkstra vs A* on random trips of a generated grid (safe to call off the UI thread). */
    private BenchData runBench(int rows, int cols, int queries, long seed) {
        Graph g = buildGridGraph(rows, cols, seed);
        PathFinder d = new Dijkstra();
        PathFinder a = new AStar();
        BenchData data = new BenchData();
        data.places = g.size();
        data.edges = g.edgeCount;
        data.queries = queries;

        for (CostMode mode : CostMode.values()) {
            Random warm = new Random(seed + 7);
            for (int i = 0; i < 20; i++) {
                int s = warm.nextInt(g.size()), t = warm.nextInt(g.size());
                d.findPath(g, s, t, mode);
                a.findPath(g, s, t, mode);
            }
            Random rnd = new Random(seed + 1);
            long dNodes = 0, aNodes = 0, dNanos = 0, aNanos = 0;
            int mismatches = 0;
            for (int i = 0; i < queries; i++) {
                int s = rnd.nextInt(g.size()), t = rnd.nextInt(g.size());
                Route rd = d.findPath(g, s, t, mode);
                Route ra = a.findPath(g, s, t, mode);
                dNodes += rd.explored;
                aNodes += ra.explored;
                dNanos += rd.nanos;
                aNanos += ra.nanos;
                if (Math.abs(rd.totalCost - ra.totalCost) > 1e-6) mismatches++;
            }
            BenchRow row = new BenchRow();
            row.mode = mode;
            row.dAvg = dNodes / (double) queries;
            row.aAvg = aNodes / (double) queries;
            row.dMs = dNanos / 1e6 / queries;
            row.aMs = aNanos / 1e6 / queries;
            row.mismatches = mismatches;
            data.rows.add(row);
        }
        return data;
    }

    private void showBench(BenchData data) {
        List<JComponent> parts = new ArrayList<>();
        Block head = column();
        head.add(text("Benchmark", Font.BOLD, 19, INK));
        head.add(text(String.format("%,d places \u00b7 %,d roads \u00b7 %d random trips",
                data.places, data.edges, data.queries), Font.PLAIN, 12, MUTED));
        parts.add(head);

        for (BenchRow r : data.rows) {
            Block group = column();
            JLabel chip = new JLabel(r.mode == CostMode.DISTANCE ? "  SHORTEST DISTANCE  " : "  FASTEST TIME  ");
            chip.setOpaque(true);
            chip.setBackground(INK);
            chip.setForeground(Color.WHITE);
            chip.setFont(new Font("SansSerif", Font.BOLD, 11));
            chip.setBorder(BorderFactory.createEmptyBorder(4, 2, 4, 2));
            chip.setAlignmentX(Component.LEFT_ALIGNMENT);
            group.add(chip);
            group.add(Box.createVerticalStrut(6));
            group.add(text("Average nodes explored per trip", Font.PLAIN, 12, MUTED));
            group.add(Box.createVerticalStrut(4));
            group.add(new BarRow("Dijkstra", String.format("%,.0f \u00b7 %.2f ms", r.dAvg, r.dMs), 1.0, new Color(0x94A3B8)));
            group.add(new BarRow("A*", String.format("%,.0f \u00b7 %.2f ms", r.aAvg, r.aMs), r.dAvg > 0 ? r.aAvg / r.dAvg : 1.0, BLUE));
            group.add(Box.createVerticalStrut(6));
            boolean ok = r.mismatches == 0;
            group.add(callout(String.format("A* explored %.1f%% fewer nodes", 100.0 * (r.dAvg - r.aAvg) / r.dAvg),
                    ok ? new Color(0xDCFCE7) : new Color(0xFEE2E2), ok ? new Color(0x166534) : new Color(0x991B1B),
                    ok ? "\u2713 Same optimal cost on all " + data.queries + " trips."
                            : "Warning: " + r.mismatches + " trips differed."));
            parts.add(group);
        }
        show(parts.toArray(new JComponent[0]));
    }

    // ======================= STYLED BUTTON =======================

    static class StyledButton extends JButton {
        enum Kind { PRIMARY, SECONDARY, DANGER }

        private final Kind kind;

        StyledButton(String text, Kind kind) {
            super(text);
            this.kind = kind;
            setContentAreaFilled(false);
            setBorderPainted(false);
            setFocusPainted(false);
            setOpaque(false);
            setRolloverEnabled(true);
            setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
            setFont(new Font("SansSerif", Font.BOLD, 13));
            setBorder(BorderFactory.createEmptyBorder(10, 12, 10, 12));
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = (Graphics2D) g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
            boolean hover = getModel().isRollover();
            boolean press = getModel().isArmed();

            Color bg, fg, border;
            if (kind == Kind.PRIMARY) {
                bg = press ? new Color(30, 64, 175) : hover ? new Color(29, 78, 216) : BLUE;
                fg = Color.WHITE;
                border = bg;
            } else if (kind == Kind.DANGER) {
                bg = hover ? new Color(254, 226, 226) : Color.WHITE;
                fg = new Color(185, 28, 28);
                border = new Color(252, 165, 165);
            } else {
                bg = hover ? new Color(241, 245, 249) : Color.WHITE;
                fg = new Color(30, 41, 59);
                border = new Color(203, 213, 225);
            }
            if (!isEnabled()) {
                bg = new Color(226, 232, 240);
                fg = new Color(148, 163, 184);
                border = bg;
            }
            g2.setColor(bg);
            g2.fillRoundRect(0, 0, getWidth() - 1, getHeight() - 1, 12, 12);
            g2.setColor(border);
            g2.drawRoundRect(0, 0, getWidth() - 1, getHeight() - 1, 12, 12);

            g2.setFont(getFont());
            FontMetrics fm = g2.getFontMetrics();
            int tx = (getWidth() - fm.stringWidth(getText())) / 2;
            int ty = (getHeight() - fm.getHeight()) / 2 + fm.getAscent();
            g2.setColor(fg);
            g2.drawString(getText(), tx, ty);
            g2.dispose();
        }
    }

    // ======================= MAP DRAWING =======================

    /** Draws the road network, the selected places, blocked roads and the animated route. */
    static class MapPanel extends JPanel {
        private static final int MARGIN_X = 85, MARGIN_Y = 60;
        private static final Color ROAD = new Color(148, 163, 184);
        private static final Color NODE = new Color(71, 85, 105);

        private final Graph g;
        private final double minLat, maxLat, minLon, maxLon;
        private int from = -1, to = -1, hover = -1;
        private List<Integer> route = new ArrayList<>();
        private double progress = 1.0;
        private javax.swing.Timer anim;
        private IntConsumer onNodeClick = id -> { };

        MapPanel(Graph g) {
            this.g = g;
            double a = Double.MAX_VALUE, b = -Double.MAX_VALUE, c = Double.MAX_VALUE, d = -Double.MAX_VALUE;
            for (Node n : g.nodes) {
                a = Math.min(a, n.lat);
                b = Math.max(b, n.lat);
                c = Math.min(c, n.lon);
                d = Math.max(d, n.lon);
            }
            minLat = a;
            maxLat = b;
            minLon = c;
            maxLon = d;
            setPreferredSize(new Dimension(760, 640));

            addMouseListener(new MouseAdapter() {
                @Override
                public void mousePressed(MouseEvent e) {
                    int id = nodeAt(e.getX(), e.getY());
                    if (id >= 0) onNodeClick.accept(id);
                }

                @Override
                public void mouseExited(MouseEvent e) {
                    setHover(-1);
                }
            });
            addMouseMotionListener(new MouseAdapter() {
                @Override
                public void mouseMoved(MouseEvent e) {
                    setHover(nodeAt(e.getX(), e.getY()));
                }
            });
        }

        private void setHover(int id) {
            if (id == hover) return;
            hover = id;
            setCursor(Cursor.getPredefinedCursor(id >= 0 ? Cursor.HAND_CURSOR : Cursor.DEFAULT_CURSOR));
            repaint();
        }

        void setOnNodeClick(IntConsumer handler) { this.onNodeClick = handler; }

        void setEndpoints(int from, int to) {
            this.from = from;
            this.to = to;
            stopAnimation();
            this.route = new ArrayList<>();      // old route no longer matches
            this.progress = 1.0;
            repaint();
        }

        /** Sets the route and animates it being drawn from start to end. */
        void setRoute(List<Integer> path) {
            stopAnimation();
            this.route = path;
            if (path.size() < 2) {
                progress = 1.0;
                repaint();
                return;
            }
            progress = 0;
            anim = new javax.swing.Timer(16, e -> {
                progress += 0.04;
                if (progress >= 1.0) {
                    progress = 1.0;
                    stopAnimation();
                }
                repaint();
            });
            anim.start();
        }

        private void stopAnimation() {
            if (anim != null) {
                anim.stop();
                anim = null;
            }
        }

        private Point2D.Double pos(int id) {
            Node n = g.nodes.get(id);
            double w = Math.max(1, getWidth() - 2.0 * MARGIN_X);
            double h = Math.max(1, getHeight() - 2.0 * MARGIN_Y);
            double fx = maxLon == minLon ? 0.5 : (n.lon - minLon) / (maxLon - minLon);
            double fy = maxLat == minLat ? 0.5 : (n.lat - minLat) / (maxLat - minLat);
            return new Point2D.Double(MARGIN_X + fx * w, MARGIN_Y + (1 - fy) * h);   // north is up
        }

        private int nodeAt(int x, int y) {
            int best = -1;
            double bestDist = 16;
            for (int i = 0; i < g.size(); i++) {
                double d = pos(i).distance(x, y);
                if (d < bestDist) {
                    bestDist = d;
                    best = i;
                }
            }
            return best;
        }

        @Override
        protected void paintComponent(Graphics graphics) {
            Graphics2D g2 = (Graphics2D) graphics.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

            drawBackground(g2);
            drawRoads(g2);
            drawRoute(g2);
            drawRoadLabels(g2);
            drawNodes(g2);
            drawLegend(g2);
            g2.dispose();
        }

        private void drawBackground(Graphics2D g2) {
            g2.setPaint(new GradientPaint(0, 0, new Color(248, 250, 252), 0, getHeight(), new Color(226, 232, 240)));
            g2.fillRect(0, 0, getWidth(), getHeight());
            g2.setColor(new Color(148, 163, 184, 70));
            for (int x = 14; x < getWidth(); x += 28) {
                for (int y = 14; y < getHeight(); y += 28) {
                    g2.fillOval(x, y, 2, 2);
                }
            }
        }

        private void drawRoads(Graphics2D g2) {
            BasicStroke casing = new BasicStroke(9f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND);
            BasicStroke solid = new BasicStroke(3.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND);
            BasicStroke dashed = new BasicStroke(3f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_ROUND,
                    1f, new float[]{8f, 6f}, 0f);
            Set<Long> drawn = new HashSet<>();
            for (int u = 0; u < g.size(); u++) {
                for (Edge e : g.adj.get(u)) {
                    int v = e.to;
                    long key = ((long) Math.min(u, v) << 32) | Math.max(u, v);
                    if (!drawn.add(key)) continue;           // draw each road once
                    Point2D.Double p = pos(u), q = pos(v);
                    boolean blocked = g.isBlocked(u, v);
                    g2.setStroke(casing);
                    g2.setColor(new Color(255, 255, 255, 235));
                    g2.drawLine((int) p.x, (int) p.y, (int) q.x, (int) q.y);
                    g2.setStroke(blocked ? dashed : solid);
                    g2.setColor(blocked ? new Color(220, 38, 38) : ROAD);
                    g2.drawLine((int) p.x, (int) p.y, (int) q.x, (int) q.y);
                }
            }
        }

        private void drawRoute(Graphics2D g2) {
            if (route.size() < 2) return;
            List<Point2D.Double> pts = new ArrayList<>();
            for (int id : route) pts.add(pos(id));

            double total = 0;
            for (int i = 0; i + 1 < pts.size(); i++) total += pts.get(i).distance(pts.get(i + 1));
            double remaining = total * progress;

            Path2D.Double path = new Path2D.Double();
            path.moveTo(pts.get(0).x, pts.get(0).y);
            for (int i = 0; i + 1 < pts.size(); i++) {
                Point2D.Double p = pts.get(i), q = pts.get(i + 1);
                double len = p.distance(q);
                if (remaining >= len) {
                    path.lineTo(q.x, q.y);
                    remaining -= len;
                } else {
                    double f = len == 0 ? 0 : remaining / len;
                    path.lineTo(p.x + (q.x - p.x) * f, p.y + (q.y - p.y) * f);
                    break;
                }
            }
            g2.setStroke(new BasicStroke(16f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2.setColor(new Color(37, 99, 235, 50));
            g2.draw(path);
            g2.setStroke(new BasicStroke(7f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2.setColor(BLUE);
            g2.draw(path);
            g2.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2.setColor(new Color(147, 197, 253));
            g2.draw(path);
        }

        /** Distance (km) on each road, drawn after the route so the numbers stay readable. */
        private void drawRoadLabels(Graphics2D g2) {
            g2.setFont(new Font("SansSerif", Font.BOLD, 10));
            FontMetrics fm = g2.getFontMetrics();
            Set<Long> drawn = new HashSet<>();
            for (int u = 0; u < g.size(); u++) {
                for (Edge e : g.adj.get(u)) {
                    int v = e.to;
                    long key = ((long) Math.min(u, v) << 32) | Math.max(u, v);
                    if (!drawn.add(key)) continue;
                    Point2D.Double p = pos(u), q = pos(v);
                    String label = String.format("%.1f", e.distanceKm);
                    int tw = fm.stringWidth(label);
                    int cx = (int) ((p.x + q.x) / 2);
                    int cy = (int) ((p.y + q.y) / 2);
                    int bx = cx - tw / 2 - 4, by = cy - fm.getHeight() / 2, bw = tw + 8, bh = fm.getHeight();
                    g2.setColor(new Color(255, 255, 255, 240));
                    g2.fillRoundRect(bx, by, bw, bh, 8, 8);
                    g2.setColor(new Color(203, 213, 225));
                    g2.drawRoundRect(bx, by, bw, bh, 8, 8);
                    g2.setColor(new Color(71, 85, 105));
                    g2.drawString(label, cx - tw / 2, by + fm.getAscent());
                }
            }
        }

        private void drawNodes(Graphics2D g2) {
            g2.setFont(new Font("SansSerif", Font.BOLD, 12));
            FontMetrics fm = g2.getFontMetrics();
            boolean routeDone = progress >= 1.0;
            for (int i = 0; i < g.size(); i++) {
                Point2D.Double p = pos(i);
                boolean endpoint = i == from || i == to;
                int r = endpoint ? 12 : 9;
                Color fill = i == from ? GREEN : i == to ? RED : (routeDone && route.contains(i)) ? BLUE : NODE;

                if (i == hover) {
                    g2.setColor(new Color(37, 99, 235, 70));
                    g2.fillOval((int) p.x - r - 8, (int) p.y - r - 8, 2 * r + 16, 2 * r + 16);
                }
                g2.setColor(new Color(15, 23, 42, 45));                       // soft shadow
                g2.fillOval((int) p.x - r + 1, (int) p.y - r + 3, 2 * r + 2, 2 * r + 2);
                g2.setColor(Color.WHITE);
                g2.fillOval((int) p.x - r - 3, (int) p.y - r - 3, 2 * r + 6, 2 * r + 6);
                g2.setColor(fill);
                g2.fillOval((int) p.x - r, (int) p.y - r, 2 * r, 2 * r);

                String name = g.nodes.get(i).name;
                int tw = fm.stringWidth(name);
                int bw = tw + 14, bh = fm.getHeight() + 2;
                int bx = (int) p.x + r + 4;
                int by = (int) p.y - r - bh + 2;
                g2.setColor(new Color(15, 23, 42, 30));
                g2.fillRoundRect(bx + 1, by + 2, bw, bh, 10, 10);
                g2.setColor(endpoint ? fill : Color.WHITE);
                g2.fillRoundRect(bx, by, bw, bh, 10, 10);
                g2.setColor(endpoint ? fill : new Color(203, 213, 225));
                g2.drawRoundRect(bx, by, bw, bh, 10, 10);
                g2.setColor(endpoint ? Color.WHITE : new Color(15, 23, 42));
                g2.drawString(name, bx + 7, by + fm.getAscent() + 1);
            }
        }

        private void drawLegend(Graphics2D g2) {
            g2.setFont(new Font("SansSerif", Font.PLAIN, 12));
            FontMetrics fm = g2.getFontMetrics();
            String[] text = {"From", "To", "Route", "Blocked road"};
            int width = 16;
            for (String t : text) width += 20 + fm.stringWidth(t) + 14;
            int x0 = 16, h = 30, y0 = getHeight() - h - 14;

            g2.setColor(new Color(15, 23, 42, 25));
            g2.fillRoundRect(x0 + 1, y0 + 3, width, h, 14, 14);
            g2.setColor(new Color(255, 255, 255, 240));
            g2.fillRoundRect(x0, y0, width, h, 14, 14);
            g2.setColor(new Color(203, 213, 225));
            g2.drawRoundRect(x0, y0, width, h, 14, 14);

            int x = x0 + 14, y = y0 + h / 2 + 4;
            for (int i = 0; i < text.length; i++) {
                if (i < 2) {
                    g2.setColor(i == 0 ? GREEN : RED);
                    g2.fillOval(x, y - 10, 11, 11);
                } else if (i == 2) {
                    g2.setColor(BLUE);
                    g2.setStroke(new BasicStroke(5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                    g2.drawLine(x, y - 4, x + 14, y - 4);
                } else {
                    g2.setColor(RED);
                    g2.setStroke(new BasicStroke(2.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_ROUND,
                            1f, new float[]{5f, 3f}, 0f));
                    g2.drawLine(x, y - 4, x + 14, y - 4);
                }
                g2.setColor(new Color(51, 65, 85));
                g2.drawString(text[i], x + 20, y);
                x += 20 + fm.stringWidth(text[i]) + 14;
            }
        }
    }
}