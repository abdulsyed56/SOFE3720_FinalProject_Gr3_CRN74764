import tkinter as tk
from tkinter import messagebox
import heapq
import time
from abc import ABC, abstractmethod

# Grid size config
# 25x25 gives enough space to visualize paths clearly
GRID_ROWS = 25
GRID_COLS = 25
CELL_SIZE = 24  # size of each square in pixels

# Color setup for UI + visualization
# Different colors for each algo to compare behavior easily
COLOR_EMPTY = "#FFFFFF"
COLOR_WALL = "#2C3E50"
COLOR_START = "#27AE60"
COLOR_END = "#E74C3C"
COLOR_EXPLORED_ASTAR = "#85C1E9"
COLOR_EXPLORED_GREEDY = "#F9E79F"
COLOR_EXPLORED_UCS = "#D2B4DE"
COLOR_PATH_ASTAR = "#2E86C1"
COLOR_PATH_GREEDY = "#F39C12"
COLOR_PATH_UCS = "#8E44AD"
COLOR_GRID_LINE = "#BDC3C7"

# Node = single cell in grid
# Stores position + search info (g, h, f, parent)
class Node:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.g_cost = float("inf")   # cost from start → node
        self.h_cost = 0              # heuristic estimate → goal
        self.f_cost = float("inf")   # total priority value
        self.parent = None           # used to rebuild final path
        self.is_wall = False
        self.is_start = False
        self.is_end = False

    # Reset search values only (keep walls/start/end)
    def reset_costs(self):
        self.g_cost = float("inf")
        self.h_cost = 0
        self.f_cost = float("inf")
        self.parent = None

    # Needed so heapq can compare nodes
    def __lt__(self, other):
        return self.f_cost < other.f_cost

# Grid class manages all nodes
class Grid:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.nodes = [[Node(r, c) for c in range(cols)] for r in range(rows)]
        self.start_node = None
        self.end_node = None

    # Full reset (clear everything)
    def reset(self):
        for r in range(self.rows):
            for c in range(self.cols):
                node = self.nodes[r][c]
                node.is_wall = False
                node.is_start = False
                node.is_end = False
                node.reset_costs()
        self.start_node = None
        self.end_node = None

    # Reset only search-related values
    def reset_costs(self):
        for r in range(self.rows):
            for c in range(self.cols):
                self.nodes[r][c].reset_costs()

    # Set start node (only one allowed)
    def set_start(self, row, col):
        if self.start_node:
            self.start_node.is_start = False
        node = self.nodes[row][col]
        node.is_start = True
        node.is_wall = False
        node.is_end = False
        if self.end_node == node:
            self.end_node = None
        self.start_node = node

    # Set goal node
    def set_end(self, row, col):
        if self.end_node:
            self.end_node.is_end = False
        node = self.nodes[row][col]
        node.is_end = True
        node.is_wall = False
        node.is_start = False
        if self.start_node == node:
            self.start_node = None
        self.end_node = node

    # Toggle wall state
    def toggle_wall(self, row, col):
        node = self.nodes[row][col]
        if node.is_start or node.is_end:
            return
        node.is_wall = not node.is_wall

    # Set wall explicitly (used in drag)
    def set_wall(self, row, col, state=True):
        node = self.nodes[row][col]
        if node.is_start or node.is_end:
            return
        node.is_wall = state

    # Get valid neighbors (4-dir movement only)
    def get_neighbors(self, node):
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # up/down/left/right
            nr, nc = node.row + dr, node.col + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                neighbor = self.nodes[nr][nc]
                if not neighbor.is_wall:
                    neighbors.append(neighbor)
        return neighbors

# Base class for algorithms
class PathfindingAlgorithm(ABC):
    def __init__(self, grid, name="Algorithm"):
        self.grid = grid
        self.name = name

    @abstractmethod
    def solve(self):
        pass

    # Manhattan distance heuristic (fits 4-dir grid)
    @staticmethod
    def manhattan_distance(node_a, node_b):
        return abs(node_a.row - node_b.row) + abs(node_a.col - node_b.col)

    # Reconstruct path using parent pointers
    @staticmethod
    def reconstruct_path(end_node):
        path = []
        current = end_node
        while current is not None:
            path.append((current.row, current.col))
            current = current.parent
        path.reverse()
        return path

# A* search (g + h)
class AStarAlgorithm(PathfindingAlgorithm):
    def __init__(self, grid):
        super().__init__(grid, "A*")

    def solve(self):
        start = self.grid.start_node
        end = self.grid.end_node

        self.grid.reset_costs()
        explored_order = []
        start_time = time.perf_counter()

        start.g_cost = 0
        start.h_cost = self.manhattan_distance(start, end)
        start.f_cost = start.g_cost + start.h_cost

        sr, sc = start.row, start.col
        er, ec = end.row, end.col
        dr_goal = er - sr
        dc_goal = ec - sc

        # tie-break helper (slightly improves path shape)
        def cross_product_tiebreak(node):
            dr_node = node.row - sr
            dc_node = node.col - sc
            return abs(dr_node * dc_goal - dc_node * dr_goal)

        counter = 0

        open_set = [(start.f_cost, start.h_cost, 0, counter, start)]
        in_open = {(start.row, start.col)}
        closed_set = set()

        while open_set:
            current_f, current_h, _, _, current = heapq.heappop(open_set)
            current_pos = (current.row, current.col)
            in_open.discard(current_pos)

            if current_pos in closed_set:
                continue

            closed_set.add(current_pos)
            explored_order.append(current_pos)

            if current == end:
                elapsed = time.perf_counter() - start_time
                path = self.reconstruct_path(end)
                return explored_order, path, {
                    "nodes_expanded": len(explored_order),
                    "path_length": len(path),
                    "time": elapsed,
                }

            for neighbor in self.grid.get_neighbors(current):
                neighbor_pos = (neighbor.row, neighbor.col)
                if neighbor_pos in closed_set:
                    continue

                # uniform cost (each step = 1)
                tentative_g = current.g_cost + 1

                if tentative_g < neighbor.g_cost:
                    neighbor.parent = current
                    neighbor.g_cost = tentative_g
                    neighbor.h_cost = self.manhattan_distance(neighbor, end)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    counter += 1
                    cross = cross_product_tiebreak(neighbor)
                    heapq.heappush(open_set, (neighbor.f_cost, neighbor.h_cost, cross, counter, neighbor))
                    in_open.add(neighbor_pos)

        elapsed = time.perf_counter() - start_time
        return explored_order, [], {
            "nodes_expanded": len(explored_order),
            "path_length": 0,
            "time": elapsed,
        }

# Greedy Best-First (uses only h)
class GreedyAlgorithm(PathfindingAlgorithm):
    def __init__(self, grid):
        super().__init__(grid, "Greedy Best-First")

    def solve(self):
        start = self.grid.start_node
        end = self.grid.end_node

        self.grid.reset_costs()
        explored_order = []
        start_time = time.perf_counter()

        start.g_cost = 0
        start.h_cost = self.manhattan_distance(start, end)
        start.f_cost = start.h_cost  # key difference

        counter = 0
        open_set = [(start.f_cost, counter, start)]
        in_open = {(start.row, start.col)}
        closed_set = set()

        while open_set:
            _, _, current = heapq.heappop(open_set)
            current_pos = (current.row, current.col)
            in_open.discard(current_pos)

            if current_pos in closed_set:
                continue

            closed_set.add(current_pos)
            explored_order.append(current_pos)

            if current == end:
                elapsed = time.perf_counter() - start_time
                path = self.reconstruct_path(end)
                return explored_order, path, {
                    "nodes_expanded": len(explored_order),
                    "path_length": len(path),
                    "time": elapsed,
                }

            for neighbor in self.grid.get_neighbors(current):
                neighbor_pos = (neighbor.row, neighbor.col)
                if neighbor_pos in closed_set:
                    continue

                tentative_g = current.g_cost + 1

                if tentative_g < neighbor.g_cost:
                    neighbor.parent = current
                    neighbor.g_cost = tentative_g
                    neighbor.h_cost = self.manhattan_distance(neighbor, end)
                    neighbor.f_cost = neighbor.h_cost
                    counter += 1
                    heapq.heappush(open_set, (neighbor.f_cost, counter, neighbor))
                    in_open.add(neighbor_pos)

        elapsed = time.perf_counter() - start_time
        return explored_order, [], {
            "nodes_expanded": len(explored_order),
            "path_length": 0,
            "time": elapsed,
        }

# UCS (uses only g)
class UCSAlgorithm(PathfindingAlgorithm):
    def __init__(self, grid):
        super().__init__(grid, "UCS")

    def solve(self):
        start = self.grid.start_node
        end = self.grid.end_node

        self.grid.reset_costs()
        explored_order = []
        start_time = time.perf_counter()

        start.g_cost = 0
        start.f_cost = start.g_cost

        counter = 0
        open_set = [(start.f_cost, counter, start)]
        in_open = {(start.row, start.col)}
        closed_set = set()

        while open_set:
            _, _, current = heapq.heappop(open_set)
            current_pos = (current.row, current.col)
            in_open.discard(current_pos)

            if current_pos in closed_set:
                continue

            closed_set.add(current_pos)
            explored_order.append(current_pos)

            if current == end:
                elapsed = time.perf_counter() - start_time
                path = self.reconstruct_path(end)
                return explored_order, path, {
                    "nodes_expanded": len(explored_order),
                    "path_length": len(path),
                    "time": elapsed,
                }

            for neighbor in self.grid.get_neighbors(current):
                neighbor_pos = (neighbor.row, neighbor.col)
                if neighbor_pos in closed_set:
                    continue

                tentative_g = current.g_cost + 1

                if tentative_g < neighbor.g_cost:
                    neighbor.parent = current
                    neighbor.g_cost = tentative_g
                    neighbor.f_cost = neighbor.g_cost
                    counter += 1
                    heapq.heappush(open_set, (neighbor.f_cost, counter, neighbor))
                    in_open.add(neighbor_pos)

        elapsed = time.perf_counter() - start_time
        return explored_order, [], {
            "nodes_expanded": len(explored_order),
            "path_length": 0,
            "time": elapsed,
        }
    
class VisualizerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Pathfinding Visualizer — A* | Greedy | UCS")
        self.root.resizable(True, True)

        self.grid = Grid(GRID_ROWS, GRID_COLS)  # main data model (grid of nodes)

        self.mode = "wall"  # current user mode (wall / start / end)
        self.is_running = False  # prevents interaction while algos are running

        self.cell_rects = {}  # maps (row, col) → canvas rectangle id (for updating colors)

        self.is_dragging = False  # used for click + drag wall drawing
        self.drag_wall_state = True  # determines if dragging adds or removes walls

        self.animation_speed = 15  # delay between animation steps (ms)

        self._build_ui()   # build full UI layout
        self._draw_grid()  # draw initial empty grid

    def _build_ui(self):
        # main container
        main_frame = tk.Frame(self.root, bg="#ECF0F1")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # top toolbar (buttons + title)
        toolbar = tk.Frame(main_frame, bg="#2C3E50", pady=8)
        toolbar.pack(fill=tk.X)

        # title text
        title_label = tk.Label(
            toolbar, text="Pathfinding Visualizer",
            font=("Helvetica", 16, "bold"), fg="white", bg="#2C3E50"
        )
        title_label.pack(side=tk.LEFT, padx=15)

        # shared button styling
        btn_style = {"font": ("Helvetica", 10), "width": 12, "relief": tk.FLAT, "cursor": "hand2"}

        # mode buttons
        self.btn_wall = tk.Button(
            toolbar, text="Draw Walls", bg="#95A5A6", fg="white",
            command=lambda: self._set_mode("wall"), **btn_style
        )
        self.btn_wall.pack(side=tk.LEFT, padx=3)

        self.btn_start = tk.Button(
            toolbar, text="Set Start", bg="#27AE60", fg="white",
            command=lambda: self._set_mode("start"), **btn_style
        )
        self.btn_start.pack(side=tk.LEFT, padx=3)

        self.btn_end = tk.Button(
            toolbar, text="Set End", bg="#E74C3C", fg="white",
            command=lambda: self._set_mode("end"), **btn_style
        )
        self.btn_end.pack(side=tk.LEFT, padx=3)

        # speed slider section
        speed_frame = tk.Frame(toolbar, bg="#2C3E50")
        speed_frame.pack(side=tk.LEFT, padx=10)

        tk.Label(speed_frame, text="Slow", fg="#BDC3C7", bg="#2C3E50",
                 font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(4, 0))

        self.speed_var = tk.IntVar(value=50)  # slider value

        speed_scale = tk.Scale(
            speed_frame, from_=1, to=100, orient=tk.HORIZONTAL,
            variable=self.speed_var, length=100, bg="#2C3E50", fg="white",
            highlightthickness=0, troughcolor="#7F8C8D", showvalue=0,
            command=self._on_speed_change
        )
        speed_scale.pack(side=tk.LEFT)

        tk.Label(speed_frame, text="Fast", fg="#BDC3C7", bg="#2C3E50",
                 font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(0, 4))

        # run button (starts all algorithms)
        self.btn_run = tk.Button(
            toolbar, text="Run All", bg="#2980B9", fg="white",
            font=("Helvetica", 10, "bold"), width=10, relief=tk.FLAT,
            cursor="hand2", command=self._run_all
        )
        self.btn_run.pack(side=tk.RIGHT, padx=3)

        # reset buttons
        self.btn_clear = tk.Button(
            toolbar, text="Reset Grid", bg="#E67E22", fg="white",
            command=self._reset_grid, **btn_style
        )
        self.btn_clear.pack(side=tk.RIGHT, padx=3)

        self.btn_clear_paths = tk.Button(
            toolbar, text="Clear Paths", bg="#D35400", fg="white",
            command=self._clear_paths, **btn_style
        )
        self.btn_clear_paths.pack(side=tk.RIGHT, padx=3)

        # main content split (left = grid, right = results)
        content_frame = tk.Frame(main_frame, bg="#ECF0F1")
        content_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(5, 10))

        # LEFT SIDE (grid + legend)
        left_frame = tk.Frame(content_frame, bg="#ECF0F1")
        left_frame.pack(side=tk.LEFT, anchor=tk.N)

        # status text showing current mode
        self.status_label = tk.Label(
            left_frame, text="Mode: Draw Walls (click & drag on grid)",
            font=("Helvetica", 11), bg="#ECF0F1", fg="#2C3E50"
        )
        self.status_label.pack(pady=(0, 4))

        # canvas for grid drawing
        canvas_width = GRID_COLS * CELL_SIZE
        canvas_height = GRID_ROWS * CELL_SIZE

        self.canvas = tk.Canvas(
            left_frame, width=canvas_width, height=canvas_height,
            bg=COLOR_EMPTY, highlightthickness=1, highlightbackground="#7F8C8D"
        )
        self.canvas.pack(padx=5, pady=(0, 5))

        # mouse bindings
        self.canvas.bind("<Button-1>", self._on_click)      # click
        self.canvas.bind("<B1-Motion>", self._on_drag)      # drag
        self.canvas.bind("<ButtonRelease-1>", self._on_release)  # release

        # legend section (shows meaning of colors)
        legend_frame = tk.Frame(left_frame, bg="#ECF0F1")
        legend_frame.pack(pady=(5, 0))

        # (legend rows — just UI labels, no logic impact)
        # shows start/end/wall + explored/path for each algo

        # RIGHT SIDE (results table)
        right_frame = tk.Frame(content_frame, bg="#ECF0F1")
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(15, 5), anchor=tk.N)

        tk.Label(
            right_frame, text="Performance Comparison",
            font=("Helvetica", 14, "bold"), bg="#ECF0F1", fg="#2C3E50"
        ).pack(pady=(0, 10), anchor=tk.W)

        self.results_frame = tk.Frame(right_frame, bg="#ECF0F1")
        self.results_frame.pack(fill=tk.BOTH, expand=True)

        # default message before running algorithms
        self.results_label = tk.Label(
            self.results_frame,
            text="Set start & end nodes, draw walls,\nthen click 'Run All' to compare\nalgorithms.",
            font=("Helvetica", 11), bg="#ECF0F1", fg="#7F8C8D", justify=tk.LEFT
        )
        self.results_label.pack(anchor=tk.W)

    # draws grid cells and stores their rectangle IDs
    def _draw_grid(self):
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                x1 = c * CELL_SIZE
                y1 = r * CELL_SIZE
                x2 = x1 + CELL_SIZE
                y2 = y1 + CELL_SIZE

                rect = self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=COLOR_EMPTY, outline=COLOR_GRID_LINE, width=1
                )

                self.cell_rects[(r, c)] = rect  # store for quick updates later

    # update a single cell color visually
    def _update_cell_color(self, row, col, color):
        rect = self.cell_rects.get((row, col))
        if rect:
            self.canvas.itemconfig(rect, fill=color)

    # redraw entire grid based on node states
    def _refresh_grid_display(self):
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                node = self.grid.nodes[r][c]

                if node.is_start:
                    color = COLOR_START
                elif node.is_end:
                    color = COLOR_END
                elif node.is_wall:
                    color = COLOR_WALL
                else:
                    color = COLOR_EMPTY

                self._update_cell_color(r, c, color)

    # convert mouse click → grid cell
    def _get_cell_from_event(self, event):
        col = event.x // CELL_SIZE
        row = event.y // CELL_SIZE

        if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
            return row, col
        return None, None

    # handle click event
    def _on_click(self, event):
        if self.is_running:
            return

        row, col = self._get_cell_from_event(event)
        if row is None:
            return

        if self.mode == "start":
            self.grid.set_start(row, col)

        elif self.mode == "end":
            self.grid.set_end(row, col)

        elif self.mode == "wall":
            node = self.grid.nodes[row][col]

            # decide whether drag is adding or removing walls
            self.drag_wall_state = not node.is_wall

            self.grid.set_wall(row, col, self.drag_wall_state)
            self.is_dragging = True

        self._refresh_grid_display()

    # drag for drawing walls continuously
    def _on_drag(self, event):
        if self.is_running or self.mode != "wall" or not self.is_dragging:
            return

        row, col = self._get_cell_from_event(event)
        if row is None:
            return

        self.grid.set_wall(row, col, self.drag_wall_state)
        self._refresh_grid_display()

    # stop dragging
    def _on_release(self, event):
        self.is_dragging = False

    # convert slider value → animation delay
    def _on_speed_change(self, value):
        self.animation_speed = max(1, int(201 - int(value) * 2))

    # switch user mode
    def _set_mode(self, mode):
        self.mode = mode

        mode_texts = {
            "wall": "Mode: Draw Walls (click & drag on grid)",
            "start": "Mode: Set Start (click a cell)",
            "end": "Mode: Set End (click a cell)",
        }

        self.status_label.config(text=mode_texts.get(mode, ""))

    # full reset (everything cleared)
    def _reset_grid(self):
        if self.is_running:
            return

        self.grid.reset()
        self._refresh_grid_display()

        for widget in self.results_frame.winfo_children():
            widget.destroy()

    # clears only paths (keeps walls/start/end)
    def _clear_paths(self):
        if self.is_running:
            return

        self.grid.reset_costs()
        self._refresh_grid_display()

    # main entry to run all algorithms
    def _run_all(self):
        if self.is_running:
            return

        if not self.grid.start_node or not self.grid.end_node:
            messagebox.showwarning("Missing Nodes", "Please set both a start and an end node.")
            return

        self.is_running = True
        self.btn_run.config(state=tk.DISABLED)

        self._clear_paths()

        # list of algorithms + their colors
        algorithms = [
            (AStarAlgorithm(self.grid), COLOR_EXPLORED_ASTAR, COLOR_PATH_ASTAR),
            (GreedyAlgorithm(self.grid), COLOR_EXPLORED_GREEDY, COLOR_PATH_GREEDY),
            (UCSAlgorithm(self.grid), COLOR_EXPLORED_UCS, COLOR_PATH_UCS),
        ]

        all_results = []

        self._run_algorithm_sequence(algorithms, 0, all_results)

            # Runs algos one after another instead of all at once visually
    # This lets each one animate cleanly, then moves to the next
    def _run_algorithm_sequence(self, algorithms, index, all_results):
        # Base case: all algos have finished
        if index >= len(algorithms):
            self.status_label.config(
                text="All algorithms complete — showing combined results",
                fg="#2C3E50"
            )

            # Reset grid view first so final combined coloring is clean
            self._refresh_grid_display()

            # Repaint explored cells for each algo
            for name, explored, path, stats, ec, pc in all_results:
                for r, c in explored:
                    node = self.grid.nodes[r][c]
                    if not node.is_start and not node.is_end:
                        self._update_cell_color(r, c, ec)

            # Repaint final paths after explored cells so paths stand out on top
            for name, explored, path, stats, ec, pc in all_results:
                for r, c in path:
                    node = self.grid.nodes[r][c]
                    if not node.is_start and not node.is_end:
                        self._update_cell_color(r, c, pc)

            # Show stats table on the right side
            self._display_results(all_results)

            # Unlock UI again
            self.is_running = False
            self.btn_run.config(state=tk.NORMAL)
            return

        # Get current algo and its display colors
        algo, explore_color, path_color = algorithms[index]

        # Update status so user knows which algo is running now
        self.status_label.config(
            text=f"Running: {algo.name}  ({index + 1} of {len(algorithms)})",
            fg=path_color
        )

        # Clear old path coloring before animating current algo
        self._refresh_grid_display()

        # Solve path first, then animate results
        explored_order, path, stats = algo.solve()

        # Save full result so it can be shown later in combined view + table
        all_results.append((algo.name, explored_order, path, stats, explore_color, path_color))

        # Animate explored cells first, then animate final path,
        # then wait 500 ms and start next algo
        self._animate_explored(explored_order, explore_color, 0,
                               lambda: self._animate_path(path, path_color, 0,
                                                          lambda: self.root.after(
                                                              500,
                                                              self._run_algorithm_sequence,
                                                              algorithms, index + 1, all_results)))

    # Animates explored cells one at a time
    def _animate_explored(self, explored, color, idx, callback):
        # Once all explored cells are done, continue to next step
        if idx >= len(explored):
            callback()
            return

        r, c = explored[idx]
        node = self.grid.nodes[r][c]

        # Keep start/end colors unchanged
        if not node.is_start and not node.is_end:
            self._update_cell_color(r, c, color)

        # Schedule next explored cell after a small delay
        self.root.after(self.animation_speed, self._animate_explored,
                        explored, color, idx + 1, callback)

    # Animates final shortest path one cell at a time
    def _animate_path(self, path, color, idx, callback):
        # Once full path is drawn, continue to next callback
        if idx >= len(path):
            callback()
            return

        r, c = path[idx]
        node = self.grid.nodes[r][c]

        # Keep start/end colors unchanged
        if not node.is_start and not node.is_end:
            self._update_cell_color(r, c, color)

        # Path anim uses a minimum delay of 30 ms so it stays visible
        self.root.after(max(self.animation_speed, 30), self._animate_path,
                        path, color, idx + 1, callback)

    # Builds and displays results table on the right panel
    def _display_results(self, all_results):
        # Clear any old widgets already in results area
        for widget in self.results_frame.winfo_children():
            widget.destroy()

        # Table container
        table = tk.Frame(self.results_frame, bg="#2C3E50")
        table.pack(anchor=tk.W)

        # Table headers
        headers = ["Algorithm", "Nodes Expanded", "Path Length", "Time (ms)"]
        for j, header in enumerate(headers):
            tk.Label(
                table, text=header, font=("Helvetica", 10, "bold"),
                bg="#2C3E50", fg="white", padx=15, pady=6
            ).grid(row=0, column=j, sticky="nsew")

        # Color map so algo name in table matches its path color on grid
        algo_colors = {
            "A*": COLOR_PATH_ASTAR,
            "Greedy Best-First": COLOR_PATH_GREEDY,
            "UCS": COLOR_PATH_UCS,
        }

        # Fill table rows with each algo's stats
        for i, (name, explored, path, stats, ec, pc) in enumerate(all_results):
            # Alternate row bg for readability
            row_bg = "#F8F9FA" if i % 2 == 0 else "#EAECEE"

            name_color = algo_colors.get(name, "#2C3E50")

            # If no path found, show text instead of 0
            path_len_str = str(stats["path_length"]) if stats["path_length"] > 0 else "No path"

            # Convert runtime from seconds → ms
            time_ms = f"{stats['time'] * 1000:.2f}"

            values = [name, str(stats["nodes_expanded"]), path_len_str, time_ms]

            for j, val in enumerate(values):
                # First col is highlighted with algo color + bold font
                fg = name_color if j == 0 else "#2C3E50"
                font = ("Helvetica", 10, "bold") if j == 0 else ("Helvetica", 10)

                tk.Label(
                    table, text=val, font=font,
                    bg=row_bg, fg=fg, padx=15, pady=5
                ).grid(row=i + 1, column=j, sticky="nsew")    

# App entry point
if __name__ == "__main__":
    root = tk.Tk()
    app = VisualizerApp(root)
    root.mainloop()
