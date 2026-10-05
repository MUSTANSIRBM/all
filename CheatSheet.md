<a id="top"></a>
<a href="#top" title="Back to top" style="position: fixed; bottom: 18px; right: 18px; display: inline-block; background: #2e6b8f; color: #fff; border-radius: 50%; width: 42px; height: 42px; line-height: 42px; text-align: center; font-size: 22px; text-decoration: none; box-shadow: 0 2px 6px rgba(0,0,0,.35); z-index: 999;">&#8593;</a>

# CHEATSHEET — DATA STRUCTURES & GAME AI
*Interview Definitions, No Code Required · synced to the 33 sections of ASCII_ART.md (BASE → ADVANCED → MORE)*


> **TIP: Click any topic in the index below to jump straight to that section.**


| # | **Topic** | # | **Topic** |
|---|-----------|---|-----------|
| **PART A — BASE DATA STRUCTURES** |  | **PART D — DECISION MAKING** | |
| 1 | [SINGLY LINKED LIST (SLL)](#1-singly-linked-list-sll) | 22 | [FINITE STATE MACHINE (FSM) - enemy AI](#22-finite-state-machine-fsm---enemy-ai) |
| 2 | [DOUBLY LINKED LIST (DLL)](#2-doubly-linked-list-dll) | 23 | [BEHAVIOR TREE](#23-behavior-tree) |
| 3 | [CIRCULAR DOUBLY LINKED LIST (CDLL)](#3-circular-doubly-linked-list-cdll) | 24 | [BLACKBOARD SYSTEM](#24-blackboard-system) |
| 4 | [SINGLY CIRCULAR LINKED LIST (SCLL)](#4-singly-circular-linked-list-scll) |  | |
| 5 | [LINKED STACK (LIFO)](#5-linked-stack-lifo---last-in-first-out) | 25 | [STEERING BEHAVIORS](#25-steering-behaviors) |
| 6 | [ARRAY STACK (Static / Fixed Size)](#6-array-stack-static--fixed-size) | 26 | [STEERING FORCE BLENDING (CHASE = seek + flock)](#26-steering-force-blending-chase--seek--flock) |
| 7 | [LINKED QUEUE (FIFO)](#7-linked-queue-dynamic---fifo) | 27 | [THE PER-FRAME PIPELINE (update vs updateMovement)](#27-the-per-frame-pipeline-update-vs-updatemovement) |
| 8 | [CIRCULAR ARRAY QUEUE (Static + Circular)](#8-circular-array-queue-static--circular) | 28 | [PATROL WAYPOINT LOOP](#28-patrol-waypoint-loop) |
| 9 | [MAX-HEAP (Binary Heap - Array Based)](#9-max-heap-binary-heap---array-based) | 29 | [WAYPOINT ADVANCE / ARRIVAL THRESHOLD](#29-waypoint-advance--arrival-threshold) |
| **PART B — ADVANCED DATA STRUCTURES** |  | 30 | [REPATH TIMER & STICKY TARGETS](#30-repath-timer--sticky-targets) |
| 10 | [BINARY SEARCH TREE (BST)](#10-binary-search-tree-bst) | 31 | [DATA-DRIVEN CONFIGS (JSON)](#31-data-driven-configs-json) |
| 11 | [TRIE (PREFIX TREE)](#11-trie-prefix-tree) | 32 | [THE 5 SIMULATION PHASES (main() end-to-end timeline)](#32-the-5-simulation-phases-main-end-to-end-timeline) |
| 12 | [GRAPH (ADJACENCY LIST + BFS/DFS)](#12-graph-adjacency-list--bfsdfs) | 33 | [MEMORY MAP & OBJECT LIFECYCLE (heap, RAII, ownership)](#33-memory-map--object-lifecycle-heap-raii-ownership) |
| 13 | [DIJKSTRA (SHORTEST PATH - WEIGHTED GRAPH)](#13-dijkstra-shortest-path---weighted-graph) | **PART E — GLOSSARY** | |
| **PART C — PATHFINDING** |  | A1 | [NAVMESH (Navigation Mesh)](#a1-navmesh-navigation-mesh) |
| 14 | [A* PATHFINDING](#14-a-pathfinding) | A2 | [POLYGON CENTROID](#a2-polygon-centroid) |
| 15 | [A* WITH WAYPOINTS (WEIGHTED NODES)](#15-a-with-waypoints-weighted-nodes) | A3 | [FSM (Finite State Machine)](#a3-fsm-finite-state-machine) |
| 16 | [TIME-SLICING (ASYNCHRONOUS A*)](#16-time-slicing-asynchronous-a) | A4 | [BEHAVIOR TREE (BT)](#a4-behavior-tree-bt) |
| 17 | [DISTANCE FORMULAS (4-dir / 8-dir / NavMesh)](#17-distance-formulas-4-dir--8-dir--navmesh) | A5 | [BLACKBOARD](#a5-blackboard) |
| 18 | [HPA* (HIERARCHICAL PATHFINDING A*)](#18-hpa-hierarchical-pathfinding-a) | A6 | [STEERING BEHAVIORS](#a6-steering-behaviors) |
| 19 | [PATH SMOOTHING (STRING-PULLING)](#19-path-smoothing-string-pulling) | A7 | [FLOCKING](#a7-flocking) |
| 20 | [FUNNEL ALGORITHM](#20-funnel-algorithm) | A8 | [REPATH TIMER / STICKY TARGET](#a8-repath-timer--sticky-target) |
| 21 | [DYNAMIC OBSTACLE MANAGEMENT](#21-dynamic-obstacle-management) | A9 | [HPA* (Heirarchical Pathfinding A*)](#a9-hpa-heirarchical-pathfinding-a) |
| |  | A10 | [Z-LOCK CAVEAT (from the project)](#a10-z-lock-caveat-from-the-project) |

### PART A — BASE DATA STRUCTURES  (§1–§9)
--------------------------------------------------------------------------------
## 1. SINGLY LINKED LIST (SLL)
```text
   DEFINITION: A linear collection of NODES where each node holds DATA and one
   NEXT pointer to the following node; the last node points to null.
   HOW IT WORKS: Traversal is strictly one-way, head -> tail. Insert/delete at
   the head is O(1); anywhere else costs a walk to find the position (O(n)).
   COMPLEXITY:
        Access   head O(1) , k-th O(n)
        Insert   at head O(1)   at tail/known position O(1) with pointer
        Delete   O(1) if you have the node's predecessor, else O(n)
        Search   O(n)
        Space    O(n) data + one pointer per node
   STRENGTHS: constant-time head insertion; no wasted capacity; easy splice.
   TRADE-OFFS: no random access; pointer overhead; poor cache locality.
   WHEN TO USE: undo log, adjacency lists, run-time-unknown sizes.
   CLASSIC QUESTION: "Reverse a linked list / detect a cycle / find the middle."
   TAKEAWAY: A container optimized for cheap head-insert and node splicing.
```

--------------------------------------------------------------------------------
## 2. DOUBLY LINKED LIST (DLL)
```text
   DEFINITION: A linked list whose nodes hold PREV and NEXT pointers, enabling
   traversal in both directions.
   HOW IT WORKS: Adding the back-pointer makes deleting a node O(1) given only
   that node (no need to search for its predecessor). A sentinel (dummy head)
   removes all the "is this the first/last?" edge cases.
   COMPLEXITY: all O(1) at head/tail; O(n) at arbitrary position; O(n) search.
   STRENGTHS: constant-time insert/delete at both ends and node deletion.
   TRADE-OFFS: double pointer overhead per node (~2x links).
   WHEN TO USE: LRU cache (with a hash map), editor undo/redo, browser history.
   CLASSIC QUESTION: "Implement an LRU cache" (DLL + hash map is the answer).
   TAKEAWAY: SLL's power plus instant delete-anywhere and backwards walking.
```

--------------------------------------------------------------------------------
## 3. CIRCULAR DOUBLY LINKED LIST (CDLL)
```text
   DEFINITION: A DLL where the tail's NEXT points back to the head and the
   head's PREV points to the tail, forming a ring.
   HOW IT WORKS: There is no "null end" — traversal wraps around until you reach
   your start node again, so you can walk forever in a loop safely.
   STRENGTHS: seamless wraparound; head is reachable from tail in O(1).
   TRADE-OFFS: risk of infinite loops if termination is coded wrong.
   WHEN TO USE: round-robin schedulers, turn-based queues, cache cycling.
   CLASSIC QUESTION: "Detect and prove a cycle in a linked list" (tortoise-hare).
   TAKEAWAY: A ring where the last node hands off to the first.
```

--------------------------------------------------------------------------------
## 4. SINGLY CIRCULAR LINKED LIST (SCLL)
```text
   DEFINITION: An SLL whose last node's NEXT wraps BACK to the head — no nullptr
   anywhere; tail->next == head, so the whole list is one continuous ring.
   HOW IT WORKS: Walking forward NEVER ends on its own; you must remember the
   head and STOP when temp wraps back to it, so a do-while loop is the safe
   way to traverse: do { print; temp = temp->next; } while (temp != head).
   COMPLEXITY: insert/delete at head O(1) (needs a walk to find the tail first);
   search O(n); space O(n) with one pointer per node.
   STRENGTHS: seamless wraparound with only ONE pointer per node (lightest ring);
   the whole list reachable from any node by walking forward.
   TRADE-OFFS: no backward traversal; infinite-loop risk if the stop condition
   is wrong; head-insert requires finding the tail (make it a ring by pointing
   tail->next at the new head).
   WHEN TO USE: round-robin / cyclic iteration, game turns, rotating buffers
   where the doubly-linked overhead is not justified.
   CLASSIC QUESTION: "Can a singly linked list be circular, and how does
   traversal ever end?" -> yes; with a saved head reference and a wrap check.
   TAKEAWAY: CDLL with half the links — a one-way ring.
```

--------------------------------------------------------------------------------
## 5. LINKED STACK (LIFO - Last In First Out)
```text
   DEFINITION: A stack built from a linked list where PUSH adds to the top and
   POP removes the top — Last In, First Out.
   HOW IT WORKS: "Top" = head of the list; push/pop are O(1) head operations.
   COMPLEXITY: push/pop/top O(1); space O(n) with per-node allocation.
   STRENGTHS: unbounded size, no capacity checks, O(1) push/pop.
   TRADE-OFFS: per-node allocation cost; worse cache behavior than array stack.
   WHEN TO USE: function call stack, expression evaluation, backtracking.
   CLASSIC QUESTION: "What is LIFO and where is a stack used in a compiler?"
   TAKEAWAY: LIFO policy backed by a linked list — size grows freely.
```

--------------------------------------------------------------------------------
## 6. ARRAY STACK (Static / Fixed Size)
```text
   DEFINITION: A stack stored in a contiguous array with one TOP index tracking
   how many elements it holds; push/pop move top by one.
   HOW IT WORKS: push checks capacity (OVERFLOW), writes at top, top++; pop
   returns top-1 (UNDERFLOW if empty).
   COMPLEXITY: push/pop/top O(1), cache-friendly; space reserved up front.
   STRENGTHS: minimal overhead, excellent cache locality, no allocation churn.
   TRADE-OFFS: fixed capacity — either overflow or wasted reserved space.
   WHEN TO USE: embedded systems, lock-free concurrent stacks, any known bound.
   CLASSIC QUESTION: "Stack vs heap memory" is different — here: "Array vs linked
   stack, which is better?" Answer: array for cache+speed if size is bounded.
   TAKEAWAY: O(1) all day, if you can afford the fixed reservation.
```

--------------------------------------------------------------------------------
## 7. LINKED QUEUE (Dynamic - FIFO)
```text
   DEFINITION: A queue built from a linked list that keeps BOTH a head and a
   tail pointer; enqueue at tail, dequeue at head — First In, First Out.
   HOW IT WORKS: enqueue appends in O(1) via tail; dequeue removes head in O(1).
   COMPLEXITY: enqueue/dequeue/peek O(1); space O(n).
   STRENGTHS: unbounded growth; O(1) at both ends.
   TRADE-OFFS: allocation per node; two pointers to maintain.
   WHEN TO USE: task scheduling, breadth-first traversal frontier, message queues.
   CLASSIC QUESTION: "Implement a queue with two stacks" (amortized O(1)).
   TAKEAWAY: FIFO order with constant-time enqueue and dequeue.
```

--------------------------------------------------------------------------------
## 8. CIRCULAR ARRAY QUEUE (Static + Circular)
```text
   DEFINITION: A queue in a fixed array with FRONT and BACK indices that wrap
   around (modulo the capacity), reusing freed slots instead of shifting.
   HOW IT WORKS: Use ((i + 1) % capacity) to advance. Size is tracked
   separately so you can tell FULL apart from EMPTY (both would show front == back).
   COMPLEXITY: enqueue/dequeue O(1); no shifting ever; O(1) indexed peek.
   STRENGTHS: no element shifting, constant time, cache-friendly, no allocation.
   TRADE-OFFS: fixed capacity; classic off-by-one on full/empty detection.
   WHEN TO USE: audio/video buffers, producer-consumer, stream processing.
   CLASSIC QUESTION: "How does a ring buffer distinguish full from empty?"
   TAKEAWAY: Fixed array + modular wrap = queue that never shifts its data.
```

--------------------------------------------------------------------------------
## 9. MAX-HEAP (Binary Heap - Array Based)
```text
   DEFINITION: A complete binary tree stored in a flat array where every parent
   is >= its children, so the LARGEST element is always at the root.
   HOW IT WORKS: Children of index i live at 2i+1 and 2i+2. Insert adds at the
   end then BUBBLES UP; extract-max swaps the root with the last leaf, removes
   it, and BUBBLES DOWN (sift-down). The structure underlies priority queues
   and heapsort.
   COMPLEXITY:
        Insert        O(log n)
        Extract max   O(log n)
        Peek max      O(1)
        Build heap    O(n)  (not O(n log n))
        Heapsort      O(n log n), in-place
   STRENGTHS: guaranteed O(log n) priority ops; contiguous memory.
   TRADE-OFFS: no search (O(n)); unstable ordering for equal keys in heapsort.
   WHEN TO USE: priority queues, Dijkstra/A*, top-K problems, scheduling.
   CLASSIC QUESTION: "Prove build-heap is O(n)" / "Sort by a heap, name it."
   TAKEAWAY: An array that thinks it's a tree — instant max, log insertion.
```

### PART B — ADVANCED DATA STRUCTURES  (§10–§13)
--------------------------------------------------------------------------------
## 10. BINARY SEARCH TREE (BST)
```text
   DEFINITION: A binary tree where every node's left subtree holds only smaller
   keys and its right subtree only larger keys — so IN-ORDER traversal yields
   sorted order.
   HOW IT WORKS: Search compares the key and descends left or right, halving
   the candidates each step. Cost depends on tree HEIGHT.
   COMPLEXITY:
        Search/Insert/Delete   O(h)  -> balanced O(log n), skewed O(n)
        In-order traversal     O(n)  -> sorted output
   STRENGTHS: ordered operations — min/max, next/predecessor, range queries.
   TRADE-OFFS: degrades to a linked list on sorted input (why self-balancing
   trees like AVL and Red-Black exist).
   WHEN TO USE: ordered map/set, range queries, in-order iteration.
   CLASSIC QUESTION: "Validate whether a tree is a BST" / "k-th smallest".
   TAKEAWAY: Sorted data with log-time search — IF the height stays balanced.
```

--------------------------------------------------------------------------------
## 11. TRIE (PREFIX TREE)
```text
   DEFINITION: A tree where each node represents one CHARACTER of a key and
   paths from root spell out keys; shared prefixes collapse into shared paths.
   HOW IT WORKS: Search follows characters down the tree — cost equals WORD
   LENGTH, not dictionary size. A flag marks nodes that end a complete word.
   COMPLEXITY: lookup/insert/delete O(L) where L = key length; space O(total
   characters), often with per-node children overhead.
   STRENGTHS: prefix queries and autocomplete in O(L); no hash collisions.
   TRADE-OFFS: memory-hungry (many nodes each holding a children map/array).
   WHEN TO USE: autocomplete, spell-check, dictionaries, IP routing (tries of
   bits), browser address bars.
   CLASSIC QUESTION: "Design autocomplete" / "word search in a dictionary".
   TAKEAWAY: Length-bound search — the input size, not the data size, matters.
```

--------------------------------------------------------------------------------
## 12. GRAPH (ADJACENCY LIST + BFS/DFS)
```text
   DEFINITION: A set of NODES (vertices) and EDGES connecting them; an
   adjacency list stores each vertex's neighbors in a list, making the total
   storage proportional to vertices + edges.
   VARIANTS: directed vs undirected; weighted vs unweighted.
   BFS:  processes level-by-level using a QUEUE -> shortest edge-count path,
         O(V + E).
   DFS:  explores as deep as possible using a STACK (or recursion), tracks
         visited nodes to avoid revisits -> O(V + E).
   COMPLEXITY:
        Storage        O(V + E)  (list) vs O(V^2)  (matrix)
        Neighbor scan  O(degree of vertex)
        BFS/DFS        O(V + E)
   STRENGTHS (list): compact for sparse graphs; fast iteration over neighbors.
   TRADE-OFFS (list): O(degree) edge-existence check; matrix wins for dense.
   WHEN TO USE: adjacency list = most real-world/sparse graphs; BFS for
   shortest-hop; DFS for cycle detection, connectivity, topological sort.
   CLASSIC QUESTION: "BFS vs DFS — when is each preferred?" BFS: closest/shortest
   hop; DFS: explore-whole, path existence, recursion-friendly, cycles.
   TAKEAWAY: Vertices + edge lists; BFS is level-order, DFS is depth-order.
```

--------------------------------------------------------------------------------
## 13. DIJKSTRA (SHORTEST PATH - WEIGHTED GRAPH)
```text
   DEFINITION: The classic single-source shortest-path algorithm for graphs
   with NON-NEGATIVE edge weights; it greedily expands the closest unsettled
   node and RELAXES (improves) its neighbors' distances.
   HOW IT WORKS: A priority queue yields the nearest known vertex each step;
   once a vertex is finalized its distance is final. The parent links rebuild
   the path.
   COMPLEXITY: O((V + E) log V) with a binary min-heap; O(V^2) naive.
   RESTRICTION: requires >= 0 weights — negative edges need Bellman-Ford, and
   negative cycles make "shortest" undefined.
   STRENGTHS: optimal for nonnegative weighted graphs; the base of A*.
   TRADE-OFFS: ignores heuristic (A* is Dijkstra + goal-directed heuristic).
   WHEN TO USE: GPS/routing, weighted graph shortest paths, network protocols.
   CLASSIC QUESTION: "Why does Dijkstra fail with negative edges?" (a negative
   edge can arrive after a vertex is already finalized -> wrong result).
   TAKEAWAY: Greedy + priority queue = exact shortest path for nonnegative
   weights.
```

### PART C — PATHFINDING  (§14–§21)
--------------------------------------------------------------------------------
## 14. A* PATHFINDING
```text
   DEFINITION: Dijkstra's algorithm boosted with a HEURISTIC h(n) that estimates
   the remaining cost, so it expands promising nodes first: f(n) = g(n) + h(n).
   HOW IT WORKS: g = cost from start so far; h = estimate to goal. A min-heap
   (open set) pops the lowest f; a closed set prevents re-expansion; if the
   heuristic is ADMISSIBLE (never overestimates) the result is optimal.
   COMPLEXITY: O(b^d) worst case (branching^depth); in practice much faster
   with a good heuristic; O(1) per pop via priority queue + lazy stale deletes.
   KEY TERMS: admissible heuristic, open/closed sets, g/h/f scores.
   SPECIAL CASES: h = 0 -> reduces to Dijkstra; greedy best-first drops g.
   CLASSIC QUESTION: "Why is A* optimal with an admissible heuristic?" / "What
   makes a heuristic admissible?" Answer: it never overestimates true cost.
   TAKEAWAY: Best-first search that is optimal exactly when the heuristic lies.
```

--------------------------------------------------------------------------------
## 15. A* WITH WAYPOINTS (WEIGHTED NODES)
```text
   DEFINITION: A* where the graph's nodes ARE hand-placed waypoints and its
   edges are custom connections with weights equal to travel cost — used to
   pathfind over a real map instead of a uniform grid of cells.
   HOW IT WORKS: f = g + h exactly as grid A* (see §14), except the heuristic
   is the straight-line (Euclidean) distance between waypoint coordinates, and
   an "edge" only exists where you explicitly connect two waypoints (weighted,
   hand-picked, NOT equal to step count).
   COMPLEXITY: O(b^d) worst case; tiny in practice because a waypoint graph
   has few nodes and sparse edges vs a fine grid.
   STRENGTHS: paths follow deliberate routes (bridges, tunnels, shortcuts);
   sparse graph = very fast search; weights model real travel cost.
   TRADE-OFFS: waypoints must be placed and connected BY HAND; the heuristic is
   still the straight-line bound, so shortcuts cut as much as the weights allow.
   WHEN TO USE: mission maps with named locations (castle, bridge, town), any
   "points of interest" navigation, non-grid maps.
   CLASSIC QUESTION: "Grid A* vs waypoint A* — when do you pick waypoints?"
   -> when the world is naturally a graph of named locations, not a grid.
   TAKEAWAY: A* on an authored graph — nodes are places, edges are roads.
```

--------------------------------------------------------------------------------
## 16. TIME-SLICING (ASYNCHRONOUS A*)
```text
   DEFINITION: Spreading ONE pathfinding search across MANY frames so a large
   A* call (which could cost 1-2 ms and stall the frame) never blocks the game.
   HOW IT WORKS: Each frame the search is allowed a fixed number of expansions
   (the budget, e.g. max_pathfinding_steps), then it PAUSES; it resumes next
   frame with the open set, g-costs, and parent links exactly where they were.
   THE THREE PIECES: PathState (IDLE/SEARCHING/FOUND/FAILED), PathRequest
   (owns everything the search needs to pause/resume), and the NavMesh
   (startPathRequest fires it, updatePathfindingSlice burns one budget).
   WHY IT MATTERS: a blocking A* freezes that frame; time-slicing keeps the
   game running at full frame rate while the route finishes a few frames later.
   COMPLEXITY: total work identical to one A*; per-frame cost capped by budget.
   TRADE-OFFS: the path arrives slightly late (a frame or three); needs extra
   state to make the search resumable; stale requests must be ignored.
   WHEN TO USE: huge maps, many simultaneous searches, any game that must not
   hitch; pair with a JSON budget setting (§31).
   CLASSIC QUESTION: "Your AI hangs the frame when it plans — what do you do?"
   -> budget the expansions per frame and resume next frame.
   TAKEAWAY: No single frame pays for the whole search — each pays a slice.
```

--------------------------------------------------------------------------------
## 17. DISTANCE FORMULAS (4-dir / 8-dir / NavMesh)
```text
   DEFINITION: How you measure "distance" between two points changes both the
   path shape and which heuristics stay admissible for A*.
   MANHATTAN: |dx| + |dy|          — grid movement with 4 directions.
   EUCLIDEAN: sqrt(dx^2 + dy^2)    — straight-line, true shortest on NavMesh.
   OCTILE:    max(|dx|,|dy|) + (sqrt2 - 1)*min(|dx|,|dy|) — 8-dir movement.
   CHEBYSHEV: max(|dx|, |dy|)      — 8-neighbor "king moves".
   NAVMESH:   Euclidean distance between polygon centroids (3D-aware).
   WHY IT MATTERS IN INTERVIEWS: the heuristic must match the movement costs,
   and any valid straight-line metric is automatically admissible (never
   overestimates) for these grids.
   CLASSIC QUESTION: "Which heuristic for an 8-directional grid?" -> octile.
   "For a 4-directional?" -> Manhattan. "For weighted NavMesh?" -> Euclidean.
   TAKEAWAY: Pick the metric that matches your movement rules.
```

--------------------------------------------------------------------------------
## 18. HPA* (HIERARCHICAL PATHFINDING A*)
```text
   DEFINITION: A-levels-of-abstraction pathfinding: A* runs first on a small
   abstract graph, then the result is refined into a detailed path.
   HOW IT WORKS (3 levels): L1 = raw polygons; L2 = CLUSTERS (groups of
   polygons); L3 = ENTRANCES (border nodes between clusters). Precompute
   intra-cluster paths between entrances, plus an abstract entrance graph.
   At runtime A* plans on the entrance graph, then stitches L1 detail.
   COMPLEXITY/EDGE: abstract search is tiny (few entrances), so queries are
   dramatically cheaper than flat A* on huge maps.
   TRADE-OFFS: slightly suboptimal (abstract edges are approximations); adding
   a new obstacle can invalidate precomputed intra-paths (recompute cost).
   CLASSIC QUESTION: "How do you pathfind on a map too big for A*?" -> HPA*.
   TAKEAWAY: Speed through hierarchy — plan on entrances, refine on polygons.
```

--------------------------------------------------------------------------------
## 19. PATH SMOOTHING (STRING-PULLING)
```text
   DEFINITION: Post-processing that shortens a polygonal path by skipping
   intermediate nodes whenever a straight line to a farther node is VALID.
   HOW IT WORKS: Repeatedly test line-of-sight (LOS) between current and next
   candidates; if visible, skip the in-between and jump further (greedy node
   skipping). Result: a shorter path with fewer, straighter hops.
   WHY DRAMATICALLY: removes the "zig-zag through polygon centers" look; a list
   of waypoints instead of raw polygon centers.
   CLASSIC QUESTION: "Why does the AI walk in a zig-zag?" -> because it targets
   polygon centers; smoothing fixes it.
   TAKEAWAY: Greedy LOS skipping straightens a path to near-straight lines.
```

--------------------------------------------------------------------------------
## 20. FUNNEL ALGORITHM
```text
   DEFINITION: Computes the OPTIMAL smooth path through a corridor of portals
   (shared edges between consecutive polygons) by pinning a shrinking funnel.
   HOW IT WORKS: Track a left wall and right wall plus an apex (current point).
   When a new edge tightens the funnel, advance; when the walls CROSS, the apex
   is finalized and the funnel restarts there — the emitted points are the
   shortest string-pulled waypoints along the corridor.
   COMPLEXITY: O(n) per portal walk; output = minimal-corner waypoints.
   RELATION TO STRING-PULLING: string-pulling uses greedy LOS checks; the funnel
   does the same optimal job exactly and cheaply.
   CLASSIC QUESTION: "Optimal smoothing vs greedy smoothing" — funnel is exact,
   greedy is faster but can miss a shortcut.
   TAKEAWAY: A collapsing funnel that snaps straight along portals — optimal
   and O(n).
```

--------------------------------------------------------------------------------
## 21. DYNAMIC OBSTACLE MANAGEMENT
```text
   DEFINITION: Letting obstacles appear or vanish at runtime by BLOCKING
   individual polygons (or whole HPA* chunks) so pathfinding re-routes around
   them immediately.
   HOW IT WORKS: A blocked polygon flags itself; A* skips blocked nodes during
   search. When the obstruction clears (UnBlock), routes are restored. Because
   HPA* groups map into chunks, you can block a whole chunk at once.
   STRENGTHS: real-time response without rebuilding the whole NavMesh.
   TRADE-OFFS: forcing a reroute costs a new search; blocking the only corridor
   can isolate goals (no valid path — must handle "no path" gracefully).
   CLASSIC QUESTION: "How does AI react when you destroy a bridge mid-path?"
   TAKEAWAY: Block flags + A* skip = dynamic obstacles without map rebuilds.
```

### PART D — DECISION MAKING  (§22–§24)
--------------------------------------------------------------------------------
## 22. FINITE STATE MACHINE (FSM) - enemy AI
```text
   DEFINITION: Enemy behavior modeled as a set of STATES (idle, patrol, chase,
   attack, flee, dead) and TRANSITIONS triggered by EVENTS (saw player, health
   low, reached waypoint). Exactly one state is active.
   HOW IT WORKS: The FSM table says "in state X, if event Y, go to state Z".
   Entering a state performs its setup (e.g., compute a path); exiting cleans up.
   STRENGTHS: legible, cheap, deterministic, easy to debug ("you're in chase").
   TRADE-OFFS: cross-product state+event tables explode; priorities get encoded
   in transition order, which gets messy (the classic "which transition wins?"
   problem).
   PRIORITY TRICK: order transitions so lethal/low-health events are checked
   first (dead > flee > combat > patrol) — this is exactly what a Behavior Tree
   encodes structurally.
   CLASSIC QUESTION: "When would you choose an FSM over a Behavior Tree?"
   TAKEAWAY: One hot state, ruled by an ordered event table.
```

--------------------------------------------------------------------------------
## 23. BEHAVIOR TREE
```text
   DEFINITION: A tree of decision nodes ticked from the root every frame;
   SELECTOORS pick the first succeeding child (priority), SEQUENCES require all
   children to succeed in order, CONDITIONS test, ACTIONS do.
   HOW IT WORKS: root tick -> selector tries children top-down: "dead? flee?
   player seen? ..." -> the winning branch runs exactly one action that frame.
   Order in the tree literally IS priority.
   STRENGTHS over FSM: no central transition table, subtrees are reusable and
   reorderable, behavior grows by composition instead of state explosion.
   TRADE-OFFS: needs a shared blackboard for conditions; tree shape must be
   kept intentional or it becomes spaghetti.
   CLASSIC QUESTION: "Map 'if/then/else' from an FSM to a tree" — selector with
   condition guards; "SEQ1 dead->doDead, SEQ2 lowHP->doFlee..." is the pattern.
   TAKEAWAY: Behavior = priority-ordered, composable condition/action rules.
```

--------------------------------------------------------------------------------
## 24. BLACKBOARD SYSTEM
```text
   DEFINITION: A key-value memory shared by AI modules; WRITE it to publish,
   READ it to consume — modules never call each other directly.
   SCOPES: GLOBAL (world facts: player position/polygon), PERSONAL (per-agent:
   health, ammo, position, is-dead), RESERVED (internal bookkeeping).
   HOW IT INTEGRATES: the behavior tree reads conditions from the blackboard;
   the movement/state code writes results back; the demo script pushes player
   data into it (or bypasses it — see A10).
   STRENGTHS: decoupling, cheap to add facts, flexible.
   TRADE-OFFS: untyped keys, stale/missing values, invisible data dependencies.
   CLASSIC QUESTION: "How do you share state between AI modules without tangled
   dependencies?" -> a blackboard IS the standard answer.
   TAKEAWAY: A message board — write facts, read facts, keep modules decoupled.
```

### PART E — MOVEMENT & REPATHING  (§25–§30)
--------------------------------------------------------------------------------
## 25. STEERING BEHAVIORS
```text
   DEFINITION: Compute a VELOCITY/force toward or away from a target —
   SEEK (head to target at full speed), FLEE (mirror of seek, run away),
   ARRIVE (slow down and stop near the target), plus wander/pursue/evade.
   HOW IT WORKS (arrive specifically): inside a SLOWING RADIUS, scale speed by
   distance-to-target so the agent decelerates to a natural stop instead of
   overshooting.
   STRENGTHS: cheap, local, believable motion; easy to combine.
   TRADE-OFFS: local only — no global planning (pair with A* for that).
   CLASSIC QUESTION: "Seek vs arrive" — seek never slows; arrive decelerates
   within the slowing radius.
   TAKEAWAY: Force vectors that blend into motion; arrive = seek + a brake.
```

--------------------------------------------------------------------------------
## 26. STEERING FORCE BLENDING (CHASE = seek + flock)
```text
   DEFINITION: Combine several force vectors by WEIGHTED SUM so multiple goals
   (follow a path, avoid a crowd) merge into one velocity change.
   THE RECIPE: force = w1*seek + w2*separation + w3*alignment + w4*cohesion;
   clamp/truncate total speed to a max, add to velocity, integrate position,
   then apply DAMPING (friction) so motion settles.
   CONTEXT-SWITCHED WEIGHTS: a chasing agent blends seek + full flocking; a
   fleeing agent uses arrive (flocking off); an idle agent is flocking-only.
   WHY NEIGHBORS: agents only sense neighbors within a radius — locality makes
   the blend cheap and panic-free.
   CLASSIC QUESTION: "What makes AI avoid each other while still chasing?" ->
   weighted blending of seek + separation forces.
   TAKEAWAY: Weighted-sum forces, clamped and damped, shaped by the current
   state.
```

--------------------------------------------------------------------------------
## 27. THE PER-FRAME PIPELINE (update vs updateMovement)
```text
   DEFINITION: Each agent frame splits into two decoupled halves —
   DECISION: pick the intent (which state, where to go, possibly replan a path);
   MOTION: turn that target into movement via steering forces.
   HOW IT WORKS: e.update() writes blackboard, repath bookkeeping, runs the
   behavior tree, and overwrites the planned path. Then the caller picks the
   concrete steering target (chase = player position, otherwise current polygon
   center) and calls updateMovement() which uses steering + flocking + damping.
   WHY IT MATTERS: the two halves are independent — you can change "where to
   aim" without touching "how to move", and vice versa.
   CLASSIC QUESTION: "Why run the behavior tree separate from movement?"
   TAKEAWAY: Decide first, move second, keep the two decoupled.
```

--------------------------------------------------------------------------------
## 28. PATROL WAYPOINT LOOP
```text
   DEFINITION: A predefined cyclic sequence of points (here: polygon 0-1-2-3)
   an idle agent walks forever, wrapping back to the first after the last.
   HOW IT WORKS: currentPathIndex walks the waypoints; reaching the end wraps
   to index 0 (and a new loop cycle starts), giving predictable coverage of an
   area.
   WHY IT EXISTS: low-cost, deterministic, believable "guarding" behavior that
   needs no player info.
   CLASSIC QUESTION: "How do you guard an area cheaply?" -> cyclic waypoints.
   TAKEAWAY: Walk the list, wrap at the end, repeat forever.
```

--------------------------------------------------------------------------------
## 29. WAYPOINT ADVANCE / ARRIVAL THRESHOLD
```text
   DEFINITION: An agent "arrives" at a waypoint when its distance to it is
   below a threshold, at which point it ADVANCES to the next waypoint and the
   path index moves on; the last reached polygon is remembered as the
   "last valid polygon".
   HOW IT WORKS: maintain currentPathIndex; on arrival increment it; when it
   passes the end, the path is exhausted -> fall back to the last valid polygon
   and trigger a replan. The remembered node keeps getCurrentPolygon() sane
   whenever the path is empty.
   THE 2D-vs-3D TRAP: chase uses 2D distance (ignores z: works); patrol/flee use
   3D distance against polygon centers whose z differs from the agent's locked
   z -> arrival can never fire (see A10). Dimension mismatch silently breaks
   the advance loop.
   CLASSIC QUESTION: "Your AI never reaches its waypoint — why?" Check the
   distance dimension and what you're measuring against.
   TAKEAWAY: Arrival threshold + index advance; mismatch between 2D/3D checks
   quietly stalls the whole loop.
```

--------------------------------------------------------------------------------
## 30. REPATH TIMER & STICKY TARGETS
```text
   DEFINITION: Throttled replanning: the agent re-runs A* only when it has no
   path OR (the throttle timer expired AND the target actually moved).
   WHY IT'S NEEDED: replanning every frame is wasteful; re-planning into a
   moving target repeatedly produces churn. Gating by "did the goal move?"
   prevents replanning at static targets.
   CONDITION: repath if hasNoPath OR (timerExpired AND playerMoved).
   THE PRE-ARMED TRICK: starting the timer expired ("pre-armed") forces a
   replan on the FIRST chase frame — intentional, so the path is fresh.
   STICKY TARGET EFFECT: a stationary player -> the agent legitimately keeps
   its current plan; the "sticky" path persists while the goal doesn't move.
   CLASSIC QUESTION: "How often should AI repath?" -> only when needed.
   TAKEAWAY: `no-path OR (stale timer AND moved goal)` gates every replan.
```

### PART F — DEMO SIMULATION & MEMORY  (§31–§33)
--------------------------------------------------------------------------------
## 31. DATA-DRIVEN CONFIGS (JSON)
```text
   DEFINITION: Moving tuning numbers OUT of the C++ source and INTO a JSON file
   (health, speeds, detection ranges, budgets, patrol routes) loaded once at
   startup with nlohmann/json — tweak a number, no recompile.
   HOW IT WORKS: loadEnemyConfig(filename, type) opens the file, parses it,
   drills into nested objects by dotted key path (data["enemy_types"]["tank"]),
   and extracts typed values with .get<float>() — with hard-coded fallbacks if
   the file or the type is missing (bad files never crash the demo).
   THE SHAPE: "system" (budgets & tuning: max_pathfinding_steps, repath
   cooldown, radii, damage, threshold), "spawn" (player start, patrol route,
   enemy list), "enemy_types" (per-archetype stats: grunt/tank/scout).
   WHY DATA-DRIVEN: balance becomes editing JSON instead of editing code —
   "add a new archetype" = add a JSON entry, zero code changes; one loader
   builds every enemy from a different value set.
   TRADE-OFFS: load-time coupling to the file format; untyped/nested access is
   easy to get wrong (guards like contains()/is_null() catch it); a missing key
   silently falls back to defaults.
   CLASSIC QUESTION: "How do you balance your game without recompiling?"
   -> runtime-loaded config files, validated with fallbacks.
   TAKEAWAY: Numbers live in files, code reads them once, behavior follows.
```

--------------------------------------------------------------------------------
## 32. THE 5 SIMULATION PHASES (main() end-to-end timeline)
```text
   DEFINITION: A scripted end-to-end timeline that exercises every subsystem in
   order: PATROL -> DETECT -> BLOCK -> COMBAT -> ESCAPE.
   PH1 PATROL   agents walk their waypoint loop; no player interaction.
   PH2 DETECT   player enters range; enemies switch to CHASE and repath.
   PH3 BLOCK    a corridor polygon is dynamically blocked mid-path -> reroute.
   PH4 COMBAT   enemies attack; a self-damage bug drains health 25/tick
                (100->75->50->25) until low health forces FLEE.
   PH5 ESCAPE   obstruction removed; fleeing agents return and recover.
   WHAT IT SHOWS: every layer (decision, nav, motion, dynamic obstacles,
   memory) interacting under one driver loop.
   CLASSIC QUESTION: "Walk me through your demo end to end" -> blow-by-blow that
   maps states to frames.
   TAKEAWAY: A narrative that proves each subsystem AND its integration works.
```

--------------------------------------------------------------------------------
## 33. MEMORY MAP & OBJECT LIFECYCLE (heap, RAII, ownership)
```text
   DEFINITION: Who OWNS each object (who is responsible for deleting it), where
   it lives (STACK = automatic, fast; HEAP = manual, explicit), and what makes
   ownership safe (RAII: tie cleanup to destruction).
   THE MAP: the 4 enemies and every behavior-tree node are on the HEAP (new);
   the NavMesh and global blackboard live on the STACK and are handed out as
   REFERENCES (borrowed). An agent OWNS its tree's root; the tree owns its
   children; "allEnemies" holds borrowed aliases of the same enemies.
   THE HAZARD: a COPY of the enemy would delete the same root twice (double-
   delete crash) — exactly why raw owning pointers are dangerous.
   THE MODERN FIX: rule of three/five, or std::unique_ptr for single ownership,
   raw pointers/refs for borrowing only.
   CLASSIC QUESTION: "What is the Rule of Three / why double-delete happens?"
   TAKEAWAY: Exactly one owner per resource; everyone else borrows. RAII runs
   cleanup automatically.
```

### PART G — ARCHITECTURE GLOSSARY (A1–A10)

*the concepts behind the ASCII_ART big-picture diagram*

--------------------------------------------------------------------------------
## A1. NAVMESH (Navigation Mesh)
```text
   DEFINITION: A graph representation of walkable space for AI movement: the map
   is split into convex polygons, connected along shared edges (portals).
   HOW IT WORKS: Each polygon has a center (centroid); neighbors are connected
   with weighted edges. Pathfinding (A*) runs over polygons, not pixels.
   STRENGTHS: cheap to pathfind, supports dynamic blocking per polygon.
   TRADE-OFFS: pre-built cost; a polygon can be blocked, forcing replanning.
   CLASSIC QUESTION: "How would you make AI walk around a real game map?"
   TAKEAWAY: NavMesh turns continuous space into a small weighted graph.
```

--------------------------------------------------------------------------------
## A2. POLYGON CENTROID
```text
   DEFINITION: The 3D center point of a convex polygon; used as the default
   steering target when an enemy walks toward a polygon.
   TAKEAWAY: The concrete "goal position" the movement system aims at.
```

--------------------------------------------------------------------------------
## A3. FSM (Finite State Machine)
```text
   DEFINITION: A model where the agent is in exactly one of a fixed set of
   STATES, and moves between them along TRANSITIONS triggered by EVENTS.
   HOW IT WORKS: States = behavior modes (idle, patrol, chase, attack...).
   Transitions = rules ("health low -> flee"). One active state at a time.
   STRENGTHS: simple, readable, predictable.
   TRADE-OFFS: state explosion when behavior grows; transitions get tangled.
   TAKEAWAY: Great for small, fixed-behavior agents; brittle at scale.
```

--------------------------------------------------------------------------------
## A4. BEHAVIOR TREE (BT)
```text
   DEFINITION: A hierarchical decision structure where a ROOT node is "ticked"
   each frame and children (Selectors, Sequences, Conditions, Actions) decide
   the single action to run.
   HOW IT WORKS: Selectors try children in priority order and stop at the first
   one that succeeds (like if/else-if). Sequences run children in order and stop
   at the first failure. Conditions gate, Actions execute.
   STRENGTHS: modular, reusable subtrees, no explicit transition graph;
   it composes cleanly and is the modern game-AI standard.
   TRADE-OFFS: more structure to learn than an FSM; needs a blackboard.
   CLASSIC QUESTION: "FSM vs Behavior Tree — which and why?"
   TAKEAWAY: Order = priority => complex behavior from simple composable pieces.
```

--------------------------------------------------------------------------------
## A5. BLACKBOARD
```text
   DEFINITION: A shared data store (key-value memory) that lets AI modules
   exchange information without knowing about each other.
   HOW IT WORKS: Producers WRITE keys (e.g., enemy health, player position);
   consumers READ them. Often split into GLOBAL (shared), PERSONAL (per-agent),
   and RESERVED (internal) scopes.
   STRENGTHS: decouples decision logic from data; cheap to extend.
   TRADE-OFFS: no type safety without effort; stale or uninitialized keys are a
   classic bug ("nothing wrote PLAYER_POSITION, read side still reads").
   TAKEAWAY: The nervous system that lets the behavior tree see the world.
```

--------------------------------------------------------------------------------
## A6. STEERING BEHAVIORS
```text
   DEFINITION: Local, velocity-based force rules that steer an agent toward a
   goal (seek, arrive) or away from danger (flee), usually combined.
   HOW IT WORKS: Each behavior outputs a force vector; forces are blended
   (weighted sum) and then integrated into position/velocity.
   TAKEAWAY: The "motion half" — turns big-picture decisions into movement.
```

--------------------------------------------------------------------------------
## A7. FLOCKING
```text
   DEFINITION: Crowd behavior from three local rules — SEPARATION (don't crowd),
   ALIGNMENT (match neighbors' heading), COHESION (stay near the group).
   HOW IT WORKS: Each agent only looks at neighbors within a radius; forces are
   weighted and combined with the current steering behavior.
   TAKEAWAY: Simple local rules produce believable group motion.
```

--------------------------------------------------------------------------------
## A8. REPATH TIMER / STICKY TARGET
```text
   DEFINITION: Instead of replanning every frame, the agent replans whenever a
   throttle timer expires AND the goal actually moved — otherwise it keeps its
   old path ("sticky target").
   WHY: replanning every frame is wasteful; only a moved goal changes the plan.
   CLASSIC QUESTION: "Why doesn't your AI recalculate its path every frame?"
   TAKEAWAY: Replan GATED by (timer expired AND goal moved), plus on lost path.
```

--------------------------------------------------------------------------------
## A9. HPA* (Heirarchical Pathfinding A*)
```text
   DEFINITION: A* on abstracted, higher-level maps (clusters of polygons and
   entrance nodes between them) to make large-map pathfinding fast.
   HOW IT WORKS: Precompute intra-cluster paths; ask A* over the abstract
   entrance graph; stitch the detail path back together at runtime.
   TRADE-OFFS: fast, scalable; slightly less optimal than flat A*.
   TAKEAWAY: Speed by planning first on a map of maps.
```

--------------------------------------------------------------------------------
## A10. Z-LOCK CAVEAT (from the project)
```text
   DEFINITION: A demo bug where enemies are pinned to z = 0.5 on the ground
   while every polygon center sits at z >= 5, so all 3D distance checks that
   compare against polygon centers can never succeed.
   WHY INTERVIEWERS LIKE IT: it explains why distance dimension matters —
   2D vs 3D distance is not interchangeable, and a locked axis changes which
   "arrival" checks can ever fire.
   TAKEAWAY: Always ask "is this distance 2D or 3D, and against what point?"
```

# END OF CHEATSHEET
