<a id="top"></a>
<a href="#top" title="Back to top" style="position: fixed; bottom: 18px; right: 18px; display: inline-block; background: #2e6b8f; color: #fff; border-radius: 50%; width: 42px; height: 42px; line-height: 42px; text-align: center; font-size: 22px; text-decoration: none; box-shadow: 0 2px 6px rgba(0,0,0,.35); z-index: 999;">&#8593;</a>


```
                        DATA STRUCTURES - ASCII ART GUIDE
              Topics by Mustansir
--------------------------------------------------------------------------------
  READ THIS FIRST
  * This file is a STUDY GUIDE, not an exact line-by-line trace of the code.
    A few variants (Circular DLL, insert-before/after) are added here to give
    the full picture even though they are not built in start.cpp.
  * Sections match the learning order: first the data structure, then its
    operations, each drawn as Before / After diagrams.

  DEMO CAVEATS (read before the AI sections 17-32)
  * The enemy's z is locked to 0.5 in updateMovement, but every polygon center
    sits at z=5, 8 or 10.  So the 3D arrival checks in updatePatrol (dz>=4.5)
    and updateFlee can NEVER pass in a real run: patrol legs never advance and
    flee never heals at polygon 0.  Only CHASE can "arrive" (2D check).
    Sections 24/26/28 describe the INTENDED design, not the runnable demo.
  * During PATROL the steering is flocking-only: the enemy does not actually
    walk toward the patrol target that main() passes in.
  * DEAD is unreachable in the demo: isHealthLow fires at <30 and health never
    hits 0, so the isDead branch never wins.
================================================================================
```

> **TIP: Click any topic in the index below to jump straight to that section.**

| # | **Topic** | # | **Topic** |
|---|-----------|---|-----------|
| 1 | [SINGLY LINKED LIST (SLL)](#1-singly-linked-list-sll) | 17 | [DISTANCE FORMULAS  (4-dir / 8-dir / NavMesh)](#17-distance-formulas--4-dir--8-dir--navmesh) |
| 2 | [DOUBLY LINKED LIST (DLL)](#2-doubly-linked-list-dll) | 18 | [HPA*  (HIERARCHICAL PATHFINDING A*)](#18-hpa--hierarchical-pathfinding-a) |
| 3 | [CIRCULAR DOUBLY LINKED LIST (CDLL)](#3-circular-doubly-linked-list-cdll) | 19 | [PATH SMOOTHING (STRING-PULLING)](#19-path-smoothing-string-pulling) |
| 4 | [SINGLY CIRCULAR LINKED LIST (SCLL)](#4-singly-circular-linked-list-scll) | 20 | [FUNNEL ALGORITHM](#20-funnel-algorithm) |
| 5 | [LINKED STACK (LIFO - Last In First Out)](#5-linked-stack-lifo---last-in-first-out) | 21 | [DYNAMIC OBSTACLE MANAGEMENT](#21-dynamic-obstacle-management) |
| 6 | [ARRAY STACK (Static / Fixed Size)](#6-array-stack-static--fixed-size) | 22 | [FINITE STATE MACHINE (FSM)  -  enemy AI](#22-finite-state-machine-fsm-----enemy-ai) |
| 7 | [LINKED QUEUE (Dynamic - FIFO)](#7-linked-queue-dynamic---fifo) | 23 | [BEHAVIOR TREE](#23-behavior-tree) |
| 8 | [CIRCULAR ARRAY QUEUE (Static + Circular)](#8-circular-array-queue-static--circular) | 24 | [BLACKBOARD SYSTEM](#24-blackboard-system) |
| 9 | [MAX-HEAP (Binary Heap - Array Based)](#9-max-heap-binary-heap---array-based) | 25 | [STEERING BEHAVIORS](#25-steering-behaviors) |
| 10 | [BINARY SEARCH TREE (BST)](#10-binary-search-tree-bst) | 26 | [STEERING FORCE BLENDING  (CHASE = seek + flock)](#26-steering-force-blending--chase--seek--flock) |
| 11 | [TRIE (PREFIX TREE)](#11-trie-prefix-tree) | 27 | [THE PER-FRAME PIPELINE  (update vs updateMovement)](#27-the-per-frame-pipeline--update-vs-updatemovement) |
| 12 | [GRAPH (ADJACENCY LIST + BFS/DFS)](#12-graph-adjacency-list--bfsdfs) | 28 | [PATROL WAYPOINT LOOP](#28-patrol-waypoint-loop) |
| 13 | [DIJKSTRA (SHORTEST PATH - WEIGHTED GRAPH)](#13-dijkstra-shortest-path---weighted-graph) | 29 | [WAYPOINT ADVANCE / ARRIVAL THRESHOLD](#29-waypoint-advance--arrival-threshold) |
| 14 | [A* PATHFINDING](#14-a-pathfinding) | 30 | [REPATH TIMER & STICKY TARGETS](#30-repath-timer--sticky-targets) |
| 15 | [A* WITH WAYPOINTS (WEIGHTED NODES)](#15-a-with-waypoints-weighted-nodes) | 31 | [DATA-DRIVEN CONFIGS (JSON)](#31-data-driven-configs-json) |
| 16 | [TIME-SLICING (ASYNCHRONOUS A*)](#16-time-slicing-asynchronous-a) | 32 | [THE 5 SIMULATION PHASES  (main() end-to-end timeline)](#32-the-5-simulation-phases--main-end-to-end-timeline) |
| 33 | [MEMORY MAP & OBJECT LIFECYCLE (heap, RAII, ownership)](#33-memory-map--object-lifecycle-heap-raii-ownership) |  | |

```

                 SYSTEM ARCHITECTURE OVERVIEW
              (maps to sections in this guide)

   DECISION → NAVIGATION → MOVEMENT   =  one enemy per frame (§27)
   COMMUNICATION (blackboard) + SIMULATION script (main) + INFRASTRUCTURE

┌──────────────────────────────────────────────────────────────────────────────┐
│  INPUT / SETUP          ·  built ONCE in main() (§31/§33)                    │
│                                                                              │
│  ┌──────────────────────────────┐  ┌──────────────────────────────┐          │
│  │  ENEMY FACTORY               │  │  WORLD STATE                  │          │
│  │  4 x Enemy · health 100      │  │  NavMesh: 4 polygons          │          │
│  │  spawn (5,5) (8,6) (4,9)     │  │           5 weighted edges    │          │
│  │         (9,4)                │  │  centroids (3D, §17)          │          │
│  │  velocity 0 · personal BB    │  │  patrolPath {0,1,2,3} (§28)   │          │
│  │  ammo 12                    │  │  player @ (40,40,10) far       │          │
│  └──────────────────────────────┘  └──────────────────────────────┘          │
│                                                                              │
│  Enemy stats (health/ammo/speed) come from JSON config, loaded in            │
│  main() via loadEnemyConfig()  (§31).  Player position set directly via      │
│  setPlayerPosition() — the global blackboard (PLAYER_POSITION key) is        │
│  NEVER filled in the demo (§24/§27).  HPA* one-time init (before spawn):     │
│  addCluster -> initializeHPA() -> findEntrances -> computeIntraPaths  (§18). │
└──────────────────────────────────────────────────────────────────────────────┘
            │
            ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  DECISION LAYER  ·  e->update() each frame  (§22, §23, §24)                  │
│                                                                              │
│  ┌──────────────────────────────┐  ┌──────────────────────────────────┐      │
│  │  FSM  (§22, 6 states)        │  │  BEHAVIOR TREE (§23 L1089-1108)   │      │
│  │                              │  │  SELECTOR = if/else-if chain      │      │
│  │  IDLE·PATROL·CHASE           │  │   ├ SEQ1 isDead  -> doDead -> DEAD│      │
│  │  ATTACK·FLEE·DEAD            │  │   ├ SEQ2 isLowHP -> doFlee -> FLEE│      │
│  │                              │  │   │      (health<30)             │      │
│  │  PATROL<->CHASE<->ATTACK     │  │   ├ SEQ3 playerSeen (dist<20)    │      │
│  │  any->FLEE  (health<30)      │  │   │    ├ SEQ inRange->doAttack   │      │
│  │  any->DEAD  (health<=0)      │  │   │    └ doChase      -> CHASE   │      │
│  │  FLEE->PATROL (reach poly 0) │  │   └ doPatrol (fallback)-> PATROL │      │
│  │                              │  │  Priority: Dead>Flee>Combat      │      │
│  └──────────────────────────────┘  └──────────────────────────────────┘      │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐      │
│  │  BLACKBOARD (§24)   unordered_map< key, variant< int,float,bool,     │      │
│  │  Vector3,string > > — keys written each tick, nothing read back      │      │
│  │  in the demo (see §24 notes).                                        │      │
│  │  GLOBAL: PLAYER_POSITION · PLAYER_POLYGON      (read side, unused)   │      │
│  │  PERSONAL: ENEMY_POSITION · ENEMY_HEALTH · ENEMY_AMMO · IS_DEAD      │      │
│  │  RESERVED: IS_IN_COMBAT · ALLY_HEALTH · TARGET_POLYGON              │      │
│  └────────────────────────────────────────────────────────────────────┘      │
│                                                                              │
│  Output: ONE state change + possibly a freshly planned path stored in        │
│  currentPath.  main() then picks the steering target for the motion half.    │
└──────────────────────────────────────────────────────────────────────────────┘
            │
            ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  NAVIGATION LAYER  ·  planning during update()  (§14-§16, §18-§21)           │
│                                                                              │
│  ┌──────────────────────────────┐  ┌──────────────────────────────────┐      │
│  │  ROUTING                     │  │  PATH REFINEMENT                  │      │
│  │  A*  (§14 line 676)          │  │  smoothPath (§19 L776-789)        │      │
│  │   f = g + h, min-heap        │  │   LOS check (line 772)            │      │
│  │   stale-entry lazy delete    │  │   greedy node-skip               │      │
│  │  waypoint A* (§15)           │  │   used by patrol AND chase        │      │
│  │   f = g + h on waypoints     │  │  funnelAlgorithm (§20 L859-895)   │      │
│  │  time-sliced A* (§16)        │  │   portals = shared edges (§20)    │      │
│  │   search split over frames   │  │   PATROL only -> DISCARDED (§28)  │      │
│  │  HPA* (§18 L371-606)         │  │  dynamic: BlockChunks /           │      │
│  │   L1 polys -> L2 clusters    │  │            UnBlockChunks (§21)    │      │
│  │   -> L3 entrance abstract    │  │                                   │      │
│  │   graph + intra-path cache   │  │                                   │      │
│  │  obstacles (§21 L648-674)    │  │                                   │      │
│  │   isBlocked -> A* skips node │  │                                   │      │
│  └──────────────────────────────┘  └──────────────────────────────────┘      │
│                                                                              │
│  Distance = Euclidean 3D between centroids (§14/§17) -> admissible.          │
│  Arrival checks: 2D (chase) vs 3D (patrol/flee) — see §29 caveat.            │
└──────────────────────────────────────────────────────────────────────────────┘
            │
            ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  FOLLOW + REPATH  ·  the middle step: path -> motion  (§30, §29)            │
│                                                                              │
│  each frame: next = currentPath[currentPathIndex]   (L1195-1198)            │
│  arrive? dist < threshold -> index++, lastValidPolygonId = that node        │
│   threshold: 2D in CHASE (L1284)   3D in PATROL/FLEE (L1240/1324)        │
│  index >= size -> hasNoPath -> getCurrentPolygon() = lastValidPolygonId    │
│                                                                              │
│   REPATH GATE  (§30 L1256-1276):  repath if hasNoPath OR (timerExpired      │
│        AND playerMoved)   sticky: stationary player -> no repath            │
│   cooldown 0.5 pre-armed -> first CHASE frame ALWAYS repaths (L1425)         │
│                                                                              │
│  on repath: aStar -> smoothPath -> index = 0 -> timer = 0                   │
│  then main() reads getCurrentPolygon() and picks the steering target        │
└──────────────────────────────────────────────────────────────────────────────┘
            │
            ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  MOVEMENT LAYER  ·  e->updateMovement(target, dt)  (§25, §27, §26)           │
│                                                                              │
│  ┌──────────────────────────────┐  ┌──────────────────────────────────┐      │
│  │  STEERING (§25)              │  │  FLOCKING (§25, §26)              │      │
│  │  seek   = desired - velocity│  │  separation x 1.5 (push apart)    │      │
│  │  flee   = -seek (run away)   │  │  alignment  x 1.0 (match heading) │      │
│  │  arrive = ramp speed to 0    │  │  cohesion   x 1.0 (stay together) │      │
│  │  at slowingRadius (5)        │  │  neighborRadius = 15            │      │
│  │  speed = 2.0 · damping 0.85  │  │  CHASE: seek + flock            │      │
│  │  dt = 0.5 · z = 0.5 locked   │  │  FLEE: arrive (flock OFF)       │      │
│  │                              │  │  other: flock only               │      │
│  └──────────────────────────────┘  └──────────────────────────────────┘      │
│  ┌────────────────────────────────────────────────────────────────────┐      │
│  │  INTEGRATION: vel += steer -> truncate(2.0) -> pos += vel*0.5     │      │
│  │  -> z = 0.5 (ground clamp) -> vel *= 0.85 (damping)  (§27, §26)   │      │
│  └────────────────────────────────────────────────────────────────────┘      │
└──────────────────────────────────────────────────────────────────────────────┘
            │
            ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  SIMULATION SCRIPT  ·  main() drives the whole demo  (§32)                   │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐      │
│  │  PH1 patrol(1-3) -> PH2 detect(4-7) -> PH3 block(8-10)              │      │
│  │  player(40,40)->(15,15)     BlockChunks({1}) reroute via 0-3 (37)   │      │
│  │  -> PH4 combat(11-39)  ->  PH5 escape(40-44)                        │      │
│  │  combat: enemies ATTACK, self-damage 25/tick (bug, line 1295)       │      │
│  │          100->75->50->25 -> FLEE   (DEAD unreachable — §22 caveat)  │      │
│  │  escape: UnBlockChunks({1}) -> flee to poly 0 -> heal -> PATROL     │      │
│  │          (intended design; z-lock stops it live — DEMO CAVEATS)     │      │
│  └────────────────────────────────────────────────────────────────────┘      │
└──────────────────────────────────────────────────────────────────────────────┘
            │
            ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  INFRASTRUCTURE  ·  memory & ownership  (§33)                                 │
│  ┌──────────────────────────────┐  ┌──────────────────────────────────┐      │
│  │  MEMORY                      │  │  OWNERSHIP / hazards              │      │
│  │  4 Enemies on heap (new)     │  │  main -> Enemy                    │      │
│  │  BT nodes on heap            │  │  Enemy -> root Selector           │      │
│  │  nav, gBb on stack (ref)     │  │  Selector/Seq -> children         │      │
│  │  personalBB by value         │  │  allEnemies = non-owning aliases  │      │
│  │  ~Enemy deletes its root     │  │  copying Enemy -> double delete  │      │
│  │                              │  │  modern fix -> unique_ptr (§33)  │      │
│  └──────────────────────────────┘  └──────────────────────────────────┘      │
└──────────────────────────────────────────────────────────────────────────────┘

   ┌────────────────────────────────────────────────────────────────────────┐
   │  PER-FRAME DATA FLOW  (one enemy, one tick — §27)                      │
   │                                                                        │
   │  e->update(dt)                DECISION HALF (L1408-1429)                │
   │    · write personal BB        · read shared BB (if filled)              │
   │    · repath timer book-keeping (§30)   · runBT() -> action + state      │
   │                                  │                                     │
   │      main() picks target: CHASE -> player pos | other -> poly center  │
   │                                  ▼                                     │
   │  e->updateMovement(target, dt)  MOTION HALF (L1367-1402)              │
   │    · neighbors within 15 (§26)  · blend forces per state (§26)         │
   │    · velocity += steer -> truncate -> integrate -> damp                │
   │    · z = 0.5 (ground clamp)                                           │
   └────────────────────────────────────────────────────────────────────────┘

================================================================================


```
## THE COMBINED ASCII ART  (every section on one canvas)

```
   .==============================================================================================.
   |                  THE COMPLETE PICTURE  ·  ALL 33 SECTIONS ON ONE CANVAS                      |
   |         (fused from the per-section diagrams  ·  layered like the SYSTEM ARCHITECTURE)       |
   '=============================================================================================='

   ┌─────────────────────────────  LINEAR DATA STRUCTURES  ·  1 - 8  ─────────────────────────────┐
   │1 SLL      head→[100|*]→[200|*]→[300|*]→NULL           3 CDLL               +------+          │
   │insert:    prev→next = curr→next              [100]←≠→[200]←≠→[300]         tail→next=head    │
   │delete:    patch around → delete              (no nullptr, both-way ring)   head→prev=tail    │
   │2 DLL      NULL←[100]↔[200]↔[300]→NULL        4 SCLL     [100]→[200]→[300]→(back to 100)      │
   │forward = follow next · reverse = prev        tail→next = head · do-while display             │
   │                                                                                              │
   │5 LINKED STACK: top→[400|*]→[100|*]→NULL      6 ARRAY STACK: arr[1,2,3,4,5]  top=4   LIFO     │
   │push/pop at HEAD (same as push_front)         overflow at cap · underflow at top=-1           │
   │                                               (balanced-brackets app)                        │
   │7 LINKED QUEUE: front→[100]→[200]→[300]→NULL                                                  │
   │enqueue at REAR · dequeue at FRONT   FIFO                                                     │
   │8 CIRCULAR ARRAY QUEUE: 0→1→2→3→4 then (4+1)%5=0 → front+rear walk ring, mod-wrap             │
   └──────────────────────────────────────────────────────────────────────────────────────────────┘
   │                                                                                              │
   │                                              ▼                                               │
   ┌─────────────────────────────────────  TREES  ·  9 - 11  ─────────────────────────────────────┐
   │9 MAX-HEAP                10 BST                   11 TRIE (prefix tree)                      │
   │        [25]                  [50]                     ROOT                                   │
   │       /    \                /      \                  | (c)                                  │
   │    [20]    [10]          [30]      [60]               v                                      │
   │    /  \                  /             \          [c]→[a]→[t|end]  "cat"                     │
   │ [5]   [15]            [25]            [55]                └→[r|end]  "car"                   │
   │parent ≥ child          left<root<right                 shared prefix "ca"                    │
   │bubble-up / down        in-order → sorted               isEndOfWord = whole word              │
   └──────────────────────────────────────────────────────────────────────────────────────────────┘
   │                                                                                              │
   │                                              ▼                                               │
   ┌─────────────────────────────  GRAPHS + PATHFINDING  ·  12 - 16  ─────────────────────────────┐
   │12 GRAPH    adj[0]→[1]→[2]→NULL        f(n) = g(n) + h(n)                                     │
   │BFS = queue, layer by layer            g = exact cost so far                                  │
   │DFS = recursion, deep dive             h = estimate to goal (admissible)                      │
   │                                                                                              │
   │13 DIJKSTRA        14 A* ON THE MESH    15 WAYPOINT A*       16 TIME-SLICING                  │
   │  [0]--2--[1]       [2]──15──[3]        Castle(0,0)--7-->    PathRequest{ pq, gcost,          │
   │  |       |  (w=3)  | 10  ╲37  | 10      Camp(5,5)--7--+      parents, FinalPath, s }         │
   │  [2]--1--[3]--5--[4][0]--10--[1]       Town(40,10) ⚠ goal   slice ≤ 50 expands/frame         │
   │pq pops smallest      h = 3D Euclidean   f = g+h curve       SEARCHING → FOUND                │
   │0→5 = 12              [0,1,3] cost 20    [0,1,4] cost 20     spread over frames               │
   └──────────────────────────────────────────────────────────────────────────────────────────────┘
   │                                                                                              │
   │                                              ▼                                               │
   ┌───────────────────────────  HEURISTICS + REFINEMENT  ·  17 - 21  ────────────────────────────┐
   │17 DISTANCE       18 HPA*  (3 layers)   19 STRING-PULLING     20 FUNNEL                       │
   │  Manhattan         L3 entrances  E0-E1-E2    [0]--[1]--[3]    apex ●  ╲ │ ╱                  │
   │  |dx|+|dy|(+|dz|)  L2 clusters {0,1} {2,3}   LOS(0,3)=true     cone  ├─├─┤ portals           │
   │  Octile:           L1 polygons + intra-      ⇒ path → [0,3]    walls ├─┼─┤ squeeze           │
   │  max+0.414·min     costs cached at load      greedy farthest   cross ⇒ ● waypoint            │
   │  Euclidean √(Σd²)  query = abstract A* on     visible jump                                   │
   │  NavMesh centroid  tiny entrance graph                                                       │
   │                                                                                              │
   │21 DYNAMIC OBSTACLES   unordered_set<int> BlockedPolygons { 1 }                               │
   │A* neighbor loop: if isBlocked(next) continue                                                 │
   │BlockChunks({1}) reroutes 0→3 direct, cost 37                                                 │
   └──────────────────────────────────────────────────────────────────────────────────────────────┘
   │                                                                                              │
   │                                              ▼                                               │
   ┌──────────────────────────────  AI DECISION LAYER  ·  22 - 24  ───────────────────────────────┐
   │22 FSM                23 BEHAVIOR TREE (root)         24 BLACKBOARD                           │
   │                                                                                              │
   │PATROL──►CHASE        ┌──── SELECTOR ────┐            sharedBB (read)   ▲                     │
   │CHASE ──►ATTACK       │ 1:SEQ1 | 2:SEQ2 │             PLAYER_POSITION    │                    │
   │CHASE ──►FLEE (hp<30) │ 3:SEQ3 | 4:act  │        ┌────┴────┬────┬────┬───┘                    │
   │any   ──►DEAD (hp≤0)  │ dead|flee|combat│        │ Enemy1 | Enemy2 | Enemy3 |                 │
   │FLEE ──►PATROL (0)    └── always ──────┘        │ personalBB (WRITE)        │                 │
   │                       fallback = patrol        │ POSITION · HEALTH · AMMO   │                │
   │                       SEQ1 isDead→DEAD         │ IS_DEAD · variant<...>    │                 │
   └──────────────────────────────────────────────────────────────────────────────────────────────┘
   │                                                                                              │
   │                                              ▼                                               │
   ┌─────────────────────────────  MOVEMENT + WAYPOINTS  ·  25 - 30  ─────────────────────────────┐
   │25 STEERING  seek = desired − velocity · flee = −seek · arrive(slowingRadius=5)               │
   │flock = separation·1.5 + alignment·1.0 + cohesion·1.0 (neighborRadius 15)                     │
   │26 BLENDING  CHASE = seek + flock · FLEE = arrive (flock OFF) · other = flock only            │
   │velocity = truncate(v,2.0) → pos += v·0.5 → z = 0.5 → v *= 0.85                               │
   │                                                                                              │
   │27 PER-FRAME PIPELINE                                                                         │
   │e->update(dt)  [BB write + BB read + repath timer + runBT()]  = DECISION                      │
   │main() picks target  (CHASE→player, other→polygon center)                                     │
   │e->updateMovement(target,dt) [neighbors + blend + integrate + damp] after = MOTION            │
   │                                                                                              │
   │28 PATROL LOOP   {0,1,2,3}→wrap: 0→1→2→3→0   per leg A*→smooth→funnel(dropped)                │
   │29 ADVANCE       walk currentPath[i] center · dist<0.5 ⇒ index++ · 2D/3D check                │
   │30 REPATH        timer(0.5)+playerMoved ⇒ repath · pre-armed ⇒ first frame                    │
   │always repaths; sticky target (player still) ⇒ follows the old path                           │
   └──────────────────────────────────────────────────────────────────────────────────────────────┘
   │                                                                                              │
   │                                              ▼                                               │
   ┌────────────────────────────────────  SYSTEM  ·  31 - 33  ────────────────────────────────────┐
   │31 JSON CONFIGS       32 THE 5 PHASES (main)       33 MEMORY / OWNERSHIP                      │
   │enemy_configs.json    PH1 patrol (1-3)              main() ──► 4 × new Enemy                  │
   │→ loadEnemyConfig()   PH2 detect (4-7)              Enemy ──► root Selector                   │
   │→ .get<T> by key      PH3 block (8-10)              Selector ──► children                     │
   │→ defaults on missing PH4 combat (11-39, self-dmg)  borrow = &ref / non-owning                │
   │balance in data,      PH5 escape (40-44,           RAII fix = unique_ptr                      │
   │not in C++ source      heal → patrol)                                                         │
   └──────────────────────────────────────────────────────────────────────────────────────────────┘

   DECISION → NAVIGATION → MOVEMENT      ·      one enemy per frame:
   e->update(dt) [BB + timer + BT] → main() picks target → e->updateMovement(target,dt)
   (decision lives in update(), motion lives in updateMovement() — 27)

```
## 1. SINGLY LINKED LIST (SLL)
```

  Each node has: [ data | next ] --> points to the next node
  Last node's next = nullptr

  ---- ANATOMY OF ONE NODE ----

      [ 100 | o-o ]      o-o = the *next* pointer slot
         |      |
         |      `--------> points to the following node (or NULL)
         `-> data int

  ---- BUILDING THE LIST FROM SCRATCH (Insert_back) ----

  Insert_back(100):
    head = new node
    head --> [100|*]--> NULL

  Insert_back(200):
    walk to the END, attach there
    head --> [100|*]-->[200|*]--> NULL

  Insert_back(300):
    head --> [100|*]-->[200|*]-->[300|*]--> NULL

  TIMELINE AS A GROWING CHAIN:
    step 1:  head --> [100|*]--> NULL
    step 2:  head --> [100|*]-->[200|*]--> NULL
    step 3:  head --> [100|*]-->[200|*]-->[300|*]--> NULL
    4th (...) head --> [100|*]-->[200|*]-->[300|*]-->[400|*]--> NULL

  ---- PUSH_FRONT (Insert at head) ----

  Before:   head --> [100|*]-->[200|*]-->[300|*]--> NULL

  Insert 50:
                  [50|*]
                    |
  After:    head --/-->[100|*]-->[200|*]-->[300|*]--> NULL


  ---- INSERT AFTER (searchVal=200, newVal=250) ----

  Before:   head -->[100|*]-->[200|*]-->[300|*]--> NULL

  After:    head -->[100|*]-->[200|*]-->[250|*]-->[300|*]--> NULL
                                 (200->next = 250)
                                 (250->next = 300)


  ---- INSERT BEFORE (searchVal=200, newVal=150) ----

  Before:   head -->[100|*]-->[200|*]-->[300|*]--> NULL

  After:    head -->[100|*]-->[150|*]-->[200|*]-->[300|*]--> NULL
                                 (100->next = 150)
                                 (150->next = 200)


  ---- DELETE NODE (val=200) ----

  Before:   head -->[100|*]-->[200|*]-->[300|*]--> NULL
                         prev    curr

  Step 1:   prev->next = curr->next     (patch around 200)

            head -->[100|*]----\      /[300|*]--> NULL
                         prev   \    /   curr
                                 [200|*]  <-- DELETE THIS
                                 (orphaned)

  After:    head -->[100|*]-->[300|*]--> NULL


  ---- DELETE TAIL ----

  Before:   head -->[100|*]-->[200|*]-->[300|*]--> NULL
                         temp

  Step 1:   Walk to second-to-last node
            while(temp->next->next != nullptr)
            temp lands on [200]

            head -->[100|*]-->[200|*]--[X]   temp->next = nullptr
                         temp      [300|*]  <-- DELETE THIS

  After:    head -->[100|*]-->[200|*]--> NULL


  ---- SEARCH (target=200) ----

  head -->[100|*]-->[200|*]-->[300|*]--> NULL
            temp
             |
             +--> 100 != 200, move on
                       temp
                        |
                        +--> 200 == 200! return true



```
## 2. DOUBLY LINKED LIST (DLL)
```

  Each node has: [prev| data | next]
  Can traverse forward AND backward

  ---- PUSH FRONT ----

  Before:   NULL <-- [100] <--> [200] <--> [300] --> NULL
                     head                      tail

  Insert 50:
                  [50]
                 /    \
  After:    NULL <-- [50] <--> [100] <--> [200] <--> [300] --> NULL
                    head                                        tail
                  (head moved to 50, old head 100's prev now points to 50)


  ---- PUSH BACK ----

  Before:   NULL <-- [100] <--> [200] <--> [300] --> NULL
                   head                      tail

  Insert 400:
                                          [300] <--> [400]
                                              tail      \
  After:    NULL <-- [100] <--> [200] <--> [300] <--> [400] --> NULL
                   head                                    tail


  ---- INSERT AFTER (searchVal=200, newVal=250) ----

  Before:   NULL <-- [100] <--> [200] <--> [300] --> NULL

  Step 1:   newnode->next = curr->next     (250 points to 300)
            newnode->prev = curr           (250 points back to 200)
            curr->next->prev = newnode     (300 points back to 250)
            curr->next = newnode           (200 points forward to 250)

  After:    NULL <-- [100] <--> [200] <--> [250] <--> [300] --> NULL


  ---- INSERT BEFORE (searchVal=200, newVal=150) ----

  Before:   NULL <-- [100] <--> [200] <--> [300] --> NULL

  Step 1:   newnode->next = curr           (150 points to 200)
            newnode->prev = prev           (150 points back to 100)
            prev->next = newnode           (100 points forward to 150)
            curr->prev = newnode           (200 points back to 150)

  After:    NULL <-- [100] <--> [150] <--> [200] <--> [300] --> NULL


  ---- DELETE NODE (val=200) ----

  Before:   NULL <-- [100] <--> [200] <--> [300] --> NULL
                         prev    curr     next

  Step 1:   prev->next = curr->next        (100 points to 300)
            next->prev = curr->prev        (300 points back to 100)

            NULL <-- [100] ----\      /----> [300] --> NULL
                         prev   \    /      next
                                 [200]  <-- DELETE THIS
                                 (orphaned)

  After:    NULL <-- [100] <--> [300] --> NULL


  ---- DELETE HEAD ----

  Before:   NULL <-- [100] <--> [200] <--> [300] --> NULL
                    head

  Step 1:   temp = head
            head = head->next
            head->prev = nullptr

  After:    [100]  (orphaned, deleted)
            NULL <-- [200] <--> [300] --> NULL
                         head


  ---- DELETE TAIL ----

  Before:   NULL <-- [100] <--> [200] <--> [300] --> NULL
                                           tail

  Step 1:   Walk to tail
            temp->prev->next = nullptr     (200->next = nullptr)

            NULL <-- [100] <--> [200]       [300]  (orphaned, deleted)
                                           tail


  ---- DISPLAY FORWARD vs DISPLAY REVERSE ----

  Structure: NULL <-- [100] <--> [200] <--> [300] --> NULL
                   head                           tail

  FORWARD (start at head, follow next):
  [100] --> [200] --> [300]
   ^           ^           ^
   |           |           |
  head       head->next  head->next->next

  REVERSE (start at tail, follow prev):
  [300] --> [200] --> [100]
   ^           ^           ^
   |           |           |
  tail       tail->prev  tail->prev->prev


  ---- SWAP NODES (swap 100 and 300) ----

  Before:   NULL <-- [100] <--> [200] <--> [300] --> NULL
                     node A              node B

  Case 1 - Non-adjacent nodes:
            Re-wire ALL prev/next pointers:
            [300] <--> [200] <--> [100]
                     (values swapped in nodes,
                      or pointers re-linked)

  After:    NULL <-- [300] <--> [200] <--> [100] --> NULL


  ---- FIND AND SQUARE (target=200) ----

  Before:   head -->[100]<-->[200]<-->[300]--> NULL

  Search for 200, then:
  temp->data = temp->data * temp->data

  After:    head -->[100]<-->[400]<-->[300]--> NULL
                          (200^2 = 400)


  ---- SOCIAL FEED (DLL Application) ----

  struct Post { postID, likes, next, prev }

  Feed:  NULL <-- [Post1] <--> [Post2] <--> [Post3] --> NULL
                  likes=0      likes=1      likes=0

  LikePost(2):   Post2->likes += 1    =>  likes=1
  DeletePost(2): Remove Post2, re-link Post1 <-> Post3
  ShowFeed():    Traverse forward, print each Post


  ---- MUSIC QUEUE (DLL Application) ----

  struct Song { id, next, prev }

  Queue: NULL <-- [Song1] <--> [Song2] <--> [Song3] --> NULL
  BoostToFront(4):  Add Song4 at head
  AddToEnd(5):      Add Song5 at tail
  SkipSong(2):      Remove Song2, re-link Song1 <-> Song3
  ShowQueue():      Forward traversal
  ShowQueueReverse(): Reverse traversal (tail to head)


  ---- TASK MANAGER (DLL Application) ----

  struct task { id, priority, next, prev }

  List:  NULL <-- [Task1|p=3] <--> [Task2|p=1] <--> [Task3|p=5] --> NULL
  addfirst(id, priority):  Add at head
  addlast(id, priority):   Add at tail



```
## 3. CIRCULAR DOUBLY LINKED LIST (CDLL)
```

  Like DLL but: tail->next = head  AND  head->prev = tail
  Forms a circle! No nullptr anywhere.

  Structure:

                     +--------+     +--------+     +--------+
                     |        v     |        v     |        v
                  +------+    +------+    +------+    +------+
                  | 100  |--->| 200  |--->| 300  |--->| 400  |
                  |      |<---|      |<---|      |<---|      |
                  +------+    +------+    +------+    +------+
                     ^  head                          tail  |
                     |                                    |
                     +------------------------------------+
                        (tail->next = head)
                        (head->prev = tail)


  ---- PUSH FRONT ----

  Before:
                     +---+
                     v   |
                  +------+    +------+    +------+
                  | 100  |--->| 200  |--->| 300  |
                  |      |<---|      |<---|      |
                  +------+    +------+    +------+
                  ^  head                  tail
                  |___________________________|


  Insert 50:

                     +---+        +---+
                     v   |        v   |
                  +------+    +------+    +------+    +------+
                  |  50  |--->| 100  |--->| 200  |--->| 300  |
                  |      |<---|      |<---|      |<---|      |
                  +------+    +------+    +------+    +------+
                  ^  head                  tail
                  |___________________________|

  (50 inserted before head, tail->next now points to 50)


  ---- INSERT AFTER (searchVal=200, newVal=250) ----

  Before:
                  +----------+      +----------+      +----------+
                  |          v      |          v      |          v
                  +--[100]---+<---->+--[200]---+<---->+--[300]---+
                     ^  head                          tail   |
                     |________________________________________|

  After:
                  +----------+      +----------+      +----------+      +----------+
                  |          v      |          v      |          v      |          v
                  +--[100]---+<---->+--[200]---+<---->+--[250]---+<---->+--[300]---+
                     ^  head                                      tail   |
                     |_________________________________________________|


  ---- DELETE NODE (val=200) ----

  Before:
                  +----------+      +----------+      +----------+
                  |          v      |          v      |          v
                  +--[100]---+<---->+--[200]---+<---->+--[300]---+
                     ^  head                          tail   |
                     |________________________________________|

  Step 1:   curr = find(200)
            curr->prev->next = curr->next    (100->next = 300)
            curr->next->prev = curr->prev    (300->prev = 100)

            +----------+              +----------+
            |          v              |          v
            +--[100]---+--\      /---+--[300]---+
               ^  head     \    /      tail   |
               |            [200]              |
               |____________(deleted)__________|

  After:
                  +----------+      +----------+
                  |          v      |          v
                  +--[100]---+<---->+--[300]---+
                     ^  head          tail   |
                     |________________________|


  ---- DISPLAY vs DISPLAY REVERSE ----

  Structure:
                  +----------+      +----------+      +----------+
                  |          v      |          v      |          v
                  +--[100]---+<---->+--[200]---+<---->+--[300]---+
                     ^  head                          tail   |
                     |________________________________________|

  DISPLAY (do-while, from head going forward):
  Start at head, follow next until back to head:
  100 --> 200 --> 300 --> (back to 100, STOP)

  DISPLAY REVERSE (from tail going backward):
  Start at head->prev (tail), follow prev until back to tail:
  300 --> 200 --> 100 --> (back to 300, STOP)


  ---- CHECK INTEGRITY ----

  Walk the list forward. For every node:
    if (curr->next->prev != curr) --> "BROKEN!"

  This verifies the bidirectional links are intact.
  Like checking every road has a return path.



```
## 4. SINGLY CIRCULAR LINKED LIST (SCLL)
```

  Like the SLL (section 1) but the LAST node's next wraps BACK to the head.
  There is NO nullptr anywhere: tail->next == head.  The whole list forms
  one continuous ring.
  (Guide-only variant - not built in start.cpp.)

  ---- ANATOMY OF THE RING ----

      [ data | next ]        next of the last node -> the first node

                +--------+     +--------+     +--------+     +--------+
                |        v     |        v     |        v     |        v
             +------+    +------+    +------+    +------+    +------+
             | 100  |--->| 200  |--->| 300  |--->| 400  |--->| 100  |
             |      |    |      |    |      |    |      |    |  ^   |
             +------+    +------+    +------+    +------+    +------+
                ^ head                                        tail |
                +-----------------------------------------------+
                                                  (tail->next = head)

  Walking forward NEVER ends on its own - you must remember the head
  and STOP when temp wraps back around to it.

  ---- DISPLAY FORWARD (do-while is REQUIRED) ----

      temp = head;
      do {
          cout << temp->data << " ";
          temp = temp->next;
      } while (temp != head);

      100 -> 200 -> 300 -> 400 -> (back to 100, STOP)


  ---- INSERT AT HEAD (PushFront, newVal = 50) ----

  Before:

              +--------+     +--------+     +--------+
              |        v     |        v     |        v
           +------+    +------+    +------+    +------+
           | 100  |--->| 200  |--->| 300  |--->| 100  |   (back to head)
           +------+    +------+    +------+    +------+
             ^head                             tail

  Step 1:  newnode->next = head;      (50 points at old head 100)
  Step 2:  walk to tail               (find the node whose next == head)
  Step 3:  tail->next = newnode;      (300 now points at 50)
  Step 4:  head = newnode;            (50 is the new head)

  After:

              +--------+     +--------+     +--------+     +--------+
              |        v     |        v     |        v     |        v
           +------+    +------+    +------+    +------+    +------+
           |  50  |--->| 100  |--->| 200  |--->| 300  |--->|  50  |
           +------+    +------+    +------+    +------+    +------+
             ^head                                        tail
              (tail keeps pointing at the CURRENT head 50)


  ---- INSERT AFTER (searchVal = 200, newVal = 250) ----

  Before:   head -> [100] -> [200] -> [300] -> (back to 100)

  Step 1:   newnode->next = curr->next;    (250 points at 300)
  Step 2:   curr->next = newnode;          (200 points at 250)

  After:    head -> [100] -> [200] -> [250] -> [300] -> (back to 100)


  ---- THE WRAP TRAP: INSERT AFTER THE TAIL (after 300, newVal = 400) ----

             curr = 300 (the tail),  curr->next == head
             newnode->next must become HEAD, not nullptr!

  Step 1:   newnode->next = head;          (400 points back to 100)
  Step 2:   curr->next = newnode;          (300 points at 400)
  After:    [100] -> [200] -> [300] -> [400] -> (back to 100)

  Writing  newnode->next = NULL  here (SLL habit) BREAKS the ring.


  ---- DELETE NODE (val = 200) ----

  Case A - delete a MIDDLE node:  (same as SLL)

  Before:    head -> [100] -> [200] -> [300] -> (back to 100)
                         prev     curr

  Step 1:    prev->next = curr->next;      (100 points at 300)
  After:     head -> [100] -> [300] -> (back to 100)   [200] deleted


  Case B - delete the HEAD:   (THE circular trap)

  Before:

              +--------+     +--------+     +--------+
              |        v     |        v     |        v
           +------+    +------+    +------+    +------+
           | 100  |--->| 200  |--->| 300  |--->| 100  |
           +------+    +------+    +------+    +------+
             ^ head                           tail

  Step 1:    if head->next == head   -> the ring had ONE node: delete,
             set head = NULL, done.
  Step 2:    walk to tail (the node whose next == head)
  Step 3:    head = head->next;              (new head = 200)
  Step 4:    tail->next = head;              (300 points at NEW head 200)
  Step 5:    delete old head 100

  After:

              +--------+     +--------+
              |        v     |        v
           +------+    +------+    +------+
           | 200  |--->| 300  |--->| 200  |   (200 -> 300 -> 200...)
           +------+    +------+    +------+
             ^ head                  ^ tail

  FORGETTING step 4 is the classic SCLL bug: tail keeps dangling at the
  deleted node and the ring silently breaks into a loose chain.


  ---- DELETE TAIL (val = 300) ----

  Walk to SECOND-TO-LAST (the node whose next->next == head),
  then:  temp->next = head;  delete old tail.

  Before:  [100] -> [200] -> [300] -> (back to 100)
                     temp      tail
  After:   [100] -> [200] -> (back to 100)      [300] deleted


  ---- SCLL vs CDLL (section 3) ----

  SCLL: only forward links + one wrap          -> 1 pointer per node
  CDLL: prev + next both wrap                  -> 2 pointers per node
  SCLL can only walk forward; CDLL walks both ways.
  Nice fit for: round-robin schedulers, token passing, turn-taking.

  ONE-LINE SUMMARY:
  A linked list that loops back on itself; the only hard parts are
  remembering the ring when you insert after the tail and fixing
  tail->next when you delete the head.

```
## 5. LINKED STACK (LIFO - Last In First Out)
```

  Uses singly linked list nodes. Push/pop from HEAD (top).

  ---- PUSH (insert at top) ----

  Before:   top -->[100|*]-->[200|*]-->[300|*]--> NULL

  Push 400:
  newnode->next = top      (400 points at old top 100)
  top = newnode            (top moves to 400)

                  [400|*]
                    |
  After:    top --/-->[100|*]-->[200|*]-->[300|*]--> NULL

  (Same as push_front in SLL)


  ---- POP (remove from top) ----

  Before:   top -->[400|*]-->[100|*]-->[200|*]-->[300|*]--> NULL

  Step 1:   temp = top
            top = top->next

  After:    top -----> [100|*]-->[200|*]-->[300|*]--> NULL
            [400|*]  (deleted)


  ---- PEEK (see top value) ----

  top -->[100|*]-->[200|*]-->[300|*]--> NULL
           ^
           |
         peek() returns 100


  ---- STACK UNDERFLOW ----

  top --> NULL    (empty stack)

  pop() or peek() called:
  "stack underflow!! Nothing to display!"


  ---- STRING STACK ----

  Same concept but Node stores std::string:

  top -->[ "hello Stack" |*]-->[ "hello linkedlists" |*]-->[ "hello world" |*]--> NULL



```
## 6. ARRAY STACK (Static / Fixed Size)
```

  Uses a plain array. top index tracks position.

  Capacity = 5, arr[0..4]

  ---- PUSH ----

  push(1):  top=-1 -> top=0    arr: [1, _, _, _, _]
  push(2):  top=0  -> top=1    arr: [1, 2, _, _, _]
  push(3):  top=1  -> top=2    arr: [1, 2, 3, _, _]
  push(4):  top=2  -> top=3    arr: [1, 2, 3, 4, _]
  push(5):  top=3  -> top=4    arr: [1, 2, 3, 4, 5]
                               idx:  0  1  2  3  4

  ---- POP ----

  pop():    arr[top] deleted, top=4 -> top=3
            arr: [1, 2, 3, 4, _]
            idx:  0  1  2  3  4

  ---- STACK OVERFLOW ----

  push(6):  top == 4 (max index)
            "stack overflow!"

            arr: [1, 2, 3, 4, 5]  <-- FULL!
                  0  1  2  3  4

  ---- STACK UNDERFLOW ----

  pop() when top == -1:
  "stack underflow!! Nothing to display!"


  ---- WHY THE STACK MATTERS (application: balanced brackets) ----

  Input:  ( { [ ( ) ] } )
  Rule:   opening bracket = PUSH, closing bracket = must MATCH top.

  STEP 1   read '('   push ->     top: (
  STEP 2   read '{'   push ->     top: ( {
  STEP 3   read '['   push ->     top: ( { [
  STEP 4   read '('   push ->     top: ( { [ (
  STEP 5   read ')'   matches '(' -> pop ->  top: ( { [
  STEP 6   read ']'   matches '[' -> pop ->  top: ( {
  STEP 7   read '}'   matches '{' -> pop ->  top: (
  STEP 8   read ')'   matches '(' -> pop ->  top: (empty)

  RESULT: stack is EMPTY at the end -> the string is BALANCED  (TRUE)

  A FAILING case   ( [ ) ]
  STEP 1   '(' -> push          top: (
  STEP 2   '[' -> push          top: ( [
  STEP 3   ')' -> TOP is '['  <-  MISMATCH!
          "unbalanced!"  reject immediately (no need to check the rest)

  Why a stack? because matching is "LAST OPENED must CLOSE FIRST"
  (LIFO). A queue or array would give the wrong answer.

  Same trick powers: undo (CTRL+Z), back-button, expression calculator,
  maze/path backtracking, function call stack (what DFS uses!).



```
## 7. LINKED QUEUE (Dynamic - FIFO)
```

  Uses singly linked list. Enqueue at REAR, Dequeue from FRONT.

  ---- ENQUEUE (add to rear) ----

  Before:   front -->[100|*]-->[200|*]--> NULL
                                  ^ rear

  rear points at the LAST node (NOT at NULL). That is exactly
  why "rear->next = newnode(300)" works below.

  Enqueue 300:
  rear->next = newnode(300)
  rear = newnode(300)

  After:    front -->[100|*]-->[200|*]-->[300|*]--> NULL
                                            ^ rear


  ---- DEQUEUE (remove from front) ----

  Before:   front -->[100|*]-->[200|*]-->[300|*]--> NULL
                                            ^ rear

  Step 1:   temp = front
            front = front->next

  After:    front -----> [200|*]-->[300|*]--> NULL
                                      ^ rear
            [100|*] (deleted)


  ---- DEQUEUE ALL (front becomes nullptr, rear also set to nullptr) ----

  front --> [200] --> [300] --> NULL
                        ^ rear

  Dequeue 200:  front --> [300] --> NULL
                            ^ rear
  Dequeue 300:  front --> NULL
                 rear = nullptr  (reset rear too!)

  This is important! If rear isn't reset, it becomes a dangling pointer.


  ---- PEEK ----

  front -->[200|*]-->[300|*]--> NULL
            ^
            |
          peek() returns 200


  Full walkthrough:

  enqueue(100):  [100]         front=rear
  enqueue(200):  [100]->[200]  rear=200
  enqueue(300):  [100]->[200]->[300]  rear=300
  dequeue():     [200]->[300]  front=200
  dequeue():     [300]         front=rear
  dequeue():     NULL          front=rear=nullptr



```
## 8. CIRCULAR ARRAY QUEUE (Static + Circular)
```

  Fixed-size array that wraps around using modulo arithmetic.
  front and rear indices move circularly.

  Capacity = 5, indices 0..4

  ---- ENQUEUE ----

  EnQueue(100):  front=0, rear=0   arr: [100, __, __, __, __]
  EnQueue(200):  front=0, rear=1   arr: [100, 200, __, __, __]
  EnQueue(300):  front=0, rear=2   arr: [100, 200, 300, __, __]
  EnQueue(400):  front=0, rear=3   arr: [100, 200, 300, 400, __]
  EnQueue(500):  front=0, rear=4   arr: [100, 200, 300, 400, 500]
                                      idx:  0    1    2    3    4

  ---- QUEUE FULL ----

  EnQueue(600):  (rear+1)%5 == front
                 (4+1)%5 = 0 == front(0)  --> TRUE
                 "Queue OverFlow!!"

  ---- DEQUEUE ----

  DeQueue():     front = 0, Deleted = 100, front becomes (0+1)%5 = 1
  arr: [100, 200, 300, 400, 500]
  idx:  0    1    2    3    4
        ^front
        (the cell still holds 100 - front simply MOVES past it,
         the value gets overwritten when the slot is reused)

  DeQueue():     front = 1, Deleted = 200, front becomes (1+1)%5 = 2

  ---- THE WRAP-AROUND (the magic part!) ----

  After more enqueues and dequeues the queue no longer starts at 0.

  Say: front = 3,  rear = 0     (rear = index of the LAST element inserted)

  arr: [ 300, __, __, 100, 200 ]
  idx:   0    1    2    3    4

  - index 3 (100) = FRONT : the next DeQueue() returns it
  - index 4 (200) = second in line
  - index 0 (300) = REAR  : the LAST element inserted - the ring
                            already wrapped around back to slot 0

  Occupied cells in circular order, starting at front:
       100 -> 200 -> (wrap) -> 300
      idx3   idx4             idx0

  Circular indexing:
    rear  = (rear  + 1) % capacity
    front = (front + 1) % capacity

  EnQueue(400):  next slot = (0+1)%5 = 1  ->  arr[1] = 400,  rear = 1
  arr: [ 300, 400, __, 100, 200 ]
  idx:   0    1    2    3    4

  EnQueue(500):  next slot = (1+1)%5 = 2  ->  arr[2] = 500,  rear = 2
  arr: [ 300, 400, 500, 100, 200 ]
  idx:   0    1    2    3    4

  The queue is FULL again now: (rear+1)%5 = (2+1)%5 = 3 = front


  ---- QUEUE EMPTY ----

  front == -1 && rear == -1
  "NOTHING TO DISPLAY!"


  ---- VISUAL of circular nature ----

  Capacity is 5 => slots 0..4. Lay the array out as a loop:

           0 --> 1 --> 2 --> 3 --> 4
           ^                         |
           |                         |
           +-------------------------+
          4's NEXT is 0 again:  (4+1) % 5 = 0

  front and rear just walk this ring.
  Every move is (index + 1) % 5,
  so the array never shifts -- we reuse the empty slots.


  ---- WHY THE QUEUE MATTERS (application: print jobs) ----

  3 students queue their print jobs. FIRST in = FIRST out (FIFO):

  arrive order:   Ali -> Sara -> Zain

   timeline   printer queue (front ... rear)      printed by printer
   -------    --------------------------------    ----------------
   t=0        [Ali]                               --
   t=1        [Ali  Sara]                         --
   t=2        [Ali  Sara  Zain]                   --
   t=3        [Sara  Zain]                        Ali (front served)
   t=4        [Zain]                              Sara
   t=5        []                                  Zain

        front always gets the NEXT document to print,
        new jobs always join at the REAR.

  Swap roles (call it a stack) and the LAST job prints FIRST -
  everyone complains. That is why OS queues are FIFO.

  Same idea powers: BFS (we did), keyboard buffers, task scheduling,
  server request queues.



```
## 9. MAX-HEAP (Binary Heap - Array Based)
```

  Complete binary tree stored in array.
  Parent is always >= children (Max-Heap property).

  Index rules:
    Parent of i    = (i - 1) / 2
    Left child of i  = 2*i + 1
    Right child of i = 2*i + 2

  ---- INSERT (with bubble-up / sift-up) ----

  Insert order: 5, 10, 15, 20, 25

  Step 1: insert(5)
  arr: [5]
  size=1

  Step 2: insert(10)
  arr: [5, 10]
  10 > parent(5)? YES -> swap
  arr: [10, 5]

           [10]          arr: [10, 5]
           /
          [5]

  Step 3: insert(15)
  arr: [10, 5, 15]
  15 > parent(10)? YES -> swap
  arr: [15, 5, 10]

           [15]
          /    \
        [5]    [10]

  Step 4: insert(20)
  arr: [15, 5, 10, 20]
  20 > parent(5)? YES -> swap
  arr: [15, 20, 10, 5]
  20 > parent(15)? YES -> swap
  arr: [20, 15, 10, 5]

              [20]
             /    \
           [15]   [10]
           /
          [5]

  Step 5: insert(25)
  arr: [20, 15, 10, 5, 25]
  25 > parent(15)? YES -> swap
  arr: [20, 25, 10, 5, 15]
  25 > parent(20)? YES -> swap
  arr: [25, 20, 10, 5, 15]

              [25]
             /    \
           [20]   [10]
           / \
         [5] [15]


  ---- EXTRACT MAX (remove root, bubble-down / sift-down) ----

  Before:
              [25]            arr: [25, 20, 10, 5, 15]  size=5
             /    \
           [20]   [10]
           / \
         [5] [15]

  Step 1:   Overwrite the ROOT with the LAST element
            arr[0] = arr[size-1]    (root 25 replaced by 15)
            size--                  (old root 25 removed, size = 4)

              [15]            arr: [15, 20, 10, 5]  size=4
             /    \
           [20]   [10]
           /
         [5]

  Step 2:   Bubble down - compare with children, swap with LARGER child
            15 < children(20, 10)?  YES -> swap with 20 (larger)

              [20]
             /    \
           [15]   [10]
           /
         [5]

            15 < children(5)?  NO -> STOP (heap property restored)

  Final:
              [20]
             /    \
           [15]   [10]
           /
         [5]

  arr: [20, 15, 10, 5]  size=4


  ---- COMPLETE INSERT/EXTRACT FLOW ----

  insert(5):     [5]
  (all 5 inserts: 5, 10, 15, 20, 25)   ->   same final heap as above:

              [25]
             /    \
           [20]   [10]
           / \
         [5] [15]

  arr: [25, 20, 10, 5, 15]   <-- array view of that heap

  extractMax():  removes 25, bubbles down 15

              [20]
             /    \
           [15]   [10]
           /
         [5]

  getMax():      returns 20 (arr[0])
  getSize():     returns 4


  ---- TREE-TO-ARRAY MAPPING ----

              [25]  index 0
             /    \
           [20]    [10]  index 1, 2
           / \
         [5]  [15]       index 3, 4

  Array: [25, 20, 10, 5, 15]
          0   1   2  3   4

  Node at index 0: left=1, right=2
  Node at index 1: left=3, right=4
  Node at index 2: left=5 (out of bounds!), right=6 (out of bounds!)

  ---- HEAPIFY (build a max-heap from a random array) ----

  The code inserts one value at a time (bubble-up).
  But you can also build a heap from an ARRAY by fixing every
  parent from the BOTTOM up (bubble-down). Same result.

  Step 1   random array      [ 3, 10, 8, 17, 2, 25, 6 ]

               [3]
              /   \
          [10]     [8]
          /  \     / \
       [17] [2] [25] [6]

  Step 2   start from the LAST parent (index 1 = 10).
           its children: 17, 2.
           10 < 17 -> swap (bubble 17 up)

               [3]
              /   \
          [17]     [8]
          /  \     / \
       [10] [2] [25] [6]       arr: [3,17,8,10,2,25,6]

  Step 3   next parent (index 0 = 3).   children: 17, 8.
           3 < 17 -> swap

               [17]
              /   \
           [3]     [8]
          /  \     / \
       [10] [2] [25] [6]       arr: [17,3,8,10,2,25,6]

           keep bubble-down at index 1: children 10, 2.
           3 < 10 -> swap

               [17]
              /   \
          [10]     [8]
          /  \     / \
        [3]  [2] [25] [6]       arr: [17,10,8,3,2,25,6]

  Step 4   still wrong! index 2 = 8, child 25.
           8 < 25 -> swap

               [17]
              /   \
          [10]     [25]
          /  \     / \
        [3]  [2] [8]  [6]       arr: [17,10,25,3,2,8,6]

  Step 5   index 0 = 17 vs children 10, 25. 17 < 25 -> swap

               [25]
              /   \
          [10]     [17]
          /  \     / \
        [3]  [2] [8]  [6]       arr: [25,10,17,3,2,8,6]

           17 has one child: 8.   17 >= 8 -> OK, STOP.

  FINAL: a valid max-heap   arr: [25,10,17,3,2,8,6]

  Read it:  Heapify fixes every parent from the BOTTOM up (bubble-down),
            Insert grows the heap one value at a time (bubble-up).

  NOTE: heapify is NOT in start.cpp; shown here to complete the picture.





---- HEAP OVERFLOW (fixed capacity) ----

  MaxHeap heap(10)  ->  capacity = 10

  After 10 inserts the array is FULL:

                    [50]
                   /    \
                 [45]   [40]
                 / \    / \
               [35][30][25][20]
               / \   /
             [15][10][5]

  arr: [50, 45, 40, 35, 30, 25, 20, 15, 10, 5]
        0   1   2   3   4   5   6   7   8   9
  size = 10 = capacity

  insert(100):
    if (size == capacity)  ->  10 == 10  TRUE
    "Heap overflow"        (value rejected, heap unchanged)


  ---- EXTRACT MAX ON EMPTY HEAP ----

  MaxHeap empty:  size = 0

  extractMax():
    if (size == 0)  ->  TRUE
    "Heap is empty!"     (returns early, no crash / no garbage)


  ---- GETMAX / GETSIZE ----

  Heap after extractMax:
              [20]
             /    \
           [15]   [10]
           /
          [5]

  getMax():   returns arr[0]  ->  20
  getSize():  returns size    ->  4

  getMax() on EMPTY heap:
    "Heap is empty!"  ->  returns -1  (sentinel value)




```
## 10. BINARY SEARCH TREE (BST)
```

  A node = [ data | left | right ]

         +--------+
         |  left  |---->  all smaller values
         | data   |
         |  right |---->  all bigger values
         +--------+

  RULE:  left subtree < root < right subtree     (no duplicates)

  Insert order: 50, 30, 60, 25, 55

  ---- INSERT STEP BY STEP ----

  STEP 1  insert(50):  tree EMPTY -> 50 becomes ROOT

              +------+
              |  50  |     <-- root (NULL left, NULL right)
              +------+

  STEP 2  insert(30):  30 < 50 -> go LEFT, empty -> attach

              +------+
              |  50  |
              +------+
              /
         +------+
         |  30  |       <-- new node
         +------+

  STEP 3  insert(60):  60 > 50 -> go RIGHT, empty -> attach

              +------+
              |  50  |
              +------+
             /        \
        +------+    +------+
        |  30  |    |  60  |     <-- new node
        +------+    +------+

  STEP 4  insert(25):  50 -> 25<50 LEFT -> 30 -> 25<30 LEFT, empty

              +------+
              |  50  |
              +------+
             /        \
        +------+    +------+
        |  30  |    |  60  |
        +------+    +------+
         /
    +------+          <-- new node
    |  25  |
    +------+

  STEP 5  insert(55):  50 -> 55>50 RIGHT -> 60 -> 55<60 LEFT, empty

              +------+
              |  50  |
              +------+
             /        \
        +------+    +------+
        |  30  |    |  60  |
        +------+    +------+
         /            /
    +------+    +------+
    |  25  |    |  55  |       <-- new node
    +------+    +------+

  DUPLICATE  insert(50):  while walking, 50 == current->data
                          "value already exists" (newnode deleted)


  ---- SEARCH 55 ----         (one comparison per level)

              +------+
              |  50  |         55 > 50 -------+
              +------+                        |
             /        \                       v  go RIGHT
        +------+    +------+
        |  30  |    |  60  |     55 < 60 -------+
        +------+    +------+                    v  go LEFT
         /            /
    +------+    +------+
    |  25  |    |  55  |     55 == 55 ->  "found it!" TRUE
    +------+    +------+        ^
                                |
                           target hit

  ---- SEARCH 99 (missing) ----

              +------+
              |  50  |         99 > 50 -------+
              +------+                        |
             /        \                       v
        +------+    +------+
        |  30  |    |  60  |     99 > 60 -------+
        +------+    +------+                    v
         /            /
    +------+    +------+          +--------+
    |  25  |    |  55  |  99 > 55 |  NULL  |  current hit NULL
    +------+    +------+          +--------+
                                  "value not found!" FALSE


  ---- THE FINAL TREE ----

                      +------+
                      |  50  |
                      +------+
                     /        \
               +------+    +------+
               |  30  |    |  60  |
               +------+    +------+
              /                \
         +------+          +------+
         |  25  |          |  55  |
         +------+          +------+


  ---- 3 TRAVERSALS (numbers = visit order) ----

  PRE-ORDER   node -> left -> right     (ROOT FIRST)

                 (1)
              +------+
              |  50  |
              +------+
             /        \
        (2)-----+   +-----(4)
        +------+   +------+
        |  30  |   |  60  |
        +------+   +------+
           /              \
      (3)-----+       +-----(5)
      +------+       +------+
      |  25  |       |  55  |
      +------+       +------+

  ORDER:  50  30  25  60  55

  IN-ORDER   left -> node -> right      (SORTED!)

                 (3)
              +------+
              |  50  |
              +------+
             /        \
        (2)-----+   +-----(4)
        +------+   +------+
        |  30  |   |  60  |
        +------+   +------+
           /              \
      (1)-----+       +-----(5)
      +------+       +------+
      |  25  |       |  55  |
      +------+       +------+

  ORDER:  25  30  50  55  60      <-- ascending!

  POST-ORDER   left -> right -> node     (ROOT LAST)

                 (5)
              +------+
              |  50  |
              +------+
             /        \
        (2)-----+   +-----(4)
        +------+   +------+
        |  30  |   |  60  |
        +------+   +------+
           /              \
      (1)-----+       +-----(3)
      +------+       +------+
      |  25  |       |  55  |
      +------+       +------+

  ORDER:  25  30  55  60  50

  MEMORY TRICK:
    PRE  = root prints FIRST
    IN   = root prints in the MIDDLE  (=> output is sorted)
    POST = root prints LAST




```
## 11. TRIE (PREFIX TREE)
```

  A trie node = 26 LETTER SLOTS (a..z) + an end-of-word flag.

        +-------------------------------------------------+
        |  children[26]                                    |
        |                                                 |
        |    a    b    c    d  ...   r    s    t  ...   z |
        |    |    |    |    |        |    |    |          |
        |    v    v    v    v        v    v    v          |
        +-------------------------------------------------+
        |  isEndOfWord : true / false                     |
        +-------------------------------------------------+


  ---- INSERT "cat" ----

  Each letter: if its slot is EMPTY, make a new node, step into it.

         +--------+    slot 'c' empty -> new node
         |  ROOT  |
         +--------+
             |
            c|
             v
         +--------+
         |   c    |
         +--------+
             |
            a|
             v
         +--------+
         |   a    |
         +--------+
             |
            t|
             v
         +--------+
         |   t    |     <-- isEndOfWord = true
         +--------+          "cat" is complete here

  ---- INSERT "car" ----

  'c' and 'a' already EXIST -> walk and REUSE them, build only 'r'.

         +--------+
         |  ROOT  |
         +--------+
             |
            c|      (already there, reuse)
             v
         +--------+
         |   c    |
         +--------+
             |
            a|      (already there, reuse)
             v
         +--------+
         |   a    |
         +--------+
          /      \
      t /          \ r
       v            v
  +--------+    +--------+
  |   t    |    |   r    |    <-- isEndOfWord = true
  +--------+    +--------+         "car" complete here
  (*) "cat"      (*) "car"

  The c and a nodes are SHARED between both words
  (the common prefix "ca").


  ---- THE STORED TRIE (as it sits in memory) ----

          ROOT                 (26 letter slots; we draw only 2)
       +-----------+
       | slot d .. :------> NULL           'd' was never inserted
       | slot c .. :------> [c-node]
       +-----------+
                     |
                     v
              +-----------+
              |  c-node   |    slot 'a' is its only used letter
              +-----------+--------> [a-node]
                (isEnd=F)
                     |
                     v
              +-----------+
              |  a-node   |    isEndOfWord = false
              +-----------+        (prefix "ca" is NOT a word)
                (isEnd=F)
               /         \
             v            v
       +-----------+   +-----------+
       |  t-node   |   |  r-node   |
       | isEndWord |   | isEndWord |
       |   true    |   |   true    |
       +-----------+   +-----------+
        "cat" ends       "car" ends

  isEndOfWord marks a COMPLETE word and nothing else.

  ---- SEARCH vs STARTSWITH ----

  search("cat")    c -> a -> t,  t.isEndOfWord = true
                   return TRUE   (1)   [a complete word]

  search("can")    c -> a, 'n' slot of a = NULL -> path dies
                   return FALSE  (0)

  search("ca")     c -> a, a.isEndOfWord = false
                   return FALSE  (0)   [only a prefix, NOT a word]

  startsWith("ca") c -> a path exists (no end check)
                   return TRUE   (1)   [prefix exists]

  startsWith("do") 'd' slot of ROOT = NULL
                   return FALSE  (0)   [not even a prefix]

  KEY DIFFERENCE:
    search()      = prefix walk + isEndOfWord check   (whole word)
    startsWith()  = prefix walk ONLY                  (any prefix)




```
## 12. GRAPH (ADJACENCY LIST + BFS/DFS)
```

  Storage:  vector<vector<int>> adj   -> ONE neighbor list per vertex.

  ---- ADJACENCY LIST (the ACTUAL layout in memory) ----

   adj[0]  -> [ 1 ] -> [ 2 ] -> NULL
   adj[1]  -> [ 0 ] -> [ 3 ] -> NULL
   adj[2]  -> [ 0 ] -> [ 3 ] -> NULL
   adj[3]  -> [ 1 ] -> [ 2 ] -> [ 4 ] -> NULL
   adj[4]  -> [ 3 ] -> [ 5 ] -> NULL
   adj[5]  -> [ 4 ] -> NULL

   Undirected: each edge appears in BOTH endpoints' lists.
   addEdge(u,v)  =>  adj[u].push_back(v);  adj[v].push_back(u);

  ---- THE SAME GRAPH AS A PICTURE ----

        [ 0 ]----- [ 1 ]
         |           |
         |           |
        [ 2 ]----- [ 3 ]-----[ 4 ]-----[ 5 ]

   Edges: (0,1) (0,2) (1,3) (2,3) (3,4) (4,5)


  ---- BFS from 0  (FIFO queue, spread like a wave) ----

  visited[] marks a vertex the moment it ENTERS the queue.

  DISCOVERY TREE (arrow = "who found whom"):

          [ 0 ]
         /   \
        v     v
      [ 1 ]   [ 2 ]      1 & 2 discovered by 0
        |
        v
      [ 3 ]              3 reached via 1 (also via 2, already seen)
        |
        v
      [ 4 ]              4 discovered by 3
        |
        v
      [ 5 ]              5 discovered by 4

  LAYERS (ring by ring):

              (0)         layer 0
            /    \
          (1)   (2)       layer 1
            \   /
            (3)           layer 2
             |
            (4)           layer 3
             |
            (5)           layer 4

  QUEUE TAPE  (front ... rear):
    step       queue             printed
    ----       -----             -------
    start      [0]
    pop 0      []                0
    push 1,2   [1  2]
    pop 1      [2]               1
    push 3     [2  3]
    pop 2      [3]               2
    pop 3      []                3
    push 4     [4]
    pop 4      []                4
    push 5     [5]
    pop 5      []                5

  BFS ORDER:   0  1  2  3  4  5

  VISITED[] ARRAY SNAPSHOTS  (1 = seen, 0 = not yet)

  code:  vector<bool> visited(adj.size(), false);

  BEFORE:         idx: 0   1   2   3   4   5
                 +---+---+---+---+---+---+
                 | 0 | 0 | 0 | 0 | 0 | 0 |
                 +---+---+---+---+---+---+
                     (all false = nothing seen yet)

  after start:    | 1 | 0 | 0 | 0 | 0 | 0 |    visited[0] = true

  push 1,2:       | 1 | 1 | 1 | 0 | 0 | 0 |    marked when entering queue

  push 3:         | 1 | 1 | 1 | 1 | 0 | 0 |

  pop 2           | 1 | 1 | 1 | 1 | 0 | 0 |    (2 already seen, nothing new)

  push 4:         | 1 | 1 | 1 | 1 | 1 | 0 |

  push 5:         | 1 | 1 | 1 | 1 | 1 | 1 |    all 6 seen -> loop ends

  WHY mark at PUSH time, not pop time?
    - a vertex can be reached by 2 neighbors (e.g. 1 and 2 both see 3).
      If we waited until pop, 3 would be pushed TWICE.
      Marking at push time guarantees exactly ONE visit per vertex.


  ---- DFS from 0  (recursive, go DEEP before going wide) ----

  DFSHelper(v): mark v, then recurse into each unvisited neighbor.

  DFS ORDER:   0  1  3  2  4  5

  DFS PATH (the numbers are the order the steps happen):

   FIRST DIVE (keep going deeper until stuck):
      (1)      (2)      (3)
     [ 0 ] -> [ 1 ] -> [ 3 ] -> [ 2 ]    2 = dead end (0 and 3 seen)
            (4) <-  2 backs out, return to 3

   SECOND DIVE (DFS(3) now tries its NEXT neighbor = 4):
      (5)                        (6)
     [ 3 ] ----> [ 4 ] -> [ 5 ]     5 = dead end (4 seen)

   then unwind all frames: 5 -> 4 -> 3 -> 1 -> 0

   Wait: node 2 hangs off 3 (drawn above it only to save space):
        [3]--[2]   and   [3]--[4]--[5]   are the two dives.

   MARK ORDER:   0    1    3    2    4    5

  CALL STACK (grows on each recursive call, shrinks on return):

    visit 0   | DFS(0)
    visit 1   | DFS(1) | DFS(0)
    visit 3   | DFS(3) | DFS(1) | DFS(0)
    visit 2   | DFS(2) | DFS(3) | DFS(1) | DFS(0)
    pop 2     | DFS(3) | DFS(1) | DFS(0)      (2 fully explored)
    visit 4   | DFS(4) | DFS(3) | DFS(1) | DFS(0)
    visit 5   | DFS(5) | DFS(4) | DFS(3) | DFS(1) | DFS(0)
    pop all   | (empty)                        (back to main)

  BFS = level by level (queue)   |   DFS = deep dive then backtrack




```
## 13. DIJKSTRA (SHORTEST PATH - WEIGHTED GRAPH)
```

  Weighted undirected graph from start.cpp:

  addEdge(0,1,2)  addEdge(0,2,4)  addEdge(1,3,3)
  addEdge(2,3,1)  addEdge(3,4,5)  addEdge(4,5,2)

  PICTURE (edge weights shown beside each line):

            w=2
       [ 0 ]------[ 1 ]
        |           |w=3
        |w=4        |
       [ 2 ]------[ 3 ]------[ 4 ]------[ 5 ]
            w=1         w=5         w=2

  We find the CHEAPEST route from START (0) to every vertex.

  dist[] = best known cost so far.
  priority_queue (min-heap) of {cost, vertex} -> smallest pops first.

  ---------------- RUN ----------------

  INITIAL:
   dist:  | 0 | INF | INF | INF | INF | INF |
           ^-- start is 0, everything else unknown
   pq:    [ (0,0) ]

  STEP 1   pop (0,0)
     check 0->1:  0+2 = 2  < INF ->  dist[1]=2   push (2,1)
     check 0->2:  0+4 = 4  < INF ->  dist[2]=4   push (4,2)
   pq = [ (2,1) (4,2) ]
   dist:  | 0 | 2 | 4 | INF | INF | INF |
           ^pop   ^new  ^new
           (0,0)  updated values from STEP 1

  STEP 2   pop (2,1)
     check 1->0:  2+2 = 4  not < 0 -> skip
     check 1->3:  2+3 = 5  < INF    -> dist[3]=5  push (5,3)
   pq = [ (4,2) (5,3) ]
   dist:  | 0 | 2 | 4 | 5 | INF | INF |

  STEP 3   pop (4,2)
     check 2->0:  4+4 = 8  not < 0 -> skip
     check 2->3:  4+1 = 5  not < 5 -> skip (same cost)
   pq = [ (5,3) ]
   dist:  | 0 | 2 | 4 | 5 | INF | INF |

  STEP 4   pop (5,3)
     check 3->1:  5+3 = 8   not < 2  -> skip
     check 3->2:  5+1 = 6   not < 4  -> skip
     check 3->4:  5+5 = 10  < INF    -> dist[4]=10 push (10,4)
   pq = [ (10,4) ]
   dist:  | 0 | 2 | 4 | 5 | 10 | INF |

  STEP 5   pop (10,4)
     check 4->3:  10+5 = 15 not < 5  -> skip
     check 4->5:  10+2 = 12 < INF    -> dist[5]=12 push (12,5)
   pq = [ (12,5) ]
   dist:  | 0 | 2 | 4 | 5 | 10 | 12 |

  STEP 6   pop (12,5)
     check 5->4:  12+2 = 14 not < 10 -> skip
   pq = [ ]  ->  DONE


  WHAT THE PRIORITY QUEUE LOOKS LIKE INSIDE  (a binary min-heap)

  The std::priority_queue used in the code is a min-heap:
  the SMALLEST cost always sits at the ROOT.

  after STEP 1  (pq has (2,1) and (4,2)):

              (2,1)          <- root = the smallest cost = popped next
              /
          (4,2)

  after STEP 2  (pq has (4,2) and (5,3)):

              (4,2)
              /
          (5,3)

  after STEP 3  (one entry left):

              (5,3)

  heap rule for a MIN-heap:  parent cost <= child cost.
  every pop: remove root, fix the heap -> the next-smallest rises.
  every push: place at the bottom, bubble UP until the rule holds.

  THAT is why "the smallest cost pops first",
  and why the first time we pop a vertex its distance is final.


  ---------------- RESULT ----------------

   vertex:    0     1     2     3     4     5
              +-----+-----+-----+-----+-----+-----+
   dist:      |  0  |  2  |  4  |  5  | 10  | 12  |
              +-----+-----+-----+-----+-----+-----+

  Answer:  shortest cost 0 -> 5  =  12

  Path rebuilt by following the saved parents:

    [ 0 ] --4--> [ 2 ] --1--> [ 3 ] --5--> [ 4 ] --2--> [ 5 ]
       \  cost           +         +            +          /
        \     4      +          1        +          5    +     2
         --------------------------------------------------
                                  total = 12

  (a second equal route: 0 -> 1 -> 3 -> 4 -> 5 also = 12)

  ---------------- DETAILS ----------------

  STALE PQ ENTRY:   if (cost > dist[current]) continue;
                    a cheaper route was already settled,
                    so pop the old junk entry and ignore it.

  UNREACHABLE:      dist[i] stays INT_MAX
                    -> the code prints "INF!" for it.

  GREEDY CORRECTNESS:
    pq always pops the smallest {cost, vertex} first,
    so the FIRST time a vertex is popped, its distance is FINAL.



```
## 14. A* PATHFINDING
```

  READ THIS FIRST
  * Everything below is traced from BIG-ONE.cpp (the game AI file).
  * The mesh, costs, centroids and function names all come from that code.
  * Numbers are rounded to 2 decimals to keep the tables readable.

  A* = Dijkstra + a HEURISTIC. Both use a min-heap and g-costs, but A*
  is "pulled" toward the goal instead of spreading evenly.

  CORE FORMULA
        f(n) = g(n) + h(n)

        g(n) = exact cost of the best path found SO FAR to node n
        h(n) = estimated cost from n to the GOAL (the heuristic)
        f(n) = total estimated cost of a path THROUGH n
        the priority queue always pops the SMALLEST f(n)

        h must be ADMISSIBLE: never overestimate the true remaining
        distance, otherwise a non-optimal path can win.

  THE MESH BUILT IN main() (BIG-ONE.cpp):

                       15                               <-- edge cost 2-3
        [2]  (5,15,8)  ──────────────────────> [3] (15,15,10)
         │             ╲                    ╱ │
         │   10          ╲       37       ╱  │       10
         │ (1-2)          ╲     (0-3)   ╱   │      (1-3)
         │                 ╲           ╱     │
         │                   ╲       ╱       │
        [0]  (5,5,5)  ─────────────────────> [1] (15,5,5)
                       10                     <-- edge cost 0-1

   centroids stored in addPolygon():
     ID   center(x, y, z)
     0    ( 5,  5,  5)
     1    (15,  5,  5)
     2    ( 5, 15,  8)
     3    (15, 15, 10)

   connection table (addConnection calls in main()):
     edge       cost
     0 - 1       10
     0 - 3       37
     1 - 2       10
     1 - 3       10
     2 - 3       15

  THE HEURISTIC USED IN CODE (heuristic() line 676)
        h(from, goal) = sqrt( dx^2 + dy^2 + dz^2 )      Euclidean, 3D

   h values toward goal 3, computed from the centroids:
        h(0) = |10,10,5|  = sqrt(100+100+25) = 15.00
        h(1) = |0,10,5|   = sqrt(0+100+25)   = 11.18
        h(2) = |10,0,2|   = sqrt(100+0+4)    = 10.20
        h(3) = 0                                          (already there)

  FULL A* TRACE:  aStar(0, 3)

        dist[]  = g cost          parent[] = previous node
        pq      = min-heap of {fCost, node}

   STEP  init          dist[0]=0, push {15.00, 0}
         dist[0..3]    0  inf inf inf        parent  -  -  -  -

   STEP  pop {15.00, 0}
         nb 1: g=0+10=10,  f=10+11.18=21.18  push {21.18, 1}
         nb 3: g=0+37=37,  f=37+0=37.00     push {37.00, 3}
         dist[0..3]    0  10  inf  37        parent  -  0  -  0

   STEP  pop {21.18, 1}
         nb 0: closed already, skip
         nb 2: g=10+10=20, f=20+10.20=30.20  push {30.20, 2}
         nb 3: g=10+10=20 < 37  ->  dist[3]=20, parent[3]=1
               f=20+0=20                      push {20.00, 3}
         dist[0..3]    0  10  20  20         parent  -  0  1  1

   STEP  pop {20.00, 3}          current == goal  ->  STOP

   pq viewed as a min-heap after step 2 (smallest f on top):

                  {20.00, 3}        <- this is the goal, pops next!
                 /         \
          {30.20, 2}     {37.00, 3}

  STALE ENTRY check (lazy deletion, line 714)
   The pq still holds {37.00, 3} from step 1. If it is ever popped:

        stored f == dist[current] + heuristic(current, goal) ?
        37.00    != 20.00 + 0        -> REJECT, skip the entry

  RECONSTRUCTION (walk parent[] backward, then std::reverse)
        at=3 -> parent[3]=1 -> parent[1]=0 -> parent[0]=-1 (stop)
        collected [3, 1, 0]   reverse   ->   [0, 1, 3]
        total cost = dist[3] = 20          (0->1 = 10, 1->3 = 10)

   IMPORTANT: the direct edge 0->3 costs 37, yet A* returned 0->1->3 for 20.
   Because h(0,3)=15 <= true 20 (admissible), A* was allowed to prefer the
   cheaper two-hop route.

  BLOCKED START / GOAL (lines 688-695)
        if (isBlocked(start) || isBlocked(goal)) return empty;  --- early
        exit: a blocked polygon is never even searched.

  ONE-LINE SUMMARY:
        A* = Dijkstra + heuristic. It stopped the moment the goal popped,
        which is what makes it faster than topic 12's plain Dijkstra.



```
## 15. A* WITH WAYPOINTS (WEIGHTED NODES)
```

  Grid A* (sections 14/17) walks CELLS.  Waypoint A* walks a hand-placed
  graph over a real map: nodes ARE the waypoints, edges are custom
  connections with weights = travel cost.  f = g + h still, but the
  heuristic is the straight-line distance between waypoint coordinates.

  ---- THE MAP (from the 4D/8D/Waypoint demo, case 3) ----

      waypoint coords:
          0  Castle   ( 0,  0)
          1  Bridge   (15,  0)
          2  Camp     ( 5,  5)       <- diagonal shortcut node
          3  Village  ( 0, 20)
          4  Town     (40, 10)       <- the goal

      custom edges (weights are hand-picked, NOT grid steps):
          0-1 : 10    Castle -> Bridge
          0-3 : 10    Castle -> Village
          0-2 :  7    Castle -> Camp     (shortcut)
          2-4 :  7    Camp -> Town       (shortcut)
          1-4 : 10    Bridge -> Town
          3-4 : 10    Village -> Town

      picture (weights beside each edge):

                        7   short
         Castle 0 -------- Camp 2 -------- 7 -------> short/ 4: Town
            \   10                        |                /
              \                           |  10          /
           Village 3 ---------------------- 10 ---- Town 4  (goal)
                                    Bridge 1 -----------10<

      (a cleaner sketch:

           Bridge 1 (15,0)
              |
             10
              |
            Town 4 (40,10)   <-- goal
              ^
              | 10
              |
         Castle 0 (0,0) --- 7 --- Camp 2 (5,5) --- 7 ---> Town 4 (via 2)
              |
             10
              |
        Village 3 (0,20)

           edges:  0-1:10  1-4:10  3-4:10  0-3:10  0-2:7  2-4:7 )

  ---- HEURISTIC (Euclidean 2D between waypoint coords) ----

      h(n) = sqrt(dx^2 + dy^2)

      waypoint   h toward goal (40,10)
      --------   ---------------------------------------------
      Castle 0   sqrt(40^2 + 10^2)  = sqrt(1700) = 41.23
      Bridge 1   sqrt(25^2 + 10^2)  = sqrt(725)  = 26.93
      Camp   2   sqrt(35^2 + 5^2)   = sqrt(1250) = 35.36
      Village 3  sqrt(40^2 + (-10)^2) = sqrt(1700) = 41.23
      Town   4   0

  ---- FULL TRACE  aStar(0, 4)  exactly like the demo runs it ----

   STEP  init       dist[0]=0, push {f = 41.23, 0}
         dist:   0    inf   inf   inf   inf       parent: -  -  -  -  -

   STEP  pop {41.23, 0}   (Castle)
         nb 1: g=0+10=10,  f=10+26.93=36.93        push {36.93, 1}
         nb 2: g=0+7=7,    f=7+35.36=42.36         push {42.36, 2}
         nb 3: g=0+10=10,  f=10+41.23=51.23        push {51.23, 3}
         dist:   0    10    7    10     inf        parent: -  0  0  0  -

   STEP  pop {36.93, 1}   (Bridge)
         nb 0: already settled, skip
         nb 4: g=10+10=20, f=20+0=20.00            push {20.00, 4}
         dist:   0    10    7    10     20         parent: -  0  0  0  1

   STEP  pop {20.00, 4}   (Town)   current == goal  ->  STOP

   RECONSTRUCTION:  4 -> parent[4]=1 -> parent[1]=0
         path = [ 0, 1, 4 ]      total cost = 20   (10 + 10)

  ---- CAVEAT: the shortcut lost to a fat f (admissibility) ----

   Through Camp the route  0 -> 2 -> 4  costs 7 + 7 = 14 -- CHEAPER than
   the 20 that A* returned!  Why did it lose?
     h(2) = 35.36  is far bigger than the true remaining cost (7).
     Overestimating h inflates f(2) to 42.36, so node 2 never bubbles to
     the top of the min-heap before the goal pops at f = 20.
   => The heuristic is NOT admissible here: hand-set "shortcut" weights
      (7) are smaller than the straight-line geometry (Camp-Town = sqrt(35^2
      +5^2) ~ 35.4).  A* only stays optimal when every edge weight is >=
      the straight-line distance between its endpoints.
   Fixes: weight edges >= geometry, or clamp h(n) to the graph's minimum
   edge weight, or use a graph-aware heuristic (cut the straight-line h
   down below the smallest possible remaining edge-sum).

  ONE-LINE SUMMARY:
  Waypoints = a sparse custom graph over a real map.  A* is unchanged
  (f = g + h); only the node coordinates and the hand-chosen weights
  differ.  Keep h admissible or A* silently returns a worse route.

```
## 16. TIME-SLICING (ASYNCHRONOUS A*)
```

  On a huge map one full A* call can cost 1-2 ms.  Called every frame it
  freezes the whole game for that frame.  TIME-SLICING spreads one search
  across MANY frames: each frame the algorithm is allowed a fixed number
  of expansions (the budget), then it pauses and resumes next frame.

  ---- THE THREE PIECES (current code) ----

   PathState    enum IDLE / SEARCHING / FOUND / FAILED
   PathRequest  struct that owns EVERYTHING the search needs to pause
                and resume in the middle.
   NavMesh      startPathRequest()  fires the search
                updatePathfindingSlice(request, maxSteps)  burns budget

              +-----------------------------------------------+
              |  PathRequest (L54)                             |
              |    state    : IDLE -> SEARCHING -> FOUND/FAILED |
              |    pq       : min-heap { f, node } = resume pt  |
              |    gcost    : best g per node (persists)        |
              |    parents  : parent of each node               |
              |    FinalPath: reconstructed route when FOUND    |
              +-----------------------------------------------+

  ---- START A SEARCH  (startPathRequest, L295) ----

        request.state     = SEARCHING
        request.FinalPath .clear()
        request.pq        = empty pq
        request.gcost     = INF everywhere
        request.parents   = -1 everywhere
        request.gcost[start] = 0
        request.pq.push({ heuristic(start, goal), start })

  ---- BURN ONE TIME-SLICE  (updatePathfindingSlice, L248) ----

        if (request.state != SEARCHING) return;     // stale req, ignore

        stepsTaken = 0;
        while (!request.pq.empty() && stepsTaken < maxSteps) {
            pop { f, current };
            if (stale f) continue;                  // f > g+h -> skip
            if (current == goal) {                  // DONE
                rebuild FinalPath from parents; reverse it;
                request.state = FOUND;
                return;
            }
            for (unblocked neighbor next) {         // expand one node
                g = request.gcost[current] + weight;
                if (g < request.gcost[next]) {
                    request.parents[next] = current;
                    request.gcost[next]   = g;
                    request.pq.push({ g + heuristic(next,goal), next });
                }
            }
            stepsTaken++;
        }

        if (request.pq.empty()) request.state = FAILED;
        // else: still SEARCHING -> resume next frame, no work lost

  ---- ONE REQUEST OVER FRAMES (budget = 3 expansions/frame) ----

      frame   state        what happened                    pq state
      -----   ----------   -----------------------------    --------
      F1      SEARCHING    startPathRequest + slice x1      more left
      F2      SEARCHING    slice x2 (2 more pops)           more left
      F3      SEARCHING    slice x3 -> GOAL popped!         FinalPath!
                     state = FOUND
      F4      FOUND        enemy walks FinalPath[0]
      ...     FOUND        enemy walks the rest of the path
      Fn      IDLE         path consumed -> ready to replan

  ---- VS A SINGLE BLOCKING CALL ----

      blocking (old):              time-sliced (new):
       [########## A* ##########]   F1 |F2 |F3 |F4 |F5 |F6
       all expansions in one frame, each slice <= budget,
       game stalls that frame       game keeps running every frame

  ---- HOW THE ENEMY USES IT (updateChase, L1316-1355) ----

      needsNewPath = state == IDLE  ||  state == FAILED
                     || playerPolygonId != lastKnownPlayerPolygon

      if needsNewPath:  startPathRequest(...);   // resets timer too
      if SEARCHING:     updatePathfindingSlice(req, maxStepsPerFrame);
                        print "Calculating path... (SEARCHING)";
                        return;                  // DON'T move this frame
      if FOUND:         walk FinalPath toward player;
                        when index reaches the end -> state = IDLE
      if FAILED:        next tick's needsNewPath fires again -> retry

  Note the ENEMY STOPS MOVING while SEARCHING (the early return) -- the
  AI visibly "thinks" for a frame or two instead of warping.

  PER-FRAME BUDGET: maxPathfindingStepsPerFrame = 50 (default), loaded
  from JSON -> "system"."max_pathfinding_steps" (section 31).

  ONE-LINE SUMMARY:
  A* moves into its own tiny state machine: start a PathRequest, pop a
  budgeted number of nodes each frame, and let the goal pop some frames
  later -- the game never blocks on one big search.

```
## 17. DISTANCE FORMULAS  (4-dir / 8-dir / NavMesh)
```

  The heuristic is only as good as the distance model it assumes.
  Pick the formula that MATCHES how movement actually works.

  ---- TWO GRID MOVEMENT MODELS ----

   4-DIRECTIONAL (Manhattan)                8-DIRECTIONAL (Octile / Chebyshev)

          ↑ (step=1)                         ↖(√2)   ↑ (1)   ↗(√2)
          │                                    \      │      /
        ← o →   (step=1)                       ← o →       (1)
          │                                     ↙(√2)   ↓ (1)   ↘(√2)
          ↓
      diagonal NOT allowed                 diagonal ALLOWED, cost √2 * step
      => Manhattan distance                => Octile distance
                                           (if diagonals cost the SAME as
                                            a side -> Chebyshev distance)

  ---- THE FORMULAS ----

   MANHATTAN      (admissible for the 4-directional model only)
        d = |x1 - x2| + |y1 - y2|                                 (2D)
        d = |x1 - x2| + |y1 - y2| + |z1 - z2|                     (3D)

   OCTILE         (admissible for 8-directional, diagonal cost = √2)
        dx = |x1 - x2|    dy = |y1 - y2|
        d  = max(dx, dy)  +  (√2 - 1) * min(dx, dy)

   CHEBYSHEV      (admissible when diagonal cost == side cost)
        d = max(|x1 - x2|, |y1 - y2|)                             (2D)
        d = max(|x1 - x2|, |y1 - y2|, |z1 - z2|)                  (3D)

   EUCLIDEAN      (admissible whenever free straight-line movement)
        d = sqrt( (x1-x2)^2 + (y1-y2)^2 )                         (2D)
        d = sqrt( (x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2 )             (3D)

   N-DIM WAYPOINTS   (a 4D waypoint just adds one more axis)
        Manhattan 4D:  d = |dx| + |dy| + |dz| + |dw|
        Euclidean 4D:  d = sqrt(dx^2 + dy^2 + dz^2 + dw^2)
        (an 8D waypoint extends the same pattern to 8 coordinates)

  ---- NAVMESH DISTANCE (what BIG-ONE.cpp actually does) ----

   A NavMesh has no grid, so there are two distances at play:

        HEURISTIC   = Euclidean between polygon CENTROIDS
                      h(from, goal) = | centroid(from) - centroid(goal) |

        ACTUAL COST = the sum of the edge weights A* walks across
                      cost([0,1,3]) = cost(0->1) + cost(1->3) = 10 + 10 = 20

        diagram of a 2-polygon move (heuristic vs true cost):

                 [0] ----------> [1]
                  \  h = 10         /
                   \               /
                    \             /
   true path 0->1->3 = 10 + 10   \
       (edges actually walked)     \  h(0,3) = 15  <- straight line,
       0->1:10  1->3:10  = 20      \  THROUGH empty space, not on a polygon

  ---- WORKED EXAMPLES on the real mesh centroids ----

        p0 = ( 5,  5,  5)      p1 = (15,  5,  5)
        p2 = ( 5, 15,  8)      p3 = (15, 15, 10)

   formula           pair     deltas         math                    result
   ----------------- ------   -------------  ----------------------  -------
   Manhattan (3D)    0 -> 3   10,10,5       10+10+5                  25.00
   Manhattan (3D)    0 -> 2     0,10,3        0+10+3                  13.00
   Manhattan (3D)    1 -> 3     0,10,5        0+10+5                  15.00
   Euclidean (3D)    0 -> 3   10,10,5       sqrt(100+100+25)         15.00
   Euclidean (3D)    0 -> 2     0,10,3       sqrt(0+100+9)           10.44
   Euclidean (3D)    1 -> 3     0,10,5       sqrt(0+100+25)          11.18
   Octile    (2D)   0 -> 3   10,10         10 + 0.414*10            14.14
   Octile    (2D)   0 -> 2     0,10         max(0,10) + 0.414*min   10.00
   Chebyshev (2D)   0 -> 3   10,10         max(10,10)               10.00
   NavMesh true      0 -> 3   path [0,1,3]  10 + 10                  20.00

   Notice Manhattan (25) is huge vs Euclidean (15): Manhattan can only go
   vertically + horizontally, so it counts the "detour". The NavMesh true
   cost (20) sits between them -- which is why Euclidean stays admissible.

  ---- ADMISSIBILITY TABLE (never overestimate, or A* returns worse) ----

   movement model                       safe heuristics
   -------------------------------      ------------------------------------
   4-directional grid                   Manhattan, Euclidean
   8-directional (diagonal = √2)        Octile, Euclidean
   8-directional (diagonal = 1)         Chebyshev, Euclidean
   free 2D / 3D space                   Euclidean
   NavMesh centroid graph               Euclidean between centroids

   The safest habit: use the CHEAPEST admissible heuristic you can think
   of (Manhattan for grids, Euclidean for navmeshes) -- smaller h means
   A* explores more but never loses optimality.



```
## 18. HPA*  (HIERARCHICAL PATHFINDING A*)
```

  Idea: pre-compute the expensive parts ONCE at load time, then answer
  per-frame queries with a tiny "abstract" search on top.

  THREE LAYERS:
        L1 polygons          the real mesh (what A* walks)
        L2 clusters          groups of polygons (here: rows)
        L3 entrances         doors between clusters = abstract nodes

  ---- STEP A: CLUSTER THE MESH (main(), lines 1486-1487) ----
        addCluster(0, {0, 1});      // bottom row
        addCluster(1, {2, 3});      // top row

        polygonToClusterMap:
          0 -> C0   1 -> C0   2 -> C1   3 -> C1     (O(1) lookups)

         .------C L U S T E R  C1 {2,3}------.
         |        [2] ----15---- [3]          |
         |         |               |          |
         |        10              10          |
         |         |               |          |
         '=============== B O R D E R ========'
         .=============== B O R D E R ========'
         |         |               |          |
         |        10(0-1)     10(1-3)         |
         |        [0] ----37---- [1]          |      <- 37 = the long 0-3
         '------C L U S T E R  C0 {0,1}-------'

  ---- STEP B: FIND ENTRANCES (findEntrances(), lines 250-307) ----
   Scan every polygon, look up each neighbor's cluster (O(1) via the map).
   Any edge that CROSSES clusters becomes an entrance at the midpoint of
   the two centroids.

        crossing edge    entrance id    position (midpoint)
        --------------   -----------    -----------------------
           0 - 3            E0            (10, 10, 7.5)
           1 - 2            E1            (10, 10, 6.5)
           1 - 3            E2            (15, 10, 7.5)

        each entrance is added to BOTH clusters' entranceids lists:
        C0.entrances = {E0, E1, E2}     C1.entrances = {E0, E1, E2}
        (duplicate pairs are de-duplicated before pushing)

  ---- STEP C: INTRA-CLUSTER PATHS (computeIntraPaths(), lines 309-346) ----
   For every PAIR of entrances inside one cluster, run A* once and cache
   the path + cost. Paths are stored both directions (Ea<->Eb).

        CLUSTER C0 {0,1}         cost      edge(s) walked
        E0 <-> E1                 10        0 - 1
        E0 <-> E2                 10        0 - 1
        E1 <-> E2                  0        1 (same polygon)

        CLUSTER C1 {2,3}         cost      edge(s) walked
        E0 <-> E1                 15        3 - 2
        E0 <-> E2                  0        3 (same polygon)
        E1 <-> E2                 15        2 - 3

   (read the cost with `cost = dist[goalpoly]` IMMEDIATELY before the next
    aStar() call -- dist[] is reused, line 331)

  ---- STEP D: THE ABSTRACT GRAPH ----
   Entrances become nodes; an edge exists between any two entrances that
   share a cluster, weighted by the intra-cluster cost. A* runs on THIS
   tiny graph when a query comes in.

            abstract nodes:      E0   E1   E2
            abstract edges:
                                   cost in C0   cost in C1
              E0 - E1                 10            15
              E0 - E2                 10             0
              E1 - E2                  0            15

drawing (each edge = the pair, annotated with both cluster costs):

                      10 / 15  (cost in C0 / cost in C1)
           E0 ───────────────────────────── E1
           │                                │
           │                                │
           │                                │
    10 / 0 │                                │   0 / 15
  (C0 / C1)│                                │ (C0 / C1)
           │                                │
           │                                │
           └─────────────── E2 ─────────────┘

        E0-E1 : 10 (C0)  or 15 (C1)
        E0-E2 : 10 (C0)  or  0 (C1)
        E1-E2 :  0 (C0)  or 15 (C1)

   At query time A* picks whichever cluster cost is relevant to the
   entrance pair it is expanding.

---- STEP E: QUERY TIME -- hpaStar(start, goal) (lines 371-606) ----

   GUARD 1: if start == goal                      -> return {start}
   GUARD 2: if either polygon has no cluster      -> fall back to plain A*
   GUARD 3: if start and goal are in the SAME cluster -> plain A* directly

   Otherwise (different clusters) run the abstract search.

   TRACE: hpaStar(0, 3)   (start = C0, goal = C1; goal cluster center
                            = average of centroids 2,3 = (10, 15, 9))

   abstract heuristic: Euclidean(entrancePos, goalClusterCenter)
        h(E0) = |(10,10,7.5)-(10,15,9)| = 5.22
        h(E1) = |(10,10,6.5)-(10,15,9)| = 5.59
        h(E2) = |(15,10,7.5)-(10,15,9)| = 7.23

   1. seed the queue: for each START-cluster entrance, A* from polygon 0
                      to that entrance's polygon inside C0
        E0 -> poly 0 : aStar(0,0) cost 0  -> g=0,   f=0+5.22   =  5.22
        E1 -> poly 1 : aStar(0,1) cost 10 -> g=10,  f=10+5.59  = 15.59
        E2 -> poly 1 : aStar(0,1) cost 10 -> g=10,  f=10+7.23  = 17.23

   2. pop {5.22, E0}:
        E0 touches the GOAL cluster (C1) -> it is a goal entrance
        run A* on its goal side: poly 3 -> goal 3 = aStar(3,3) cost 0
        total = g(0) + 0 = 0   -> best = 0, bestEntrance = E0
        prune: gCost(0) >= best(0)? YES -> break immediately

      abstract path found: [E0]      (E1 and E2 were never expanded)

   3. STITCH THE FULL PATH (refinement, lines 534-598):

        SEGMENT 1  start -> first entrance poly (inside start cluster)
                   aStar(0, 0) = [0]                      -> final [0]
        SEGMENT 2  single-entrance bridge
                   firstE = E0 -> goal poly inside C1 = poly 3
                   aStar(0, 3) = [0, 1, 3]  append tail    -> [0,1,3]
        SEGMENT 3  last entrance poly -> goal (inside goal cluster)
                   aStar(3, 3) = [3]  append tail          -> [0,1,3]

      HPA* result: [0, 1, 3]  cost 20  (matches plain A* here -- expected,
      HPA* never does worse on a map this small)

   FULL MULTI-ENTRANCE EXAMPLE: hpaStar(0, 2)
        (this one shows the abstract search competing for the best cost)
        pop E0 (g=0)  : goal side aStar(3,2) cost 15 -> total 15
        pop E1 (g=10) : goal side aStar(2,2) cost 0  -> total 10 < 15
                        best = 10; prune: 10 >= 10 -> break
        abstract path [E1] -> stitch:
            aStar(0,1)=[0,1] + intra(1,2)=[1,2] tail + aStar(2,2)=[2]
            => [0,1,2]  cost 10
        (E0's route 0->0->3->2 costs 15; E1's 0->1->2 costs 10 -> E1 wins)

  ---- COMPLEXITY COMPARISON ----

        plain A*:  expands every polygon along the way; on a huge map that
                   is thousands of nodes per query
        HPA*:      the query only searches the ENTRANCE graph (3 nodes here,
                   a few dozen on a huge map); each entrance expansion is
                   a cached O(1) intra-cluster cost. Raw A* runs only 3
                   short local searches while stitching.

   Trade-off: HPA* can be slightly sub-optimal (it never re-checks the
   inside of a cluster mid-flight) but it is dramatically faster, and the
   intra-cluster path memory is paid once at initializeHPA().

```
## 19. PATH SMOOTHING (STRING-PULLING)
```

  A* returns polygon IDs in a chain. The enemy does not need to visit
  every polygon center: if two nodes can "see" each other, everything
  between them can be skipped. smoothPath() (lines 776-789) removes the
  unnecessary middle nodes.

  ---- LINE-OF-SIGHT CHECK (simplified, line 772) ----
        hasLineOfSight(from, to):
            return !isBlocked(from) && !isBlocked(to);
        (no real ray-casting against geometry -- just an open check)

  ---- WALKTHROUGH on aStar(0,3) = [0,1,3] ----

        raw path:    [0] ----> [1] ----> [3]

   smoothed list, current pointer starts at index 0 (polygon 0):

        out: [0]
        current=0:
          lookahead next=1 -> can we see path[2]=poly 3?  LOS(0,3)=TRUE
            -> next=2 (skip past 1 to the far end)
          next=2 is the last node -> stop, append path[2]=3
        out: [0, 3]
        current=2 (last) -> loop ends

        RESULT:  [0, 1, 3]   becomes   [0, 3]

   diagram:

           [1]  x(x(x(x(x  polygon 1 dropped - nothing blocked in sight
            ^  /
            | /
        [0]*<--   straight shot 0 -> 3 allowed
            |
            v
           [3]

  ---- GENERAL RULE (greedy skip) ----
        while current < end:
            next = current + 1
            while next < end AND lineOfSight(current, next+1):
                next++                    // stretch to the farthest
            out.push(path[next])
            if (current == next) break    // no progress -> stop
            current = next

   It is a greedy "make the biggest jump you can see" pass. It does not
   re-route; it only shortens an already-valid path. The funnel algorithm
   (next section) does the same job with real geometry on portals.

```
## 20. FUNNEL ALGORITHM
```

  While smoothPath() simply drops unseen nodes, the funnel algorithm
  produces a smooth line through the actual GAPS (portals) between two
  adjacent polygons. This is the classic "Simple Stupid Funnel" (SSF).

  ---- STEP 1: BUILD PORTALS (getPortals(), lines 791-851) ----
   For each consecutive pair (from, to):
     - find the SHARED EDGE: vertices present in both polygons
       (findSharedEdge compares every vertex against every vertex,
        matching ones within 0.001 distance)
     - portal center = midpoint of the two polygon centroids
     - direction    = (toCenter - fromCenter), normalized
     - perpendicular = dir x up   (up = (0,0,1))
     - portalWidth  = |right-left| / 2
     - left  = midpoint - perp * portalWidth
     - right = midpoint + perp * portalWidth

   On the mesh, path [0,1,3]:
        portal 0-1 : shared edge is the vertical line  x = 10,
                     from (10,0) up to (10,10)
        portal 1-3 : shared edge is the horizontal line y = 10,
                     from (10,10) right to (20,10)

        top-down sketch (0 bottom-left, 1 bottom-right, 3 top-right):

              y=20   +-------------------+
                     ¦         [3]       ¦
              y=15   ¦                   ¦
                     ¦   1-3 doorway --> ¦     <- portal 1-3 (y=10 line)
              y=10   +---[1]----+        ¦
                     ¦      P1          ¦
              y=5    ¦         [0]      ¦
                     ¦      P0          ¦     <- portal 0-1 (x=10 line)
              y=0    +-------------------+
                   x=0                x=20

   The agent walks 0 -> 1 -> 3 through BOTH doorways; the funnel uses
   P0 and P1 as the walls it squeezes between.

  ---- STEP 2: WALK THE FUNNEL (funnelAlgorithm(), lines 859-895) ----
   Keep a cone: apex (current corner), left wall, right wall.

        funnel walls:
              apex ●
                 ╲ │ ╱        <- open cone narrowing as we advance
                ╲  │  ╱
              wall fan │
            left ●  │  right
              ╲       │    ╱
               ╲      │   ╱
          portal ├────┼───┤                   <- a doorway
              ╲       │ ╱
               ╲      │╱
                 ● goal

   Rules, for each portal (leftL, rightR):
     1. if leftL is outside the funnel on the left  -> move left wall to
        leftL (funnel tightens)
     2. if rightR is outside on the right          -> move right wall to
        rightR
     3. if the walls CROSS (left moved past right, or vice versa):
           - the crossing corner becomes a WAYPOINT
           - apex moves there, walls reset to the apex
           - the scan restarts from that portal (rewind i)
   Finish by appending the goal.

   On path [0,1,3] the funnel wings stay open: the straight 0->3 line
   already crosses both doorways, so output = [start(center0), goal3] --
   exactly the same straight shot that section 19 produced.

   For winding corridors the two agents differ:
        smoothPath  : only drops nodes it can see in a straight line
        funnel      : hugs portal corners, producing the SHORTEST POSSIBLE
                      path through the doorways even on an L-shaped corridor

        L-corridor example (funnel follows the corner):

              start *- - - -\
                  |          \
                  |    corner >+--->   <- funnel emit corner as waypoint
                  |          /
                  end *- - - /

```
## 21. DYNAMIC OBSTACLE MANAGEMENT
```

  The map can change WHILE the game runs. Polygon 1 gets blocked mid-game
  (Phase 3 of main(), "Suddenly blocking Corridor!"), forcing the enemies
  to find a new route -- without restarting the whole simulation.

  ---- STORAGE: an unordered_set of blocked polygon ids ----

        .----------------------------.
        |  unordered_set<int>        |
        |  BlockedPolygons            |
        |                            |
        |    +---+                   |
        |    | 1 |   <-- isBlocked(1) returns true
        |    +---+
        '----------------------------'

   API (lines 648-674):
        BlockPolygon(id)      -> BlockedPolygons.insert(id)
        UnBlockedPolygon(id)  -> BlockedPolygons.erase(id)
        BlockChunks({ ids })  -> BlockPolygon() for each id in the list
        UnBlockChunks({ ids })-> UnBlockedPolygon() for each id
        isBlocked(id)         -> id is currently in the set

  ---- HOW A* REACTS (line 732) ----

        while expanding node `current`:
            for each neighbor `next`:
                if (isBlocked(next)) continue;    <- never enter it
                g = dist[current] + cost(current, next)
                ...

   The blocked polygon is not removed -- it is just SKIPPED, so all the
   surrounding ADJACENCY stays valid and the rest of the map is untouched.

---- BEFORE / AFTER on the real mesh (path 0 -> 3) ----

   Both snapshots share this layout (edges: 0-1=10, 0-3=37, 1-2=10,
   1-3=10, 2-3=15):

                        [2] ----15---- [3]
                         |             ▲
                        10           10      <- 1-3 edge = 10
                         |             |
                        [0] ----10---- [1]
                          ╲_ 37( 0-3)_/

   BEFORE, polygon 1 open:            AFTER, BlockChunks({1}):
      A* 0->3 explores 0's            A* 0->3: 0's neighbors = {1,3};
      neighbors {1,3}:                  isBlocked(1) == TRUE -> skip 1
         0->1 (10) then 1->3 (10)      only {3} remains:
         0->3 direct (37) comes        take the long diagonal
         out more expensive            directly.
      => best path  [0,1,3]          => best path  [0,3]
         cost 10 + 10 = 20               cost 37

  ---- SIMULATION IMPACT (Phase 3 vs Phase 5) ----
        Phase 3:  BlockChunks({1})  -> chase reroutes to the direct edge
        Phase 5:  UnBlockChunks({1})->
                  corridor reopens, flee path back to polygon 0 works again

  NOTE ON HPA* + DYNAMIC BLOCKS:
        The precomputed intra-cluster path COSTS (section 18) are NOT
        refreshed when a polygon is blocked. Only the A* calls inside the
        stitching step check isBlocked() at run time. On this tiny mesh
        the effect is invisible; on a big map rebuilding the HPA* tables
        after major blocking keeps the abstract costs correct.

```
## 22. FINITE STATE MACHINE (FSM)  -  enemy AI
```

  The Enemy keeps one AIState and its update functions move between states.
  Constants from the constructor: detectionRange = 20.0, attackRange = 2.5,
  health starts at 100, isHealthLow() is < 30, isDead() is <= 0.

  ---- THE SIX STATES (enum AIState, lines 34-42) ----
        IDLE  PATROL  CHASE  ATTACK  FLEE  DEAD

  ---- STATE DIAGRAM (arrow = transition, label = the trigger) ----

                  patrol list empty
      PATROL ─────────────────────────► IDLE
        │                                 │
        │ isPlayerDetected                │ isPlayerDetected
        │ (dist < 20)                     │ (dist < 20)
        ▼                                 ▼
      CHASE ────────── in attack range ──► ATTACK
      (dist < 2.5)                         │
        │                                  │
        │ isHealthLow (health < 30)        │ isHealthLow (health < 30)
        │                                  │    re-checked every tick
        ▼                                  ▼
      FLEE ◄──────────────────────────────┘
        │
        │  reached polygon 0 AND dist < 0.5
        │  -> health = 100, resume patrol
        └──────────────────────────► (PATROL)

      DEAD:  reachable from ANY state by isDead() (health <= 0).
             It is terminal: updateDead() prints "Game over."

   Real simplification in the CODE: state changes are driven by the
   Behavior Tree (next section), which checks in PRIORITY order:

       1. isDead()        (health <= 0)  -> DEAD     always wins
       2. isHealthLow()   (health < 30)  -> FLEE
       3. player detected + (in range ? ATTACK : CHASE)
       4. otherwise                        -> PATROL

  ---- TRANSITION TABLE (from / to / trigger / code) ----

   from     to       trigger                     condition          line
   -------  -------  -------------------------   -----------------  -----
   IDLE     CHASE    isPlayerDetected            dist < 20         1204
   PATROL   IDLE     patrol path empty           !patrolPath.size() 1210
   PATROL   CHASE    isPlayerDetected            dist < 20         1249
   CHASE    ATTACK   isInAttackRange             dist < 2.5        1104
   any      FLEE     isHealthLow (BT seqFlee)    health < 30       1102/1353
   any      DEAD     isDead (BT seqDead)         health <= 0       1101/1348
   FLEE     DEAD     isDead checked every tick   health <= 0       1101
                                                 (never true in demo)
   FLEE     PATROL   reached flee goal           at polygon 0,     1328-1331
                                                  dist < 0.5
                                                  then health = 100

  ---- SIMULATION PHASES IN main() (frames that drive the FSM) ----

   PHASE   frames   scenario                      state change seen
   ----    ------   ---------------------------   -----------------
   1       1 - 3    player far @ (40,40,10)       PATROL -> PATROL
   2       4 - 7    player @ (15,15,10) in range  PATROL -> CHASE
   3       8 - 10   BlockChunks({1})              CHASE reroutes
   4       11 - 39  "attacks" self-damage 25/tick health drops:
                   updateAttack() does health-=  CHASE -> FLEE (<30)
                   25, so ~3 attacks trigger FLEE  (DEAD never fires in demo:
                                                  FLEE kicks in at <30 and
                                                  health never reaches 0)
   5       escape   UnBlockChunks({1})            FLEE -> PATROL (at 0)


   IDLE IS A DEAD-END IN THIS CODE: updateIdle() exists but the BT never
   calls it -- the root selectors default action is always doPatrol.
   IDLE is only reachable if the external code sets it directly.



```
## 23. BEHAVIOR TREE
```

  The FSM's decisions come from a Behavior Tree executed every tick.
  A tree is a smarter if/else: conditions and actions are NODES wired in
  a hierarchy, evaluated left to right, only ONE winner per tick.

  ---- NODE TYPES (classes in the code) ----

   type       behaves like
   --------   -----------------------------------------------------------
   Condition  ask a yes/no question   -> SUCCESS or FAILURE
   Action     do one thing, always ends SUCCESS
   Sequence   children in order; FAIL on first FAILURE      (AND)
   Selector   children in order; SUCCESS on first SUCCESS   (OR)

   statuses returned:  SUCCESS / FAILURE / RUNNING

  ---- THE ACTUAL TREE (buildBehaviorTree(), lines 1089-1108) ----

                    .--------------------------.
                    |   SELECTOR (root)        |
                    | try children left->right |
                    '------------+-------------'
          .---------------------------|---------------------------.
          v                           v                           v
 .-----------------.       .-----------------.       .-------------------.    .----------.
 |  SEQUENCE 1     |       |  SEQUENCE 2     |       |  SEQUENCE 3       |    |  ACTION  |
 |  "is it dead?"  |       |  "flee?"        |       |  "player seen?"   |    | doPatrol |
 '--------+--------'       '--------+--------'       '--------+---------'    '----------'
          |                          |                          |
    .-----+-----.              .----+----.             .--------+---------.
    v           v              v         v             v                  v
 isDead()    doDead()      isHealthLow  doFlee     isPlayerDetected   (fallback)
 (hp<=0)     -> DEAD       (hp<30)      -> FLEE    (dist<20)
                                                       |
                                                       |  combat branch:
                                                       |  .----------------------------.
                                                       '->|  SELECTOR  "attack first?" |
                                                          '------------+---------------'
                                                              .--------+------.
                                                              v               v
                                                       .----------------.  .----------.
                                                       | SEQUENCE       |  |  ACTION  |
                                                       | "attack?"     |  |  doChase |
                                                       '-------+-------'  |  -> CHASE |
                                                               |          '----------'
                                                     .---------+---------.
                                                     v                   v
                                                isInAttackRange       doAttack
                                                (dist < 2.5)          -> ATTACK

---- PRIORITY ORDER (reads like an if / else-if / else chain) ----

    order   branch            condition            action      result state
    -----   ---------------   -------------------  ----------  ------------
    1       SEQUENCE 1        isDead()             doDead      DEAD
    2       SEQUENCE 2        isHealthLow()        doFlee      FLEE
    3a      SEQUENCE attack   isInAttackRange()    doAttack    ATTACK
    3b      (selector)        player detected      doChase     CHASE
    4       doPatrol          always true          doPatrol    PATROL

  ---- TICK TRACES (what runs each frame) ----

   CASE A: health 50, player at distance 10 (seen, not in attack range)
        root SELECTOR:
          SEQ1 -> isDead()? 50<=0 = false        -> FAILURE, skip
          SEQ2 -> isHealthLow()? 50<30 = false   -> FAILURE, skip
          SEQ3 -> isPlayerDetected()? 10<20 = TRUE ->
              inner SELECTOR:
                SEQ attack: isInAttackRange()? 10<2.5 = false -> FAILURE
                doChase: RUNS  -> state CHASE, updateChase() paths to player
              inner SELECTOR returns SUCCESS
          SEQ3 returns SUCCESS
        root returns SUCCESS   (only doChase executed this tick)

   CASE B: health 20, player at distance 5 (in attack range)
        SEQ1 -> false
        SEQ2 -> isHealthLow()? 20<30 = TRUE -> doFlee RUNS -> FLEE
        (dead and flee out-rank combat -- that is why a hurt enemy stops
         fighting and runs away instead of attacking)

   CASE C: health 0
        SEQ1 -> isDead()? 0<=0 = TRUE -> doDead RUNS -> DEAD (terminal)

   CASE D: health 80, player at distance 50 (out of detection range)
        SEQ1 false, SEQ2 false, SEQ3 false
        -> doPatrol RUNS every tick -> PATROL, walks the patrol loop

   ONE-LINE SUMMARY:
        The root SELECTOR is an if/else-if chain. The first branch whose
        conditions all pass is the only one that acts this tick, and that
        action ALSO sets the FSM state from section 22.

```
## 24. BLACKBOARD SYSTEM
```

  A Blackboard is a shared dictionary of named values that the AI writes
  to and reads from. BIG-ONE.cpp keeps TWO tiers:

        .================================================================.
        |  GLOBAL blackboard (one per game)           <- sharedBb ref   |
        |  holds PLAYER data the enemies are allowed to see            |
        |  Enemies only READ from here                                 |
        '================================================================'
                              ^ reads
          .-------------------.---.------------------------------------.
          |   Enemy 1            |   Enemy 2             Enemy 3       |
          |  personalBB          |  personalBB           personalBB    |
          |  (own health, ammo,  |  (own health, ammo,   (own health,  |
          |   position, isDead)  |   position, isDead)    ammo, ... )  |
          '----------------------'-------------------------------------'
                          each Enemy WRITES only its own personalBB

  ---- STORAGE ----
        data : unordered_map<BlackboardKey, BlackboardValue>
        BlackboardValue = variant<int, float, bool, Vector3, string>
        so one map can hold ints, floats, bools, 3D points and strings.

  ---- KEYS (enum BlackboardKey) ----

   key                value type    kind      written by          read by
   -----------------  -----------   --------  ------------------  --------
   PLAYER_POSITION    Vector3       shared    external/game       Enemy::update
   PLAYER_POLYGON     int           shared    external/game       (reserved)
   ENEMY_POSITION     Vector3       personal  Enemy::update       (reserved)
   ENEMY_HEALTH       float         personal  Enemy::update       (unused in demo)
   ENEMY_AMMO         int           personal  Enemy::update       (reserved)
   IS_DEAD            bool          personal  Enemy::update       (reserved)
   IS_IN_COMBAT       bool          reserved  -                   -
   ALLY_HEALTH        int           reserved  -                   -
   TARGET_POLYGON     int           reserved  -                   -

  ---- ONE update() TICK FLOW (Enemy::update, lines 1408-1429) ----

        +-------------------------------------------------------------+
        |  1. WRITE to own personalBB:                                 |
        |       ENEMY_POSITION = {x,y,z}                               |
        |       ENEMY_HEALTH   = health                                |
        |       IS_DEAD        = (health <= 0)                         |
        |       ENEMY_AMMO     = ammo                                  |
        |                                                              |
        |  2. READ from sharedBB:                                      |
        |       if (sharedBb.has(PLAYER_POSITION))                     |
        |           playerX/Y/Z = sharedBb.get(PLAYER_POSITION)        |
        |                                                              |
        |  3. repath timer bookkeeping                                  |
        |  4. runBT()  -> section 23 tree makes the decision           |
        +-------------------------------------------------------------+

   DEMO NOTE: in this main() the shared blackboard is never filled with
   PLAYER_POSITION -- enemies get player data via setPlayerPosition().
   The Blackboard machinery is scaffolding ready for bigger games.

```
## 25. STEERING BEHAVIORS
```

  Pathfinding says WHERE to go; steering says how to MOVE this frame.
  The steering class exposes pure vector math (normalize, scale, add,
  subtract, truncate) and builds combos of forces.

  ---- BUILDING BLOCKS (Vector3 math) ----
        magnitude(v) = sqrt(vx^2 + vy^2 + vz^2)
        normalize(v) = v / magnitude(v)        (0-vector guards vs div-0)
        truncate(v, max): if |v| > max, scale v down to length max

  ---- SEEK  (chase: steer = desired - velocity) ----
        desired = normalize(target - position) * maxspeed
        return  desired - velocity            <- the correction force

         target ●
                │ desired (already scaled to maxspeed)
                ▼
        agent ● ───────────────────> velocity
                ◄────── steering = desired - velocity

   (on heading straight at target, velocity ~= desired -> steer ~0)

  ---- FLEE  (mirror image of seek) ----
        desired = normalize(position - target) * maxspeed
        return  desired - velocity

         agent ● ◄──── desired  (runs AWAY from target)
                ▲
         target ●

  ---- ARRIVE  (slow down near the goal, updateFlee uses this) ----
        stop = target - position;  dist = |stop|
        if dist > 0.001:
            speed = maxspeed
            if dist < slowingRadius:            <- slowingRadius = 5
                speed = maxspeed * (dist / slowingRadius)
            return desired(speed) - velocity
        else: return zero

        speed
         │▔▔▔▔▔▔▔ maxspeed      (far away: full speed)
         │        ▔▔▔
         │            ▔▔▔      (inside radius: ramp down linearly)
         │                ▔▔▔
         └───────────────────────► distance
                    slowingRadius

  ---- FLOCKING (3 forces, neighborRadius = 15) ----
        separation = for each neighbor within 10: normalize(mine - theirs)
                                                / distance   (push apart)
        alignment  = average(neighborVelocities) - myVelocity  (match heading)
        cohesion   = centerOfMass(neighbors) - myPosition      (stick together)

        steer = separation * 1.5  +  alignment * 1.0  +  cohesion * 1.0

        three agents diagram:
                 A ●
                 ╲  │    separation pushes A-B apart,
                  ╲ │     alignment lines them up,
             B ●   ╲│    cohesion pulls all three together
                ╲    │
                 ╲   │
              C ●────┘      (within neighborRadius 15)

  ---- HOW STATES COMBINE FORCES (updateMovement, lines 1367-1402) ----

        CHASE : seek(target)  +          flocking(neighbors)
        FLEE  : arrive(target)                      (flocking off)
        other : flocking(neighbors) only            (patrol/idle drift)
        DEAD  : velocity = 0                        (no movement)

  ---- INTEGRATION CHAIN (per frame, deltaTime = 0.5 in main) ----
        velocity += steer;                 // accumulate force
        velocity  = truncate(velocity, speed);  // clamp to max speed
        position += velocity * deltaTime;  // integrate
        z = 0.5f;                          // locked to the ground
        velocity *= 0.85f;                 // friction/damping

        force  ->  velocity  ->  position        (the classic 3-step)

  ONE-LINE SUMMARY:
        steering is vector soup: EVERY behavior returns a correction
        force, all forces are weighted and summed, the sum is truncated
        to maxspeed, then integrated into position with damping.

```
## 26. STEERING FORCE BLENDING  (CHASE = seek + flock)
```

   Section 22 showed each behavior in isolation.  This section shows HOW
   the forces are combined per frame for each state, and the vector math
   that makes "blending" work.

   ---- THE FORCE BLENDER (updateMovement, lines 1367-1402) ----

       state   steer = f(...)
       -----   -------------------------------------------------
       CHASE   seek(target)  +  flocking(neighbors)
       FLEE    arrive(target)              (flocking disabled)
       other   flocking(neighbors) only    (patrol / idle drift)
       DEAD    velocity = {0,0,0}          (no output, returns early)

   ---- CHASE: TWO FORCES ADDED (diagram) ----

       seek force          flocking force          steer = seek + flock
       (toward player)     (stay with the group)
                                   
            player ●
               │  seek
               ▼
            ●    ──────────────►  seek  (from section 25)
               │
               ▼
            ● ──► (overlay)   group ●──►  flock (averaged nearby heading)

                 resultant steer ● ──╲
                                      ╲  the weighted SUM of both arrows
                                       ⟶  (vector addition, tip-to-tail)

       ┌──────────────────────────────────────────────────────────┐
       │  The enemy does NOT simply run in a straight line at the │
       │  player.  In a crowd, flocking pulls it toward the pack  │
       │  while seek pulls it toward the target — the blend is a  │
       │  compromise direction (avoids separating from friends).  │
       └──────────────────────────────────────────────────────────┘

   ---- THE MATH PIPELINE for a CHASE frame ----

       INPUT:  target (player pos), velocity, speed=2.0, dt=0.5

       seek   = normalize(target - position) * speed - velocity
       flock  = separation*1.5 + alignment*1.0 + cohesion*1.0
                (only neighbor pairs within neighborRadius=15)
       steer  = seek + flock                     (tip-to-tail add)

       velocity = velocity + steer
       velocity = truncate(velocity, speed)      // clamp length to 2.0
       position = position + velocity * dt       // integrate (dt=0.5)
       z        = 0.5                            // ground clamp
       velocity = velocity * 0.85                // damping

       force → velocity → position  (the classic 3-step chain)

   ---- TRUNCATE (why the speed limit matters) ----

       steering::truncate(v, max):
         if |v| > max:  v = v * (max / |v|)      // scale DOWN to exactly max
         else:          v = v                    // leave as-is

       Before truncate:  velocity length might be 3.4 (too fast)
       After truncate:   length is clamped to speed = 2.0

       ┌──────────────────────────┐
       │       /\  length 3.4     │
       │      /  \    →  truncate │
       │     /    ┊  →  length 2.0│
       │    /      │              │
       │   └───────┘              │
       └──────────────────────────┘

   ---- DAMPING (why enemies don't skid forever) ----

       velocity = velocity * 0.85  every frame.

       With dt=0.5 and friction 0.85, velocity decays quickly:
         after 1 frame:   v * 0.85
         after 2 frames:  v * 0.72
         after 5 frames:  v * 0.44
       Steady-state: the enemy reaches a "terminal" speed where damping
       balances the steering force.  Result: smooth gliding, no eternal
       drift when the target is reached.

   ---- FLEE CASE (arrive only, flocking off) ----

       arrive(target, ...):
         dist = |target - position|
                           _____________________________________
         speed = dist > slowingRadius(5) ? maxspeed            │
                                      : maxspeed * dist/5      │  linear ramp-down
                            (inside 5 units: slow down smoothly)│
         return desired - velocity

       While fleeing, the enemy ONLY cares about reaching polygon 0 —
       it ignores friends entirely.  The flock force would pull it back
       toward the pack (toward danger), so it's disabled for FLEE.

   ---- ONE-FRAME NUMERIC CHECK (corner case: e0 CHASE) ----

       e0 pos (5,5,0.5), player (15,15,10)
       target - position = (10, 10, 9.5), |..| = sqrt(100+100+90.25) ~ 17.03
       seek* = normalize(...) * 2 = (10/17.03, 10/17.03, 9.5/17.03)*2
            ≈ (1.17, 1.17, 1.12)      // only the +x,+y parts matter (2D-ish)
       *ignoring flocking for clarity — in reality all 4 enemies spawn within
        3.2-4.1 units of each other (5,5 / 8,6 / 4,9 / 9,4 — nearest pair is
        sqrt(9+1)=3.16), far inside neighborRadius=15, so flocking is ALWAYS
        active at spawn and the real blended steer differs from this trace.

       steer = seek*;  velocity was ~0 → velocity ≈ (1.17, 1.17, 1.12)
       |velocity| ≈ sqrt(1.37+1.37+1.25) = sqrt(3.99) ≈ 2.0 → just under max
       position += velocity * 0.5 → moves ~1.0 units toward the player.
       velocity *= 0.85 → next frame starts slower, re-accelerates.

       This produces steady closing speed that never exceeds `speed`.

   ---- WHY THE DEMO LOOKS "CORRECT" EVEN WITHOUT FUNNEL ----

       Steering + 0.5 threshold arrival + repath timer hide the fact that
       movement ignores funnel points (section 28).  The enemy smoothly
       walks center-to-center, and the arrival overlap at each door makes
       the path look continuous.  Only fine-grained "cut the corner"
       behavior is missing.


```
## 27. THE PER-FRAME PIPELINE  (update vs updateMovement)
```

   In main() each enemy frame is TWO separate calls — a DECISION half and a
   MOTION half — with main() choosing the steering target between them:

       e->update(deltaTime)            <- DECISION HALF
       target = ...                     <- main() picks target
       e->updateMovement(target, dt)   <- MOTION HALF

   ---- THE TWO HALVES (flow diagram) ----

        .----------- DECISION: e->update(dt) ------------------------.
        |                                                             |
        |  1. WRITE to personalBB:                                    |
        |       position = {x,y,z}, health, isDead, ammo              |
        |                                                             |
        |  2. READ from sharedBB if PLAYER_POSITION key exists         |
        |     (demo uses setPlayerPosition() instead — see note)       |
        |                                                             |
        |  3. REPATH TIMER bookkeeping:                                |
        |       if CHASE  -> timeSinceLastRepath += deltaTime          |
        |       else      -> timeSinceLastRepath  = repathCooldown     |
        |                      (0.5, so the timer is "armed" every     |
        |                       time the enemy leaves CHASE)           |
        |                                                             |
        |  4. runBT()  ->  section 23 tree picks exactly ONE action    |
        |       that action ALSO sets the FSM state (section 22)       |
        |       [doPatrol/doChase/doFlee/doAttack/doDead]              |
        |                                                             |
        '-------------------------------------------------------------'
                              |
                   main() picks target based on state:
                   ┌───────────────────────────────────────┐
                   │  state    target                       │
                   │  ------   ------                       │
                   │  CHASE    {playerX, playerY, playerZ}  │
                   │  other    polygon center (current poly) │
                   └───────────────────────────────────────┘
                              |
        .----------- MOTION: e->updateMovement(target, dt) ----------.
        |                                                             |
        |  1. collect neighbors (positions + velocities) within 15     |
        |     (skip self + DEAD enemies, section 26)                   |
        |                                                             |
        |  2. blend forces (per state, section 26):                    |
        |       CHASE -> seek + flocking                               |
        |       FLEE  -> arrive                                       |
        |       other -> flocking only                                |
        |                                                             |
        |  3. velocity += steer                                       |
        |     velocity  = truncate(velocity, speed=2.0)               |
        |                                                             |
        |  4. position += velocity * deltaTime                        |
        |     z = 0.5f  (locked to ground)                            |
        |                                                             |
        |  5. velocity *= 0.85  (damping/friction)                    |
        '-------------------------------------------------------------'

   NOTE ON BLACKBOARD WRITE:  the demo never calls sharedBb.set(PLAYER_POSITION)
   anywhere — instead it uses setPlayerPosition() directly.  The Blackboard code
   (section 24) is ready for future scaling; in a real game the external system
   would write to the shared blackboard and enemies would read it during update().

   NOTE ON A*/FUNNEL IN THE DEMO:   updatePatrol() runs the full chain
   aStar + smoothPath + funnelAlgorithm but stores only currentPath (the
   smoothed polygon list); the funnel output is computed and discarded.
   main() drives the enemy toward polygon CENTERS, not funnel waypoints.
   updateChase() never calls funnelAlgorithm at all -- it goes straight
   from smoothPath into currentPath.  See sections 28 and 29 for detail.

   ---- WHO CALLS WHAT (call chain per frame) ----

        main()
        ├── e->update(dt)
        │   ├── BB write (personal)
        │   ├── BB read  (shared)
        │   ├── timer bookkeeping
        │   └── runBT()
        │       └── root->execute(e)
        │           └── first branch that succeeds:
        │               doPatrol / doChase / doFlee / doAttack / doDead
        │                   ├── sets state
        │                   ├── may call aStar + smooth + funnel
        │                   │   (planning only, stores in currentPath)
        │                   └── returns SUCCESS to root
        │
        ├── main() computes target (see table above)
        │
        └── e->updateMovement(target, dt)
            ├── getNeighborPositions()
            ├── getNeighborVelocities()
            ├── steering::seek / flock / arrive  (blend)
            ├── velocity += steer
            ├── truncate to speed
            ├── integrate position
            └── dampen velocity

   The key insight:  **planning happens in update(); movement happens in
   updateMovement().**  They are decoupled — the BT decides WHERE to go,
   and main() + steering decides HOW to get there this frame.


```
## 28. PATROL WAYPOINT LOOP
```

   An enemy has a list of polygon IDs: patrolPath = { 0, 1, 2, 3 }.
   It walks them in order and loops back to the first via modular wrap.
   Each "leg" (moving from one patrol polygon to the next) triggers a
   fresh A* → smoothPath → funnel chain.

   ---- THE RING (top-down, matching section 21 mesh) ----

              patrolPath
         ┌──── index 0: polygon 0 ────┐
         │                             │
     3 ──┘                             └── 1
         \                             /
          \          ring of          /
           \       4 polygons        /
            \                       /
              ───────── 2 ────────
                    index 2

         patrolIndex cycles: 0 → 1 → 2 → 3 → (wrap) 0 → 1 → ...

   ---- A SINGLE LEG (updatePatrol, lines 1208-1251) ----

   When currentPath is empty or fully consumed:

       start = getCurrentPolygon()    // where am I now
       goal  = patrolPath[patrolIndex]// which polygon to go to next

       rawPath        = aStar(start, goal)
       smoothPolygon  = smoothPath(rawPath)
       funnelPoints   = funnelAlgorithm(smoothPolygon)  // computed, DISCARDED
       currentPath    = smoothPolygon                    // THIS is what we follow
       currentPathIndex = 0

   ---- LEGS ON THE REAL MESH (edge costs repeated) ----

       edges: 0-1:10   0-3:37   1-2:10   1-3:10   2-3:15

       leg       aStar result     cost   smoothed       funnel discarded?
       --------  --------------   ----   ----------     ----------------
       0 → 1     [0, 1]           10     [0, 1]         YES
       1 → 2     [1, 2]           10     [1, 2]         YES
       2 → 3     [2, 3]           15     [2, 3]         YES
       3 → 0     [3, 1, 0]        20     [3, 0]*        YES

       * smoothPath([3,1,0]) :  can polygon 3 see polygon 0?
         hasLineOfSight(3,0) = !isBlocked(3) && !isBlocked(0) = TRUE
         => smoothed to [3, 0]  (section 19: greedy skip drops 1)

   ---- ARRIVAL & WRAP (per-frame, inside updatePatrol) ----

   Each frame: advance toward the current path node's center.

       nextPolygon = currentPath[currentPathIndex]
       center      = getPolygonCenter(nextPolygon)
       dist        = |{center.x - x, center.y - y, center.z - z}|

       if dist < 0.5:                        // arrival threshold
           currentPathIndex++
           if currentPathIndex >= currentPath.size():
               patrolIndex = (patrolIndex + 1) % patrolPath.size()
               currentPath.clear()            // triggers re-plan next tick

   diagram of one frame's arrival:

       BEFORE:  enemy ● ──── 0.6 units ──────►  ● center of polygon 1
                          (not there yet)

        AFTER:   enemy ●                          (moved closer by speed*dt)
                center ●   now within 0.5!
                -> advance, maybe next leg

        CAVEAT (see header DEMO CAVEATS): this arrival uses the 3D distance
        (line 1240).  Since z is locked to 0.5 while every polygon center is
        at z >= 5, dz >= 4.5 keeps the check false forever in a real run:
        checking at 3D, patrol legs never really advance.  Diagram shows the
        intended design.

   ---- WHAT HAPPENS WHEN PATROL PATH IS EMPTY ----

       if patrolPath.empty() -> setState(IDLE)
       IDLE is a dead-end in this demo (BT never calls updateIdle,
       see section 22 "IDLE is a dead-end" note).  The only exit is
       if the code outside directly sets the state.

   ---- QUIRK: FUNNEL OUTPUT IS COMPUTED BUT NEVER USED ----

       In updatePatrol (lines 1220-1221):
         smoothpoints = navMesh.funnelAlgorithm(smoothPolygonpath);
       This local variable is assigned but never read.
       The enemy steers toward polygon CENTERS (via currentPath), not
       toward funnel waypoints.  main() passes the center as the
       updateMovement target (section 27).

       Compare updateChase (line 1272):
         currentPath = navMesh.smoothPath(rawPath);
       Chasing doesn't even compute funnel — same centers-only behavior.

       For a demo where funnel points ARE used, you would need:
         1. Store funnel waypoints in currentPath instead of polygon IDs
         2. Pass the next waypoint (not polygon center) to updateMovement
       This is left as a future enhancement.


```
## 29. WAYPOINT ADVANCE / ARRIVAL THRESHOLD
```

   currentPath is a list of polygon IDs.  Each frame, the enemy tries to
   walk toward the CENTER of the current path node.  When it gets close
   enough, the index advances — "arriving" at that waypoint.

   ---- THE THREE STATE VARIABLES ----

       currentPath       vector<int>     polygon ID chain (e.g. [0, 1, 3])
       currentPathIndex  size_t          which node we're heading toward
       lastValidPolygonId int             last polygon we confirmed arrival in

   ---- PER-FRAME ADVANCE LOGIC (updatePatrol, lines 1234-1246) ----

       nextPolygon = currentPath[currentPathIndex]
       center      = getPolygonCenter(nextPolygon)
       dist        = 3D distance (center, enemy position)
       if dist < 0.5:
           currentPathIndex++
           if currentPathIndex >= currentPath.size():
               patrolIndex = (patrolIndex + 1) % patrolPath.size()
               currentPath.clear()              // empty → next tick re-plans

   diagram of advancing through [0, 1, 3]:

       frame 1:   enemy ● ──── 8.3 ──────► ● center 0 (5,5,5)
                           (walking toward poly 0)

       frame 3:   enemy ●─ 0.4 ─► ● center 0
                   dist = 0.4 < 0.5  →  ARRIVED, advance index
                   currentPathIndex: 0 → 1

       frame 4:   enemy ● ──── 10.0 ────► ● center 1 (15,5,5)
                           (new target: polygon 1)

       frame 6:   enemy ● ── 0.3 ──► ● center 1
                   dist = 0.3 < 0.5  →  ARRIVED, advance index
                   currentPathIndex: 1 → 2

       frame 7:   enemy ● ──── 10.0 ────► ● center 3 (15,15,10)
                   (final target: polygon 3)

       frame 10:  enemy ● ─ 0.45 ─► ● center 3
                   ARRIVED, index 2 → 3 (past end)
                   currentPath.clear()  → next tick re-plans

   ---- CHASE vs PATROL: 2D vs 3D DISTANCE ----

       updatePatrol (line 1240):   3D sqrt(dx^2 + dy^2 + dz^2)
       updateChase  (line 1284):   2D sqrt(dx^2 + dy^2)   <- ignores z
       updateFlee   (line 1324):   3D sqrt(dx^2 + dy^2 + dz^2)

HARD CONSEQUENCE IN THIS DEMO:  z is locked to 0.5 while polygon
        centers are at z = 5/8/10, so dz >= 4.5.  The 3D checks in
        updatePatrol / updateFlee come out >= 4.5 > 0.5 EVERY time: patrol
        never advances and flee never "reaches" polygon 0 to heal.  Only the
        2D chase check (line 1284) can ever fire.  (See section 28 caveat.)
        In a game with height variation (multi-floor buildings), the 2D
        chase check would count enemies as "arrived" even if they are
        vertically misaligned — a potential source of jitter.

   ---- ARRIVAL TRIGGER (distance < 0.5) ----

       ┌──────────────────────────────────────────────────────┐
       │              distance from center of target polygon   │
       │                                                      │
       │  10 ──────  ──────  ──────  ──────  ──────          │
       │            (moving closer each frame)                │
       │  5                                                      │
       │  2                                                      │
       │  0.5 ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄ ARRIVAL THRESHOLD    │
       │  0   ● arrival! index++  (then next polygon center)  │
       └──────────────────────────────────────────────────────┘

   ---- SPECIAL: FLEE HEALS AT POLYGON 0 ----

       updateFlee (lines 1328-1331):

       if start == 0 AND dist < 0.5:
           health = 100
           setState(PATROL)

       The enemy only heals once it reaches polygon 0 and is within 0.5.
       This is the "escape successful" reset — enemy re-enters the patrol
       loop at full health.  If the enemy is at polygon 0 but the path to
       goal 0 is blocked, it falls back to flee steering (line 1309-1311).

   ---- WHAT HAPPENS WHEN currentPathIndex GOES PAST END ----

       currentPathIndex reaches currentPath.size()
       next tick: getCurrentPolygon() returns lastValidPolygonId
       (because currentPath is empty — line 1197)
       updateChase/updatePatrol re-plan a new path from lastValidPolygonId

       lastValidPolygonId is updated ONLY when a node is "arrived" at,
       ensuring the enemy never re-plans from the wrong polygon.

   ---- INCONSISTENCY: arrival in chase doesn't set lastValidPolygonId
        when currentPathIndex goes past end (only when dist < 0.5). ----

       If the last polygon's center is never reached (e.g., enemy is
       blocked by something else), currentPathIndex would stall — but in
       the demo this never happens because movement always closes the gap.


```
## 30. REPATH TIMER & STICKY TARGETS
```

   Enemies don't pathfind every frame.  A cooldown timer + a "player moved?"
   check together decide WHEN to recompute the route.  This avoids wasting
   A* calls when the target is standing still.

   ---- THE TWO VARIABLES ----

       timeSinceLastRepath   float   seconds since the last aStar call
       lastKnownPlayerPolygon int   which polygon the player was in at last repath

   ---- TIMER BOOKKEEPING (update(), lines 1421-1426) ----

       if state == CHASE:
           timeSinceLastRepath += deltaTime         (each frame adds 0.5)
       else:
           timeSinceLastRepath  = repathCooldown    (set to 0.5, "pre-armed")

   ---- REPATH CONDITION (updateChase(), lines 1256-1276) ----

       hasNoPath   = currentPath.empty() || currentPathIndex >= currentPath.size()
       timerExpired = timeSinceLastRepath >= repathCooldown    (>= 0.5)
       playerMoved  = playerPolygonId != lastKnownPlayerPolygon

       IF  hasNoPath  OR  (timerExpired AND playerMoved):
           re-run aStar, reset timer, store new path

   ---- WHY "pre-armed" MATTERS ----

       When the enemy leaves CHASE (e.g., PATROL or FLEE), the timer is
       set to 0.5 immediately.  The moment the enemy re-enters CHASE, the
       timer starts at 0.5 — one frame later (dt=0.5) it hits 1.0, which
       is >= 0.5 → timerExpired is TRUE right away.

       Combined with hasNoPath (path is empty when first entering CHASE),
       the first CHASE frame ALWAYS repaths.  This is intentional — the
       enemy needs a fresh path the instant it decides to chase.

   ---- TIMELINE (e0, deltaTime=0.5, player enters range at frame 4) ----

       frame   state   timer+dt    timer after   hasNoPath? playerMoved?  REPATH?
       -----   -----   --------    -----------   ---------  -----------   -------
       1       PATROL  (else)      0.5 armed     --         --            no
       2       PATROL  (else)      0.5           --         --            no
       3       PATROL  (else)      0.5           --         --            no
       4       PATROL  (else)      0.5           --         --            no
             runBT -> doChase (player at (15,15,10), dist=14.1 < 20)
             updateChase: hasNoPath=T → YES (aStar stored path, timer=0)
       5       CHASE   +=0.5       0.5           F          F            no*
       6       CHASE   +=0.5       1.0           F          F            no
       7       CHASE   +=0.5       1.5           F          F            no
       ...     CHASE   +=0.5       ...           F          F            no
             (player stays in polygon 3 the whole time → playerMoved always false)

       * frame 5: timerExpired=T but playerMoved=F → OR short-circuits

   ---- WHAT MAKES IT REPATH (frame 11+: BlockChunks then player moves) ----

       frame 11: BlockChunks({1})  — polygon 1 blocked
                 BUT playerMoved = FALSE (still polygon 3) → no repath!
                 enemy walks its OLD path even though it goes through 1
                 (updateChase doesn't check isBlocked during walking)

       frame 12: player's polygon changes (hypothetical to polygon 2)
                 timeSinceLastRepath = 6.0 → timerExpired=T
                 playerMoved = T
                 -> REPATH!  new aStar skips blocked 1 → route via 0→3 direct

   ---- THE "STICKY TARGET" EFFECT (diagram) ----

       Enemy ●──────► current path ──────►── eventually ──► player ●

       While the player stays in the same polygon:
       ┌──────────────────────────────────────────────────────────┐
       │  timerExpired = TRUE  but  playerMoved = FALSE           │
       │  => OR condition is FALSE  =>  enemy follows old path     │
       └──────────────────────────────────────────────────────────┘

       Only when the player crosses into a NEW polygon:
       ┌──────────────────────────────────────────────────────────┐
       │  playerMoved = TRUE  and  timerExpired = TRUE            │
       │  => REPATH!  enemy recalculates with updated info        │
       └──────────────────────────────────────────────────────────┘

   ---- EDGE CASE: path runs out (hasNoPath) ----

       if the enemy walks its entire current path without repathing
       (e.g., chasing a stationary player), currentPathIndex eventually
       reaches currentPath.size() → hasNoPath = TRUE → repath fires
       regardless of playerMoved.  This handles the "arrived but player
       hasn't moved" case.

       hasNoPath alone is enough to trigger a repath — playerMoved is
       only needed to prevent redundant re-paths mid-chase.


```
## 31. DATA-DRIVEN CONFIGS (JSON)
```

  AI balance (health, speeds, detection ranges, budgets, patrol routes)
  lives in a .json file, NOT hard-coded in C++.  The game loads it ONCE
  at startup through nlohmann/json, and reads values by key with
  .get<T>().  Tweak a number in the file -> no recompile needed.

  ---- enemy_configs.json (the shape the demo reads) ----

      {
        "system": {
          "max_pathfinding_steps": 50,      budget for time-slicing (§16)
          "repath_cooldown": 0.5,
          "neighbor_radius": 15.0,
          "slowing_radius": 5.0,
          "flee_safe_room_id": 0,
          "delta_time": 0.5,
          "attack_damage": 25.0,
          "low_health_threshold": 25.0,
          "flee_heal_amount": 100.0,
          "starting_ammo": 30
        },
        "spawn": {
          "player_start":  { "x": 40, "y": 40, "z": 10, "polygon": 3 },
          "player_phase2": { "x": 15, "y": 15, "z": 10 },
          "patrol_route": [ 0, 1, 2, 3 ],
          "enemies": [
            { "type": "grunt", "x": 5, "y": 5, "z": 0.5 },
            { "type": "tank",  "x": 8, "y": 6, "z": 0.5 },
            { "type": "scout", "x": 4, "y": 9, "z": 0.5 },
            { "type": "grunt", "x": 9, "y": 4, "z": 0.5 }
          ]
        },
        "enemy_types": {
          "grunt": { "health": 100.0, "speed": 2.0,
                     "detection_range": 20.0, "attack_range": 2.5 },
          "tank":  { "health": 200.0, "speed": 1.2,
                     "detection_range": 15.0, "attack_range": 3.0 },
          "scout": { "health": 60.0,  "speed": 3.5,
                     "detection_range": 30.0, "attack_range": 2.0 }
        }
      }

  (the exact numbers above are illustrative; the loader only needs the
   keys it reads and falls back to defaults when they are missing)

  ---- THE LOAD FUNCTION (loadEnemyConfig, L1510) ----

      call: loadEnemyConfig("enemy_configs.json", "tank")

       1.  ifstream file(filename); if (!file) return defaults;
       2.  json data;  file >> data;              parse whole file
       3.  if !data["enemy_types"].contains("tank")
               -> error + return defaults
       4.  typeData = data["enemy_types"]["tank"]
       5.  config.health         = typeData["health"].get<float>();
           config.speed          = typeData["speed"].get<float>();
           config.detectionRange = typeData["detection_range"].get<float>();
           config.attackRange    = typeData["attack_range"].get<float>();
       6.  return config;

      ASCII view of the dotted lookup:

          data
           |-- "enemy_types"
                 |-- "tank"          <- contains() checked first
                       |-- "health" = 200.0
                       |-- "speed"  = 1.2
                       |-- "detection_range" = 15.0
                       |-- "attack_range"    = 3.0

  ---- THREE ARCHETYPES, ONE LOADER ----

      stat            grunt     tank      scout       what it creates
      -----------     ------    ------    ------      ----------------
      health          100       200       60          tank soaks hits
      speed           2.0       1.2       3.5         scout is fast
      detection       20        15        30          scout sees far
      attack range    2.5       3.0       2.0         tank hits farther

      One C++ function loads them all: only the value DIFFERS per type.
      Add a new archetype = add a JSON entry, zero code changes.

  ---- DATA-DRIVEN SPAWNING (main, L1682) ----

      GameConfig gc      = loadGameConfig("enemy_configs.json");
      json rawJson;  ifstream f("enemy_configs.json");  f >> rawJson;

      for (entry : rawJson["spawn"]["enemies"]) {
          type = entry["type"];                    // "grunt" / "tank" / ...
          ec   = loadEnemyConfig(file, type);      // per-type stats
          spawn Enemy(nav, blackboard, ec, gc);
          setPosition(entry["x"], entry["y"], entry["z"]);
          setPatrolPath(gc.patrolRoute);           // "spawn"."patrol_route"
          setState(PATROL);
      }

  -> The C++ holds almost NO tuning numbers: balance = editing JSON,
     behavior = reading it.

  ---- FALLBACKS (bad files never crash the demo) ----

      file missing / won't open -> EnemyConfig{ 100, 2.0, 20, 2.5 }
      type "tank" absent        -> "Enemy type 'tank' not found in JSON!"
                                   + defaults
      safe reads                -> arrays use contains()/is_null() style
                                   guards before .get<T>()

  ONE-LINE SUMMARY:
  Configuration data moves OUT of source code and INTO JSON; C++ loads
  the file once, drills into nested objects by dotted key path, and
  extracts typed values with .get<T>(), using hard-coded fallbacks when
  a key is missing.
```

## 32. THE 5 SIMULATION PHASES  (main() end-to-end timeline)
```

   main() runs a scripted demo: 5 phases push the enemies through their
   whole AI lifecycle — patrol, chase, dynamic obstacle, combat, escape.
   This section draws the ENTIRE run as one timeline.

   ---- THE BIG TIMELINE (frames along the bottom) ----

        PHASE 1       PHASE 2        PHASE 3        PHASE 4           PHASE 5
        Patrol        Detected       Blocked        Combat            Escape
      ████████      ████████      ████████      ████████████████    ████████
      frames 1-3    frames 4-7    frames 8-10   frames 11-39+       frames 40-44+
        │              │              │              │                │
        ▼              ▼              ▼              ▼                ▼
     player@40,40   player@15,15   BlockChunks   while !allFleeing  UnBlockChunks
     (not in range) (in range)     ({1})         && frame<40        ({1})
     -> PATROL      -> CHASE       -> CHASE       -> CHASE/ATTACK    -> FLEE
                                    reroutes        then FLEE         back to 0

   ---- WHAT CHANGES BETWEEN PHASES (drivers) ----

        driver                   value            effect
        -----------------------  ---------------  -------------------------------
        PH1 playerX/Y/Z          (40,40,10)       dist to enemies ~ 49.5+ > 20
                                                   -> not detected -> PATROL
        PH2 playerX/Y/Z          (15,15,10)       dist ~ 14.1 < 20
                                                   -> detected -> CHASE
        PH3 BlockChunks({1})     poly 1 blocked   A* can't route through 1
                                                   -> use direct 0-3 (37)
        PH4 deltaTime=0.5 loop   combatFrame<40   enemies attack; health drops
                                                   by 25 per updateAttack (bug)
        PH5 UnBlockChunks({1})   poly 1 open     flee path to 0 restored
                                                    -> arrive -> heal 100 -> PATROL *
        * z-lock means arrival at polygon 0 never fires (see header
          DEMO CAVEATS): enemies stay FLEE in a real run.

   ---- FOCUS ON PHASE 4 (the combat loop) ----

       int combatFrame = 11;
       bool allFleeing = false;
       while (!allFleeing && combatFrame < 40) {
           allFleeing = true;                        // assume everyone flees
           for each enemy e:
               e->update(dt);                         // BT: chase/attack/flee
               if (e->getState() != FLEE && e->getHealth() > 0)
                   allFleeing = false;                // someone still fighting

       LOOP EXIT CONDITIONS:
         1. every enemy is FLEE or DEAD  (allFleeing stays true)
         2. combatFrame reaches 40        (hard safety knob)

   ---- THE SELF-DAMAGE SEQUENCE (the bug, step by step) ----

       updateAttack() prints  "ATTACK! -10 HP to player!"   (line 1294)
       ...then runs         health -= 25.0f                  (line 1295)

       So every attack reduces the ENEMY's own health — not the player's.

        health:   100 → 75 → 50 → 25 → FLEE   (health never reaches 0)
                  ATTACK(1) ATTACK(2) ATTACK(3)

       tick 1: health 100 → 75   (48.9% gone, still  > 30 → keeps attacking)
       tick 2: health  75 → 50   (still > 30)
       tick 3: health  50 → 25   (25 < 30 → isHealthLow → FLEE!)

   So after just 3 hits, the enemy abandons combat and runs — "in reality"
   the enemy is tanking its own damage.  The intended code presumably
   subtracted from the PLAYER's health / a global counter.

   ---- FOCUS ON PHASE 5 (escape & heal) ----

       UnBlockChunks({1})                    // corridor 1 reopens
       each frame:
           update() -> runBT -> doFlee -> state = FLEE, updateFlee()
           updateFlee(): aStar(current, 0)   // run back to polygon 0
            on arrival at 0 within 0.5:
            health = 100.0                     // full heal
            setState(PATROL)                   // resume normal patrol

        CAVEAT: arrival at polygon 0 uses a 3D check (dz >= 4.5);
        the z-lock (see header DEMO CAVEATS) prevents it ever firing,
        so in a real run enemies stay FLEE — the loop below is intended
        design, not live behavior.

        enemy μ after escape:  FLEE ──(reach 0, heal)──► PATROL ──► ring loop again

   ---- THE FULL LIFE CYCLE ONE-LINER ----

       PATROL ──► CHASE ──► (block 1) CHASE reroute ──► ATTACK ──(self dmg)──►
        FLEE ──► (unblock, reach 0, heal) ──► PATROL ──► ...  (intended cycle —
        z-lock prevents the heal/re-patrol leg in the runnable demo)


```
## 33. MEMORY MAP & OBJECT LIFECYCLE (heap, RAII, ownership)
```

   This is not a chapter on AI or pathfinding — it shows WHO OWNS WHAT
   in BIG-ONE.cpp, which is the single biggest source of bugs in a C++
   project of this shape (raw `new`/`delete` everywhere).

   ---- THE OWNERSHIP PICTURE (who allocates, who frees) ----

       main()  OWNER of the 4 Enemies:
         std::vector<Enemy*> enemyPtrs;
         for 4 times:  enemyPtrs.push_back(new Enemy(nav, gBb));
         at the end:   delete each e;  enemyPtrs.clear();

       Enemy  OWNER of its Behavior Tree:
         BTNode* root = new Selector({...});         (constructor)
         ~Enemy() { delete root; }                   (destructor)

       Selector / Sequence  OWNER of their children:
         children = std::vector<BTNode*>(nodes);     (copies pointers)
         ~Selector(){ for(auto c : children) delete c; }

       Enemy  BORROWER (non-owning) of:
         NavMesh& navMesh;           <- reference into stack object `nav`
         Blackboard& sharedBlackboard; <- reference into `globalBlackboard`
         std::vector<Enemy*> allEnemies;  <- pointer COPY of the same 4,
                                            only to call neighbor queries
       Enemy  contains (by value): personalBlackboard, velocity, floats...

   diagram:

         main() stack
         ┌──────────────────────────────┐
         │ nav  (NavMesh, one object)   │──┐
         │ globalBlackboard (Blackboard)│──┤ borrowed references
         └──────────────────────────────┘  │ (no ownership, no delete)
                                           │
         heap:                             ▼
         ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐
         │ Enemy [0]   │  │ Enemy [1]  │  │ Enemy [2] │  │ Enemy [3] │
         │  root ──────┼──┼──► root ──┼──┼──► root   │  │  root     │
         │  allEnemies┼──┼──► same 4 (non-owning)    │  │           │
         │  nav&  ────►IN┘  (shared refs)            │  │           │
         └────────────┘  └────────────┘  └────────────┘  └────────────┘
              │                │               │
              ▼                ▼               ▼
         allEnemies points BACK to enemyPtrs[0..3]
         (updated after construction via setEnemyList())

   ---- WHY raw `new` + pointer vectors are dangerous here ----

       Enemy is NOT copyable in a safe way:
         root is a raw owning pointer.
         Rule of Three (destructor + copy ctor + copy assign) is only
         ~half-implemented: there IS a destructor, but NO copy constructor
         and NO copy-assignment operator.

         enemyPtrs.push_back(new Enemy(...)) -- fine, we pass pointers.

         BUT if anyone wrote:  Enemy copy = *enemyPtrs[0];
         -> two Enemy objects now share `root`, and BOTH destructors would
            `delete root` / `delete` through the SAME pointer → double delete.

       Change:

         std::vector<Enemy*> enemyPtrs;   (manual ownership, easy to leak)
         std::vector<std::unique_ptr<Enemy>>  (RAII, auto-freed)

   ---- SAFE ALTERNATIVE (modern C++) ----

         std::vector<std::unique_ptr<Enemy>> enemyPtrs;
         enemyPtrs.push_back(std::make_unique<Enemy>(nav, gBb));
         // no manual delete at the end — vector's dtor handles it

   ---- THE BEHAVIOR TREE'S OWNERSHIP CHAIN (who deletes what) ----

       Enemy::~Enemy()           delete root;           (root Selector)
       root Selector::~Selector  delete each child:     seqDead, seqFlee,
                                                         seqCombat, doPatrol
       each Sequence::~Sequence  delete its condition+action:
                                 seqDead:   delete isDeadCond, doDead...
       -> Result:  every node created with `new` is freed exactly once,
          assuming no copies are made.  This is why it works in the demo
          but would explode the moment someone copies an Enemy.

   ---- BUT: Enemy has no virtual destructor? ----

       BTNode has `virtual ~BTNode()` (line 100)  -> polymorphic delete OK.
       The 4 enemy objects are freed through Enemy* (their concrete type),
       not through base pointers, so root's virtual-ness is what matters.

   ---- WHERE BUGS COME FROM IN THIS FILE (memory part) ----

       - No copy protection on Enemy (root double-delete risk)
       - enemyPtrs uses raw owning pointers (leak risk if not deleted)
       - allEnemies duplicates the same 4 pointers (non-owning alias);
         correct here only because main() cleans up before the vector
         is destroyed.
       - `new Enemy` returns a pointer into the heap; if main() had an
         early `return` (e.g., an exception path), all 4 leaks.

   ---- WHAT "SHOULD" IT LOOK LIKE (dream diagram) ----

         using EnemyPtr = std::unique_ptr<Enemy>;      // owns
         std::vector<EnemyPtr> enemies;                  // owns the 4
         Enemy* self = enemies[0].get();                 // borrow, safe
         std::vector<Enemy*> neighbors;                  // alias, fine
         nav & gBb stay as references (borrowed)

   TEN LINE TAKEAWAY:  "Ownership is whoever calls delete. That must be
   exactly one place (RAII), and all other holders are borrowers."


================================================================================
```
## END OF ASCII ART GUIDE
```
================================================================================
```
