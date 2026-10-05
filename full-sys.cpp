#include <iostream>                 // used for the basic std::couts and cins.
#include <vector>                   // its the dynamic arrays
#include <string>                   // strings included
#include <queue>                    // the priority queue is from here
#include <algorithm>                // used for find reserve functions
#include <chrono>                   // for deltatime
#include <unordered_set>            // Used for std::unordered_set<int> BlockedPolygons. It allows O(1) instant checking of isBlocked(id)
#include <unordered_map>            // Used heavily! polygonToClusterMap, intrapaths, intracosts, and the Blackboard data storage. It’s the "dictionary" that links keys to values instantly.
#include <cmath>                    // Used for std::sqrtf (calculating distances/heuristics) and other math functions.
#include <variant>                  // Very cool usage! This is what allows your Blackboard to hold any of those different data types safely in one place.
#include <limits>                   // This is how you set the initial gCost to "Infinity" so the algorithm knows it hasn't found a path yet.
#include <memory>                   // for the smart pointers
#include <functional>               // std::function, used for the NavMesh event listeners
#include <cstdint>                  // fixed width ints, used for nav versions / subscription ids
#include <nlohmann/json.hpp>
#include <fstream>                  // for file I/O

using json = nlohmann::json;

class Enemy;

struct Vector3 {
    float x, y, z;
};

struct Portal {
    Vector3 left;
    Vector3 right;
};

struct Polygon {
    int id;
    float centerX, centerY, centerZ;
    std::vector<int> neighbors;
    std::vector<float> costs;
    std::vector<Vector3> vertices;
};

enum class AIState {
    IDLE,
    PATROL,
    CHASE,
    ATTACK,
    FLEES,
    FLEE = FLEES,
    DEAD
};

enum class PathState {
    IDLE,
    SEARCHING,
    FOUND,
    FAILED
};

struct PathRequest {
    PathState state = PathState::IDLE;

    int startnode = -1;
    int goalnode = -1;

    // NavMesh::blockVersion captured when the search started. If the world is
    // mutated mid-search (a polygon gets blocked), the version no longer matches
    // and the whole request is thrown away instead of returning a stale path.
    std::uint32_t navVersion = 0;

    std::priority_queue<
        std::pair<float, int>,
        std::vector<std::pair<float, int>>,
        std::greater<std::pair<float, int>>
    > pq;
    std::vector<float> gcost;
    std::vector<int> parents;
    std::vector<int> FinalPath;
};

// A squad is a flat hierarchy with three coordination layers:
//   1. PERCEPTION   - who has seen the player, and do the others trust that report
//   2. COMBAT SIGNAL - who is engaging, who is the current focus target
//   3. SURVIVAL     - ally wounds / deaths and the morale they cause
enum class SquadRole {
    ATTACKER,   // closes on the focus target and deals damage
    SUPPORT,    // flanks, keeps distance, does not steal the focus target
    RETREAT     // heading for the rally polygon, out of the fight
};

struct GameConfig {
    // --- pathfinding / frame budget ---
    int maxPathfindingSteps;
    float repathCooldown;
    float neighborRadius;
    float slowingRadius;
    int fleeSafeRoomId;
    float deltaTime;

    // --- combat ---
    float attackDamage;
    float lowHealthThreshold;
    float fleeHealAmount;
    float playerHealthConfig;
    int startingAmmo;
    float attackInterval;            // seconds between two swings
    float playerDamage;              // return fire the player lands on an attacker
    float arrivalRadius;             // waypoint arrival threshold

    // --- movement / steering blend ---
    float chaseFlockWeight;           // 0 = pure seek, >0 = seek + flocking
    float patrolFlockWeight;
    float groundPlaneZ;              // the plane the agents walk on

    // --- dynamic environment ---
    bool eventDrivenRepath;          // nav events force a repath without cooldown
    bool rebuildHPAOnBlock;          // refresh the HPA* abstraction when the map mutates
    float losSampleStep;             // sampling resolution for the line-of-sight test

    // --- multi-agent coordination ---
    float squadMemoryDuration;       // how long a shared sighting stays trusted
    float allyLowHealthRatio;        // fraction of max health that counts as "wounded"
    float squadRetreatRatio;         // squad panics below this min ally ratio
    int squadPanicDeaths;            // squad panics after this many ally deaths
    int maxAttackers;                // how many squad slots may attack at once
    float supportFlankDistance;      // how far support slots orbit the focus target
    float squadRetreatCooldown;      // re-tasking delay for a squad retreat

    // --- spawn ---
    float playerStartX, playerStartY, playerStartZ;
    int playerStartPolygon;
    float playerPhase2X, playerPhase2Y, playerPhase2Z;
    float playerCombatX, playerCombatY, playerCombatZ;
    std::vector<int> patrolRoute;

    GameConfig();
};

struct EnemyConfig {
    float health;
    float speed;
    float detectionRange;
    float attackRange;

    // data-driven behaviour flags
    bool canFlee;
    bool joinsSquadRetreat;
    bool supportOnly;          // never takes the focus target
    SquadRole preferredRole;

    EnemyConfig(
        float hp = 100.0f,
        float spd = 2.0f,
        float detect = 20.0f,
        float atkRange = 2.5f,
        bool flee = true,
        bool squadRetreat = true,
        bool support = false,
        SquadRole role = SquadRole::ATTACKER);
};

enum class BTNodeStatus {
    SUCCESS,
    FAILURE,
    RUNNING
};

enum class BlackboardKey {
    // ---- shared world truth (written by the game, read by every agent) ----
    PLAYER_POSITION,
    PLAYER_POLYGON,
    PLAYER_HEALTH,
    SIM_TIME,

    // ---- layer 1: shared perception ----
    PLAYER_KNOWN,            // bool   - somebody currently has eyes on the player
    PLAYER_KNOWN_POSITION,   // Vector3 - where they were seen
    PLAYER_KNOWN_TIME,       // float  - sim time of the sighting
    PLAYER_KNOWN_BY,         // int    - which agent produced the sighting

    // ---- layer 2: combat signalling ----
    IS_IN_COMBAT,            // bool  - this agent is engaged
    SQUAD_COMBAT_COUNT,      // int   - how many squad slots are engaged
    SQUAD_FOCUS_TARGET,      // int   - the agent everyone concentrates fire on

    // ---- layer 3: squad survival ----
    SQUAD_ROSTER,            // std::vector<SquadMemberInfo>
    SQUAD_DEAD_COUNT,        // int
    SQUAD_MIN_HEALTH_RATIO,  // float - 1.0 = untouched squad
    ALLY_DEAD_COUNT,         // int   - deaths of OTHERS (this agent excluded)
    ALLY_MIN_HEALTH_RATIO,   // float - worst ally ratio (this agent excluded)
    SQUAD_RETREAT_ORDERED,   // bool  - a squad-wide retreat is in progress
    SQUAD_RETREAT_TARGET,    // int   - rally polygon for that retreat

    // ---- personal / legacy keys ----
    ENEMY_POSITION,
    ENEMY_HEALTH,
    ENEMY_AMMO,
    TARGET_POLYGON,
    ALLY_HEALTH,
    IS_DEAD
};

// One row of the squad roster. Every agent owns exactly one row (matched by
// `agentId`) and refreshes it every tick; the squad-wide layers then fold the
// roster into aggregates instead of overwriting a single shared float, which is
// what made "is an ally dead?" accidentally mean "am I dead?".
struct SquadMemberInfo {
    int agentId = -1;
    float health = 0.0f;
    float maxHealth = 1.0f;
    Vector3 position;
    AIState state = AIState::IDLE;
    SquadRole role = SquadRole::ATTACKER;
    bool canFlee = true;
    bool supportOnly = false;
    bool inCombat = false;
    bool spottedPlayer = false;
    float distanceToPlayer = 0.0f;
    float healthRatio() const {
        return maxHealth > 0.0f ? health / maxHealth : 0.0f;
    }
};

using BlackboardValue = std::variant<int, float, bool, Vector3, std::string,
                                     std::vector<SquadMemberInfo>>;

class Blackboard {
private:
    std::unordered_map<BlackboardKey, BlackboardValue> data;
public:
    template<typename T>
    void set(BlackboardKey key, T value) {
        data[key] = value;
    }

    template<typename T>
    T get(BlackboardKey key) const {
        auto it = data.find(key);
        if (it == data.end()) return T{};

        if (const T* val = std::get_if<T>(&it->second)) {
            return *val;
        }
        return T{};
    }

    // Same as get(), but lets the caller supply the value used when the key was
    // never written, or was written with a different variant alternative.
    template<typename T>
    T getOr(BlackboardKey key, const T& fallback) const {
        auto it = data.find(key);
        if (it == data.end()) return fallback;
        if (const T* val = std::get_if<T>(&it->second)) {
            return *val;
        }
        return fallback;
    }

    // Upserts one roster row, matched by agentId. Each agent calls this for its
    // own id only, so concurrent writers never clobber each other's rows.
    void upsertRosterRow(BlackboardKey key, const SquadMemberInfo& value) {
        std::vector<SquadMemberInfo> rows = get<std::vector<SquadMemberInfo>>(key);
        for (auto& row : rows) {
            if (row.agentId == value.agentId) {
                row = value;
                set<std::vector<SquadMemberInfo>>(key, rows);
                return;
            }
        }
        rows.push_back(value);
        set<std::vector<SquadMemberInfo>>(key, rows);
    }

    std::vector<SquadMemberInfo> getRoster(BlackboardKey key) const {
        return get<std::vector<SquadMemberInfo>>(key);
    }

    bool has(BlackboardKey key) const {
        return data.find(key) != data.end();
    }

    void remove(BlackboardKey key) {
        data.erase(key);
    }

    void clear() {
        data.clear();
    }
};

class BTNode {
public:
    virtual BTNodeStatus execute(Enemy* enemy) = 0;
    // Nodes that carry per-tick state (waits) clear it here so a re-tick after
    // an interruption does not resume with a stale elapsed time.
    virtual void reset() {}
    virtual ~BTNode() = default;
};

class Selector : public BTNode {
private:
    std::vector<std::unique_ptr<BTNode>> children;
public:
    Selector(std::vector<std::unique_ptr<BTNode>>nodes) : children(std::move(nodes)) {}

    BTNodeStatus execute(Enemy* enemy) override {
        for (auto& child : children) {
            BTNodeStatus status = child->execute(enemy);
            if (status == BTNodeStatus::SUCCESS) return BTNodeStatus::SUCCESS;
            if (status == BTNodeStatus::RUNNING) return BTNodeStatus::RUNNING;
        }
        return BTNodeStatus::FAILURE;
    }
};

class Sequence : public BTNode {
private:
    std::vector<std::unique_ptr<BTNode>> children;
public:
    Sequence(std::vector<std::unique_ptr<BTNode>> nodes) : children(std::move(nodes)) {}
    BTNodeStatus execute(Enemy* enemy) override {
        for (auto& child : children) {
            BTNodeStatus status = child->execute(enemy);
            if (status == BTNodeStatus::FAILURE) return BTNodeStatus::FAILURE;
            if (status == BTNodeStatus::RUNNING) return BTNodeStatus::RUNNING;
        }
        return BTNodeStatus::SUCCESS;
    }
};

class ConditionNode : public BTNode {
private:
    bool (Enemy::* condition)();
public:
    ConditionNode(bool (Enemy::* cond)()) : condition(cond) {}
    BTNodeStatus execute(Enemy* enemy) override;
};

class ActionNode : public BTNode {
private:
    void (Enemy::* action)();
public:
    ActionNode(void (Enemy::* act)()) : action(act) {}
    BTNodeStatus execute(Enemy* enemy) override;
};

class TimedWaitNode : public BTNode {
private:
    float waitDuration;
    float elapsed = 0.0f;
    std::string debugLabel;
public:
    TimedWaitNode(float seconds, const std::string& label = "TimedWait")
        : waitDuration(seconds), debugLabel(label) {
    }

    // Defined out of line (after Enemy is complete): the body calls Enemy
    // members, which an incomplete type does not allow.
    BTNodeStatus execute(Enemy* enemy) override;

    void reset() override { elapsed = 0.0f; }
};

// Dynamic-environment node: instead of sleeping a fixed number of seconds it
// returns RUNNING until the agent has actually finished reacting to the last
// navigation event (re-path issued, search complete, route free of blocks). This
// is what makes "wait for the obstacle to clear" event-driven rather than
// time-driven: the wait ends the frame the repath lands, not on a timer.
class PathClearWaitNode : public BTNode {
private:
    float maxWait;
    float elapsed = 0.0f;
    std::string debugLabel;
public:
    PathClearWaitNode(float maxSeconds, const std::string& label = "PathClear")
        : maxWait(maxSeconds), debugLabel(label) {
    }

    BTNodeStatus execute(Enemy* enemy) override;

    void reset() override { elapsed = 0.0f; }
};

struct Entrance {
    int id;
    int polygon1, polygon2;
    int cluster1, cluster2;
    Vector3 position;
    Entrance(int _id, int p1, int p2, int c1, int c2, Vector3 pos) : id(_id),
        polygon1(p1), polygon2(p2),
        cluster1(c1), cluster2(c2),
        position(pos) {
    }
};

struct Cluster {
    int id;
    float centerx, centery, centerz;
    std::vector<int> polygonids;
    std::vector<int> entranceids; // Fixed typo

    std::unordered_map<int, std::unordered_map<int, std::vector<int>>> intrapaths;
    std::unordered_map<int, std::unordered_map<int, float>> intracosts;
};

// ---- navigation event bus -------------------------------------------------
// Every mutation of the walkable set is published so agents can react in the
// same frame instead of discovering the change by re-scanning their path every
// tick (and, before this, waiting out `repathCooldown` first).
enum class NavEventType {
    POLYGON_BLOCKED,
    POLYGON_UNBLOCKED,
    CHUNK_BLOCKED,
    CHUNK_UNBLOCKED
};

struct NavEvent {
    NavEventType type = NavEventType::POLYGON_BLOCKED;
    std::vector<int> polygons;
    std::uint32_t version = 0;   // nav version *after* the mutation
    const char* label() const {
        switch (type) {
        case NavEventType::POLYGON_BLOCKED:  return "POLYGON_BLOCKED";
        case NavEventType::POLYGON_UNBLOCKED:return "POLYGON_UNBLOCKED";
        case NavEventType::CHUNK_BLOCKED:    return "CHUNK_BLOCKED";
        case NavEventType::CHUNK_UNBLOCKED:  return "CHUNK_UNBLOCKED";
        }
        return "UNKNOWN";
    }
};

using NavEventHandler = std::function<void(const NavEvent&)>;

class NavMesh {
private:
    std::vector<Polygon> polygons;
    std::unordered_set<int> BlockedPolygons;

    std::vector<float> dist;
    std::vector<int> parent;
    std::vector<bool> inOpenlist;
    std::vector<bool> inClosedlist;
    int mapsize;

    std::vector<Cluster> clusters;
    std::vector<Entrance> entrances;
    int clusterSize;
    bool hpaInitialized;

    std::unordered_map<int, int> polygonToClusterMap;

    // ---- dynamic-environment bookkeeping ----
    std::uint32_t blockVersion = 0;          // bumped on every block/unblock
    bool rebuildHPAOnBlock = false;          // data-driven: refresh abstraction?
    float losSampleStep = 1.0f;              // data-driven: LOS sampling resolution
    bool hpaDirty = false;                   // abstraction no longer matches the world
    std::unordered_map<std::uint32_t, NavEventHandler> listeners;
    std::uint32_t nextSubscriptionId = 1;

    static float distanceXZ(float ax, float ay, float bx, float by) {
        float dx = ax - bx;
        float dy = ay - by;
        return std::sqrt(dx * dx + dy * dy);
    }

    void publish(NavEventType type, const std::vector<int>& ids) {
        NavEvent ev;
        ev.type = type;
        ev.polygons = ids;
        ev.version = blockVersion;
        for (auto& entry : listeners) {
            entry.second(ev);
        }
    }

public:
    NavMesh() : mapsize(0), clusterSize(2), hpaInitialized(false) {}

    int getMapSize() const {
        return mapsize;
    }

    // ---- event subscription -------------------------------------------------
    std::uint32_t subscribe(NavEventHandler handler) {
        std::uint32_t id = nextSubscriptionId++;
        listeners[id] = std::move(handler);
        return id;
    }

    void unsubscribe(std::uint32_t id) {
        listeners.erase(id);
    }

    std::uint32_t getBlockVersion() const {
        return blockVersion;
    }

    void setDynamicConfig(bool rebuildHPA, float sampleStep) {
        rebuildHPAOnBlock = rebuildHPA;
        losSampleStep = sampleStep > 0.01f ? sampleStep : 1.0f;
    }

    // A search started at version A is meaningless at version B: the world moved
    // underneath it, so the partial open list is discarded instead of returned.
    bool isRequestStale(const PathRequest& request) const {
        return request.navVersion != blockVersion;
    }

    void updatePathfindingSlice(PathRequest& request, int maxSteps) {
        if (request.state != PathState::SEARCHING) return;

        // Dynamic environment: the world changed while this search was in flight.
        if (isRequestStale(request)) {
            request.state = PathState::IDLE;
            request.pq = decltype(request.pq)();
            request.FinalPath.clear();
            return;
        }

        int stepsTaken = 0;

        while (!request.pq.empty() && stepsTaken < maxSteps) {
            float f = request.pq.top().first;
            int current = request.pq.top().second;
            request.pq.pop();

            if (f > request.gcost[current] + heuristic(current, request.goalnode) + 0.01f) {
                continue;
            }

            if (current == request.goalnode) {
                request.FinalPath.clear();
                for (int at = request.goalnode;at != -1;at = request.parents[at]) {
                    request.FinalPath.push_back(at);
                }
                std::reverse(request.FinalPath.begin(), request.FinalPath.end());
                request.state = PathState::FOUND;
                return;
            }

            for (size_t i = 0; i < polygons[current].neighbors.size();i++) {
                int next = polygons[current].neighbors[i];
                float weight = polygons[current].costs[i];

                if (isBlocked(next)) continue;

                float tentativeG = request.gcost[current] + weight;

                if (tentativeG < request.gcost[next]) {
                    request.parents[next] = current;
                    request.gcost[next] = tentativeG;
                    float h = heuristic(next, request.goalnode);
                    request.pq.push({ tentativeG + h,next });
                }
            }
            stepsTaken++;
        }

        if (request.pq.empty()) {
            request.state = PathState::FAILED;
        }
    }

    void startPathRequest(PathRequest& request, int start, int goal, int mapsize) {
        request.state = PathState::SEARCHING;
        request.startnode = start;
        request.goalnode = goal;
        request.FinalPath.clear();
        request.navVersion = blockVersion;   // the search is valid for this world state

        request.pq = decltype(request.pq)();
        request.gcost.assign(mapsize, std::numeric_limits<float>::max());
        request.parents.assign(mapsize, -1);

        request.gcost[start] = 0.0f;
        request.pq.push({ heuristic(start,goal),start });
    }

    void finalizeMap() {
        int maxId = 0;
        for (const auto& p : polygons) {
            maxId = std::max(maxId, p.id);
        }
        mapsize = maxId + 1;
        dist.resize(mapsize);
        parent.resize(mapsize);
        inOpenlist.resize(mapsize, false);
        inClosedlist.resize(mapsize, false);
    }

    void resetbuffers() {
        std::fill(dist.begin(), dist.end(), std::numeric_limits<float>::max());
        std::fill(parent.begin(), parent.end(), -1);
        std::fill(inOpenlist.begin(), inOpenlist.end(), false);
        std::fill(inClosedlist.begin(), inClosedlist.end(), false);
    }

    void addPolygon(int id, float x, float y, float z, const std::vector<Vector3>& verts = {}) {
        polygons.push_back({ id, x, y, z, {}, {}, verts });
    }

    void addCluster(int id, const std::vector<int>& polygonids) {
        Cluster cluster;
        cluster.id = id;
        cluster.polygonids = polygonids;

        float sumx = 0.0f, sumy = 0.0f, sumz = 0.0f;
        for (int polyid : polygonids) {
            Vector3 center = getPolygonCenter(polyid);
            sumx += center.x;
            sumy += center.y;
            sumz += center.z;
            polygonToClusterMap[polyid] = id;
        }

        cluster.centerx = sumx / static_cast<float>(polygonids.size());
        cluster.centery = sumy / static_cast<float>(polygonids.size());
        cluster.centerz = sumz / static_cast<float>(polygonids.size());

        clusters.push_back(cluster);

        std::cout << "Cluster [" << id << "] created at ("
            << cluster.centerx << ", " << cluster.centery << ", " << cluster.centerz
            << ") containing " << polygonids.size() << " polygons: { ";
        for (int pid : polygonids) {
            std::cout << pid << " ";
        }
        std::cout << "}\n";
    }

    void findEntrances() { // Fixed typo
        std::cout << "\n=====================================================\n";
        std::cout << ">>> FINDING CLUSTER ENTRANCES <<<\n";
        std::cout << "=====================================================\n";

        entrances.clear();
        int entranceId = 0;

        for (auto& cluster : clusters) {
            for (int polyid : cluster.polygonids) {
                for (size_t i = 0; i < polygons[polyid].neighbors.size(); i++) {
                    int neighborid = polygons[polyid].neighbors[i];

                    // O(1) Cluster lookup
                    auto it = polygonToClusterMap.find(neighborid);
                    if (it == polygonToClusterMap.end()) continue;
                    int neighborcluster = it->second;

                    // Cross-cluster connection found
                    if (neighborcluster != cluster.id) {
                        Vector3 pos1 = getPolygonCenter(polyid);
                        Vector3 pos2 = getPolygonCenter(neighborid);
                        Vector3 entrancepos = {
                            (pos1.x + pos2.x) / 2.0f,
                            (pos1.y + pos2.y) / 2.0f,
                            (pos1.z + pos2.z) / 2.0f
                        };

                        bool exists = false;
                        for (const auto& e : entrances) {
                            if ((e.polygon1 == polyid && e.polygon2 == neighborid) ||
                                (e.polygon1 == neighborid && e.polygon2 == polyid)) {
                                exists = true;
                                break;
                            }
                        }

                        if (!exists) {
                            Entrance e(entranceId++, polyid, neighborid, cluster.id, neighborcluster, entrancepos);
                            entrances.push_back(e);

                            cluster.entranceids.push_back(e.id);
                            clusters[neighborcluster].entranceids.push_back(e.id);

                            std::cout << "  Entrance [" << e.id << "]: "
                                << "Polygon " << polyid << " (Cluster " << cluster.id << ") "
                                << "<-> Polygon " << neighborid << " (Cluster " << neighborcluster << ")\n";
                            std::cout << "    Position: (" << entrancepos.x << ", "
                                << entrancepos.y << ", " << entrancepos.z << ")\n";
                        }
                    }
                }
            }
        }

        std::cout << "\nTotal Entrances Created: " << entrances.size() << "\n";
        std::cout << "=====================================================\n\n";
    }

    void computeIntraPaths() { // Fixed typo
        std::cout << "\n=====================================================\n";
        std::cout << ">>> COMPUTING INTRA-CLUSTER PATHS <<<\n";
        std::cout << "=====================================================\n";

        for (auto& cluster : clusters) {
            std::cout << "Cluster [" << cluster.id << "]:\n";

            for (size_t i = 0; i < cluster.entranceids.size(); i++) {
                for (size_t j = i + 1; j < cluster.entranceids.size(); j++) {
                    int entrance1id = cluster.entranceids[i];
                    int entrance2id = cluster.entranceids[j];

                    Entrance& e1 = entrances[entrance1id];
                    Entrance& e2 = entrances[entrance2id];

                    int startpoly = (std::find(cluster.polygonids.begin(), cluster.polygonids.end(), e1.polygon1) != cluster.polygonids.end()) ? e1.polygon1 : e1.polygon2;
                    int goalpoly = (std::find(cluster.polygonids.begin(), cluster.polygonids.end(), e2.polygon1) != cluster.polygonids.end()) ? e2.polygon1 : e2.polygon2;

                    std::vector<int> path = aStar(startpoly, goalpoly);

                    if (!path.empty()) {
                        float cost = dist[goalpoly]; // Read immediately before next aStar call!

                        cluster.intrapaths[entrance1id][entrance2id] = path;
                        cluster.intrapaths[entrance2id][entrance1id] = path;
                        cluster.intracosts[entrance1id][entrance2id] = cost;
                        cluster.intracosts[entrance2id][entrance1id] = cost;

                        std::cout << "  Entrance " << entrance1id << " <-> Entrance " << entrance2id
                            << " | Cost: " << cost << " | Path Length: " << path.size() << "\n";
                    }
                }
            }
            std::cout << "\n";
        }
        std::cout << "=====================================================\n\n";
    }

    void initializeHPA() { // Fixed typo
        std::cout << "\n";
        std::cout << "=====================================================\n";
        std::cout << "         INITIALIZING HPA* SYSTEM                    \n";
        std::cout << "=====================================================\n";

        if (clusters.empty()) {
            std::cout << "ERROR: No clusters defined! Use addCluster() first.\n";
            return;
        }
        findEntrances();
        computeIntraPaths();

        hpaInitialized = true;
        hpaDirty = false;

        std::cout << "=====================================================\n";
        std::cout << " HPA* INITIALIZATION COMPLETE!\n";
        std::cout << "   - Clusters: " << clusters.size() << "\n";
        std::cout << "   - Entrances: " << entrances.size() << "\n";
        std::cout << "   - System Ready for Hierarchical Pathfinding\n";
        std::cout << "=====================================================\n\n";
    }

    std::vector<int> hpaStar(int start, int goal) { // Fixed typo & completely rewritten
        std::cout << "\n=====================================================\n";
        std::cout << ">>> HPA* PATHFINDING: " << start << " → " << goal << " <<<\n";
        std::cout << "=====================================================\n";

        if (!hpaInitialized) {
            std::cout << "ERROR: HPA* not initialized! Falling back to regular A*\n";
            return aStar(start, goal);
        }

        // The abstraction was computed against an older walkable set: rebuild it
        // before trusting its costs, otherwise a stale abstract edge routes the
        // agent straight through a freshly blocked polygon.
        if (rebuildHPAOnBlock) {
            refreshHPAIfStale();
        }

        if (start == goal) return { start };

        auto startIt = polygonToClusterMap.find(start);
        auto goalIt = polygonToClusterMap.find(goal);

        if (startIt == polygonToClusterMap.end() || goalIt == polygonToClusterMap.end()) {
            std::cout << "ERROR: Start or goal polygon not in any cluster!\n";
            return aStar(start, goal);
        }

        int startClusterId = startIt->second;
        int goalClusterId = goalIt->second;

        std::cout << "Start Polygon " << start << " is in Cluster " << startClusterId << "\n";
        std::cout << "Goal Polygon " << goal << " is in Cluster " << goalClusterId << "\n";

        if (startClusterId == goalClusterId) {
            std::cout << "Both in same cluster! Using direct A*\n";
            std::cout << "=====================================================\n\n";
            return aStar(start, goal);
        }

        std::vector<int> startEntrances;
        std::vector<int> goalEntrances;

        for (const auto& e : entrances) {
            if (e.cluster1 == startClusterId || e.cluster2 == startClusterId) {
                startEntrances.push_back(e.id);
            }
            if (e.cluster1 == goalClusterId || e.cluster2 == goalClusterId) {
                goalEntrances.push_back(e.id);
            }
        }

        std::cout << "Start Cluster Entrances: ";
        for (int eid : startEntrances) std::cout << eid << " ";
        std::cout << "\nGoal Cluster Entrances: ";
        for (int eid : goalEntrances) std::cout << eid << " ";
        std::cout << "\n\n";

        // --- ABSTRACT GRAPH A* SEARCH ---
        struct AbstractNode {
            int entranceId;
            float gCost;
            float fCost;
            bool operator>(const AbstractNode& other) const {
                return fCost > other.fCost;
            }
        };

        std::priority_queue<AbstractNode, std::vector<AbstractNode>, std::greater<AbstractNode>> abstractPQ;
        std::unordered_map<int, float> abstractG;
        std::unordered_map<int, int> abstractParent;

        auto heuristicEntrance = [&](int eId, int targetClusterId) {
            const Entrance& e = entrances[eId];
            Vector3 eCenter = e.position;
            Vector3 gCenter = getClusterCenter(targetClusterId);
            float dx = eCenter.x - gCenter.x;
            float dy = eCenter.y - gCenter.y;
            float dz = eCenter.z - gCenter.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
            };

        // Initialize PQ with start entrances
        for (int seId : startEntrances) {
            const Entrance& se = entrances[seId];
            int poly = (se.cluster1 == startClusterId) ? se.polygon1 : se.polygon2;
            std::vector<int> p = aStar(start, poly);
            if (!p.empty()) {
                float g = dist[poly]; // Read immediately!
                abstractG[seId] = g;
                abstractParent[seId] = -1;
                abstractPQ.push({ seId, g, g + heuristicEntrance(seId, goalClusterId) });
            }
        }

        int bestGoalEntranceId = -1;
        float bestTotalCost = std::numeric_limits<float>::max();

        while (!abstractPQ.empty()) {
            AbstractNode current = abstractPQ.top();
            abstractPQ.pop();

            // Lazy deletion check
            if (current.fCost > abstractG[current.entranceId] + heuristicEntrance(current.entranceId, goalClusterId) + 0.01f) {
                continue;
            }

            const Entrance& ce = entrances[current.entranceId];
            bool isGoalEntrance = (ce.cluster1 == goalClusterId || ce.cluster2 == goalClusterId);

            if (isGoalEntrance) {
                int goalPoly = (ce.cluster1 == goalClusterId) ? ce.polygon1 : ce.polygon2;
                std::vector<int> p = aStar(goalPoly, goal);
                if (!p.empty()) {
                    float costToGoal = dist[goal]; // Read immediately!
                    float totalCost = current.gCost + costToGoal;
                    if (totalCost < bestTotalCost) {
                        bestTotalCost = totalCost;
                        bestGoalEntranceId = current.entranceId;
                    }
                }
                // Optimization: If current path cost already exceeds best found, stop searching
                if (current.gCost >= bestTotalCost) {
                    break;
                }
                continue;
            }

            // Expand neighbors: other entrances in the SAME cluster(s)
            auto expandCluster = [&](int clusterId) {
                const Cluster& cluster = clusters[clusterId];
                for (int nextEId : cluster.entranceids) {
                    if (nextEId == current.entranceId) continue;

                    auto it1 = cluster.intrapaths.find(current.entranceId);
                    if (it1 == cluster.intrapaths.end()) continue;
                    auto it2 = it1->second.find(nextEId);
                    if (it2 == it1->second.end()) continue;

                    float edgeCost = cluster.intracosts.at(current.entranceId).at(nextEId);
                    float nextG = current.gCost + edgeCost;

                    if (abstractG.find(nextEId) == abstractG.end() || nextG < abstractG[nextEId]) {
                        abstractG[nextEId] = nextG;
                        abstractParent[nextEId] = current.entranceId;
                        abstractPQ.push({ nextEId, nextG, nextG + heuristicEntrance(nextEId, goalClusterId) });
                    }
                }
                };

            expandCluster(ce.cluster1);
            expandCluster(ce.cluster2);
        }

        if (bestGoalEntranceId == -1) {
            std::cout << "No valid entrance path found! Falling back to A*\n";
            std::cout << "=====================================================\n\n";
            return aStar(start, goal);
        }

        std::cout << "\nBest abstract path found! Refining...\n";

        // Reconstruct abstract path
        std::vector<int> abstractPath;
        int curr = bestGoalEntranceId;
        while (curr != -1) {
            abstractPath.push_back(curr);
            curr = abstractParent[curr];
        }
        std::reverse(abstractPath.begin(), abstractPath.end());

        // --- PATH REFINEMENT (STITCHING) ---
                // --- PATH REFINEMENT (STITCHING) ---
        std::vector<int> finalPath;

        int firstEId = abstractPath.front();
        const Entrance& firstE = entrances[firstEId];
        int startPoly = (firstE.cluster1 == startClusterId) ? firstE.polygon1 : firstE.polygon2;

        std::vector<int> p1 = aStar(start, startPoly);
        if (!p1.empty()) {
            finalPath = p1;
        }
        else {
            return aStar(start, goal);
        }

        if (abstractPath.size() == 1) {
            // Single entrance bridges both clusters directly
            int goalPoly = (firstE.cluster1 == goalClusterId) ? firstE.polygon1 : firstE.polygon2;

            std::vector<int> pMid = aStar(startPoly, goalPoly);
            for (size_t j = 1; j < pMid.size(); j++) {
                finalPath.push_back(pMid[j]);
            }

            std::vector<int> p3 = aStar(goalPoly, goal);
            for (size_t j = 1; j < p3.size(); j++) {
                finalPath.push_back(p3[j]);
            }
        }
        else {
            // 2. Between entrances (using precomputed intrapaths)
            for (size_t i = 0; i < abstractPath.size() - 1; i++) {
                int e1Id = abstractPath[i];
                int e2Id = abstractPath[i + 1];

                const Entrance& e1 = entrances[e1Id];
                const Entrance& e2 = entrances[e2Id];
                int sharedClusterId = -1;
                if (e1.cluster1 == e2.cluster1 || e1.cluster1 == e2.cluster2) sharedClusterId = e1.cluster1;
                else if (e1.cluster2 == e2.cluster1 || e1.cluster2 == e2.cluster2) sharedClusterId = e1.cluster2;

                if (sharedClusterId != -1) {
                    const Cluster& cluster = clusters[sharedClusterId];
                    auto it1 = cluster.intrapaths.find(e1Id);
                    if (it1 != cluster.intrapaths.end()) {
                        auto it2 = it1->second.find(e2Id);
                        if (it2 != it1->second.end()) {
                            for (size_t j = 1; j < it2->second.size(); j++) {
                                finalPath.push_back(it2->second[j]);
                            }
                        }
                    }
                }
            }

            // 3. Last entrance to goal
            int lastEId = abstractPath.back();
            const Entrance& lastE = entrances[lastEId];
            int goalPoly = (lastE.cluster1 == goalClusterId) ? lastE.polygon1 : lastE.polygon2;
            std::vector<int> p3 = aStar(goalPoly, goal);
            for (size_t j = 1; j < p3.size(); j++) {
                finalPath.push_back(p3[j]);
            }
        }

        for (int p : finalPath) {
            if (isBlocked(p)) {
                std::cout << "Final HPA* path hits a blocked polygon! Falling back to A*\n";
                std::cout << "=====================================================\n\n";
                return aStar(start, goal);
            }
        }

        std::cout << "Final HPA* Path: ";
        for (int p : finalPath) std::cout << p << " → ";
        std::cout << "Goal\n";
        std::cout << "=====================================================\n\n";

        return finalPath;
    }

    Vector3 getClusterCenter(int id) { // Fixed typo
        for (auto& c : clusters) {
            if (c.id == id) {
                return { c.centerx, c.centery, c.centerz };
            }
        }
        return { 0.0f, 0.0f, 0.0f };
    }

    std::pair<Vector3, Vector3> findSharedEdge(int from, int to) {
        auto& polyA = polygons[from];
        auto& polyB = polygons[to];
        std::vector<Vector3> sharedVerts;

        for (const auto& vA : polyA.vertices) {
            for (const auto& vB : polyB.vertices) {
                float dx = vA.x - vB.x;
                float dy = vA.y - vB.y;
                float dz = vA.z - vB.z;
                float dist = sqrtf(dx * dx + dy * dy + dz * dz);
                if (dist < 0.001f) {
                    sharedVerts.push_back(vA);
                    break;
                }
            }
        }
        if (sharedVerts.size() >= 2) return { sharedVerts[0], sharedVerts[1] };

        Vector3 centerA = { polyA.centerX, polyA.centerY, polyA.centerZ };
        Vector3 centerB = { polyB.centerX, polyB.centerY, polyB.centerZ };
        return { centerA, centerB };
    }

    void addConnection(int from, int to, float cost) {
        for (auto& p : polygons) {
            if (p.id == from) { p.neighbors.push_back(to); p.costs.push_back(cost); }
            if (p.id == to) { p.neighbors.push_back(from); p.costs.push_back(cost); }
        }
    }

    // ---- dynamic obstacle API (now event-driven) ------------------------------
// Each mutation bumps `blockVersion`, marks the HPA* abstraction stale and
// publishes a NavEvent so every subscribed agent re-paths immediately.
    void BlockPolygon(int id, bool notify = true) {
        if (!BlockedPolygons.insert(id).second) return;   // already blocked: no event
        ++blockVersion;
        hpaDirty = true;
        std::cout << "Polygon " << id << " is now Blocked! (nav v" << blockVersion << ")\n";
        if (notify) publish(NavEventType::POLYGON_BLOCKED, { id });
    }

    void UnBlockedPolygon(int id, bool notify = true) {
        if (BlockedPolygons.erase(id) == 0) return;       // was not blocked: no event
        ++blockVersion;
        hpaDirty = true;
        std::cout << "Polygon " << id << " is now UnBlocked! (nav v" << blockVersion << ")\n";
        if (notify) publish(NavEventType::POLYGON_UNBLOCKED, { id });
    }

    // Chunk operations batch the events: one version bump and one notification
    // for the whole chunk, so N polygons do not produce N repath storms.
    void BlockChunks(std::vector<int> ChunkId) {
        std::vector<int> changed;
        for (int id : ChunkId) {
            if (BlockedPolygons.insert(id).second) changed.push_back(id);
        }
        if (changed.empty()) return;
        ++blockVersion;
        hpaDirty = true;
        std::cout << "Chunk of map is now BLOCKED! {";
        for (int id : changed) std::cout << " " << id;
        std::cout << " } (nav v" << blockVersion << ")\n";
        publish(NavEventType::CHUNK_BLOCKED, changed);
    }

    void UnBlockChunks(std::vector<int> ChunkId) {
        std::vector<int> changed;
        for (int id : ChunkId) {
            if (BlockedPolygons.erase(id) > 0) changed.push_back(id);
        }
        if (changed.empty()) return;
        ++blockVersion;
        hpaDirty = true;
        std::cout << "Chunk of map is now UNBLOCKED! {";
        for (int id : changed) std::cout << " " << id;
        std::cout << " } (nav v" << blockVersion << ")\n";
        publish(NavEventType::CHUNK_UNBLOCKED, changed);
    }

    bool isBlocked(int id) const {
        return BlockedPolygons.find(id) != BlockedPolygons.end();
    }

    bool isHPADirty() const { return hpaDirty; }

    // The precomputed intra-cluster costs were built against the old walkable
    // set. Rebuild them lazily (once) before the next hierarchical query, so the
    // abstract graph does not keep routing agents through a blocked polygon.
    void refreshHPAIfStale() {
        if (!hpaDirty || !hpaInitialized) return;
        std::cout << "[HPA*] Abstract graph is stale (world changed) - rebuilding intra-cluster paths\n";
        for (auto& cluster : clusters) {
            cluster.entranceids.clear();
            cluster.intrapaths.clear();
            cluster.intracosts.clear();
        }
        findEntrances();
        computeIntraPaths();
        hpaDirty = false;
    }

    float heuristic(int from, int goal) {
        Vector3 p1 = getPolygonCenter(from);
        Vector3 p2 = getPolygonCenter(goal);
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        float dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    std::vector<int> aStar(int start, int goal) {
        std::vector<int> emptyPath;

        if (isBlocked(start)) {
            std::cout << "  Start polygon " << start << " is BLOCKED!\n";
            return emptyPath;
        }
        if (isBlocked(goal)) {
            std::cout << "  Goal polygon " << goal << " is BLOCKED!\n";
            return emptyPath;
        }

        resetbuffers();

        std::priority_queue<
            std::pair<float, int>,
            std::vector<std::pair<float, int>>,
            std::greater<std::pair<float, int>>
        > pq;

        dist[start] = 0;
        inOpenlist[start] = true;
        pq.push({ heuristic(start, goal), start });

        while (!pq.empty()) {
            float f = pq.top().first;
            int current = pq.top().second;
            pq.pop();

            // Lazy deletion: discard entries superseded by a cheaper route.
            if (f > dist[current] + heuristic(current, goal) + 0.01f) {
                continue;
            }

            if (inClosedlist[current]) {
                continue;
            }

            inClosedlist[current] = true;

            if (current == goal) {
                break;
            }

            for (size_t i = 0; i < polygons[current].neighbors.size(); i++) {
                int next = polygons[current].neighbors[i];
                float weight = polygons[current].costs[i];

                if (isBlocked(next)) {
                    continue;
                }

                if (inClosedlist[next]) {
                    continue;
                }

                float g = dist[current] + weight;

                if (g < dist[next]) {
                    dist[next] = g;
                    parent[next] = current;
                    float h = heuristic(next, goal);

                    // Always re-push on improvement. The old code guarded this with
                    // !inOpenlist[next], but inOpenlist was never cleared for the
                    // lifetime of a search, so a node whose cost improved was never
                    // re-inserted and its stale entry was discarded above,
                    // permanently dropping it from the search.
                    pq.push({ g + h, next });
                    inOpenlist[next] = true;
                }
            }
        }

        if (dist[goal] == std::numeric_limits<float>::max()) {
            return emptyPath;
        }

        std::vector<int> path;
        for (int at = goal; at != -1; at = parent[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());

        return path;
    }

    Vector3 getPolygonCenter(int id) {
        for (auto& p : polygons) {
            if (p.id == id) return { p.centerX, p.centerY, p.centerZ };
        }
        return { 0.0f, 0.0f, 0.0f };
    }

    // Nearest polygon to a world position, resolved on the XZ->XY ground plane.
    // The agent z is snapped to the walk plane, so a 3D nearest-centre search
    // would mismatch; XY is the plane the mesh is actually laid out on.
    int getPolygonAt(float px, float py) const {
        int best = -1;
        float bestDist = std::numeric_limits<float>::max();
        for (const auto& p : polygons) {
            float d = distanceXZ(px, py, p.centerX, p.centerY);
            if (d < bestDist) {
                bestDist = d;
                best = p.id;
            }
        }
        return best;
    }

    int getPolygonAt(const Vector3& pos) const {
        return getPolygonAt(pos.x, pos.y);
    }

    // Real line of sight: walk the straight segment between the two polygon
    // centres at `losSampleStep` intervals and reject the shortcut if any
    // polygon the segment crosses is blocked. The old version only checked the
    // two endpoints, which let smoothPath() cut corners straight through a wall.
    bool hasLineOfSight(int from, int to) {
        if (from == to) return !isBlocked(from);
        if (isBlocked(from) || isBlocked(to)) return false;

        // Adjacent polygons share an edge: the connecting segment is the doorway.
        const Polygon& a = polygons[from];
        for (int nb : a.neighbors) {
            if (nb == to) return true;
        }

        Vector3 p1 = getPolygonCenter(from);
        Vector3 p2 = getPolygonCenter(to);
        float length = distanceXZ(p1.x, p1.y, p2.x, p2.y);
        int steps = static_cast<int>(length / losSampleStep);
        if (steps < 2) steps = 2;

        for (int i = 1; i < steps; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(steps);
            float sx = p1.x + (p2.x - p1.x) * t;
            float sy = p1.y + (p2.y - p1.y) * t;
            int hit = getPolygonAt(sx, sy);
            if (hit < 0 || hit == from || hit == to) continue;
            if (isBlocked(hit)) return false;
        }
        return true;
    }

    // True when every polygon still ahead on the route is walkable. Used by the
    // dynamic-environment node to know a re-path has actually landed.
    bool pathIsClear(const std::vector<int>& path, size_t fromIndex) const {
        for (size_t i = fromIndex; i < path.size(); ++i) {
            if (isBlocked(path[i])) return false;
        }
        return true;
    }

    std::vector<int> smoothPath(const std::vector<int>& path) {
        if (path.size() < 3) return path;
        std::vector<int> smoothed;
        smoothed.push_back(path[0]);
        size_t current = 0;
        while (current < path.size() - 1) {
            size_t next = current + 1;
            while (next < path.size() - 1 && hasLineOfSight(path[current], path[next + 1])) next++;
            smoothed.push_back(path[next]);
            if (current == next) break;
            current = next;
        }
        return smoothed;
    }

    std::vector<Portal> getPortals(const std::vector<int>& path) { // Fixed typo
        std::vector<Portal> Portals;
        for (size_t i = 0; i < path.size() - 1; i++) {
            int from = path[i];
            int to = path[i + 1];
            auto [left, right] = findSharedEdge(from, to);

            float dx = right.x - left.x;
            float dy = right.y - left.y;
            float dz = right.z - left.z;
            float portalwidth = sqrtf(dx * dx + dy * dy + dz * dz) / 2.0f;

            Vector3 fromcenter = getPolygonCenter(from);
            Vector3 tocenter = getPolygonCenter(to);
            Vector3 mid = {
                    (fromcenter.x + tocenter.x) / 2.0f,
                    (fromcenter.y + tocenter.y) / 2.0f,
                    (fromcenter.z + tocenter.z) / 2.0f
            };
            Vector3 dir = {
                    tocenter.x - fromcenter.x,
                    tocenter.y - fromcenter.y,
                    tocenter.z - fromcenter.z
            };
            float len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
            if (len > 0.001f) {
                dir.x /= len;
                dir.y /= len;
                dir.z /= len;
            }

            Vector3 up = { 0.0f, 0.0f, 1.0f };
            Vector3 prep = {
                    dir.y * up.z - dir.z * up.y,
                    dir.z * up.x - dir.x * up.z,
                    dir.x * up.y - dir.y * up.x
            };

            float perplen = sqrtf(prep.x * prep.x + prep.y * prep.y + prep.z * prep.z);
            if (perplen > 0.001f) {
                prep.x /= perplen;
                prep.y /= perplen;
                prep.z /= perplen;
            }

            Portal portal;
            portal.left = {
                    mid.x - prep.x * portalwidth,
                    mid.y - prep.y * portalwidth,
                    mid.z - prep.z * portalwidth
            };
            portal.right = {
                    mid.x + prep.x * portalwidth,
                    mid.y + prep.y * portalwidth,
                    mid.z + prep.z * portalwidth
            };

            Portals.push_back(portal);
        }
        return Portals;
    }

    float cross2D(Vector3 a, Vector3 b, Vector3 c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }
    bool isLeftOfLine(Vector3 a, Vector3 b, Vector3 point) { return cross2D(a, b, point) > 0.0f; }
    bool isRightOfLine(Vector3 a, Vector3 b, Vector3 point) { return cross2D(a, b, point) < 0.0f; }

    std::vector<Vector3> funnelAlgorithm(const std::vector<int>& path) {
        std::vector<Vector3> smoothenPath;
        if (path.size() < 2) {
            for (int id : path) smoothenPath.push_back(getPolygonCenter(id));
            return smoothenPath;
        }

        std::vector<Portal> portals = getPortals(path);
        Vector3 start = getPolygonCenter(path[0]);
        Vector3 goal = getPolygonCenter(path[path.size() - 1]);

        Vector3 apex = start;
        Vector3 left = start;
        Vector3 right = start;
        int leftIndex = 0;
        int rightIndex = 0;
        smoothenPath.push_back(start);

        for (int i = 0; i < static_cast<int>(portals.size()); i++) {
            Vector3 portalleft = portals[i].left;
            Vector3 portalright = portals[i].right;

            if (isLeftOfLine(apex, right, portalleft)) { left = portalleft; leftIndex = i; }
            if (isRightOfLine(apex, left, portalright)) { right = portalright; rightIndex = i; }

            if (isLeftOfLine(apex, left, right)) {
                smoothenPath.push_back(left);
                apex = left; left = apex; right = apex; i = leftIndex; continue;
            }
            if (isRightOfLine(apex, right, left)) {
                smoothenPath.push_back(right);
                apex = right; left = apex; right = apex; i = rightIndex; continue;
            }
        }
        smoothenPath.push_back(goal);
        return smoothenPath;
    }
};

class steering {
public:
    static Vector3 normalize(const Vector3& v) {
        float len = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
        if (len > 0.001f) {
            return { v.x / len, v.y / len, v.z / len };
        }
        return { 0.0f,0.0f,0.0f };
    }

    static float magnitude(const Vector3& v) {
        return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    static Vector3 scale(const Vector3& v, float s) {
        return { v.x * s, v.y * s, v.z * s };
    }

    static Vector3 subtract(const Vector3& a, const Vector3& b) {
        return { a.x - b.x, a.y - b.y, a.z - b.z };
    }

    static Vector3 add(const Vector3& a, const Vector3& b) {
        return { a.x + b.x, a.y + b.y, a.z + b.z };
    }

    static Vector3 truncate(const Vector3& v, float max) {
        float len = magnitude(v);
        if (len > max) {
            return scale(v, max / len);
        }
        return v;
    }

    static Vector3 seek(const Vector3& target, const Vector3& position, const Vector3& velocity, float maxspeed) {
        Vector3 desired = scale(normalize(subtract(target, position)), maxspeed);
        return subtract(desired, velocity);
    }

    static Vector3 flee(const Vector3& target, const Vector3& position, const Vector3& velocity, float maxspeed) {
        Vector3 desired = scale(normalize(subtract(position, target)), maxspeed);
        return subtract(desired, velocity);
    }

    static Vector3 arrive(const Vector3& target, const Vector3& position, const Vector3& velocity, float maxspeed, float slowingRadius) {
        Vector3 desired = subtract(target, position);
        float distance = magnitude(desired);
        if (distance > 0.001f) {
            float speed = maxspeed;
            if (distance < slowingRadius) {
                speed = maxspeed * (distance / slowingRadius);
            }
            desired = normalize(desired);
            desired = scale(desired, speed);

            Vector3 steer = subtract(desired, velocity);
            return steer;
        }
        return { 0.0f,0.0f,0.0f };
    }

    static Vector3 separation(const std::vector<Vector3>& neighbors, const Vector3& position) {
        Vector3 steer = { 0, 0, 0 };
        for (const auto& neighbor : neighbors) {
            Vector3 diff = subtract(position, neighbor);
            float dist = magnitude(diff);

            if (dist > 0.001f && dist < 10.0f) {
                steer = add(steer, scale(normalize(diff), 1.0f / dist));
            }
        }
        return steer;
    }

    static Vector3 alignment(const std::vector<Vector3>& neighbors, const Vector3& velocity) {
        if (neighbors.empty()) return { 0,0,0 };
        Vector3 Avgvelocity = { 0,0,0 };
        for (auto& neighbor : neighbors) {
            Avgvelocity = add(Avgvelocity, neighbor);
        }
        Avgvelocity = scale(Avgvelocity, 1.0f / neighbors.size());
        return subtract(Avgvelocity, velocity);
    }

    static Vector3 cohesion(const std::vector<Vector3>& neighbors, const Vector3& position) {
        if (neighbors.empty()) return { 0.0f, 0.0f, 0.0f };
        Vector3 center = { 0, 0, 0 };
        for (const auto& neighbor : neighbors) {
            center = add(center, neighbor);
        }
        center = scale(center, 1.0f / static_cast<float>(neighbors.size()));
        return subtract(center, position);
    }

    static Vector3 flocking(const std::vector<Vector3>& neighborPositions, const std::vector<Vector3>& neighborVelocities,const Vector3& position, const Vector3& velocity) {
        Vector3 sep = separation(neighborPositions, position);
        Vector3 ali = alignment(neighborVelocities, velocity);
        Vector3 coh = cohesion(neighborPositions, position);

        Vector3 steer = add(scale(sep, 1.5f), add(scale(ali, 1.0f), scale(coh, 1.0f)));
        return steer;
    }
};

class Enemy {
private:
    // ---- identity ----
    int agentId = -1;

    // ---- stats (all data-driven from JSON) ----
    float health;
    float maxHealth;
    float detectionRange;
    float attackRange;
    float speed;

    // ---- kinematics ----
    float x, y, z;
    Vector3 velocity;

    // ---- navigation ----
    NavMesh& navMesh;
    std::vector<int> patrolPath;
    int patrolIndex = 0;
    std::vector<int> currentPath;
    size_t currentPathIndex = 0;
    size_t chasePathIndex = 0;
    PathRequest currentPathRequest;
    int maxPathfindingStepsPerFrame = 50;
    int lastValidPolygonId = 0;

    // ---- dynamic environment (event driven re-pathing) ----
    bool pendingRepath = false;          // set by a NavEvent, cleared by a fresh search
    int pendingEventCount = 0;           // how many nav events we have absorbed
    float timeSinceLastRepath = 0.0f;
    float repathCooldown = 0.5f;
    bool eventDrivenRepath = true;
    std::uint32_t navSubscription = 0;
    std::string lastRepathReason = "none";

    // ---- player / perception ----
    AIState currentState = AIState::IDLE;
    float playerX = 0.0f, playerY = 0.0f, playerZ = 0.0f;
    int playerPolygonId = -1;
    int lastKnownPlayerPolygon = -1;
    float simTime = 0.0f;
    bool personallySpotted = false;       // this agent's own eyes
    bool investigatedLastKnown = false;   // walking to a shared sighting

    // ---- squad (multi-agent coordination) ----
    SquadRole currentRole = SquadRole::ATTACKER;
    bool canFlee = true;
    bool joinsSquadRetreat = true;
    bool supportOnly = false;
    int focusTargetId = -1;
    float attackCooldownTimer = 0.0f;
    float attackInterval = 1.0f;
    bool attackSwingThisTick = false;

    // ---- steering / behaviour ----
    float slowingRadius = 5.0f;
    float arrivalRadius = 0.5f;
    float chaseFlockWeight = 0.15f;
    float patrolFlockWeight = 1.0f;
    float groundPlaneZ = 0.5f;
    float neighborRadius = 15.0f;
    std::vector<Enemy*> allEnemies;
    std::unique_ptr<BTNode> root;
    Blackboard personalBlackboard;
    Blackboard& sharedBlackboard;

    // ---- damage / economy ----
    int ammo;
    float attackDamage;
    float lowHealthThreshold;
    float fleeHealAmount;
    int fleeSafeRoomId;
    float playerHealth;
    float playerDamage;

    // ---- coordination tuning ----
    float squadMemoryDuration = 3.0f;
    float allyLowHealthRatio = 0.3f;
    float squadRetreatRatio = 0.25f;
    int squadPanicDeaths = 1;
    int maxAttackers = 2;
    float supportFlankDistance = 6.0f;
    float squadRetreatCooldown = 3.0f;
    float timeSinceSquadRetreat = 0.0f;
    bool arrivedAtRally = false;
    std::string roleSignature;         // hysteresis for the role election

    // =====================================================================
    // dynamic environment: react to nav events the frame they are published
    // =====================================================================
    void onNavEvent(const NavEvent& ev) {
        pendingRepath = true;
        ++pendingEventCount;

        // Any cached route is now suspect: drop it so the next tick searches
        // against the new walkable set instead of walking into a wall.
        currentPath.clear();
        currentPathIndex = 0;
        chasePathIndex = 0;
        if (currentPathRequest.state == PathState::FOUND) {
            currentPathRequest.state = PathState::IDLE;
        }
        lastRepathReason = ev.label();
        std::cout << "  [NAV-EVENT] Agent " << agentId << " <- " << ev.label()
            << " {";
        for (int id : ev.polygons) std::cout << " " << id;
        std::cout << " } (nav v" << ev.version << ") -> event-driven repath queued\n";
    }

    // =====================================================================
    // LAYER 1 - PERCEPTION: publish what I see, trust what allies report
    // =====================================================================
    void publishPerception() {
        personallySpotted = isPlayerDetected();

        if (personallySpotted) {
            sharedBlackboard.set<bool>(BlackboardKey::PLAYER_KNOWN, true);
            sharedBlackboard.set<Vector3>(BlackboardKey::PLAYER_KNOWN_POSITION,
                { playerX, playerY, playerZ });
            sharedBlackboard.set<float>(BlackboardKey::PLAYER_KNOWN_TIME, simTime);
            sharedBlackboard.set<int>(BlackboardKey::PLAYER_KNOWN_BY, agentId);
            investigatedLastKnown = false;
        }
        else {
            // Nobody new reported, so age the shared sighting out.
            float seenAt = sharedBlackboard.getOr<float>(BlackboardKey::PLAYER_KNOWN_TIME, -1000.0f);
            if (simTime - seenAt > squadMemoryDuration) {
                if (sharedBlackboard.getOr<bool>(BlackboardKey::PLAYER_KNOWN, false)) {
                    std::cout << "  [PERCEPTION] Agent " << agentId
                        << ": shared sighting is stale (> " << squadMemoryDuration
                        << "s) -> squad loses contact\n";
                }
                sharedBlackboard.set<bool>(BlackboardKey::PLAYER_KNOWN, false);
            }
        }

        personalBlackboard.set<int>(BlackboardKey::ENEMY_AMMO, ammo);
    }

    // Do I act on the player? Either I see it, or a squadmate reported it and
    // the report is still inside the shared-memory window.
    bool isPlayerKnown() {
        if (personallySpotted) return true;
        if (!sharedBlackboard.getOr<bool>(BlackboardKey::PLAYER_KNOWN, false)) return false;
        float seenAt = sharedBlackboard.getOr<float>(BlackboardKey::PLAYER_KNOWN_TIME, -1000.0f);
        return (simTime - seenAt) <= squadMemoryDuration;
    }

    bool isSharedSightingOnly() {
        return isPlayerKnown() && !personallySpotted;
    }

    Vector3 lastKnownPlayerPosition() {
        return sharedBlackboard.getOr<Vector3>(BlackboardKey::PLAYER_KNOWN_POSITION,
            { playerX, playerY, playerZ });
    }

    int whoSpottedPlayer() const {
        return sharedBlackboard.getOr<int>(BlackboardKey::PLAYER_KNOWN_BY, -1);
    }

    // The polygon we are allowed to path to: our own knowledge while we have
    // eyes on the target, otherwise the *reported* position. Never the live
    // player polygon while blind - that would be cheating the perception layer.
    int getTargetPolygon() {
        if (personallySpotted) return playerPolygonId;
        Vector3 last = lastKnownPlayerPosition();
        int p = navMesh.getPolygonAt(last);
        return p >= 0 ? p : lastValidPolygonId;
    }

    Vector3 getTargetWorldPosition() {
        if (personallySpotted) return { playerX, playerY, playerZ };
        if (currentRole == SquadRole::SUPPORT && focusTargetId >= 0) {
            return getSupportAnchor();
        }
        return lastKnownPlayerPosition();
    }

    // =====================================================================
    // LAYER 2 - COMBAT SIGNAL: who is engaged, who leads, who supports
    // =====================================================================
    void publishRosterRow() {
        SquadMemberInfo row;
        row.agentId = agentId;
        row.health = health;
        row.maxHealth = maxHealth;
        row.position = { x, y, z };
        row.state = currentState;
        row.role = currentRole;
        row.canFlee = canFlee;
        row.supportOnly = supportOnly;
        row.inCombat = isEngaged();
        row.spottedPlayer = personallySpotted;
        row.distanceToPlayer = distanceToPlayer();
        sharedBlackboard.upsertRosterRow(BlackboardKey::SQUAD_ROSTER, row);
    }

    // Fold the roster into the aggregates every other layer reads. Aggregates
    // are computed (never written by one agent and hoped to be shared), which is
    // what stops "is an ally dead?" from silently meaning "am I dead?".
    void refreshSquadAggregates() {
        std::vector<SquadMemberInfo> rows = sharedBlackboard.getRoster(BlackboardKey::SQUAD_ROSTER);

        // Drop rows for agents that no longer exist, so the demo can spawn /
        // destroy agents without stale state poisoning the squad.
        std::vector<SquadMemberInfo> live;
        for (const auto& row : rows) {
            bool stillExists = false;
            for (Enemy* other : allEnemies) {
                if (other != nullptr && other->getId() == row.agentId) { stillExists = true; break; }
            }
            if (stillExists) live.push_back(row);
        }
        if (live.size() != rows.size()) {
            sharedBlackboard.set<std::vector<SquadMemberInfo>>(BlackboardKey::SQUAD_ROSTER, live);
            rows = live;
        }

        int combatCount = 0, deadCount = 0, allyDead = 0;
        float minRatio = 1.0f, allyMinRatio = 1.0f;

        for (const auto& row : rows) {
            bool isSelf = (row.agentId == agentId);
            float ratio = row.healthRatio();
            if (row.inCombat) ++combatCount;
            if (row.health <= 0.0f) {
                ++deadCount;
                if (!isSelf) ++allyDead;
            }
            minRatio = std::min(minRatio, ratio);
            if (!isSelf) allyMinRatio = std::min(allyMinRatio, ratio);
        }

        // Role assignment is a pure function of the roster (nearest-to-player
        // gets an attack slot), so every agent independently derives the same
        // answer without a leader having to broadcast it.
        std::vector<SquadMemberInfo> sorted = rows;
        std::sort(sorted.begin(), sorted.end(), [](const SquadMemberInfo& a, const SquadMemberInfo& b) {
            if (a.distanceToPlayer != b.distanceToPlayer) return a.distanceToPlayer < b.distanceToPlayer;
            return a.agentId < b.agentId;
        });

        // Hysteresis: reassigning on every tick makes roles thrash, because the
        // sort key (distance to player) changes by centimetres each frame. Roles
        // are only re-elected when the *shape* of the squad changes. The
        // signature is built in agent-id order so a reshuffle of the distance
        // sort cannot masquerade as a real change.
        std::vector<SquadMemberInfo> canonical = rows;
        std::sort(canonical.begin(), canonical.end(), [](const SquadMemberInfo& a, const SquadMemberInfo& b) {
            return a.agentId < b.agentId;
        });
        std::string signature;
        for (const auto& row : canonical) {
            signature += std::to_string(row.agentId);
            signature += row.inCombat ? "C" : "-";
            signature += row.supportOnly ? "S" : "-";
            signature += row.health <= 0.0f ? "D" : "-";
        }

        if (signature == roleSignature) {
            focusTargetId = sharedBlackboard.getOr<int>(BlackboardKey::SQUAD_FOCUS_TARGET, focusTargetId);
            sharedBlackboard.set<int>(BlackboardKey::SQUAD_COMBAT_COUNT, combatCount);
            sharedBlackboard.set<int>(BlackboardKey::SQUAD_DEAD_COUNT, deadCount);
            sharedBlackboard.set<float>(BlackboardKey::SQUAD_MIN_HEALTH_RATIO, minRatio);
            sharedBlackboard.set<int>(BlackboardKey::ALLY_DEAD_COUNT, allyDead);
            sharedBlackboard.set<float>(BlackboardKey::ALLY_MIN_HEALTH_RATIO, allyMinRatio);
            return;
        }
        roleSignature = signature;

        int attackerSlots = maxAttackers;
        for (const auto& row : sorted) {
            SquadRole assigned;
            if (row.health <= 0.0f) {
                assigned = SquadRole::SUPPORT;              // bodies are out of the fight
            }
            else if (row.supportOnly) {
                assigned = SquadRole::SUPPORT;
            }
            else if (row.inCombat && attackerSlots > 0) {
                assigned = SquadRole::ATTACKER;
                --attackerSlots;
            }
            else {
                assigned = SquadRole::SUPPORT;
            }
            if (row.agentId == agentId) currentRole = assigned;
        }

        // Focus target: the engaged agent closest to the player. It anchors the
        // squad - everyone else supports relative to it instead of all piling
        // onto the same approach vector.
        int bestFocus = -1;
        float bestDist = std::numeric_limits<float>::max();
        float currentFocusDist = std::numeric_limits<float>::max();
        for (const auto& row : sorted) {
            if (!row.inCombat || row.supportOnly || row.health <= 0.0f) continue;
            if (row.agentId == focusTargetId) currentFocusDist = row.distanceToPlayer;
            if (row.distanceToPlayer < bestDist) {
                bestDist = row.distanceToPlayer;
                bestFocus = row.agentId;
            }
        }
        // Focus hysteresis: a near-tie in distance must not bounce the role
        // between agents every frame. Hold the incumbent unless it is clearly
        // beaten or has left the fight.
        if (currentFocusDist < std::numeric_limits<float>::max() &&
            bestFocus != focusTargetId && bestDist > currentFocusDist - 2.0f) {
            bestFocus = focusTargetId;
        }
        if (bestFocus != focusTargetId) {
            if (focusTargetId >= 0 || bestFocus >= 0) {
                std::cout << "  [COMBAT-SIGNAL] Squad focus target -> " << bestFocus
                    << " (closest engaged agent)\n";
            }
            focusTargetId = bestFocus;
        }

        sharedBlackboard.set<int>(BlackboardKey::SQUAD_COMBAT_COUNT, combatCount);
        sharedBlackboard.set<int>(BlackboardKey::SQUAD_DEAD_COUNT, deadCount);
        sharedBlackboard.set<float>(BlackboardKey::SQUAD_MIN_HEALTH_RATIO, minRatio);
        sharedBlackboard.set<int>(BlackboardKey::ALLY_DEAD_COUNT, allyDead);
        sharedBlackboard.set<float>(BlackboardKey::ALLY_MIN_HEALTH_RATIO, allyMinRatio);
        sharedBlackboard.set<int>(BlackboardKey::SQUAD_FOCUS_TARGET, focusTargetId);
    }

    bool isEngaged() const {
        return currentState == AIState::CHASE || currentState == AIState::ATTACK;
    }

    bool isSquadInCombat() {
        return sharedBlackboard.getOr<int>(BlackboardKey::SQUAD_COMBAT_COUNT, 0) > 0;
    }

    bool isFocusTarget() {
        return focusTargetId == agentId;
    }

    // Support slots orbit the focus target on the far side of the player, so the
    // squad surrounds instead of queueing up in one line.
    Vector3 getSupportAnchor() {
        Vector3 playerPos = getTargetWorldPositionRaw();
        Vector3 focusPos = playerPos;
        if (focusTargetId >= 0) {
            for (Enemy* other : allEnemies) {
                if (other != nullptr && other->getId() == focusTargetId) {
                    focusPos = other->getPosition();
                    break;
                }
            }
        }

        float ax = playerPos.x - focusPos.x;
        float ay = playerPos.y - focusPos.y;
        float len = std::sqrt(ax * ax + ay * ay);
        if (len < 0.001f) { ax = 1.0f; ay = 0.0f; len = 1.0f; }

        // Spread the support slots by rotating the "away from focus" vector so
        // they do not stack on a single point.
        float spread = 0.45f * static_cast<float>(agentId % 4) - 0.675f;
        float cs = std::cos(spread);
        float sn = std::sin(spread);
        float rx = (ax / len) * cs - (ay / len) * sn;
        float ry = (ax / len) * sn + (ay / len) * cs;

        return { playerPos.x + rx * supportFlankDistance,
                 playerPos.y + ry * supportFlankDistance,
                 playerPos.z };
    }

    Vector3 getTargetWorldPositionRaw() {
        if (personallySpotted) return { playerX, playerY, playerZ };
        return lastKnownPlayerPosition();
    }

    // =====================================================================
    // LAYER 3 - SQUAD SURVIVAL: morale from ally wounds and deaths
    // =====================================================================
    // A retreat may always continue once ordered, but a *new* retreat needs a
    // morale cooldown, otherwise one wounded scout re-triggers it forever and
    // the squad never gets a window to recover.
    bool canCallSquadRetreat() {
        if (sharedBlackboard.getOr<bool>(BlackboardKey::SQUAD_RETREAT_ORDERED, false)) return true;
        return timeSinceSquadRetreat >= squadRetreatCooldown;
    }

    bool isAllyDead() {
        return joinsSquadRetreat &&
            sharedBlackboard.getOr<int>(BlackboardKey::ALLY_DEAD_COUNT, 0) > 0 &&
            canCallSquadRetreat();
    }

    bool isAllyLowHealth() {
        return joinsSquadRetreat &&
            sharedBlackboard.getOr<float>(BlackboardKey::ALLY_MIN_HEALTH_RATIO, 1.0f)
            <= allyLowHealthRatio &&
            canCallSquadRetreat();
    }

    // Squad morale: enough dead bodies, or a badly mauled squad, breaks it.
    bool isSquadPanicking() {
        if (!joinsSquadRetreat) return false;
        // A healed agent that already made the rally point stops panicking,
        // otherwise a single corpse would loop the squad retreat forever.
        if (arrivedAtRally && !isHealthLow()) return false;
        if (!canCallSquadRetreat()) return false;
        if (sharedBlackboard.getOr<int>(BlackboardKey::SQUAD_DEAD_COUNT, 0) >= squadPanicDeaths) return true;
        return sharedBlackboard.getOr<float>(BlackboardKey::SQUAD_MIN_HEALTH_RATIO, 1.0f)
            <= squadRetreatRatio;
    }

public:
    Enemy(int id, NavMesh& nav, Blackboard& globalBb,
        const EnemyConfig& config, const GameConfig& gameConfig)
        : agentId(id),
        health(config.health),
        maxHealth(config.health > 0.0f ? config.health : 1.0f),
        detectionRange(config.detectionRange),
        attackRange(config.attackRange),
        speed(config.speed),
        x(5.0f), y(5.0f), z(gameConfig.groundPlaneZ),
        velocity({ 0.0f, 0.0f, 0.0f }),
        navMesh(nav),
        playerX(gameConfig.playerStartX),
        playerY(gameConfig.playerStartY),
        playerZ(gameConfig.playerStartZ),
        playerPolygonId(gameConfig.playerStartPolygon),
        repathCooldown(gameConfig.repathCooldown),
        eventDrivenRepath(gameConfig.eventDrivenRepath),
        canFlee(config.canFlee),
        joinsSquadRetreat(config.joinsSquadRetreat),
        supportOnly(config.supportOnly),
        currentRole(config.preferredRole),
        slowingRadius(gameConfig.slowingRadius),
        arrivalRadius(gameConfig.arrivalRadius),
        chaseFlockWeight(gameConfig.chaseFlockWeight),
        patrolFlockWeight(gameConfig.patrolFlockWeight),
        groundPlaneZ(gameConfig.groundPlaneZ),
        neighborRadius(gameConfig.neighborRadius),
        sharedBlackboard(globalBb),
        ammo(gameConfig.startingAmmo),
        attackDamage(gameConfig.attackDamage),
        lowHealthThreshold(gameConfig.lowHealthThreshold),
        fleeHealAmount(gameConfig.fleeHealAmount),
        fleeSafeRoomId(gameConfig.fleeSafeRoomId),
        playerHealth(gameConfig.playerHealthConfig),
        playerDamage(gameConfig.playerDamage),
        attackInterval(gameConfig.attackInterval),
        squadMemoryDuration(gameConfig.squadMemoryDuration),
        allyLowHealthRatio(gameConfig.allyLowHealthRatio),
        squadRetreatRatio(gameConfig.squadRetreatRatio),
        squadPanicDeaths(gameConfig.squadPanicDeaths),
        maxAttackers(gameConfig.maxAttackers),
        supportFlankDistance(gameConfig.supportFlankDistance),
        squadRetreatCooldown(gameConfig.squadRetreatCooldown),
        maxPathfindingStepsPerFrame(gameConfig.maxPathfindingSteps)
    {
        lastValidPolygonId = navMesh.getPolygonAt(x, y);
        if (lastValidPolygonId < 0) lastValidPolygonId = 0;

        navSubscription = navMesh.subscribe([this](const NavEvent& ev) { this->onNavEvent(ev); });
        buildBehaviorTree();
    }

    ~Enemy() {
        if (navSubscription != 0) {
            navMesh.unsubscribe(navSubscription);
            navSubscription = 0;
        }
    }

    Enemy(const Enemy&) = delete;
    Enemy& operator=(const Enemy&) = delete;

    // =====================================================================
    // identity / simple accessors
    // =====================================================================
    int getId() const { return agentId; }
    bool isDead() { return health <= 0.0f; }
    bool isHealthLow() { return health <= lowHealthThreshold; }
    bool canSelfFlee() { return canFlee; }
    SquadRole getRole() const { return currentRole; }
    // Public read-only view of this agent's perception, for the driver/debug UI.
    bool knowsPlayerPosition() { return isPlayerKnown(); }
    bool hasEyesOnPlayer() { return personallySpotted; }
    Vector3 getLastKnownPlayerPosition() { return lastKnownPlayerPosition(); }
    float getHealth() const { return health; }
    float getMaxHealth() const { return maxHealth; }
    float getDeltaTime() const { return lastDeltaTime; }
    int getAmmo() const { return ammo; }
    float getAttackDamage() const { return attackDamage; }
    float getPlayerDamage() const { return playerDamage; }
    float getPlayerHealth() const { return playerHealth; }
    // True once per landed swing; the simulation uses this to return fire in
    // step with the attacker's cadence instead of every frame.
    bool consumeAttackSwing() { bool s = attackSwingThisTick; attackSwingThisTick = false; return s; }
    void setPlayerHealth(float hp) { playerHealth = hp; }
    void setPosition(float px, float py, float pz) { x = px; y = py; z = pz; }
    Vector3 getPosition() const { return { x, y, z }; }
    void setEnemyList(std::vector<Enemy*>& Enemies) { allEnemies = Enemies; }
    void setPatrolPath(const std::vector<int>& path) { patrolPath = path; patrolIndex = 0; }
    void setState(AIState state) { currentState = state; }
    AIState getState() const { return currentState; }

    void setPlayerPosition(float px, float py, float pz, int Playerpolygonid) {
        playerX = px; playerY = py; playerZ = pz; playerPolygonId = Playerpolygonid;
    }

    float distanceToPlayer() const {
        float dx = playerX - x;
        float dy = playerY - y;
        return std::sqrtf(dx * dx + dy * dy);
    }

    // External damage source (the player's return fire). Damage never removes
    // the agent itself; the behaviour tree reacts to health on the next tick.
    void applyDamage(float amount) {
        if (amount <= 0.0f || isDead()) return;
        health -= amount;
        if (health < 0.0f) health = 0.0f;
        std::cout << "  [COMBAT] Agent " << agentId << " takes " << amount
            << " damage -> " << health << " HP\n";
    }

    // =====================================================================
    // conditions used by the behaviour tree
    // =====================================================================
    bool isPlayerDetected() {
        return distanceToPlayer() < detectionRange;
    }

    bool isInAttackRange() {
        return distanceToPlayer() < attackRange;
    }

    // Only the attack slots commit; support slots flank instead of dogpiling.
    bool isAttackSlot() {
        if (!isSquadInCombat()) return true;      // solo: no squad rules apply
        return currentRole == SquadRole::ATTACKER;
    }

    bool isSupportSlot() {
        return isSquadInCombat() && !isAttackSlot();
    }

    // Reached the rally point but the squad has not fully regrouped yet.
    bool isRegrouping() {
        return arrivedAtRally &&
            sharedBlackboard.getOr<bool>(BlackboardKey::SQUAD_RETREAT_ORDERED, false);
    }

    // Ends the regroup wait and re-opens the patrol route.
    void finishRegroup() {
        arrivedAtRally = false;
        currentRole = configPreferredRole();

        bool anyoneStillRetreating = false;
        for (Enemy* other : allEnemies) {
            if (other == nullptr || other == this) continue;
            if (other->getState() == AIState::FLEE && !other->hasArrivedAtRally()) {
                anyoneStillRetreating = true;
                break;
            }
        }
        if (!anyoneStillRetreating) {
            sharedBlackboard.set<bool>(BlackboardKey::SQUAD_RETREAT_ORDERED, false);
            std::cout << "  [SURVIVAL] Agent " << agentId
                << " regroups - squad retreat order cleared\n";
        }
        currentPath.clear();
        currentPathIndex = 0;
        setState(AIState::PATROL);
    }

    bool hasArrivedAtRally() const { return arrivedAtRally; }

    // ---- dynamic-environment helpers, called by PathClearWaitNode ----
    bool isPathClear() {
        if (pendingRepath) return false;
        if (currentState == AIState::CHASE) {
            if (currentPathRequest.state != PathState::FOUND) return false;
            return navMesh.pathIsClear(currentPathRequest.FinalPath, chasePathIndex);
        }
        return navMesh.pathIsClear(currentPath, currentPathIndex);
    }

    int getPendingEventCount() const { return pendingEventCount; }
    bool isRepathPending() const { return pendingRepath; }

    void forceRepath() {
        pendingRepath = true;
        currentPathRequest.state = PathState::IDLE;
        timeSinceLastRepath = repathCooldown;   // do not wait out the cooldown
    }

    // =====================================================================
    // actions used by the behaviour tree
    // =====================================================================
    void doAttack() { setState(AIState::ATTACK); updateAttack(); }
    void doDead() { setState(AIState::DEAD); updateDead(); }
    void doFlee() { setState(AIState::FLEE); updateFlee(); }
    void doChase() { setState(AIState::CHASE); updateChase(); }
    void doPatrol() { setState(AIState::PATROL); updatePatrol(); }
    void doSupport() { setState(AIState::CHASE); updateSupport(); }
    void doSquadRetreat() { setState(AIState::FLEE); updateSquadRetreat(); }

    void buildBehaviorTree() {
        // Every leaf gets its OWN node object: a node is moved into exactly one
        // parent, and re-pushing the same unique_ptr would leave a null child
        // that segfaults the first time the selector evaluates it.

        // ---- 0. terminal ----
        std::vector<std::unique_ptr<BTNode>> deadChildren;
        deadChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isDead));
        deadChildren.push_back(std::make_unique<ActionNode>(&Enemy::doDead));
        auto seqDead = std::make_unique<Sequence>(std::move(deadChildren));

        // ---- 1. regroup at the rally point (timed, deltaTime driven) ----
        std::vector<std::unique_ptr<BTNode>> regroupChildren;
        regroupChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isRegrouping));
        regroupChildren.push_back(std::make_unique<TimedWaitNode>(
            squadRetreatCooldown, "RegroupAtRally"));
        regroupChildren.push_back(std::make_unique<ActionNode>(&Enemy::finishRegroup));
        auto seqRegroup = std::make_unique<Sequence>(std::move(regroupChildren));

        // ---- 2. personal survival ----
        std::vector<std::unique_ptr<BTNode>> seqFleeChildren;
        seqFleeChildren.push_back(std::make_unique<ConditionNode>(&Enemy::canSelfFlee));
        seqFleeChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isHealthLow));
        seqFleeChildren.push_back(std::make_unique<ActionNode>(&Enemy::doFlee));
        auto seqFlee = std::make_unique<Sequence>(std::move(seqFleeChildren));

        // ---- 3. LAYER 3: squad survival (morale) ----
        std::vector<std::unique_ptr<BTNode>> panicChildren;
        panicChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isSquadPanicking));
        panicChildren.push_back(std::make_unique<ActionNode>(&Enemy::doSquadRetreat));
        auto seqPanic = std::make_unique<Sequence>(std::move(panicChildren));

        std::vector<std::unique_ptr<BTNode>> allyDeadChildren;
        allyDeadChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isAllyDead));
        allyDeadChildren.push_back(std::make_unique<ActionNode>(&Enemy::doSquadRetreat));
        auto seqAllyDead = std::make_unique<Sequence>(std::move(allyDeadChildren));

        std::vector<std::unique_ptr<BTNode>> allyLowHpChildren;
        allyLowHpChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isAllyLowHealth));
        allyLowHpChildren.push_back(std::make_unique<ActionNode>(&Enemy::doSquadRetreat));
        auto seqAllyLowHp = std::make_unique<Sequence>(std::move(allyLowHpChildren));

        std::vector<std::unique_ptr<BTNode>> squadSurvivalChildren;
        squadSurvivalChildren.push_back(std::move(seqPanic));
        squadSurvivalChildren.push_back(std::move(seqAllyDead));
        squadSurvivalChildren.push_back(std::move(seqAllyLowHp));
        auto squadSurvivalSel = std::make_unique<Selector>(std::move(squadSurvivalChildren));

        // ---- 4. LAYERS 1+2: perception gate, then combat signalling ----
        std::vector<std::unique_ptr<BTNode>> supportChildren;
        supportChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isSupportSlot));
        supportChildren.push_back(std::make_unique<ActionNode>(&Enemy::doSupport));
        auto seqSupport = std::make_unique<Sequence>(std::move(supportChildren));

        // Only the elected focus target lands the finishing blow in a squad fight.
        std::vector<std::unique_ptr<BTNode>> focusAttackChildren;
        focusAttackChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isInAttackRange));
        focusAttackChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isFocusTarget));
        focusAttackChildren.push_back(std::make_unique<ActionNode>(&Enemy::doAttack));
        auto seqFocusAttack = std::make_unique<Sequence>(std::move(focusAttackChildren));

        // Solo fight: no squad rules apply.
        std::vector<std::unique_ptr<BTNode>> soloAttackChildren;
        soloAttackChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isInAttackRange));
        soloAttackChildren.push_back(std::make_unique<ActionNode>(&Enemy::doAttack));
        auto seqSoloAttack = std::make_unique<Sequence>(std::move(soloAttackChildren));

        // Chase stays RUNNING until the route it depends on is usable again,
        // which is how a navigation event turns into a wait instead of a stall.
        std::vector<std::unique_ptr<BTNode>> chaseChildren;
        chaseChildren.push_back(std::make_unique<ActionNode>(&Enemy::doChase));
        chaseChildren.push_back(std::make_unique<PathClearWaitNode>(3.0f, "ObstacleClear"));
        auto seqChase = std::make_unique<Sequence>(std::move(chaseChildren));

        std::vector<std::unique_ptr<BTNode>> combatChildren;
        combatChildren.push_back(std::move(seqSupport));
        combatChildren.push_back(std::move(seqFocusAttack));
        combatChildren.push_back(std::move(seqSoloAttack));
        combatChildren.push_back(std::move(seqChase));
        auto combatSel = std::make_unique<Selector>(std::move(combatChildren));

        std::vector<std::unique_ptr<BTNode>> combatGateChildren;
        combatGateChildren.push_back(std::make_unique<ConditionNode>(&Enemy::isPlayerKnown));
        combatGateChildren.push_back(std::move(combatSel));
        auto combatGateSeq = std::make_unique<Sequence>(std::move(combatGateChildren));

        // ---- ROOT SELECTOR: dead > regroup > self-preservation > squad >
        // ---- combat > patrol ----
        std::vector<std::unique_ptr<BTNode>> rootChildren;
        rootChildren.push_back(std::move(seqDead));
        rootChildren.push_back(std::move(seqRegroup));
        rootChildren.push_back(std::move(seqFlee));
        rootChildren.push_back(std::move(squadSurvivalSel));
        rootChildren.push_back(std::move(combatGateSeq));
        rootChildren.push_back(std::make_unique<ActionNode>(&Enemy::doPatrol));
        root = std::make_unique<Selector>(std::move(rootChildren));
    }


    void runBT() {
        if (root) root->execute(this);
    }

    int getCurrentPolygon() {
        if (currentState == AIState::CHASE) {
            if (currentPathRequest.state == PathState::FOUND &&
                chasePathIndex < currentPathRequest.FinalPath.size()) {
                return currentPathRequest.FinalPath[chasePathIndex];
            }
        }
        else if (!currentPath.empty() && currentPathIndex < currentPath.size()) {
            return currentPath[currentPathIndex];
        }
        return lastValidPolygonId;
    }

    std::string stateToString(AIState state) {
        switch (state) {
        case AIState::IDLE:   return "IDLE";
        case AIState::PATROL: return "PATROL";
        case AIState::CHASE:  return "CHASE";
        case AIState::ATTACK: return "ATTACK";
        case AIState::FLEE:   return "FLEE";
        case AIState::DEAD:   return "DEAD";
        }
        return "UNKNOWN";
    }

    std::string roleToString(SquadRole role) {
        switch (role) {
        case SquadRole::ATTACKER: return "ATTACKER";
        case SquadRole::SUPPORT:  return "SUPPORT";
        case SquadRole::RETREAT:  return "RETREAT";
        }
        return "UNKNOWN";
    }

    void updateIdle() {
        std::cout << "Idle...\n";
        if (isPlayerDetected()) setState(AIState::CHASE);
    }

    void updatePatrol() {
        if (patrolPath.empty()) { setState(AIState::IDLE); return; }

        if (currentPath.empty() || currentPathIndex >= currentPath.size()) {
            int start = getCurrentPolygon();
            int goal = patrolPath[patrolIndex];
            std::vector<int> rawPath = navMesh.aStar(start, goal);
            currentPath = navMesh.smoothPath(rawPath);
            currentPathIndex = 0;

            if (currentPath.empty()) {
                std::cout << "  No path to patrol point " << patrolIndex << "\n";
                patrolIndex = (patrolIndex + 1) % static_cast<int>(patrolPath.size());
                return;
            }
            std::cout << "  Patrolling to polygon " << goal << "\n";
        }

        int nextPolygon = currentPath[currentPathIndex];
        auto center = navMesh.getPolygonCenter(nextPolygon);
        float dx = center.x - x;
        float dy = center.y - y;

        if (std::sqrtf(dx * dx + dy * dy) < arrivalRadius) {
            lastValidPolygonId = nextPolygon;
            currentPathIndex++;
            if (currentPathIndex >= currentPath.size()) {
                patrolIndex = (patrolIndex + 1) % static_cast<int>(patrolPath.size());
                currentPath.clear();
                currentPathIndex = 0;
            }
        }
        if (isPlayerDetected()) setState(AIState::CHASE);
    }

    // CHASE: one search, one path, and a decision about *when* to re-issue it.
    void updateChase() {
        // --- who are we walking towards? ---
        int goal = getTargetPolygon();
        if (isSharedSightingOnly() && !investigatedLastKnown) {
            std::cout << "  [PERCEPTION] Agent " << agentId
                << " acting on ally " << whoSpottedPlayer()
                << "'s sighting at polygon " << goal << "\n";
            investigatedLastKnown = true;
        }

        // --- repath triggers ---
        bool cooldownElapsed = timeSinceLastRepath >= repathCooldown;

        // A navigation event outranks the cooldown: the route we are following
        // physically does not exist any more, so waiting is never correct.
        bool eventForcesRepath = eventDrivenRepath && pendingRepath;
        std::string reason = "none";

        bool goalMoved = (goal != lastKnownPlayerPolygon);
        bool requestStale = navMesh.isRequestStale(currentPathRequest) &&
            currentPathRequest.state == PathState::SEARCHING;

        bool pathBlocked = false;
        if (currentPathRequest.state == PathState::FOUND) {
            for (size_t i = chasePathIndex; i < currentPathRequest.FinalPath.size(); ++i) {
                if (navMesh.isBlocked(currentPathRequest.FinalPath[i])) {
                    pathBlocked = true;
                    std::cout << "  [PATH] Current path hit blocked polygon "
                        << currentPathRequest.FinalPath[i] << "! Forcing repath.\n";
                    break;
                }
            }
        }

        bool noUsableRoute = currentPathRequest.state == PathState::IDLE ||
            currentPathRequest.state == PathState::FAILED;

        if (eventForcesRepath) reason = "nav-event";
        else if (requestStale) reason = "search-invalidated";
        else if (pathBlocked) reason = "path-blocked";
        else if (goalMoved) reason = "goal-moved";
        else if (noUsableRoute) reason = "no-route";

        bool needsNewPath = eventForcesRepath || requestStale || pathBlocked ||
            (cooldownElapsed && (goalMoved || noUsableRoute));

        if (needsNewPath) {
            int start = getCurrentPolygon();
            navMesh.startPathRequest(currentPathRequest, start, goal, navMesh.getMapSize());
            chasePathIndex = 0;
            timeSinceLastRepath = 0.0f;
            lastKnownPlayerPolygon = goal;
            pendingRepath = false;
            if (reason != lastRepathReason) {
                std::cout << "  [PATH] Agent " << agentId << " repath (" << reason
                    << ") -> polygon " << goal << "\n";
                lastRepathReason = reason;
            }
        }

        if (currentPathRequest.state == PathState::SEARCHING) {
            navMesh.updatePathfindingSlice(currentPathRequest, maxPathfindingStepsPerFrame);
            std::cout << "  [Pathfinding] Calculating path... (State: SEARCHING)\n";
            return; // Exit early, wait for next frame
        }

        if (currentPathRequest.state == PathState::FOUND) {
            if (chasePathIndex < currentPathRequest.FinalPath.size()) {
                int nextPolygon = currentPathRequest.FinalPath[chasePathIndex];
                auto center = navMesh.getPolygonCenter(nextPolygon);
                float dx = center.x - x;
                float dy = center.y - y;

                if (std::sqrtf(dx * dx + dy * dy) < arrivalRadius) {
                    lastValidPolygonId = nextPolygon;
                    chasePathIndex++;
                }
            }
            else {
                // Reached the end of the route. If it was only a shared sighting
                // we are done investigating and go back to patrol.
                if (isSharedSightingOnly()) {
                    std::cout << "  [PERCEPTION] Agent " << agentId
                        << " reached the last known position, no contact -> PATROL\n";
                    sharedBlackboard.set<bool>(BlackboardKey::PLAYER_KNOWN, false);
                    investigatedLastKnown = false;
                    currentPathRequest.state = PathState::IDLE;
                    chasePathIndex = 0;
                    setState(AIState::PATROL);
                }
                // Otherwise contact is still live: keep the request FOUND and
                // let movement home in on the target. Clearing it here would
                // flip the agent to IDLE for a tick and thrash the whole squad.
            }
        }
        else if (currentPathRequest.state == PathState::FAILED) {
            std::cout << "  [PATH] Agent " << agentId << " has no route to polygon "
                << goal << " - falling back to direct steering\n";
        }
    }

    // SUPPORT slot: hold a flanking position instead of the attacker's lane.
    void updateSupport() {
        Vector3 anchor = getSupportAnchor();
        float dx = anchor.x - x;
        float dy = anchor.y - y;

        if (std::sqrtf(dx * dx + dy * dy) < arrivalRadius * 2.0f) {
            std::cout << "  [COMBAT-SIGNAL] Agent " << agentId
                << " holding flank at (" << anchor.x << ", " << anchor.y << ")\n";
        }

        // Support still needs a legal polygon to path through when the anchor is
        // across a wall, so it reuses the chase search toward the anchor.
        int goal = navMesh.getPolygonAt(anchor);
        if (goal >= 0 && currentPathRequest.state == PathState::IDLE &&
            (timeSinceLastRepath >= repathCooldown || pendingRepath)) {
            int start = getCurrentPolygon();
            navMesh.startPathRequest(currentPathRequest, start, goal, navMesh.getMapSize());
            chasePathIndex = 0;
            timeSinceLastRepath = 0.0f;
            lastKnownPlayerPolygon = goal;
            pendingRepath = false;
            std::cout << "  [PATH] Agent " << agentId << " support repath ("
                << (navMesh.getBlockVersion() > 0 && lastRepathReason != "support" ? "nav-event" : "flank-move")
                << ") -> polygon " << goal << "\n";
            lastRepathReason = "support";
        }

        if (currentPathRequest.state == PathState::SEARCHING) {
            navMesh.updatePathfindingSlice(currentPathRequest, maxPathfindingStepsPerFrame);
            return;
        }
        if (currentPathRequest.state == PathState::FOUND &&
            chasePathIndex < currentPathRequest.FinalPath.size()) {
            int nextPolygon = currentPathRequest.FinalPath[chasePathIndex];
            auto center = navMesh.getPolygonCenter(nextPolygon);
            float dx2 = center.x - x;
            float dy2 = center.y - y;
            if (std::sqrtf(dx2 * dx2 + dy2 * dy2) < arrivalRadius) {
                lastValidPolygonId = nextPolygon;
                chasePathIndex++;
            }
        }
        else if (currentPathRequest.state == PathState::FOUND) {
            currentPathRequest.state = PathState::IDLE;
            chasePathIndex = 0;
        }
    }

    void updateAttack() {
        attackCooldownTimer += lastDeltaTime;
        if (attackCooldownTimer < attackInterval) return;

        attackCooldownTimer = 0.0f;
        if (ammo <= 0) {
            std::cout << "  [COMBAT] Agent " << agentId << " is out of ammo!\n";
            return;
        }
        --ammo;

        // Player health is shared state, not four private copies that drift.
        float sharedPlayerHealth = sharedBlackboard.getOr<float>(
            BlackboardKey::PLAYER_HEALTH, playerHealth);
        sharedPlayerHealth = std::max(0.0f, sharedPlayerHealth - attackDamage);
        sharedBlackboard.set<float>(BlackboardKey::PLAYER_HEALTH, sharedPlayerHealth);
        playerHealth = sharedPlayerHealth;
        attackSwingThisTick = true;

        std::cout << "  [COMBAT] Agent " << agentId << " (role " << roleToString(currentRole)
            << ") hits the player for " << attackDamage << " (player hp " << sharedPlayerHealth
            << ", ammo " << ammo << ")\n";
    }

    void updateFlee() {
        std::cout << "Fleeing from player!\n";
        int start = getCurrentPolygon();
        int goal = fleeSafeRoomId;

        if (currentPath.empty() || currentPathIndex >= currentPath.size()) {
            std::vector<int> rawPath = navMesh.aStar(start, goal);
            currentPath = navMesh.smoothPath(rawPath);
            currentPathIndex = 0;
        }

        if (currentPath.empty()) {
            Vector3 playerPos = { playerX, playerY, playerZ };
            Vector3 currentPos = { x, y, z };
            Vector3 fleeSteer = steering::flee(playerPos, currentPos, velocity, speed);
            velocity = steering::add(velocity, fleeSteer);
            return;
        }

        int nextPolygon = currentPath[currentPathIndex];
        auto center = navMesh.getPolygonCenter(nextPolygon);
        float dx = center.x - x;
        float dy = center.y - y;

        if (std::sqrtf(dx * dx + dy * dy) < arrivalRadius) {
            lastValidPolygonId = nextPolygon;
            currentPathIndex++;

            // Arrival is tested against the polygon just reached, not the polygon the
            // path was planned from. The old check used `start`, so the enemy healed
            // and switched to PATROL the first time it grazed any path node instead
            // of when it actually stood in the safe room.
            if (nextPolygon == fleeSafeRoomId) {
                health += fleeHealAmount;
                if (health > maxHealth) health = maxHealth;
                std::cout << "  [SURVIVAL] Agent " << agentId << " reached the safe room, healed to "
                    << health << " HP\n";
                currentPath.clear();
                currentPathIndex = 0;
                setState(AIState::PATROL);
            }
        }
    }

    // LAYER 3 action: the retreat is a squad order, not a private panic.
    void updateSquadRetreat() {
        int rally = sharedBlackboard.getOr<int>(BlackboardKey::SQUAD_RETREAT_TARGET, fleeSafeRoomId);
        if (!sharedBlackboard.getOr<bool>(BlackboardKey::SQUAD_RETREAT_ORDERED, false)) {
            sharedBlackboard.set<bool>(BlackboardKey::SQUAD_RETREAT_ORDERED, true);
            sharedBlackboard.set<int>(BlackboardKey::SQUAD_RETREAT_TARGET, rally);
            sharedBlackboard.set<float>(BlackboardKey::SIM_TIME, simTime);
            std::cout << "  [SURVIVAL] Agent " << agentId
                << " calls a SQUAD RETREAT to polygon " << rally
                << " (dead=" << sharedBlackboard.getOr<int>(BlackboardKey::SQUAD_DEAD_COUNT, 0)
                << ", worst ally ratio="
                << sharedBlackboard.getOr<float>(BlackboardKey::SQUAD_MIN_HEALTH_RATIO, 1.0f)
                << ")\n";
        }

        currentRole = SquadRole::RETREAT;
        std::cout << "  [SURVIVAL] Agent " << agentId << " retreating to rally polygon " << rally << "\n";

        int start = getCurrentPolygon();
        if (currentPath.empty() || currentPathIndex >= currentPath.size()) {
            std::vector<int> rawPath = navMesh.aStar(start, rally);
            currentPath = navMesh.smoothPath(rawPath);
            currentPathIndex = 0;
        }

        if (currentPath.empty()) {
            Vector3 playerPos = { playerX, playerY, playerZ };
            Vector3 currentPos = { x, y, z };
            Vector3 fleeSteer = steering::flee(playerPos, currentPos, velocity, speed);
            velocity = steering::add(velocity, fleeSteer);
            return;
        }

        int nextPolygon = currentPath[currentPathIndex];
        auto center = navMesh.getPolygonCenter(nextPolygon);
        float dx = center.x - x;
        float dy = center.y - y;

        if (std::sqrtf(dx * dx + dy * dy) < arrivalRadius) {
            lastValidPolygonId = nextPolygon;
            currentPathIndex++;

            if (nextPolygon == rally) {
                health += fleeHealAmount;
                if (health > maxHealth) health = maxHealth;
                std::cout << "  [SURVIVAL] Agent " << agentId << " reached the rally point, healed to "
                    << health << " HP\n";
                currentPath.clear();
                currentPathIndex = 0;
                arrivedAtRally = true;
                timeSinceSquadRetreat = 0.0f;

                // While the order stands, the root branch parks this agent in a
                // timed regroup instead of sending it straight back into the fight.
                if (!sharedBlackboard.getOr<bool>(BlackboardKey::SQUAD_RETREAT_ORDERED, false)) {
                    setState(AIState::PATROL);
                }
            }
        }
    }

    SquadRole configPreferredRole() const {
        return supportOnly ? SquadRole::SUPPORT : SquadRole::ATTACKER;
    }

    void updateDead() {
        std::cout << "Agent " << agentId << " is DEAD.\n";
        velocity = { 0.0f, 0.0f, 0.0f };
    }

    void updateMovement(const Vector3& target, float deltaTime) {
        lastDeltaTime = deltaTime;
        if (currentState == AIState::DEAD) {
            velocity = { 0.0f, 0.0f, 0.0f };
            std::cout << "  [Steering] Agent " << agentId << " is DEAD at Pos: ("
                << x << ", " << y << ", " << z << ")\n";
            return;
        }

        Vector3 currentPos = { x, y, z };
        Vector3 steer;
        std::vector<Vector3> neighborPositions = getNeighborPositions();
        std::vector<Vector3> neighborVelocities = getNeighborVelocities();
        Vector3 flockForce = steering::flocking(neighborPositions, neighborVelocities, currentPos, velocity);

        if (currentState == AIState::CHASE) {
            Vector3 moveTarget = target;

            bool routeExhausted = !(currentPathRequest.state == PathState::FOUND &&
                chasePathIndex < currentPathRequest.FinalPath.size());

            if (currentRole == SquadRole::SUPPORT) {
                // Support steers at its flank anchor, not at the enemy itself.
                moveTarget = getSupportAnchor();
            }
            else if (!routeExhausted) {
                moveTarget = navMesh.getPolygonCenter(currentPathRequest.FinalPath[chasePathIndex]);
            }
            else {
                // Attacker has arrived in the target polygon: home in on the
                // enemy's last known position so it can enter attack range
                // instead of orbiting its own waypoint forever.
                moveTarget = getTargetWorldPositionRaw();
            }

            // Weights are data-driven: an unweighted seek+flock sum lets cohesion
            // win and the squad parks short of the waypoint (never arriving).
            Vector3 seekForce = steering::seek(moveTarget, currentPos, velocity, speed);
            steer = steering::add(seekForce, steering::scale(flockForce, chaseFlockWeight));
        }
        else if (currentState == AIState::FLEE) {
            steer = steering::arrive(target, currentPos, velocity, speed, slowingRadius);
        }
        else {
            Vector3 seekForce = steering::seek(target, currentPos, velocity, speed);
            steer = steering::add(seekForce, steering::scale(flockForce, patrolFlockWeight));
        }

        velocity = steering::add(velocity, steer);
        velocity = steering::truncate(velocity, speed);

        x += velocity.x * deltaTime;
        y += velocity.y * deltaTime;
        z = groundPlaneZ;
        velocity = steering::scale(velocity, 0.85f);

        std::cout << "  [Steering] Agent " << agentId << " Pos: (" << x << ", " << y
            << ", " << z << ") role=" << roleToString(currentRole)
            << " hp=" << health << "\n";
    }

    void update(float deltaTime) {
        lastDeltaTime = deltaTime;
        simTime = sharedBlackboard.getOr<float>(BlackboardKey::SIM_TIME, simTime);

        // --- personal telemetry ---
        personalBlackboard.set<Vector3>(BlackboardKey::ENEMY_POSITION, { x, y, z });
        personalBlackboard.set<float>(BlackboardKey::ENEMY_HEALTH, health);
        personalBlackboard.set<bool>(BlackboardKey::IS_DEAD, isDead());

        // --- layer 1: perception ---
        publishPerception();

        // --- read the shared world truth ---
        if (sharedBlackboard.has(BlackboardKey::PLAYER_POSITION)) {
            Vector3 playerpos = sharedBlackboard.get<Vector3>(BlackboardKey::PLAYER_POSITION);
            playerX = playerpos.x; playerY = playerpos.y; playerZ = playerpos.z;
        }
        if (sharedBlackboard.has(BlackboardKey::PLAYER_POLYGON)) {
            playerPolygonId = sharedBlackboard.get<int>(BlackboardKey::PLAYER_POLYGON);
        }

        // --- layer 2: combat signalling (fold the roster, then publish my row
        //     with the role that fold just assigned me) ---
        sharedBlackboard.set<bool>(BlackboardKey::IS_IN_COMBAT, isEngaged());
        refreshSquadAggregates();
        publishRosterRow();

        // --- repath timer bookkeeping ---
        if (currentState == AIState::CHASE) {
            timeSinceLastRepath += deltaTime;
        }
        else {
            timeSinceLastRepath = repathCooldown;
        }
        timeSinceSquadRetreat += deltaTime;

        runBT();
    }

    std::vector<Vector3> getNeighborPositions() {
        std::vector<Vector3> positions;
        Vector3 myPos = getPosition();
        for (Enemy* other : allEnemies) {
            if (other == nullptr || other == this || other->getState() == AIState::DEAD) continue;
            if (steering::magnitude(steering::subtract(other->getPosition(), myPos)) < neighborRadius) {
                positions.push_back(other->getPosition());
            }
        }
        return positions;
    }

    std::vector<Vector3> getNeighborVelocities() {
        std::vector<Vector3> velocities;
        Vector3 myPos = getPosition();
        for (Enemy* other : allEnemies) {
            if (other == nullptr || other == this || other->getState() == AIState::DEAD) continue;
            if (steering::magnitude(steering::subtract(other->getPosition(), myPos)) < neighborRadius) {
                velocities.push_back(other->getVelocity());
            }
        }
        return velocities;
    }

    Vector3 getVelocity() const { return velocity; }

    void moveToPolygon(int polygonId) {
        Vector3 center = navMesh.getPolygonCenter(polygonId);
        float dx = center.x - x;
        float dy = center.y - y;
        float distance = std::sqrtf(dx * dx + dy * dy);

        if (distance > 0.1f) {
            float currentStep = std::min(speed, distance);
            x += (dx / distance) * currentStep;
            y += (dy / distance) * currentStep;
            std::cout << "  Moving to polygon " << polygonId << " (" << x << ", " << y << ", " << z << ")\n";
        }
        else {
            lastValidPolygonId = polygonId;
            std::cout << "  Reached polygon " << polygonId << "\n";
        }
    }

private:
    float lastDeltaTime = 0.5f;   // frame delta, published for the wait nodes
};

BTNodeStatus ConditionNode::execute(Enemy* enemy) {
    return (enemy->*condition)() ? BTNodeStatus::SUCCESS : BTNodeStatus::FAILURE;
}

BTNodeStatus ActionNode::execute(Enemy* enemy) {
    (enemy->*action)();
    return BTNodeStatus::SUCCESS;
}

BTNodeStatus TimedWaitNode::execute(Enemy* enemy) {
    // Real frame delta, not a hard-coded step: the node stays correct when
    // delta_time changes in the JSON config.
    elapsed += enemy->getDeltaTime();
    if (elapsed >= waitDuration) {
        std::cout << "  [BT] " << debugLabel << " COMPLETE (" << waitDuration << "s)\n";
        reset();
        return BTNodeStatus::SUCCESS;
    }
    std::cout << "  [BT] " << debugLabel << " WAITING... (" << elapsed
        << "/" << waitDuration << "s)\n";
    return BTNodeStatus::RUNNING;
}

BTNodeStatus PathClearWaitNode::execute(Enemy* enemy) {
    elapsed += enemy->getDeltaTime();
    bool hadPendingEvent = enemy->isRepathPending();

    if (enemy->isPathClear()) {
        if (hadPendingEvent) {
            std::cout << "  [BT] " << debugLabel << " CLEARED by nav event ("
                << enemy->getPendingEventCount() << " event(s) absorbed)\n";
        }
        reset();
        return BTNodeStatus::SUCCESS;
    }

    if (elapsed >= maxWait) {
        std::cout << "  [BT] " << debugLabel << " TIMED OUT after " << maxWait
            << "s - forcing a fresh search\n";
        enemy->forceRepath();
        reset();
        return BTNodeStatus::SUCCESS;
    }

    std::cout << "  [BT] " << debugLabel << " WAITING on repath... ("
        << elapsed << "/" << maxWait << "s, "
        << enemy->getPendingEventCount() << " nav event(s) pending)\n";
    return BTNodeStatus::RUNNING;
}

// Default tuning lives in one place so a missing / partial JSON key always has
// a sane value to fall back to.
GameConfig::GameConfig()
    : maxPathfindingSteps(50),
      repathCooldown(0.5f),
      neighborRadius(15.0f),
      slowingRadius(5.0f),
      fleeSafeRoomId(0),
      deltaTime(0.5f),
      attackDamage(25.0f),
      lowHealthThreshold(25.0f),
      fleeHealAmount(100.0f),
      playerHealthConfig(100.0f),
      startingAmmo(30),
      attackInterval(1.0f),
      playerDamage(25.0f),
      arrivalRadius(0.5f),
      chaseFlockWeight(0.15f),
      patrolFlockWeight(1.0f),
      groundPlaneZ(0.5f),
      eventDrivenRepath(true),
      rebuildHPAOnBlock(true),
      losSampleStep(1.0f),
      squadMemoryDuration(3.0f),
      allyLowHealthRatio(0.3f),
      squadRetreatRatio(0.25f),
      squadPanicDeaths(1),
      maxAttackers(2),
      supportFlankDistance(6.0f),
      squadRetreatCooldown(3.0f),
      playerStartX(40.0f), playerStartY(40.0f), playerStartZ(10.0f),
      playerStartPolygon(3),
      playerPhase2X(15.0f), playerPhase2Y(15.0f), playerPhase2Z(10.0f),
      playerCombatX(15.0f), playerCombatY(15.0f), playerCombatZ(10.0f),
      patrolRoute({ 0, 1, 2, 3 })
{
}

EnemyConfig::EnemyConfig(float hp, float spd, float detect, float atkRange,
    bool flee, bool squadRetreat, bool support, SquadRole role)
    : health(hp), speed(spd), detectionRange(detect), attackRange(atkRange),
      canFlee(flee), joinsSquadRetreat(squadRetreat), supportOnly(support),
      preferredRole(role)
{
}

// =====================================================================
// DATA-DRIVEN CONFIGURATION LAYER
// ---------------------------------------------------------------------
// Everything tunable lives in enemy_configs.json: the map, the graph, the
// clusters, the system tunables, the squad coordination rules and the per
// archetype behaviour flags. The helpers below make the reads defensive so a
// missing file, a malformed document or a missing key degrades to a default
// instead of throwing out of a .get<T>().
// =====================================================================

// Reads one typed value, falling back when the key is absent, null or the
// wrong type. Also counts misses so the demo can report a bad config.
template<typename T>
T jsonValue(const json& node, const char* key, const T& fallback) {
    if (!node.is_object()) return fallback;
    auto it = node.find(key);
    if (it == node.end() || it->is_null()) return fallback;
    try {
        return it->get<T>();
    }
    catch (const json::exception&) {
        std::cerr << "  [CONFIG] Key '" << key << "' has the wrong type - using default\n";
        return fallback;
    }
}

bool jsonFlag(const json& node, const char* key, bool fallback) {
    return jsonValue<bool>(node, key, fallback);
}

SquadRole jsonRole(const json& node, const char* key, SquadRole fallback) {
    std::string raw = jsonValue<std::string>(node, key, std::string());
    if (raw == "attacker") return SquadRole::ATTACKER;
    if (raw == "support") return SquadRole::SUPPORT;
    if (raw == "retreat") return SquadRole::RETREAT;
    if (!raw.empty()) std::cerr << "  [CONFIG] Unknown role '" << raw << "'\n";
    return fallback;
}

// Parses the file once; returns an empty object when it cannot be read, which
// every loader then treats as "all defaults".
json loadConfigFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[CONFIG] Could not open '" << filename << "' - falling back to defaults\n";
        return json::object();
    }

    json data;
    try {
        file >> data;
    }
    catch (const json::parse_error& e) {
        std::cerr << "[CONFIG] '" << filename << "' is not valid JSON: " << e.what()
            << "\n[CONFIG] Falling back to defaults\n";
        return json::object();
    }
    return data;
}

EnemyConfig loadEnemyConfig(const json& data, const std::string& enemyType) {
    EnemyConfig defaults;

    if (!data.contains("enemy_types")) {
        std::cerr << "[CONFIG] No 'enemy_types' block - using default enemy stats\n";
        return defaults;
    }

    const json& types = data["enemy_types"];
    if (!types.is_object() || !types.contains(enemyType)) {
        std::cerr << "[CONFIG] Enemy type '" << enemyType << "' not found - using defaults\n";
        return defaults;
    }

    const json& t = types[enemyType];
    EnemyConfig config;
    config.health = jsonValue<float>(t, "health", defaults.health);
    config.speed = jsonValue<float>(t, "speed", defaults.speed);
    config.detectionRange = jsonValue<float>(t, "detection_range", defaults.detectionRange);
    config.attackRange = jsonValue<float>(t, "attack_range", defaults.attackRange);

    // behaviour flags - these are what make the coordination layers generic
    config.canFlee = jsonFlag(t, "can_flee", defaults.canFlee);
    config.joinsSquadRetreat = jsonFlag(t, "joins_squad_retreat", defaults.joinsSquadRetreat);
    config.supportOnly = jsonFlag(t, "support_only", defaults.supportOnly);
    config.preferredRole = jsonRole(t, "preferred_role", defaults.preferredRole);

    std::cout << "[CONFIG] " << enemyType << ": hp=" << config.health
        << " speed=" << config.speed
        << " detect=" << config.detectionRange
        << " atkRange=" << config.attackRange
        << (config.canFlee ? "" : " (no self-flee)")
        << (config.supportOnly ? " (support only)" : "")
        << "\n";
    return config;
}

GameConfig loadGameConfig(const json& data) {
    GameConfig config;   // defaults are baked into the constructor

    const json sys = data.contains("system") ? data["system"] : json::object();

    config.maxPathfindingSteps = jsonValue<int>(sys, "max_pathfinding_steps", config.maxPathfindingSteps);
    config.repathCooldown = jsonValue<float>(sys, "repath_cooldown", config.repathCooldown);
    config.neighborRadius = jsonValue<float>(sys, "neighbor_radius", config.neighborRadius);
    config.slowingRadius = jsonValue<float>(sys, "slowing_radius", config.slowingRadius);
    config.fleeSafeRoomId = jsonValue<int>(sys, "flee_safe_room_id", config.fleeSafeRoomId);
    config.deltaTime = jsonValue<float>(sys, "delta_time", config.deltaTime);
    config.attackDamage = jsonValue<float>(sys, "attack_damage", config.attackDamage);
    config.lowHealthThreshold = jsonValue<float>(sys, "low_health_threshold", config.lowHealthThreshold);
    config.fleeHealAmount = jsonValue<float>(sys, "flee_heal_amount", config.fleeHealAmount);
    config.playerHealthConfig = jsonValue<float>(sys, "player_health", config.playerHealthConfig);
    config.startingAmmo = jsonValue<int>(sys, "starting_ammo", config.startingAmmo);
    config.attackInterval = jsonValue<float>(sys, "attack_interval", config.attackInterval);
    config.playerDamage = jsonValue<float>(sys, "player_damage", config.playerDamage);
    config.arrivalRadius = jsonValue<float>(sys, "arrival_radius", config.arrivalRadius);

    config.chaseFlockWeight = jsonValue<float>(sys, "chase_flock_weight", config.chaseFlockWeight);
    config.patrolFlockWeight = jsonValue<float>(sys, "patrol_flock_weight", config.patrolFlockWeight);
    config.groundPlaneZ = jsonValue<float>(sys, "ground_plane_z", config.groundPlaneZ);

    config.eventDrivenRepath = jsonFlag(sys, "event_driven_repath", config.eventDrivenRepath);
    config.rebuildHPAOnBlock = jsonFlag(sys, "rebuild_hpa_on_block", config.rebuildHPAOnBlock);
    config.losSampleStep = jsonValue<float>(sys, "los_sample_step", config.losSampleStep);

    config.squadMemoryDuration = jsonValue<float>(sys, "squad_memory_duration", config.squadMemoryDuration);
    config.allyLowHealthRatio = jsonValue<float>(sys, "ally_low_health_ratio", config.allyLowHealthRatio);
    config.squadRetreatRatio = jsonValue<float>(sys, "squad_retreat_ratio", config.squadRetreatRatio);
    config.squadPanicDeaths = jsonValue<int>(sys, "squad_panic_deaths", config.squadPanicDeaths);
    config.maxAttackers = jsonValue<int>(sys, "max_attackers", config.maxAttackers);
    config.supportFlankDistance = jsonValue<float>(sys, "support_flank_distance", config.supportFlankDistance);
    config.squadRetreatCooldown = jsonValue<float>(sys, "squad_retreat_cooldown", config.squadRetreatCooldown);

    const json spawn = data.contains("spawn") ? data["spawn"] : json::object();
    const json pStart = spawn.contains("player_start") ? spawn["player_start"] : json::object();
    const json pPhase2 = spawn.contains("player_phase2") ? spawn["player_phase2"] : json::object();

    config.playerStartX = jsonValue<float>(pStart, "x", config.playerStartX);
    config.playerStartY = jsonValue<float>(pStart, "y", config.playerStartY);
    config.playerStartZ = jsonValue<float>(pStart, "z", config.playerStartZ);
    config.playerStartPolygon = jsonValue<int>(pStart, "polygon", config.playerStartPolygon);
    config.playerPhase2X = jsonValue<float>(pPhase2, "x", config.playerPhase2X);
    config.playerPhase2Y = jsonValue<float>(pPhase2, "y", config.playerPhase2Y);
    config.playerPhase2Z = jsonValue<float>(pPhase2, "z", config.playerPhase2Z);

    const json pCombat = spawn.contains("player_combat") ? spawn["player_combat"] : json::object();
    config.playerCombatX = jsonValue<float>(pCombat, "x", config.playerCombatX);
    config.playerCombatY = jsonValue<float>(pCombat, "y", config.playerCombatY);
    config.playerCombatZ = jsonValue<float>(pCombat, "z", config.playerCombatZ);

    config.patrolRoute.clear();
    if (spawn.contains("patrol_route") && spawn["patrol_route"].is_array()) {
        for (const auto& p : spawn["patrol_route"]) {
            if (p.is_number_integer()) config.patrolRoute.push_back(p.get<int>());
        }
    }
    if (config.patrolRoute.empty()) {
        std::cerr << "[CONFIG] No patrol_route - falling back to {0,1,2,3}\n";
        config.patrolRoute = { 0, 1, 2, 3 };
    }

    return config;
}

// The map itself is data too: polygons (with vertices), the weighted edge list
// and the HPA* cluster definitions all come from the "map" block.
void loadNavConfig(NavMesh& nav, const json& data) {
    if (!data.contains("map")) {
        std::cerr << "[CONFIG] No 'map' block - the navmesh stays empty\n";
        return;
    }
    const json& map = data["map"];

    if (map.contains("polygons") && map["polygons"].is_array()) {
        for (const auto& p : map["polygons"]) {
            int id = jsonValue<int>(p, "id", -1);
            if (id < 0) continue;

            std::vector<Vector3> verts;
            if (p.contains("vertices") && p["vertices"].is_array()) {
                for (const auto& v : p["vertices"]) {
                    if (!v.is_array() || v.size() < 3) continue;
                    verts.push_back({ v[0].get<float>(), v[1].get<float>(), v[2].get<float>() });
                }
            }
            nav.addPolygon(id,
                jsonValue<float>(p, "center_x", 0.0f),
                jsonValue<float>(p, "center_y", 0.0f),
                jsonValue<float>(p, "center_z", 0.0f),
                verts);
        }
    }

    if (map.contains("edges") && map["edges"].is_array()) {
        for (const auto& e : map["edges"]) {
            int from = jsonValue<int>(e, "from", -1);
            int to = jsonValue<int>(e, "to", -1);
            if (from < 0 || to < 0) continue;
            nav.addConnection(from, to, jsonValue<float>(e, "cost", 1.0f));
        }
    }

    nav.finalizeMap();

    if (map.contains("clusters") && map["clusters"].is_array()) {
        for (const auto& c : map["clusters"]) {
            int id = jsonValue<int>(c, "id", -1);
            if (id < 0) continue;
            std::vector<int> polys;
            if (c.contains("polygons") && c["polygons"].is_array()) {
                for (const auto& pid : c["polygons"]) {
                    if (pid.is_number_integer()) polys.push_back(pid.get<int>());
                }
            }
            if (polys.empty()) continue;
            nav.addCluster(id, polys);
        }
    }
    else {
        std::cerr << "[CONFIG] No 'clusters' block - HPA* stays disabled, A* is used\n";
    }
}
// =====================================================================
// SIMULATION DRIVER
// ---------------------------------------------------------------------
// Six scripted phases. Each one exists to make a topic observable:
//
//   1 PATROL              baseline: waypoint loop + steering
//   2 PERCEPTION          only the scout sees the player; the squad acts on
//                         the shared sighting anyway (layer 1)
//   3 DYNAMIC NAV EVENT   corridor blocked mid-chase: every agent re-paths on
//                         the event, not on the cooldown (layer: dynamic env)
//   4 COMBAT SIGNAL       focus target elected, attack slots commit, support
//                         slots flank (layer 2)
//   5 SQUAD SURVIVAL      return fire wounds allies, morale breaks, squad
//                         retreats to the rally point (layer 3)
//   6 RECOVERY            corridor reopens, squad resumes patrol
// =====================================================================

namespace {

const char* stateName(AIState s) {
    switch (s) {
    case AIState::IDLE:   return "IDLE";
    case AIState::PATROL: return "PATROL";
    case AIState::CHASE:  return "CHASE";
    case AIState::ATTACK: return "ATTACK";
    case AIState::FLEE:   return "FLEE";
    case AIState::DEAD:   return "DEAD";
    }
    return "?";
}

const char* roleName(SquadRole r) {
    switch (r) {
    case SquadRole::ATTACKER: return "ATTACKER";
    case SquadRole::SUPPORT:  return "SUPPORT";
    case SquadRole::RETREAT:  return "RETREAT";
    }
    return "?";
}

struct SimContext {
    NavMesh& nav;
    Blackboard& blackboard;
    GameConfig& gc;
    std::vector<std::unique_ptr<Enemy>>& enemies;
    std::vector<Enemy*>& rawPointers;

    float simTime = 0.0f;
    float playerX = 0.0f;
    float playerY = 0.0f;
    float playerZ = 0.0f;

    SimContext(NavMesh& n, Blackboard& b, GameConfig& g,
        std::vector<std::unique_ptr<Enemy>>& e, std::vector<Enemy*>& r)
        : nav(n), blackboard(b), gc(g), enemies(e), rawPointers(r) {
        playerX = g.playerStartX;
        playerY = g.playerStartY;
        playerZ = g.playerStartZ;
        publishPlayer();
    }

    void publishPlayer() {
        blackboard.set<Vector3>(BlackboardKey::PLAYER_POSITION, { playerX, playerY, playerZ });
        blackboard.set<int>(BlackboardKey::PLAYER_POLYGON, nav.getPolygonAt(playerX, playerY));
    }

    void movePlayer(float px, float py, float pz) {
        playerX = px;
        playerY = py;
        playerZ = pz;
        publishPlayer();
    }

    // One frame: decide (update) -> steer (updateMovement) -> player return fire.
    void tick(int frame) {
        simTime += gc.deltaTime;
        blackboard.set<float>(BlackboardKey::SIM_TIME, simTime);
        std::cout << "--- Frame " << frame << " ---";

        for (auto& e : enemies) {
            e->update(gc.deltaTime);
        }

        for (auto& e : enemies) {
            e->updateMovement(nav.getPolygonCenter(e->getCurrentPolygon()), gc.deltaTime);
        }

        for (auto& e : enemies) {
            // Return fire only lands when the agent actually swung this tick.
            if (!e->isDead() && e->consumeAttackSwing()) {
                e->applyDamage(e->getPlayerDamage());
            }
        }

        std::cout << "\n";
    }

    void printSquadStatus() {
        std::vector<SquadMemberInfo> roster = blackboard.getRoster(BlackboardKey::SQUAD_ROSTER);
        if (roster.empty()) return;

        int spotters = 0;
        for (const auto& row : roster) {
            if (row.spottedPlayer) ++spotters;
        }

        std::cout << "  [SQUAD] size=" << roster.size()
            << " eyes-on-target=" << spotters
            << " inCombat=" << blackboard.getOr<int>(BlackboardKey::SQUAD_COMBAT_COUNT, 0)
            << " focus=" << blackboard.getOr<int>(BlackboardKey::SQUAD_FOCUS_TARGET, -1)
            << " dead=" << blackboard.getOr<int>(BlackboardKey::SQUAD_DEAD_COUNT, 0)
            << " worstAllyRatio="
            << blackboard.getOr<float>(BlackboardKey::ALLY_MIN_HEALTH_RATIO, 1.0f)
            << " playerKnown="
            << (blackboard.getOr<bool>(BlackboardKey::PLAYER_KNOWN, false) ? "yes" : "no")
            << " retreatOrdered="
            << (blackboard.getOr<bool>(BlackboardKey::SQUAD_RETREAT_ORDERED, false) ? "yes" : "no")
            << "\n";

        for (const auto& row : roster) {
            std::cout << "    agent " << row.agentId
                << " hp=" << row.health << "/" << row.maxHealth
                << " state=" << stateName(row.state)
                << " role=" << roleName(row.role)
                << " dist=" << row.distanceToPlayer
                << " eyes=" << (row.spottedPlayer ? "yes" : "no ")
                << (row.inCombat ? " ENGAGED" : "        ")
                << (row.agentId == blackboard.getOr<int>(BlackboardKey::SQUAD_FOCUS_TARGET, -1)
                    ? " <-- focus" : "")
                << "\n";
        }
    }

    int aliveCount() const {
        int n = 0;
        for (const auto& e : enemies) {
            if (!e->isDead()) ++n;
        }
        return n;
    }
};

} // namespace

int main() {
    NavMesh nav;
    Blackboard globalBlackboard;

    // ---- single parse of the whole config document ----
    const std::string configFile = "enemy_configs.json";
    json rawJson = loadConfigFile(configFile);

    GameConfig gc = loadGameConfig(rawJson);

    std::cout << "\n=====================================================\n";
    std::cout << ">>> LOADING NAVMESH FROM JSON (" << configFile << ") <<<\n";
    std::cout << "=====================================================\n";
    loadNavConfig(nav, rawJson);
    nav.setDynamicConfig(gc.rebuildHPAOnBlock, gc.losSampleStep);

    if (nav.getMapSize() == 0) {
        std::cerr << "[FATAL] The navmesh is empty - nothing to simulate\n";
        return 1;
    }

    // ==================== HPA* INITIALIZATION ====================
    std::cout << "\n=====================================================\n";
    std::cout << ">>> INITIALIZING HPA* (clusters from JSON) <<<\n";
    std::cout << "=====================================================\n";
    nav.initializeHPA();

    // ==================== HPA* / FUNNEL SANITY CHECK ====================
    {
        std::cout << "\n=====================================================\n";
        std::cout << ">>> TESTING HPA* PATHFINDING (Polygon 0 to 3) <<<\n";
        std::cout << "=====================================================\n";
        const int hpaStart = 0;
        const int hpaGoal = std::min(3, nav.getMapSize() - 1);
        std::vector<int> hpaPath = nav.hpaStar(hpaStart, hpaGoal);
        std::cout << "HPA* Final Polygon Path: ";
        for (size_t i = 0; i < hpaPath.size(); ++i) {
            std::cout << hpaPath[i] << (i < hpaPath.size() - 1 ? " -> " : "");
        }
        std::cout << "\n=====================================================\n\n";

        std::cout << "=====================================================\n";
        std::cout << ">>> TESTING FUNNEL ALGORITHM PATH SMOOTHING <<<\n";
        std::cout << "=====================================================\n";
        std::vector<int> rawPath = nav.aStar(hpaStart, hpaGoal);
        std::cout << "Raw A* Polygon Path: ";
        for (int id : rawPath) std::cout << id << " -> ";
        std::cout << "Goal\n";

        std::vector<Vector3> smoothWaypoints = nav.funnelAlgorithm(rawPath);
        std::cout << "Funnel Algorithm Output Waypoints (3D):\n";
        for (size_t i = 0; i < smoothWaypoints.size(); ++i) {
            std::cout << "  Waypoint [" << i << "]: ("
                << smoothWaypoints[i].x << ", "
                << smoothWaypoints[i].y << ", "
                << smoothWaypoints[i].z << ")\n";
        }
    }

    // ==================== ENEMY FACTORY (data driven) ====================
    std::vector<std::unique_ptr<Enemy>> enemyPtrs;
    std::vector<Enemy*> rawEnemyPtrs;

    int enemyCount = 0;
    if (rawJson.contains("spawn") && rawJson["spawn"].contains("enemies") &&
        rawJson["spawn"]["enemies"].is_array()) {
        enemyCount = static_cast<int>(rawJson["spawn"]["enemies"].size());
    }

    enemyPtrs.reserve(enemyCount);
    rawEnemyPtrs.reserve(enemyCount);

    for (int i = 0; i < enemyCount; ++i) {
        const json& entry = rawJson["spawn"]["enemies"][i];
        std::string type = jsonValue<std::string>(entry, "type", std::string("grunt"));
        EnemyConfig ec = loadEnemyConfig(rawJson, type);

        enemyPtrs.push_back(std::make_unique<Enemy>(i, nav, globalBlackboard, ec, gc));
        Enemy* raw = enemyPtrs.back().get();
        rawEnemyPtrs.push_back(raw);

        raw->setPosition(jsonValue<float>(entry, "x", 5.0f),
                         jsonValue<float>(entry, "y", 5.0f),
                         jsonValue<float>(entry, "z", gc.groundPlaneZ));
        raw->setPatrolPath(gc.patrolRoute);
        raw->setState(AIState::PATROL);
    }

    for (auto& e : enemyPtrs) {
        e->setEnemyList(rawEnemyPtrs);
    }

    if (enemyPtrs.empty()) {
        std::cerr << "[FATAL] No enemies in the spawn list\n";
        return 1;
    }

    // ==================== PLAYER ====================
    globalBlackboard.set<float>(BlackboardKey::SIM_TIME, 0.0f);
    globalBlackboard.set<float>(BlackboardKey::PLAYER_HEALTH, gc.playerHealthConfig);

    SimContext sim{ nav, globalBlackboard, gc, enemyPtrs, rawEnemyPtrs };

    std::cout << "================ STARTING SIMULATION ================\n";
    std::cout << "Agents: " << enemyPtrs.size()
        << " | max attackers: " << gc.maxAttackers
        << " | squad memory: " << gc.squadMemoryDuration << "s"
        << " | event-driven repath: " << (gc.eventDrivenRepath ? "on" : "off")
        << "\n\n";
    // ==================== PHASE 1: PATROL ====================
    std::cout << ">>> PHASE 1: Squad patrolling its route (player far away) <<<\n";
    for (int frame = 1; frame <= 3; ++frame) {
        sim.tick(frame);
    }
    sim.printSquadStatus();

    // ==================== PHASE 2: SHARED PERCEPTION ====================
    // The player is placed where only the long-range archetype can see it, so
    // the chase can only begin through a *shared* sighting.
    std::cout << "\n>>> PHASE 2: Player spotted by ONE agent, reported to the squad <<<\n";
    sim.movePlayer(gc.playerPhase2X, gc.playerPhase2Y, gc.playerPhase2Z);
    std::cout << "  Player moved to (" << gc.playerPhase2X << ", " << gc.playerPhase2Y
        << ") - polygon " << globalBlackboard.getOr<int>(BlackboardKey::PLAYER_POLYGON, -1) << "\n";

    int frame = 4;
    for (int i = 0; i < 10; ++i, ++frame) {
        sim.tick(frame);
    }
    sim.printSquadStatus();

    // ==================== PHASE 3: DYNAMIC NAVIGATION EVENT ====================
    std::cout << "\n>>> PHASE 3: Corridor blocked mid-chase (navigation event) <<<\n";
    nav.BlockChunks({ 1 });
    std::cout << "  Blocked polygons:";
    for (int i = 0; i < nav.getMapSize(); ++i) {
        if (nav.isBlocked(i)) std::cout << " " << i;
    }
    std::cout << "\n";

    for (int i = 0; i < 6; ++i, ++frame) {
        sim.tick(frame);
    }
    sim.printSquadStatus();

    // ==================== PHASE 4: COMBAT ====================
    std::cout << "\n>>> PHASE 4: Player closes in - focus target elected, support flanks <<<\n";
    sim.movePlayer(gc.playerCombatX, gc.playerCombatY, gc.playerCombatZ);
    std::cout << "  Player at (" << gc.playerCombatX << ", " << gc.playerCombatY
        << ") - polygon " << globalBlackboard.getOr<int>(BlackboardKey::PLAYER_POLYGON, -1) << "\n";

    for (int i = 0; i < 12; ++i, ++frame) {
        sim.tick(frame);
    }
    sim.printSquadStatus();

    // ==================== PHASE 5: SQUAD SURVIVAL ====================
    std::cout << "\n>>> PHASE 5: Player return fire - squad morale breaks <<<\n";
    for (int i = 0; i < 14; ++i, ++frame) {
        sim.tick(frame);
    }
    sim.printSquadStatus();

    // ==================== PHASE 6: RECOVERY ====================
    std::cout << "\n>>> PHASE 6: Corridor reopens, contact lost, squad regroups and patrols <<<\n";
    nav.UnBlockChunks({ 1 });
    sim.movePlayer(gc.playerStartX, gc.playerStartY, gc.playerStartZ);
    std::cout << "  Player withdraws to (" << gc.playerStartX << ", " << gc.playerStartY << ")\n";
    for (int i = 0; i < 16; ++i, ++frame) {
        sim.tick(frame);
    }
    sim.printSquadStatus();

    std::cout << "\n================ SIMULATION CONCLUDED ================\n";
    std::cout << "Alive: " << sim.aliveCount() << "/" << enemyPtrs.size() << "\n";
    return 0;
}
