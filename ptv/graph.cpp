#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>
#include <queue>
#include <functional>
#include <cmath>
#include <limits>

// Node base class
struct Node {
    virtual ~Node() = default;
    virtual void print() const = 0;
};

// Edge base class
struct Edge {
    virtual ~Edge() = default;
    virtual double getCost() const = 0;
    virtual void print() const = 0;
};

// Graph class with edge state management
class Graph {
public:
    using NodePtr = std::shared_ptr<Node>;
    using EdgePtr = std::shared_ptr<Edge>;

    void addNode(int id, NodePtr node) {
        nodes[id] = node;
    }

    void addEdge(int from, int to, EdgePtr edge) {
        adjacencyList[from].emplace_back(to, edge);
    }

    void hideEdge(int from, int to) {
        for (auto it = adjacencyList[from].begin(); it != adjacencyList[from].end(); ++it) {
            if (it->first == to) {
                hiddenEdges.emplace_back(from, to, it->second);
                adjacencyList[from].erase(it);
                break;
            }
        }
    }

    void restoreEdges() {
        for (const auto& [from, to, edge] : hiddenEdges) {
            adjacencyList[from].emplace_back(to, edge);
        }
        hiddenEdges.clear();
    }

    NodePtr getNode(int id) const {
        return nodes.at(id);
    }

    const std::vector<std::pair<int, EdgePtr>>& getEdges(int id) const {
        return adjacencyList.at(id);
    }

    // A* Search Algorithm
    std::vector<int> aStarSearch(int start, int goal,
                                 std::function<double(const NodePtr&, const NodePtr&)> heuristic) const {
        struct NodeRecord {
            int id;
            double cost; // f(n) = g(n) + h(n)
        };

        auto compare = [](const NodeRecord& a, const NodeRecord& b) {
            return a.cost > b.cost; // Min-heap
        };

        std::priority_queue<NodeRecord, std::vector<NodeRecord>, decltype(compare)> openSet(compare);
        std::unordered_map<int, double> gScore; // Cost to reach each node
        std::unordered_map<int, int> cameFrom; // Path reconstruction

        // Initialize
        gScore[start] = 0;
        openSet.push({start, heuristic(getNode(start), getNode(goal))});

        while (!openSet.empty()) {
            int current = openSet.top().id;
            openSet.pop();

            // Goal reached
            if (current == goal) {
                return reconstructPath(cameFrom, current);
            }

            // Explore neighbors
            for (const auto& [neighbor, edge] : getEdges(current)) {
                double tentativeGScore = gScore[current] + edge->getCost();
                if (tentativeGScore < gScore[neighbor] || !gScore.count(neighbor)) {
                    gScore[neighbor] = tentativeGScore;
                    double fScore = tentativeGScore + heuristic(getNode(neighbor), getNode(goal));
                    openSet.push({neighbor, fScore});
                    cameFrom[neighbor] = current;
                }
            }
        }

        return {}; // No path found
    }

private:
    std::unordered_map<int, NodePtr> nodes;
    std::unordered_map<int, std::vector<std::pair<int, EdgePtr>>> adjacencyList;
    std::vector<std::tuple<int, int, EdgePtr>> hiddenEdges; // Temporarily hidden edges

    std::vector<int> reconstructPath(const std::unordered_map<int, int>& cameFrom, int current) const {
        std::vector<int> path;
        while (cameFrom.count(current)) {
            path.push_back(current);
            current = cameFrom.at(current);
        }
        path.push_back(current);
        std::reverse(path.begin(), path.end());
        return path;
    }
};

// LocationNode with coordinates
struct LocationNode : public Node {
    std::string name;
    double x, y;

    LocationNode(const std::string& name, double x, double y)
        : name(name), x(x), y(y) {}

    void print() const override {
        std::cout << "LocationNode [Name: " << name << ", Coordinates: (" << x << ", " << y << ")]\n";
    }
};

// TrafficNode with traffic delay
struct TrafficNode : public Node {
    std::string name;
    int trafficDelay; // in minutes

    TrafficNode(const std::string& name, int trafficDelay)
        : name(name), trafficDelay(trafficDelay) {}

    void print() const override {
        std::cout << "TrafficNode [Name: " << name << ", Traffic Delay: " << trafficDelay << " mins]\n";
    }
};

// WeatherNode with temperature
struct WeatherNode : public Node {
    std::string name;
    double temperature; // in Celsius

    WeatherNode(const std::string& name, double temperature)
        : name(name), temperature(temperature) {}

    void print() const override {
        std::cout << "WeatherNode [Name: " << name << ", Temperature: " << temperature << "\u00b0C]\n";
    }
};

// WeightedEdge with cost
struct WeightedEdge : public Edge {
    double weight;

    WeightedEdge(double weight) : weight(weight) {}

    double getCost() const override { return weight; }

    void print() const override {
        std::cout << "WeightedEdge [Weight: " << weight << "]\n";
    }
};

// TimedEdge with time in seconds
struct TimedEdge : public Edge {
    int time; // in seconds

    TimedEdge(int time) : time(time) {}

    double getCost() const override { return static_cast<double>(time); }

    void print() const override {
        std::cout << "TimedEdge [Time: " << time << " seconds]\n";
    }
};

// TollEdge with toll cost
struct TollEdge : public Edge {
    double toll;

    TollEdge(double toll) : toll(toll) {}

    double getCost() const override { return toll; }

    void print() const override {
        std::cout << "TollEdge [Toll: $" << toll << "]\n";
    }
};

// Heuristic function for LocationNode using Euclidean distance
double locationHeuristic(const Graph::NodePtr& a, const Graph::NodePtr& b) {
    auto locA = std::dynamic_pointer_cast<LocationNode>(a);
    auto locB = std::dynamic_pointer_cast<LocationNode>(b);
    if (locA && locB) {
        return std::sqrt(std::pow(locA->x - locB->x, 2) + std::pow(locA->y - locB->y, 2));
    }
    return std::numeric_limits<double>::infinity();
}

// Heuristic function for TrafficNode based on traffic delay difference
double trafficHeuristic(const Graph::NodePtr& a, const Graph::NodePtr& b) {
    auto trafficA = std::dynamic_pointer_cast<TrafficNode>(a);
    auto trafficB = std::dynamic_pointer_cast<TrafficNode>(b);
    if (trafficA && trafficB) {
        return std::abs(trafficA->trafficDelay - trafficB->trafficDelay);
    }
    return std::numeric_limits<double>::infinity();
}

// Heuristic function for WeatherNode based on temperature difference
double weatherHeuristic(const Graph::NodePtr& a, const Graph::NodePtr& b) {
    auto weatherA = std::dynamic_pointer_cast<WeatherNode>(a);
    auto weatherB = std::dynamic_pointer_cast<WeatherNode>(b);
    if (weatherA && weatherB) {
        return std::abs(weatherA->temperature - weatherB->temperature);
    }
    return std::numeric_limits<double>::infinity();
}

int main() {
    Graph graph;

    // Add nodes
    graph.addNode(1, std::make_shared<LocationNode>("A", 0, 0));
    graph.addNode(2, std::make_shared<TrafficNode>("B", 5));
    graph.addNode(3, std::make_shared<WeatherNode>("C", 22.5));

    // Add edges
    graph.addEdge(1, 2, std::make_shared<TimedEdge>(300)); // 5 minutes
    graph.addEdge(2, 3, std::make_shared<TollEdge>(2.5));  // $2.50 toll

    // Perform A* Search
    auto path = graph.aStarSearch(1, 3, locationHeuristic);

    std::cout << "Path found: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << "\n";

    return 0;
}