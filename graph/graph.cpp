#include "graph.h"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>
#include <queue>
#include <functional>
#include <cmath>
#include <limits>
#include <algorithm>

void Graph::addNode(int id, NodePtr node) {
    nodes[id] = node;
}

void Graph::addEdge(int from, int to, EdgePtr edge) {
    adjacencyList[from].emplace_back(to, edge);
}

void Graph::hideEdge(int from, int to) {
    for (auto it = adjacencyList[from].begin(); it != adjacencyList[from].end(); ++it) {
        if (it->first == to) {
            hiddenEdges.emplace_back(from, to, it->second);
            adjacencyList[from].erase(it);
            break;
        }
    }
}

void Graph::restoreEdges() {
    for (const auto& [from, to, edge] : hiddenEdges) {
        adjacencyList[from].emplace_back(to, edge);
    }
    hiddenEdges.clear();
}

Graph::NodePtr Graph::getNode(int id) const {
    return nodes.at(id);
}

const std::vector<std::pair<int, Graph::EdgePtr>>& Graph::getEdges(int id) const {
    return adjacencyList.at(id);
}


// A* Search Algorithm
std::vector<int> Graph::aStarSearch(const Agent& agent, int start, int goal) const {
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
    double startHeuristic = getNode(start)->getHeuristic(*getNode(goal), agent);
    std::cout << "Start Heuristic: " << startHeuristic << "\n";
    openSet.push({start, startHeuristic});

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
                double fScore = tentativeGScore + getNode(neighbor)->getHeuristic(*getNode(goal), agent);
                openSet.push({neighbor, fScore});
                cameFrom[neighbor] = current;
            }
        }
    }

    return {}; // No path found
}


std::vector<int> Graph::reconstructPath(const std::unordered_map<int, int>& cameFrom, int current) const {
    std::vector<int> path;
    while (cameFrom.count(current)) {
        path.push_back(current);
        current = cameFrom.at(current);
    }
    path.push_back(current);
    std::reverse(path.begin(), path.end());
    return path;
}

