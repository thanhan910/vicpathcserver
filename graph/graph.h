#pragma once

#include <memory>
#include <vector>
#include <unordered_map>

// Agent base class
struct Agent {
    virtual ~Agent() = default;
    virtual void print() const = 0;
};

// Node base class
struct Node {
    virtual ~Node() = default;
    virtual void print() const = 0;
    virtual double getHeuristic(const Node& other, const Agent& agent) const { return 0; } // Default heuristic
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

    void addNode(int id, NodePtr node);
    void addEdge(int from, int to, EdgePtr edge);
    void hideEdge(int from, int to);
    void restoreEdges();

    NodePtr getNode(int id) const;
    const std::vector<std::pair<int, EdgePtr>>& getEdges(int id) const;

    // A* Search Algorithm
    std::vector<int> aStarSearch(const Agent& agent, int start, int goal) const;

private:
    std::unordered_map<int, NodePtr> nodes;
    std::unordered_map<int, std::vector<std::pair<int, EdgePtr>>> adjacencyList;
    std::vector<std::tuple<int, int, EdgePtr>> hiddenEdges; // Temporarily hidden edges

    std::vector<int> reconstructPath(const std::unordered_map<int, int>& cameFrom, int current) const;
};
