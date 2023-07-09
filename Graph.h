#pragma once
#include <vector>
#include <fstream>
#include <queue>
#include <iostream>

using namespace std;

#ifndef PROGRAMMING_ASSIGNMENT_3_GRAPH_H
#define PROGRAMMING_ASSIGNMENT_3_GRAPH_H

class GraphAlgorithms
{
public:
    void cycleFinder( vector<vector<int>> &cycles, vector<int> &path, vector<bool> &visited, int currNode, int lastNode );
    vector<int> topSort();
    void getGraph( ifstream &fin );
    void graphToAdja();
    void calcIndegree();
    void directedGraphSolver();
    void shortedPathrunner();
    void unWeightedShortestPath( vector<vector<int>> &path, int source );
    string graphVariety();
    void createAdjMatrix();
    void prims( vector<int> &tree, int startSpot );
    void primsPrinter();
    bool isWeighted();
    vector<int> Dijkstras( vector<vector<int>> &path, int source );
    void DijkstraPrinter();
    int SourceSink( int source, int sink );
    void fordsPrinter();
    vector<pair<int,int>> kruskals( vector<int> &minWeights );
    vector<int> sortEdges();
    void kruskalPrinter();
    bool eulers( vector<pair<int,int>> &edgeList, vector<vector<int>> &tempGraph, int start );
    void eulersPrinter();
    int startVertex();

private:
    vector<vector<int>> graph;
    string graphType;
    vector<vector<int>> adjList;
    vector<int> indegree;
    vector<int> weights;
    vector<vector<int>> adjmat;

};

void printTop( vector<int> topSort );

bool SinkToSourcePathFinder( vector<vector<int>> resGraph, int source, int sink, vector<int> &path );

bool bridge( vector<vector<int>> tempGraph, int vertex );

int edgeNum( vector<vector<int>> tempGraph );

bool alreadyPrinted( vector<pair<int,int>> treeEdges, int vert1, int vert2 );

#endif //PROGRAMMING_ASSIGNMENT_3_GRAPH_H
