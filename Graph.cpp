#include "Graph.h"
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include "DisSets.h"

void GraphAlgorithms::cycleFinder( vector<vector<int>> &cycles, vector<int> &path, vector<bool> &visited, int currNode, int lastNode )
{
    int i = 1;

    if( !cycles.empty() )
    {
        return;
    }
    if( lastNode == currNode && !path.empty() )
    {
        path.clear();
        path.push_back(currNode);
        path.push_back(lastNode);
        cycles.push_back(path);
        path.clear();
        return;
    }
    if(visited[currNode])
    {
        path.push_back(currNode);
        cycles.push_back(path);
        path.clear();
        return;
    }

    path.push_back(currNode);
    visited[currNode] = true;

    for( i = 0; i < adjList[currNode].size(); i++ )
    {
        cycleFinder( cycles, path, visited, adjList[currNode][i], currNode );
    }
    visited[currNode] = false;
    if( !path.empty() )
        path.pop_back();
}


vector<int> GraphAlgorithms::topSort()
{
    queue<int> q;
    int i;
    int vertex;
    vector<int> sorted;

    for( i = 1; i < adjList.size(); i++ )
    {
        if( indegree[i] == 0 )
        {
            q.push(i);
            indegree[i] = -1;
        }
    }
    while( !q.empty() )
    {
        vertex = q.front();
        q.pop();

        sorted.push_back(vertex);

        for( i = 0; i < adjList[vertex].size(); i++ )
        {
            indegree[adjList[vertex][i]]--;
        }
        for( i = 1; i < indegree.size(); i++ )
        {
            if( indegree[i] == 0 )
            {
                q.push(i);
                indegree[i] = -1;
            }
        }
    }
    return sorted;
}


void GraphAlgorithms::getGraph( ifstream &fin )
{
    string temp;
    int i = 0;
    string num1;
    string num2;
    string weight;
    getline( fin, temp);
    graphType = temp.substr(0, temp.find('{')-1 );
    if( temp[temp.size()-1] != '{' )
        getline( fin, temp );
    while( temp != "}" )
    {
        getline(fin,temp);
        if( temp != "}" )
        {
            graph.resize(i+1);
            graph[i].resize(2);
            num1 = temp.substr(0, temp.find('-'));
            graph[i][0] = stoi(num1);
            if( temp.find('[') == string::npos )
                num2 = temp.substr(temp.find('-') + 3, temp.find(';') - temp.find('-') - 3);
            else if( temp.find("weight") != string::npos ) {
                num2 = temp.substr(temp.find('-') + 3, temp.find('[') - temp.find('-') - 3);
                weight = temp.substr( temp.find("weight=") + 8, temp.find(']') - temp.find("weight=") - 9 );
                weights.push_back(stoi(weight));
            }
            else{
                num2 = temp.substr(temp.find('-') + 3, temp.find('[') - temp.find('-') - 3);
            }
            graph[i][1] = stoi(num2);
            i++;
        }
    }
    graphToAdja();
    createAdjMatrix();
    calcIndegree();
}


void GraphAlgorithms::graphToAdja()
{
    int i;
    int greatestVertex = 0;

    adjList.resize(graph.size() + 1);

    for( i = 0; i < graph.size(); i++ )
    {
        adjList[graph[i][0]].push_back(graph[i][1]);
        if( graphType == "graph" )
            adjList[graph[i][1]].push_back(graph[i][0]);
        if( graph[i][1] > greatestVertex )
            greatestVertex = graph[i][1];
    }
    adjList.resize(greatestVertex + 1);
}

void printTop( vector<int> topSort )
{
    int i;
    ofstream fout;

    fout.open( "TopSort.gv", std::ofstream::out | std::ofstream::trunc );

    fout << "digraph {" << endl;
    cout << endl << "Topological Sort: ";
    for( i = 0; i < topSort.size(); i++ )
    {
        cout << topSort[i];
        if( i < topSort.size() - 1 )
        {
            cout << ", ";
            fout << topSort[i] << " -> " << topSort[i+1] << ";" << endl;
        }
    }
    fout << "}";
}

void GraphAlgorithms::calcIndegree()
{
    int i;

    indegree.resize(adjList.size());
    for( i = 0; i < graph.size(); i++ )
    {
        indegree[graph[i][1]]++;
    }
}


void GraphAlgorithms::directedGraphSolver()
{
    vector<vector<int>> cycles;
    vector<bool> visited;
    vector<int> path;
    int i;

    visited.resize( adjList.size(), false );
    cycleFinder( cycles, path, visited, graph[0][0], graph[0][0] );

    if( cycles.empty() )
    {
        cout << endl << "Acyclic Graph" << endl;
        printTop( topSort() );
    }
    else
    {
        cout << endl << "Cycles Exist" << endl;
        cout << "First Cycle: ";
        for( i = 0; i < cycles[0].size(); i++ )
        {
            cout << cycles[0][i];
            if( i < cycles[0].size() - 1 )
            {
                cout << " -> ";
            }
        }
        cout << endl;
    }
}


void GraphAlgorithms::unWeightedShortestPath( vector<vector<int>> &path, int source )
{
    int i;
    queue<int> q;
    vector<bool> visited;
    vector<int> dist;

    dist.resize(adjList.size(), 2147483647);
    path.resize(adjList.size());
    visited.resize(adjList.size(), false);

    dist[source] = 0;

    visited[source] = true;
    q.push(source);
    path[source].push_back(source);

    while(!q.empty())
    {
        i = q.front();
        q.pop();

        for( int v : adjList[i] )
        {
            if(!visited[v])
            {
                dist[v] = dist[i] + 1;
                visited[v] = true;
                q.push(v);
                path[v] = path[i];
                path[v].push_back(v);
            }
        }
    }
}

void GraphAlgorithms::shortedPathrunner()
{
    vector<vector<int>> path;
    int i, j;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    unWeightedShortestPath(path, graph[0][0] ); // The second parameter is the starting vertex, change it to change the starting spot.

    cout << endl << endl << "Shortest Path To Each Vertex: " << endl;
    for( i = 1; i < path.size(); i++ )
    {
        cout << i << ": ";
        for( j = 0; j < path[i].size(); j++ )
        {
            cout << path[i][j];
            if( j < path[i].size() - 1 )
            {
                cout << " -> ";
            }
        }
        if( !path[i].empty() )
            cout << endl;
    }
}


string GraphAlgorithms::graphVariety()
{
    return graphType;
}

void GraphAlgorithms::createAdjMatrix()
{
    int i;

    adjmat.resize( adjList.size() );
    for( i = 0; i < adjList.size(); i++ )
    {
        adjmat[i].resize( adjList.size(), 0 );
    }

    for( i = 0; i < graph.size(); i++ )
    {
        if( weights.empty() ) {
            adjmat[graph[i][0]][graph[i][1]] = 1;
            if( graphType == "graph" )
            {
                adjmat[graph[i][1]][graph[i][0]] = 1;
            }
        }
        else{
            adjmat[graph[i][0]][graph[i][1]] = weights[i];
            if( graphType == "graph" )
            {
                adjmat[graph[i][1]][graph[i][0]] = weights[i];
            }
        }
    }
}


void GraphAlgorithms::prims( vector<int> &tree, int startSpot )
{
    vector<int> minWeight;
    vector<bool> visited;
    int i;
    int j;
    int min = 2147483647;
    int minI;

    tree.resize(adjList.size(), -1);
    minWeight.resize(adjList.size(), 2147483647);
    visited.resize(adjList.size(), false);

    minWeight[startSpot] = 0;
    tree[startSpot] = 0;

    for( i = 0; i < adjList.size(); i++ )
    {
        min = 2147483647;
        for( j = 0; j < adjList.size(); j++ )
        {
            if( !visited[j] && minWeight[j] < min )
            {
                min = minWeight[j];
                minI = j;
            }
        }

        visited[minI] = true;

        for( j = 0; j < adjList.size(); j++ )
        {
            if( adjmat[minI][j] != 0 && !visited[j] && adjmat[minI][j] < minWeight[j] )
            {
                tree[j] = minI;
                minWeight[j] = adjmat[minI][j];
            }
        }
    }
}


void GraphAlgorithms::primsPrinter()
{
    vector<int> tree;
    int i;
    int j;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    int startSpot = 1; // This is the starting vertex, change it to change where the program starts.

    ofstream fout;
    vector<pair<int,int>> treeEdges;
    int weightSum = 0;

    prims( tree, startSpot );

    treeEdges.resize(tree.size());

    fout.open( "MinSpanTree.gv", std::ofstream::out | std::ofstream::trunc );


    cout << endl << "Minimum Spanning Tree: " << endl;
    cout << "Starting Vertex: " << startSpot << endl;
    fout << "graph {" << endl;
    for( i = 1; i < tree.size(); i++ )
    {
        if( tree[i] != 0 )
        {
            cout << tree[i] << " - " << i << setw(5) << "W: " << adjmat[i][tree[i]] << endl;
            fout << tree[i] << " -- " << i << "[label=" << '"' << adjmat[i][tree[i]] << '"'
                 << ",weight=" << '"' << adjmat[i][tree[i]] << '"' << ",color=red" << "];" << endl;
            treeEdges[i].first = tree[i];
            treeEdges[i].second = i;
            weightSum += adjmat[i][tree[i]];
        }
    }
    cout << "Sum of Min Weights: " << weightSum << endl;

    for( i = 0; i < graph.size(); i++ )
    {
        if( !alreadyPrinted( treeEdges, graph[i][0], graph[i][1] ) )
        {
            fout << graph[i][0] << " -- " << graph[i][1] << "[label=" << '"' << weights[i] << '"'
                 << ",weight=" << '"' << weights[i] << '"' << "];" << endl;
        }
    }
    fout << "label=" << '"' << "Sum of Min Weights: " << weightSum << '"' << ";" << endl;
    fout << "}";
}

// Checks if an edge is in the vector of pairs.
bool alreadyPrinted( vector<pair<int,int>> treeEdges, int vert1, int vert2 )
{
    int i;

    for( i = 0; i < treeEdges.size(); i++ )
    {
        if( treeEdges[i].first == vert1 && treeEdges[i].second == vert2 ||
            treeEdges[i].first == vert2 && treeEdges[i].second == vert1 )
        {
            return true;
        }
    }
    return false;
}


bool GraphAlgorithms::isWeighted()
{
    if( weights.empty() )
    {
        return false;
    }
    return true;
}


vector<int> GraphAlgorithms::Dijkstras( vector<vector<int>> &path, int source )
{
    int i, j;
    int min = 2147483647;
    int minIndex;
    vector<bool> visited;
    vector<int> dist;
    int k;

    dist.resize(adjList.size(), 2147483647);
    path.resize(adjList.size());
    visited.resize(adjList.size(), false);

    dist[source] = 0;

    path[source].push_back(source);

    for( k = 0; k < adjList.size(); k++ )
    {
        min = 2147483647;
        for( i = 1; i < visited.size(); i++ )
        {
            if(!visited[i] && dist[i] <= min )
            {
                min = dist[i];
                minIndex = i;
            }
        }
        visited[minIndex] = true;

        for( j = 0; j < visited.size(); j++ )
        {
            if( !visited[j] && adjmat[minIndex][j] && dist[minIndex] != 2147483647 && dist[minIndex] + adjmat[minIndex][j] < dist[j] )
            {
                dist[j] = dist[minIndex] + adjmat[minIndex][j];
                path[j] = path[minIndex];
                path[j].push_back(j);
            }
        }
    }
    return dist;
}


void GraphAlgorithms::DijkstraPrinter()
{
    int i, j;
    vector<vector<int>> path;
    vector<int> dist;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // The second parameter is the starting vertex s. Change it to change the starting point.
    dist = Dijkstras( path, graph[0][0] );

    cout << endl << "Shortest Paths:" << endl;
    for( i = 1; i < path.size(); i++ )
    {
        cout << i << ": ";
        cout << setw(5) << "W: " << dist[i] << setw(5);
        for( j = 0; j < path[i].size(); j++ )
        {
            cout << path[i][j];
            if( j < path[i].size() - 1 )
            {
                cout << " - > ";
            }
        }
        cout << endl;
    }
}


int GraphAlgorithms::SourceSink( int source, int sink )
{
    int maxFlow = 0;
    int i;
    int flow;
    int edgeFlow;
    vector<vector<int>> resGraph;
    vector<int> path;

    path.resize(adjList.size());
    resGraph = adjmat;

    while( SinkToSourcePathFinder( resGraph, source, sink, path ) )
    {
        flow = 2147483647;
        for( i = sink; i != source ; i = path[i] )
        {
            edgeFlow = path[i];
            if( flow > resGraph[edgeFlow][i] )
            {
                flow = resGraph[edgeFlow][i];
            }
        }

        for( i = sink; i != source; i = path[i] )
        {
            edgeFlow = path[i];
            resGraph[i][edgeFlow] += flow;
            resGraph[edgeFlow][i] -= flow;
        }

        maxFlow = maxFlow + flow;
    }
    return maxFlow;
}


bool SinkToSourcePathFinder( vector<vector<int>> resGraph, int source, int sink, vector<int> &path )
{
    vector<bool> visited;
    queue<int> queue;
    int front;
    int i;

    visited.resize(path.size());
    queue.push(source);
    visited[source] = true;
    path[source] = -1;

    while( !queue.empty() )
    {
        front = queue.front();
        queue.pop();

        for( i = 0; i < resGraph.size(); i++ )
        {
            if( !visited[i] && resGraph[front][i] > 0)
            {
                if( i == sink )
                {
                    path[i] = front;
                    return true;
                }
                queue.push(i);
                path[i] = front;
                visited[i] = true;
            }
        }
    }
    return false;
}


void GraphAlgorithms::fordsPrinter()
{
    int maxFlow;
    int i;
    ofstream fout;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    //Changing these parameters will change the source and sink.
    maxFlow = SourceSink(graph[0][0], graph[graph.size()-1][1]); // The first one is the source, the second is the sink.

    fout.open( "Fords.gv", std::ofstream::out | std::ofstream::trunc );
    fout << "digraph {" << endl;
    fout << "label=" << '"' << "Max Flow: " << maxFlow << '"' << ";" << endl;

    cout << endl << "Maximum Flow: " << maxFlow << endl;

    for( i = 0; i < graph.size(); i++ )
    {
        fout << graph[i][0] << " -> " << graph[i][1] << "[label=" << '"' << weights[i] << '"'
             << "weight=" << '"' << weights[i] << '"' << "];" << endl;
    }
    fout << "}";
}


vector<pair<int,int>> GraphAlgorithms::kruskals( vector<int> &minWeights )
{
    vector<pair<int,int>> mst;
    vector<pair<int,int>> pgraph;
    pair<int,int> edge;
    vector<int> sortedIndexes;
    int i;

    pgraph.resize(graph.size());

    for( i = 0; i < graph.size(); i++ )
    {
        pgraph[i].first = graph[i][0];
        pgraph[i].second = graph[i][1];
    }

    disSets sets;

    sortedIndexes = sortEdges();
    sets.initSets( adjList.size() );

    i = 0;
    while( mst.size() != adjList.size() - 2 )
    {
        edge = pgraph[sortedIndexes[i]];

        int u = sets.find(edge.first);
        int v = sets.find(edge.second);

        if( u != v )
        {
            mst.push_back( edge );
            minWeights.push_back(weights[sortedIndexes[i]]);
            sets.setUnion(u,v);
        }

        i++;
    }

    return mst;
}

// Sorts the edges by there weight and returns a vector of the indexes
// corresponding to the smallest to the largest weight.
vector<int> GraphAlgorithms::sortEdges()
{
    vector<int> tempWeights;
    vector<int> sortedIndexes;
    int i, j;

    tempWeights = weights;

    sortedIndexes.resize(tempWeights.size());

    for( i = 0; i < tempWeights.size(); i++ )
    {
        sortedIndexes[i] = i;
    }

    for( i = 0; i < tempWeights.size(); i++ )
    {
        for( j = 0; j < tempWeights.size() - i - 1; j++ )
        {
            if( tempWeights[j] > tempWeights[j+1] )
            {
                swap(tempWeights[j], tempWeights[j+1]);
                swap(sortedIndexes[j], sortedIndexes[j+1]);
            }
        }
    }
    return sortedIndexes;
}


void GraphAlgorithms::kruskalPrinter()
{
    vector<pair<int,int>> mst;
    int i;
    ofstream fout;
    vector<int> minWeights;
    int minWeight = 0;

    fout.open( "Kruskals.gv", std::ofstream::out | std::ofstream::trunc );

    mst = kruskals(minWeights);

    fout << "graph {" << endl;
    cout << endl << "Kruskal's Minimum Tree:" << endl;
    for( i = 0; i < mst.size(); i++ )
    {
        cout << mst[i].first << " - " << mst[i].second << " W: " << minWeights[i] << endl;
        fout << mst[i].first << " -- " << mst[i].second << "[label=" << '"' << minWeights[i] << '"'
             << "weight=" << '"' << minWeights[i] << '"' << ",color=red" << "];" << endl;
        minWeight += minWeights[i];
    }
    cout << "Sum of Min Weights: " << minWeight << endl;
    for( i = 0; i < graph.size(); i++ )
    {
        if( !alreadyPrinted( mst, graph[i][0], graph[i][1] ) )
        {
            fout << graph[i][0] << " -- " << graph[i][1] << "[label=" << '"' << weights[i] << '"'
                 << ",weight=" << '"' << weights[i] << '"' << "];" << endl;
        }
    }
    fout << "label=" << '"' << "Sum of Min Weights: " << minWeight << '"' << ";" << endl;
    fout << "}";
}


bool GraphAlgorithms::eulers( vector<pair<int,int>> &edgeList, vector<vector<int>> &tempGraph, int start )
{
    int edgeCount = 0;
    int i, j;
    pair<int,int> edge;

    edgeCount = edgeNum(tempGraph);

    for( i = 0; i < adjmat.size(); i++ )
    {
        if( tempGraph[start][i] != 0 )
        {
            if( edgeCount <= 1 || !bridge( tempGraph, i) )
            {
                edge.first = start;
                edge.second = i;
                edgeList.push_back(edge);
                tempGraph[start][i] = 0;
                tempGraph[i][start] = 0;
                edgeCount--;
                eulers(edgeList, tempGraph, i);
                edgeCount = edgeNum(tempGraph);
                if( edgeCount > 1 )
                {
                    tempGraph[start][i] = 1;
                    tempGraph[i][start] = 1;
                }
            }
        }
    }
    if( edgeCount == 0 )
        return true;
    else{
        return false;
    }
}

int edgeNum( vector<vector<int>> tempGraph )
{
    int edgeCount = 0;
    int i, j;

    for( i = 0; i < tempGraph.size(); i++ )
    {
        for( j = i; j < tempGraph.size(); j++ )
        {
            if( tempGraph[i][j] != 0 )
            {
                edgeCount++;
            }
        }
    }
    return edgeCount;
}


bool bridge( vector<vector<int>> tempGraph, int vertex )
{
    int edgeCount = 0;
    int i;

    for( i = 0; i < tempGraph.size(); i++ )
    {
        if( tempGraph[vertex][i] != 0 )
            edgeCount++;
        if( edgeCount > 1 )
            return false;
    }
    return true;
}


void GraphAlgorithms::eulersPrinter()
{
    vector<pair<int,int>> edges;
    vector<vector<int>> tempGraph;
    int start = 1;
    int i;
    ofstream fout;


    tempGraph = adjmat;
    start = startVertex();

    if( eulers( edges, tempGraph, start ) )
    {
        fout.open( "Eulers.gv", std::ofstream::out | std::ofstream::trunc );

        fout << "graph {" << endl;
        cout << endl << "Eluers Circuit: " << endl;
        cout << "Start Spot: " << start << endl;
        for( i = 0; i < edges.size(); i++ )
        {
            cout << edges[i].first << " - " << edges[i].second << endl;
            fout << edges[i].first << " -- " << edges[i].second << ";" << endl;
        }
        fout << "}";
    }
    else
    {
        cout << endl << "No Euler Circuits." << endl;
    }
}

// Finds the optimal starting vertex to find an Euler circuit.
int GraphAlgorithms::startVertex()
{
    int i;

    for( i = 0; i < adjList.size(); i++ )
    {
        if( adjList[i].size() % 2 != 0 )
            return i;
    }
    return 1;
}