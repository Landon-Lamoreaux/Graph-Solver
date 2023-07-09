#include <iostream>
#include "Graph.h"
#include <fstream>
#include <vector>

// Created by Landon Lamoreaux.

/* To use this program just provide a dot language file in the command line, and the program will read it in and output
 * the results of the graph. This program cannot read in anything fancy from the file at the moment.
 * Things like subgraphs, labels, and such will not work. I can only read in a list of edges, weighted or unweighted.
 * If you need examples of what to use as a test case, check the included txt files, they are good examples of what I
 * can read in. I have only tried reading in text files, it can probably read in .dot and .gv as long as they look the
 * same on the inside as a text file. But I haven't tested these.
 *
 * Any functions that were required to have their output be in a dot language file, will output to the screen and to a
 * file with the extension .gv, if this needs to be in a .dot file, just find the function definitions for the
 * print/runner function of that function, and you can change the output file name from there.
 *
 * If a function needs to be able to solve from a given vertex, the function call for that function is pointed out in
 * graph.cpp by a line of dashes crossing the screen. Further instructions to change those starting spots are given at
 * that point as it depends on the function. Defaults are typically just the first vertex in the provided file.
 * So if the first line was "1 -- 5", the starting spot would typically be 1. I would've liked to use user input for it
 * but for some reason clion wasn't waiting for user input when I was trying to use cin, so I gave up on that part.
 */


using namespace std;

int main( int argc, char** argv )
{
    ifstream inFile;
    vector<int> path;
    vector<vector<int>> cycles;
    vector<bool> visited;
    GraphAlgorithms graphClass;
    int size;
    vector<int> topSorted;

    if( argc != 2 )
    {
        cout << "Invalid number of arguments" << endl;
        return 0;
    }

    inFile.open( argv[1] );
    if( !inFile.is_open() )
    {
        cout << "Can't open selected file.";
        return 0;
    }
    graphClass.getGraph( inFile );

    if(graphClass.graphVariety() == "digraph")
        graphClass.directedGraphSolver();

    if(!graphClass.isWeighted())
        graphClass.shortedPathrunner();

    if( graphClass.graphVariety() == "graph" )
        graphClass.eulersPrinter();

    if( graphClass.isWeighted() )
    {
        if( graphClass.graphVariety() == "graph" )
        {
            graphClass.primsPrinter();
            graphClass.kruskalPrinter();
        }
        graphClass.DijkstraPrinter();
        if( graphClass.graphVariety() == "digraph" )
        {
            graphClass.fordsPrinter();
        }
    }

    return 0;
}