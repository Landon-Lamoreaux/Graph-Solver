//
// Created by 101060665 on 12/7/2022.
//
// This is just used for Kruskal's algorithm.
//
#include <unordered_map>

using namespace std;

#ifndef GRAPH_CPP_DISSETS_H
#define GRAPH_CPP_DISSETS_H

class disSets {

public:
    int find( int num );
    void setUnion( int num1, int num2 );
    void initSets( int size );

private:
    unordered_map<int,int> set;

};

int disSets::find( int num )
{
    if( set[num] == num )
        return num;
    return find(set[num]);
}

void disSets::setUnion( int num1, int num2 )
{
    int a;
    int b;

    a = find(num1);
    b = find(num2);

    set[a] = b;
}

void disSets::initSets( int size )
{
    int i;

    for( i = 0; i < size; i++ )
        set[i] = i;
}

#endif //GRAPH_CPP_DISSETS_H
