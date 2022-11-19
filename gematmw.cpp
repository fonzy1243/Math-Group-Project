/*
 * This program is based on the code submitted to the geeksforgeeks website
 * by shivanisinghss2110
 * Source: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/
 * Time complexity: O(V^2)
 * Space complexity: O(V)
 */

#include <iostream>
#include <limits.h>

using std::cout;
using std::endl;

// Amount of stops (represented as vertices)
#define V 9

// Find the vertex with the minimum distance
int minDistance(int dist[], bool visited[])
{
    int min = INT_MAX,
        min_index;

    for (int v = 0; v < V; v++)
    {
        if (visited[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;
    }

    return min_index;
}

// Print distances to stops
void printTravelTime(int dist[])
{
    cout << "Stop \t Travel time from source" << endl;
    for (int i = 0; i < V; i++)
        cout << i << "\t " << dist[i] << endl;
}

// Dijkstra's algorithm implemented with adjacency matrix
void shortestPathFrom(int graph[V][V], int src)
{
    int dist[V];    // Distance from source to i

    bool visited[V]; // Will be true if vertex is visited

    for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, visited[i] = false;

    dist[src] = 0;  // Distance from 1st vertice to itself is always 0

    // Find shortest path for vertices
    for (int count = 0; count < V - 1; count++)
    {
        /*
         * Visit the unvisited vertex with the smallest known distance from
         * the start vertex.
         */
        int u = minDistance(dist, visited);

        // Set visited to true
        visited[u] = true;

        // Update the distance value of the adjacent vertices of the current vertex
        for (int v = 0; v < V; v++)
            if (!visited[v] &&
                graph[u][v] &&
                dist[u] != INT_MAX &&
                dist[u] + graph[u][v] < dist[v])
            {
                dist[v] = dist[u] + graph[u][v];
            }
    }
    // Print the calculated distances
    printTravelTime(dist);
}

int main()
{
    // Adjacency matrix
    /*
     * HOW TO READ:
     * Example: { 0, 4, 0, 0, 0, 0, 0, 8, 0 }
     * Assuming that the origin vertice is labeled 0.
     * Adjacent vertices are vertices 1 and 7.
     * Distance to vertice 1 is 4, distance to vertice 7 is 8.
     */
    int graph[V][V] = { { 0, 4, 0, 0, 0, 0, 0, 8, 0 },      // Vertice 1
                        { 4, 0, 8, 0, 0, 0, 0, 11, 0 },     // Vertice 2
                        { 0, 8, 0, 7, 0, 4, 0, 0, 2 },      // Vertice 3
                        { 0, 0, 7, 0, 9, 14, 0, 0, 0 },     // Vertice 4
                        { 0, 0, 0, 9, 0, 10, 0, 0, 0 },     // Vertice 5
                        { 0, 0, 4, 14, 10, 0, 2, 0, 0 },    // Vertice 6
                        { 0, 0, 0, 0, 0, 2, 0, 1, 6 },      // Vertice 7
                        { 8, 11, 0, 0, 0, 0, 1, 0, 7 },     // Vertice 8
                        { 0, 0, 2, 0, 0, 0, 6, 7, 0 } };    // Vertice 9

    // Call the Dijkstra's algorithm function
    shortestPathFrom(graph, 3);

    return 0;
}