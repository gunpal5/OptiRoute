using OptiRoute.Core.Models;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.TSP;

/// <summary>
/// Implements the Christofides heuristic for TSP (inspired by VROOM).
/// </summary>
internal static class Christofides
{
    /// <summary>
    /// Computes an approximate TSP solution using Christofides heuristic.
    /// </summary>
    public static List<int> Solve(double[,] symmetricMatrix)
    {
        var n = symmetricMatrix.GetLength(0);
        if (n < 3)
        {
            return Enumerable.Range(0, n).ToList();
        }

        // Build minimum spanning tree
        var mst = ComputeMinimumSpanningTree(symmetricMatrix);
        
        // Find odd degree vertices
        var oddVertices = FindOddDegreeVertices(mst, n);
        
        // Compute minimum weight perfect matching on odd vertices
        var matching = ComputeMinimumWeightPerfectMatching(symmetricMatrix, oddVertices);
        
        // Build Eulerian graph by combining MST and matching
        var eulerianGraph = BuildEulerianGraph(mst, matching, oddVertices);
        
        // Find Eulerian tour
        var eulerianTour = FindEulerianTour(eulerianGraph, n);
        
        // Convert to Hamiltonian tour by removing duplicate vertices
        return ConvertToHamiltonianTour(eulerianTour);
    }

    private static List<(int from, int to, double weight)> ComputeMinimumSpanningTree(double[,] matrix)
    {
        var n = matrix.GetLength(0);
        var edges = new List<(int from, int to, double weight)>();
        
        // Create all edges
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                if (!double.IsInfinity(matrix[i, j]))
                {
                    edges.Add((i, j, matrix[i, j]));
                }
            }
        }
        
        // Sort edges by weight
        edges.Sort((a, b) => a.weight.CompareTo(b.weight));
        
        // Kruskal's algorithm
        var parent = Enumerable.Range(0, n).ToArray();
        var rank = new int[n];
        var mst = new List<(int from, int to, double weight)>();
        
        int Find(int x)
        {
            if (parent[x] != x)
                parent[x] = Find(parent[x]);
            return parent[x];
        }
        
        void Union(int x, int y)
        {
            int px = Find(x);
            int py = Find(y);
            
            if (rank[px] < rank[py])
                parent[px] = py;
            else if (rank[px] > rank[py])
                parent[py] = px;
            else
            {
                parent[py] = px;
                rank[px]++;
            }
        }
        
        foreach (var edge in edges)
        {
            if (Find(edge.from) != Find(edge.to))
            {
                mst.Add(edge);
                Union(edge.from, edge.to);
                if (mst.Count == n - 1) break;
            }
        }
        
        return mst;
    }

    private static List<int> FindOddDegreeVertices(List<(int from, int to, double weight)> mst, int n)
    {
        var degree = new int[n];
        foreach (var edge in mst)
        {
            degree[edge.from]++;
            degree[edge.to]++;
        }
        
        var oddVertices = new List<int>();
        for (int i = 0; i < n; i++)
        {
            if (degree[i] % 2 == 1)
                oddVertices.Add(i);
        }
        
        return oddVertices;
    }

    private static List<(int from, int to)> ComputeMinimumWeightPerfectMatching(
        double[,] matrix, List<int> oddVertices)
    {
        // Create submatrix for odd vertices
        var n = oddVertices.Count;
        var subMatrix = new double[n, n];
        
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                subMatrix[i, j] = matrix[oddVertices[i], oddVertices[j]];
            }
        }
        
        // Compute minimum weight perfect matching using Hungarian algorithm
        var mwpm = Munkres.MinimumWeightPerfectMatching(subMatrix);
        
        // Process matching to ensure symmetry (as in VROOM)
        var mwpmFinal = new Dictionary<int, int>();
        var wrongVertices = new List<int>();
        
        foreach (var kvp in mwpm)
        {
            if (mwpm.ContainsKey(kvp.Value) && mwpm[kvp.Value] == kvp.Key)
            {
                // Symmetric pair found
                int minIdx = Math.Min(kvp.Key, kvp.Value);
                int maxIdx = Math.Max(kvp.Key, kvp.Value);
                if (!mwpmFinal.ContainsKey(minIdx))
                {
                    mwpmFinal[minIdx] = maxIdx;
                }
            }
            else
            {
                wrongVertices.Add(kvp.Key);
            }
        }
        
        // Handle non-symmetric matches with greedy approach
        if (wrongVertices.Count > 0)
        {
            var wrongSubMatrix = new double[wrongVertices.Count, wrongVertices.Count];
            for (int i = 0; i < wrongVertices.Count; i++)
            {
                for (int j = 0; j < wrongVertices.Count; j++)
                {
                    wrongSubMatrix[i, j] = subMatrix[wrongVertices[i], wrongVertices[j]];
                }
            }
            
            var greedyMatching = Munkres.GreedySymmetricApproxMWPM(wrongSubMatrix);
            foreach (var kvp in greedyMatching)
            {
                int origI = wrongVertices[kvp.Key];
                int origJ = wrongVertices[kvp.Value];
                mwpmFinal[Math.Min(origI, origJ)] = Math.Max(origI, origJ);
            }
        }
        
        // Convert to list of edges with original indices
        var matching = new List<(int from, int to)>();
        foreach (var kvp in mwpmFinal)
        {
            matching.Add((oddVertices[kvp.Key], oddVertices[kvp.Value]));
        }
        
        return matching;
    }

    private static List<List<int>> BuildEulerianGraph(
        List<(int from, int to, double weight)> mst,
        List<(int from, int to)> matching,
        List<int> oddVertices)
    {
        var n = mst.Max(e => Math.Max(e.from, e.to)) + 1;
        var graph = new List<List<int>>(n);
        for (int i = 0; i < n; i++)
            graph.Add(new List<int>());
        
        // Add MST edges
        foreach (var edge in mst)
        {
            graph[edge.from].Add(edge.to);
            graph[edge.to].Add(edge.from);
        }
        
        // Add matching edges
        foreach (var edge in matching)
        {
            graph[edge.from].Add(edge.to);
            graph[edge.to].Add(edge.from);
        }
        
        return graph;
    }

    private static List<int> FindEulerianTour(List<List<int>> graph, int n)
    {
        var tour = new List<int>();
        var stack = new Stack<int>();
        var current = 0;
        
        // Make a copy of the graph to modify during traversal
        var tempGraph = new List<List<int>>(n);
        for (int i = 0; i < n; i++)
            tempGraph.Add(new List<int>(graph[i]));
        
        stack.Push(current);
        
        while (stack.Count > 0)
        {
            if (tempGraph[current].Count > 0)
            {
                stack.Push(current);
                int next = tempGraph[current][0];
                tempGraph[current].RemoveAt(0);
                tempGraph[next].Remove(current);
                current = next;
            }
            else
            {
                tour.Add(current);
                current = stack.Pop();
            }
        }
        
        tour.Reverse();
        return tour;
    }

    private static List<int> ConvertToHamiltonianTour(List<int> eulerianTour)
    {
        var visited = new HashSet<int>();
        var hamiltonianTour = new List<int>();
        
        foreach (var vertex in eulerianTour)
        {
            if (!visited.Contains(vertex))
            {
                visited.Add(vertex);
                hamiltonianTour.Add(vertex);
            }
        }
        
        return hamiltonianTour;
    }
}