namespace OptiRoute.Core.Utils;

/// <summary>
/// Represents an undirected graph edge.
/// </summary>
public struct Edge<T> where T : IComparable<T>
{
    public int From { get; }
    public int To { get; }
    public T Weight { get; }

    public Edge(int from, int to, T weight)
    {
        From = from;
        To = to;
        Weight = weight;
    }
}

/// <summary>
/// Represents an undirected graph.
/// </summary>
public class UndirectedGraph<T> where T : IComparable<T>
{
    private readonly List<Edge<T>> _edges;
    private readonly int _nodeCount;

    public UndirectedGraph(int nodeCount)
    {
        _nodeCount = nodeCount;
        _edges = new List<Edge<T>>();
    }

    public UndirectedGraph(T[,] matrix)
    {
        _nodeCount = matrix.GetLength(0);
        _edges = new List<Edge<T>>();

        for (int i = 0; i < _nodeCount; i++)
        {
            for (int j = i + 1; j < _nodeCount; j++)
            {
                _edges.Add(new Edge<T>(i, j, matrix[i, j]));
            }
        }
    }

    public void AddEdge(int from, int to, T weight)
    {
        _edges.Add(new Edge<T>(from, to, weight));
    }

    public List<Edge<T>> GetEdges() => new List<Edge<T>>(_edges);

    public int NodeCount => _nodeCount;

    public Dictionary<int, List<int>> GetAdjacencyList()
    {
        var adjacencyList = new Dictionary<int, List<int>>();
        
        for (int i = 0; i < _nodeCount; i++)
        {
            adjacencyList[i] = new List<int>();
        }

        foreach (var edge in _edges)
        {
            adjacencyList[edge.From].Add(edge.To);
            adjacencyList[edge.To].Add(edge.From);
        }

        return adjacencyList;
    }
}