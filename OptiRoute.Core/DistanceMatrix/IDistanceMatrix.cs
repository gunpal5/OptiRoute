namespace OptiRoute.Core.DistanceMatrix;

/// <summary>
/// Represents a distance/duration matrix between locations.
/// </summary>
public interface IDistanceMatrix
{
    /// <summary>
    /// Gets the number of locations in the matrix.
    /// </summary>
    int Size { get; }
    
    /// <summary>
    /// Gets the distance between two locations in meters.
    /// </summary>
    double GetDistance(int from, int to);
    
    /// <summary>
    /// Gets the travel duration between two locations in seconds.
    /// </summary>
    long GetDuration(int from, int to);
    
    /// <summary>
    /// Gets the cost of traveling between two locations.
    /// </summary>
    decimal GetCost(int from, int to);
}

/// <summary>
/// A simple in-memory implementation of IDistanceMatrix.
/// </summary>
public class MemoryDistanceMatrix : IDistanceMatrix
{
    private readonly double[,] _distances;
    private readonly long[,] _durations;
    private readonly decimal[,]? _costs;

    public int Size { get; }

    public MemoryDistanceMatrix(double[,] distances, long[,] durations, decimal[,]? costs = null)
    {
        if (distances.GetLength(0) != distances.GetLength(1))
            throw new ArgumentException("Distance matrix must be square.");
            
        if (durations.GetLength(0) != durations.GetLength(1))
            throw new ArgumentException("Duration matrix must be square.");
            
        if (distances.GetLength(0) != durations.GetLength(0))
            throw new ArgumentException("Distance and duration matrices must have the same size.");
            
        if (costs != null && (costs.GetLength(0) != costs.GetLength(1) || costs.GetLength(0) != distances.GetLength(0)))
            throw new ArgumentException("Cost matrix must be square and match the size of distance matrix.");

        _distances = distances;
        _durations = durations;
        _costs = costs;
        Size = distances.GetLength(0);
    }

    public double GetDistance(int from, int to)
    {
        ValidateIndices(from, to);
        return _distances[from, to];
    }

    public long GetDuration(int from, int to)
    {
        ValidateIndices(from, to);
        return _durations[from, to];
    }

    public decimal GetCost(int from, int to)
    {
        ValidateIndices(from, to);
        
        if (_costs != null)
            return _costs[from, to];
            
        // Default cost calculation based on duration and distance
        var hours = _durations[from, to] / 3600.0m;
        var km = (decimal)_distances[from, to] / 1000m;
        
        return hours * 60m + km * 0.5m; // Default: $60/hour + $0.50/km
    }

    private void ValidateIndices(int from, int to)
    {
        if (from < 0 || from >= Size)
            throw new ArgumentOutOfRangeException(nameof(from));
            
        if (to < 0 || to >= Size)
            throw new ArgumentOutOfRangeException(nameof(to));
    }
}