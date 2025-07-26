namespace OptiRoute.Core.Models;

/// <summary>
/// Represents a location that can be either an index reference or a coordinate.
/// </summary>
public class Location
{
    /// <summary>
    /// Gets the index of this location in the distance matrix.
    /// </summary>
    public int Index { get; private set; }
    
    /// <summary>
    /// Gets the coordinate of this location, if available.
    /// </summary>
    public Coordinate? Coordinate { get; }
    
    /// <summary>
    /// Indicates whether this location was created with a user-provided index.
    /// </summary>
    public bool HasUserIndex { get; }

    /// <summary>
    /// Creates a location with an index reference.
    /// </summary>
    public Location(int index)
    {
        Index = index;
        HasUserIndex = true;
    }

    /// <summary>
    /// Creates a location with a coordinate.
    /// </summary>
    public Location(Coordinate coordinate)
    {
        Coordinate = coordinate;
        HasUserIndex = false;
        Index = -1; // Will be set later when added to input
    }

    /// <summary>
    /// Creates a location with both index and coordinate.
    /// </summary>
    public Location(int index, Coordinate coordinate)
    {
        Index = index;
        Coordinate = coordinate;
        HasUserIndex = true;
    }

    /// <summary>
    /// Sets the index for locations created with coordinates only.
    /// </summary>
    internal void SetIndex(int index)
    {
        if (HasUserIndex)
            throw new InvalidOperationException("Cannot set index on a location with user-provided index.");
        Index = index;
    }

    public override bool Equals(object? obj)
    {
        if (obj is not Location other) return false;
        
        if (HasUserIndex && other.HasUserIndex)
            return Index == other.Index;
            
        if (Coordinate.HasValue && other.Coordinate.HasValue)
            return Coordinate.Value == other.Coordinate.Value;
            
        return false;
    }

    public override int GetHashCode()
    {
        if (HasUserIndex)
            return Index.GetHashCode();
            
        return Coordinate?.GetHashCode() ?? 0;
    }
}