namespace OptiRoute.Core.Models;

/// <summary>
/// Represents a time window constraint with start and end times.
/// </summary>
public readonly record struct TimeWindow
{
    /// <summary>
    /// Gets the start time of the window in seconds.
    /// </summary>
    public long Start { get; }
    
    /// <summary>
    /// Gets the end time of the window in seconds.
    /// </summary>
    public long End { get; }
    
    /// <summary>
    /// Gets the duration of the window in seconds.
    /// </summary>
    public long Duration => End - Start;

    /// <summary>
    /// Creates a time window with specified start and end times.
    /// </summary>
    public TimeWindow(long start, long end)
    {
        if (start > end)
            throw new ArgumentException($"Invalid time window: start ({start}) must be <= end ({end})");
            
        Start = start;
        End = end;
    }

    /// <summary>
    /// Creates a default time window covering the entire day (0 to 86400 seconds).
    /// </summary>
    public static TimeWindow Default => new(0, 86400);

    /// <summary>
    /// Checks if a given time falls within this window.
    /// </summary>
    public bool Contains(long time) => time >= Start && time <= End;

    /// <summary>
    /// Checks if this window overlaps with another window.
    /// </summary>
    public bool Overlaps(TimeWindow other) => Start < other.End && End > other.Start;

    /// <summary>
    /// Gets the intersection of this window with another window.
    /// </summary>
    public TimeWindow? IntersectWith(TimeWindow other)
    {
        var start = Math.Max(Start, other.Start);
        var end = Math.Min(End, other.End);
        
        return start <= end ? new TimeWindow(start, end) : null;
    }
}