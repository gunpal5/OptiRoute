namespace OptiRoute.Core.Models;

/// <summary>
/// Represents a job (task) to be performed at a specific location.
/// </summary>
public class Job
{
    /// <summary>
    /// Gets the unique identifier for this job.
    /// </summary>
    public string Id { get; }
    
    /// <summary>
    /// Gets the location where this job should be performed.
    /// </summary>
    public Location Location { get; }
    
    /// <summary>
    /// Gets the default service duration in seconds.
    /// </summary>
    public long DefaultService { get; init; }
    
    /// <summary>
    /// Gets the default setup duration in seconds.
    /// </summary>
    public long DefaultSetup { get; init; }
    
    /// <summary>
    /// Gets the service duration per vehicle type.
    /// </summary>
    public IReadOnlyDictionary<string, long> ServicePerType { get; init; } = 
        new Dictionary<string, long>();
    
    /// <summary>
    /// Gets the setup duration per vehicle type.
    /// </summary>
    public IReadOnlyDictionary<string, long> SetupPerType { get; init; } = 
        new Dictionary<string, long>();
    
    /// <summary>
    /// Computed service durations for each vehicle (by index).
    /// </summary>
    internal long[] Services { get; set; } = Array.Empty<long>();
    
    /// <summary>
    /// Computed setup durations for each vehicle (by index).
    /// </summary>
    internal long[] Setups { get; set; } = Array.Empty<long>();
    
    // Compatibility properties for existing code
    public long Service => DefaultService;
    public long Setup => DefaultSetup;
    
    /// <summary>
    /// Gets the job type (Single, Pickup, or Delivery).
    /// </summary>
    public JobType Type { get; init; } = JobType.Single;
    
    /// <summary>
    /// Gets the index for this job in the input.
    /// </summary>
    public int Index { get; internal set; }
    
    /// <summary>
    /// Gets the delivery amounts for each dimension.
    /// </summary>
    public Amount Delivery { get; init; } = new Amount(0);
    
    /// <summary>
    /// Gets the pickup amounts for each dimension.
    /// </summary>
    public Amount Pickup { get; init; } = new Amount(0);
    
    /// <summary>
    /// Gets the time windows during which this job can be performed.
    /// </summary>
    public IReadOnlyList<TimeWindow> TimeWindows { get; init; } = new[] { TimeWindow.Default };
    
    /// <summary>
    /// Gets the priority of this job (0-100, higher is more important).
    /// </summary>
    public int Priority { get; init; } = 0;
    
    /// <summary>
    /// Gets the required skills to perform this job.
    /// </summary>
    public IReadOnlySet<int> Skills { get; init; } = new HashSet<int>();
    
    /// <summary>
    /// Gets the description of this job.
    /// </summary>
    public string? Description { get; init; }

    public Job(string id, Location location)
    {
        Id = id ?? throw new ArgumentNullException(nameof(id));
        Location = location ?? throw new ArgumentNullException(nameof(location));
    }

    /// <summary>
    /// Checks if the job can be started at the given time.
    /// </summary>
    public bool CanStartAt(long time) => TimeWindows.Any(tw => tw.Contains(time));

    /// <summary>
    /// Gets the earliest possible start time for this job.
    /// </summary>
    public long EarliestStart => TimeWindows.Min(tw => tw.Start);

    /// <summary>
    /// Gets the latest possible start time for this job.
    /// </summary>
    public long LatestStart => TimeWindows.Max(tw => tw.End);
}