namespace OptiRoute.Core.Models;

/// <summary>
/// Represents a vehicle that can perform jobs.
/// </summary>
public class Vehicle
{
    /// <summary>
    /// Gets the unique identifier for this vehicle.
    /// </summary>
    public string Id { get; }
    
    /// <summary>
    /// Gets the starting location of the vehicle.
    /// </summary>
    public Location? StartLocation { get; init; }
    
    /// <summary>
    /// Gets the ending location of the vehicle.
    /// </summary>
    public Location? EndLocation { get; init; }
    
    /// <summary>
    /// Gets the capacity for each dimension.
    /// </summary>
    public Amount Capacity { get; init; } = new Amount(0);
    
    /// <summary>
    /// Gets the time window during which the vehicle is available.
    /// </summary>
    public TimeWindow TimeWindow { get; init; } = TimeWindow.Default;
    
    /// <summary>
    /// Gets the skills that this vehicle possesses.
    /// </summary>
    public IReadOnlySet<int> Skills { get; init; } = new HashSet<int>();
    
    /// <summary>
    /// Gets the maximum number of jobs this vehicle can handle.
    /// </summary>
    public int MaxTasks { get; init; } = int.MaxValue;
    
    /// <summary>
    /// Gets the maximum travel time in seconds.
    /// </summary>
    public long? MaxTravelTime { get; init; }
    
    /// <summary>
    /// Gets the maximum distance in meters.
    /// </summary>
    public double? MaxDistance { get; init; }
    
    /// <summary>
    /// Gets the speed factor (multiplier for travel times).
    /// </summary>
    public double SpeedFactor { get; init; } = 1.0;
    
    /// <summary>
    /// Gets the cost structure for this vehicle.
    /// </summary>
    public VehicleCosts Costs { get; init; } = new();
    
    /// <summary>
    /// Gets the breaks for this vehicle.
    /// </summary>
    public IReadOnlyList<Break> Breaks { get; init; } = Array.Empty<Break>();
    
    /// <summary>
    /// Gets the description of this vehicle.
    /// </summary>
    public string? Description { get; init; }
    
    /// <summary>
    /// Gets the routing profile for this vehicle (e.g., "car", "truck").
    /// </summary>
    public string Profile { get; init; } = "car";
    
    /// <summary>
    /// Gets the vehicle type string (for type-specific durations).
    /// </summary>
    public string TypeStr { get; init; } = "";
    
    /// <summary>
    /// Gets or sets the vehicle type index (set by Input).
    /// </summary>
    internal int Type { get; set; } = 0;
    
    /// <summary>
    /// Gets predefined steps for this vehicle.
    /// </summary>
    public IReadOnlyList<VehicleStep> Steps { get; init; } = Array.Empty<VehicleStep>();
    
    /// <summary>
    /// Break ID to rank mapping for fast lookup.
    /// </summary>
    internal Dictionary<string, int> BreakIdToRank { get; set; } = new();
    
    /// <summary>
    /// Whether any break has max load constraint.
    /// </summary>
    internal bool HasBreakMaxLoad { get; set; }

    public Vehicle(string id)
    {
        Id = id ?? throw new ArgumentNullException(nameof(id));
    }
    
    /// <summary>
    /// Validates that the vehicle has at least a start or end location.
    /// </summary>
    public void Validate()
    {
        if (StartLocation == null && EndLocation == null)
            throw new ArgumentException("Vehicle must have at least a start or end location.");
    }

    /// <summary>
    /// Checks if this vehicle can handle a job based on skills.
    /// </summary>
    public bool CanHandle(Job job)
    {
        return job.Skills.All(skill => Skills.Contains(skill));
    }

    /// <summary>
    /// Calculates the available working duration for this vehicle.
    /// </summary>
    public long AvailableDuration => TimeWindow.Duration;

    /// <summary>
    /// Checks if the vehicle is ok with the given evaluation bounds.
    /// </summary>
    public bool IsOkForRangeBounds(Eval eval)
    {
        if (MaxTravelTime.HasValue && eval.Duration > MaxTravelTime.Value)
            return false;
        if (MaxDistance.HasValue && eval.Distance > MaxDistance.Value)
            return false;
        return true;
    }
    
}

/// <summary>
/// Represents the cost structure for a vehicle.
/// </summary>
public record VehicleCosts
{
    /// <summary>
    /// Fixed cost for using this vehicle.
    /// </summary>
    public long Fixed { get; init; } = 0;
    
    /// <summary>
    /// Cost per hour of operation.
    /// </summary>
    public long PerHour { get; init; } = 0;
    
    /// <summary>
    /// Cost per kilometer traveled.
    /// </summary>
    public long PerKm { get; init; } = 0;
}

/// <summary>
/// Represents a break period for a vehicle.
/// </summary>
public record Break
{
    /// <summary>
    /// Gets the unique identifier for this break.
    /// </summary>
    public string Id { get; init; } = Guid.NewGuid().ToString();
    
    /// <summary>
    /// Gets the time windows during which the break can be taken.
    /// </summary>
    public IReadOnlyList<TimeWindow> TimeWindows { get; init; } = new[] { TimeWindow.Default };
    
    /// <summary>
    /// Gets the duration of the break in seconds.
    /// </summary>
    public long Duration { get; init; }
    
    /// <summary>
    /// Gets the maximum load allowed during the break.
    /// </summary>
    public Amount? MaxLoad { get; init; }
    
    /// <summary>
    /// Gets the description of this break.
    /// </summary>
    public string? Description { get; init; }
}