namespace OptiRoute.Core.Models;

/// <summary>
/// Represents a predefined step in a vehicle's route (based on VROOM's vehicle_step).
/// </summary>
public class VehicleStep
{
    /// <summary>
    /// Gets the step type.
    /// </summary>
    public VehicleStepType Type { get; init; }
    
    /// <summary>
    /// Gets the job ID for job steps.
    /// </summary>
    public string? JobId { get; init; }
    
    /// <summary>
    /// Gets the break ID for break steps.
    /// </summary>
    public string? BreakId { get; init; }
    
    /// <summary>
    /// Gets the location for this step.
    /// </summary>
    public Location? Location { get; init; }
    
    /// <summary>
    /// Gets the service duration at this step.
    /// </summary>
    public long Service { get; init; }
    
    /// <summary>
    /// Gets the forced service time for this step.
    /// </summary>
    public long? ForcedService { get; init; }
}

/// <summary>
/// Type of vehicle step.
/// </summary>
public enum VehicleStepType
{
    Start,
    End,
    Break,
    Job
}