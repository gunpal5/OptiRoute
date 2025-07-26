namespace OptiRoute.Core.Models;

/// <summary>
/// Represents a complete solution to a routing problem (inspired by VROOM's Solution).
/// </summary>
public class Solution
{
    /// <summary>
    /// Gets the routes for each vehicle.
    /// </summary>
    public List<Route> Routes { get; init; } = new();

    /// <summary>
    /// Gets the list of unassigned jobs.
    /// </summary>
    public List<UnassignedJob> Unassigned { get; init; } = new();

    /// <summary>
    /// Gets the list of unassigned job IDs (for simple VRP).
    /// </summary>
    public List<string> UnassignedJobs { get; init; } = new();

    /// <summary>
    /// Gets the summary statistics for the solution.
    /// </summary>
    public SolutionSummary Summary { get; init; } = new();

    /// <summary>
    /// Gets the total cost of the solution.
    /// </summary>
    public long Cost => Summary.Cost;

    /// <summary>
    /// Gets the computation time in milliseconds.
    /// </summary>
    public long ComputingTime { get; set; }
}

/// <summary>
/// Represents a vehicle route in the solution.
/// </summary>
public class Route
{
    /// <summary>
    /// Gets the vehicle ID.
    /// </summary>
    public string VehicleId { get; init; } = string.Empty;

    /// <summary>
    /// Gets the steps in the route.
    /// </summary>
    public List<Step> Steps { get; init; } = new();

    /// <summary>
    /// Gets the job IDs in the route (for simple VRP without detailed steps).
    /// </summary>
    public List<string> Jobs { get; init; } = new();

    /// <summary>
    /// Gets the total cost for this route.
    /// </summary>
    public long Cost { get; init; }

    /// <summary>
    /// Gets the total service duration.
    /// </summary>
    public long Service { get; init; }

    /// <summary>
    /// Gets the total duration including travel and service.
    /// </summary>
    public long Duration { get; init; }

    /// <summary>
    /// Gets the total waiting time.
    /// </summary>
    public long WaitingTime { get; init; }

    /// <summary>
    /// Gets the total distance.
    /// </summary>
    public long Distance { get; init; }

    /// <summary>
    /// Gets the vehicle description.
    /// </summary>
    public string? Description { get; init; }
}

/// <summary>
/// Represents a step in a route.
/// </summary>
public abstract class Step
{
    /// <summary>
    /// Gets the step type.
    /// </summary>
    public abstract StepType Type { get; }

    /// <summary>
    /// Gets the location for this step.
    /// </summary>
    public Location Location { get; init; } = null!;

    /// <summary>
    /// Gets the arrival time.
    /// </summary>
    public long Arrival { get; init; }

    /// <summary>
    /// Gets the duration at this step.
    /// </summary>
    public long Duration { get; init; }

    /// <summary>
    /// Gets the distance from previous step.
    /// </summary>
    public long Distance { get; init; }
}

/// <summary>
/// Represents a job step in a route.
/// </summary>
public class JobStep : Step
{
    public override StepType Type => StepType.Job;

    /// <summary>
    /// Gets the job ID.
    /// </summary>
    public string JobId { get; init; } = string.Empty;

    /// <summary>
    /// Gets the job description.
    /// </summary>
    public string? Description { get; init; }

    /// <summary>
    /// Gets the service duration.
    /// </summary>
    public long Service { get; init; }

    /// <summary>
    /// Gets the waiting time before service.
    /// </summary>
    public long WaitingTime { get; init; }
}

/// <summary>
/// Represents a start step in a route.
/// </summary>
public class StartStep : Step
{
    public override StepType Type => StepType.Start;
}

/// <summary>
/// Represents an end step in a route.
/// </summary>
public class EndStep : Step
{
    public override StepType Type => StepType.End;
}

/// <summary>
/// Represents a break step in a route.
/// </summary>
public class BreakStep : Step
{
    public override StepType Type => StepType.Break;

    /// <summary>
    /// Gets the break ID.
    /// </summary>
    public string BreakId { get; init; } = string.Empty;

    /// <summary>
    /// Gets the break description.
    /// </summary>
    public string? Description { get; init; }
}

/// <summary>
/// Type of route step.
/// </summary>
public enum StepType
{
    Start,
    Job,
    Break,
    End
}

/// <summary>
/// Represents an unassigned job.
/// </summary>
public class UnassignedJob
{
    /// <summary>
    /// Gets the job ID.
    /// </summary>
    public string JobId { get; init; } = string.Empty;

    /// <summary>
    /// Gets the reason codes for why the job wasn't assigned.
    /// </summary>
    public List<int> ReasonCodes { get; init; } = new();

    /// <summary>
    /// Gets the location of the unassigned job.
    /// </summary>
    public Location Location { get; init; } = null!;
}

/// <summary>
/// Summary statistics for a solution.
/// </summary>
public class SolutionSummary
{
    /// <summary>
    /// Gets the total cost.
    /// </summary>
    public long Cost { get; init; }

    /// <summary>
    /// Gets the number of routes.
    /// </summary>
    public int Routes { get; init; }

    /// <summary>
    /// Gets the number of unassigned jobs.
    /// </summary>
    public int Unassigned { get; init; }

    /// <summary>
    /// Gets the total setup duration.
    /// </summary>
    public long Setup { get; init; }

    /// <summary>
    /// Gets the total service duration.
    /// </summary>
    public long Service { get; init; }

    /// <summary>
    /// Gets the total duration.
    /// </summary>
    public long Duration { get; init; }

    /// <summary>
    /// Gets the total waiting time.
    /// </summary>
    public long WaitingTime { get; init; }

    /// <summary>
    /// Gets the total priority of served jobs.
    /// </summary>
    public int Priority { get; init; }

    /// <summary>
    /// Gets the total distance.
    /// </summary>
    public long Distance { get; init; }

    /// <summary>
    /// Gets amount statistics.
    /// </summary>
    public List<long> Amount { get; init; } = new();
}