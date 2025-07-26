using OptiRoute.Core.Models;

namespace OptiRoute.Core.Algorithms.Interfaces;

/// <summary>
/// Base interface for all routing problem solvers.
/// </summary>
public interface IRoutingSolver
{
    /// <summary>
    /// Solves the routing problem and returns the solution.
    /// </summary>
    /// <param name="cancellationToken">Token to cancel the operation.</param>
    /// <returns>The routing solution.</returns>
    Task<RoutingSolution> SolveAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// Represents a solution to a routing problem.
/// </summary>
public class RoutingSolution
{
    /// <summary>
    /// Gets the routes for each vehicle.
    /// </summary>
    public IReadOnlyList<Route> Routes { get; }
    
    /// <summary>
    /// Gets the unassigned jobs.
    /// </summary>
    public IReadOnlyList<Job> UnassignedJobs { get; }
    
    /// <summary>
    /// Gets the summary of the solution.
    /// </summary>
    public SolutionSummary Summary { get; }
    
    /// <summary>
    /// Gets the computation time in milliseconds.
    /// </summary>
    public long ComputationTime { get; }

    public RoutingSolution(
        IReadOnlyList<Route> routes, 
        IReadOnlyList<Job> unassignedJobs,
        long computationTime)
    {
        Routes = routes ?? throw new ArgumentNullException(nameof(routes));
        UnassignedJobs = unassignedJobs ?? throw new ArgumentNullException(nameof(unassignedJobs));
        ComputationTime = computationTime;
        Summary = CalculateSummary();
    }

    private SolutionSummary CalculateSummary()
    {
        var totalCost = Routes.Sum(r => r.TotalCost);
        var totalDistance = Routes.Sum(r => r.TotalDistance);
        var totalDuration = Routes.Sum(r => r.TotalDuration);
        var assignedJobs = Routes.Sum(r => r.Jobs.Count);
        
        return new SolutionSummary
        {
            TotalCost = totalCost,
            TotalDistance = totalDistance,
            TotalDuration = totalDuration,
            NumRoutes = Routes.Count,
            NumAssignedJobs = assignedJobs,
            NumUnassignedJobs = UnassignedJobs.Count
        };
    }
}

/// <summary>
/// Represents a route for a vehicle.
/// </summary>
public class Route
{
    /// <summary>
    /// Gets the vehicle assigned to this route.
    /// </summary>
    public Vehicle Vehicle { get; }
    
    /// <summary>
    /// Gets the jobs in the order they should be visited.
    /// </summary>
    public IReadOnlyList<Job> Jobs { get; }
    
    /// <summary>
    /// Gets the arrival times at each job.
    /// </summary>
    public IReadOnlyList<long> ArrivalTimes { get; }
    
    /// <summary>
    /// Gets the total cost of this route.
    /// </summary>
    public decimal TotalCost { get; }
    
    /// <summary>
    /// Gets the total distance of this route in meters.
    /// </summary>
    public double TotalDistance { get; }
    
    /// <summary>
    /// Gets the total duration of this route in seconds.
    /// </summary>
    public long TotalDuration { get; }
    
    /// <summary>
    /// Gets any violations in this route.
    /// </summary>
    public IReadOnlyList<RouteViolation> Violations { get; }

    public Route(
        Vehicle vehicle,
        IReadOnlyList<Job> jobs,
        IReadOnlyList<long> arrivalTimes,
        decimal totalCost,
        double totalDistance,
        long totalDuration,
        IReadOnlyList<RouteViolation>? violations = null)
    {
        Vehicle = vehicle ?? throw new ArgumentNullException(nameof(vehicle));
        Jobs = jobs ?? throw new ArgumentNullException(nameof(jobs));
        ArrivalTimes = arrivalTimes ?? throw new ArgumentNullException(nameof(arrivalTimes));
        
        if (jobs.Count != arrivalTimes.Count)
            throw new ArgumentException("Jobs and arrival times must have the same count.");
            
        TotalCost = totalCost;
        TotalDistance = totalDistance;
        TotalDuration = totalDuration;
        Violations = violations ?? Array.Empty<RouteViolation>();
    }
}

/// <summary>
/// Represents a violation in a route.
/// </summary>
public record RouteViolation(string Type, string Description, double Amount);

/// <summary>
/// Summary statistics for a routing solution.
/// </summary>
public record SolutionSummary
{
    public decimal TotalCost { get; init; }
    public double TotalDistance { get; init; }
    public long TotalDuration { get; init; }
    public int NumRoutes { get; init; }
    public int NumAssignedJobs { get; init; }
    public int NumUnassignedJobs { get; init; }
}