using OptiRoute.Core.Models;

namespace OptiRoute.Core.Problems.VRP;

/// <summary>
/// Indicators for solution quality (inspired by VROOM's SolutionIndicators).
/// </summary>
public struct SolutionIndicators : IComparable<SolutionIndicators>, IEquatable<SolutionIndicators>
{
    public int AssignedJobs { get; init; }
    public long Cost { get; init; }
    public int UsedVehicles { get; init; }
    public long TotalSetup { get; init; }
    public long TotalService { get; init; }
    public long TotalWaiting { get; init; }
    public int TotalPriority { get; init; }

    public SolutionIndicators()
    {
        AssignedJobs = 0;
        Cost = 0;
        UsedVehicles = 0;
        TotalSetup = 0;
        TotalService = 0;
        TotalWaiting = 0;
        TotalPriority = 0;
    }

    public SolutionIndicators(Input input, IEnumerable<IRoute> routes)
    {
        AssignedJobs = 0;
        Cost = 0;
        UsedVehicles = 0;
        TotalSetup = 0;
        TotalService = 0;
        TotalWaiting = 0;
        TotalPriority = 0;

        foreach (var route in routes)
        {
            var jobs = route.GetJobSequence();
            if (jobs.Count > 0)
            {
                UsedVehicles++;
                AssignedJobs += jobs.Count;
                
                // Calculate costs and times
                foreach (var jobIndex in jobs)
                {
                    var job = input.Jobs[jobIndex];
                    TotalSetup += job.Setup;
                    TotalService += job.Service;
                    TotalPriority += job.Priority;
                }
            }
        }
    }

    public int CompareTo(SolutionIndicators other)
    {
        // First compare by assigned jobs (more is better)
        var jobsComp = other.AssignedJobs.CompareTo(AssignedJobs);
        if (jobsComp != 0) return jobsComp;

        // Then by cost (less is better)
        var costComp = Cost.CompareTo(other.Cost);
        if (costComp != 0) return costComp;

        // Then by used vehicles (less is better)
        var vehiclesComp = UsedVehicles.CompareTo(other.UsedVehicles);
        if (vehiclesComp != 0) return vehiclesComp;

        // Then by priority sum (more is better)
        return other.TotalPriority.CompareTo(TotalPriority);
    }

    public bool Equals(SolutionIndicators other)
    {
        return AssignedJobs == other.AssignedJobs &&
               Cost == other.Cost &&
               UsedVehicles == other.UsedVehicles &&
               TotalSetup == other.TotalSetup &&
               TotalService == other.TotalService &&
               TotalWaiting == other.TotalWaiting &&
               TotalPriority == other.TotalPriority;
    }

    public override bool Equals(object? obj)
    {
        return obj is SolutionIndicators other && Equals(other);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(AssignedJobs, Cost, UsedVehicles, TotalPriority);
    }

    public static bool operator ==(SolutionIndicators left, SolutionIndicators right) => left.Equals(right);
    public static bool operator !=(SolutionIndicators left, SolutionIndicators right) => !left.Equals(right);
    public static bool operator <(SolutionIndicators left, SolutionIndicators right) => left.CompareTo(right) < 0;
    public static bool operator >(SolutionIndicators left, SolutionIndicators right) => left.CompareTo(right) > 0;
    public static bool operator <=(SolutionIndicators left, SolutionIndicators right) => left.CompareTo(right) <= 0;
    public static bool operator >=(SolutionIndicators left, SolutionIndicators right) => left.CompareTo(right) >= 0;
}