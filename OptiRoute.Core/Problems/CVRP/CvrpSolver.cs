using OptiRoute.Core.Algorithms;
using OptiRoute.Core.Algorithms.Heuristics;
using OptiRoute.Core.Models;
using OptiRoute.Core.Problems.VRP;

namespace OptiRoute.Core.Problems.CVRP;

/// <summary>
/// Capacitated Vehicle Routing Problem solver (based on VROOM's CVRP).
/// </summary>
public class CvrpSolver : VrpSolver
{
    private static readonly List<HeuristicParameters> HomogeneousParameters = new()
    {
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.None, RegretCoeff = 0.9 },
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.HigherAmount, RegretCoeff = 0.9 },
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.Nearest, RegretCoeff = 0.9 },
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.Furthest, RegretCoeff = 0.7 },
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.None, RegretCoeff = 0 },
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.HigherAmount, RegretCoeff = 0.3 },
        new() { Heuristic = HeuristicType.Dynamic, Init = InitStrategy.None, RegretCoeff = 0.4 },
    };

    private static readonly List<HeuristicParameters> HeterogeneousParameters = new()
    {
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.None, RegretCoeff = 0.9 },
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.HigherAmount, RegretCoeff = 0.9 },
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.None, RegretCoeff = 0.3 },
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.HigherAmount, RegretCoeff = 0.2 },
        new() { Heuristic = HeuristicType.Basic, Init = InitStrategy.None, RegretCoeff = 0.1 },
        new() { Heuristic = HeuristicType.Dynamic, Init = InitStrategy.None, RegretCoeff = 0.6 },
        new() { Heuristic = HeuristicType.Dynamic, Init = InitStrategy.HigherAmount, RegretCoeff = 0.9 },
    };

    public CvrpSolver(Input input) : base(input)
    {
    }

    public override Solution Solve(
        int nbSearches = 1,
        int depth = 4,
        int nbThreads = 1,
        TimeSpan? timeout = null,
        List<HeuristicParameters>? heuristicParams = null)
    {
        return SolveInternal<CvrpRoute>(nbSearches, depth, nbThreads, timeout, heuristicParams);
    }

    protected override List<HeuristicParameters> GetDefaultParameters()
    {
        return _input.HasHomogeneousLocations ? HomogeneousParameters : HeterogeneousParameters;
    }

    protected override Solution FormatSolution<TRoute>(List<TRoute> routes)
    {
        var solutionRoutes = new List<Route>();
        var unassignedJobs = new List<UnassignedJob>();

        // Convert internal routes to solution format
        for (int v = 0; v < routes.Count; v++)
        {
            var rawRoute = routes[v];
            var jobSequence = rawRoute.GetJobSequence();
            
            if (jobSequence.Count == 0)
                continue; // Skip empty routes
                
            var vehicle = _input.Vehicles[v];
            var steps = new List<Step>();
            
            // Add start step if vehicle has start location
            if (vehicle.StartLocation != null)
            {
                steps.Add(new StartStep
                {
                    Location = vehicle.StartLocation
                });
            }
            
            // Add job steps
            foreach (var jobIndex in jobSequence)
            {
                var job = _input.Jobs[jobIndex];
                steps.Add(new JobStep
                {
                    JobId = job.Id,
                    Location = job.Location,
                    Description = job.Description,
                    Service = job.DefaultService
                });
            }
            
            // Add end step if vehicle has end location
            if (vehicle.EndLocation != null)
            {
                steps.Add(new EndStep
                {
                    Location = vehicle.EndLocation
                });
            }
            
            // Calculate route metrics
            long totalDistance = 0;
            long totalDuration = 0;
            long totalCost = vehicle.Costs.Fixed;
            
            // Calculate edge costs
            var prevLocation = vehicle.StartLocation;
            long totalService = 0;
            foreach (var step in steps)
            {
                if (step is JobStep jobStep)
                {
                    if (prevLocation != null)
                    {
                        var eval = _input.GetEval(v, prevLocation.Index, jobStep.Location.Index);
                        totalDistance += eval.Distance;
                        totalDuration += eval.Duration;
                        totalCost += eval.Cost;
                    }
                    totalDuration += jobStep.Service;
                    totalService += jobStep.Service;
                    prevLocation = jobStep.Location;
                }
                else if (step is EndStep endStep)
                {
                    if (prevLocation != null)
                    {
                        var eval = _input.GetEval(v, prevLocation.Index, endStep.Location.Index);
                        totalDistance += eval.Distance;
                        totalDuration += eval.Duration;
                        totalCost += eval.Cost;
                    }
                }
            }
            
            var route = new Route
            {
                VehicleId = vehicle.Id,
                Description = vehicle.Description,
                Steps = steps,
                Distance = totalDistance,
                Duration = totalDuration,
                Cost = totalCost,
                Service = totalService
            };
            
            solutionRoutes.Add(route);
        }
        
        // Add unassigned jobs
        var assignedJobs = new HashSet<int>();
        foreach (var route in routes)
        {
            foreach (var jobIndex in route.GetJobSequence())
            {
                assignedJobs.Add(jobIndex);
            }
        }
        
        for (int j = 0; j < _input.Jobs.Count; j++)
        {
            if (!assignedJobs.Contains(j))
            {
                var job = _input.Jobs[j];
                unassignedJobs.Add(new UnassignedJob
                {
                    JobId = job.Id,
                    Location = job.Location
                });
            }
        }
        
        // Create solution with summary
        return new Solution
        {
            Routes = solutionRoutes,
            Unassigned = unassignedJobs,
            Summary = new SolutionSummary
            {
                Cost = solutionRoutes.Sum(r => r.Cost),
                Distance = solutionRoutes.Sum(r => r.Distance),
                Duration = solutionRoutes.Sum(r => r.Duration),
                Routes = solutionRoutes.Count,
                Unassigned = unassignedJobs.Count,
                Service = solutionRoutes.Sum(r => r.Service)
            }
        };
    }
}

/// <summary>
/// CVRP-specific route implementation.
/// </summary>
internal class CvrpRoute : RawRoute
{
    public CvrpRoute() : base()
    {
    }
    
    public CvrpRoute(Input input, int vehicleIndex) : base(input, vehicleIndex)
    {
    }
}