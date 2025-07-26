using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.Heuristics;

namespace OptiRoute.Core.Algorithms.VRP;

/// <summary>
/// Capacitated Vehicle Routing Problem solver (based on VROOM's CVRP).
/// </summary>
public class CvrpSolver : VrpSolver
{
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
        return SolveInternal<RawRoute>(nbSearches, depth, nbThreads, timeout, heuristicParams);
    }

    protected override List<HeuristicParameters> GetDefaultParameters()
    {
        // Default parameters based on VROOM's approach
        var parameters = new List<HeuristicParameters>();

        // Basic heuristics with different initialization and sorting strategies
        var initStrategies = new[] { InitStrategy.None, InitStrategy.HigherAmount, InitStrategy.Furthest };
        var sortStrategies = new[] { SortStrategy.Availability, SortStrategy.Cost };
        var regretCoeffs = new[] { 0.0, 0.1, 0.3, 0.5, 0.9, 1.5, 2.0, 2.5, 3.0 };

        foreach (var init in initStrategies)
        {
            foreach (var sort in sortStrategies)
            {
                foreach (var regret in regretCoeffs)
                {
                    parameters.Add(new HeuristicParameters
                    {
                        Heuristic = HeuristicType.Basic,
                        Init = init,
                        Sort = sort,
                        RegretCoeff = regret
                    });
                }
            }
        }

        // Add some dynamic heuristics
        foreach (var regret in new[] { 0.0, 0.5, 1.0, 2.0 })
        {
            parameters.Add(new HeuristicParameters
            {
                Heuristic = HeuristicType.Dynamic,
                Init = InitStrategy.None,
                Sort = SortStrategy.Availability,
                RegretCoeff = regret
            });
        }

        return parameters;
    }

    protected override Solution FormatSolution<TRoute>(List<TRoute> routes)
    {
        var solution = new Solution();
        
        for (int v = 0; v < routes.Count; v++)
        {
            var route = routes[v];
            var jobs = route.GetJobSequence();
            
            if (jobs.Count == 0)
                continue;

            // Calculate route statistics
            var eval = Utils.Helpers.RouteEvalForVehicle(_input, v, jobs);
            
            var vehicleRoute = new Route
            {
                VehicleId = _input.Vehicles[v].Id,
                Cost = eval.Cost,
                Duration = eval.Duration,
                Distance = eval.Distance
            };

            // Add jobs to route
            foreach (var jobRank in jobs)
            {
                vehicleRoute.Jobs.Add(_input.Jobs[jobRank].Id);
            }

            solution.Routes.Add(vehicleRoute);
        }

        // Add unassigned jobs
        var assignedJobs = new HashSet<int>();
        foreach (var route in routes)
        {
            foreach (var job in route.GetJobSequence())
            {
                assignedJobs.Add(job);
            }
        }

        for (int j = 0; j < _input.Jobs.Count; j++)
        {
            if (!assignedJobs.Contains(j))
            {
                solution.UnassignedJobs.Add(_input.Jobs[j].Id);
            }
        }

        return solution;
    }
}

/// <summary>
/// Basic heuristic implementation.
/// </summary>
internal static class BasicHeuristic<TRoute> where TRoute : RawRoute, new()
{
    public static Eval Solve(
        Input input,
        List<TRoute> routes,
        HashSet<int> unassigned,
        List<int> vehicleRanks,
        InitStrategy init,
        double regretCoeff,
        SortStrategy sort)
    {
        return SolomonI1.Basic(input, routes, unassigned, vehicleRanks, init, regretCoeff, sort);
    }
}

/// <summary>
/// Dynamic vehicle choice heuristic implementation.
/// </summary>
internal static class DynamicHeuristic<TRoute> where TRoute : RawRoute, new()
{
    public static Eval Solve(
        Input input,
        List<TRoute> routes,
        HashSet<int> unassigned,
        List<int> vehicleRanks,
        InitStrategy init,
        double regretCoeff,
        SortStrategy sort)
    {
        // For now, fall back to basic heuristic
        // Full implementation would dynamically choose vehicles based on job proximity
        return BasicHeuristic<TRoute>.Solve(input, routes, unassigned, vehicleRanks, init, regretCoeff, sort);
    }
}