using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.Heuristics;

namespace OptiRoute.Core.Algorithms.VRP;

/// <summary>
/// Vehicle Routing Problem with Time Windows solver (based on VROOM's VRPTW).
/// </summary>
public class VrptwSolver : VrpSolver
{
    public VrptwSolver(Input input) : base(input)
    {
    }

    public override Solution Solve(
        int nbSearches = 1,
        int depth = 4,
        int nbThreads = 1,
        TimeSpan? timeout = null,
        List<HeuristicParameters>? heuristicParams = null)
    {
        return SolveInternal<TWRoute>(nbSearches, depth, nbThreads, timeout, heuristicParams);
    }

    protected override List<HeuristicParameters> GetDefaultParameters()
    {
        // Parameters optimized for time window problems
        var parameters = new List<HeuristicParameters>();

        // Time window problems benefit from earliest deadline initialization
        var initStrategies = new[] { 
            InitStrategy.None, 
            InitStrategy.EarliestDeadline,  // Important for TW
            InitStrategy.Nearest,
            InitStrategy.Furthest 
        };
        
        var sortStrategies = new[] { SortStrategy.Availability, SortStrategy.Cost };
        var regretCoeffs = new[] { 0.0, 0.2, 0.5, 1.0, 1.5, 2.0, 3.0 };

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

            // Add jobs to route with time information
            if (route is TWRoute twRoute)
            {
                for (int i = 0; i < jobs.Count; i++)
                {
                    vehicleRoute.Jobs.Add(_input.Jobs[jobs[i]].Id);
                    
                    // Could add arrival/departure times here if needed
                    // vehicleRoute.Arrivals.Add(twRoute.Earliest[i]);
                    // vehicleRoute.Departures.Add(twRoute.Earliest[i] + twRoute.ActionTime[i]);
                }
            }
            else
            {
                foreach (var jobRank in jobs)
                {
                    vehicleRoute.Jobs.Add(_input.Jobs[jobRank].Id);
                }
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