using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.Heuristics;
using OptiRoute.Core.Algorithms.LocalSearch;

namespace OptiRoute.Core.Algorithms.VRP;

/// <summary>
/// Abstract base class for vehicle routing problems (inspired by VROOM's VRP class).
/// </summary>
public abstract class VrpSolver
{
    protected readonly Input _input;

    protected VrpSolver(Input input)
    {
        _input = input ?? throw new ArgumentNullException(nameof(input));
    }

    /// <summary>
    /// Solves the VRP problem.
    /// </summary>
    public abstract Solution Solve(
        int nbSearches = 1,
        int depth = 4,
        int nbThreads = 1,
        TimeSpan? timeout = null,
        List<HeuristicParameters>? heuristicParams = null);

    /// <summary>
    /// Runs the VRP solving algorithm with multiple search threads.
    /// </summary>
    protected Solution SolveInternal<TRoute>(
        int nbSearches,
        int depth,
        int nbThreads,
        TimeSpan? timeout,
        List<HeuristicParameters>? heuristicParams)
        where TRoute : RawRoute, new()
    {
        // Use provided parameters or default based on problem type
        var parameters = heuristicParams ?? GetDefaultParameters();
        nbSearches = Math.Min(nbSearches, parameters.Count);

        var context = new SolvingContext<TRoute>(_input, nbSearches);
        
        // Distribute searches among threads
        var threadTasks = DistributeSearches(nbSearches, nbThreads);
        var searchTimeout = timeout.HasValue ? timeout.Value / nbSearches : (TimeSpan?)null;

        var tasks = new List<Task>();
        foreach (var searchIndices in threadTasks)
        {
            var indices = searchIndices; // Capture variable
            var task = Task.Run(() =>
            {
                foreach (var index in indices)
                {
                    RunSingleSearch<TRoute>(
                        parameters[index],
                        index,
                        depth,
                        searchTimeout,
                        context);
                }
            });
            tasks.Add(task);
        }

        Task.WaitAll(tasks.ToArray());

        // Find best solution
        var bestIndex = 0;
        var bestIndicators = context.SolutionIndicators[0];
        for (int i = 1; i < nbSearches; i++)
        {
            if (context.SolutionIndicators[i].CompareTo(bestIndicators) < 0)
            {
                bestIndex = i;
                bestIndicators = context.SolutionIndicators[i];
            }
        }

        return FormatSolution(context.Solutions[bestIndex]);
    }

    /// <summary>
    /// Runs a single search iteration.
    /// </summary>
    private void RunSingleSearch<TRoute>(
        HeuristicParameters parameters,
        int rank,
        int depth,
        TimeSpan? searchTimeout,
        SolvingContext<TRoute> context)
        where TRoute : RawRoute, new()
    {
        var startTime = DateTime.UtcNow;

        // Apply heuristic
        Eval heuristicEval;
        switch (parameters.Heuristic)
        {
            case HeuristicType.Basic:
                heuristicEval = BasicHeuristic<TRoute>.Solve(
                    _input,
                    context.Solutions[rank],
                    context.Unassigned,
                    context.VehicleRanks,
                    parameters.Init,
                    parameters.RegretCoeff,
                    parameters.Sort);
                break;
                
            case HeuristicType.Dynamic:
                heuristicEval = DynamicHeuristic<TRoute>.Solve(
                    _input,
                    context.Solutions[rank],
                    context.Unassigned,
                    context.VehicleRanks,
                    parameters.Init,
                    parameters.RegretCoeff,
                    parameters.Sort);
                break;
                
            default:
                throw new ArgumentException($"Unknown heuristic type: {parameters.Heuristic}");
        }

        // Check if solution has been found before
        context.SolutionIndicators[rank] = new SolutionIndicators(_input, context.Solutions[rank].Cast<IRoute>());
        if (context.HeuristicSolutionAlreadyFound(rank))
        {
            return; // Skip local search for duplicate
        }

        // Calculate remaining time for local search
        var heuristicTime = DateTime.UtcNow - startTime;
        TimeSpan? lsTimeout = null;
        if (searchTimeout.HasValue)
        {
            var remaining = searchTimeout.Value - heuristicTime;
            if (remaining <= TimeSpan.Zero)
                return; // No time for local search
            lsTimeout = remaining;
        }

        // Apply local search
        // Convert TRoute to RawRoute for local search
        var rawRoutes = context.Solutions[rank].Select(r => 
        {
            var raw = new RawRoute(_input, r.GetVehicleIndex());
            raw.Route.AddRange(r.GetJobSequence());
            raw.UpdateAmounts();
            return raw;
        }).ToList();

        var deadline = lsTimeout.HasValue ? DateTime.UtcNow + lsTimeout.Value : DateTime.MaxValue;
        var localSearch = new LocalSearch.LocalSearch(_input, rawRoutes, depth, deadline);
        localSearch.Run();

        // Copy back results
        for (int v = 0; v < rawRoutes.Count; v++)
        {
            context.Solutions[rank][v].GetJobSequence().Clear();
            context.Solutions[rank][v].GetJobSequence().AddRange(rawRoutes[v].Route);
        }

        // Update solution indicators
        context.SolutionIndicators[rank] = new SolutionIndicators(_input, context.Solutions[rank].Cast<IRoute>());
    }

    /// <summary>
    /// Distributes search indices among threads.
    /// </summary>
    private List<List<int>> DistributeSearches(int nbSearches, int nbThreads)
    {
        var distribution = new List<List<int>>();
        for (int i = 0; i < nbThreads; i++)
        {
            distribution.Add(new List<int>());
        }

        for (int i = 0; i < nbSearches; i++)
        {
            distribution[i % nbThreads].Add(i);
        }

        return distribution;
    }

    /// <summary>
    /// Gets default heuristic parameters based on problem characteristics.
    /// </summary>
    protected abstract List<HeuristicParameters> GetDefaultParameters();

    /// <summary>
    /// Formats the raw solution routes into a final solution.
    /// </summary>
    protected abstract Solution FormatSolution<TRoute>(List<TRoute> routes) where TRoute : IRoute;
}

/// <summary>
/// Context for VRP solving process.
/// </summary>
internal class SolvingContext<TRoute> where TRoute : IRoute, new()
{
    public HashSet<int> InitAssigned { get; }
    public List<TRoute> InitialSolution { get; }
    public HashSet<int> Unassigned { get; }
    public List<int> VehicleRanks { get; }
    public List<List<TRoute>> Solutions { get; }
    public List<SolutionIndicators> SolutionIndicators { get; }
    
    private readonly HashSet<SolutionIndicators> _heuristicIndicators = new();
    private readonly object _indicatorLock = new();

    public SolvingContext(Input input, int nbSearches)
    {
        InitAssigned = new HashSet<int>();
        InitialSolution = SetInitialSolution(input);
        VehicleRanks = Enumerable.Range(0, input.Vehicles.Count).ToList();
        Solutions = new List<List<TRoute>>(nbSearches);
        SolutionIndicators = new List<SolutionIndicators>(nbSearches);

        // Initialize solutions with initial solution
        for (int i = 0; i < nbSearches; i++)
        {
            Solutions.Add(new List<TRoute>(InitialSolution));
            SolutionIndicators.Add(new SolutionIndicators());
        }

        // Determine unassigned jobs
        Unassigned = new HashSet<int>();
        for (int i = 0; i < input.Jobs.Count; i++)
        {
            if (!InitAssigned.Contains(i))
                Unassigned.Add(i);
        }
    }

    public bool HeuristicSolutionAlreadyFound(int rank)
    {
        lock (_indicatorLock)
        {
            return !_heuristicIndicators.Add(SolutionIndicators[rank]);
        }
    }

    private List<TRoute> SetInitialSolution(Input input)
    {
        var routes = new List<TRoute>();
        for (int v = 0; v < input.Vehicles.Count; v++)
        {
            var route = new TRoute();
            route.Initialize(input, v);
            routes.Add(route);
        }

        // TODO: Handle initial routes if provided
        return routes;
    }
}

/// <summary>
/// Interface for route implementations.
/// </summary>
public interface IRoute
{
    void Initialize(Input input, int vehicleIndex);
    int GetVehicleIndex();
    Amount GetCapacity();
    List<int> GetJobSequence();
    void InsertJob(int jobRank, int position);
}