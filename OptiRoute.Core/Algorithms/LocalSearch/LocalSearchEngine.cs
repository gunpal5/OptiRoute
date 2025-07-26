using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.LocalSearch.Operators;
using OptiRoute.Core.Problems.VRP;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch;

/// <summary>
/// Local search engine for VRP optimization (based on VROOM's LocalSearch).
/// </summary>
public class LocalSearchEngine<TRoute> where TRoute : RawRoute, new()
{
    private readonly Input _input;
    private readonly List<TRoute> _routes;
    private readonly int _depth;
    private readonly DateTime? _deadline;
    private readonly SolutionState _solState;
    private List<TRoute> _bestRoutes;
    private SolutionIndicators _bestIndicators;
    private int? _completedDepth;

    public LocalSearchEngine(Input input, List<TRoute> routes, int depth, TimeSpan? timeout)
    {
        _input = input;
        _routes = routes;
        _depth = depth;
        _deadline = timeout.HasValue ? DateTime.UtcNow + timeout.Value : null;
        _solState = new SolutionState(_input, _input.Vehicles.Count);
        _bestRoutes = new List<TRoute>(routes);
        _bestIndicators = new SolutionIndicators(_input, _routes.Cast<IRoute>());
        
        // Setup solution state
        _solState.Setup(_routes.Cast<IRoute>().ToList());
    }

    /// <summary>
    /// Runs the local search optimization.
    /// </summary>
    public void Run()
    {
        bool tryLsStep = true;

        while (tryLsStep)
        {
            // A round of local search
            RunLsStep();

            // Comparison with indicators for current solution
            var currentIndicators = new SolutionIndicators(_input, _routes.Cast<IRoute>());
            if (currentIndicators.CompareTo(_bestIndicators) < 0)
            {
                _bestIndicators = currentIndicators;
                _bestRoutes = new List<TRoute>(_routes);
            }
            else
            {
                // No improvement so back to previous best known for further steps
                if (_bestIndicators.CompareTo(currentIndicators) < 0)
                {
                    _routes.Clear();
                    _routes.AddRange(_bestRoutes);
                    _solState.Setup(_routes.Cast<IRoute>().ToList());
                }

                if (_completedDepth.HasValue)
                {
                    // Rule out situation with first descent not yielding a better solution
                    _completedDepth++;
                }
            }

            if (!_completedDepth.HasValue)
            {
                // End of first descent
                _completedDepth = 0;
            }

            // Try again on each improvement until we reach last job removal level or deadline is met
            var nbRemoval = _completedDepth.Value + 1;
            tryLsStep = nbRemoval <= _depth && 
                       (!_deadline.HasValue || DateTime.UtcNow < _deadline.Value);

            if (tryLsStep)
            {
                // Get a looser situation by removing jobs
                // Note: This would involve removing jobs and adding them to unassigned
                // For now, we'll skip this part as it requires additional implementation
            }
        }

        // Ensure best solution is returned
        _routes.Clear();
        _routes.AddRange(_bestRoutes);
    }

    /// <summary>
    /// Runs a single local search step.
    /// </summary>
    private void RunLsStep()
    {
        // Store best move involving a pair of routes
        var bestOps = new Operator[_input.Vehicles.Count, _input.Vehicles.Count];
        var bestGains = new Eval[_input.Vehicles.Count, _input.Vehicles.Count];
        
        // Initialize gains
        for (int i = 0; i < _input.Vehicles.Count; i++)
        {
            for (int j = 0; j < _input.Vehicles.Count; j++)
            {
                bestGains[i, j] = new Eval();
            }
        }

        // Store best priority increase and number of assigned tasks for operators
        // involving a single route and unassigned jobs
        var bestPriorities = new int[_input.Vehicles.Count];
        var bestRemovals = Enumerable.Repeat(int.MaxValue, _input.Vehicles.Count).ToArray();

        // List of source/target pairs we need to test
        var stPairs = new List<(int source, int target)>();
        for (int s = 0; s < _input.Vehicles.Count; s++)
        {
            for (int t = 0; t < _input.Vehicles.Count; t++)
            {
                if (VehicleOkWithVehicle(s, t))
                {
                    stPairs.Add((s, t));
                }
            }
        }

        // Dummy init to enter first loop
        var bestGain = new Eval { Cost = 1 };
        int bestPriority = 0;
        int bestRemoval = int.MaxValue;

        while (bestGain.Cost > 0 || bestPriority > 0)
        {
            if (_deadline.HasValue && _deadline.Value < DateTime.UtcNow)
            {
                break;
            }

            // Reset best values
            bestGain = new Eval();
            bestPriority = 0;
            bestRemoval = int.MaxValue;

            // Try all operators
            EvaluateOperators(stPairs, bestOps, bestGains, bestPriorities, bestRemovals);

            // Find best move
            int bestSource = -1;
            int bestTarget = -1;
            
            for (int s = 0; s < _input.Vehicles.Count; s++)
            {
                if (bestPriorities[s] > bestPriority ||
                    (bestPriorities[s] == bestPriority && bestRemovals[s] < bestRemoval))
                {
                    bestPriority = bestPriorities[s];
                    bestRemoval = bestRemovals[s];
                    bestSource = s;
                    bestTarget = s;
                }

                for (int t = 0; t < _input.Vehicles.Count; t++)
                {
                    if (bestGains[s, t].Cost > bestGain.Cost)
                    {
                        bestGain = bestGains[s, t];
                        bestSource = s;
                        bestTarget = t;
                    }
                }
            }

            // Apply best operator if any
            if (bestGain.Cost > 0 || bestPriority > 0)
            {
                bestOps[bestSource, bestTarget].Apply();
                
                // Update solution state
                _solState.Setup(_routes.Cast<IRoute>().ToList());
                
                // Reset gains for affected routes
                for (int v = 0; v < _input.Vehicles.Count; v++)
                {
                    bestGains[bestSource, v] = new Eval();
                    bestGains[v, bestSource] = new Eval();
                    if (bestSource != bestTarget)
                    {
                        bestGains[bestTarget, v] = new Eval();
                        bestGains[v, bestTarget] = new Eval();
                    }
                }
                bestPriorities[bestSource] = 0;
                if (bestSource != bestTarget)
                {
                    bestPriorities[bestTarget] = 0;
                }
            }
        }
    }

    /// <summary>
    /// Evaluates all operators for the current solution.
    /// </summary>
    private void EvaluateOperators(
        List<(int source, int target)> stPairs,
        Operator[,] bestOps,
        Eval[,] bestGains,
        int[] bestPriorities,
        int[] bestRemovals)
    {
        // UnassignedExchange
        if (_input.Jobs.Any())
        {
            foreach (int u in _solState.Unassigned)
            {
                var job = _input.Jobs[u];
                if (job.Type != JobType.Single) continue;

                foreach (var (source, target) in stPairs)
                {
                    if (source != target || !VehicleOkWithJob(source, u) || 
                        _routes[source].Route.Count == 0)
                    {
                        continue;
                    }

                    // For unassigned exchange, find the best position to insert
                    // For simplicity, we'll try the end position (append)
                    var op = new UnassignedExchange(
                        _input, _solState, new HashSet<int>(_solState.Unassigned), 
                        _routes[source], source, _routes[source].Route.Count, _routes[source].Route.Count, u);
                        
                    if (op.IsValid())
                    {
                        var priority = (int)job.Priority;
                        var removal = _routes[source].Route.Count - 1;
                        
                        if (priority > bestPriorities[source] ||
                            (priority == bestPriorities[source] && removal < bestRemovals[source]))
                        {
                            bestPriorities[source] = priority;
                            bestRemovals[source] = removal;
                            bestOps[source, source] = op;
                        }
                    }
                }
            }
        }

        // CrossExchange
        foreach (var (source, target) in stPairs)
        {
            if (target <= source || // This operator is symmetric
                bestPriorities[source] > 0 || bestPriorities[target] > 0 ||
                _routes[source].Route.Count < 2 || _routes[target].Route.Count < 2)
            {
                continue;
            }

            for (int sRank = 0; sRank < _routes[source].Route.Count - 1; sRank++)
            {
                for (int tRank = 0; tRank < _routes[target].Route.Count - 1; tRank++)
                {
                    var op = new CrossExchange(
                        _input, _solState, _routes[source], source, sRank,
                        _routes[target], target, tRank, true, true);

                    if (op.IsValid() && bestGains[source, target].Cost < op.Gain().Cost)
                    {
                        bestGains[source, target] = op.Gain();
                        bestOps[source, target] = op;
                    }
                }
            }
        }

        // Relocate
        foreach (var (source, target) in stPairs)
        {
            if (source == target || bestPriorities[source] > 0 || 
                bestPriorities[target] > 0 || _routes[source].Route.Count == 0)
            {
                continue;
            }

            if (_routes[target].Route.Count + 1 > _input.Vehicles[target].MaxTasks)
            {
                continue;
            }

            for (int sRank = 0; sRank < _routes[source].Route.Count; sRank++)
            {
                for (int tRank = 0; tRank <= _routes[target].Route.Count; tRank++)
                {
                    var op = new Relocate(
                        _input, _solState, _routes[source], source, sRank,
                        _routes[target], target, tRank);

                    if (op.IsValid() && bestGains[source, target].Cost < op.Gain().Cost)
                    {
                        bestGains[source, target] = op.Gain();
                        bestOps[source, target] = op;
                    }
                }
            }
        }

        // OrOpt
        foreach (var (source, target) in stPairs)
        {
            if (source == target || bestPriorities[source] > 0 || 
                bestPriorities[target] > 0 || _routes[source].Route.Count < 2)
            {
                continue;
            }

            if (_routes[target].Route.Count + 2 > _input.Vehicles[target].MaxTasks)
            {
                continue;
            }

            for (int sRank = 0; sRank < _routes[source].Route.Count - 1; sRank++)
            {
                for (int tRank = 0; tRank <= _routes[target].Route.Count; tRank++)
                {
                    var op = new OrOpt(
                        _input, _solState, _routes[source], source, sRank,
                        _routes[target], target, tRank);

                    if (op.IsValid() && bestGains[source, target].Cost < op.Gain().Cost)
                    {
                        bestGains[source, target] = op.Gain();
                        bestOps[source, target] = op;
                    }
                }
            }
        }

        // TwoOpt
        foreach (var (source, target) in stPairs)
        {
            if (source == target || bestPriorities[source] > 0 || 
                bestPriorities[target] > 0 || _routes[source].Route.Count == 0 ||
                _routes[target].Route.Count == 0)
            {
                continue;
            }

            // Regular TwoOpt
            for (int sRank = 0; sRank <= _routes[source].Route.Count; sRank++)
            {
                for (int tRank = 0; tRank <= _routes[target].Route.Count; tRank++)
                {
                    var op = new TwoOpt(
                        _input, _solState, _routes[source], source, sRank,
                        _routes[target], target, tRank);

                    if (op.IsValid() && bestGains[source, target].Cost < op.Gain().Cost)
                    {
                        bestGains[source, target] = op.Gain();
                        bestOps[source, target] = op;
                    }
                }
            }

            // ReverseTwoOpt (only for different vehicles)
            if (source != target)
            {
                for (int sRank = 0; sRank <= _routes[source].Route.Count; sRank++)
                {
                    for (int tRank = 0; tRank <= _routes[target].Route.Count; tRank++)
                    {
                        var op = new ReverseTwoOpt(
                            _input, _solState, _routes[source], source, sRank,
                            _routes[target], target, tRank);

                        if (op.IsValid() && bestGains[source, target].Cost < op.Gain().Cost)
                        {
                            bestGains[source, target] = op.Gain();
                            bestOps[source, target] = op;
                        }
                    }
                }
            }
        }

        // Add more operators as needed (MixedExchange, PDShift, etc.)
    }

    /// <summary>
    /// Checks if two vehicles are compatible.
    /// </summary>
    private bool VehicleOkWithVehicle(int v1, int v2)
    {
        // For now, all vehicles are compatible
        // In VROOM, this checks profile compatibility
        return true;
    }

    /// <summary>
    /// Checks if a vehicle can handle a job.
    /// </summary>
    private bool VehicleOkWithJob(int vehicle, int job)
    {
        // Check skills if any
        var v = _input.Vehicles[vehicle];
        var j = _input.Jobs[job];
        
        if (j.Skills != null && j.Skills.Count > 0)
        {
            if (v.Skills == null || !j.Skills.All(s => v.Skills.Contains(s)))
            {
                return false;
            }
        }
        
        return true;
    }

    /// <summary>
    /// Gets the solution indicators after optimization.
    /// </summary>
    public SolutionIndicators GetIndicators() => _bestIndicators;
}