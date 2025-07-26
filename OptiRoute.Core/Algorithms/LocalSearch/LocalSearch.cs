using System.Diagnostics;
using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.LocalSearch.Operators;
using OptiRoute.Core.Problems.VRP;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch;

/// <summary>
/// Local search implementation for VRP optimization (based on VROOM).
/// </summary>
internal class LocalSearch
{
    private readonly Input _input;
    private readonly int _nbVehicles;
    private readonly int _depth;
    private readonly DateTime _deadline;
    private readonly List<RawRoute> _solution;
    private readonly List<RawRoute> _bestSolution;
    private readonly SolutionState _solutionState;
    private Eval _bestCost;
    private int? _completedDepth;
    private readonly List<int> _allRoutes;

    public LocalSearch(Input input, List<RawRoute> solution, int depth, DateTime deadline)
    {
        _input = input;
        _solution = solution;
        _nbVehicles = solution.Count;
        _depth = depth;
        _deadline = deadline;
        _bestSolution = new List<RawRoute>(solution.Count);
        _solutionState = new SolutionState(input, _nbVehicles);
        _allRoutes = Enumerable.Range(0, _nbVehicles).ToList();
        
        // Initialize best solution
        foreach (var route in solution)
        {
            _bestSolution.Add(new RawRoute(input, route.VehicleRank));
            _bestSolution[^1].Route.AddRange(route.Route);
            _bestSolution[^1].UpdateAmounts();
        }
        
        // Setup solution state
        _solutionState.Setup(_solution.Cast<IRoute>().ToList());
        
        _bestCost = ComputeTotalCost();
    }

    /// <summary>
    /// Run the local search optimization.
    /// </summary>
    public void Run()
    {
        // Try to add unassigned jobs
        if (_solutionState.Unassigned.Count > 0)
        {
            TryJobAdditions(_allRoutes, 1.0);
        }
        
        for (int d = 0; d < _depth; d++)
        {
            if (DateTime.Now >= _deadline)
                break;
                
            RunLocalSearchStep();
            
            // Try job additions with decreasing regret coefficient
            if (_solutionState.Unassigned.Count > 0)
            {
                double regretCoeff = 1.0 - (double)d / _depth;
                TryJobAdditions(_allRoutes, regretCoeff);
            }
            
            var currentCost = ComputeTotalCost();
            if (currentCost.Cost < _bestCost.Cost)
            {
                _bestCost = currentCost;
                UpdateBestSolution();
            }
            
            // Try ruin and recreate if we're stuck
            if (d > 0 && d % 10 == 0 && DateTime.Now < _deadline)
            {
                // Remove some jobs to get looser situation
                int nbRemoval = Math.Min(3, (int)Math.Sqrt(_input.Jobs.Count));
                for (int i = 0; i < nbRemoval; i++)
                {
                    RemoveFromRoutes();
                    
                    // Update required state after removal
                    for (int v = 0; v < _solution.Count; v++)
                    {
                        _solutionState.UpdateRoute(v, _solution[v]);
                    }
                }
                
                // Refill jobs with higher regret
                const double refillRegret = 1.5;
                TryJobAdditions(_allRoutes, refillRegret);
            }
            
            _completedDepth = d + 1;
        }
        
        // Restore best solution
        RestoreBestSolution();
    }

    /// <summary>
    /// Run a single local search step trying all operators.
    /// </summary>
    private void RunLocalSearchStep()
    {
        bool improved;
        do
        {
            improved = false;
            
            // Try relocate moves
            improved |= TryRelocateOperators();
            
            // Try cross-exchange moves
            improved |= TryCrossExchangeOperators();
            
            // Try or-opt moves
            improved |= TryOrOptOperators();
            
            // Try two-opt moves
            improved |= TryTwoOptOperators();
            
            // Try intra-route moves
            improved |= TryIntraRelocateOperators();
            improved |= TryIntraOrOptOperators();
            improved |= TryIntraTwoOptOperators();
            
            // Try mixed exchange moves
            improved |= TryMixedExchangeOperators();
            
            // Try intra cross exchange moves
            improved |= TryIntraCrossExchangeOperators();
            
            // Try route exchange moves
            improved |= TryRouteExchangeOperators();
            
            // Try intra exchange operators
            improved |= TryIntraExchangeOperators();
            
            // Try intra mixed exchange operators
            improved |= TryIntraMixedExchangeOperators();
            
            // Try TSP fix operators
            improved |= TryTSPFixOperators();
            
            // Try unassigned exchange if we have unassigned jobs
            if (_solutionState.Unassigned.Count > 0)
            {
                improved |= TryUnassignedExchangeOperators();
            }
            
            // Try other operators
            improved |= TryReverseTwoOptOperators();
            improved |= TryPDShiftOperators();
            improved |= TryRouteSplitOperators();
            improved |= TrySwapStarOperators();
            improved |= TryPriorityReplaceOperators();
            
        } while (improved && DateTime.Now < _deadline);
    }

    /// <summary>
    /// Try all relocate moves.
    /// </summary>
    private bool TryRelocateOperators()
    {
        var bestGain = Eval.NoEval;
        Relocate? bestMove = null;
        
        // Try all possible relocate moves
        for (int s = 0; s < _nbVehicles; s++)
        {
            var sourceRoute = _solution[s];
            if (sourceRoute.Route.Count == 0) continue;
            
            for (int sRank = 0; sRank < sourceRoute.Route.Count; sRank++)
            {
                for (int t = 0; t < _nbVehicles; t++)
                {
                    if (s == t) continue;
                    
                    var targetRoute = _solution[t];
                    
                    // Check vehicle compatibility
                    if (!_input.VehicleOkWithJob(t, sourceRoute.Route[sRank]))
                        continue;
                    
                    for (int tRank = 0; tRank <= targetRoute.Route.Count; tRank++)
                    {
                        try
                        {
                            var move = new Relocate(_input, _solutionState, sourceRoute, s, sRank, 
                                                   targetRoute, t, tRank);
                            
                            if (move.IsValid())
                            {
                                var gain = move.Gain();
                                if (gain.Cost > bestGain.Cost)
                                {
                                    bestGain = gain;
                                    bestMove = move;
                                }
                            }
                        }
                        catch
                        {
                            // Invalid move configuration
                        }
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            // Update solution state for affected routes
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }

    /// <summary>
    /// Try all cross-exchange moves.
    /// </summary>
    private bool TryCrossExchangeOperators()
    {
        var bestGain = Eval.NoEval;
        CrossExchange? bestMove = null;
        
        for (int s = 0; s < _nbVehicles; s++)
        {
            var sourceRoute = _solution[s];
            if (sourceRoute.Route.Count < 2) continue;
            
            for (int sRank = 0; sRank < sourceRoute.Route.Count - 1; sRank++)
            {
                for (int t = s + 1; t < _nbVehicles; t++)
                {
                    var targetRoute = _solution[t];
                    if (targetRoute.Route.Count < 2) continue;
                    
                    for (int tRank = 0; tRank < targetRoute.Route.Count - 1; tRank++)
                    {
                        try
                        {
                            var move = new CrossExchange(_input, _solutionState, sourceRoute, s, sRank,
                                                       targetRoute, t, tRank, true, true);
                            
                            var upperBound = move.GainUpperBound();
                            if (upperBound.Cost <= bestGain.Cost)
                                continue;
                            
                            if (move.IsValid())
                            {
                                var gain = move.Gain();
                                if (gain.Cost > bestGain.Cost)
                                {
                                    bestGain = gain;
                                    bestMove = move;
                                }
                            }
                        }
                        catch
                        {
                            // Invalid move configuration
                        }
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }

    /// <summary>
    /// Try all or-opt moves.
    /// </summary>
    private bool TryOrOptOperators()
    {
        var bestGain = Eval.NoEval;
        OrOpt? bestMove = null;
        
        // OrOpt only handles edges (2-job sequences)
        for (int s = 0; s < _nbVehicles; s++)
        {
            var sourceRoute = _solution[s];
            if (sourceRoute.Route.Count < 2) continue;
            
            for (int sRank = 0; sRank <= sourceRoute.Route.Count - 2; sRank++)
            {
                for (int t = 0; t < _nbVehicles; t++)
                {
                    if (s == t) continue;
                    
                    var targetRoute = _solution[t];
                    
                    for (int tRank = 0; tRank <= targetRoute.Route.Count; tRank++)
                    {
                        try
                        {
                            var move = new OrOpt(_input, _solutionState, sourceRoute, s, sRank,
                                               targetRoute, t, tRank);
                            
                            var upperBound = move.GainUpperBound();
                            if (upperBound.Cost > bestGain.Cost && move.IsValid())
                            {
                                var gain = move.Gain();
                                if (gain.Cost > bestGain.Cost)
                                {
                                    bestGain = gain;
                                    bestMove = move;
                                }
                            }
                        }
                        catch
                        {
                            // Invalid move configuration
                        }
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }

    /// <summary>
    /// Try all two-opt moves.
    /// </summary>
    private bool TryTwoOptOperators()
    {
        var bestGain = Eval.NoEval;
        TwoOpt? bestMove = null;
        
        for (int s = 0; s < _nbVehicles; s++)
        {
            var sourceRoute = _solution[s];
            if (sourceRoute.Route.Count == 0) continue;
            
            for (int sRank = 0; sRank < sourceRoute.Route.Count; sRank++)
            {
                for (int t = s + 1; t < _nbVehicles; t++)
                {
                    var targetRoute = _solution[t];
                    if (targetRoute.Route.Count == 0) continue;
                    
                    for (int tRank = 0; tRank < targetRoute.Route.Count; tRank++)
                    {
                        try
                        {
                            var move = new TwoOpt(_input, _solutionState, sourceRoute, s, sRank,
                                                targetRoute, t, tRank);
                            
                            if (move.IsValid())
                            {
                                var gain = move.Gain();
                                if (gain.Cost > bestGain.Cost)
                                {
                                    bestGain = gain;
                                    bestMove = move;
                                }
                            }
                        }
                        catch
                        {
                            // Invalid move configuration
                        }
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }

    /// <summary>
    /// Compute the total cost of the current solution.
    /// </summary>
    private Eval ComputeTotalCost()
    {
        var total = new Eval(0, 0, 0);
        for (int v = 0; v < _nbVehicles; v++)
        {
            total = total + _solutionState.RouteEvals[v];
        }
        return total;
    }

    /// <summary>
    /// Update the best solution with the current solution.
    /// </summary>
    private void UpdateBestSolution()
    {
        for (int v = 0; v < _nbVehicles; v++)
        {
            _bestSolution[v].Route.Clear();
            _bestSolution[v].Route.AddRange(_solution[v].Route);
            _bestSolution[v].UpdateAmounts();
        }
    }

    /// <summary>
    /// Restore the best solution to the current solution.
    /// </summary>
    private void RestoreBestSolution()
    {
        for (int v = 0; v < _nbVehicles; v++)
        {
            _solution[v].Route.Clear();
            _solution[v].Route.AddRange(_bestSolution[v].Route);
            _solution[v].UpdateAmounts();
            _solutionState.UpdateRoute(v, _solution[v]);
        }
    }

    /// <summary>
    /// Get the best cost found.
    /// </summary>
    public Eval GetBestCost() => _bestCost;

    /// <summary>
    /// Get the number of completed depth levels.
    /// </summary>
    public int? GetCompletedDepth() => _completedDepth;
    
    /// <summary>
    /// Try all intra-relocate moves (moving a job within the same route).
    /// </summary>
    private bool TryIntraRelocateOperators()
    {
        var bestGain = Eval.NoEval;
        IntraRelocate? bestMove = null;
        
        for (int v = 0; v < _nbVehicles; v++)
        {
            var route = _solution[v];
            if (route.Route.Count < 2) continue;
            
            for (int sRank = 0; sRank < route.Route.Count; sRank++)
            {
                for (int tRank = 0; tRank < route.Route.Count; tRank++)
                {
                    if (sRank == tRank) continue;
                    
                    // Target rank is position AFTER removal
                    int actualTargetRank = tRank < sRank ? tRank : tRank - 1;
                    
                    try
                    {
                        var move = new IntraRelocate(_input, _solutionState, route, v, sRank, actualTargetRank);
                        
                        if (move.IsValid())
                        {
                            var gain = move.Gain();
                            if (gain.Cost > bestGain.Cost)
                            {
                                bestGain = gain;
                                bestMove = move;
                            }
                        }
                    }
                    catch
                    {
                        // Invalid move configuration
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all intra-or-opt moves (moving an edge within the same route).
    /// </summary>
    private bool TryIntraOrOptOperators()
    {
        var bestGain = Eval.NoEval;
        IntraOrOpt? bestMove = null;
        
        for (int v = 0; v < _nbVehicles; v++)
        {
            var route = _solution[v];
            if (route.Route.Count < 4) continue; // Need at least 4 jobs for intra-or-opt
            
            for (int sRank = 0; sRank < route.Route.Count - 1; sRank++)
            {
                // Check if we can reverse the edge
                var job1 = _input.Jobs[route.Route[sRank]];
                var job2 = _input.Jobs[route.Route[sRank + 1]];
                bool checkReverse = job1.Type == JobType.Single && job2.Type == JobType.Single;
                
                for (int tRank = 0; tRank <= route.Route.Count - 2; tRank++)
                {
                    if (sRank == tRank) continue;
                    
                    try
                    {
                        var move = new IntraOrOpt(_input, _solutionState, route, v, sRank, tRank, checkReverse);
                        
                        var upperBound = move.GainUpperBound();
                        if (upperBound.Cost > bestGain.Cost && move.IsValid())
                        {
                            var gain = move.Gain();
                            if (gain.Cost > bestGain.Cost)
                            {
                                bestGain = gain;
                                bestMove = move;
                            }
                        }
                    }
                    catch
                    {
                        // Invalid move configuration
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all intra-two-opt moves (reversing a segment within the same route).
    /// </summary>
    private bool TryIntraTwoOptOperators()
    {
        var bestGain = Eval.NoEval;
        IntraTwoOpt? bestMove = null;
        
        for (int v = 0; v < _nbVehicles; v++)
        {
            var route = _solution[v];
            if (route.Route.Count < 3) continue; // Need at least 3 jobs
            
            for (int sRank = 0; sRank < route.Route.Count - 1; sRank++)
            {
                for (int tRank = sRank + 2; tRank < route.Route.Count; tRank++)
                {
                    try
                    {
                        var move = new IntraTwoOpt(_input, _solutionState, route, v, sRank, tRank);
                        
                        if (move.IsValid())
                        {
                            var gain = move.Gain();
                            if (gain.Cost > bestGain.Cost)
                            {
                                bestGain = gain;
                                bestMove = move;
                            }
                        }
                    }
                    catch
                    {
                        // Invalid move configuration
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all mixed exchange moves (exchange single job with edge).
    /// </summary>
    private bool TryMixedExchangeOperators()
    {
        var bestGain = Eval.NoEval;
        MixedExchange? bestMove = null;
        
        for (int s = 0; s < _nbVehicles; s++)
        {
            var sourceRoute = _solution[s];
            if (sourceRoute.Route.Count < 1) continue;
            
            for (int sRank = 0; sRank < sourceRoute.Route.Count; sRank++)
            {
                for (int t = 0; t < _nbVehicles; t++)
                {
                    if (s == t) continue;
                    
                    var targetRoute = _solution[t];
                    if (targetRoute.Route.Count < 2) continue;
                    
                    for (int tRank = 0; tRank < targetRoute.Route.Count - 1; tRank++)
                    {
                        // Check if target edge can be reversed
                        var job1 = _input.Jobs[targetRoute.Route[tRank]];
                        var job2 = _input.Jobs[targetRoute.Route[tRank + 1]];
                        bool checkReverse = job1.Type == JobType.Single && job2.Type == JobType.Single;
                        
                        try
                        {
                            var move = new MixedExchange(_input, _solutionState, sourceRoute, s, sRank,
                                                       targetRoute, t, tRank, checkReverse);
                            
                            var upperBound = move.GainUpperBound();
                            if (upperBound.Cost > bestGain.Cost && move.IsValid())
                            {
                                var gain = move.Gain();
                                if (gain.Cost > bestGain.Cost)
                                {
                                    bestGain = gain;
                                    bestMove = move;
                                }
                            }
                        }
                        catch
                        {
                            // Invalid move configuration
                        }
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all intra cross exchange moves (exchange two edges within same route).
    /// </summary>
    private bool TryIntraCrossExchangeOperators()
    {
        var bestGain = Eval.NoEval;
        IntraCrossExchange? bestMove = null;
        
        for (int v = 0; v < _nbVehicles; v++)
        {
            var route = _solution[v];
            if (route.Route.Count < 5) continue; // Need at least 5 jobs
            
            for (int sRank = 0; sRank < route.Route.Count - 3; sRank++)
            {
                // Check if source edge can be reversed
                var sJob1 = _input.Jobs[route.Route[sRank]];
                var sJob2 = _input.Jobs[route.Route[sRank + 1]];
                bool checkSReverse = sJob1.Type == JobType.Single && sJob2.Type == JobType.Single;
                
                for (int tRank = sRank + 2; tRank < route.Route.Count - 1; tRank++)
                {
                    // Check if target edge can be reversed
                    var tJob1 = _input.Jobs[route.Route[tRank]];
                    var tJob2 = _input.Jobs[route.Route[tRank + 1]];
                    bool checkTReverse = tJob1.Type == JobType.Single && tJob2.Type == JobType.Single;
                    
                    try
                    {
                        var move = new IntraCrossExchange(_input, _solutionState, route, v, sRank, tRank,
                                                         checkSReverse, checkTReverse);
                        
                        var upperBound = move.GainUpperBound();
                        if (upperBound.Cost > bestGain.Cost && move.IsValid())
                        {
                            var gain = move.Gain();
                            if (gain.Cost > bestGain.Cost)
                            {
                                bestGain = gain;
                                bestMove = move;
                            }
                        }
                    }
                    catch
                    {
                        // Invalid move configuration
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all route exchange moves (exchange complete routes between vehicles).
    /// </summary>
    private bool TryRouteExchangeOperators()
    {
        var bestGain = Eval.NoEval;
        RouteExchange? bestMove = null;
        
        for (int s = 0; s < _nbVehicles; s++)
        {
            var sourceRoute = _solution[s];
            
            for (int t = s + 1; t < _nbVehicles; t++)
            {
                var targetRoute = _solution[t];
                
                // Skip if both routes are empty
                if (sourceRoute.Route.Count == 0 && targetRoute.Route.Count == 0)
                    continue;
                    
                // Check if routes can be exchanged (skill compatibility)
                if (_solutionState.BwdSkillRank[s][t] != 0 || 
                    _solutionState.BwdSkillRank[t][s] != 0)
                    continue;
                
                try
                {
                    var move = new RouteExchange(_input, _solutionState, sourceRoute, s, targetRoute, t);
                    
                    if (move.IsValid())
                    {
                        var gain = move.Gain();
                        if (gain.Cost > bestGain.Cost)
                        {
                            bestGain = gain;
                            bestMove = move;
                        }
                    }
                }
                catch
                {
                    // Invalid move configuration
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all intra-exchange moves (swapping two jobs within the same route).
    /// </summary>
    private bool TryIntraExchangeOperators()
    {
        var bestGain = Eval.NoEval;
        IntraExchange? bestMove = null;
        
        for (int v = 0; v < _nbVehicles; v++)
        {
            var route = _solution[v];
            if (route.Route.Count < 3) continue; // Need at least 3 jobs for intra-exchange
            
            for (int sRank = 0; sRank < route.Route.Count - 2; sRank++)
            {
                for (int tRank = sRank + 2; tRank < route.Route.Count; tRank++)
                {
                    try
                    {
                        var move = new IntraExchange(_input, _solutionState, route, v, sRank, tRank);
                        
                        if (move.IsValid())
                        {
                            var gain = move.Gain();
                            if (gain.Cost > bestGain.Cost)
                            {
                                bestGain = gain;
                                bestMove = move;
                            }
                        }
                    }
                    catch
                    {
                        // Invalid move configuration
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all intra-mixed-exchange moves (swapping a job with an edge within the same route).
    /// </summary>
    private bool TryIntraMixedExchangeOperators()
    {
        var bestGain = Eval.NoEval;
        IntraMixedExchange? bestMove = null;
        
        for (int v = 0; v < _nbVehicles; v++)
        {
            var route = _solution[v];
            if (route.Route.Count < 4) continue; // Need at least 4 jobs
            
            for (int sRank = 0; sRank < route.Route.Count; sRank++)
            {
                for (int tRank = 0; tRank < route.Route.Count - 1; tRank++)
                {
                    // Skip invalid configurations
                    if (sRank + 1 >= tRank && tRank + 2 >= sRank) continue;
                    
                    // Check if we can reverse the edge
                    var job1 = _input.Jobs[route.Route[tRank]];
                    var job2 = _input.Jobs[route.Route[tRank + 1]];
                    bool checkReverse = job1.Type == JobType.Single && job2.Type == JobType.Single;
                    
                    try
                    {
                        var move = new IntraMixedExchange(_input, _solutionState, route, v, sRank, tRank, checkReverse);
                        
                        var gainUpperBound = move.GainUpperBound();
                        if (gainUpperBound.Cost > bestGain.Cost && move.IsValid())
                        {
                            var gain = move.Gain();
                            if (gain.Cost > bestGain.Cost)
                            {
                                bestGain = gain;
                                bestMove = move;
                            }
                        }
                    }
                    catch
                    {
                        // Invalid move configuration
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try TSP fix operators to optimize job ordering within routes.
    /// </summary>
    private bool TryTSPFixOperators()
    {
        var bestGain = Eval.NoEval;
        TSPFix? bestMove = null;
        
        for (int v = 0; v < _nbVehicles; v++)
        {
            var route = _solution[v];
            if (route.Route.Count < 2) continue; // Need at least 2 jobs
            
            try
            {
                var move = new TSPFix(_input, _solutionState, route, v);
                
                if (move.IsValid())
                {
                    var gain = move.Gain();
                    if (gain.Cost > bestGain.Cost)
                    {
                        bestGain = gain;
                        bestMove = move;
                    }
                }
            }
            catch
            {
                // Invalid move configuration
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try to add unassigned jobs to routes using regret-based insertion.
    /// </summary>
    private HashSet<int> TryJobAdditions(List<int> routes, double regretCoeff)
    {
        bool jobAdded;
        var modifiedVehicles = new HashSet<int>();
        
        // Store best insertions for each route and job
        var routeJobInsertions = new List<List<RouteInsertion>>(routes.Count);
        for (int i = 0; i < routes.Count; i++)
        {
            routeJobInsertions.Add(new List<RouteInsertion>(_input.Jobs.Count));
            for (int j = 0; j < _input.Jobs.Count; j++)
            {
                routeJobInsertions[i].Add(new RouteInsertion(_input.AmountSize));
            }
            
            var v = routes[i];
            var fixedCost = _solution[v].Route.Count == 0 ? _input.Vehicles[v].Costs.Fixed : 0;
            
            foreach (var j in _solutionState.Unassigned)
            {
                var currentJob = _input.Jobs[j];
                if (currentJob.Type == JobType.Delivery)
                    continue;
                    
                routeJobInsertions[i][j] = ComputeBestInsertion(j, v, _solution[v]);
                
                if (routeJobInsertions[i][j].Eval != Eval.NoEval)
                {
                    var insertion = routeJobInsertions[i][j];
                    insertion.Eval = insertion.Eval with 
                    { 
                        Cost = insertion.Eval.Cost + fixedCost 
                    };
                    routeJobInsertions[i][j] = insertion;
                }
            }
        }
        
        do
        {
            int bestPriority = 0;
            var bestInsertion = new RouteInsertion(_input.AmountSize);
            double bestCost = double.MaxValue;
            int bestJobRank = -1;
            int bestRoute = -1;
            int bestRouteIdx = -1;
            
            foreach (var j in _solutionState.Unassigned)
            {
                var currentJob = _input.Jobs[j];
                if (currentJob.Type == JobType.Delivery)
                    continue;
                    
                var jobPriority = currentJob.Priority;
                
                if (jobPriority < bestPriority)
                    continue;
                    
                long smallest = _input.CostUpperBound;
                long secondSmallest = _input.CostUpperBound;
                int smallestIdx = -1;
                
                for (int i = 0; i < routes.Count; i++)
                {
                    if (routeJobInsertions[i][j].Eval.Cost < smallest)
                    {
                        smallestIdx = i;
                        secondSmallest = smallest;
                        smallest = routeJobInsertions[i][j].Eval.Cost;
                    }
                    else if (routeJobInsertions[i][j].Eval.Cost < secondSmallest)
                    {
                        secondSmallest = routeJobInsertions[i][j].Eval.Cost;
                    }
                }
                
                // Find best route for current job based on cost and regret
                for (int i = 0; i < routes.Count; i++)
                {
                    if (routeJobInsertions[i][j].Eval == Eval.NoEval)
                        continue;
                        
                    var currentRoute = _solution[routes[i]];
                    var vehicle = _input.Vehicles[routes[i]];
                    
                    bool isPickup = (_input.Jobs[j].Type == JobType.Pickup);
                    if (currentRoute.Route.Count + (isPickup ? 2 : 1) > vehicle.MaxTasks)
                        continue;
                        
                    var regretCost = (i == smallestIdx) ? secondSmallest : smallest;
                    
                    double currentCost = routeJobInsertions[i][j].Eval.Cost - 
                                       regretCoeff * regretCost;
                    
                    if ((jobPriority > bestPriority) ||
                        (jobPriority == bestPriority && currentCost < bestCost))
                    {
                        bestPriority = jobPriority;
                        bestJobRank = j;
                        bestRoute = routes[i];
                        bestInsertion = routeJobInsertions[i][j];
                        bestCost = currentCost;
                        bestRouteIdx = i;
                    }
                }
            }
            
            jobAdded = (bestCost < double.MaxValue);
            
            if (jobAdded)
            {
                _solutionState.Unassigned.Remove(bestJobRank);
                
                var bestJob = _input.Jobs[bestJobRank];
                if (bestJob.Type == JobType.Single)
                {
                    _solution[bestRoute].Add(_input, bestJobRank, bestInsertion.SingleRank);
                }
                else if (bestJob.Type == JobType.Pickup)
                {
                    var modifiedWithPd = new List<int>();
                    modifiedWithPd.Add(bestJobRank);
                    
                    for (int i = bestInsertion.PickupRank; i < bestInsertion.DeliveryRank; i++)
                    {
                        modifiedWithPd.Add(_solution[bestRoute].Route[i]);
                    }
                    modifiedWithPd.Add(bestJobRank + 1);
                    
                    _solution[bestRoute].Replace(_input,
                                               bestInsertion.Delivery,
                                               modifiedWithPd.ToArray(),
                                               0,
                                               modifiedWithPd.Count,
                                               bestInsertion.PickupRank,
                                               bestInsertion.DeliveryRank);
                    
                    _solutionState.Unassigned.Remove(bestJobRank + 1);
                }
                
                // Update best_route data for consistency
                modifiedVehicles.Add(bestRoute);
                _solutionState.UpdateRoute(bestRoute, _solution[bestRoute]);
                
                var fixedCost = _solution[bestRoute].Route.Count == 0 ? 
                              _input.Vehicles[bestRoute].Costs.Fixed : 0;
                
                // Update insertions for modified route
                foreach (var j in _solutionState.Unassigned)
                {
                    var currentJob = _input.Jobs[j];
                    if (currentJob.Type == JobType.Delivery)
                        continue;
                        
                    routeJobInsertions[bestRouteIdx][j] = ComputeBestInsertion(j, bestRoute, _solution[bestRoute]);
                    
                    if (routeJobInsertions[bestRouteIdx][j].Eval != Eval.NoEval)
                    {
                        var insertion = routeJobInsertions[bestRouteIdx][j];
                        insertion.Eval = insertion.Eval with 
                        { 
                            Cost = insertion.Eval.Cost + fixedCost 
                        };
                        routeJobInsertions[bestRouteIdx][j] = insertion;
                    }
                }
            }
        } while (jobAdded);
        
        return modifiedVehicles;
    }
    
    private void RemoveFromRoutes()
    {
        // Store nearest job from and to any job in any route for constant
        // time access down the line.
        for (int v1 = 0; v1 < _nbVehicles; v1++)
        {
            for (int v2 = 0; v2 < _nbVehicles; v2++)
            {
                if (v2 == v1) continue;
                
                _solutionState.UpdateCheapestJobRankInRoutes(
                    _solution[v1].Route, _solution[v2].Route, v1, v2);
            }
        }
        
        // Remove best node candidate from all routes.
        var routesAndRanks = new List<(int Vehicle, int Rank)>();
        
        for (int v = 0; v < _solution.Count; v++)
        {
            if (_solution[v].IsEmpty()) continue;
            
            // Try removing the best node (good gain on current route and
            // small cost to closest node in another compatible route).
            int bestRank = 0;
            var bestGain = Eval.NoGain;
            
            var routeEval = _solutionState.RouteEvals[v];
            
            for (int r = 0; r < _solution[v].Size; r++)
            {
                var currentJob = _input.Jobs[_solution[v].Route[r]];
                if (currentJob.Type == JobType.Delivery) continue;
                
                Eval currentGain;
                bool validRemoval = false;
                
                if (currentJob.Type == JobType.Single)
                {
                    var removalGain = _solutionState.NodeGains[v][r];
                    currentGain = removalGain - RelocateCostLowerBound(v, r);
                    
                    if (bestGain < currentGain)
                    {
                        // Only check validity if required.
                        validRemoval = _input.Vehicles[v].IsOkForRangeBounds(routeEval - removalGain) &&
                                     _solution[v].IsValidRemoval(_input, r, 1);
                    }
                }
                else
                {
                    // Pickup job
                    Debug.Assert(currentJob.Type == JobType.Pickup);
                    var deliveryRank = _solutionState.MatchingDeliveryRank[v][r];
                    var removalGain = _solutionState.PdGains[v][_solutionState.MatchingDeliveryRank[v].Keys.ToList().IndexOf(r)];
                    currentGain = removalGain - RelocateCostLowerBound(v, r, deliveryRank);
                    
                    if (bestGain < currentGain &&
                        _input.Vehicles[v].IsOkForRangeBounds(routeEval - removalGain))
                    {
                        // Only check validity if required.
                        if (deliveryRank == r + 1)
                        {
                            validRemoval = _solution[v].IsValidRemoval(_input, r, 2);
                        }
                        else
                        {
                            var betweenPd = _solution[v].Route.GetRange(r + 1, deliveryRank - r - 1);
                            var deliveryBetweenPd = _solution[v].DeliveryInRange(r + 1, deliveryRank);
                            
                            validRemoval = _solution[v].IsValidAdditionForTw(
                                _input, deliveryBetweenPd, betweenPd.ToArray(), 0, betweenPd.Count, r, deliveryRank + 1);
                        }
                    }
                }
                
                if (bestGain < currentGain && validRemoval)
                {
                    bestGain = currentGain;
                    bestRank = r;
                }
            }
            
            if (bestGain != Eval.NoGain)
            {
                routesAndRanks.Add((v, bestRank));
            }
        }
        
        // Execute removals
        foreach (var (v, r) in routesAndRanks)
        {
            _solutionState.Unassigned.Add(_solution[v].Route[r]);
            
            var currentJob = _input.Jobs[_solution[v].Route[r]];
            if (currentJob.Type == JobType.Single)
            {
                _solution[v].Remove(_input, r, 1);
            }
            else
            {
                Debug.Assert(currentJob.Type == JobType.Pickup);
                var deliveryRank = _solutionState.MatchingDeliveryRank[v][r];
                _solutionState.Unassigned.Add(_solution[v].Route[deliveryRank]);
                
                if (deliveryRank == r + 1)
                {
                    _solution[v].Remove(_input, r, 2);
                }
                else
                {
                    var betweenPd = _solution[v].Route.GetRange(r + 1, deliveryRank - r - 1).ToArray();
                    var deliveryBetweenPd = _solution[v].DeliveryInRange(r + 1, deliveryRank);
                    
                    _solution[v].Replace(_input, deliveryBetweenPd, betweenPd, 0, betweenPd.Length, r, deliveryRank + 1);
                }
            }
        }
    }
    
    private Eval RelocateCostLowerBound(int v, int r)
    {
        var bestBound = Eval.NoEval;
        
        for (int otherV = 0; otherV < _solution.Count; otherV++)
        {
            if (otherV == v || !_input.VehicleOkWithJob(otherV, _solution[v].Route[r]))
                continue;
            
            var cost = JobRouteCost(otherV, v, r);
            if (cost.Cost < bestBound.Cost)
                bestBound = cost;
        }
        
        return bestBound;
    }
    
    private Eval RelocateCostLowerBound(int v, int r1, int r2)
    {
        var bestBound = Eval.NoEval;
        
        for (int otherV = 0; otherV < _solution.Count; otherV++)
        {
            if (otherV == v || !_input.VehicleOkWithJob(otherV, _solution[v].Route[r1]))
                continue;
            
            var cost = JobRouteCost(otherV, v, r1) + JobRouteCost(otherV, v, r2);
            if (cost.Cost < bestBound.Cost)
                bestBound = cost;
        }
        
        return bestBound;
    }
    
    private Eval JobRouteCost(int vTarget, int v, int r)
    {
        Debug.Assert(v != vTarget);
        
        var eval = Eval.NoEval;
        var jobIndex = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[_solution[v].Route[r]].Location);
        
        var vehicle = _input.Vehicles[vTarget];
        if (vehicle.StartLocation != null)
        {
            var startIndex = Array.IndexOf(_input.Locations.ToArray(), vehicle.StartLocation);
            var startEval = _input.GetEval(vTarget, startIndex, jobIndex);
            if (startEval.Cost < eval.Cost)
                eval = startEval;
        }
        if (vehicle.EndLocation != null)
        {
            var endIndex = Array.IndexOf(_input.Locations.ToArray(), vehicle.EndLocation);
            var endEval = _input.GetEval(vTarget, jobIndex, endIndex);
            if (endEval.Cost < eval.Cost)
                eval = endEval;
        }
        if (_solution[vTarget].Size != 0)
        {
            var cheapestFromRank = _solutionState.CheapestJobRankInRoutesFrom[v][vTarget][r];
            var cheapestFromIndex = Array.IndexOf(_input.Locations.ToArray(), 
                _input.Jobs[_solution[vTarget].Route[cheapestFromRank]].Location);
            var evalFrom = _input.GetEval(vTarget, cheapestFromIndex, jobIndex);
            if (evalFrom.Cost < eval.Cost)
                eval = evalFrom;
            
            var cheapestToRank = _solutionState.CheapestJobRankInRoutesTo[v][vTarget][r];
            var cheapestToIndex = Array.IndexOf(_input.Locations.ToArray(), 
                _input.Jobs[_solution[vTarget].Route[cheapestToRank]].Location);
            var evalTo = _input.GetEval(vTarget, jobIndex, cheapestToIndex);
            if (evalTo.Cost < eval.Cost)
                eval = evalTo;
        }
        
        return eval;
    }
    
    /// <summary>
    /// Try all unassigned exchange moves.
    /// </summary>
    private bool TryUnassignedExchangeOperators()
    {
        var bestGain = Eval.NoEval;
        UnassignedExchange? bestMove = null;
        
        foreach (var unassignedJob in _solutionState.Unassigned)
        {
            // Skip delivery jobs
            if (_input.Jobs[unassignedJob].Type == JobType.Delivery)
                continue;
                
            for (int v = 0; v < _nbVehicles; v++)
            {
                var route = _solution[v];
                if (route.Route.Count == 0) continue;
                
                // Check if vehicle is compatible with unassigned job
                if (!_input.VehicleOkWithJob(v, unassignedJob))
                    continue;
                    
                for (int sourceRank = 0; sourceRank < route.Route.Count; sourceRank++)
                {
                    // Check if we can exchange with this job
                    var sourceJob = route.Route[sourceRank];
                    if (_input.Jobs[sourceJob].Type != JobType.Single)
                        continue;
                        
                    for (int targetRank = 0; targetRank <= route.Route.Count; targetRank++)
                    {
                        if (targetRank == sourceRank || targetRank == sourceRank + 1)
                            continue;
                            
                        try
                        {
                            var move = new UnassignedExchange(_input, _solutionState, _solutionState.Unassigned,
                                                            route, v, sourceRank, targetRank, unassignedJob);
                            
                            if (move.IsValid())
                            {
                                var gain = move.Gain();
                                if (gain.Cost > bestGain.Cost)
                                {
                                    bestGain = gain;
                                    bestMove = move;
                                }
                            }
                        }
                        catch
                        {
                            // Invalid move configuration
                        }
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all reverse two-opt moves.
    /// </summary>
    private bool TryReverseTwoOptOperators()
    {
        var bestGain = Eval.NoEval;
        ReverseTwoOpt? bestMove = null;
        
        for (int s = 0; s < _nbVehicles; s++)
        {
            var sourceRoute = _solution[s];
            if (sourceRoute.Route.Count == 0) continue;
            
            for (int sRank = 0; sRank < sourceRoute.Route.Count; sRank++)
            {
                for (int t = s + 1; t < _nbVehicles; t++)
                {
                    var targetRoute = _solution[t];
                    if (targetRoute.Route.Count == 0) continue;
                    
                    for (int tRank = 0; tRank < targetRoute.Route.Count; tRank++)
                    {
                        try
                        {
                            var move = new ReverseTwoOpt(_input, _solutionState, sourceRoute, s, sRank,
                                                       targetRoute, t, tRank);
                            
                            if (move.IsValid())
                            {
                                var gain = move.Gain();
                                if (gain.Cost > bestGain.Cost)
                                {
                                    bestGain = gain;
                                    bestMove = move;
                                }
                            }
                        }
                        catch
                        {
                            // Invalid move configuration
                        }
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all PD shift moves.
    /// </summary>
    private bool TryPDShiftOperators()
    {
        var bestGain = Eval.NoEval;
        PDShift? bestMove = null;
        
        for (int s = 0; s < _nbVehicles; s++)
        {
            var sourceRoute = _solution[s];
            if (sourceRoute.Route.Count < 2) continue;
            
            // Find pickup-delivery pairs
            foreach (var pickupRank in _solutionState.MatchingDeliveryRank[s].Keys)
            {
                var deliveryRank = _solutionState.MatchingDeliveryRank[s][pickupRank];
                
                for (int t = 0; t < _nbVehicles; t++)
                {
                    if (s == t) continue;
                    
                    var targetRoute = _solution[t];
                    
                    // Check if target vehicle can handle the pair
                    if (!_input.VehicleOkWithJob(t, sourceRoute.Route[pickupRank]))
                        continue;
                        
                    for (int tRank = 0; tRank <= targetRoute.Route.Count; tRank++)
                    {
                        try
                        {
                            var move = new PDShift(_input, _solutionState, sourceRoute, s, pickupRank, deliveryRank,
                                                 targetRoute, t, bestGain);
                            
                            if (move.IsValid())
                            {
                                var gain = move.Gain();
                                if (gain.Cost > bestGain.Cost)
                                {
                                    bestGain = gain;
                                    bestMove = move;
                                }
                            }
                        }
                        catch
                        {
                            // Invalid move configuration
                        }
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all route split moves.
    /// </summary>
    private bool TryRouteSplitOperators()
    {
        var bestGain = Eval.NoEval;
        RouteSplit? bestMove = null;
        
        // Find empty routes
        var emptyRoutes = new List<int>();
        for (int v = 0; v < _nbVehicles; v++)
        {
            if (_solution[v].Route.Count == 0)
            {
                emptyRoutes.Add(v);
            }
        }
        
        if (emptyRoutes.Count == 0)
            return false;
            
        for (int s = 0; s < _nbVehicles; s++)
        {
            var sourceRoute = _solution[s];
            if (sourceRoute.Route.Count < 2) continue;
            
            try
            {
                var move = new RouteSplit(_input, _solutionState, sourceRoute, s, 
                                        emptyRoutes, _solution, bestGain);
                
                if (move.IsValid())
                {
                    var gain = move.Gain();
                    if (gain.Cost > bestGain.Cost)
                    {
                        bestGain = gain;
                        bestMove = move;
                    }
                }
            }
            catch
            {
                // Invalid move configuration
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all swap star moves.
    /// </summary>
    private bool TrySwapStarOperators()
    {
        var bestGain = Eval.NoEval;
        SwapStar? bestMove = null;
        
        for (int s = 0; s < _nbVehicles; s++)
        {
            var sourceRoute = _solution[s];
            if (sourceRoute.Route.Count < 1) continue;
            
            for (int t = 0; t < _nbVehicles; t++)
            {
                if (s == t) continue;
                
                var targetRoute = _solution[t];
                if (targetRoute.Route.Count < 1) continue;
                
                try
                {
                    var move = new SwapStar(_input, _solutionState, sourceRoute, s, 
                                          targetRoute, t, bestGain);
                    
                    if (move.IsValid())
                    {
                        var gain = move.Gain();
                        if (gain.Cost > bestGain.Cost)
                        {
                            bestGain = gain;
                            bestMove = move;
                        }
                    }
                }
                catch
                {
                    // Invalid move configuration
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Try all priority replace moves.
    /// </summary>
    private bool TryPriorityReplaceOperators()
    {
        var bestGain = Eval.NoEval;
        PriorityReplace? bestMove = null;
        
        // Find unassigned jobs with high priority
        var highPriorityUnassigned = new List<int>();
        foreach (var j in _solutionState.Unassigned)
        {
            if (_input.Jobs[j].Priority > 0 && _input.Jobs[j].Type != JobType.Delivery)
            {
                highPriorityUnassigned.Add(j);
            }
        }
        
        if (highPriorityUnassigned.Count == 0)
            return false;
            
        foreach (var unassignedJob in highPriorityUnassigned)
        {
            for (int v = 0; v < _nbVehicles; v++)
            {
                var route = _solution[v];
                if (route.Route.Count == 0) continue;
                
                // Check if vehicle is compatible
                if (!_input.VehicleOkWithJob(v, unassignedJob))
                    continue;
                    
                // Find low priority jobs to replace
                for (int sRank = 0; sRank < route.Route.Count; sRank++)
                {
                    var currentJob = route.Route[sRank];
                    if (_input.Jobs[currentJob].Priority >= _input.Jobs[unassignedJob].Priority)
                        continue;
                        
                    for (int tRank = 0; tRank <= route.Route.Count; tRank++)
                    {
                        try
                        {
                            // Calculate priority gain
                            int priorityGain = _input.Jobs[unassignedJob].Priority - _input.Jobs[currentJob].Priority;
                            
                            var move = new PriorityReplace(_input, _solutionState, _solutionState.Unassigned,
                                                         route, v, sRank, tRank, unassignedJob, priorityGain);
                            
                            if (move.IsValid())
                            {
                                var gain = move.Gain();
                                if (gain.Cost > bestGain.Cost)
                                {
                                    bestGain = gain;
                                    bestMove = move;
                                }
                            }
                        }
                        catch
                        {
                            // Invalid move configuration
                        }
                    }
                }
            }
        }
        
        if (bestMove != null && bestGain.Cost > 0)
        {
            bestMove.Apply();
            
            foreach (var v in bestMove.UpdateCandidates())
            {
                _solutionState.UpdateRoute(v, _solution[v]);
            }
            
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Compute best insertion position for a job in a route.
    /// </summary>
    private RouteInsertion ComputeBestInsertion(int jobRank, int vehicleRank, RawRoute route)
    {
        var currentJob = _input.Jobs[jobRank];
        
        if (currentJob.Type == JobType.Single)
        {
            return ComputeBestInsertionSingle(jobRank, vehicleRank, route);
        }
        else if (currentJob.Type == JobType.Pickup)
        {
            return ComputeBestInsertionPd(jobRank, vehicleRank, route, Eval.NoEval);
        }
        
        return new RouteInsertion(_input.AmountSize);
    }
    
    /// <summary>
    /// Compute best insertion for a single job.
    /// </summary>
    private RouteInsertion ComputeBestInsertionSingle(int j, int v, RawRoute route)
    {
        var result = new RouteInsertion(_input.AmountSize);
        var currentJob = _input.Jobs[j];
        var vehicle = _input.Vehicles[v];
        
        if (!_input.VehicleOkWithJob(v, j))
            return result;
            
        for (int rank = 0; rank <= route.Route.Count; rank++)
        {
            var currentEval = Helpers.AdditionCost(_input, j, vehicle, route.Route, rank);
            
            if (currentEval.Cost < result.Eval.Cost &&
                vehicle.IsOkForRangeBounds(_solutionState.RouteEvals[v] + currentEval) &&
                route.IsValidAdditionForCapacity(currentJob.Pickup, currentJob.Delivery, rank) &&
                route.IsValidAdditionForTw(_input, j, rank))
            {
                result.Eval = currentEval;
                result.Delivery = currentJob.Delivery;
                result.SingleRank = rank;
            }
        }
        
        return result;
    }
    
    /// <summary>
    /// Compute best insertion for a pickup-delivery pair.
    /// </summary>
    private RouteInsertion ComputeBestInsertionPd(int j, int v, RawRoute route, Eval costThreshold)
    {
        var result = new RouteInsertion(_input.AmountSize);
        var currentJob = _input.Jobs[j];
        var vehicle = _input.Vehicles[v];
        
        if (!_input.VehicleOkWithJob(v, j))
            return result;
            
        result.Eval = costThreshold;
        
        // Pre-compute delivery costs
        var dAdds = new List<Eval>(route.Route.Count + 1);
        var validDeliveryInsertions = new List<bool>(route.Route.Count + 1);
        
        bool foundValid = false;
        for (int dRank = 0; dRank <= route.Route.Count; dRank++)
        {
            dAdds.Add(Helpers.AdditionCost(_input, j + 1, vehicle, route.Route, dRank));
            
            if (result.Eval != Eval.NoEval && result.Eval < dAdds[dRank])
            {
                validDeliveryInsertions.Add(false);
            }
            else
            {
                validDeliveryInsertions.Add(route.IsValidAdditionForTwWithoutMaxLoad(_input, j + 1, dRank));
            }
            foundValid |= validDeliveryInsertions[dRank];
        }
        
        if (!foundValid)
        {
            result.Eval = Eval.NoEval;
            return result;
        }
        
        // Try all pickup positions
        for (int pickupR = 0; pickupR <= route.Route.Count; pickupR++)
        {
            var pAdd = Helpers.AdditionCost(_input, j, vehicle, route.Route, pickupR);
            
            if (result.Eval != Eval.NoEval && result.Eval < pAdd)
                continue;
                
            if (!route.IsValidAdditionForLoad(_input, currentJob.Pickup, pickupR) ||
                !route.IsValidAdditionForTwWithoutMaxLoad(_input, j, pickupR))
                continue;
                
            // Build replacement sequence
            var modifiedWithPd = new List<int>();
            modifiedWithPd.Add(j);
            
            var modifiedDelivery = _input.ZeroAmount;
            
            for (int deliveryR = pickupR; deliveryR <= route.Route.Count; deliveryR++)
            {
                if (pickupR < deliveryR)
                {
                    modifiedWithPd.Add(route.Route[deliveryR - 1]);
                    var newModifiedJob = _input.Jobs[route.Route[deliveryR - 1]];
                    if (newModifiedJob.Type == JobType.Single)
                    {
                        modifiedDelivery = modifiedDelivery + newModifiedJob.Delivery;
                    }
                }
                
                if (!validDeliveryInsertions[deliveryR])
                    continue;
                    
                Eval pdEval;
                if (pickupR == deliveryR)
                {
                    pdEval = Helpers.AdditionCost(_input, j, vehicle, route.Route, pickupR, pickupR + 1);
                }
                else
                {
                    pdEval = pAdd + dAdds[deliveryR];
                }
                
                if ((result.Eval == Eval.NoEval || pdEval < result.Eval) &&
                    vehicle.IsOkForRangeBounds(_solutionState.RouteEvals[v] + pdEval))
                {
                    modifiedWithPd.Add(j + 1);
                    
                    // Check validity
                    bool isValid = ValidForCapacity(route, modifiedWithPd, pickupR, deliveryR);
                    
                    isValid = isValid && route.IsValidAdditionForTw(_input,
                                                                   modifiedDelivery,
                                                                   modifiedWithPd.ToArray(),
                                                                   0,
                                                                   modifiedWithPd.Count,
                                                                   pickupR,
                                                                   deliveryR);
                    
                    modifiedWithPd.RemoveAt(modifiedWithPd.Count - 1);
                    
                    if (isValid)
                    {
                        result.Eval = pdEval;
                        result.Delivery = modifiedDelivery;
                        result.PickupRank = pickupR;
                        result.DeliveryRank = deliveryR;
                    }
                }
            }
        }
        
        if (result.Eval == costThreshold)
        {
            result.Eval = Eval.NoEval;
        }
        
        return result;
    }
    
    /// <summary>
    /// Check if insertion is valid for capacity.
    /// </summary>
    private bool ValidForCapacity(RawRoute route, List<int> modifiedWithPd, int pickupR, int deliveryR)
    {
        var amount = _input.ZeroAmount;
        
        for (int i = 1; i < modifiedWithPd.Count - 1; i++)
        {
            var newModifiedJob = _input.Jobs[modifiedWithPd[i]];
            if (newModifiedJob.Type == JobType.Single)
            {
                amount = amount + newModifiedJob.Delivery;
            }
        }
        
        return route.IsValidAdditionForCapacityInclusion(amount,
                                                         modifiedWithPd.ToArray(),
                                                         0,
                                                         modifiedWithPd.Count,
                                                         pickupR,
                                                         deliveryR);
    }
}