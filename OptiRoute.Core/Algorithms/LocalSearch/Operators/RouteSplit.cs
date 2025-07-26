using OptiRoute.Core.Models;
using static OptiRoute.Core.Algorithms.LocalSearch.RouteSplitUtils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// RouteSplit operator - splits a route into two routes using empty vehicles (based on VROOM).
/// </summary>
internal class RouteSplit : Operator
{
    private readonly Eval _bestKnownGain;
    private readonly List<int> _emptyRouteRanks;
    private readonly List<RawRoute> _solution;
    private int _beginRouteRank;
    private int _endRouteRank;
    private SplitChoice _choice = EmptyRouteSplitChoice;

    public RouteSplit(
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        List<int> emptyRouteRanks,
        List<RawRoute> solution,
        Eval bestKnownGain)
        : base(OperatorName.RouteSplit, input, solutionState, sourceRoute, sourceVehicle,
               0, sourceRoute, sourceVehicle, 0) // Dummy values for ranks
    {
        if (sourceRoute.Route.Count < 2)
            throw new ArgumentException("Source route must have at least 2 jobs");
        if (emptyRouteRanks.Count < 2)
            throw new ArgumentException("Need at least 2 empty routes");
            
        _bestKnownGain = bestKnownGain;
        _emptyRouteRanks = emptyRouteRanks;
        _solution = solution;
    }

    protected override void ComputeGain()
    {
        _choice = ComputeBestRouteSplitChoice(
            Input,
            SolutionState,
            SourceVehicle,
            Source,
            _emptyRouteRanks,
            _bestKnownGain);
            
        if (_choice.Gain.Cost > 0)
        {
            StoredGain = _choice.Gain;
            
            // Ranks in choice are relative to _emptyRouteRanks so we go
            // back to initial vehicle ranks in _solution
            _beginRouteRank = _emptyRouteRanks[_choice.VBegin];
            _endRouteRank = _emptyRouteRanks[_choice.VEnd];
        }
        
        GainComputed = true;
    }

    public override bool IsValid()
    {
        // Not supposed to be used in VROOM implementation
        throw new NotImplementedException("RouteSplit.IsValid() should not be called");
    }

    public override void Apply()
    {
        if (_choice.Gain == Eval.NoEval)
            throw new InvalidOperationException("Cannot apply route split with no gain");
            
        // Empty route holding the end of the split
        var endRoute = _solution[_endRouteRank];
        if (endRoute.Route.Count > 0)
            throw new InvalidOperationException("End route must be empty");
            
        // Move jobs from split rank to end to the end route
        for (int i = _choice.SplitRank; i < SourceRoute.Count; i++)
        {
            endRoute.Route.Add(SourceRoute[i]);
        }
        endRoute.UpdateAmounts();
        
        // Verify the max load matches expected
        var expectedEndMaxLoad = Source.SubRouteMaxLoadAfter(_choice.SplitRank);
        if (endRoute.MaxLoad() != expectedEndMaxLoad)
            throw new InvalidOperationException("End route max load mismatch");
            
        // Empty route holding the beginning of the split
        var beginRoute = _solution[_beginRouteRank];
        if (beginRoute.Route.Count > 0)
            throw new InvalidOperationException("Begin route must be empty");
            
        // Move jobs from start to split rank to the begin route
        for (int i = 0; i < _choice.SplitRank; i++)
        {
            beginRoute.Route.Add(SourceRoute[i]);
        }
        beginRoute.UpdateAmounts();
        
        // Verify the max load matches expected
        var expectedBeginMaxLoad = Source.SubRouteMaxLoadBefore(_choice.SplitRank);
        if (beginRoute.MaxLoad() != expectedBeginMaxLoad)
            throw new InvalidOperationException("Begin route max load mismatch");
            
        // Clear the source route
        SourceRoute.Clear();
        Source.UpdateAmounts();
    }

    public override List<int> AdditionCandidates()
    {
        return new List<int> { SourceVehicle, _beginRouteRank, _endRouteRank };
    }

    public override List<int> UpdateCandidates()
    {
        return new List<int> { SourceVehicle, _beginRouteRank, _endRouteRank };
    }

    /// <summary>
    /// Check if this operator is invalidated by changes to the given vehicle rank.
    /// </summary>
    public override bool InvalidatedBy(int rank)
    {
        if (_choice.Gain == Eval.NoEval)
            throw new InvalidOperationException("Cannot check invalidation with no gain");
        return rank == _beginRouteRank || rank == _endRouteRank;
    }
}