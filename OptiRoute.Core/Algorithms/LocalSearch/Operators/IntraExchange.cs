using OptiRoute.Core.Models;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// IntraExchange operator - exchanges two jobs within the same route (based on VROOM).
/// </summary>
internal class IntraExchange : Operator
{
    private readonly int[] _movedJobs;
    private readonly int _firstRank;
    private readonly int _lastRank;
    private readonly Amount _delivery;

    public IntraExchange(
        Input input,
        SolutionState solutionState,
        RawRoute route,
        int vehicle,
        int sourceRank,
        int targetRank)
        : base(OperatorName.IntraExchange, input, solutionState, route, vehicle,
               sourceRank, route, vehicle, targetRank)
    {
        // Assume s_rank < t_rank for symmetry reasons. Set aside cases
        // where t_rank = s_rank + 1, as the move is also an intra_relocate.
        if (targetRank == 0)
            throw new ArgumentException("Target rank must be greater than 0");
        if (sourceRank >= targetRank - 1)
            throw new ArgumentException("Source rank must be less than target rank - 1");
        if (route.Route.Count < 3)
            throw new ArgumentException("Route must have at least 3 jobs");
        if (targetRank >= route.Route.Count)
            throw new ArgumentException("Target rank out of bounds");
            
        _movedJobs = new int[targetRank - sourceRank + 1];
        _firstRank = sourceRank;
        _lastRank = targetRank + 1;
        _delivery = route.DeliveryInRange(_firstRank, _lastRank);
        
        // Copy jobs and swap first with last
        for (int i = 0; i < _movedJobs.Length; i++)
        {
            _movedJobs[i] = route.Route[_firstRank + i];
        }
        var temp = _movedJobs[0];
        _movedJobs[0] = _movedJobs[_movedJobs.Length - 1];
        _movedJobs[_movedJobs.Length - 1] = temp;
    }

    protected override void ComputeGain()
    {
        SourceGain = SolutionState.NodeGains[SourceVehicle][SourceRank] -
                     Helpers.InPlaceDeltaCost(
                         Input,
                         SourceRoute[TargetRank],
                         Input.Vehicles[SourceVehicle],
                         SourceRoute,
                         SourceRank);
                         
        TargetGain = SolutionState.NodeGains[SourceVehicle][TargetRank] -
                     Helpers.InPlaceDeltaCost(
                         Input,
                         SourceRoute[SourceRank],
                         Input.Vehicles[SourceVehicle],
                         SourceRoute,
                         TargetRank);
                         
        StoredGain = SourceGain + TargetGain;
        GainComputed = true;
    }

    public override bool IsValid()
    {
        return IsValidForRangeBounds() &&
               Source.IsValidAdditionForCapacityInclusion(
                   _delivery,
                   _movedJobs,
                   0,
                   _movedJobs.Length,
                   _firstRank,
                   _lastRank);
    }

    public override void Apply()
    {
        var temp = SourceRoute[SourceRank];
        SourceRoute[SourceRank] = TargetRoute[TargetRank];
        TargetRoute[TargetRank] = temp;
        
        Source.UpdateAmounts();
    }

    public override List<int> AdditionCandidates()
    {
        return new List<int>();
    }

    public override List<int> UpdateCandidates()
    {
        return new List<int> { SourceVehicle };
    }
}