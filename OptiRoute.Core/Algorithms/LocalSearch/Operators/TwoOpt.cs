using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.VRP;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// TwoOpt operator - exchanges portions of two routes by breaking and reconnecting them (based on VROOM).
/// </summary>
internal class TwoOpt : Operator
{
    private readonly Amount _sDelivery;
    private readonly Amount _tDelivery;
    
    public TwoOpt(
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        int sourceRank,
        RawRoute targetRoute,
        int targetVehicle,
        int targetRank)
        : base(OperatorName.TwoOpt, input, solutionState, sourceRoute, sourceVehicle,
               sourceRank, targetRoute, targetVehicle, targetRank)
    {
        if (sourceVehicle == targetVehicle)
            throw new ArgumentException("Source and target vehicles must be different");
        if (sourceRoute.Empty)
            throw new ArgumentException("Source route cannot be empty");
        if (targetRoute.Empty)
            throw new ArgumentException("Target route cannot be empty");
        if (sourceRank >= sourceRoute.Size)
            throw new ArgumentException("Source rank out of bounds");
        if (targetRank >= targetRoute.Size)
            throw new ArgumentException("Target rank out of bounds");

        _sDelivery = sourceRoute.bwd_deliveries(sourceRank);
        _tDelivery = targetRoute.bwd_deliveries(targetRank);

        // TODO: Check bwd_skill_rank when implemented
        // assert(_sol_state.bwd_skill_rank[s_vehicle][t_vehicle] <= s_rank + 1);
        // assert(_sol_state.bwd_skill_rank[t_vehicle][s_vehicle] <= t_rank + 1);
    }

    protected override void ComputeGain()
    {
        // Use addition_cost_delta to compute gain for swapping tails
        var (sCost, _) = Utils.Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Source,
            SourceRank + 1,
            SourceRoute.Count,
            Target,
            TargetRank + 1,
            TargetRoute.Count);

        var (tCost, _) = Utils.Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Target,
            TargetRank + 1,
            TargetRoute.Count,
            Source,
            SourceRank + 1,
            SourceRoute.Count);

        SourceGain = sCost;
        TargetGain = tCost;
        StoredGain = SourceGain + TargetGain;
        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!GainComputed)
            ComputeGain();

        var tPickup = Target.bwd_pickups(TargetRank);
        var sPickup = Source.bwd_pickups(SourceRank);

        // Match VROOM's is_valid exactly - need to check capacity inclusion too
        return IsValidForSourceRangeBounds() &&
               IsValidForTargetRangeBounds() &&
               Source.IsValidAdditionForCapacityMargins(
                   tPickup,
                   _tDelivery,
                   SourceRank + 1,
                   SourceRoute.Count) &&
               Target.IsValidAdditionForCapacityMargins(
                   sPickup,
                   _sDelivery,
                   TargetRank + 1,
                   TargetRoute.Count) &&
               Source.IsValidAdditionForCapacityInclusion(
                   _tDelivery,
                   TargetRoute.ToArray(),
                   TargetRank + 1,
                   TargetRoute.Count,
                   SourceRank + 1,
                   SourceRoute.Count) &&
               Target.IsValidAdditionForCapacityInclusion(
                   _sDelivery,
                   SourceRoute.ToArray(),
                   SourceRank + 1,
                   SourceRoute.Count,
                   TargetRank + 1,
                   TargetRoute.Count);
    }

    public override void Apply()
    {
        // Extract tails
        var sourceTail = new List<int>();
        for (int i = SourceRank + 1; i < SourceRoute.Count; i++)
        {
            sourceTail.Add(SourceRoute[i]);
        }

        var targetTail = new List<int>();
        for (int i = TargetRank + 1; i < TargetRoute.Count; i++)
        {
            targetTail.Add(TargetRoute[i]);
        }

        // Remove tails
        if (sourceTail.Count > 0)
            SourceRoute.RemoveRange(SourceRank + 1, sourceTail.Count);
        if (targetTail.Count > 0)
            TargetRoute.RemoveRange(TargetRank + 1, targetTail.Count);

        // Swap tails
        SourceRoute.AddRange(targetTail);
        TargetRoute.AddRange(sourceTail);

        Source.UpdateAmounts();
        Target.UpdateAmounts();
    }

    public override List<int> AdditionCandidates()
    {
        // Match VROOM - TwoOpt returns both vehicles as addition candidates
        return new List<int> { SourceVehicle, TargetVehicle };
    }

    public override List<int> UpdateCandidates()
    {
        return new List<int> { SourceVehicle, TargetVehicle };
    }
}