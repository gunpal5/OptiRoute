using OptiRoute.Core.Models;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// ReverseTwoOpt operator - swaps route tails with reversal (based on VROOM).
/// </summary>
internal class ReverseTwoOpt : Operator
{
    private readonly Amount _sDelivery;
    private readonly Amount _tDelivery;

    public ReverseTwoOpt(
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        int sourceRank,
        RawRoute targetRoute,
        int targetVehicle,
        int targetRank)
        : base(OperatorName.ReverseTwoOpt, input, solutionState, sourceRoute, sourceVehicle,
               sourceRank, targetRoute, targetVehicle, targetRank)
    {
        if (sourceVehicle == targetVehicle)
            throw new ArgumentException("Source and target vehicles must be different");
        if (sourceRoute.Empty)
            throw new ArgumentException("Source route cannot be empty");
        if (targetRoute.Empty)
            throw new ArgumentException("Target route cannot be empty");
        if (sourceRank >= sourceRoute.Route.Count)
            throw new ArgumentException("Source rank out of bounds");
        if (targetRank >= targetRoute.Route.Count)
            throw new ArgumentException("Target rank out of bounds");
            
        _sDelivery = sourceRoute.BwdDeliveries(sourceRank);
        _tDelivery = targetRoute.FwdDeliveries(targetRank);
        
        // Check skill compatibility
        if (solutionState.BwdSkillRank[sourceVehicle][targetVehicle] > sourceRank + 1)
            throw new ArgumentException("Skill incompatibility in source route");
        if (targetRank >= solutionState.FwdSkillRank[targetVehicle][sourceVehicle])
            throw new ArgumentException("Skill incompatibility in target route");
    }

    protected override void ComputeGain()
    {
        // Gain from replacing source tail with reversed target head
        SourceGain = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Source,
            SourceRank + 1,
            SourceRoute.Count,
            Target,
            0,
            TargetRank + 1).Item2; // Use reversed gain
            
        // Gain from replacing target head with reversed source tail
        TargetGain = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Target,
            0,
            TargetRank + 1,
            Source,
            SourceRank + 1,
            SourceRoute.Count).Item2; // Use reversed gain
            
        StoredGain = SourceGain + TargetGain;
        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!GainComputed)
            ComputeGain();
            
        var tPickup = Target.FwdPickups(TargetRank);
        var sPickup = Source.BwdPickups(SourceRank);
        
        // Check range bounds
        if (!IsValidForSourceRangeBounds() || !IsValidForTargetRangeBounds())
            return false;
            
        // Check capacity margins
        if (!Source.IsValidAdditionForCapacityMargins(
                tPickup, _tDelivery, SourceRank + 1, SourceRoute.Count))
            return false;
            
        if (!Target.IsValidAdditionForCapacityMargins(
                sPickup, _sDelivery, 0, TargetRank + 1))
            return false;
            
        // Check capacity inclusion for reversed sequences
        // For source: check if we can add reversed target head after source rank
        // Create array of target jobs in reverse order (0 to t_rank)
        var reversedTargetJobs = new int[TargetRank + 1];
        for (int i = 0; i <= TargetRank; i++)
        {
            reversedTargetJobs[i] = TargetRoute[TargetRank - i];
        }
        
        if (!Source.IsValidAdditionForCapacityInclusion(
                _tDelivery,
                reversedTargetJobs,
                0,
                reversedTargetJobs.Length,
                SourceRank + 1,
                SourceRoute.Count))
            return false;
            
        // For target: check if we can add reversed source tail at beginning
        // Create array of source jobs in reverse order (s_rank+1 to end)
        var reversedSourceJobs = new int[SourceRoute.Count - SourceRank - 1];
        for (int i = 0; i < reversedSourceJobs.Length; i++)
        {
            reversedSourceJobs[i] = SourceRoute[SourceRoute.Count - 1 - i];
        }
        
        if (!Target.IsValidAdditionForCapacityInclusion(
                _sDelivery,
                reversedSourceJobs,
                0,
                reversedSourceJobs.Length,
                0,
                TargetRank + 1))
            return false;
            
        return true;
    }

    public override void Apply()
    {
        var nbSource = SourceRoute.Count - 1 - SourceRank;
        
        // Insert reversed source tail at beginning of target
        var reversedSourceTail = new List<int>();
        for (int i = SourceRoute.Count - 1; i > SourceRank; i--)
        {
            reversedSourceTail.Add(SourceRoute[i]);
        }
        TargetRoute.InsertRange(0, reversedSourceTail);
        
        // Remove source tail
        SourceRoute.RemoveRange(SourceRank + 1, nbSource);
        
        // Add reversed target head to source
        var reversedTargetHead = new List<int>();
        for (int i = TargetRank + nbSource; i >= nbSource; i--)
        {
            reversedTargetHead.Add(TargetRoute[i]);
        }
        SourceRoute.AddRange(reversedTargetHead);
        
        // Remove original target head (accounting for inserted source jobs)
        TargetRoute.RemoveRange(nbSource, TargetRank + 1);
        
        Source.UpdateAmounts();
        Target.UpdateAmounts();
    }

    public override List<int> AdditionCandidates()
    {
        return new List<int> { SourceVehicle, TargetVehicle };
    }

    public override List<int> UpdateCandidates()
    {
        return new List<int> { SourceVehicle, TargetVehicle };
    }
}