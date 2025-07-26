using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.VRP;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// MixedExchange operator - exchanges a single job from route 1 with an edge (2 jobs) from route 2 (based on VROOM).
/// </summary>
internal class MixedExchange : Operator
{
    private bool _gainUpperBoundComputed = false;
    private Eval _normalTGain;
    private Eval _reversedTGain = Eval.NoEval;
    
    protected bool ReverseTEdge { get; set; }
    protected readonly bool CheckTReverse;
    protected bool TIsNormalValid { get; set; }
    protected bool TIsReverseValid { get; set; }
    
    protected readonly Amount SourceDelivery;
    protected readonly Amount TargetDelivery;

    public MixedExchange(
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        int sourceRank,
        RawRoute targetRoute,
        int targetVehicle,
        int targetRank,
        bool checkTReverse)
        : base(OperatorName.MixedExchange, input, solutionState, sourceRoute, sourceVehicle,
               sourceRank, targetRoute, targetVehicle, targetRank)
    {
        CheckTReverse = checkTReverse;
        
        if (sourceVehicle == targetVehicle)
            throw new ArgumentException("Source and target vehicles must be different");
        if (sourceRoute.Route.Count < 1)
            throw new ArgumentException("Source route must have at least 1 job");
        if (targetRoute.Route.Count < 2)
            throw new ArgumentException("Target route must have at least 2 jobs");
        if (sourceRank >= sourceRoute.Route.Count)
            throw new ArgumentException("Source rank out of bounds");
        if (targetRank >= targetRoute.Route.Count - 1)
            throw new ArgumentException("Target rank out of bounds");
            
        // Check vehicle compatibility
        if (!input.VehicleOkWithJob(targetVehicle, sourceRoute.Route[sourceRank]) ||
            !input.VehicleOkWithJob(sourceVehicle, targetRoute.Route[targetRank]) ||
            !input.VehicleOkWithJob(sourceVehicle, targetRoute.Route[targetRank + 1]))
        {
            throw new ArgumentException("Vehicle incompatible with jobs");
        }
        
        // Either moving edge with single jobs or a whole shipment.
        var tJob1 = input.Jobs[targetRoute.Route[targetRank]];
        var tJob2 = input.Jobs[targetRoute.Route[targetRank + 1]];
        if (!((tJob1.Type == JobType.Single && tJob2.Type == JobType.Single && checkTReverse) ||
              (tJob1.Type == JobType.Pickup && tJob2.Type == JobType.Delivery && !checkTReverse &&
               solutionState.MatchingDeliveryRank[targetVehicle][targetRank] == targetRank + 1)))
        {
            throw new ArgumentException("Invalid job types for target edge");
        }
        
        // Compute deliveries
        SourceDelivery = input.GetJobDelivery(sourceRoute.Route[sourceRank]);
        TargetDelivery = input.GetJobDelivery(targetRoute.Route[targetRank]) +
                        input.GetJobDelivery(targetRoute.Route[targetRank + 1]);
    }

    /// <summary>
    /// Compute and store all possible costs depending on whether target edge is reversed.
    /// </summary>
    public Eval GainUpperBound()
    {
        // Match VROOM's implementation exactly
        // Calculate cost of inserting target edge into source route (replacing single job)
        (_normalTGain, _reversedTGain) = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Source,
            SourceRank,
            SourceRank + 1,
            Target,
            TargetRank,
            TargetRank + 2);
            
        var sGainUpperBound = _normalTGain;
        
        if (CheckTReverse && _reversedTGain.Cost > sGainUpperBound.Cost)
        {
            sGainUpperBound = _reversedTGain;
        }
        
        // Calculate cost of inserting source job into target route (replacing edge)
        TargetGain = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Target,
            TargetRank,
            TargetRank + 2,
            Source,
            SourceRank,
            SourceRank + 1).Item1;
            
        _gainUpperBoundComputed = true;
        
        return sGainUpperBound + TargetGain;
    }

    protected override void ComputeGain()
    {
        if (!_gainUpperBoundComputed)
            throw new InvalidOperationException("GainUpperBound must be called before ComputeGain");
            
        if (!TIsNormalValid && !TIsReverseValid)
            throw new InvalidOperationException("No valid configuration");
            
        // Match VROOM's implementation
        StoredGain = new Eval(0, 0, 0);
        
        if (_normalTGain.Cost < _reversedTGain.Cost)
        {
            // Biggest potential gain is obtained when reversing edge
            if (TIsReverseValid)
            {
                StoredGain = StoredGain + _reversedTGain;
                ReverseTEdge = true;
            }
            else
            {
                StoredGain = StoredGain + _normalTGain;
            }
        }
        else
        {
            // Biggest potential gain is obtained when not reversing edge
            if (TIsNormalValid)
            {
                StoredGain = StoredGain + _normalTGain;
            }
            else
            {
                StoredGain = StoredGain + _reversedTGain;
                ReverseTEdge = true;
            }
        }
        
        StoredGain = StoredGain + TargetGain;
        
        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!_gainUpperBoundComputed)
            GainUpperBound();
            
        // Check capacity constraints
        var sourcePickup = Input.GetJobPickup(SourceRoute[SourceRank]);
        var targetPickup = Input.GetJobPickup(TargetRoute[TargetRank]) +
                          Input.GetJobPickup(TargetRoute[TargetRank + 1]);
                          
        // Target must handle single job from source
        bool valid = Target.IsValidAdditionForCapacityMargins(
            sourcePickup, SourceDelivery, TargetRank, TargetRank + 2);
            
        if (!valid) return false;
        
        // Source must handle edge from target  
        valid = Source.IsValidAdditionForCapacityMargins(
            targetPickup, TargetDelivery, SourceRank, SourceRank + 1);
            
        if (!valid) return false;
        
        // Check route bounds
        var sourceVehicle = Input.Vehicles[SourceVehicle];
        var targetVehicle = Input.Vehicles[TargetVehicle];
        var sourceEval = SolutionState.RouteEvals[SourceVehicle];
        var targetEval = SolutionState.RouteEvals[TargetVehicle];
        
        // Check target can handle the single job
        if (!targetVehicle.IsOkForRangeBounds(targetEval - TargetGain))
            return false;
            
        // Check source with normal edge insertion
        TIsNormalValid = sourceVehicle.IsOkForRangeBounds(sourceEval - SourceGain - _normalTGain) &&
                        Source.IsValidAdditionForCapacityInclusion(
                            TargetDelivery,
                            new[] { TargetRoute[TargetRank], TargetRoute[TargetRank + 1] },
                            0, 2, SourceRank, SourceRank + 1);
                            
        if (CheckTReverse)
        {
            // Check source with reversed edge insertion
            TIsReverseValid = sourceVehicle.IsOkForRangeBounds(sourceEval - SourceGain - _reversedTGain) &&
                             Source.IsValidAdditionForCapacityInclusion(
                                 TargetDelivery,
                                 new[] { TargetRoute[TargetRank + 1], TargetRoute[TargetRank] },
                                 0, 2, SourceRank, SourceRank + 1);
        }
        
        return TIsNormalValid || TIsReverseValid;
    }

    public override void Apply()
    {
        // Match VROOM's implementation exactly
        // Ensure we only reverse single jobs, not P&D pairs
        if (ReverseTEdge && 
            !(Input.Jobs[TargetRoute[TargetRank]].Type == JobType.Single &&
              Input.Jobs[TargetRoute[TargetRank + 1]].Type == JobType.Single))
        {
            throw new InvalidOperationException("Cannot reverse pickup-delivery pairs");
        }
        
        // Swap single job with first job of edge
        (SourceRoute[SourceRank], TargetRoute[TargetRank]) = (TargetRoute[TargetRank], SourceRoute[SourceRank]);
        
        // Insert second job of edge after the swapped job in source
        SourceRoute.Insert(SourceRank + 1, TargetRoute[TargetRank + 1]);
        
        // Remove the second job from target (now at position TargetRank + 1)
        TargetRoute.RemoveAt(TargetRank + 1);
        
        // If we need to reverse the edge in source route
        if (ReverseTEdge)
        {
            (SourceRoute[SourceRank], SourceRoute[SourceRank + 1]) = 
                (SourceRoute[SourceRank + 1], SourceRoute[SourceRank]);
        }
        
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