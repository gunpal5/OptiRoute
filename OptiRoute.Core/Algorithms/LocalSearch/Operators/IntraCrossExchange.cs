using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.VRP;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// IntraCrossExchange operator - exchanges two edges within the same route (based on VROOM).
/// </summary>
internal class IntraCrossExchange : Operator
{
    private bool _gainUpperBoundComputed = false;
    private Eval _normalSGain;
    private Eval _reversedSGain = Eval.NoEval;
    private Eval _normalTGain;
    private Eval _reversedTGain = Eval.NoEval;
    
    protected bool ReverseSEdge { get; set; }
    protected bool ReverseTEdge { get; set; }
    protected readonly bool CheckSReverse;
    protected readonly bool CheckTReverse;
    
    protected bool SNormalTNormalIsValid { get; set; }
    protected bool SNormalTReverseIsValid { get; set; }
    protected bool SReverseTReverseIsValid { get; set; }
    protected bool SReverseTNormalIsValid { get; set; }
    
    private readonly List<int> _movedJobs;
    private readonly int _firstRank;
    private readonly int _lastRank;
    private readonly Amount _delivery;

    public IntraCrossExchange(
        Input input,
        SolutionState solutionState,
        RawRoute route,
        int vehicle,
        int sourceRank,
        int targetRank,
        bool checkSReverse,
        bool checkTReverse)
        : base(OperatorName.IntraCrossExchange, input, solutionState, route, vehicle,
               sourceRank, route, vehicle, targetRank)
    {
        // Use sourceRank as smallest rank for symmetry reasons
        if (sourceRank + 2 >= targetRank)
            throw new ArgumentException("Source rank + 2 must be less than target rank to avoid common edge");
        if (route.Route.Count < 5)
            throw new ArgumentException("Route must have at least 5 jobs");
        if (targetRank >= route.Route.Count - 1)
            throw new ArgumentException("Target rank out of bounds");
            
        CheckSReverse = checkSReverse;
        CheckTReverse = checkTReverse;
        
        // Either moving edges of single jobs or whole shipments
        var sJob1 = input.Jobs[route.Route[sourceRank]];
        var sJob2 = input.Jobs[route.Route[sourceRank + 1]];
        var tJob1 = input.Jobs[route.Route[targetRank]];
        var tJob2 = input.Jobs[route.Route[targetRank + 1]];
        
        if (!((sJob1.Type == JobType.Single && sJob2.Type == JobType.Single && checkSReverse) ||
              (sJob1.Type == JobType.Pickup && sJob2.Type == JobType.Delivery && !checkSReverse &&
               solutionState.MatchingDeliveryRank[vehicle][sourceRank] == sourceRank + 1)))
        {
            throw new ArgumentException("Invalid source edge configuration");
        }
        
        if (!((tJob1.Type == JobType.Single && tJob2.Type == JobType.Single && checkTReverse) ||
              (tJob1.Type == JobType.Pickup && tJob2.Type == JobType.Delivery && !checkTReverse &&
               solutionState.MatchingDeliveryRank[vehicle][targetRank] == targetRank + 1)))
        {
            throw new ArgumentException("Invalid target edge configuration");
        }
        
        _movedJobs = new List<int>(targetRank - sourceRank + 2);
        _firstRank = sourceRank;
        _lastRank = targetRank + 2;
        _delivery = route.DeliveryInRange(_firstRank, _lastRank);
        
        // Initialize moved jobs array like VROOM
        _movedJobs.Add(route.Route[targetRank]);
        _movedJobs.Add(route.Route[targetRank + 1]);
        
        // Copy middle section
        for (int i = sourceRank + 2; i < targetRank; i++)
        {
            _movedJobs.Add(route.Route[i]);
        }
        
        _movedJobs.Add(route.Route[sourceRank]);
        _movedJobs.Add(route.Route[sourceRank + 1]);
    }

    /// <summary>
    /// Compute and store all possible costs depending on whether edges are reversed.
    /// </summary>
    public Eval GainUpperBound()
    {
        // Calculate cost of replacing source edge with target edge
        (_normalSGain, _reversedSGain) = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Source,
            SourceRank,
            SourceRank + 2,
            Source,
            TargetRank,
            TargetRank + 2);
            
        var sGainUpperBound = _normalSGain;
        
        if (CheckTReverse && _reversedSGain.Cost > sGainUpperBound.Cost)
        {
            sGainUpperBound = _reversedSGain;
        }
        
        // Calculate cost of replacing target edge with source edge
        (_normalTGain, _reversedTGain) = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Source,
            TargetRank,
            TargetRank + 2,
            Source,
            SourceRank,
            SourceRank + 2);
            
        var tGainUpperBound = _normalTGain;
        
        if (CheckSReverse && _reversedTGain.Cost > tGainUpperBound.Cost)
        {
            tGainUpperBound = _reversedTGain;
        }
        
        _gainUpperBoundComputed = true;
        
        return sGainUpperBound + tGainUpperBound;
    }

    protected override void ComputeGain()
    {
        if (!_gainUpperBoundComputed)
            throw new InvalidOperationException("GainUpperBound must be called before ComputeGain");
            
        if (!SNormalTNormalIsValid && !SNormalTReverseIsValid &&
            !SReverseTReverseIsValid && !SReverseTNormalIsValid)
        {
            throw new InvalidOperationException("No valid configuration");
        }
        
        StoredGain = Eval.NoEval;
        
        if (SNormalTNormalIsValid)
        {
            var currentGain = _normalSGain + _normalTGain;
            if (currentGain.Cost > StoredGain.Cost)
            {
                StoredGain = currentGain;
                ReverseSEdge = false;
                ReverseTEdge = false;
            }
        }
        
        if (SNormalTReverseIsValid)
        {
            var currentGain = _reversedSGain + _normalTGain;
            if (currentGain.Cost > StoredGain.Cost)
            {
                StoredGain = currentGain;
                ReverseSEdge = false;
                ReverseTEdge = true;
            }
        }
        
        if (SReverseTReverseIsValid)
        {
            var currentGain = _reversedSGain + _reversedTGain;
            if (currentGain.Cost > StoredGain.Cost)
            {
                StoredGain = currentGain;
                ReverseSEdge = true;
                ReverseTEdge = true;
            }
        }
        
        if (SReverseTNormalIsValid)
        {
            var currentGain = _normalSGain + _reversedTGain;
            if (currentGain.Cost > StoredGain.Cost)
            {
                StoredGain = currentGain;
                ReverseSEdge = true;
                ReverseTEdge = false;
            }
        }
        
        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!_gainUpperBoundComputed)
            GainUpperBound();
            
        var vehicle = Input.Vehicles[SourceVehicle];
        var routeEval = SolutionState.RouteEvals[SourceVehicle];
        var sNormalTNormalEval = _normalSGain + _normalTGain;
        
        // Check normal-normal configuration
        SNormalTNormalIsValid = vehicle.IsOkForRangeBounds(routeEval - sNormalTNormalEval) &&
                               Source.IsValidAdditionForCapacityInclusion(
                                   _delivery,
                                   _movedJobs.ToArray(),
                                   0,
                                   _movedJobs.Count,
                                   _firstRank,
                                   _lastRank);
        
        // Swap first edge to check normal-reverse
        (_movedJobs[0], _movedJobs[1]) = (_movedJobs[1], _movedJobs[0]);
        
        if (CheckTReverse)
        {
            var sNormalTReverseEval = _reversedSGain + _normalTGain;
            
            SNormalTReverseIsValid = vehicle.IsOkForRangeBounds(routeEval - sNormalTReverseEval) &&
                                    Source.IsValidAdditionForCapacityInclusion(
                                        _delivery,
                                        _movedJobs.ToArray(),
                                        0,
                                        _movedJobs.Count,
                                        _firstRank,
                                        _lastRank);
        }
        
        // Swap last edge to check reverse-reverse
        var lastIdx = _movedJobs.Count - 1;
        (_movedJobs[lastIdx - 1], _movedJobs[lastIdx]) = (_movedJobs[lastIdx], _movedJobs[lastIdx - 1]);
        
        if (CheckSReverse && CheckTReverse)
        {
            var sReverseTReverseEval = _reversedSGain + _reversedTGain;
            
            SReverseTReverseIsValid = vehicle.IsOkForRangeBounds(routeEval - sReverseTReverseEval) &&
                                     Source.IsValidAdditionForCapacityInclusion(
                                         _delivery,
                                         _movedJobs.ToArray(),
                                         0,
                                         _movedJobs.Count,
                                         _firstRank,
                                         _lastRank);
        }
        
        // Swap first edge back to check reverse-normal
        (_movedJobs[0], _movedJobs[1]) = (_movedJobs[1], _movedJobs[0]);
        
        if (CheckSReverse)
        {
            var sReverseTNormalEval = _normalSGain + _reversedTGain;
            
            SReverseTNormalIsValid = vehicle.IsOkForRangeBounds(routeEval - sReverseTNormalEval) &&
                                    Source.IsValidAdditionForCapacityInclusion(
                                        _delivery,
                                        _movedJobs.ToArray(),
                                        0,
                                        _movedJobs.Count,
                                        _firstRank,
                                        _lastRank);
        }
        
        // Reset to initial situation
        (_movedJobs[lastIdx - 1], _movedJobs[lastIdx]) = (_movedJobs[lastIdx], _movedJobs[lastIdx - 1]);
        
        return SNormalTNormalIsValid || SNormalTReverseIsValid ||
               SReverseTReverseIsValid || SReverseTNormalIsValid;
    }

    public override void Apply()
    {
        // Ensure we only reverse single jobs, not P&D pairs
        if (ReverseSEdge)
        {
            var job1 = Input.Jobs[SourceRoute[SourceRank]];
            var job2 = Input.Jobs[SourceRoute[SourceRank + 1]];
            if (job1.Type != JobType.Single || job2.Type != JobType.Single)
            {
                throw new InvalidOperationException("Cannot reverse pickup-delivery pairs");
            }
        }
        
        if (ReverseTEdge)
        {
            var job1 = Input.Jobs[SourceRoute[TargetRank]];
            var job2 = Input.Jobs[SourceRoute[TargetRank + 1]];
            if (job1.Type != JobType.Single || job2.Type != JobType.Single)
            {
                throw new InvalidOperationException("Cannot reverse pickup-delivery pairs");
            }
        }
        
        // Swap the edges
        (SourceRoute[SourceRank], SourceRoute[TargetRank]) = (SourceRoute[TargetRank], SourceRoute[SourceRank]);
        (SourceRoute[SourceRank + 1], SourceRoute[TargetRank + 1]) = (SourceRoute[TargetRank + 1], SourceRoute[SourceRank + 1]);
        
        // Reverse edges if needed
        if (ReverseSEdge)
        {
            (SourceRoute[TargetRank], SourceRoute[TargetRank + 1]) = (SourceRoute[TargetRank + 1], SourceRoute[TargetRank]);
        }
        
        if (ReverseTEdge)
        {
            (SourceRoute[SourceRank], SourceRoute[SourceRank + 1]) = (SourceRoute[SourceRank + 1], SourceRoute[SourceRank]);
        }
        
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