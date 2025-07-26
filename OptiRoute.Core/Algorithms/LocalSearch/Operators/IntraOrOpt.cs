using OptiRoute.Core.Models;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// IntraOrOpt operator - moves an edge (2 consecutive jobs) within the same route with optional reversal (based on VROOM).
/// </summary>
internal class IntraOrOpt : Operator
{
    private bool _gainUpperBoundComputed = false;
    private Eval _normalTGain;
    private Eval _reversedTGain;
    
    protected bool ReverseEdge = false;
    protected bool IsNormalValid = false;
    protected bool IsReverseValid = false;
    private readonly bool _checkReverse;
    
    private readonly List<int> _movedJobs;
    private readonly int _firstRank;
    private readonly int _lastRank;
    private readonly Amount _delivery;
    private int _sEdgeFirst;
    private int _sEdgeLast;

    public IntraOrOpt(
        Input input,
        SolutionState solutionState,
        RawRoute route,
        int vehicle,
        int sourceRank,
        int targetRank,   // rank *after* removal
        bool checkReverse)
        : base(OperatorName.IntraOrOpt, input, solutionState, route, vehicle,
               sourceRank, route, vehicle, targetRank)
    {
        if (route.Route.Count < 4)
            throw new ArgumentException("Route must have at least 4 jobs");
        if (sourceRank >= route.Route.Count - 1)
            throw new ArgumentException("Source rank out of bounds");
        if (targetRank > route.Route.Count - 2)
            throw new ArgumentException("Target rank out of bounds");
        if (sourceRank == targetRank)
            throw new ArgumentException("Source and target ranks must be different");
            
        _checkReverse = checkReverse;
        
        // Match VROOM's calculation for moved jobs
        int movedCount = (sourceRank < targetRank) ? targetRank - sourceRank + 2 : sourceRank - targetRank + 2;
        _movedJobs = new List<int>(movedCount);
        
        _firstRank = Math.Min(sourceRank, targetRank);
        _lastRank = Math.Max(sourceRank, targetRank) + 2;
        _delivery = route.DeliveryInRange(_firstRank, _lastRank);
        
        // Either moving an edge of single jobs or a whole shipment
        var job1 = input.Jobs[route.Route[sourceRank]];
        var job2 = input.Jobs[route.Route[sourceRank + 1]];
        
        if (!((job1.Type == JobType.Single && job2.Type == JobType.Single && checkReverse) ||
              (job1.Type == JobType.Pickup && job2.Type == JobType.Delivery && !checkReverse &&
               solutionState.MatchingDeliveryRank[vehicle][sourceRank] == sourceRank + 1)))
        {
            throw new ArgumentException("Invalid job types for IntraOrOpt");
        }
        
        // Build moved jobs array matching VROOM
        if (targetRank < sourceRank)
        {
            _sEdgeFirst = 0;
            _sEdgeLast = 1;
            
            // Copy jobs from targetRank to sourceRank (exclusive)
            for (int i = targetRank; i < sourceRank; i++)
            {
                _movedJobs.Add(route.Route[i]);
            }
            // Add padding to reach expected size
            while (_movedJobs.Count < movedCount - 2)
            {
                _movedJobs.Add(0);
            }
        }
        else
        {
            _sEdgeFirst = movedCount - 2;
            _sEdgeLast = movedCount - 1;
            
            // Copy jobs from sourceRank+2 to targetRank+2 (exclusive)
            for (int i = sourceRank + 2; i < targetRank + 2 && i < route.Route.Count; i++)
            {
                _movedJobs.Add(route.Route[i]);
            }
            // Add padding to reach expected size
            while (_movedJobs.Count < movedCount - 2)
            {
                _movedJobs.Add(0);
            }
        }
        
        // Insert source edge at appropriate positions
        if (_sEdgeFirst < _movedJobs.Count)
            _movedJobs.Insert(_sEdgeFirst, route.Route[sourceRank]);
        else
            _movedJobs.Add(route.Route[sourceRank]);
            
        if (_sEdgeLast < _movedJobs.Count)
            _movedJobs.Insert(_sEdgeLast, route.Route[sourceRank + 1]);
        else
            _movedJobs.Add(route.Route[sourceRank + 1]);
            
        // Ensure we have exactly the expected size
        while (_movedJobs.Count > movedCount)
        {
            _movedJobs.RemoveAt(_movedJobs.Count - 1);
        }
    }

    /// <summary>
    /// Compute and store all possible costs depending on whether edge is reversed or not.
    /// </summary>
    public Eval GainUpperBound()
    {
        // For addition, consider the cost of adding source edge at new rank *after* removal
        var newRank = TargetRank + ((SourceRank < TargetRank) ? 2 : 0);
        
        SourceGain = Helpers.RemovalCostDelta(Input, SolutionState, Source, SourceRank, 2);
        
        var (normalGain, reversedGain) = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Target,
            newRank,
            newRank,
            Source,
            SourceRank,
            SourceRank + 2);
            
        _normalTGain = normalGain;
        _reversedTGain = reversedGain;
        
        _gainUpperBoundComputed = true;
        
        var tGainUpperBound = _normalTGain;
        if (_checkReverse && _reversedTGain.Cost > tGainUpperBound.Cost)
        {
            tGainUpperBound = _reversedTGain;
        }
        
        return SourceGain + tGainUpperBound;
    }

    protected override void ComputeGain()
    {
        if (!_gainUpperBoundComputed)
            throw new InvalidOperationException("GainUpperBound must be called before ComputeGain");
        
        if (!IsNormalValid && !IsReverseValid)
            throw new InvalidOperationException("At least one direction must be valid");

        StoredGain = SourceGain;

        if (_normalTGain.Cost < _reversedTGain.Cost)
        {
            // Biggest potential gain is obtained when reversing edge
            if (IsReverseValid)
            {
                ReverseEdge = true;
                StoredGain = StoredGain + _reversedTGain;
            }
            else
            {
                StoredGain = StoredGain + _normalTGain;
            }
        }
        else
        {
            // Biggest potential gain is obtained when not reversing edge
            if (IsNormalValid)
            {
                StoredGain = StoredGain + _normalTGain;
            }
            else
            {
                ReverseEdge = true;
                StoredGain = StoredGain + _reversedTGain;
            }
        }

        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!_gainUpperBoundComputed)
            throw new InvalidOperationException("GainUpperBound must be called first");

        var vehicle = Input.Vehicles[SourceVehicle];
        var routeEval = SolutionState.RouteEvals[SourceVehicle];
        var normalEval = SourceGain + _normalTGain;
        
        // Check normal direction
        IsNormalValid = vehicle.IsOkForRangeBounds(routeEval - normalEval) &&
                       Source.IsValidAdditionForCapacityInclusion(
                           _delivery,
                           _movedJobs.ToArray(),
                           0,
                           _movedJobs.Count,
                           _firstRank,
                           _lastRank);
        
        // Check reverse direction if requested
        if (_checkReverse)
        {
            var reversedEval = SourceGain + _reversedTGain;
            
            if (vehicle.IsOkForRangeBounds(routeEval - reversedEval))
            {
                // Temporarily swap the edge jobs to check validity
                (_movedJobs[_sEdgeFirst], _movedJobs[_sEdgeLast]) = (_movedJobs[_sEdgeLast], _movedJobs[_sEdgeFirst]);
                
                IsReverseValid = Source.IsValidAdditionForCapacityInclusion(
                    _delivery,
                    _movedJobs.ToArray(),
                    0,
                    _movedJobs.Count,
                    _firstRank,
                    _lastRank);
                
                // Reset to initial situation
                (_movedJobs[_sEdgeFirst], _movedJobs[_sEdgeLast]) = (_movedJobs[_sEdgeLast], _movedJobs[_sEdgeFirst]);
            }
        }
        
        return IsNormalValid || IsReverseValid;
    }

    public override void Apply()
    {
        // Ensure we only reverse single jobs, not P&D pairs
        if (ReverseEdge && 
            !(Input.Jobs[SourceRoute[SourceRank]].Type == JobType.Single &&
              Input.Jobs[SourceRoute[SourceRank + 1]].Type == JobType.Single))
        {
            throw new InvalidOperationException("Cannot reverse pickup-delivery pairs");
        }
        
        var firstJobRank = SourceRoute[SourceRank];
        var secondJobRank = SourceRoute[SourceRank + 1];
        
        // Remove the edge
        SourceRoute.RemoveRange(SourceRank, 2);
        
        // Insert at target position
        if (ReverseEdge)
        {
            TargetRoute.Insert(TargetRank, secondJobRank);
            TargetRoute.Insert(TargetRank + 1, firstJobRank);
        }
        else
        {
            TargetRoute.Insert(TargetRank, firstJobRank);
            TargetRoute.Insert(TargetRank + 1, secondJobRank);
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