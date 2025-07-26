using OptiRoute.Core.Models;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// IntraMixedExchange operator - exchanges a single job with an edge within the same route (based on VROOM).
/// </summary>
internal class IntraMixedExchange : Operator
{
    private bool _gainUpperBoundComputed = false;
    private Eval _normalSGain;
    private Eval _reversedSGain = Eval.NoGain;
    
    protected bool ReverseTargetEdge = false;
    protected readonly bool CheckTargetReverse;
    
    protected bool SourceIsNormalValid = false;
    protected bool SourceIsReverseValid = false;
    
    private readonly int[] _movedJobs;
    private readonly int _firstRank;
    private readonly int _lastRank;
    private readonly Amount _delivery;
    private readonly int _tEdgeFirst;
    private readonly int _tEdgeLast;

    public IntraMixedExchange(
        Input input,
        SolutionState solutionState,
        RawRoute route,
        int vehicle,
        int sourceRank,
        int targetRank,
        bool checkTargetReverse)
        : base(OperatorName.IntraMixedExchange, input, solutionState, route, vehicle,
               sourceRank, route, vehicle, targetRank)
    {
        // If node at s_rank is right before/after edge at t_rank, then the
        // move is a relocate.
        if (!(sourceRank + 1 < targetRank || targetRank + 2 < sourceRank))
            throw new ArgumentException("Invalid ranks - would be a relocate operation");
        if (route.Route.Count < 4)
            throw new ArgumentException("Route must have at least 4 jobs");
        if (sourceRank >= route.Route.Count)
            throw new ArgumentException("Source rank out of bounds");
        if (targetRank >= route.Route.Count - 1)
            throw new ArgumentException("Target rank out of bounds");
            
        CheckTargetReverse = checkTargetReverse;
        
        // Either moving edge with single jobs or a whole shipment.
        var targetJob1 = input.Jobs[route.Route[targetRank]];
        var targetJob2 = input.Jobs[route.Route[targetRank + 1]];
        
        if (!((targetJob1.Type == JobType.Single && targetJob2.Type == JobType.Single && checkTargetReverse) ||
              (targetJob1.Type == JobType.Pickup && targetJob2.Type == JobType.Delivery && !checkTargetReverse &&
               solutionState.MatchingDeliveryRank[vehicle][targetRank] == targetRank + 1)))
        {
            throw new ArgumentException("Invalid job types for target edge");
        }
        
        _movedJobs = (sourceRank < targetRank) ? 
                     new int[targetRank - sourceRank + 2] : 
                     new int[sourceRank - targetRank + 1];
        _firstRank = Math.Min(sourceRank, targetRank);
        _lastRank = (targetRank < sourceRank) ? sourceRank + 1 : targetRank + 2;
        _delivery = route.DeliveryInRange(_firstRank, _lastRank);
        
        int sNode;
        if (targetRank < sourceRank)
        {
            sNode = 0;
            _tEdgeFirst = _movedJobs.Length - 2;
            _tEdgeLast = _movedJobs.Length - 1;
            
            // Copy jobs between target and source
            for (int i = 0; i < sourceRank - targetRank - 2; i++)
            {
                _movedJobs[i + 1] = route.Route[targetRank + 2 + i];
            }
        }
        else
        {
            _tEdgeFirst = 0;
            _tEdgeLast = 1;
            sNode = _movedJobs.Length - 1;
            
            // Copy jobs between source and target
            for (int i = 0; i < targetRank - sourceRank - 1; i++)
            {
                _movedJobs[i + 2] = route.Route[sourceRank + 1 + i];
            }
        }
        
        _movedJobs[sNode] = route.Route[sourceRank];
        _movedJobs[_tEdgeFirst] = route.Route[targetRank];
        _movedJobs[_tEdgeLast] = route.Route[targetRank + 1];
    }

    /// <summary>
    /// Compute and store all possible cost depending on whether edges
    /// are reversed or not. Return only an upper bound for gain as
    /// precise gain requires validity information.
    /// </summary>
    public Eval GainUpperBound()
    {
        var gains = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Source,
            SourceRank,
            SourceRank + 1,
            Target,
            TargetRank,
            TargetRank + 2);
            
        _normalSGain = gains.Item1;
        _reversedSGain = gains.Item2;
        
        var sGainUpperBound = _normalSGain;
        
        if (CheckTargetReverse)
        {
            if (_reversedSGain.Cost > sGainUpperBound.Cost)
                sGainUpperBound = _reversedSGain;
        }
        
        TargetGain = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Source,
            TargetRank,
            TargetRank + 2,
            Source,
            SourceRank,
            SourceRank + 1).Item1;
            
        _gainUpperBoundComputed = true;
        
        return sGainUpperBound + TargetGain;
    }

    public override Eval Gain()
    {
        if (!GainComputed)
        {
            ComputeGain();
        }
        return StoredGain;
    }
    
    protected override void ComputeGain()
    {
        if (!_gainUpperBoundComputed)
            throw new InvalidOperationException("Must call GainUpperBound() first");
        if (!SourceIsNormalValid && !SourceIsReverseValid)
            throw new InvalidOperationException("At least one configuration must be valid");
            
        StoredGain = new Eval(0, 0, 0);
        
        if (_normalSGain.Cost < _reversedSGain.Cost)
        {
            // Biggest potential gain is obtained when reversing edge.
            if (SourceIsReverseValid)
            {
                StoredGain = StoredGain + _reversedSGain;
                ReverseTargetEdge = true;
            }
            else
            {
                StoredGain = StoredGain + _normalSGain;
            }
        }
        else
        {
            // Biggest potential gain is obtained when not reversing edge.
            if (SourceIsNormalValid)
            {
                StoredGain = StoredGain + _normalSGain;
            }
            else
            {
                StoredGain = StoredGain + _reversedSGain;
                ReverseTargetEdge = true;
            }
        }
        
        StoredGain = StoredGain + TargetGain;
        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!_gainUpperBoundComputed)
            GainUpperBound();
            
        var vehicle = Input.Vehicles[SourceVehicle];
        var routeEval = SolutionState.RouteEvals[SourceVehicle];
        var normalEval = _normalSGain + TargetGain;
        
        SourceIsNormalValid = 
            vehicle.IsOkForRangeBounds(routeEval - normalEval) &&
            Source.IsValidAdditionForCapacityInclusion(
                _delivery,
                _movedJobs,
                0,
                _movedJobs.Length,
                _firstRank,
                _lastRank);
                
        if (CheckTargetReverse)
        {
            var reversedEval = _reversedSGain + TargetGain;
            
            if (vehicle.IsOkForRangeBounds(routeEval - reversedEval))
            {
                // Swap edge jobs temporarily
                var temp = _movedJobs[_tEdgeFirst];
                _movedJobs[_tEdgeFirst] = _movedJobs[_tEdgeLast];
                _movedJobs[_tEdgeLast] = temp;
                
                SourceIsReverseValid = 
                    Source.IsValidAdditionForCapacityInclusion(
                        _delivery,
                        _movedJobs,
                        0,
                        _movedJobs.Length,
                        _firstRank,
                        _lastRank);
                        
                // Reset to initial situation
                temp = _movedJobs[_tEdgeFirst];
                _movedJobs[_tEdgeFirst] = _movedJobs[_tEdgeLast];
                _movedJobs[_tEdgeLast] = temp;
            }
        }
        
        return SourceIsNormalValid || SourceIsReverseValid;
    }

    public override void Apply()
    {
        if (ReverseTargetEdge)
        {
            var job1 = Input.Jobs[TargetRoute[TargetRank]];
            var job2 = Input.Jobs[TargetRoute[TargetRank + 1]];
            if (job1.Type != JobType.Single || job2.Type != JobType.Single)
                throw new InvalidOperationException("Can only reverse single jobs");
                
            var temp = SourceRoute[TargetRank];
            SourceRoute[TargetRank] = SourceRoute[TargetRank + 1];
            SourceRoute[TargetRank + 1] = temp;
        }
        
        // Swap source job with first job of target edge
        var temp2 = SourceRoute[SourceRank];
        SourceRoute[SourceRank] = SourceRoute[TargetRank];
        SourceRoute[TargetRank] = temp2;
        
        // Save second job of target edge
        var tAfter = SourceRoute[TargetRank + 1];
        SourceRoute.RemoveAt(TargetRank + 1);
        
        // Calculate insertion position
        var endTargetRank = SourceRank + 1;
        if (TargetRank < SourceRank)
        {
            endTargetRank--;
        }
        
        SourceRoute.Insert(endTargetRank, tAfter);
        
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