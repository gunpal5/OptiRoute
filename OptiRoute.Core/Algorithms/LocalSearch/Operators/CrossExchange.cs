using OptiRoute.Core.Models;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// CrossExchange operator - exchanges two edges between routes (based on VROOM).
/// </summary>
internal class CrossExchange : Operator
{
    private bool _gainUpperBoundComputed;
    private Eval _normalSGain;
    private Eval _reversedSGain = Eval.NoEval;
    private Eval _normalTGain;
    private Eval _reversedTGain = Eval.NoEval;

    protected bool ReverseSEdge { get; set; }
    protected bool ReverseTEdge { get; set; }
    protected readonly bool CheckSReverse;
    protected readonly bool CheckTReverse;

    protected bool SIsNormalValid { get; set; }
    protected bool SIsReverseValid { get; set; }
    protected bool TIsNormalValid { get; set; }
    protected bool TIsReverseValid { get; set; }

    protected readonly Amount SourceDelivery;
    protected readonly Amount TargetDelivery;

    public CrossExchange(
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        int sourceRank,
        RawRoute targetRoute,
        int targetVehicle,
        int targetRank,
        bool checkSReverse,
        bool checkTReverse)
        : base(OperatorName.CrossExchange, input, solutionState, sourceRoute, sourceVehicle,
               sourceRank, targetRoute, targetVehicle, targetRank)
    {
        CheckSReverse = checkSReverse;
        CheckTReverse = checkTReverse;

        if (sourceVehicle == targetVehicle)
            throw new ArgumentException("Source and target vehicles must be different");
        if (sourceRoute.Route.Count < 2)
            throw new ArgumentException("Source route must have at least 2 jobs");
        if (targetRoute.Route.Count < 2)
            throw new ArgumentException("Target route must have at least 2 jobs");
        if (sourceRank >= sourceRoute.Route.Count - 1)
            throw new ArgumentException("Source rank out of bounds");
        if (targetRank >= targetRoute.Route.Count - 1)
            throw new ArgumentException("Target rank out of bounds");

        // Check vehicle compatibility
        if (!input.VehicleOkWithJob(targetVehicle, sourceRoute.Route[sourceRank]) ||
            !input.VehicleOkWithJob(targetVehicle, sourceRoute.Route[sourceRank + 1]) ||
            !input.VehicleOkWithJob(sourceVehicle, targetRoute.Route[targetRank]) ||
            !input.VehicleOkWithJob(sourceVehicle, targetRoute.Route[targetRank + 1]))
        {
            throw new ArgumentException("Vehicle incompatible with jobs");
        }

        // Either moving edges of single jobs or whole shipments.
        var sJob1 = input.Jobs[sourceRoute.Route[sourceRank]];
        var sJob2 = input.Jobs[sourceRoute.Route[sourceRank + 1]];
        if (!((sJob1.Type == JobType.Single && sJob2.Type == JobType.Single && checkSReverse) ||
              (sJob1.Type == JobType.Pickup && sJob2.Type == JobType.Delivery && !checkSReverse &&
               solutionState.MatchingDeliveryRank[sourceVehicle][sourceRank] == sourceRank + 1)))
        {
            throw new ArgumentException("Invalid job types for source edge");
        }
        
        var tJob1 = input.Jobs[targetRoute.Route[targetRank]];
        var tJob2 = input.Jobs[targetRoute.Route[targetRank + 1]];
        if (!((tJob1.Type == JobType.Single && tJob2.Type == JobType.Single && checkTReverse) ||
              (tJob1.Type == JobType.Pickup && tJob2.Type == JobType.Delivery && !checkTReverse &&
               solutionState.MatchingDeliveryRank[targetVehicle][targetRank] == targetRank + 1)))
        {
            throw new ArgumentException("Invalid job types for target edge");
        }

        // Compute deliveries for the edges
        SourceDelivery = input.GetJobDelivery(sourceRoute.Route[sourceRank]) +
                         input.GetJobDelivery(sourceRoute.Route[sourceRank + 1]);
        TargetDelivery = input.GetJobDelivery(targetRoute.Route[targetRank]) +
                         input.GetJobDelivery(targetRoute.Route[targetRank + 1]);
    }

    /// <summary>
    /// Compute and store all possible costs depending on whether edges are reversed or not.
    /// </summary>
    public Eval GainUpperBound()
    {
        // Calculate addition cost delta for source route
        (_normalSGain, _reversedSGain) = CalculateAdditionCostDelta(
            Source, SourceRank, SourceRank + 2,
            Target, TargetRank, TargetRank + 2);

        var sGainUpperBound = _normalSGain;
        if (CheckTReverse)
        {
            sGainUpperBound = MaxEval(sGainUpperBound, _reversedSGain);
        }

        // Calculate addition cost delta for target route
        (_normalTGain, _reversedTGain) = CalculateAdditionCostDelta(
            Target, TargetRank, TargetRank + 2,
            Source, SourceRank, SourceRank + 2);

        var tGainUpperBound = _normalTGain;
        if (CheckSReverse)
        {
            tGainUpperBound = MaxEval(tGainUpperBound, _reversedTGain);
        }

        _gainUpperBoundComputed = true;
        return sGainUpperBound + tGainUpperBound;
    }

    protected override void ComputeGain()
    {
        if (!_gainUpperBoundComputed)
            throw new InvalidOperationException("Must call GainUpperBound first");
        if (!(SIsNormalValid || SIsReverseValid))
            throw new InvalidOperationException("No valid source configuration");

        StoredGain = new Eval(0, 0, 0);

        // Choose best gain for source
        if (_normalSGain.Cost < _reversedSGain.Cost)
        {
            if (SIsReverseValid)
            {
                StoredGain = StoredGain + _reversedSGain;
                ReverseTEdge = true;
            }
            else
            {
                StoredGain = StoredGain + _normalSGain;
            }
        }
        else
        {
            if (SIsNormalValid)
            {
                StoredGain = StoredGain + _normalSGain;
            }
            else
            {
                StoredGain = StoredGain + _reversedSGain;
                ReverseTEdge = true;
            }
        }

        // Choose best gain for target
        if (!(TIsNormalValid || TIsReverseValid))
            throw new InvalidOperationException("No valid target configuration");

        if (_normalTGain.Cost < _reversedTGain.Cost)
        {
            if (TIsReverseValid)
            {
                StoredGain = StoredGain + _reversedTGain;
                ReverseSEdge = true;
            }
            else
            {
                StoredGain = StoredGain + _normalTGain;
            }
        }
        else
        {
            if (TIsNormalValid)
            {
                StoredGain = StoredGain + _normalTGain;
            }
            else
            {
                StoredGain = StoredGain + _reversedTGain;
                ReverseSEdge = true;
            }
        }

        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!_gainUpperBoundComputed)
            GainUpperBound();

        var targetPickup = Input.GetJobPickup(TargetRoute[TargetRank]) +
                          Input.GetJobPickup(TargetRoute[TargetRank + 1]);

        bool valid = Source.IsValidAdditionForCapacityMargins(
            targetPickup, TargetDelivery, SourceRank, SourceRank + 2);

        if (valid)
        {
            var sourceVehicle = Input.Vehicles[SourceVehicle];
            var sourceEval = SolutionState.RouteEvals[SourceVehicle];

            // Keep target edge direction when inserting in source route
            var targetJobs = new[] { TargetRoute[TargetRank], TargetRoute[TargetRank + 1] };
            SIsNormalValid = sourceVehicle.IsOkForRangeBounds(sourceEval - _normalSGain) &&
                            Source.IsValidAdditionForCapacityInclusion(
                                TargetDelivery, targetJobs, 0, 2, SourceRank, SourceRank + 2);

            if (CheckTReverse)
            {
                // Reverse target edge direction when inserting in source route
                var targetJobsReversed = new[] { TargetRoute[TargetRank + 1], TargetRoute[TargetRank] };
                SIsReverseValid = sourceVehicle.IsOkForRangeBounds(sourceEval - _reversedSGain) &&
                                 Source.IsValidAdditionForCapacityInclusion(
                                     TargetDelivery, targetJobsReversed, 0, 2, SourceRank, SourceRank + 2);
            }

            valid = SIsNormalValid || SIsReverseValid;
        }

        var sourcePickup = Input.GetJobPickup(SourceRoute[SourceRank]) +
                          Input.GetJobPickup(SourceRoute[SourceRank + 1]);

        valid = valid && Target.IsValidAdditionForCapacityMargins(
            sourcePickup, SourceDelivery, TargetRank, TargetRank + 2);

        if (valid)
        {
            var targetVehicle = Input.Vehicles[TargetVehicle];
            var targetEval = SolutionState.RouteEvals[TargetVehicle];

            // Keep source edge direction when inserting in target route
            var sourceJobs = new[] { SourceRoute[SourceRank], SourceRoute[SourceRank + 1] };
            TIsNormalValid = targetVehicle.IsOkForRangeBounds(targetEval - _normalTGain) &&
                            Target.IsValidAdditionForCapacityInclusion(
                                SourceDelivery, sourceJobs, 0, 2, TargetRank, TargetRank + 2);

            if (CheckSReverse)
            {
                // Reverse source edge direction when inserting in target route
                var sourceJobsReversed = new[] { SourceRoute[SourceRank + 1], SourceRoute[SourceRank] };
                TIsReverseValid = targetVehicle.IsOkForRangeBounds(targetEval - _reversedTGain) &&
                                 Target.IsValidAdditionForCapacityInclusion(
                                     SourceDelivery, sourceJobsReversed, 0, 2, TargetRank, TargetRank + 2);
            }

            valid = TIsNormalValid || TIsReverseValid;
        }

        return valid;
    }

    public override void Apply()
    {
        // Swap the edges
        var tempFirst = SourceRoute[SourceRank];
        var tempSecond = SourceRoute[SourceRank + 1];
        
        SourceRoute[SourceRank] = TargetRoute[TargetRank];
        SourceRoute[SourceRank + 1] = TargetRoute[TargetRank + 1];
        
        TargetRoute[TargetRank] = tempFirst;
        TargetRoute[TargetRank + 1] = tempSecond;

        // Apply reversals if needed
        if (ReverseSEdge)
        {
            (TargetRoute[TargetRank], TargetRoute[TargetRank + 1]) = 
                (TargetRoute[TargetRank + 1], TargetRoute[TargetRank]);
        }
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

    /// <summary>
    /// Calculate the cost delta for adding/removing edges.
    /// Returns normal and reversed gains.
    /// </summary>
    private (Eval normal, Eval reversed) CalculateAdditionCostDelta(
        RawRoute fromRoute, int fromStart, int fromEnd,
        RawRoute toRoute, int toStart, int toEnd)
    {
        return Utils.Helpers.AdditionCostDelta(
            Input, SolutionState, toRoute, toStart, toEnd, fromRoute, fromStart, fromEnd);
    }

    private Eval MaxEval(Eval a, Eval b)
    {
        return a.Cost >= b.Cost ? a : b;
    }
}