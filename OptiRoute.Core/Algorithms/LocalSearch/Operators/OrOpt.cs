using OptiRoute.Core.Models;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// OrOpt operator - moves an edge (2 consecutive jobs) from source to target route with optional reversal (based on VROOM).
/// </summary>
internal class OrOpt : Operator
{
    private bool _gainUpperBoundComputed = false;
    private Eval _normalTGain;
    private Eval _reversedTGain;
    protected bool ReverseEdge = false;
    protected bool IsNormalValid = false;
    protected bool IsReverseValid = false;
    protected readonly Amount EdgeDelivery;

    public OrOpt(
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        int sourceRank,
        RawRoute targetRoute,
        int targetVehicle,
        int targetRank)
        : base(OperatorName.OrOpt, input, solutionState, sourceRoute, sourceVehicle,
               sourceRank, targetRoute, targetVehicle, targetRank)
    {
        if (sourceVehicle == targetVehicle)
            throw new ArgumentException("Source and target vehicles must be different");
        if (sourceRoute.Route.Count < 2)
            throw new ArgumentException("Source route must have at least 2 jobs");
        if (sourceRank >= sourceRoute.Route.Count - 1)
            throw new ArgumentException("Source rank out of bounds");
        if (targetRank > targetRoute.Route.Count)
            throw new ArgumentException("Target rank out of bounds");

        // Check vehicle compatibility for both jobs in the edge
        if (!input.VehicleOkWithJob(targetVehicle, sourceRoute.Route[sourceRank]) ||
            !input.VehicleOkWithJob(targetVehicle, sourceRoute.Route[sourceRank + 1]))
        {
            throw new ArgumentException("Target vehicle incompatible with edge jobs");
        }

        // Calculate total delivery for the edge
        EdgeDelivery = input.GetJobDelivery(sourceRoute.Route[sourceRank]) +
                      input.GetJobDelivery(sourceRoute.Route[sourceRank + 1]);
    }

    /// <summary>
    /// Compute and store all possible costs depending on whether edges
    /// are reversed or not. Return only an upper bound for gain as
    /// precise gain requires validity information.
    /// </summary>
    public Eval GainUpperBound()
    {
        // Match VROOM exactly - use removal_cost_delta
        SourceGain = Helpers.RemovalCostDelta(Input, SolutionState, Source, SourceRank, 2);

        (_normalTGain, _reversedTGain) = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Target,
            TargetRank,
            TargetRank,
            Source,
            SourceRank,
            SourceRank + 2);

        _gainUpperBoundComputed = true;

        return SourceGain + (_normalTGain.Cost >= _reversedTGain.Cost ? _normalTGain : _reversedTGain);
    }

    protected override void ComputeGain()
    {
        if (!_gainUpperBoundComputed)
            throw new InvalidOperationException("GainUpperBound must be called before ComputeGain");
        
        if (!IsNormalValid && !IsReverseValid)
            throw new InvalidOperationException("At least one direction must be valid");

        StoredGain = SourceGain;

        if (_normalTGain < _reversedTGain)
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
            GainUpperBound();

        var edgePickup = Input.GetJobPickup(SourceRoute[SourceRank]) +
                        Input.GetJobPickup(SourceRoute[SourceRank + 1]);

        bool valid = IsValidForSourceRangeBounds() &&
                     Target.IsValidAdditionForCapacity(edgePickup, EdgeDelivery, TargetRank);

        if (valid)
        {
            // Keep edge direction
            var targetVehicle = Input.Vehicles[TargetVehicle];
            var targetEval = SolutionState.RouteEvals[TargetVehicle];

            // Check normal direction
            IsNormalValid = targetVehicle.IsOkForRangeBounds(targetEval - _normalTGain) &&
                           Target.IsValidAdditionForCapacityInclusion(
                               EdgeDelivery,
                               SourceRoute.ToArray(),
                               SourceRank,
                               SourceRank + 2,
                               TargetRank,
                               TargetRank);

            // Check reverse direction
            // Note: VROOM uses reverse_iterator here, but we'll simulate by reversing the jobs
            var reversedJobs = new int[] { SourceRoute[SourceRank + 1], SourceRoute[SourceRank] };
            IsReverseValid = targetVehicle.IsOkForRangeBounds(targetEval - _reversedTGain) &&
                            Target.IsValidAdditionForCapacityInclusion(
                                EdgeDelivery,
                                reversedJobs,
                                0,
                                2,
                                TargetRank,
                                TargetRank);

            valid = IsNormalValid || IsReverseValid;
        }

        return valid;
    }

    public override void Apply()
    {
        // Extract the edge from source
        var edge = new List<int> { SourceRoute[SourceRank], SourceRoute[SourceRank + 1] };
        
        // Insert into target
        TargetRoute.InsertRange(TargetRank, edge);
        
        if (ReverseEdge)
        {
            // Swap the two jobs that were just inserted
            var temp = TargetRoute[TargetRank];
            TargetRoute[TargetRank] = TargetRoute[TargetRank + 1];
            TargetRoute[TargetRank + 1] = temp;
        }

        // Remove from source
        SourceRoute.RemoveRange(SourceRank, 2);

        Source.UpdateAmounts();
        Target.UpdateAmounts();
    }

    public override List<int> AdditionCandidates()
    {
        return new List<int> { SourceVehicle };
    }

    public override List<int> UpdateCandidates()
    {
        return new List<int> { SourceVehicle, TargetVehicle };
    }
}