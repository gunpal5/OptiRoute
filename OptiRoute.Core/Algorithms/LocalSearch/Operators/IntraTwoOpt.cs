using OptiRoute.Core.Models;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// IntraTwoOpt operator - reverses a segment within the same route (based on VROOM).
/// </summary>
internal class IntraTwoOpt : Operator
{
    private readonly Amount _delivery;

    public IntraTwoOpt(
        Input input,
        SolutionState solutionState,
        RawRoute route,
        int vehicle,
        int sourceRank,
        int targetRank)
        : base(OperatorName.IntraTwoOpt, input, solutionState, route, vehicle,
               sourceRank, route, vehicle, targetRank)
    {
        // Match VROOM's assertions
        if (route.Route.Count < 3)
            throw new ArgumentException("Route must have at least 3 jobs");
        if (sourceRank >= targetRank - 1)
            throw new ArgumentException("Source rank must be less than target rank - 1");
        if (targetRank >= route.Route.Count)
            throw new ArgumentException("Target rank out of bounds");
            
        // Get delivery amount in the range to be reversed
        _delivery = route.DeliveryInRange(sourceRank, targetRank + 1);
    }

    protected override void ComputeGain()
    {
        var vehicle = Input.Vehicles[SourceVehicle];
        
        // Match VROOM's implementation using forward/backward costs
        // Cost of reversing vehicle route between s_rank and t_rank included
        StoredGain = StoredGain + SolutionState.FwdCosts[SourceVehicle][SourceVehicle][TargetRank];
        StoredGain = StoredGain - SolutionState.FwdCosts[SourceVehicle][SourceVehicle][SourceRank];
        StoredGain = StoredGain + SolutionState.BwdCosts[SourceVehicle][SourceVehicle][SourceRank];
        StoredGain = StoredGain - SolutionState.BwdCosts[SourceVehicle][SourceVehicle][TargetRank];
        
        var sJobIndex = SourceRoute[SourceRank];
        var tJobIndex = TargetRoute[TargetRank];
        var sLocationIndex = Array.IndexOf(Input.Locations.ToArray(), Input.Jobs[sJobIndex].Location);
        var tLocationIndex = Array.IndexOf(Input.Locations.ToArray(), Input.Jobs[tJobIndex].Location);
        
        // Cost of going to t_rank first instead of s_rank
        if (SourceRank > 0)
        {
            var previousJobIndex = SourceRoute[SourceRank - 1];
            var previousLocationIndex = Array.IndexOf(Input.Locations.ToArray(), Input.Jobs[previousJobIndex].Location);
            StoredGain = StoredGain + Input.GetEval(SourceVehicle, previousLocationIndex, sLocationIndex);
            StoredGain = StoredGain - Input.GetEval(SourceVehicle, previousLocationIndex, tLocationIndex);
        }
        else if (vehicle.StartLocation != null)
        {
            var startLocationIndex = Array.IndexOf(Input.Locations.ToArray(), vehicle.StartLocation);
            StoredGain = StoredGain + Input.GetEval(SourceVehicle, startLocationIndex, sLocationIndex);
            StoredGain = StoredGain - Input.GetEval(SourceVehicle, startLocationIndex, tLocationIndex);
        }
        
        // Cost of going from s_rank after instead of t_rank
        if (TargetRank < SourceRoute.Count - 1)
        {
            var nextJobIndex = SourceRoute[TargetRank + 1];
            var nextLocationIndex = Array.IndexOf(Input.Locations.ToArray(), Input.Jobs[nextJobIndex].Location);
            StoredGain = StoredGain + Input.GetEval(SourceVehicle, tLocationIndex, nextLocationIndex);
            StoredGain = StoredGain - Input.GetEval(SourceVehicle, sLocationIndex, nextLocationIndex);
        }
        else if (vehicle.EndLocation != null)
        {
            var endLocationIndex = Array.IndexOf(Input.Locations.ToArray(), vehicle.EndLocation);
            StoredGain = StoredGain + Input.GetEval(SourceVehicle, tLocationIndex, endLocationIndex);
            StoredGain = StoredGain - Input.GetEval(SourceVehicle, sLocationIndex, endLocationIndex);
        }
        
        GainComputed = true;
    }

    private bool ReversalOkForShipments()
    {
        bool valid = true;
        int current = SourceRank;
        
        while (valid && current < TargetRank)
        {
            var job = Input.Jobs[SourceRoute[current]];
            valid = (job.Type != JobType.Pickup) ||
                    (SolutionState.MatchingDeliveryRank[SourceVehicle][current] > TargetRank);
            
            current++;
        }
        
        return valid;
    }

    public override bool IsValid()
    {
        bool valid = (!Input.HasShipments || ReversalOkForShipments()) &&
                     IsValidForRangeBounds();
        
        if (valid)
        {
            // Create reverse iterators using LINQ
            var reversedSegment = SourceRoute
                .Skip(SourceRank)
                .Take(TargetRank - SourceRank + 1)
                .Reverse()
                .ToArray();
            
            valid = Source.IsValidAdditionForCapacityInclusion(
                _delivery,
                reversedSegment,
                0,
                reversedSegment.Length,
                SourceRank,
                TargetRank + 1);
        }
        
        return valid;
    }

    public override void Apply()
    {
        // Reverse the segment [SourceRank, TargetRank] inclusive
        SourceRoute.Reverse(SourceRank, TargetRank - SourceRank + 1);
        
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