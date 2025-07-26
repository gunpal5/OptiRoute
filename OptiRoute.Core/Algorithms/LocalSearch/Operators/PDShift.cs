using OptiRoute.Core.Models;
using static OptiRoute.Core.Algorithms.LocalSearch.InsertionSearch;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// PDShift operator - moves a pickup-delivery pair from one route to another (based on VROOM).
/// </summary>
internal class PDShift : Operator
{
    protected readonly int SourcePickupRank;
    protected readonly int SourceDeliveryRank;
    protected int BestTargetPickupRank;
    protected int BestTargetDeliveryRank;
    protected bool Valid = false;

    public PDShift(
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        int sourcePickupRank,
        int sourceDeliveryRank,
        RawRoute targetRoute,
        int targetVehicle,
        Eval gainThreshold)
        : base(OperatorName.PDShift, input, solutionState, sourceRoute, sourceVehicle,
               0, targetRoute, targetVehicle, 0) // Dummy rank values
    {
        if (sourceVehicle == targetVehicle)
            throw new ArgumentException("Source and target vehicles must be different");
        if (sourceRoute.Route.Count < 2)
            throw new ArgumentException("Source route must have at least 2 jobs");
        if (sourcePickupRank >= sourceDeliveryRank)
            throw new ArgumentException("Pickup rank must be less than delivery rank");
        if (sourceDeliveryRank >= sourceRoute.Route.Count)
            throw new ArgumentException("Delivery rank out of bounds");
        
        // Verify pickup-delivery pair
        var pickupJob = sourceRoute.Route[sourcePickupRank];
        var deliveryJob = sourceRoute.Route[sourceDeliveryRank];
        if (pickupJob + 1 != deliveryJob)
            throw new ArgumentException("Jobs must be a pickup-delivery pair");
            
        SourcePickupRank = sourcePickupRank;
        SourceDeliveryRank = sourceDeliveryRank;
        
        // Compute source gain (cost saved by removing P&D pair)
        SourceGain = solutionState.PdGains[sourceVehicle][sourcePickupRank];
        
        // Add fixed cost if this empties the source route
        if (sourceRoute.Route.Count == 2)
        {
            SourceGain = new Eval(
                SourceGain.Cost + input.Vehicles[sourceVehicle].Costs.Fixed,
                SourceGain.Duration,
                SourceGain.Distance);
        }
        
        // Subtract fixed cost if target route is empty
        if (targetRoute.Empty)
        {
            TargetGain = new Eval(
                -input.Vehicles[targetVehicle].Costs.Fixed,
                0,
                0);
        }
        else
        {
            TargetGain = new Eval(0, 0, 0);
        }
        
        // Verify source vehicle can handle the reduced route
        var sourceEval = solutionState.RouteEvals[sourceVehicle];
        if (!input.Vehicles[sourceVehicle].IsOkForRangeBounds(sourceEval - SourceGain))
            throw new InvalidOperationException("Source vehicle cannot handle reduced route");
            
        StoredGain = gainThreshold;
    }

    protected override void ComputeGain()
    {
        var pickupJob = SourceRoute[SourcePickupRank];
        
        // Find best insertion position for P&D pair in target route
        var insertion = ComputeBestInsertionPd(
            Input,
            SolutionState,
            pickupJob,
            TargetVehicle,
            Target,
            SourceGain - StoredGain);
            
        if (insertion.Eval != Eval.NoEval)
        {
            Valid = true;
            TargetGain = TargetGain - insertion.Eval;
            StoredGain = SourceGain + TargetGain;
            BestTargetPickupRank = insertion.PickupRank;
            BestTargetDeliveryRank = insertion.DeliveryRank;
        }
        
        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!GainComputed)
            ComputeGain();
        return Valid;
    }

    public override void Apply()
    {
        var pickupJob = SourceRoute[SourcePickupRank];
        var deliveryJob = SourceRoute[SourceDeliveryRank];
        
        // Build target route with P&D pair inserted
        var targetWithPd = new List<int>();
        targetWithPd.Add(pickupJob);
        
        // Add jobs between pickup and delivery insertion points
        for (int i = BestTargetPickupRank; i < BestTargetDeliveryRank - 1; i++)
        {
            if (i < TargetRoute.Count)
                targetWithPd.Add(TargetRoute[i]);
        }
        
        targetWithPd.Add(deliveryJob);
        
        // Replace range in target route
        Target.Replace(
            Input,
            targetWithPd[^1] == deliveryJob ? Input.GetJobDelivery(deliveryJob) : Input.ZeroAmount,
            targetWithPd.ToArray(),
            0,
            targetWithPd.Count,
            BestTargetPickupRank,
            BestTargetDeliveryRank);
            
        // Remove P&D pair from source route
        if (SourceDeliveryRank == SourcePickupRank + 1)
        {
            // Consecutive P&D pair - remove both at once
            SourceRoute.RemoveRange(SourcePickupRank, 2);
            Source.UpdateAmounts();
        }
        else
        {
            // Non-consecutive - remove delivery first, then pickup
            var sourceWithoutPd = new List<int>();
            for (int i = SourcePickupRank + 1; i < SourceDeliveryRank; i++)
            {
                sourceWithoutPd.Add(SourceRoute[i]);
            }
            
            Source.Replace(
                Input,
                Input.ZeroAmount,
                sourceWithoutPd.ToArray(),
                0,
                sourceWithoutPd.Count,
                SourcePickupRank,
                SourceDeliveryRank + 1);
        }
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