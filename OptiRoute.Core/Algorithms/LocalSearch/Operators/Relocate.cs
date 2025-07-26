using OptiRoute.Core.Models;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// Relocate operator - moves a single job from source to target route (based on VROOM).
/// </summary>
internal class Relocate : Operator
{
    public Relocate(
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        int sourceRank,
        RawRoute targetRoute,
        int targetVehicle,
        int targetRank)
        : base(OperatorName.Relocate, input, solutionState, sourceRoute, sourceVehicle, 
               sourceRank, targetRoute, targetVehicle, targetRank)
    {
        if (sourceVehicle == targetVehicle)
            throw new ArgumentException("Source and target vehicles must be different");
        if (sourceRoute.Route.Count == 0)
            throw new ArgumentException("Source route cannot be empty");
        if (sourceRank >= sourceRoute.Route.Count)
            throw new ArgumentException("Source rank out of bounds");
        if (targetRank > targetRoute.Route.Count)
            throw new ArgumentException("Target rank out of bounds");
        if (!input.VehicleOkWithJob(targetVehicle, sourceRoute.Route[sourceRank]))
            throw new ArgumentException("Target vehicle incompatible with job");
    }

    protected override void ComputeGain()
    {
        // For source vehicle, we consider the cost of removing job at rank s_rank
        SourceGain = SolutionState.NodeGains[SourceVehicle][SourceRank];

        if (SourceRoute.Count == 1)
        {
            // Removing the only job saves the fixed cost
            SourceGain = SourceGain with { Cost = SourceGain.Cost + Input.Vehicles[SourceVehicle].Costs.Fixed };
        }

        // For target vehicle, we consider the cost of adding source job at rank t_rank
        var targetVehicle = Input.Vehicles[TargetVehicle];
        var additionCost = Utils.Helpers.AdditionCost(Input, SourceRoute[SourceRank], targetVehicle, TargetRoute, TargetRank);
        TargetGain = new Eval(-additionCost.Cost, -additionCost.Duration, -additionCost.Distance);

        if (TargetRoute.Count == 0)
        {
            // Adding to empty route incurs fixed cost
            TargetGain = TargetGain with { Cost = TargetGain.Cost - targetVehicle.Costs.Fixed };
        }

        StoredGain = SourceGain + TargetGain;
        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!GainComputed)
            ComputeGain();

        return IsValidForSourceRangeBounds() &&
               IsValidForTargetRangeBounds() &&
               Target.IsValidAdditionForCapacity(
                   Input.GetJobPickup(SourceRoute[SourceRank]),
                   Input.GetJobDelivery(SourceRoute[SourceRank]),
                   TargetRank);
    }

    public override void Apply()
    {
        var relocateJobRank = SourceRoute[SourceRank];
        SourceRoute.RemoveAt(SourceRank);
        TargetRoute.Insert(TargetRank, relocateJobRank);

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