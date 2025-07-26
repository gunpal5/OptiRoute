using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.VRP;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// RouteExchange operator - exchanges complete routes between two vehicles (based on VROOM).
/// </summary>
internal class RouteExchange : Operator
{
    public RouteExchange(
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        RawRoute targetRoute,
        int targetVehicle)
        : base(OperatorName.RouteExchange, input, solutionState, sourceRoute, sourceVehicle,
               0, targetRoute, targetVehicle, 0) // Dummy values for ranks
    {
        if (sourceVehicle == targetVehicle)
            throw new ArgumentException("Source and target vehicles must be different");
        
        if (sourceRoute.Route.Count == 0 && targetRoute.Route.Count == 0)
            throw new ArgumentException("At least one route must be non-empty");
            
        // Whole routes should be transferable
        if (solutionState.BwdSkillRank[sourceVehicle][targetVehicle] != 0)
            throw new ArgumentException("Source route has jobs incompatible with target vehicle");
        if (solutionState.BwdSkillRank[targetVehicle][sourceVehicle] != 0)
            throw new ArgumentException("Target route has jobs incompatible with source vehicle");
    }

    protected override void ComputeGain()
    {
        // Cost of replacing source route with target route
        SourceGain = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Source,
            0,
            SourceRoute.Count,
            Target,
            0,
            TargetRoute.Count).Item1;
            
        // Cost of replacing target route with source route
        TargetGain = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Target,
            0,
            TargetRoute.Count,
            Source,
            0,
            SourceRoute.Count).Item1;
            
        StoredGain = SourceGain + TargetGain;
        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!GainComputed)
            ComputeGain();
            
        // Check range bounds
        var sourceVehicle = Input.Vehicles[SourceVehicle];
        var targetVehicle = Input.Vehicles[TargetVehicle];
        var sourceEval = SolutionState.RouteEvals[SourceVehicle];
        var targetEval = SolutionState.RouteEvals[TargetVehicle];
        
        bool validRangeBounds = sourceVehicle.IsOkForRangeBounds(sourceEval - SourceGain) &&
                               targetVehicle.IsOkForRangeBounds(targetEval - TargetGain);
                               
        if (!validRangeBounds)
            return false;
            
        // Check capacity constraints
        var sourceMaxLoad = Source.MaxLoad();
        var targetMaxLoad = Target.MaxLoad();
        var sourceCapacity = Input.GetVehicleCapacity(SourceVehicle);
        var targetCapacity = Input.GetVehicleCapacity(TargetVehicle);
        
        return sourceMaxLoad <= targetCapacity && targetMaxLoad <= sourceCapacity;
    }

    public override void Apply()
    {
        // Create temporary copy of source route
        var tempRoute = new List<int>(SourceRoute);
        
        // Clear and swap routes
        SourceRoute.Clear();
        SourceRoute.AddRange(TargetRoute);
        
        TargetRoute.Clear();
        TargetRoute.AddRange(tempRoute);
        
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