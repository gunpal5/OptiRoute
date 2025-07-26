using OptiRoute.Core.Models;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// IntraRelocate operator - moves a job within the same route (based on VROOM).
/// Target rank is the position AFTER removal.
/// </summary>
internal class IntraRelocate : Operator
{
    private readonly List<int> _movedJobs;
    private readonly int _firstRank;
    private readonly int _lastRank;
    private readonly Amount _delivery;
    
    public IntraRelocate(
        Input input,
        SolutionState solutionState,
        RawRoute route,
        int vehicle,
        int sourceRank,
        int targetRank)  // targetRank is position AFTER removal
        : base(OperatorName.IntraRelocate, input, solutionState, route, vehicle, 
               sourceRank, route, vehicle, targetRank)
    {
        if (route.Route.Count < 2)
            throw new ArgumentException("Route must have at least 2 jobs");
        if (sourceRank >= route.Route.Count)
            throw new ArgumentException("Source rank out of bounds");
        if (targetRank > route.Route.Count - 1)
            throw new ArgumentException("Target rank out of bounds");
        if (sourceRank == targetRank)
            throw new ArgumentException("Invalid move - no change in position");
            
        // Calculate moved jobs and ranks
        _firstRank = Math.Min(sourceRank, targetRank);
        _lastRank = Math.Max(sourceRank, targetRank) + 1;
        int movedCount = (sourceRank < targetRank) ? targetRank - sourceRank + 1 : sourceRank - targetRank + 1;
        _movedJobs = new List<int>(movedCount);
        
        // Fill moved jobs array based on direction
        if (targetRank < sourceRank)
        {
            // Moving backward
            _movedJobs.Add(route.Route[sourceRank]);
            for (int i = targetRank; i < sourceRank; i++)
            {
                _movedJobs.Add(route.Route[i]);
            }
        }
        else
        {
            // Moving forward
            for (int i = sourceRank + 1; i <= targetRank; i++)
            {
                _movedJobs.Add(route.Route[i]);
            }
            _movedJobs.Add(route.Route[sourceRank]);
        }
        
        // Calculate delivery amount for the moved range
        _delivery = route.DeliveryInRange(_firstRank, _lastRank);
    }

    protected override void ComputeGain()
    {
        var vehicle = Input.Vehicles[SourceVehicle];
        
        // For removal, we consider the cost of removing job at rank s_rank,
        // already stored in _sol_state.node_gains[s_vehicle][s_rank].
        
        // For addition, consider the cost of adding source job at new rank
        // *after* removal.
        var newRank = TargetRank;
        if (SourceRank < TargetRank)
        {
            newRank++;
        }
        
        StoredGain = SolutionState.NodeGains[SourceVehicle][SourceRank] -
                     Utils.Helpers.AdditionCost(Input, SourceRoute[SourceRank], vehicle, TargetRoute, newRank);
                     
        GainComputed = true;
    }

    public override bool IsValid()
    {
        return IsValidForRangeBounds() &&
               Source.IsValidAdditionForCapacityInclusion(
                   _delivery,
                   _movedJobs.ToArray(),
                   0,
                   _movedJobs.Count,
                   _firstRank,
                   _lastRank);
    }

    public override void Apply()
    {
        var relocateJobRank = SourceRoute[SourceRank];
        SourceRoute.RemoveAt(SourceRank);
        TargetRoute.Insert(TargetRank, relocateJobRank);
        
        Source.UpdateAmounts();
    }

    public override List<int> AdditionCandidates()
    {
        // No addition candidates for intra-route moves
        return new List<int>();
    }

    public override List<int> UpdateCandidates()
    {
        return new List<int> { SourceVehicle };
    }
}