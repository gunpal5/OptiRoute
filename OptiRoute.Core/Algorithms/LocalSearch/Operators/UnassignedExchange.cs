using OptiRoute.Core.Models;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// UnassignedExchange operator - exchanges an unassigned job with a job in a route (based on VROOM).
/// </summary>
internal class UnassignedExchange : Operator
{
    private readonly int _unassignedJob; // Unassigned job to insert
    private readonly HashSet<int> _unassigned;
    private readonly int _firstRank;
    private readonly int _lastRank;
    private readonly List<int> _movedJobs;
    private readonly int _removedJob;
    private Amount _delivery;

    public UnassignedExchange(
        Input input,
        SolutionState solutionState,
        HashSet<int> unassigned,
        RawRoute route,
        int vehicle,
        int sourceRank,
        int targetRank,
        int unassignedJob)
        : base(OperatorName.UnassignedExchange, input, solutionState, route, vehicle,
               sourceRank, route, vehicle, targetRank)
    {
        if (targetRank == sourceRank + 1)
            throw new ArgumentException("Target rank cannot be source rank + 1");
        if (route.Route.Count == 0)
            throw new ArgumentException("Route cannot be empty");
        if (sourceRank >= route.Route.Count)
            throw new ArgumentException("Source rank out of bounds");
        if (targetRank > route.Route.Count)
            throw new ArgumentException("Target rank out of bounds");
            
        _unassignedJob = unassignedJob;
        _unassigned = unassigned;
        _firstRank = Math.Min(sourceRank, targetRank);
        _lastRank = sourceRank < targetRank ? targetRank : sourceRank + 1;
        _movedJobs = new List<int>(_lastRank - _firstRank);
        _removedJob = route.Route[sourceRank];
        _delivery = route.DeliveryInRange(_firstRank, _lastRank);
        
        // Verify removed job's delivery fits within range delivery
        var removedDelivery = input.GetJobDelivery(_removedJob);
        if (!(removedDelivery <= _delivery))
            throw new InvalidOperationException("Removed job delivery exceeds range delivery");
            
        _delivery = _delivery - removedDelivery;
        _delivery = _delivery + input.GetJobDelivery(_unassignedJob);
        
        // Initialize moved jobs array
        if (sourceRank < targetRank)
        {
            // Copy jobs after source rank up to target rank
            for (int i = sourceRank + 1; i < targetRank; i++)
            {
                _movedJobs.Add(route.Route[i]);
            }
            _movedJobs.Add(unassignedJob);
        }
        else
        {
            // Copy jobs from target rank up to source rank
            _movedJobs.Add(unassignedJob);
            for (int i = targetRank; i < sourceRank; i++)
            {
                _movedJobs.Add(route.Route[i]);
            }
        }
    }

    protected override void ComputeGain()
    {
        if (TargetRank == SourceRank)
        {
            // Simple replacement at same position
            SourceGain = Helpers.AdditionCostDelta(
                Input,
                SolutionState,
                Source,
                SourceRank,
                SourceRank + 1,
                _unassignedJob);
        }
        else
        {
            // No common edge so both gains can be computed independently
            var vehicle = Input.Vehicles[SourceVehicle];
            
            SourceGain = SolutionState.NodeGains[SourceVehicle][SourceRank] -
                        Helpers.AdditionCost(Input, _unassignedJob, vehicle, SourceRoute, TargetRank);
        }
        
        StoredGain = SourceGain;
        GainComputed = true;
    }

    public override bool IsValid()
    {
        var pickup = Source.PickupInRange(_firstRank, _lastRank);
        var removedPickup = Input.GetJobPickup(_removedJob);
        
        // Verify removed job's pickup fits within range pickup
        if (!(removedPickup <= pickup))
            return false;
            
        pickup = pickup - removedPickup;
        pickup = pickup + Input.GetJobPickup(_unassignedJob);
        
        bool valid = Source.IsValidAdditionForCapacityMargins(
            pickup,
            _delivery,
            _firstRank,
            _lastRank);
            
        if (!valid)
            return false;
            
        valid = Source.IsValidAdditionForCapacityInclusion(
            _delivery,
            _movedJobs.ToArray(),
            0,
            _movedJobs.Count,
            _firstRank,
            _lastRank);
            
        if (valid)
        {
            // Check validity with regard to vehicle range bounds
            if (!GainComputed)
            {
                ComputeGain();
            }
            
            valid = IsValidForSourceRangeBounds();
        }
        
        return valid;
    }

    public override void Apply()
    {
        // Replace the range with moved jobs
        int idx = _firstRank;
        foreach (var job in _movedJobs)
        {
            SourceRoute[idx++] = job;
        }
        
        // Update unassigned set
        if (!_unassigned.Contains(_unassignedJob))
            throw new InvalidOperationException("Unassigned job not in unassigned set");
        _unassigned.Remove(_unassignedJob);
        
        if (_unassigned.Contains(_removedJob))
            throw new InvalidOperationException("Removed job already in unassigned set");
        _unassigned.Add(_removedJob);
        
        Source.UpdateAmounts();
    }

    public override List<int> AdditionCandidates()
    {
        return Input.CompatibleVehiclesForJob[_removedJob];
    }

    public override List<int> UpdateCandidates()
    {
        return new List<int> { SourceVehicle };
    }

    /// <summary>
    /// Gets the required unassigned jobs for this operator.
    /// </summary>
    public override List<int> RequiredUnassigned()
    {
        return new List<int> { _unassignedJob };
    }
}