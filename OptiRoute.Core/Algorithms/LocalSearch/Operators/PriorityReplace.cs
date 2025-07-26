using OptiRoute.Core.Models;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// PriorityReplace operator - replaces low-priority jobs with a high-priority unassigned job (based on VROOM).
/// </summary>
internal class PriorityReplace : Operator
{
    private bool _startGainComputed = false;
    private bool _endGainComputed = false;
    private readonly int _startPriorityGain;
    private readonly int _endPriorityGain;
    private readonly int _startAssignedNumber;
    private readonly int _endAssignedNumber;
    
    protected readonly int UnassignedJob;
    protected readonly int BestKnownPriorityGain;
    protected readonly HashSet<int> Unassigned;
    
    private readonly Amount _delivery;
    
    protected bool ReplaceStartValid = false;
    protected bool ReplaceEndValid = false;

    public PriorityReplace(
        Input input,
        SolutionState solutionState,
        HashSet<int> unassigned,
        RawRoute route,
        int vehicle,
        int sourceRank, // last rank (included) when replacing route start
        int targetRank, // first rank when replacing route end
        int unassignedJob,
        int bestKnownPriorityGain)
        : base(OperatorName.PriorityReplace, input, solutionState, route, vehicle,
               sourceRank, route, vehicle, targetRank)
    {
        if (route.Empty)
            throw new ArgumentException("Route cannot be empty");
        if (targetRank == 0)
            throw new ArgumentException("Target rank must be greater than 0");
            
        _startPriorityGain = input.Jobs[unassignedJob].Priority - 
                            solutionState.FwdPriority[vehicle][sourceRank];
        _endPriorityGain = input.Jobs[unassignedJob].Priority - 
                          solutionState.BwdPriority[vehicle][targetRank];
                          
        if (_startPriorityGain <= 0 && _endPriorityGain <= 0)
            throw new ArgumentException("At least one priority gain must be positive");
            
        _startAssignedNumber = route.Route.Count - sourceRank;
        _endAssignedNumber = targetRank + 1;
        UnassignedJob = unassignedJob;
        BestKnownPriorityGain = bestKnownPriorityGain;
        Unassigned = unassigned;
        _delivery = input.Jobs[unassignedJob].Delivery;
    }

    private void ComputeStartGain()
    {
        SourceGain = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Source,
            0,
            SourceRank + 1,
            UnassignedJob);
            
        _startGainComputed = true;
    }

    private void ComputeEndGain()
    {
        TargetGain = Helpers.AdditionCostDelta(
            Input,
            SolutionState,
            Source,
            TargetRank,
            Source.Route.Count,
            UnassignedJob);
            
        _endGainComputed = true;
    }

    protected override void ComputeGain()
    {
        if (!ReplaceStartValid && !ReplaceEndValid)
            throw new InvalidOperationException("At least one replacement must be valid");
        if (ReplaceStartValid && !_startGainComputed)
            throw new InvalidOperationException("Start gain must be computed");
        if (ReplaceEndValid && !_endGainComputed)
            throw new InvalidOperationException("End gain must be computed");
            
        if (ReplaceStartValid && ReplaceEndValid)
        {
            // Decide based on priority and cost.
            // Tuple comparison: (priority_gain, assigned_number, gain)
            var endTuple = (_endPriorityGain, _endAssignedNumber, TargetGain.Cost);
            var startTuple = (_startPriorityGain, _startAssignedNumber, SourceGain.Cost);
            
            if (endTuple.CompareTo(startTuple) < 0)
            {
                ReplaceEndValid = false;
            }
            else
            {
                ReplaceStartValid = false;
            }
        }
        
        if (ReplaceStartValid)
        {
            StoredGain = SourceGain;
        }
        else
        {
            StoredGain = TargetGain;
        }
        
        GainComputed = true;
    }

    public override bool IsValid()
    {
        var job = Input.Jobs[UnassignedJob];
        
        // Early abort if priority gain is not interesting anyway or the
        // move is not interesting: s_rank is zero if the candidate start
        // portion is empty or with a single job (that would be an
        // UnassignedExchange move).
        ReplaceStartValid = 
            (0 < _startPriorityGain) &&
            (BestKnownPriorityGain <= _startPriorityGain) && 
            (SourceRank > 0) &&
            Source.IsValidAdditionForCapacityMargins(
                job.Pickup,
                job.Delivery,
                0,
                SourceRank + 1);
                
        if (ReplaceStartValid && Source.HasPendingDeliveryAfterRank(SourceRank))
            throw new InvalidOperationException("Should not have pending delivery after source rank");
            
        // Don't bother if the candidate end portion is empty or with a
        // single job (that would be an UnassignedExchange move).
        ReplaceEndValid = 
            (0 < _endPriorityGain) &&
            (BestKnownPriorityGain <= _endPriorityGain) &&
            (TargetRank < SourceRoute.Count - 1) &&
            Source.IsValidAdditionForCapacityMargins(
                job.Pickup,
                job.Delivery,
                TargetRank,
                SourceRoute.Count);
                
        if (ReplaceEndValid && Source.HasPendingDeliveryAfterRank(TargetRank - 1))
            throw new InvalidOperationException("Should not have pending delivery after target rank - 1");
            
        // Check validity with regard to vehicle range bounds, requires
        // valid gain values for both options.
        if (ReplaceStartValid)
        {
            ComputeStartGain();
            ReplaceStartValid = IsValidForSourceRangeBounds();
        }
        
        if (ReplaceEndValid)
        {
            ComputeEndGain();
            ReplaceEndValid = IsValidForTargetRangeBounds();
        }
        
        return ReplaceStartValid || ReplaceEndValid;
    }

    public override void Apply()
    {
        if (!Unassigned.Contains(UnassignedJob))
            throw new InvalidOperationException("Unassigned job must be in unassigned set");
            
        Unassigned.Remove(UnassignedJob);
        
        var addition = new[] { UnassignedJob };
        
        if (ReplaceStartValid && ReplaceEndValid)
            throw new InvalidOperationException("Only one replacement should be valid at this point");
            
        if (ReplaceStartValid)
        {
            // Check all jobs being replaced are not in unassigned
            for (int i = 0; i <= SourceRank; i++)
            {
                if (Unassigned.Contains(SourceRoute[i]))
                    throw new InvalidOperationException($"Job {SourceRoute[i]} should not be in unassigned");
            }
            
            // Add replaced jobs to unassigned
            for (int i = 0; i <= SourceRank; i++)
            {
                Unassigned.Add(SourceRoute[i]);
            }
            
            Source.Replace(Input, _delivery, addition, 0, addition.Length, 0, SourceRank + 1);
        }
        else
        {
            // Check all jobs being replaced are not in unassigned
            for (int i = TargetRank; i < SourceRoute.Count; i++)
            {
                if (Unassigned.Contains(SourceRoute[i]))
                    throw new InvalidOperationException($"Job {SourceRoute[i]} should not be in unassigned");
            }
            
            // Add replaced jobs to unassigned
            for (int i = TargetRank; i < SourceRoute.Count; i++)
            {
                Unassigned.Add(SourceRoute[i]);
            }
            
            Source.Replace(Input, _delivery, addition, 0, addition.Length, TargetRank, SourceRoute.Count);
        }
    }

    public int PriorityGain()
    {
        if (!GainComputed)
        {
            // Priority gain depends on actual option, decided in compute_gain.
            ComputeGain();
        }
        
        if (ReplaceStartValid && ReplaceEndValid)
            throw new InvalidOperationException("Only one replacement should be valid");
            
        return ReplaceStartValid ? _startPriorityGain : _endPriorityGain;
    }

    public int Assigned()
    {
        if (!GainComputed)
            throw new InvalidOperationException("Gain must be computed first");
        if (ReplaceStartValid && ReplaceEndValid)
            throw new InvalidOperationException("Only one replacement should be valid");
            
        return ReplaceStartValid ? _startAssignedNumber : _endAssignedNumber;
    }

    public override List<int> AdditionCandidates()
    {
        return new List<int> { SourceVehicle };
    }

    public override List<int> UpdateCandidates()
    {
        return new List<int> { SourceVehicle };
    }

    public override List<int> RequiredUnassigned()
    {
        return new List<int> { UnassignedJob };
    }
}