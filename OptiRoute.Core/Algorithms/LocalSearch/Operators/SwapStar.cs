using OptiRoute.Core.Models;
using static OptiRoute.Core.Algorithms.LocalSearch.SwapStarUtils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// SwapStar operator - advanced swap that can move jobs to different positions (based on VROOM).
/// </summary>
internal class SwapStar : Operator
{
    private readonly Eval _bestKnownGain;
    private SwapChoice _choice;

    public SwapStar(
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        RawRoute targetRoute,
        int targetVehicle,
        Eval bestKnownGain)
        : base(OperatorName.SwapStar, input, solutionState, sourceRoute, sourceVehicle,
               0, targetRoute, targetVehicle, 0) // Dummy rank values
    {
        if (sourceVehicle == targetVehicle)
            throw new ArgumentException("Source and target vehicles must be different");
        if (sourceRoute.Empty)
            throw new ArgumentException("Source route cannot be empty");
        if (targetRoute.Empty)
            throw new ArgumentException("Target route cannot be empty");
        if (!input.VehicleOkWithVehicle(sourceVehicle, targetVehicle))
            throw new ArgumentException("Vehicles are not compatible");
            
        _bestKnownGain = bestKnownGain;
    }

    protected override void ComputeGain()
    {
        _choice = ComputeBestSwapStarChoice(
            Input,
            SolutionState,
            SourceVehicle,
            Source,
            TargetVehicle,
            Target,
            _bestKnownGain);
            
        if (_choice.Gain.Cost > 0)
        {
            StoredGain = _choice.Gain;
        }
        
        GainComputed = true;
    }

    public override bool IsValid()
    {
        // Not supposed to be used in VROOM implementation
        throw new NotImplementedException("SwapStar.IsValid() should not be called");
    }

    public override void Apply()
    {
        var sValue = SourceRoute[_choice.SRank];
        var tValue = TargetRoute[_choice.TRank];

        // Apply changes to source route
        if (_choice.SRank == _choice.InsertionInSource)
        {
            SourceRoute[_choice.SRank] = tValue;
        }
        else
        {
            if (_choice.SRank < _choice.InsertionInSource)
            {
                // Shift jobs left and insert at end
                for (int i = _choice.SRank; i < _choice.InsertionInSource - 1; i++)
                {
                    SourceRoute[i] = SourceRoute[i + 1];
                }
                SourceRoute[_choice.InsertionInSource - 1] = tValue;
            }
            else
            {
                // Shift jobs right and insert at beginning
                for (int i = _choice.SRank; i > _choice.InsertionInSource; i--)
                {
                    SourceRoute[i] = SourceRoute[i - 1];
                }
                SourceRoute[_choice.InsertionInSource] = tValue;
            }
        }

        // Apply changes to target route
        if (_choice.TRank == _choice.InsertionInTarget)
        {
            TargetRoute[_choice.TRank] = sValue;
        }
        else
        {
            if (_choice.TRank < _choice.InsertionInTarget)
            {
                // Shift jobs left and insert at end
                for (int i = _choice.TRank; i < _choice.InsertionInTarget - 1; i++)
                {
                    TargetRoute[i] = TargetRoute[i + 1];
                }
                TargetRoute[_choice.InsertionInTarget - 1] = sValue;
            }
            else
            {
                // Shift jobs right and insert at beginning
                for (int i = _choice.TRank; i > _choice.InsertionInTarget; i--)
                {
                    TargetRoute[i] = TargetRoute[i - 1];
                }
                TargetRoute[_choice.InsertionInTarget] = sValue;
            }
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
}