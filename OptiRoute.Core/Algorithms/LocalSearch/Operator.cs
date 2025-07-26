using OptiRoute.Core.Models;

namespace OptiRoute.Core.Algorithms.LocalSearch;

/// <summary>
/// Base class for local search operators (based on VROOM).
/// </summary>
internal abstract class Operator
{
    protected readonly OperatorName Name;
    protected readonly Input Input;
    protected readonly SolutionState SolutionState;

    // Source of move for this operator
    protected readonly RawRoute Source;
    protected readonly List<int> SourceRoute;
    protected readonly int SourceVehicle;
    protected readonly int SourceRank;

    // Target of move for this operator
    protected readonly RawRoute Target;
    protected readonly List<int> TargetRoute;
    protected readonly int TargetVehicle;
    protected readonly int TargetRank;

    protected bool GainComputed { get; set; }
    protected Eval SourceGain { get; set; }
    protected Eval TargetGain { get; set; }
    protected Eval StoredGain { get; set; }

    protected Operator(
        OperatorName name,
        Input input,
        SolutionState solutionState,
        RawRoute sourceRoute,
        int sourceVehicle,
        int sourceRank,
        RawRoute targetRoute,
        int targetVehicle,
        int targetRank)
    {
        Name = name;
        Input = input;
        SolutionState = solutionState;
        Source = sourceRoute;
        SourceRoute = sourceRoute.Route;
        SourceVehicle = sourceVehicle;
        SourceRank = sourceRank;
        Target = targetRoute;
        TargetRoute = targetRoute.Route;
        TargetVehicle = targetVehicle;
        TargetRank = targetRank;
    }

    /// <summary>
    /// Compute the gain of this operator.
    /// </summary>
    protected abstract void ComputeGain();

    /// <summary>
    /// Get the gain value for this operator.
    /// </summary>
    public virtual Eval Gain()
    {
        if (!GainComputed)
        {
            ComputeGain();
        }
        return StoredGain;
    }

    /// <summary>
    /// Check whether this operator is valid for application.
    /// </summary>
    public abstract bool IsValid();

    /// <summary>
    /// Apply this operator to modify the routes.
    /// </summary>
    public abstract void Apply();

    /// <summary>
    /// Get candidates for addition.
    /// </summary>
    public abstract List<int> AdditionCandidates();

    /// <summary>
    /// Get candidates that need updating after this operator.
    /// </summary>
    public abstract List<int> UpdateCandidates();

    /// <summary>
    /// Get unassigned jobs required for this move.
    /// </summary>
    public virtual List<int> RequiredUnassigned()
    {
        return new List<int>();
    }

    /// <summary>
    /// Check if this move is invalidated by a change at given rank.
    /// </summary>
    public virtual bool InvalidatedBy(int rank)
    {
        return false;
    }

    /// <summary>
    /// Check if valid for source range bounds.
    /// </summary>
    protected bool IsValidForSourceRangeBounds()
    {
        var sourceVehicle = Input.Vehicles[SourceVehicle];
        var sourceEval = SolutionState.RouteEvals[SourceVehicle];
        return sourceVehicle.IsOkForRangeBounds(sourceEval - SourceGain);
    }

    /// <summary>
    /// Check if valid for target range bounds.
    /// </summary>
    protected bool IsValidForTargetRangeBounds()
    {
        var targetVehicle = Input.Vehicles[TargetVehicle];
        var targetEval = SolutionState.RouteEvals[TargetVehicle];
        return targetVehicle.IsOkForRangeBounds(targetEval - TargetGain);
    }

    /// <summary>
    /// Check if valid for range bounds (internal operators).
    /// </summary>
    protected bool IsValidForRangeBounds()
    {
        return IsValidForSourceRangeBounds();
    }
}

/// <summary>
/// Names of available local search operators.
/// </summary>
internal enum OperatorName
{
    UnassignedExchange,
    CrossExchange,
    MixedExchange,
    TwoOpt,
    ReverseTwoOpt,
    Relocate,
    OrOpt,
    IntraExchange,
    IntraCrossExchange,
    IntraMixedExchange,
    IntraRelocate,
    IntraOrOpt,
    IntraTwoOpt,
    PDShift,
    RouteExchange,
    SwapStar,
    RouteSplit,
    PriorityReplace,
    TSPFix
}