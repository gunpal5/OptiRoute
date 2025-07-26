namespace OptiRoute.Core.Algorithms.Heuristics;

/// <summary>
/// Parameters for heuristic algorithms (inspired by VROOM).
/// </summary>
public class HeuristicParameters
{
    /// <summary>
    /// Type of heuristic to use.
    /// </summary>
    public HeuristicType Heuristic { get; init; } = HeuristicType.Basic;

    /// <summary>
    /// Initialization strategy.
    /// </summary>
    public InitStrategy Init { get; init; } = InitStrategy.None;

    /// <summary>
    /// Regret coefficient for insertion decisions.
    /// </summary>
    public double RegretCoeff { get; init; } = 0.9;

    /// <summary>
    /// Vehicle sorting strategy.
    /// </summary>
    public SortStrategy Sort { get; init; } = SortStrategy.Availability;
}

/// <summary>
/// Type of heuristic algorithm.
/// </summary>
public enum HeuristicType
{
    Basic,
    Dynamic
}

/// <summary>
/// Job initialization strategy.
/// </summary>
public enum InitStrategy
{
    None,
    HigherAmount,
    Nearest,
    Furthest,
    EarliestDeadline
}

/// <summary>
/// Vehicle sorting strategy.
/// </summary>
public enum SortStrategy
{
    Availability,
    Cost
}