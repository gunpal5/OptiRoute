namespace OptiRoute.Core.Models;

/// <summary>
/// Represents an evaluation of cost, duration, and distance (inspired by VROOM's Eval).
/// </summary>
public readonly record struct Eval : IComparable<Eval>
{
    public long Cost { get; init; }
    public long Duration { get; init; }
    public long Distance { get; init; }

    public Eval(long cost, long duration = 0, long distance = 0)
    {
        Cost = cost;
        Duration = duration;
        Distance = distance;
    }

    public static Eval operator +(Eval a, Eval b) =>
        new(a.Cost + b.Cost, a.Duration + b.Duration, a.Distance + b.Distance);

    public static Eval operator -(Eval a, Eval b) =>
        new(a.Cost - b.Cost, a.Duration - b.Duration, a.Distance - b.Distance);

    public static Eval operator -(Eval eval) =>
        new(-eval.Cost, -eval.Duration, -eval.Distance);

    public int CompareTo(Eval other)
    {
        var costComparison = Cost.CompareTo(other.Cost);
        if (costComparison != 0) return costComparison;
        
        var durationComparison = Duration.CompareTo(other.Duration);
        if (durationComparison != 0) return durationComparison;
        
        return Distance.CompareTo(other.Distance);
    }

    public static bool operator <(Eval a, Eval b) => a.CompareTo(b) < 0;
    public static bool operator >(Eval a, Eval b) => a.CompareTo(b) > 0;
    public static bool operator <=(Eval a, Eval b) => a.Cost <= b.Cost;
    public static bool operator >=(Eval a, Eval b) => a.Cost >= b.Cost;

    public static readonly Eval NoEval = new(long.MaxValue, 0, 0);
    public static readonly Eval NoGain = new(long.MinValue, 0, 0);
}