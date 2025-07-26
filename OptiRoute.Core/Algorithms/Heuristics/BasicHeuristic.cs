using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.VRP;

namespace OptiRoute.Core.Algorithms.Heuristics;

/// <summary>
/// Basic Solomon I1 heuristic implementation (based on VROOM).
/// </summary>
public static class BasicHeuristic<TRoute> where TRoute : RawRoute, new()
{
    public static Eval Solve(
        Input input,
        List<TRoute> routes,
        HashSet<int> unassigned,
        List<int> vehicleRanks,
        InitStrategy init,
        double regretCoeff,
        SortStrategy sort)
    {
        // Use the Solomon I1 implementation
        return SolomonI1.Basic(input, routes, unassigned, vehicleRanks, init, regretCoeff, sort);
    }
}