using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.VRP;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch;

/// <summary>
/// Utilities for finding top insertion positions (based on VROOM).
/// </summary>
internal static class TopInsertions
{
    public struct InsertionOption
    {
        public Eval Cost { get; set; }
        public int Rank { get; set; }
    }

    public static readonly InsertionOption NoInsert = new() { Cost = Eval.NoEval, Rank = 0 };

    /// <summary>
    /// Find top 3 insertion positions for job j in route.
    /// </summary>
    public static InsertionOption[] FindTop3Insertions(Input input, int jobIndex, RawRoute route)
    {
        var result = new[] { NoInsert, NoInsert, NoInsert };
        
        if (route.Route.Count == 0)
        {
            // Only one insertion position in empty route
            var cost = Helpers.AdditionCost(input, jobIndex, input.Vehicles[route.VehicleRank], route.Route, 0);
            result[0] = new InsertionOption { Cost = cost, Rank = 0 };
            return result;
        }
        
        var vehicle = input.Vehicles[route.VehicleRank];
        var options = new List<InsertionOption>();
        
        // Try all insertion positions
        for (int rank = 0; rank <= route.Route.Count; rank++)
        {
            var cost = Helpers.AdditionCost(input, jobIndex, vehicle, route.Route, rank);
            options.Add(new InsertionOption { Cost = cost, Rank = rank });
        }
        
        // Sort by cost and take top 3
        options.Sort((a, b) => a.Cost.Cost.CompareTo(b.Cost.Cost));
        
        for (int i = 0; i < Math.Min(3, options.Count); i++)
        {
            result[i] = options[i];
        }
        
        return result;
    }
}