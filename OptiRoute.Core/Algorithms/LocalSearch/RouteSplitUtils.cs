using OptiRoute.Core.Models;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch;

/// <summary>
/// Utilities for route splitting operations (based on VROOM).
/// </summary>
internal static class RouteSplitUtils
{
    public struct SplitChoice
    {
        public Eval Gain { get; set; }
        public int SplitRank { get; set; }
        // Vehicle ranks are relative to empty_route_ranks
        public int VBegin { get; set; }
        public int VEnd { get; set; }
    }

    public static readonly SplitChoice EmptyRouteSplitChoice = new()
    {
        Gain = Eval.NoEval,
        SplitRank = 0,
        VBegin = 0,
        VEnd = 0
    };

    /// <summary>
    /// Compute the best route split choice for a given route.
    /// </summary>
    public static SplitChoice ComputeBestRouteSplitChoice(
        Input input,
        SolutionState solutionState,
        int sourceVehicle,
        RawRoute source,
        List<int> emptyRouteRanks,
        Eval bestKnownGain)
    {
        var bestChoice = EmptyRouteSplitChoice;

        // Create actual empty routes for idle vehicles to use in validity checks
        var emptyRoutes = new List<RawRoute>();
        foreach (var v in emptyRouteRanks)
        {
            emptyRoutes.Add(new RawRoute(input, v));
        }

        for (int r = 1; r < source.Route.Count; r++)
        {
            // Starting at 1 to split in two "real" routes
            // "Begin" route from start up to r (excluded)
            // "End" route from r to the end

            if (source.HasPendingDeliveryAfterRank(r - 1))
            {
                continue;
            }

            var firstBestEndEval = Eval.NoEval;
            int firstVEnd = 0;
            var secondBestEndEval = Eval.NoEval;
            int secondVEnd = 0;

            var endMaxLoad = source.SubRouteMaxLoadAfter(r);
            var endDelivery = source.DeliveryInRange(r, source.Route.Count);

            // Find best vehicle for end route
            for (int vRank = 0; vRank < emptyRouteRanks.Count; vRank++)
            {
                var v = emptyRouteRanks[vRank];
                var endVehicle = input.Vehicles[v];

                if (r < solutionState.BwdSkillRank[sourceVehicle][v] ||
                    !(endMaxLoad <= input.GetVehicleCapacity(v)) ||
                    endVehicle.MaxTasks < source.Route.Count - r)
                {
                    continue;
                }

                var currentEndEval = Helpers.AdditionCostDelta(
                    input,
                    solutionState,
                    emptyRoutes[vRank],
                    0,
                    0,
                    source,
                    r,
                    source.Route.Count).Item1;
                currentEndEval = new Eval(-currentEndEval.Cost, -currentEndEval.Duration, -currentEndEval.Distance);

                if (!endVehicle.IsOkForRangeBounds(currentEndEval))
                {
                    continue;
                }

                if (currentEndEval.Cost < secondBestEndEval.Cost &&
                    emptyRoutes[vRank].IsValidAdditionForTw(
                        input,
                        endDelivery,
                        source.Route.ToArray(),
                        r,
                        source.Route.Count,
                        0,
                        0))
                {
                    if (currentEndEval.Cost < firstBestEndEval.Cost)
                    {
                        // New first choice
                        secondVEnd = firstVEnd;
                        secondBestEndEval = firstBestEndEval;

                        firstVEnd = vRank;
                        firstBestEndEval = currentEndEval;
                    }
                    else
                    {
                        // New second choice
                        secondVEnd = vRank;
                        secondBestEndEval = currentEndEval;
                    }
                }
            }

            if (firstBestEndEval == Eval.NoEval)
            {
                // End route is valid for none of the empty vehicles
                continue;
            }

            var initEval = solutionState.RouteEvals[sourceVehicle];
            if ((initEval - firstBestEndEval).Cost <= bestKnownGain.Cost)
            {
                // Overall gain will be even lower with begin route cost
                continue;
            }

            var firstBestBeginEval = Eval.NoEval;
            int firstVBegin = 0;
            var secondBestBeginEval = Eval.NoEval;
            int secondVBegin = 0;

            var beginMaxLoad = source.SubRouteMaxLoadBefore(r);
            var beginDelivery = source.DeliveryInRange(0, r);

            // Find best vehicle for begin route
            for (int vRank = 0; vRank < emptyRouteRanks.Count; vRank++)
            {
                var v = emptyRouteRanks[vRank];
                var beginVehicle = input.Vehicles[v];

                if (solutionState.FwdSkillRank[sourceVehicle][v] < r ||
                    !(beginMaxLoad <= input.GetVehicleCapacity(v)) ||
                    beginVehicle.MaxTasks < r)
                {
                    continue;
                }

                var currentBeginEval = Helpers.AdditionCostDelta(
                    input,
                    solutionState,
                    emptyRoutes[vRank],
                    0,
                    0,
                    source,
                    0,
                    r).Item1;
                currentBeginEval = new Eval(-currentBeginEval.Cost, -currentBeginEval.Duration, -currentBeginEval.Distance);

                if (!beginVehicle.IsOkForRangeBounds(currentBeginEval))
                {
                    continue;
                }

                if (currentBeginEval.Cost < secondBestBeginEval.Cost &&
                    emptyRoutes[vRank].IsValidAdditionForTw(
                        input,
                        beginDelivery,
                        source.Route.ToArray(),
                        0,
                        r,
                        0,
                        0))
                {
                    if (currentBeginEval.Cost < firstBestBeginEval.Cost)
                    {
                        // New first choice
                        secondVBegin = firstVBegin;
                        secondBestBeginEval = firstBestBeginEval;

                        firstVBegin = vRank;
                        firstBestBeginEval = currentBeginEval;
                    }
                    else
                    {
                        // New second choice
                        secondVBegin = vRank;
                        secondBestBeginEval = currentBeginEval;
                    }
                }
            }

            if (firstBestBeginEval == Eval.NoEval)
            {
                // Begin route is valid for none of the empty vehicles
                continue;
            }

            // Now we have at least one valid candidate for begin and end route
            SplitChoice currentSplitChoice;

            if (firstVBegin != firstVEnd)
            {
                currentSplitChoice = new SplitChoice
                {
                    Gain = initEval - firstBestBeginEval - firstBestEndEval,
                    SplitRank = r,
                    VBegin = firstVBegin,
                    VEnd = firstVEnd
                };
            }
            else
            {
                // Candidates are identical, check second bests
                if (secondBestBeginEval == Eval.NoEval)
                {
                    if (secondBestEndEval == Eval.NoEval)
                    {
                        // No split possible - only one valid vehicle
                        continue;
                    }
                    currentSplitChoice = new SplitChoice
                    {
                        Gain = initEval - firstBestBeginEval - secondBestEndEval,
                        SplitRank = r,
                        VBegin = firstVBegin,
                        VEnd = secondVEnd
                    };
                }
                else
                {
                    if (secondBestEndEval == Eval.NoEval)
                    {
                        currentSplitChoice = new SplitChoice
                        {
                            Gain = initEval - secondBestBeginEval - firstBestEndEval,
                            SplitRank = r,
                            VBegin = secondVBegin,
                            VEnd = firstVEnd
                        };
                    }
                    else
                    {
                        // We have second bests for both, check best option
                        if ((firstBestBeginEval + secondBestEndEval).Cost < 
                            (secondBestBeginEval + firstBestEndEval).Cost)
                        {
                            currentSplitChoice = new SplitChoice
                            {
                                Gain = initEval - firstBestBeginEval - secondBestEndEval,
                                SplitRank = r,
                                VBegin = firstVBegin,
                                VEnd = secondVEnd
                            };
                        }
                        else
                        {
                            currentSplitChoice = new SplitChoice
                            {
                                Gain = initEval - secondBestBeginEval - firstBestEndEval,
                                SplitRank = r,
                                VBegin = secondVBegin,
                                VEnd = firstVEnd
                            };
                        }
                    }
                }
            }

            if (currentSplitChoice.Gain.Cost > bestChoice.Gain.Cost)
            {
                bestChoice = currentSplitChoice;
            }
        }

        return bestChoice;
    }
}