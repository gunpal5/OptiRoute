using OptiRoute.Core.Models;
using OptiRoute.Core.Utils;
using static OptiRoute.Core.Algorithms.LocalSearch.TopInsertions;

namespace OptiRoute.Core.Algorithms.LocalSearch;

/// <summary>
/// Utilities for SwapStar operator (based on VROOM).
/// </summary>
internal static class SwapStarUtils
{
    public struct SwapChoice
    {
        public Eval Gain { get; set; }
        public int SRank { get; set; }
        public int TRank { get; set; }
        public int InsertionInSource { get; set; }
        public int InsertionInTarget { get; set; }
        public Amount SourceRangeDelivery { get; set; }
        public Amount TargetRangeDelivery { get; set; }

        public SwapChoice(Eval gain, int sRank, int tRank, int insertionInSource, int insertionInTarget)
        {
            Gain = gain;
            SRank = sRank;
            TRank = tRank;
            InsertionInSource = insertionInSource;
            InsertionInTarget = insertionInTarget;
            SourceRangeDelivery = new Amount(0);
            TargetRangeDelivery = new Amount(0);
        }
    }

    public static readonly SwapChoice EmptySwapChoice = new() 
    { 
        Gain = Eval.NoEval, 
        SRank = 0, 
        TRank = 0, 
        InsertionInSource = 0, 
        InsertionInTarget = 0 
    };

    private struct InsertionRange
    {
        public List<int> Range { get; set; }
        public int FirstRank { get; set; }
        public int LastRank { get; set; }
    }

    /// <summary>
    /// Check if swap choice is valid given current insertion ranks.
    /// </summary>
    private static bool ValidChoiceForInsertionRanks(
        SolutionState solutionState,
        int sVehicle,
        RawRoute source,
        int tVehicle,
        RawRoute target,
        SwapChoice sc)
    {
        var sourceJobRank = source.Route[sc.SRank];
        var targetJobRank = target.Route[sc.TRank];

        // Complex validation logic from VROOM
        bool valid = solutionState.WeakInsertionRanksBegin[tVehicle][sourceJobRank] == sc.TRank + 1 ||
                     solutionState.WeakInsertionRanksBegin[tVehicle][sourceJobRank] <= sc.InsertionInTarget;

        valid = valid && (solutionState.WeakInsertionRanksEnd[tVehicle][sourceJobRank] == sc.TRank + 1 ||
                         sc.InsertionInTarget < solutionState.WeakInsertionRanksEnd[tVehicle][sourceJobRank]);

        valid = valid && (solutionState.WeakInsertionRanksBegin[sVehicle][targetJobRank] == sc.SRank + 1 ||
                         solutionState.WeakInsertionRanksBegin[sVehicle][targetJobRank] <= sc.InsertionInSource);

        valid = valid && (solutionState.WeakInsertionRanksEnd[sVehicle][targetJobRank] == sc.SRank + 1 ||
                         sc.InsertionInSource < solutionState.WeakInsertionRanksEnd[sVehicle][targetJobRank]);

        valid = valid && (sc.TRank < sc.InsertionInTarget ||
                         sc.InsertionInTarget < solutionState.InsertionRanksEnd[tVehicle][sourceJobRank]);

        valid = valid && (sc.SRank < sc.InsertionInSource ||
                         sc.InsertionInSource < solutionState.InsertionRanksEnd[sVehicle][targetJobRank]);

        valid = valid && (sc.TRank >= sc.InsertionInTarget ||
                         solutionState.InsertionRanksBegin[tVehicle][sourceJobRank] <= sc.InsertionInTarget);

        valid = valid && (sc.SRank >= sc.InsertionInSource ||
                         solutionState.InsertionRanksBegin[sVehicle][targetJobRank] <= sc.InsertionInSource);

        return valid;
    }

    /// <summary>
    /// Compute insertion range when removing job at sRank and adding job at insertion rank.
    /// </summary>
    private static InsertionRange GetInsertRange(List<int> sRoute, int sRank, int jobRank, int insertionRank)
    {
        var insert = new InsertionRange { Range = new List<int>() };

        if (sRank == insertionRank)
        {
            insert.Range.Add(jobRank);
            insert.FirstRank = sRank;
            insert.LastRank = sRank + 1;
        }
        else
        {
            if (sRank < insertionRank)
            {
                for (int i = sRank + 1; i < insertionRank; i++)
                {
                    insert.Range.Add(sRoute[i]);
                }
                insert.Range.Add(jobRank);
                insert.FirstRank = sRank;
                insert.LastRank = insertionRank;
            }
            else
            {
                insert.Range.Add(jobRank);
                for (int i = insertionRank; i < sRank; i++)
                {
                    insert.Range.Add(sRoute[i]);
                }
                insert.FirstRank = insertionRank;
                insert.LastRank = sRank + 1;
            }
        }

        return insert;
    }

    /// <summary>
    /// Compute best swap star choice between two routes.
    /// </summary>
    public static SwapChoice ComputeBestSwapStarChoice(
        Input input,
        SolutionState solutionState,
        int sVehicle,
        RawRoute source,
        int tVehicle,
        RawRoute target,
        Eval bestKnownGain)
    {
        // Preprocessing phase - find top insertions for each job
        var topInsertionsInTarget = new InsertionOption[source.Route.Count][];
        for (int sRank = 0; sRank < source.Route.Count; sRank++)
        {
            var sourceJobRank = source.Route[sRank];
            
            if (input.Jobs[sourceJobRank].Type == JobType.Single &&
                input.VehicleOkWithJob(tVehicle, sourceJobRank))
            {
                topInsertionsInTarget[sRank] = FindTop3Insertions(input, sourceJobRank, target);
            }
            else
            {
                topInsertionsInTarget[sRank] = new[] { NoInsert, NoInsert, NoInsert };
            }
        }

        var topInsertionsInSource = new InsertionOption[target.Route.Count][];
        for (int tRank = 0; tRank < target.Route.Count; tRank++)
        {
            var targetJobRank = target.Route[tRank];
            
            if (input.Jobs[targetJobRank].Type == JobType.Single &&
                input.VehicleOkWithJob(sVehicle, targetJobRank))
            {
                topInsertionsInSource[tRank] = FindTop3Insertions(input, targetJobRank, source);
            }
            else
            {
                topInsertionsInSource[tRank] = new[] { NoInsert, NoInsert, NoInsert };
            }
        }

        // Search phase
        var bestChoice = EmptySwapChoice;
        var bestGain = bestKnownGain;

        var sVehicleObj = input.Vehicles[sVehicle];
        var tVehicleObj = input.Vehicles[tVehicle];

        var sEval = solutionState.RouteEvals[sVehicle];
        var tEval = solutionState.RouteEvals[tVehicle];

        var sDeliveryMargin = source.DeliveryMargin();
        var sPickupMargin = source.PickupMargin();
        var tDeliveryMargin = target.DeliveryMargin();
        var tPickupMargin = target.PickupMargin();

        for (int sRank = 0; sRank < source.Route.Count; sRank++)
        {
            var targetInsertions = topInsertionsInTarget[sRank];
            if (targetInsertions[0].Cost == Eval.NoEval)
                continue;

            // Handle special case of single-step route with start and end
            var sourceStartEndCost = new Eval(0, 0, 0);
            if (source.Route.Count == 1 && sVehicleObj.StartLocation != null && sVehicleObj.EndLocation != null)
            {
                var startIdx = Array.IndexOf(input.Locations.ToArray(), sVehicleObj.StartLocation);
                var endIdx = Array.IndexOf(input.Locations.ToArray(), sVehicleObj.EndLocation);
                sourceStartEndCost = input.GetEval(sVehicle, startIdx, endIdx);
            }
            
            var sourceDelta = solutionState.NodeGains[sVehicle][sRank] - sourceStartEndCost;

            for (int tRank = 0; tRank < target.Route.Count; tRank++)
            {
                var sourceInsertions = topInsertionsInSource[tRank];
                if (sourceInsertions[0].Cost == Eval.NoEval)
                    continue;

                // Handle special case for target route
                var targetStartEndCost = new Eval(0, 0, 0);
                if (target.Route.Count == 1 && tVehicleObj.StartLocation != null && tVehicleObj.EndLocation != null)
                {
                    var startIdx = Array.IndexOf(input.Locations.ToArray(), tVehicleObj.StartLocation);
                    var endIdx = Array.IndexOf(input.Locations.ToArray(), tVehicleObj.EndLocation);
                    targetStartEndCost = input.GetEval(tVehicle, startIdx, endIdx);
                }
                
                var targetDelta = solutionState.NodeGains[tVehicle][tRank] - targetStartEndCost;

                if ((sourceDelta + targetDelta).Cost <= bestGain.Cost)
                    continue;

                var targetInPlaceDelta = Helpers.InPlaceDeltaCost(
                    input, source.Route[sRank], tVehicleObj, target.Route, tRank);
                    
                var sourceInPlaceDelta = Helpers.InPlaceDeltaCost(
                    input, target.Route[tRank], sVehicleObj, source.Route, sRank);

                var swapChoiceOptions = new List<SwapChoice>();

                // In-place insertions
                var inPlaceSGain = sourceDelta - sourceInPlaceDelta;
                var inPlaceTGain = targetDelta - targetInPlaceDelta;
                var currentGain = inPlaceSGain + inPlaceTGain;

                if (sVehicleObj.IsOkForRangeBounds(sEval - inPlaceSGain))
                {
                    if (bestGain.Cost < currentGain.Cost && 
                        tVehicleObj.IsOkForRangeBounds(tEval - inPlaceTGain))
                    {
                        var sc = new SwapChoice(currentGain, sRank, tRank, sRank, tRank);
                        if (ValidChoiceForInsertionRanks(solutionState, sVehicle, source, tVehicle, target, sc))
                        {
                            swapChoiceOptions.Add(sc);
                        }
                    }

                    // Try other target insertions
                    foreach (var ti in targetInsertions)
                    {
                        if (ti.Rank != tRank && ti.Rank != tRank + 1 && ti.Cost != Eval.NoEval)
                        {
                            var tGain = targetDelta - ti.Cost;
                            currentGain = inPlaceSGain + tGain;
                            if (bestGain.Cost < currentGain.Cost &&
                                tVehicleObj.IsOkForRangeBounds(tEval - tGain))
                            {
                                var sc = new SwapChoice(currentGain, sRank, tRank, sRank, ti.Rank);
                                if (ValidChoiceForInsertionRanks(solutionState, sVehicle, source, tVehicle, target, sc))
                                {
                                    swapChoiceOptions.Add(sc);
                                }
                            }
                        }
                    }
                }

                // Try other source insertions
                foreach (var si in sourceInsertions)
                {
                    if (si.Rank != sRank && si.Rank != sRank + 1 && si.Cost != Eval.NoEval)
                    {
                        var sGain = sourceDelta - si.Cost;
                        
                        if (!sVehicleObj.IsOkForRangeBounds(sEval - sGain))
                            continue;

                        currentGain = sGain + inPlaceTGain;
                        if (bestGain.Cost < currentGain.Cost &&
                            tVehicleObj.IsOkForRangeBounds(tEval - inPlaceTGain))
                        {
                            var sc = new SwapChoice(currentGain, sRank, tRank, si.Rank, tRank);
                            if (ValidChoiceForInsertionRanks(solutionState, sVehicle, source, tVehicle, target, sc))
                            {
                                swapChoiceOptions.Add(sc);
                            }
                        }

                        foreach (var ti in targetInsertions)
                        {
                            if (ti.Rank != tRank && ti.Rank != tRank + 1 && ti.Cost != Eval.NoEval)
                            {
                                var tGain = targetDelta - ti.Cost;
                                currentGain = sGain + tGain;
                                if (bestGain.Cost < currentGain.Cost &&
                                    tVehicleObj.IsOkForRangeBounds(tEval - tGain))
                                {
                                    var sc = new SwapChoice(currentGain, sRank, tRank, si.Rank, ti.Rank);
                                    if (ValidChoiceForInsertionRanks(solutionState, sVehicle, source, tVehicle, target, sc))
                                    {
                                        swapChoiceOptions.Add(sc);
                                    }
                                }
                            }
                        }
                    }
                }

                // Sort options by decreasing gain
                swapChoiceOptions.Sort((a, b) => b.Gain.Cost.CompareTo(a.Gain.Cost));

                // Check validity of best options
                foreach (var sc in swapChoiceOptions)
                {
                    // Early abort on invalid capacity bounds
                    var sIndex = source.Route[sc.SRank];
                    var sDelivery = input.GetJobDelivery(sIndex);
                    var sPickup = input.GetJobPickup(sIndex);
                    var tIndex = target.Route[sc.TRank];
                    var tDelivery = input.GetJobDelivery(tIndex);
                    var tPickup = input.GetJobPickup(tIndex);

                    if (!(tDelivery <= sDeliveryMargin + sDelivery) ||
                        !(tPickup <= sPickupMargin + sPickup) ||
                        !(sDelivery <= tDeliveryMargin + tDelivery) ||
                        !(sPickup <= tPickupMargin + tPickup))
                    {
                        continue;
                    }

                    // Check detailed validity
                    var sInsert = GetInsertRange(source.Route, sRank, target.Route[tRank], sc.InsertionInSource);
                    
                    var sourcePickup = input.ZeroAmount;
                    var sourceDelivery = input.ZeroAmount;
                    foreach (var i in sInsert.Range)
                    {
                        var job = input.Jobs[i];
                        if (job.Type == JobType.Single)
                        {
                            sourcePickup = sourcePickup + job.Pickup;
                            sourceDelivery = sourceDelivery + job.Delivery;
                        }
                    }

                    bool sourceValid = source.IsValidAdditionForCapacityMargins(
                        sourcePickup, sourceDelivery, sInsert.FirstRank, sInsert.LastRank);

                    if (sourceValid)
                    {
                        sourceValid = source.IsValidAdditionForCapacityInclusion(
                            sourceDelivery,
                            sInsert.Range.ToArray(),
                            0,
                            sInsert.Range.Count,
                            sInsert.FirstRank,
                            sInsert.LastRank);
                    }

                    if (sourceValid)
                    {
                        sourceValid = source.IsValidAdditionForTw(
                            input,
                            sourceDelivery,
                            sInsert.Range.ToArray(),
                            0,
                            sInsert.Range.Count,
                            sInsert.FirstRank,
                            sInsert.LastRank);
                    }

                    if (sourceValid)
                    {
                        var tInsert = GetInsertRange(target.Route, tRank, source.Route[sRank], sc.InsertionInTarget);
                        
                        var targetPickup = input.ZeroAmount;
                        var targetDelivery = input.ZeroAmount;
                        foreach (var i in tInsert.Range)
                        {
                            var job = input.Jobs[i];
                            if (job.Type == JobType.Single)
                            {
                                targetPickup = targetPickup + job.Pickup;
                                targetDelivery = targetDelivery + job.Delivery;
                            }
                        }

                        bool targetValid = target.IsValidAdditionForCapacityMargins(
                            targetPickup, targetDelivery, tInsert.FirstRank, tInsert.LastRank);

                        if (targetValid)
                        {
                            targetValid = target.IsValidAdditionForCapacityInclusion(
                                targetDelivery,
                                tInsert.Range.ToArray(),
                                0,
                                tInsert.Range.Count,
                                tInsert.FirstRank,
                                tInsert.LastRank);
                        }

                        if (targetValid)
                        {
                            targetValid = target.IsValidAdditionForTw(
                                input,
                                targetDelivery,
                                tInsert.Range.ToArray(),
                                0,
                                tInsert.Range.Count,
                                tInsert.FirstRank,
                                tInsert.LastRank);
                        }

                        if (targetValid)
                        {
                            bestGain = sc.Gain;
                            bestChoice = sc;
                            bestChoice.SourceRangeDelivery = sourceDelivery;
                            bestChoice.TargetRangeDelivery = targetDelivery;
                            // First valid option has best gain
                            break;
                        }
                    }
                }
            }
        }

        return bestChoice;
    }
}