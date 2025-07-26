using OptiRoute.Core.Models;
using OptiRoute.Core.Problems.VRP;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.Heuristics;

/// <summary>
/// Solomon I1 insertion heuristic implementation (based on VROOM).
/// </summary>
internal static class SolomonI1
{
    /// <summary>
    /// Basic heuristic - fills routes using Solomon I1 insertion with regrets.
    /// </summary>
    public static Eval Basic<TRoute>(
        Input input,
        List<TRoute> routes,
        HashSet<int> unassigned,
        List<int> vehicleRanks,
        InitStrategy init,
        double lambda,
        SortStrategy sort) where TRoute : RawRoute, new()
    {
        // Ordering is based on vehicles description only so do not account
        // for initial routes if any.
        int nbVehicles = vehicleRanks.Count;

        // Sort vehicles based on strategy
        SortVehicles(input, vehicleRanks, sort);
        
        // Get job-vehicle evaluations matrix
        var evals = input.GetJobVehicleEvals();
        
        // regrets[v][j] holds the min cost for reaching job j in an empty
        // route across all remaining vehicles **after** vehicle at rank v
        var regrets = new List<Dictionary<int, long>>(nbVehicles);
        for (int i = 0; i < nbVehicles; i++)
        {
            regrets.Add(new Dictionary<int, long>());
        }
        
        // Use own cost for last vehicle regret values
        foreach (var j in unassigned)
        {
            regrets[nbVehicles - 1][j] = evals[j][vehicleRanks[nbVehicles - 1]].Cost;
        }
        
        // Compute regrets backward from second to last vehicle
        for (int revV = 0; revV < nbVehicles - 1; revV++)
        {
            // Going through vehicles backward from second to last
            int v = nbVehicles - 2 - revV;
            
            bool allCompatibleJobsLaterUndoable = true;
            foreach (var j in unassigned)
            {
                regrets[v][j] = Math.Min(regrets[v + 1][j], evals[j][vehicleRanks[v + 1]].Cost);
                if (input.VehicleOkWithJob(vehicleRanks[v], j) &&
                    regrets[v][j] < input.CostUpperBound)
                {
                    allCompatibleJobsLaterUndoable = false;
                }
            }
            
            if (allCompatibleJobsLaterUndoable)
            {
                // We don't want to use all regrets equal to the cost upper
                // bound in this situation: it would defeat the purpose of using
                // regrets in the first place as all lambda values would yield
                // the same choices. Using the same approach as with last vehicle.
                foreach (var j in unassigned)
                {
                    regrets[v][j] = evals[j][vehicleRanks[v]].Cost;
                }
            }
        }
        
        var solEval = new Eval(0, 0, 0);
        
        // Fill routes
        for (int v = 0; v < nbVehicles && unassigned.Count > 0; v++)
        {
            var vRank = vehicleRanks[v];
            var currentRoute = routes[vRank];
            
            if (currentRoute.Route.Count == 0 && init != InitStrategy.None)
            {
                // Trivial lambda for no additional job validity constraint
                Func<int, bool> jobNotOk = (jobRank) => false;
                
                SeedRoute(input, currentRoute, init, evals, unassigned, jobNotOk);
            }
            
            var currentEval = FillRoute(input, currentRoute, unassigned, regrets[v], lambda);
            solEval = solEval + currentEval;
        }
        
        return solEval;
    }
    
    /// <summary>
    /// Dynamic vehicle choice heuristic.
    /// </summary>
    public static Eval DynamicVehicleChoice<TRoute>(
        Input input,
        List<TRoute> routes,
        HashSet<int> unassigned,
        List<int> vehicleRanks,
        InitStrategy init,
        double lambda,
        SortStrategy sort) where TRoute : RawRoute, new()
    {
        var evals = input.GetJobVehicleEvals();
        var solEval = new Eval(0, 0, 0);
        
        while (vehicleRanks.Count > 0 && unassigned.Count > 0)
        {
            // For any unassigned job at j, jobs_min_costs[j]
            // (resp. jobs_second_min_costs[j]) holds the min cost
            // (resp. second min cost) of picking the job in an empty route
            // for any remaining vehicle.
            var jobsMinCosts = new Dictionary<int, long>();
            var jobsSecondMinCosts = new Dictionary<int, long>();
            
            foreach (var j in unassigned)
            {
                jobsMinCosts[j] = input.CostUpperBound;
                jobsSecondMinCosts[j] = input.CostUpperBound;
                
                foreach (var v in vehicleRanks)
                {
                    var cost = evals[j][v].Cost;
                    if (cost <= jobsMinCosts[j])
                    {
                        jobsSecondMinCosts[j] = jobsMinCosts[j];
                        jobsMinCosts[j] = cost;
                    }
                    else if (cost < jobsSecondMinCosts[j])
                    {
                        jobsSecondMinCosts[j] = cost;
                    }
                }
            }
            
            // Pick vehicle that has the biggest number of compatible
            // unassigned jobs closest to him than to any other different
            // vehicle still available.
            var closestJobsCount = new Dictionary<int, int>();
            foreach (var v in vehicleRanks)
            {
                closestJobsCount[v] = 0;
            }
            
            foreach (var j in unassigned)
            {
                foreach (var v in vehicleRanks)
                {
                    if (evals[j][v].Cost == jobsMinCosts[j])
                    {
                        closestJobsCount[v]++;
                    }
                }
            }
            
            // Choose vehicle based on sort strategy
            int vRank = ChooseVehicle(input, vehicleRanks, closestJobsCount, sort);
            vehicleRanks.Remove(vRank);
            
            // Compute regrets for chosen vehicle
            var regrets = new Dictionary<int, long>();
            bool allCompatibleJobsLaterUndoable = true;
            
            foreach (var j in unassigned)
            {
                if (jobsMinCosts[j] < evals[j][vRank].Cost)
                {
                    regrets[j] = jobsMinCosts[j];
                }
                else
                {
                    regrets[j] = jobsSecondMinCosts[j];
                }
                
                if (input.VehicleOkWithJob(vRank, j) &&
                    regrets[j] < input.CostUpperBound)
                {
                    allCompatibleJobsLaterUndoable = false;
                }
            }
            
            if (allCompatibleJobsLaterUndoable)
            {
                // Same approach as for basic heuristic
                foreach (var j in unassigned)
                {
                    regrets[j] = evals[j][vRank].Cost;
                }
            }
            
            var currentRoute = routes[vRank];
            
            if (currentRoute.Route.Count == 0 && init != InitStrategy.None)
            {
                Func<int, bool> jobNotOk = (jobRank) =>
                {
                    // One of the remaining vehicles is closest to that job
                    return jobsMinCosts[jobRank] < evals[jobRank][vRank].Cost;
                };
                
                SeedRoute(input, currentRoute, init, evals, unassigned, jobNotOk);
            }
            
            var currentEval = FillRoute(input, currentRoute, unassigned, regrets, lambda);
            solEval = solEval + currentEval;
        }
        
        return solEval;
    }
    
    private static void SortVehicles(Input input, List<int> vehicleRanks, SortStrategy sort)
    {
        switch (sort)
        {
            case SortStrategy.Availability:
                // Sort by decreasing "availability"
                vehicleRanks.Sort((lhs, rhs) =>
                {
                    var vLhs = input.Vehicles[lhs];
                    var vRhs = input.Vehicles[rhs];
                    return VehicleAvailabilityCompare(vLhs, vRhs);
                });
                break;
                
            case SortStrategy.Cost:
                // Sort by increasing fixed cost, then same as above
                vehicleRanks.Sort((lhs, rhs) =>
                {
                    var vLhs = input.Vehicles[lhs];
                    var vRhs = input.Vehicles[rhs];
                    
                    var costComp = vLhs.Costs.Fixed.CompareTo(vRhs.Costs.Fixed);
                    if (costComp != 0) return costComp;
                    
                    return VehicleAvailabilityCompare(vLhs, vRhs);
                });
                break;
        }
    }
    
    private static int VehicleAvailabilityCompare(Vehicle lhs, Vehicle rhs)
    {
        // Compare vehicles by "availability" - based on VROOM's operator<
        // This involves capacity, time window, max travel time, etc.
        
        // Compare capacity (larger is "less" in terms of availability ordering)
        var capComp = rhs.Capacity.CompareTo(lhs.Capacity);
        if (capComp != 0) return capComp;
        
        // Compare time window duration
        var twComp = rhs.TimeWindow.Duration.CompareTo(lhs.TimeWindow.Duration);
        if (twComp != 0) return twComp;
        
        // Compare max travel time
        var mttComp = Nullable.Compare(rhs.MaxTravelTime, lhs.MaxTravelTime);
        if (mttComp != 0) return mttComp;
        
        // Compare max distance
        return Nullable.Compare(rhs.MaxDistance, lhs.MaxDistance);
    }
    
    private static int ChooseVehicle(
        Input input,
        List<int> vehicleRanks,
        Dictionary<int, int> closestJobsCount,
        SortStrategy sort)
    {
        if (sort == SortStrategy.Availability)
        {
            return vehicleRanks.OrderByDescending(v => closestJobsCount[v])
                .ThenBy(v => input.Vehicles[v], new VehicleAvailabilityComparer())
                .First();
        }
        else // SortStrategy.Cost
        {
            return vehicleRanks.OrderByDescending(v => closestJobsCount[v])
                .ThenBy(v => input.Vehicles[v].Costs.Fixed)
                .ThenBy(v => input.Vehicles[v], new VehicleAvailabilityComparer())
                .First();
        }
    }
    
    private class VehicleAvailabilityComparer : IComparer<Vehicle>
    {
        public int Compare(Vehicle? x, Vehicle? y)
        {
            if (x == null || y == null) return 0;
            return VehicleAvailabilityCompare(x, y);
        }
    }
    
    private static List<Dictionary<int, long>> ComputeRegrets(
        Input input, 
        HashSet<int> unassigned, 
        List<int> vehicleRanks)
    {
        var nbVehicles = vehicleRanks.Count;
        var regrets = new List<Dictionary<int, long>>(nbVehicles);
        
        // Initialize regret dictionaries
        for (int i = 0; i < nbVehicles; i++)
        {
            regrets.Add(new Dictionary<int, long>());
        }
        
        // Use own cost for last vehicle
        foreach (var j in unassigned)
        {
            var lastVRank = vehicleRanks[^1];
            var eval = input.GetEval(lastVRank, 
                                    input.Vehicles[lastVRank].StartLocation != null ? 
                                        Array.IndexOf(input.Locations.ToArray(), input.Vehicles[lastVRank].StartLocation) : 0,
                                    Array.IndexOf(input.Locations.ToArray(), input.Jobs[j].Location));
            regrets[^1][j] = eval.Cost;
        }
        
        // Compute regrets for other vehicles
        for (int v = nbVehicles - 2; v >= 0; v--)
        {
            foreach (var j in unassigned)
            {
                var nextVRank = vehicleRanks[v + 1];
                var nextEval = input.GetEval(nextVRank,
                                           input.Vehicles[nextVRank].StartLocation != null ?
                                               Array.IndexOf(input.Locations.ToArray(), input.Vehicles[nextVRank].StartLocation) : 0,
                                           Array.IndexOf(input.Locations.ToArray(), input.Jobs[j].Location));
                
                regrets[v][j] = Math.Min(regrets[v + 1][j], nextEval.Cost);
            }
        }
        
        return regrets;
    }
    
    private static void SeedRoute<TRoute>(
        Input input,
        TRoute route,
        InitStrategy init,
        List<List<Eval>> evals,
        HashSet<int> unassigned,
        Func<int, bool> jobNotOk) where TRoute : RawRoute, new()
    {
        if (route.Route.Count != 0 || init == InitStrategy.None)
            return;

        var vRank = route.VehicleRank;
        var vehicle = input.Vehicles[vRank];

        // Initialize current route with the "best" valid job
        bool initOk = false;

        var higherAmount = input.ZeroAmount;
        long furthestCost = 0;
        long nearestCost = long.MaxValue;
        long earliestDeadline = long.MaxValue;
        int bestJobRank = -1;
        
        foreach (var jobRank in unassigned)
        {
            var currentJob = input.Jobs[jobRank];

            if (!input.VehicleOkWithJob(vRank, jobRank) ||
                currentJob.Type == JobType.Delivery || jobNotOk(jobRank))
            {
                continue;
            }

            bool isPickup = (currentJob.Type == JobType.Pickup);

            if (route.Route.Count + (isPickup ? 2 : 1) > vehicle.MaxTasks)
            {
                continue;
            }

            bool tryValidity = false;

            switch (init)
            {
                case InitStrategy.HigherAmount:
                    tryValidity = (higherAmount < currentJob.Pickup ||
                                  higherAmount < currentJob.Delivery);
                    break;
                case InitStrategy.EarliestDeadline:
                    var currentDeadline = isPickup ? 
                        (input.Jobs[jobRank + 1].TimeWindows.Count > 0 ? 
                            input.Jobs[jobRank + 1].TimeWindows.Last().End : long.MaxValue) :
                        (currentJob.TimeWindows.Count > 0 ? 
                            currentJob.TimeWindows.Last().End : long.MaxValue);
                    tryValidity = (currentDeadline < earliestDeadline);
                    break;
                case InitStrategy.Furthest:
                    tryValidity = (furthestCost < evals[jobRank][vRank].Cost);
                    break;
                case InitStrategy.Nearest:
                    tryValidity = (evals[jobRank][vRank].Cost < nearestCost);
                    break;
            }

            if (!tryValidity)
                continue;

            bool isValid = vehicle.IsOkForRangeBounds(evals[jobRank][vRank]) &&
                          route.IsValidAdditionForCapacity(currentJob.Pickup, currentJob.Delivery, 0);
            
            if (isPickup)
            {
                var pD = new[] { jobRank, jobRank + 1 };
                isValid = isValid && route.IsValidAdditionForTw(input, input.ZeroAmount,
                                                               pD, 0, pD.Length, 0, 0);
            }
            else if (currentJob.Type == JobType.Single)
            {
                isValid = isValid && route.IsValidAdditionForTw(input, jobRank, 0);
            }

            if (isValid)
            {
                initOk = true;
                bestJobRank = jobRank;

                switch (init)
                {
                    case InitStrategy.HigherAmount:
                        if (higherAmount < currentJob.Pickup)
                            higherAmount = currentJob.Pickup;
                        if (higherAmount < currentJob.Delivery)
                            higherAmount = currentJob.Delivery;
                        break;
                    case InitStrategy.EarliestDeadline:
                        earliestDeadline = isPickup ? 
                            (input.Jobs[jobRank + 1].TimeWindows.Count > 0 ? 
                                input.Jobs[jobRank + 1].TimeWindows.Last().End : long.MaxValue) :
                            (currentJob.TimeWindows.Count > 0 ? 
                                currentJob.TimeWindows.Last().End : long.MaxValue);
                        break;
                    case InitStrategy.Furthest:
                        furthestCost = evals[jobRank][vRank].Cost;
                        break;
                    case InitStrategy.Nearest:
                        nearestCost = evals[jobRank][vRank].Cost;
                        break;
                }
            }
        }

        if (initOk && bestJobRank >= 0)
        {
            if (input.Jobs[bestJobRank].Type == JobType.Single)
            {
                route.Add(input, bestJobRank, 0);
                unassigned.Remove(bestJobRank);
            }
            else if (input.Jobs[bestJobRank].Type == JobType.Pickup)
            {
                var pD = new[] { bestJobRank, bestJobRank + 1 };
                route.Replace(input, input.ZeroAmount, pD, 0, pD.Length, 0, 0);
                unassigned.Remove(bestJobRank);
                unassigned.Remove(bestJobRank + 1);
            }
        }
    }
    
    private static Eval FillRoute<TRoute>(
        Input input,
        TRoute route,
        HashSet<int> unassigned,
        Dictionary<int, long> regrets,
        double lambda) where TRoute : RawRoute
    {
        var vRank = route.VehicleRank;
        var vehicle = input.Vehicles[vRank];
        
        bool initRouteIsEmpty = route.Route.Count == 0;
        var routeEval = Helpers.RouteEvalForVehicle(input, vRank, route.Route);
        
        // Store bounds to be able to cut out some loops
        var unassignedCosts = new UnassignedCosts(input, route, unassigned);
        
        bool keepGoing = true;
        while (keepGoing && unassigned.Count > 0)
        {
            keepGoing = false;
            double bestCost = double.MaxValue;
            int bestJobRank = -1;
            int bestR = -1;
            int bestPickupR = -1;
            int bestDeliveryR = -1;
            Amount bestModifiedDelivery = input.ZeroAmount;
            Eval bestEval = Eval.NoEval;
            
            foreach (var jobRank in unassigned)
            {
                if (!input.VehicleOkWithJob(vRank, jobRank))
                    continue;
                
                var currentJob = input.Jobs[jobRank];
                
                if (currentJob.Type == JobType.Delivery)
                    continue;
                
                if (currentJob.Type == JobType.Single && 
                    route.Route.Count + 1 <= vehicle.MaxTasks)
                {
                    var regretValue = regrets.ContainsKey(jobRank) ? regrets[jobRank] : 0;
                    if (bestCost < unassignedCosts.GetInsertionLowerBound(jobRank) - lambda * regretValue)
                        continue;
                    
                    for (int r = 0; r <= route.Route.Count; r++)
                    {
                        var currentEval = Helpers.AdditionCost(input, jobRank, vehicle, route.Route, r);
                        
                        double currentCost = currentEval.Cost - lambda * regretValue;
                        
                        if (currentCost < bestCost &&
                            vehicle.IsOkForRangeBounds(routeEval + currentEval) &&
                            route.IsValidAdditionForCapacity(currentJob.Pickup, currentJob.Delivery, r) &&
                            route.IsValidAdditionForTw(input, jobRank, r))
                        {
                            bestCost = currentCost;
                            bestJobRank = jobRank;
                            bestR = r;
                            bestEval = currentEval;
                        }
                    }
                }
                
                if (currentJob.Type == JobType.Pickup &&
                    route.Route.Count + 2 <= vehicle.MaxTasks)
                {
                    var regretValue = regrets.ContainsKey(jobRank) ? regrets[jobRank] : 0;
                    if (bestCost < unassignedCosts.GetPdInsertionLowerBound(input, route, jobRank) - lambda * regretValue)
                        continue;
                    
                    // Pre-compute cost of addition for matching delivery
                    var dAdds = new List<Eval>(route.Route.Count + 1);
                    var validDeliveryInsertions = new List<bool>(route.Route.Count + 1);
                    
                    for (int dRank = 0; dRank <= route.Route.Count; dRank++)
                    {
                        dAdds.Add(Helpers.AdditionCost(input, jobRank + 1, vehicle, route.Route, dRank));
                        validDeliveryInsertions.Add(route.IsValidAdditionForTwWithoutMaxLoad(input, jobRank + 1, dRank));
                    }
                    
                    for (int pickupR = 0; pickupR <= route.Route.Count; pickupR++)
                    {
                        var pAdd = Helpers.AdditionCost(input, jobRank, vehicle, route.Route, pickupR);
                        
                        if (!route.IsValidAdditionForLoad(input, currentJob.Pickup, pickupR) ||
                            !route.IsValidAdditionForTwWithoutMaxLoad(input, jobRank, pickupR))
                        {
                            continue;
                        }
                        
                        // Build replacement sequence for current insertion
                        var modifiedWithPd = new List<int>();
                        modifiedWithPd.Add(jobRank);
                        
                        var modifiedDelivery = input.ZeroAmount;
                        
                        for (int deliveryR = pickupR; deliveryR <= route.Route.Count; deliveryR++)
                        {
                            // Update state variables along the way before potential early abort
                            if (pickupR < deliveryR)
                            {
                                modifiedWithPd.Add(route.Route[deliveryR - 1]);
                                var newModifiedJob = input.Jobs[route.Route[deliveryR - 1]];
                                if (newModifiedJob.Type == JobType.Single)
                                {
                                    modifiedDelivery = modifiedDelivery + newModifiedJob.Delivery;
                                }
                            }
                            
                            if (!validDeliveryInsertions[deliveryR])
                                continue;
                            
                            Eval currentEval;
                            if (pickupR == deliveryR)
                            {
                                currentEval = Helpers.AdditionCost(input, jobRank, vehicle, route.Route, pickupR, pickupR + 1);
                            }
                            else
                            {
                                currentEval = pAdd + dAdds[deliveryR];
                            }
                            
                            double currentCost = currentEval.Cost - lambda * regretValue;
                            
                            if (currentCost < bestCost)
                            {
                                modifiedWithPd.Add(jobRank + 1);
                                
                                // Update best cost depending on validity
                                bool valid = vehicle.IsOkForRangeBounds(routeEval + currentEval) &&
                                           route.IsValidAdditionForCapacityInclusion(
                                               modifiedDelivery,
                                               modifiedWithPd.ToArray(),
                                               0,
                                               modifiedWithPd.Count,
                                               pickupR,
                                               deliveryR) &&
                                           route.IsValidAdditionForTw(
                                               input,
                                               modifiedDelivery,
                                               modifiedWithPd.ToArray(),
                                               0,
                                               modifiedWithPd.Count,
                                               pickupR,
                                               deliveryR);
                                
                                modifiedWithPd.RemoveAt(modifiedWithPd.Count - 1);
                                
                                if (valid)
                                {
                                    bestCost = currentCost;
                                    bestJobRank = jobRank;
                                    bestPickupR = pickupR;
                                    bestDeliveryR = deliveryR;
                                    bestModifiedDelivery = modifiedDelivery;
                                    bestEval = currentEval;
                                }
                            }
                        }
                    }
                }
            }
            
            if (bestCost < double.MaxValue)
            {
                var bestJob = input.Jobs[bestJobRank];
                if (bestJob.Type == JobType.Single)
                {
                    route.Add(input, bestJobRank, bestR);
                    unassigned.Remove(bestJobRank);
                    keepGoing = true;
                    
                    unassignedCosts.UpdateMaxEdge(input, route);
                    unassignedCosts.UpdateMinCosts(input, route, unassigned, bestJob.Index);
                }
                else if (bestJob.Type == JobType.Pickup)
                {
                    var modifiedWithPd = new List<int>();
                    modifiedWithPd.Add(bestJobRank);
                    
                    for (int i = bestPickupR; i < bestDeliveryR; i++)
                    {
                        modifiedWithPd.Add(route.Route[i]);
                    }
                    modifiedWithPd.Add(bestJobRank + 1);
                    
                    route.Replace(input,
                                 bestModifiedDelivery,
                                 modifiedWithPd.ToArray(),
                                 0,
                                 modifiedWithPd.Count,
                                 bestPickupR,
                                 bestDeliveryR);
                    unassigned.Remove(bestJobRank);
                    unassigned.Remove(bestJobRank + 1);
                    keepGoing = true;
                    
                    unassignedCosts.UpdateMaxEdge(input, route);
                    unassignedCosts.UpdateMinCosts(input, route, unassigned, bestJob.Index);
                    unassignedCosts.UpdateMinCosts(input, route, unassigned, input.Jobs[bestJobRank + 1].Index);
                }
                
                routeEval = routeEval + bestEval;
            }
        }
        
        if (initRouteIsEmpty && route.Route.Count > 0)
        {
            // Account for fixed cost if we actually filled an empty route
            routeEval = routeEval with { Cost = routeEval.Cost + vehicle.Costs.Fixed };
        }
        
        return routeEval;
    }
    
    private static Eval CalculateInsertionCost<TRoute>(
        Input input,
        TRoute route,
        int jobRank,
        int insertionRank) where TRoute : IRoute
    {
        var vehicleIndex = route.GetVehicleIndex();
        var routeJobs = route.GetJobSequence();
        
        return SolomonI1Insertion.CalculateAdditionCost(
            input,
            jobRank,
            vehicleIndex,
            routeJobs,
            insertionRank);
    }
    
    private static bool IsValidInsertion<TRoute>(
        Input input,
        TRoute route,
        int jobRank,
        int insertionRank) where TRoute : IRoute
    {
        var job = input.Jobs[jobRank];
        var vehicleIndex = route.GetVehicleIndex();
        var vehicle = input.Vehicles[vehicleIndex];
        
        // Check vehicle constraints
        var insertionCost = CalculateInsertionCost(input, route, jobRank, insertionRank);
        
        // Check if vehicle can handle the additional cost
        // This would include checking max travel time, distance, etc.
        // For now, just check basic capacity
        
        if (route is RawRoute rawRoute)
        {
            return rawRoute.IsValidAdditionForCapacity(
                input.GetJobPickup(jobRank), 
                input.GetJobDelivery(jobRank), 
                insertionRank);
        }
        
        // Fallback to basic check
        var vehicleCapacity = vehicle.Capacity;
        var currentLoad = input.ZeroAmount; // Would need to track actual load
        
        return (currentLoad + job.Pickup <= vehicleCapacity) && 
               (currentLoad + job.Delivery <= vehicleCapacity);
    }
    
    private static void InsertJob<TRoute>(
        TRoute route,
        int jobRank,
        int insertionRank) where TRoute : IRoute
    {
        route.InsertJob(jobRank, insertionRank);
    }
    
}

/// <summary>
/// Tracks costs between route and unassigned jobs for efficient updates.
/// </summary>
internal class UnassignedCosts
{
    private readonly Vehicle _vehicle;
    private long _maxEdgeCost;
    private readonly long[] _minRouteToUnassigned;
    private readonly long[] _minUnassignedToRoute;
    
    public UnassignedCosts(Input input, RawRoute route, HashSet<int> unassigned)
    {
        _vehicle = input.Vehicles[route.VehicleRank];
        _maxEdgeCost = Helpers.MaxEdgeEval(input, _vehicle, route.Route).Cost;
        _minRouteToUnassigned = new long[input.Jobs.Count];
        _minUnassignedToRoute = new long[input.Jobs.Count];
        
        // Initialize with max values
        for (int i = 0; i < input.Jobs.Count; i++)
        {
            _minRouteToUnassigned[i] = long.MaxValue;
            _minUnassignedToRoute[i] = long.MaxValue;
        }
        
        foreach (var jobRank in unassigned)
        {
            var unassignedJobIndex = input.Jobs[jobRank].Index;
            
            if (_vehicle.StartLocation != null)
            {
                var startToJob = input.GetEval(
                    route.VehicleRank, _vehicle.StartLocation.Index, unassignedJobIndex).Cost;
                _minRouteToUnassigned[jobRank] = startToJob;
            }
            
            if (_vehicle.EndLocation != null)
            {
                var jobToEnd = input.GetEval(
                    route.VehicleRank, unassignedJobIndex, _vehicle.EndLocation.Index).Cost;
                _minUnassignedToRoute[jobRank] = jobToEnd;
            }
            
            foreach (var j in route.Route)
            {
                var jobIndex = input.Jobs[j].Index;
                
                var jobToUnassigned = input.GetEval(route.VehicleRank, jobIndex, unassignedJobIndex).Cost;
                _minRouteToUnassigned[jobRank] = Math.Min(
                    _minRouteToUnassigned[jobRank], jobToUnassigned);
                
                var unassignedToJob = input.GetEval(route.VehicleRank, unassignedJobIndex, jobIndex).Cost;
                _minUnassignedToRoute[jobRank] = Math.Min(
                    _minUnassignedToRoute[jobRank], unassignedToJob);
            }
        }
    }
    
    public double GetInsertionLowerBound(int jobRank)
    {
        return _minRouteToUnassigned[jobRank] + 
               _minUnassignedToRoute[jobRank] - 
               _maxEdgeCost;
    }
    
    public double GetPdInsertionLowerBound(Input input, RawRoute route, int pickupRank)
    {
        if (input.Jobs[pickupRank].Type != JobType.Pickup)
            throw new ArgumentException("Expected pickup job");
            
        // Situation where pickup and delivery are not inserted in a row
        var apartInsertion = (double)(
            _minRouteToUnassigned[pickupRank] + _minUnassignedToRoute[pickupRank] +
            _minRouteToUnassigned[pickupRank + 1] + _minUnassignedToRoute[pickupRank + 1] -
            2 * _maxEdgeCost);
        
        // Situation where delivery is inserted next to the pickup
        var nextInsertion = (double)(
            _minRouteToUnassigned[pickupRank] + _minUnassignedToRoute[pickupRank + 1] +
            input.GetEval(route.VehicleRank, input.Jobs[pickupRank].Index, input.Jobs[pickupRank + 1].Index).Cost -
            _maxEdgeCost);
        
        return Math.Min(apartInsertion, nextInsertion);
    }
    
    public void UpdateMaxEdge<TRoute>(Input input, TRoute route) where TRoute : RawRoute
    {
        _maxEdgeCost = Helpers.MaxEdgeEval(input, _vehicle, route.Route).Cost;
    }
    
    public void UpdateMinCosts(Input input, RawRoute route, HashSet<int> unassigned, int insertedIndex)
    {
        foreach (var j in unassigned)
        {
            var unassignedJobIndex = input.Jobs[j].Index;
            
            var toUnassigned = input.GetEval(route.VehicleRank, insertedIndex, unassignedJobIndex).Cost;
            _minRouteToUnassigned[j] = Math.Min(_minRouteToUnassigned[j], toUnassigned);
            
            var fromUnassigned = input.GetEval(route.VehicleRank, unassignedJobIndex, insertedIndex).Cost;
            _minUnassignedToRoute[j] = Math.Min(_minUnassignedToRoute[j], fromUnassigned);
        }
    }
    
    public void UpdateAfterInsertion<TRoute>(Input input, TRoute route, HashSet<int> unassigned, int insertedJobRank) 
        where TRoute : RawRoute
    {
        // Update max edge cost after insertion
        UpdateMaxEdge(input, route);
        
        // Update min costs with the newly inserted job
        var insertedJobIndex = input.Jobs[insertedJobRank].Index;
        UpdateMinCosts(input, route, unassigned, insertedJobIndex);
    }
    
}

// Extension to support delivery jobs
public class DeliveryJob : Job
{
    public DeliveryJob(string id, Location location) : base(id, location)
    {
    }
}