using OptiRoute.Core.Models;
using OptiRoute.Core.Problems.VRP;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.Heuristics;

/// <summary>
/// Dynamic vehicle choice heuristic for VRP (based on VROOM's dynamic_vehicle_choice).
/// Iteratively chooses the most relevant vehicle, then build a route using regrets.
/// </summary>
public static class DynamicHeuristic<TRoute> where TRoute : IRoute, new()
{
    public static Eval Solve(
        Input input,
        List<TRoute> routes,
        HashSet<int> unassignedSet,
        List<int> vehicleRanks,
        InitStrategy init,
        double lambda,
        SortStrategy sort)
    {
        var unassigned = new HashSet<int>(unassignedSet);
        var availableVehicles = new List<int>(vehicleRanks);
        var solEval = new Eval(0, 0, 0);
        var jobsVehicleEvals = ComputeJobsVehicleEvals(input);

        while (availableVehicles.Count > 0 && unassigned.Count > 0)
        {
            // For any unassigned job, compute min cost and second min cost
            // of picking the job in an empty route for any remaining vehicle
            var jobsMinCosts = new long[input.Jobs.Count];
            var jobsSecondMinCosts = new long[input.Jobs.Count];
            
            // Initialize with upper bound
            var upperBound = GetCostUpperBound();
            for (int i = 0; i < input.Jobs.Count; i++)
            {
                jobsMinCosts[i] = upperBound;
                jobsSecondMinCosts[i] = upperBound;
            }

            foreach (var j in unassigned)
            {
                foreach (var v in availableVehicles)
                {
                    var cost = jobsVehicleEvals[j][v].Cost;
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
            // unassigned jobs closest to it than to any other vehicle
            var closestJobsCount = new int[input.Vehicles.Count];
            foreach (var j in unassigned)
            {
                foreach (var v in availableVehicles)
                {
                    if (jobsVehicleEvals[j][v].Cost == jobsMinCosts[j])
                    {
                        closestJobsCount[v]++;
                    }
                }
            }

            // Choose vehicle based on sort criteria
            int vRank;
            if (sort == SortStrategy.Availability)
            {
                vRank = availableVehicles
                    .OrderByDescending(v => closestJobsCount[v])
                    .ThenBy(v => input.Vehicles[v].Id)
                    .First();
            }
            else // SortStrategy.Cost
            {
                vRank = availableVehicles
                    .OrderByDescending(v => closestJobsCount[v])
                    .ThenBy(v => GetVehicleCostRank(input, v))
                    .ThenBy(v => input.Vehicles[v].Id)
                    .First();
            }
            
            availableVehicles.Remove(vRank);

            // Compute regrets for remaining jobs
            var regrets = new long[input.Jobs.Count];
            for (int i = 0; i < input.Jobs.Count; i++)
            {
                regrets[i] = upperBound;
            }

            bool allCompatibleJobsLaterUndoable = true;
            foreach (var j in unassigned)
            {
                if (jobsMinCosts[j] < jobsVehicleEvals[j][vRank].Cost)
                {
                    regrets[j] = jobsMinCosts[j];
                }
                else
                {
                    regrets[j] = jobsSecondMinCosts[j];
                }

                if (VehicleOkWithJob(input, vRank, j) && regrets[j] < upperBound)
                {
                    allCompatibleJobsLaterUndoable = false;
                }
            }

            if (allCompatibleJobsLaterUndoable)
            {
                // Same approach as for basic heuristic
                foreach (var j in unassigned)
                {
                    regrets[j] = jobsVehicleEvals[j][vRank].Cost;
                }
            }

            var currentRoute = routes[vRank];

            // Seed route if empty and init type specified
            if (currentRoute.Route.Count == 0 && init != InitStrategy.None)
            {
                SeedRoute(input, currentRoute, init, unassigned, jobsMinCosts, jobsVehicleEvals, vRank);
            }

            // Fill route using regret-based insertion
            var currentEval = FillRoute(input, currentRoute, unassigned, regrets, lambda);
            solEval = new Eval(
                solEval.Cost + currentEval.Cost,
                solEval.Duration + currentEval.Duration,
                solEval.Distance + currentEval.Distance);
        }

        return solEval;
    }

    /// <summary>
    /// Computes job-vehicle evaluation matrix.
    /// </summary>
    private static List<List<Eval>> ComputeJobsVehicleEvals(Input input)
    {
        var evals = new List<List<Eval>>(input.Jobs.Count);
        
        for (int j = 0; j < input.Jobs.Count; j++)
        {
            evals.Add(new List<Eval>(input.Vehicles.Count));
            for (int v = 0; v < input.Vehicles.Count; v++)
            {
                if (!VehicleOkWithJob(input, v, j))
                {
                    evals[j].Add(new Eval(GetCostUpperBound(), 0, 0));
                }
                else
                {
                    // Compute cost for single job in empty route
                    var vehicle = input.Vehicles[v];
                    var job = input.Jobs[j];
                    
                    // From start to job
                    var startEval = input.GetEval(input.GetVehicleStartIndex(v), input.GetJobLocationIndex(j), v);
                    // From job to end
                    var endEval = input.GetEval(input.GetJobLocationIndex(j), input.GetVehicleEndIndex(v), v);
                    
                    var totalEval = new Eval(
                        startEval.Cost + endEval.Cost,
                        startEval.Duration + endEval.Duration,
                        startEval.Distance + endEval.Distance);
                    
                    evals[j].Add(totalEval);
                }
            }
        }
        
        return evals;
    }

    /// <summary>
    /// Seeds a route with initial job(s).
    /// </summary>
    private static void SeedRoute(Input input, TRoute route, InitStrategy init, 
        HashSet<int> unassigned, long[] jobsMinCosts, List<List<Eval>> jobsVehicleEvals, int vRank)
    {
        if (init == InitStrategy.Furthest)
        {
            // Find furthest compatible unassigned job
            var compatibleJobs = unassigned
                .Where(j => VehicleOkWithJob(input, vRank, j) && 
                           jobsMinCosts[j] >= jobsVehicleEvals[j][vRank].Cost)
                .ToList();
            
            if (compatibleJobs.Any())
            {
                var furthest = compatibleJobs
                    .OrderByDescending(j => jobsVehicleEvals[j][vRank].Cost)
                    .First();
                
                route.Add(furthest);
                unassigned.Remove(furthest);
            }
        }
        else if (init == InitStrategy.Nearest)
        {
            // Find nearest compatible unassigned job
            var compatibleJobs = unassigned
                .Where(j => VehicleOkWithJob(input, vRank, j) && 
                           jobsMinCosts[j] >= jobsVehicleEvals[j][vRank].Cost)
                .ToList();
            
            if (compatibleJobs.Any())
            {
                var nearest = compatibleJobs
                    .OrderBy(j => jobsVehicleEvals[j][vRank].Cost)
                    .First();
                
                route.Add(nearest);
                unassigned.Remove(nearest);
            }
        }
        // For InitStrategy.None or InitStrategy.HigherAmount, route starts empty
    }

    /// <summary>
    /// Fills a route using regret-based insertion.
    /// </summary>
    private static Eval FillRoute(Input input, TRoute route, HashSet<int> unassigned, 
        long[] regrets, double lambda)
    {
        var routeEval = new Eval(0, 0, 0);
        bool keepGoing = true;
        
        while (keepGoing && unassigned.Count > 0)
        {
            keepGoing = false;
            double bestScore = 0;
            int bestJob = -1;
            int bestRank = -1;
            Eval bestEval = new Eval(0, 0, 0);
            
            // Try to insert each unassigned job
            foreach (var j in unassigned)
            {
                if (!VehicleOkWithJob(input, route.VehicleId, j))
                {
                    continue;
                }
                
                var job = input.Jobs[j];
                
                // Check capacity
                if (!route.CanAddForCapacity(job.Pickup, job.Delivery))
                {
                    continue;
                }
                
                // Try all insertion positions
                for (int rank = 0; rank <= route.Route.Count; rank++)
                {
                    var insertionEval = ComputeInsertionCost(input, route, j, rank);
                    
                    if (insertionEval == null)
                    {
                        continue;
                    }
                    
                    var score = lambda * regrets[j] - insertionEval.Value.Cost;
                    
                    if (score > bestScore)
                    {
                        bestScore = score;
                        bestJob = j;
                        bestRank = rank;
                        bestEval = insertionEval.Value;
                        keepGoing = true;
                    }
                }
            }
            
            if (keepGoing)
            {
                route.Add(bestJob, bestRank);
                unassigned.Remove(bestJob);
                routeEval = new Eval(
                    routeEval.Cost + bestEval.Cost,
                    routeEval.Duration + bestEval.Duration,
                    routeEval.Distance + bestEval.Distance);
            }
        }
        
        return routeEval;
    }

    /// <summary>
    /// Computes the cost of inserting a job at a specific position.
    /// </summary>
    private static Eval? ComputeInsertionCost(Input input, TRoute route, int jobIndex, int rank)
    {
        var job = input.Jobs[jobIndex];
        var vehicle = input.Vehicles[route.VehicleId];
        
        if (rank == 0)
        {
            // Insert at beginning
            var startToJob = input.GetEval(input.GetVehicleStartIndex(route.VehicleId), input.GetJobLocationIndex(jobIndex), route.VehicleId);
            if (route.Route.Count == 0)
            {
                var jobToEnd = input.GetEval(input.GetJobLocationIndex(jobIndex), input.GetVehicleEndIndex(route.VehicleId), route.VehicleId);
                return new Eval(
                    startToJob.Cost + jobToEnd.Cost,
                    startToJob.Duration + jobToEnd.Duration,
                    startToJob.Distance + jobToEnd.Distance);
            }
            else
            {
                var firstJobIndex = route.Route[0];
                var jobToFirst = input.GetEval(input.GetJobLocationIndex(jobIndex), input.GetJobLocationIndex(firstJobIndex), route.VehicleId);
                var oldStartToFirst = input.GetEval(input.GetVehicleStartIndex(route.VehicleId), input.GetJobLocationIndex(firstJobIndex), route.VehicleId);
                return new Eval(
                    startToJob.Cost + jobToFirst.Cost - oldStartToFirst.Cost,
                    startToJob.Duration + jobToFirst.Duration - oldStartToFirst.Duration,
                    startToJob.Distance + jobToFirst.Distance - oldStartToFirst.Distance);
            }
        }
        else if (rank == route.Route.Count)
        {
            // Insert at end
            var lastJobIndex = route.Route[rank - 1];
            var lastToJob = input.GetEval(input.GetJobLocationIndex(lastJobIndex), input.GetJobLocationIndex(jobIndex), route.VehicleId);
            var jobToEnd = input.GetEval(input.GetJobLocationIndex(jobIndex), input.GetVehicleEndIndex(route.VehicleId), route.VehicleId);
            var oldLastToEnd = input.GetEval(input.GetJobLocationIndex(lastJobIndex), input.GetVehicleEndIndex(route.VehicleId), route.VehicleId);
            return new Eval(
                lastToJob.Cost + jobToEnd.Cost - oldLastToEnd.Cost,
                lastToJob.Duration + jobToEnd.Duration - oldLastToEnd.Duration,
                lastToJob.Distance + jobToEnd.Distance - oldLastToEnd.Distance);
        }
        else
        {
            // Insert in middle
            var prevJobIndex = route.Route[rank - 1];
            var nextJobIndex = route.Route[rank];
            var prevToJob = input.GetEval(input.GetJobLocationIndex(prevJobIndex), input.GetJobLocationIndex(jobIndex), route.VehicleId);
            var jobToNext = input.GetEval(input.GetJobLocationIndex(jobIndex), input.GetJobLocationIndex(nextJobIndex), route.VehicleId);
            var oldPrevToNext = input.GetEval(input.GetJobLocationIndex(prevJobIndex), input.GetJobLocationIndex(nextJobIndex), route.VehicleId);
            return new Eval(
                prevToJob.Cost + jobToNext.Cost - oldPrevToNext.Cost,
                prevToJob.Duration + jobToNext.Duration - oldPrevToNext.Duration,
                prevToJob.Distance + jobToNext.Distance - oldPrevToNext.Distance);
        }
    }

    /// <summary>
    /// Checks if a vehicle can handle a job.
    /// </summary>
    private static bool VehicleOkWithJob(Input input, int vehicleIndex, int jobIndex)
    {
        var vehicle = input.Vehicles[vehicleIndex];
        var job = input.Jobs[jobIndex];
        
        // Check skills
        if (job.Skills != null && job.Skills.Count > 0)
        {
            if (vehicle.Skills == null || !job.Skills.All(s => vehicle.Skills.Contains(s)))
            {
                return false;
            }
        }
        
        return true;
    }

    /// <summary>
    /// Gets the cost upper bound.
    /// </summary>
    private static long GetCostUpperBound()
    {
        // Simple upper bound - could be improved
        return long.MaxValue / 1000;
    }

    /// <summary>
    /// Gets a cost-based rank for vehicle ordering.
    /// </summary>
    private static long GetVehicleCostRank(Input input, int vehicleIndex)
    {
        // For now, use fixed cost as proxy
        return input.Vehicles[vehicleIndex].Costs.Fixed;
    }
}