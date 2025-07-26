using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms;
using OptiRoute.Core.Algorithms.LocalSearch;

namespace OptiRoute.Core.Utils;

/// <summary>
/// Helper functions for VRP algorithms (based on VROOM's utils/helpers.h).
/// </summary>
internal static class Helpers
{
    /// <summary>
    /// Evaluate adding job with rank job_rank in given route at given rank for vehicle v.
    /// </summary>
    public static Eval AdditionCost(
        Input input,
        int jobRank,
        Vehicle vehicle,
        List<int> route,
        int rank)
    {
        if (rank > route.Count)
            throw new ArgumentOutOfRangeException(nameof(rank));

        var vehicleIndex = input.Vehicles.ToList().IndexOf(vehicle);
        var jobIndex = Array.IndexOf(input.Locations.ToArray(), input.Jobs[jobRank].Location);
        
        Eval previousEval = new Eval(0, 0, 0);
        Eval nextEval = new Eval(0, 0, 0);
        Eval oldEdgeEval = new Eval(0, 0, 0);

        if (rank == route.Count)
        {
            if (route.Count == 0)
            {
                // Adding into empty route
                if (vehicle.StartLocation != null)
                {
                    var startIndex = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                    previousEval = input.GetEval(vehicleIndex, startIndex, jobIndex);
                }
                if (vehicle.EndLocation != null)
                {
                    var endIndex = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
                    nextEval = input.GetEval(vehicleIndex, jobIndex, endIndex);
                }
            }
            else
            {
                // Adding job past the end after a real job
                var prevIndex = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[rank - 1]].Location);
                previousEval = input.GetEval(vehicleIndex, prevIndex, jobIndex);
                if (vehicle.EndLocation != null)
                {
                    var endIndex = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
                    oldEdgeEval = input.GetEval(vehicleIndex, prevIndex, endIndex);
                    nextEval = input.GetEval(vehicleIndex, jobIndex, endIndex);
                }
            }
        }
        else
        {
            // Adding before one of the jobs
            var nextIndex = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[rank]].Location);
            nextEval = input.GetEval(vehicleIndex, jobIndex, nextIndex);

            if (rank == 0)
            {
                if (vehicle.StartLocation != null)
                {
                    var startIndex = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                    previousEval = input.GetEval(vehicleIndex, startIndex, jobIndex);
                    oldEdgeEval = input.GetEval(vehicleIndex, startIndex, nextIndex);
                }
            }
            else
            {
                var prevIndex = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[rank - 1]].Location);
                previousEval = input.GetEval(vehicleIndex, prevIndex, jobIndex);
                oldEdgeEval = input.GetEval(vehicleIndex, prevIndex, nextIndex);
            }
        }

        return previousEval + nextEval - oldEdgeEval;
    }

    /// <summary>
    /// Evaluate adding pickup with rank job_rank and associated delivery in given route.
    /// </summary>
    public static Eval AdditionCost(
        Input input,
        int jobRank,
        Vehicle vehicle,
        List<int> route,
        int pickupRank,
        int deliveryRank)
    {
        if (pickupRank >= deliveryRank || deliveryRank > route.Count + 1)
            throw new ArgumentException("Invalid pickup/delivery ranks");

        // Start with pickup eval
        var eval = AdditionCost(input, jobRank, vehicle, route, pickupRank);

        if (deliveryRank == pickupRank + 1)
        {
            // Delivery is inserted just after pickup
            var vehicleIndex = input.Vehicles.ToList().IndexOf(vehicle);
            var pIndex = Array.IndexOf(input.Locations.ToArray(), input.Jobs[jobRank].Location);
            var dIndex = Array.IndexOf(input.Locations.ToArray(), input.Jobs[jobRank + 1].Location);
            eval = eval + input.GetEval(vehicleIndex, pIndex, dIndex);

            Eval afterDelivery = new Eval(0, 0, 0);
            Eval removeAfterPickup = new Eval(0, 0, 0);

            if (pickupRank == route.Count)
            {
                // Addition at the end of a route
                if (vehicle.EndLocation != null)
                {
                    var endIndex = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
                    afterDelivery = input.GetEval(vehicleIndex, dIndex, endIndex);
                    removeAfterPickup = input.GetEval(vehicleIndex, pIndex, endIndex);
                }
            }
            else
            {
                // There is a job after insertion
                var nextIndex = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[pickupRank]].Location);
                afterDelivery = input.GetEval(vehicleIndex, dIndex, nextIndex);
                removeAfterPickup = input.GetEval(vehicleIndex, pIndex, nextIndex);
            }

            eval = eval + afterDelivery - removeAfterPickup;
        }
        else
        {
            // Delivery is further away so edges sets for pickup and delivery addition are disjoint
            eval = eval + AdditionCost(input, jobRank + 1, vehicle, route, deliveryRank - 1);
        }

        return eval;
    }

    /// <summary>
    /// Get indices for positions in route (before_first, first_index, last_index).
    /// </summary>
    public static (int? beforeFirst, int? firstIndex, int? lastIndex) GetIndices(
        Input input,
        RawRoute route,
        int firstRank,
        int lastRank)
    {
        var r = route.Route;
        var vehicle = input.Vehicles[route.VehicleRank];

        int? beforeFirst = null;
        if (firstRank > 0)
        {
            beforeFirst = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r[firstRank - 1]].Location);
        }
        else if (vehicle.StartLocation != null)
        {
            beforeFirst = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
        }

        int? firstIndex = null;
        if (firstRank < r.Count)
        {
            firstIndex = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r[firstRank]].Location);
        }
        else if (vehicle.EndLocation != null)
        {
            firstIndex = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
        }

        int? lastIndex = null;
        if (lastRank < r.Count)
        {
            lastIndex = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r[lastRank]].Location);
        }
        else if (vehicle.EndLocation != null)
        {
            lastIndex = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
        }

        return (beforeFirst, firstIndex, lastIndex);
    }

    /// <summary>
    /// Get the gain from removing a range of jobs from a route.
    /// </summary>
    public static Eval GetRangeRemovalGain(
        SolutionState solutionState,
        int vehicleIndex,
        int firstRank,
        int lastRank)
    {
        if (firstRank > lastRank)
            throw new ArgumentException("firstRank must be <= lastRank");

        Eval removalGain = new Eval(0, 0, 0);

        if (lastRank > firstRank)
        {
            // Gain related to removed portion
            // Uses pre-computed forward costs in SolutionState
            if (solutionState.FwdCosts[vehicleIndex][vehicleIndex].Count > 0)
            {
                if (lastRank - 1 < solutionState.FwdCosts[vehicleIndex][vehicleIndex].Count)
                {
                    removalGain = removalGain + solutionState.FwdCosts[vehicleIndex][vehicleIndex][lastRank - 1];
                }
                if (firstRank < solutionState.FwdCosts[vehicleIndex][vehicleIndex].Count)
                {
                    removalGain = removalGain - solutionState.FwdCosts[vehicleIndex][vehicleIndex][firstRank];
                }
            }
        }

        return removalGain;
    }

    /// <summary>
    /// Compute cost variation when replacing a range in route1 with a range from route2.
    /// Returns both normal and reversed insertion costs.
    /// </summary>
    public static (Eval straight, Eval reversed) AdditionCostDelta(
        Input input,
        SolutionState solutionState,
        RawRoute route1,
        int firstRank,
        int lastRank,
        RawRoute route2,
        int insertionStart,
        int insertionEnd)
    {
        if (firstRank > lastRank || lastRank > route1.Route.Count)
            throw new ArgumentException("Invalid range for route1");
        if (insertionStart > insertionEnd)
            throw new ArgumentException("Invalid insertion range");

        bool emptyInsertion = (insertionStart == insertionEnd);

        var r1 = route1.Route;
        var v1Rank = route1.VehicleRank;
        var r2 = route2.Route;
        var v2Rank = route2.VehicleRank;
        var v1 = input.Vehicles[v1Rank];

        // Common part of the cost
        Eval costDelta = GetRangeRemovalGain(solutionState, v1Rank, firstRank, lastRank);

        // Part of the cost that depends on insertion orientation
        Eval straightDelta = new Eval(0, 0, 0);
        Eval reversedDelta = new Eval(0, 0, 0);

        if (insertionStart != insertionEnd)
        {
            // Add costs for the inserted segment
            // Forward costs
            for (int i = insertionStart; i < insertionEnd - 1; i++)
            {
                var fromIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r2[i]].Location);
                var toIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r2[i + 1]].Location);
                straightDelta = straightDelta + input.GetEval(v1Rank, fromIdx, toIdx);
            }

            // Reversed costs
            for (int i = insertionEnd - 1; i > insertionStart; i--)
            {
                var fromIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r2[i]].Location);
                var toIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r2[i - 1]].Location);
                reversedDelta = reversedDelta + input.GetEval(v1Rank, fromIdx, toIdx);
            }
        }

        // Determine useful values if present
        var (beforeFirst, firstIndex, lastIndex) = GetIndices(input, route1, firstRank, lastRank);

        // Gain of removed edge before replaced range
        if (beforeFirst.HasValue && firstIndex.HasValue && r1.Count > 0)
        {
            costDelta = costDelta + input.GetEval(v1Rank, beforeFirst.Value, firstIndex.Value);
        }

        if (emptyInsertion)
        {
            if (beforeFirst.HasValue && lastIndex.HasValue && 
                !(firstRank == 0 && lastRank == r1.Count))
            {
                // Add cost of new edge replacing removed range
                costDelta = costDelta - input.GetEval(v1Rank, beforeFirst.Value, lastIndex.Value);
            }
        }
        else
        {
            if (beforeFirst.HasValue)
            {
                // Cost of new edge to inserted range
                var startIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r2[insertionStart]].Location);
                straightDelta = straightDelta - input.GetEval(v1Rank, beforeFirst.Value, startIdx);
                
                var endIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r2[insertionEnd - 1]].Location);
                reversedDelta = reversedDelta - input.GetEval(v1Rank, beforeFirst.Value, endIdx);
            }

            if (lastIndex.HasValue)
            {
                // Cost of new edge after inserted range
                var endIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r2[insertionEnd - 1]].Location);
                straightDelta = straightDelta - input.GetEval(v1Rank, endIdx, lastIndex.Value);
                
                var startIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r2[insertionStart]].Location);
                reversedDelta = reversedDelta - input.GetEval(v1Rank, startIdx, lastIndex.Value);
            }
        }

        // Gain of removed edge after replaced range, if any
        if (lastIndex.HasValue && lastRank > firstRank)
        {
            var beforeLastIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[r1[lastRank - 1]].Location);
            costDelta = costDelta + input.GetEval(v1Rank, beforeLastIdx, lastIndex.Value);
        }

        // Handle fixed cost addition
        if (r1.Count == 0 && !emptyInsertion)
        {
            costDelta = costDelta with { Cost = costDelta.Cost - v1.Costs.Fixed };
        }

        if (emptyInsertion && firstRank == 0 && lastRank == r1.Count)
        {
            costDelta = costDelta with { Cost = costDelta.Cost + v1.Costs.Fixed };
        }

        return (costDelta + straightDelta, costDelta + reversedDelta);
    }

    /// <summary>
    /// Compute the maximum edge cost in a route.
    /// </summary>
    public static Eval MaxEdgeEval(Input input, Vehicle vehicle, List<int> route)
    {
        var vehicleIndex = input.Vehicles.ToList().IndexOf(vehicle);
        Eval maxEval = new Eval(0, 0, 0);

        if (route.Count == 0)
        {
            if (vehicle.StartLocation != null && vehicle.EndLocation != null)
            {
                var startIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                var endIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
                return input.GetEval(vehicleIndex, startIdx, endIdx);
            }
            return maxEval;
        }

        // Check edge from start to first job
        if (vehicle.StartLocation != null)
        {
            var startIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
            var firstIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[0]].Location);
            var eval = input.GetEval(vehicleIndex, startIdx, firstIdx);
            if (eval.Cost > maxEval.Cost)
                maxEval = eval;
        }

        // Check edges between jobs
        for (int i = 0; i < route.Count - 1; i++)
        {
            var fromIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[i]].Location);
            var toIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[i + 1]].Location);
            var eval = input.GetEval(vehicleIndex, fromIdx, toIdx);
            if (eval.Cost > maxEval.Cost)
                maxEval = eval;
        }

        // Check edge from last job to end
        if (vehicle.EndLocation != null)
        {
            var lastIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[^1]].Location);
            var endIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
            var eval = input.GetEval(vehicleIndex, lastIdx, endIdx);
            if (eval.Cost > maxEval.Cost)
                maxEval = eval;
        }

        return maxEval;
    }

    /// <summary>
    /// Compute evaluation for a complete route including fixed costs.
    /// </summary>
    public static Eval RouteEvalForVehicle(Input input, int vehicleIndex, List<int> route)
    {
        var vehicle = input.Vehicles[vehicleIndex];
        var eval = new Eval(0, 0, 0);

        if (route.Count == 0)
            return eval;

        // Add fixed cost
        eval = eval with { Cost = eval.Cost + vehicle.Costs.Fixed };

        // Add edge from start to first job
        if (vehicle.StartLocation != null)
        {
            var startIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
            var firstIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[0]].Location);
            eval = eval + input.GetEval(vehicleIndex, startIdx, firstIdx);
        }

        // Add edges between jobs
        for (int i = 0; i < route.Count - 1; i++)
        {
            var fromIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[i]].Location);
            var toIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[i + 1]].Location);
            eval = eval + input.GetEval(vehicleIndex, fromIdx, toIdx);
        }

        // Add edge from last job to end
        if (vehicle.EndLocation != null)
        {
            var lastIdx = Array.IndexOf(input.Locations.ToArray(), input.Jobs[route[^1]].Location);
            var endIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
            eval = eval + input.GetEval(vehicleIndex, lastIdx, endIdx);
        }

        return eval;
    }
    
    /// <summary>
    /// Compute cost variation when removing the "count" elements starting from rank in route.
    /// </summary>
    public static Eval RemovalCostDelta(
        Input input,
        SolutionState solutionState,
        RawRoute route,
        int rank,
        int count)
    {
        if (route.Route.Count == 0)
            throw new ArgumentException("Route cannot be empty");
        if (rank + count > route.Route.Count)
            throw new ArgumentOutOfRangeException(nameof(count));
            
        // Use AdditionCostDelta with empty insertion
        var (costDelta, _) = AdditionCostDelta(
            input,
            solutionState,
            route,
            rank,
            rank + count,
            route,  // dummy route for empty insertion
            0,
            0);
            
        return costDelta;
    }
    
    /// <summary>
    /// Compute cost delta for in-place replacement of job at rank with new job (for SwapStar).
    /// </summary>
    public static Eval InPlaceDeltaCost(
        Input input,
        int jobRank,
        Vehicle vehicle,
        List<int> route,
        int rank)
    {
        if (route.Count == 0)
            throw new ArgumentException("Route cannot be empty");
            
        var vehicleIndex = input.Vehicles.ToList().IndexOf(vehicle);
        var newJobLocation = input.Jobs[jobRank].Location;
        var newIndex = Array.IndexOf(input.Locations.ToArray(), newJobLocation);
        
        var newPreviousEval = new Eval(0, 0, 0);
        var newNextEval = new Eval(0, 0, 0);
        int? pIndex = null;
        int? nIndex = null;
        
        // Calculate previous connection
        if (rank == 0)
        {
            if (vehicle.StartLocation != null)
            {
                pIndex = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                newPreviousEval = input.GetEval(vehicleIndex, pIndex.Value, newIndex);
            }
        }
        else
        {
            var prevJobLocation = input.Jobs[route[rank - 1]].Location;
            pIndex = Array.IndexOf(input.Locations.ToArray(), prevJobLocation);
            newPreviousEval = input.GetEval(vehicleIndex, pIndex.Value, newIndex);
        }
        
        // Calculate next connection
        if (rank == route.Count - 1)
        {
            if (vehicle.EndLocation != null)
            {
                nIndex = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
                newNextEval = input.GetEval(vehicleIndex, newIndex, nIndex.Value);
            }
        }
        else
        {
            var nextJobLocation = input.Jobs[route[rank + 1]].Location;
            nIndex = Array.IndexOf(input.Locations.ToArray(), nextJobLocation);
            newNextEval = input.GetEval(vehicleIndex, newIndex, nIndex.Value);
        }
        
        // Calculate virtual edge if both ends exist
        var oldVirtualEval = new Eval(0, 0, 0);
        if (pIndex.HasValue && nIndex.HasValue)
        {
            oldVirtualEval = input.GetEval(vehicleIndex, pIndex.Value, nIndex.Value);
        }
        
        return newPreviousEval + newNextEval - oldVirtualEval;
    }
    
    /// <summary>
    /// Calculate sum of priorities for all jobs in route.
    /// </summary>
    public static int PrioritySumForRoute(Input input, List<int> route)
    {
        return route.Sum(jobIndex => input.Jobs[jobIndex].Priority);
    }
    
    /// <summary>
    /// Compute cost difference for replacing a job at a specific position with another job.
    /// </summary>
    public static Eval AdditionCostDelta(
        Input input,
        SolutionState solutionState,
        RawRoute route,
        int jobRank,
        int nextJobRank,
        int newJobIndex)
    {
        if (jobRank >= route.Route.Count || nextJobRank != jobRank + 1)
            throw new ArgumentException("Invalid job ranks");
            
        var vehicle = input.Vehicles[route.VehicleRank];
        var oldJobIndex = route.Route[jobRank];
        
        // Cost of removing old job
        var removalGain = solutionState.NodeGains[route.VehicleRank][jobRank];
        
        // Cost of inserting new job at same position
        var insertionCost = AdditionCost(input, newJobIndex, vehicle, route.Route, jobRank);
        
        return removalGain - insertionCost;
    }
}