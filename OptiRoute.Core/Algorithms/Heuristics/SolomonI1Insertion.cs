using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.VRP;

namespace OptiRoute.Core.Algorithms.Heuristics;

/// <summary>
/// Helper methods for Solomon I1 insertion calculations (based on VROOM).
/// </summary>
internal static class SolomonI1Insertion
{
    /// <summary>
    /// Calculates the cost of inserting a job at a specific rank in the route.
    /// </summary>
    public static Eval CalculateAdditionCost(
        Input input,
        int jobRank,
        int vehicleIndex,
        List<int> route,
        int rank)
    {
        if (rank > route.Count)
            throw new ArgumentOutOfRangeException(nameof(rank));

        var vehicle = input.Vehicles[vehicleIndex];
        var jobLocation = Array.IndexOf(input.Locations.ToArray(), input.Jobs[jobRank].Location);

        Eval previousEval = new Eval(0, 0, 0);
        Eval nextEval = new Eval(0, 0, 0);
        Eval oldEdgeEval = new Eval(0, 0, 0);

        if (rank == route.Count)
        {
            // Adding at the end of route
            if (route.Count == 0)
            {
                // Empty route
                if (vehicle.StartLocation != null)
                {
                    var startIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                    previousEval = input.GetEval(vehicleIndex, startIdx, jobLocation);
                }
                if (vehicle.EndLocation != null)
                {
                    var endIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
                    nextEval = input.GetEval(vehicleIndex, jobLocation, endIdx);
                }
            }
            else
            {
                // Adding after last job
                var lastJobLocation = Array.IndexOf(input.Locations.ToArray(), 
                    input.Jobs[route[rank - 1]].Location);
                previousEval = input.GetEval(vehicleIndex, lastJobLocation, jobLocation);
                
                if (vehicle.EndLocation != null)
                {
                    var endIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
                    oldEdgeEval = input.GetEval(vehicleIndex, lastJobLocation, endIdx);
                    nextEval = input.GetEval(vehicleIndex, jobLocation, endIdx);
                }
            }
        }
        else
        {
            // Adding before an existing job
            var nextJobLocation = Array.IndexOf(input.Locations.ToArray(), 
                input.Jobs[route[rank]].Location);
            nextEval = input.GetEval(vehicleIndex, jobLocation, nextJobLocation);

            if (rank == 0)
            {
                // Adding at the beginning
                if (vehicle.StartLocation != null)
                {
                    var startIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                    previousEval = input.GetEval(vehicleIndex, startIdx, jobLocation);
                    oldEdgeEval = input.GetEval(vehicleIndex, startIdx, nextJobLocation);
                }
            }
            else
            {
                // Adding between two jobs
                var prevJobLocation = Array.IndexOf(input.Locations.ToArray(), 
                    input.Jobs[route[rank - 1]].Location);
                previousEval = input.GetEval(vehicleIndex, prevJobLocation, jobLocation);
                oldEdgeEval = input.GetEval(vehicleIndex, prevJobLocation, nextJobLocation);
            }
        }

        return previousEval + nextEval - oldEdgeEval;
    }

    /// <summary>
    /// Seeds a route with the best initial job based on the initialization strategy.
    /// </summary>
    public static void SeedRoute<TRoute>(
        Input input,
        TRoute route,
        InitStrategy init,
        HashSet<int> unassigned) where TRoute : IRoute
    {
        if (init == InitStrategy.None) return;

        var vehicleIndex = route.GetVehicleIndex();
        var vehicle = input.Vehicles[vehicleIndex];

        bool foundSeed = false;
        int bestJobRank = -1;
        
        var higherAmount = input.ZeroAmount;
        long furthestCost = 0;
        long nearestCost = long.MaxValue;
        long earliestDeadline = long.MaxValue;

        foreach (var jobRank in unassigned)
        {
            var job = input.Jobs[jobRank];
            
            // Skip if vehicle can't handle this job
            if (!input.VehicleOkWithJob(vehicleIndex, jobRank))
                continue;

            // Skip delivery jobs (handled with pickup)
            if (job is DeliveryJob)
                continue;

            bool tryValidity = false;

            // Determine if this job is a candidate based on init strategy
            switch (init)
            {
                case InitStrategy.HigherAmount:
                    tryValidity = job.Pickup > higherAmount || job.Delivery > higherAmount;
                    break;

                case InitStrategy.Furthest:
                    if (vehicle.StartLocation != null)
                    {
                        var startIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                        var jobIdx = Array.IndexOf(input.Locations.ToArray(), job.Location);
                        var cost = input.GetEval(vehicleIndex, startIdx, jobIdx).Cost;
                        tryValidity = cost > furthestCost;
                    }
                    break;

                case InitStrategy.Nearest:
                    if (vehicle.StartLocation != null)
                    {
                        var startIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                        var jobIdx = Array.IndexOf(input.Locations.ToArray(), job.Location);
                        var cost = input.GetEval(vehicleIndex, startIdx, jobIdx).Cost;
                        tryValidity = cost < nearestCost;
                    }
                    break;

                case InitStrategy.EarliestDeadline:
                    var deadline = job.TimeWindows.Count > 0 ? job.TimeWindows[0].End : long.MaxValue;
                    tryValidity = deadline < earliestDeadline;
                    break;

            }

            if (!tryValidity)
                continue;

            // Check if addition is valid
            var rawRoute = route as RawRoute;
            if (rawRoute != null)
            {
                bool isValid = rawRoute.IsValidAdditionForCapacity(job.Pickup, job.Delivery, 0);
                
                // Additional time window check would go here
                
                if (isValid)
                {
                    foundSeed = true;
                    bestJobRank = jobRank;

                    // Update comparison values
                    switch (init)
                    {
                        case InitStrategy.HigherAmount:
                            if (job.Pickup > higherAmount)
                                higherAmount = job.Pickup;
                            if (job.Delivery > higherAmount)
                                higherAmount = job.Delivery;
                            break;

                        case InitStrategy.Furthest:
                            if (vehicle.StartLocation != null)
                            {
                                var startIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                                var jobIdx = Array.IndexOf(input.Locations.ToArray(), job.Location);
                                furthestCost = input.GetEval(vehicleIndex, startIdx, jobIdx).Cost;
                            }
                            break;

                        case InitStrategy.Nearest:
                            if (vehicle.StartLocation != null)
                            {
                                var startIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                                var jobIdx = Array.IndexOf(input.Locations.ToArray(), job.Location);
                                nearestCost = input.GetEval(vehicleIndex, startIdx, jobIdx).Cost;
                            }
                            break;

                        case InitStrategy.EarliestDeadline:
                            earliestDeadline = job.TimeWindows.Count > 0 ? job.TimeWindows[0].End : long.MaxValue;
                            break;
                    }
                }
            }
        }

        // Insert the best seed job if found
        if (foundSeed && bestJobRank >= 0)
        {
            route.InsertJob(bestJobRank, 0);
            unassigned.Remove(bestJobRank);
        }
    }

    /// <summary>
    /// Calculates the cost of adding a pickup-delivery pair.
    /// </summary>
    public static Eval CalculatePickupDeliveryAdditionCost(
        Input input,
        int pickupRank,
        int vehicleIndex,
        List<int> route,
        int pickupInsertRank,
        int deliveryInsertRank)
    {
        if (pickupInsertRank >= deliveryInsertRank)
            throw new ArgumentException("Pickup must be inserted before delivery");

        // Start with pickup insertion cost
        var eval = CalculateAdditionCost(input, pickupRank, vehicleIndex, route, pickupInsertRank);

        if (deliveryInsertRank == pickupInsertRank + 1)
        {
            // Delivery inserted right after pickup
            var pickupLocation = Array.IndexOf(input.Locations.ToArray(), 
                input.Jobs[pickupRank].Location);
            var deliveryLocation = Array.IndexOf(input.Locations.ToArray(), 
                input.Jobs[pickupRank + 1].Location);
            
            eval = eval + input.GetEval(vehicleIndex, pickupLocation, deliveryLocation);

            Eval afterDelivery = new Eval(0, 0, 0);
            Eval removeAfterPickup = new Eval(0, 0, 0);

            if (pickupInsertRank == route.Count)
            {
                // Adding at the end
                var vehicle = input.Vehicles[vehicleIndex];
                if (vehicle.EndLocation != null)
                {
                    var endIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
                    afterDelivery = input.GetEval(vehicleIndex, deliveryLocation, endIdx);
                    removeAfterPickup = input.GetEval(vehicleIndex, pickupLocation, endIdx);
                }
            }
            else
            {
                // There's a job after insertion
                var nextJobLocation = Array.IndexOf(input.Locations.ToArray(), 
                    input.Jobs[route[pickupInsertRank]].Location);
                afterDelivery = input.GetEval(vehicleIndex, deliveryLocation, nextJobLocation);
                removeAfterPickup = input.GetEval(vehicleIndex, pickupLocation, nextJobLocation);
            }

            eval = eval + afterDelivery - removeAfterPickup;
        }
        else
        {
            // Delivery is further away, calculate separately
            // Note: delivery rank is adjusted because pickup will be inserted first
            eval = eval + CalculateAdditionCost(input, pickupRank + 1, vehicleIndex, 
                route, deliveryInsertRank - 1);
        }

        return eval;
    }
}