using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.VRP;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch;

/// <summary>
/// Utilities for insertion search operations (based on VROOM).
/// </summary>
internal static class InsertionSearch
{
    public struct RouteInsertion
    {
        public Eval Eval { get; set; }
        public Amount Delivery { get; set; }
        public int SingleRank { get; set; }
        public int PickupRank { get; set; }
        public int DeliveryRank { get; set; }

        public RouteInsertion(int amountSize)
        {
            Eval = Eval.NoEval;
            Delivery = new Amount(amountSize);
            SingleRank = 0;
            PickupRank = 0;
            DeliveryRank = 0;
        }
    }

    /// <summary>
    /// Compute best insertion for a pickup-delivery pair in a route.
    /// </summary>
    public static RouteInsertion ComputeBestInsertionPd(
        Input input,
        SolutionState solutionState,
        int jobIndex,
        int vehicleIndex,
        RawRoute route,
        Eval costThreshold)
    {
        var result = new RouteInsertion(input.AmountSize);
        var job = input.Jobs[jobIndex];
        var vehicle = input.Vehicles[vehicleIndex];

        if (!input.VehicleOkWithJob(vehicleIndex, jobIndex))
        {
            return result;
        }

        // Pickup-delivery pair jobs
        var pickupIndex = jobIndex;
        var deliveryIndex = jobIndex + 1;

        // Best cost for pickup and delivery insertion
        var bestCost = Eval.NoEval;
        int bestPickupRank = 0;
        int bestDeliveryRank = 0;

        // Try all possible pickup positions
        for (int pRank = 0; pRank <= route.Route.Count; pRank++)
        {
            // Compute pickup insertion cost
            var pickupCost = Helpers.AdditionCost(input, pickupIndex, vehicle, route.Route, pRank);

            // Try all possible delivery positions after pickup
            for (int dRank = pRank + 1; dRank <= route.Route.Count + 1; dRank++)
            {
                // Account for the pickup being inserted before
                var adjustedDRank = dRank;
                
                // Compute delivery insertion cost (with pickup already inserted)
                var tempRoute = new List<int>(route.Route);
                tempRoute.Insert(pRank, pickupIndex);
                var deliveryCost = Helpers.AdditionCost(input, deliveryIndex, vehicle, tempRoute, adjustedDRank);

                var totalCost = pickupCost + deliveryCost;

                if (totalCost.Cost < bestCost.Cost && totalCost.Cost < costThreshold.Cost)
                {
                    // Check capacity constraints
                    var pickup = input.GetJobPickup(pickupIndex) + input.GetJobPickup(deliveryIndex);
                    var delivery = input.GetJobDelivery(pickupIndex) + input.GetJobDelivery(deliveryIndex);

                    bool capacityValid = true;
                    if (pRank == adjustedDRank - 1)
                    {
                        // Pickup and delivery inserted consecutively
                        capacityValid = route.IsValidAdditionForCapacityMargins(
                            pickup, delivery, pRank, pRank);
                    }
                    else
                    {
                        // Pickup and delivery inserted at different positions
                        capacityValid = route.IsValidAdditionForCapacityMarginsPickupDelivery(
                            pickup, delivery, pRank, adjustedDRank - 1);
                    }

                    if (capacityValid)
                    {
                        bestCost = totalCost;
                        bestPickupRank = pRank;
                        bestDeliveryRank = adjustedDRank;
                        result.Eval = totalCost;
                        result.Delivery = delivery;
                        result.PickupRank = pRank;
                        result.DeliveryRank = adjustedDRank;
                    }
                }
            }
        }

        return result;
    }
}