using OptiRoute.Core.Models;

namespace OptiRoute.Core.Algorithms.LocalSearch;

/// <summary>
/// Represents a potential insertion of a job into a route (based on VROOM).
/// </summary>
internal struct RouteInsertion
{
    public Eval Eval { get; set; }
    public Amount Delivery { get; set; }
    public int SingleRank { get; set; }
    public int PickupRank { get; set; }
    public int DeliveryRank { get; set; }
    
    public RouteInsertion(int amountSize)
    {
        Eval = Models.Eval.NoEval;
        Delivery = new Amount(amountSize);
        SingleRank = 0;
        PickupRank = 0;
        DeliveryRank = 0;
    }
}