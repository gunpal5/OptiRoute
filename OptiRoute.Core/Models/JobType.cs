namespace OptiRoute.Core.Models;

/// <summary>
/// Type of job in the routing problem.
/// </summary>
public enum JobType
{
    /// <summary>
    /// Standard single job with pickup and/or delivery.
    /// </summary>
    Single,
    
    /// <summary>
    /// Pickup job that must be paired with a delivery.
    /// </summary>
    Pickup,
    
    /// <summary>
    /// Delivery job that must be paired with a pickup.
    /// </summary>
    Delivery
}