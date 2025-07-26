namespace OptiRoute.Core.Models;

/// <summary>
/// Represents a geographic coordinate with longitude and latitude.
/// </summary>
public readonly record struct Coordinate(double Longitude, double Latitude)
{
    /// <summary>
    /// Calculates the Haversine distance between two coordinates in meters.
    /// </summary>
    public double DistanceTo(Coordinate other)
    {
        const double earthRadiusKm = 6371.0;
        
        var dLat = ToRadians(other.Latitude - Latitude);
        var dLon = ToRadians(other.Longitude - Longitude);
        
        var a = Math.Sin(dLat / 2) * Math.Sin(dLat / 2) +
                Math.Cos(ToRadians(Latitude)) * Math.Cos(ToRadians(other.Latitude)) *
                Math.Sin(dLon / 2) * Math.Sin(dLon / 2);
        
        var c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
        return earthRadiusKm * c * 1000; // Convert to meters
    }
    
    private static double ToRadians(double degrees) => degrees * Math.PI / 180.0;
}