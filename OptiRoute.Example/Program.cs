using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.TSP;
using OptiRoute.Core.Algorithms.CVRP;
using OptiRoute.Core.DistanceMatrix;

namespace OptiRoute.Example;

class Program
{
    static void Main(string[] args)
    {
        Console.WriteLine("OptiRoute.Net Example - Vehicle Routing Optimization\n");

        // Example 1: Simple TSP
        Console.WriteLine("Example 1: Traveling Salesman Problem");
        RunTspExample();

        Console.WriteLine("\n" + new string('-', 50) + "\n");

        // Example 2: CVRP
        Console.WriteLine("Example 2: Capacitated Vehicle Routing Problem");
        RunCvrpExampleSeparate();
    }

    static void RunTspExample()
    {
        // Create locations with coordinates
        var locations = new[]
        {
            new Location(new Coordinate(0, 0)),   // Depot
            new Location(new Coordinate(1, 1)),   // Customer 1
            new Location(new Coordinate(2, 0)),   // Customer 2  
            new Location(new Coordinate(1, -1)),  // Customer 3
            new Location(new Coordinate(-1, 0))   // Customer 4
        };
        
        var descriptions = new[] { "Depot", "Customer 1", "Customer 2", "Customer 3", "Customer 4" };

        // Create distance matrix (using Haversine distances)
        var distanceMatrix = new CustomDistanceMatrix(locations.Length);
        for (int i = 0; i < locations.Length; i++)
        {
            for (int j = 0; j < locations.Length; j++)
            {
                if (i == j)
                {
                    distanceMatrix.SetDistance(i, j, 0);
                    distanceMatrix.SetDuration(i, j, 0);
                }
                else
                {
                    var distance = locations[i].Coordinate!.Value.DistanceTo(locations[j].Coordinate!.Value);
                    distanceMatrix.SetDistance(i, j, distance);
                    distanceMatrix.SetDuration(i, j, distance / 10); // 10 m/s speed
                }
            }
        }

        // Create input
        var input = new Input();
        
        // Add jobs (skip depot at index 0)
        for (int i = 1; i < locations.Length; i++)
        {
            input.AddJob(new Job($"job_{i}", locations[i])
            {
                DefaultService = 300, // 5 minutes
                Description = descriptions[i]
            });
        }

        // Add vehicle starting from depot
        input.AddVehicle(new Vehicle("vehicle_1")
        {
            StartLocation = locations[0],
            EndLocation = locations[0],
            Description = "Delivery van"
        });

        input.SetDistanceMatrix(distanceMatrix);

        // Solve TSP
        var jobIndices = Enumerable.Range(0, input.Jobs.Count).ToList();
        var tspSolver = new TspSolver(input, 0, jobIndices);
        var solution = tspSolver.Solve(threadCount: 1, timeout: TimeSpan.FromSeconds(5));

        // Display results
        Console.WriteLine($"Optimal route:");
        Console.WriteLine("Depot");
        foreach (var jobIndex in solution)
        {
            var job = input.Jobs[jobIndex];
            Console.WriteLine($"-> {job.Description ?? job.Id}");
        }
        Console.WriteLine("-> Depot");
    }

    static void RunCvrpExampleSeparate()
    {
        // Create locations
        var depot = new Location(new Coordinate(0, 0));
        var customers = new[]
        {
            new Location(new Coordinate(2, 3)),   // Customer A
            new Location(new Coordinate(-1, 2)),  // Customer B
            new Location(new Coordinate(3, -1)),  // Customer C
            new Location(new Coordinate(-2, -2)), // Customer D
            new Location(new Coordinate(1, -3))   // Customer E
        };
        
        var customerNames = new[] { "Customer A", "Customer B", "Customer C", "Customer D", "Customer E" };

        // Create distance matrix
        var allLocations = new[] { depot }.Concat(customers).ToArray();
        var distanceMatrix = new CustomDistanceMatrix(allLocations.Length);
        
        for (int i = 0; i < allLocations.Length; i++)
        {
            for (int j = 0; j < allLocations.Length; j++)
            {
                if (i == j)
                {
                    distanceMatrix.SetDistance(i, j, 0);
                    distanceMatrix.SetDuration(i, j, 0);
                }
                else
                {
                    var distance = allLocations[i].Coordinate!.Value.DistanceTo(allLocations[j].Coordinate!.Value);
                    distanceMatrix.SetDistance(i, j, distance); // Already in meters
                    distanceMatrix.SetDuration(i, j, distance / 10); // 10m/s speed
                }
            }
        }

        // Create input
        var input = new Input();

        // Add jobs with demands
        var demands = new[] { 10, 15, 20, 25, 30 }; // kg
        for (int i = 0; i < customers.Length; i++)
        {
            input.AddJob(new Job($"delivery_{i + 1}", customers[i])
            {
                DefaultService = 600, // 10 minutes
                Delivery = new Amount(new[] { (long)demands[i] }),
                Pickup = new Amount(new[] { 0L }), // Need consistent size
                Description = $"{customerNames[i]} ({demands[i]}kg)"
            });
        }

        // Add vehicles with capacity
        for (int i = 0; i < 2; i++)
        {
            input.AddVehicle(new Vehicle($"truck_{i + 1}")
            {
                StartLocation = depot,
                EndLocation = depot,
                Capacity = new Amount(new[] { 50L }), // 50kg capacity
                Description = $"Delivery truck {i + 1}"
            });
        }

        input.SetDistanceMatrix(distanceMatrix);

        Console.WriteLine($"Created input with {input.Jobs.Count} jobs and {input.Vehicles.Count} vehicles");

        // Solve CVRP
        var cvrpSolver = new CvrpSolver(input);
        var solution = cvrpSolver.Solve(
            nbSearches: 5,
            depth: 4,
            nbThreads: 1,
            timeout: TimeSpan.FromSeconds(10)
        );

        // Display results
        Console.WriteLine($"Solution found with {solution.Routes.Count} routes:");
        Console.WriteLine($"Total cost: {solution.Cost}");
        Console.WriteLine($"Unassigned jobs: {solution.Unassigned.Count}");
        
        if (solution.Unassigned.Count > 0)
        {
            Console.WriteLine($"Unassigned job IDs: {string.Join(", ", solution.Unassigned.Select(u => u.JobId))}");
        }
        
        foreach (var route in solution.Routes)
        {
            Console.WriteLine($"\n{route.Description ?? route.VehicleId}:");
            Console.WriteLine($"  Cost: {route.Cost}, Distance: {route.Distance}m, Duration: {route.Duration}s");
            
            // Calculate total delivery for this route
            var totalDelivery = route.Steps.OfType<JobStep>()
                .Sum(js => input.Jobs.First(j => j.Id == js.JobId).Delivery[0]);
            Console.WriteLine($"  Total delivery: {totalDelivery}kg");
            Console.WriteLine($"  Route: {string.Join(" -> ", route.Steps.Select(s => GetStepDescription(s)))}");
        }
    }

    static string GetStepDescription(Step step)
    {
        return step switch
        {
            StartStep => "Depot (start)",
            EndStep => "Depot (end)",
            JobStep js => js.Description ?? js.JobId,
            _ => step.Type.ToString()
        };
    }
}

// Simple custom distance matrix implementation
public class CustomDistanceMatrix : IDistanceMatrix
{
    private readonly double[,] _distances;
    private readonly double[,] _durations;

    public CustomDistanceMatrix(int size)
    {
        Size = size;
        _distances = new double[size, size];
        _durations = new double[size, size];
    }

    public int Size { get; }

    public void SetDistance(int from, int to, double distance)
    {
        _distances[from, to] = distance;
    }

    public void SetDuration(int from, int to, double duration)
    {
        _durations[from, to] = duration;
    }

    public double GetDistance(int from, int to) => _distances[from, to];
    public long GetDuration(int from, int to) => (long)_durations[from, to];
    public decimal GetCost(int from, int to) => (decimal)_distances[from, to]; // Use distance as cost
}