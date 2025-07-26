# OptiRoute.Net

A high-performance route optimization library for .NET, inspired by [VROOM](https://github.com/VROOM-Project/vroom) (Vehicle Routing Open-source Optimization Machine).

## Overview

OptiRoute.Net is a C# port of VROOM's core routing algorithms, providing efficient solutions for:
- **TSP** (Traveling Salesman Problem)
- **VRP** (Vehicle Routing Problem)
- **CVRP** (Capacitated Vehicle Routing Problem)
- **VRPTW** (VRP with Time Windows) - *coming soon*

The library implements the same optimization algorithms as VROOM but with idiomatic C# APIs, making it easy to integrate into .NET applications.

## Features

- 🚀 High-performance routing algorithms ported from VROOM
- 📦 Multi-dimensional capacity constraints
- ⏰ Time window support (in development)
- 🗺️ Distance matrix abstraction for custom map providers
- 🔧 Extensible architecture for custom constraints
- 🧵 Multi-threaded optimization
- 📍 OSRM integration support (coming soon)

## Installation

```bash
dotnet add package OptiRoute.Net
```

## Quick Start

### Simple TSP Example

```csharp
using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.TSP;

// Create locations
var locations = new[]
{
    new Location(0, 0) { Description = "Depot" },
    new Location(1, 1) { Description = "Customer 1" },
    new Location(2, 0) { Description = "Customer 2" },
    new Location(1, -1) { Description = "Customer 3" }
};

// Create input
var input = new Input();

// Add jobs (skip depot)
for (int i = 1; i < locations.Length; i++)
{
    input.AddJob(new Job($"job_{i}", locations[i]));
}

// Add vehicle
input.AddVehicle(new Vehicle("vehicle_1")
{
    StartLocation = locations[0],
    EndLocation = locations[0]
});

// Set distance matrix
input.SetDistanceMatrix(new YourDistanceMatrix());

// Solve
var tspSolver = new TspSolver(input, 0, jobIndices);
var route = tspSolver.Solve();
```

### CVRP Example with Capacity Constraints

```csharp
using OptiRoute.Core.Algorithms.CVRP;

// Create jobs with delivery amounts
input.AddJob(new Job("delivery_1", customerLocation)
{
    Delivery = new Amount(new[] { 20L }), // 20kg
    ServiceDuration = 600 // 10 minutes
});

// Add vehicle with capacity
input.AddVehicle(new Vehicle("truck_1")
{
    Capacity = new Amount(new[] { 100L }), // 100kg capacity
    StartLocation = depot
});

// Solve CVRP
var cvrpSolver = new CvrpSolver(input);
var solution = cvrpSolver.Solve(
    nbSearches: 5,
    depth: 4,
    nbThreads: 4,
    timeout: TimeSpan.FromSeconds(30)
);
```

## Core Concepts

### Input
The `Input` class represents a routing problem with:
- **Jobs**: Tasks to be performed at specific locations
- **Vehicles**: Resources that can perform jobs
- **Distance Matrix**: Travel costs between locations

### Jobs
Jobs represent tasks with:
- Location
- Service duration
- Pickup/delivery amounts (multi-dimensional)
- Time windows (coming soon)
- Skills requirements

### Vehicles
Vehicles have:
- Start/end locations
- Capacity constraints (multi-dimensional)
- Working hours
- Skills
- Cost structure

### Solution
The optimization returns a `Solution` containing:
- Routes for each vehicle
- Unassigned jobs (if any)
- Total cost and statistics

## Algorithms

### TSP (Traveling Salesman Problem)
- Christofides heuristic with Hungarian algorithm
- Local search improvements (2-opt, relocate, or-opt)
- Asymmetric TSP support

### VRP/CVRP (Vehicle Routing Problem)
- Solomon I1 insertion heuristic
- Dynamic vehicle selection
- Local search operators:
  - Intra-route: 2-opt, relocate, or-opt
  - Inter-route: cross-exchange, relocate (coming soon)

## Distance Matrix

OptiRoute.Net uses an abstract `IDistanceMatrix` interface, allowing integration with any mapping service:

```csharp
public interface IDistanceMatrix
{
    int Size { get; }
    double GetDistance(int from, int to);
    double GetDuration(int from, int to);
    double GetCost(int from, int to);
}
```

Built-in implementations:
- `CustomDistanceMatrix`: For pre-calculated matrices
- `OsrmDistanceMatrix`: OSRM integration (coming soon)

## Performance

OptiRoute.Net is designed for performance:
- Efficient data structures ported from VROOM
- Multi-threaded local search
- Memory-efficient representations
- Optimized for real-world problem sizes

## Development Status

This is a port of VROOM's core algorithms to C#. Current status:

✅ **Completed**:
- Core models (Location, Job, Vehicle, Amount, Eval, TimeWindow, Solution)
- TSP solver with Christofides heuristic and Hungarian algorithm
- TSP local search (2-opt, relocate, or-opt)
- VRP/CVRP solver with multi-threaded search
- VRPTW solver with time window tracking
- Solomon I1 insertion heuristic
- VRP local search operators (Relocate, CrossExchange, OrOpt, TwoOpt)
- Forward/backward capacity tracking (RawRoute)
- Time window feasibility checking (TWRoute)
- Solution state management with cost caching
- Helper functions for cost calculations
- Distance matrix abstraction
- Comprehensive examples for TSP, CVRP, and VRPTW

📋 **Planned**:
- OSRM integration package
- Additional operators (MixedExchange, PDShift, RouteExchange)
- Pickup/delivery problem support
- Multi-depot support
- Performance benchmarks
- Comprehensive unit tests

## Contributing

Contributions are welcome! This project aims to:
1. Port VROOM's algorithms accurately to C#
2. Provide idiomatic .NET APIs
3. Maintain compatibility with VROOM's problem definitions

## License

This project is licensed under the same license as VROOM. See [LICENSE](LICENSE) for details.

## Acknowledgments

This library is based on [VROOM](https://github.com/VROOM-Project/vroom) by Julien Coupey and contributors. VROOM has been developed and refined over 10+ years and this port aims to bring its powerful algorithms to the .NET ecosystem.

## References

- [VROOM GitHub Repository](https://github.com/VROOM-Project/vroom)
- [VROOM Documentation](https://github.com/VROOM-Project/vroom/blob/master/docs/API.md)
- [Vehicle Routing Problem on Wikipedia](https://en.wikipedia.org/wiki/Vehicle_routing_problem)