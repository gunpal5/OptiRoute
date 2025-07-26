using OptiRoute.Core.DistanceMatrix;
using OptiRoute.Core.Algorithms;
using OptiRoute.Core.Algorithms.VRP;

namespace OptiRoute.Core.Models;

/// <summary>
/// Problem type enumeration.
/// </summary>
public enum ProblemType
{
    TSP,
    CVRP,
    VRPTW,
    PDPTW
}

/// <summary>
/// Represents the input for a routing problem (inspired by VROOM's Input class).
/// </summary>
public class Input
{
    private readonly List<Job> _jobs = new();
    private readonly List<Vehicle> _vehicles = new();
    private readonly List<Location> _locations = new();
    private readonly Dictionary<Location, int> _locationToIndex = new();
    private IDistanceMatrix? _distanceMatrix;
    private int? _amountSize;
    private List<List<bool>>? _vehicleToJobCompatibility;
    private List<List<bool>>? _vehicleToVehicleCompatibility;
    private bool _hasSkills = false;

    /// <summary>
    /// Gets all jobs in the problem.
    /// </summary>
    public IReadOnlyList<Job> Jobs => _jobs;

    /// <summary>
    /// Gets all vehicles in the problem.
    /// </summary>
    public IReadOnlyList<Vehicle> Vehicles => _vehicles;

    /// <summary>
    /// Gets all unique locations.
    /// </summary>
    public IReadOnlyList<Location> Locations => _locations;

    /// <summary>
    /// Gets the distance matrix.
    /// </summary>
    public IDistanceMatrix DistanceMatrix => _distanceMatrix ?? 
        throw new InvalidOperationException("Distance matrix not set");

    /// <summary>
    /// Gets the size of amount dimensions.
    /// </summary>
    public int AmountSize => _amountSize ?? 1; // Default to 1 dimension if not set

    /// <summary>
    /// Gets a zero amount with the correct dimension size.
    /// </summary>
    public Amount ZeroAmount => new Amount(AmountSize);

    /// <summary>
    /// Gets the upper bound for costs (used for regret computation).
    /// </summary>
    public long CostUpperBound => long.MaxValue / 2; // Avoid overflow in computations
    
    /// <summary>
    /// Gets whether the problem has shipments (pickup-delivery pairs).
    /// </summary>
    public bool HasShipments { get; private set; }
    
    /// <summary>
    /// Gets whether the problem has skills defined.
    /// </summary>
    public bool HasSkills => _hasSkills;
    
    /// <summary>
    /// Gets the list of compatible vehicles for each job.
    /// </summary>
    public List<List<int>> CompatibleVehiclesForJob { get; private set; } = new();
    
    /// <summary>
    /// Gets whether the problem has time windows.
    /// </summary>
    public bool HasTimeWindows { get; private set; }
    
    /// <summary>
    /// Gets whether the problem has custom objectives.
    /// </summary>
    public bool HasCustomObjective => false; // Not implemented yet
    
    /// <summary>
    /// Gets the problem type detected.
    /// </summary>
    public ProblemType ProblemType { get; private set; } = ProblemType.TSP;

    /// <summary>
    /// Adds a job to the problem.
    /// </summary>
    public void AddJob(Job job)
    {
        ArgumentNullException.ThrowIfNull(job);
        
        // Check amount size consistency
        CheckAmountSize(job.Delivery);
        CheckAmountSize(job.Pickup);
        
        // Jobs with zero-dimensional amounts will be normalized through GetJobPickup/GetJobDelivery
        
        _jobs.Add(job);
        AddLocation(job.Location);
        
        // Check for time windows
        if (job.TimeWindows != null && job.TimeWindows.Count > 0)
        {
            HasTimeWindows = true;
        }
    }
    
    /// <summary>
    /// Adds a shipment (pickup-delivery pair) to the problem.
    /// </summary>
    public void AddShipment(Job pickup, Job delivery)
    {
        ArgumentNullException.ThrowIfNull(pickup);
        ArgumentNullException.ThrowIfNull(delivery);
        
        if (pickup.Type != JobType.Pickup)
            throw new ArgumentException("First job must be a pickup", nameof(pickup));
        if (delivery.Type != JobType.Delivery)
            throw new ArgumentException("Second job must be a delivery", nameof(delivery));
            
        // Ensure amounts match
        if (pickup.Pickup != delivery.Delivery)
            throw new ArgumentException("Pickup amount must match delivery amount");
            
        AddJob(pickup);
        AddJob(delivery);
        HasShipments = true;
    }

    /// <summary>
    /// Adds a vehicle to the problem.
    /// </summary>
    public void AddVehicle(Vehicle vehicle)
    {
        ArgumentNullException.ThrowIfNull(vehicle);
        vehicle.Validate();
        
        // Check amount size consistency
        CheckAmountSize(vehicle.Capacity);
        
        _vehicles.Add(vehicle);
        
        if (vehicle.StartLocation != null)
            AddLocation(vehicle.StartLocation);
            
        if (vehicle.EndLocation != null)
            AddLocation(vehicle.EndLocation);
    }
    
    /// <summary>
    /// Gets the normalized capacity for a vehicle.
    /// </summary>
    public Amount GetVehicleCapacity(int vehicleIndex)
    {
        var capacity = _vehicles[vehicleIndex].Capacity;
        if (capacity.Size == 0)
        {
            // Create infinite capacity
            var values = new long[AmountSize];
            for (int i = 0; i < AmountSize; i++)
                values[i] = long.MaxValue / 2; // Avoid overflow
            return new Amount(values);
        }
        return capacity;
    }

    /// <summary>
    /// Sets the distance matrix for the problem.
    /// </summary>
    public void SetDistanceMatrix(IDistanceMatrix matrix)
    {
        ArgumentNullException.ThrowIfNull(matrix);
        
        if (matrix.Size != _locations.Count)
            throw new ArgumentException(
                $"Distance matrix size ({matrix.Size}) does not match number of locations ({_locations.Count})");
                
        _distanceMatrix = matrix;
    }

    /// <summary>
    /// Checks if a vehicle can handle a job based on skills and compatibility.
    /// </summary>
    public bool VehicleOkWithJob(int vehicleIndex, int jobIndex)
    {
        if (_vehicleToJobCompatibility != null)
            return _vehicleToJobCompatibility[vehicleIndex][jobIndex];
            
        var vehicle = _vehicles[vehicleIndex];
        var job = _jobs[jobIndex];
        
        return job.Skills.All(skill => vehicle.Skills.Contains(skill));
    }
    
    /// <summary>
    /// Checks if two vehicles have common job candidates.
    /// </summary>
    public bool VehicleOkWithVehicle(int v1Index, int v2Index)
    {
        if (_vehicleToVehicleCompatibility != null)
            return _vehicleToVehicleCompatibility[v1Index][v2Index];
            
        // Default to true if not computed
        return true;
    }

    /// <summary>
    /// Gets the evaluation (cost, duration, distance) between two locations for a vehicle.
    /// </summary>
    public Eval GetEval(int vehicleIndex, int fromLocation, int toLocation)
    {
        var vehicle = _vehicles[vehicleIndex];
        var distance = _distanceMatrix!.GetDistance(fromLocation, toLocation);
        var duration = (long)(_distanceMatrix.GetDuration(fromLocation, toLocation) / vehicle.SpeedFactor);
        var cost = (long)(_distanceMatrix.GetCost(fromLocation, toLocation) * 100); // Scale to avoid decimals
        
        return new Eval(cost, duration, (long)distance);
    }

    /// <summary>
    /// Gets the duration between two locations for a vehicle.
    /// </summary>
    public long GetDuration(int vehicleIndex, int fromLocation, int toLocation)
    {
        var vehicle = _vehicles[vehicleIndex];
        return (long)(_distanceMatrix!.GetDuration(fromLocation, toLocation) / vehicle.SpeedFactor);
    }

    /// <summary>
    /// Gets the normalized pickup amount for a job.
    /// </summary>
    public Amount GetJobPickup(int jobIndex)
    {
        var pickup = _jobs[jobIndex].Pickup;
        return pickup.Size == 0 ? ZeroAmount : pickup;
    }
    
    /// <summary>
    /// Gets the normalized delivery amount for a job.
    /// </summary>
    public Amount GetJobDelivery(int jobIndex)
    {
        var delivery = _jobs[jobIndex].Delivery;
        return delivery.Size == 0 ? ZeroAmount : delivery;
    }
    
    /// <summary>
    /// Gets the job-vehicle evaluation matrix (pre-computed costs from empty vehicle to each job).
    /// </summary>
    public List<List<Eval>> GetJobVehicleEvals()
    {
        var evals = new List<List<Eval>>();
        
        // For each job
        for (int j = 0; j < _jobs.Count; j++)
        {
            var jobEvals = new List<Eval>();
            var jobIdx = Array.IndexOf(_locations.ToArray(), _jobs[j].Location);
            
            // For each vehicle
            for (int v = 0; v < _vehicles.Count; v++)
            {
                // Compute cost from vehicle start to job (or 0 if no start)
                if (_vehicles[v].StartLocation != null)
                {
                    var startIdx = Array.IndexOf(_locations.ToArray(), _vehicles[v].StartLocation);
                    jobEvals.Add(GetEval(v, startIdx, jobIdx));
                }
                else
                {
                    jobEvals.Add(new Eval(0, 0, 0));
                }
            }
            
            evals.Add(jobEvals);
        }
        
        return evals;
    }

    private void AddLocation(Location location)
    {
        if (_locationToIndex.ContainsKey(location))
            return;
            
        if (!location.HasUserIndex)
            location.SetIndex(_locations.Count);
            
        _locationToIndex[location] = _locations.Count;
        _locations.Add(location);
    }

    private void CheckAmountSize(Amount amount)
    {
        // Skip zero-dimensional amounts (default initialization)
        if (amount.Size == 0)
            return;
            
        if (!_amountSize.HasValue)
        {
            _amountSize = amount.Size;
        }
        else if (amount.Size != _amountSize.Value)
        {
            throw new ArgumentException(
                $"Inconsistent amount size. Expected {_amountSize.Value}, got {amount.Size}");
        }
    }
    
    /// <summary>
    /// Sets up vehicle-to-job compatibility based on skills.
    /// </summary>
    public void SetSkillsCompatibility()
    {
        _vehicleToJobCompatibility = new List<List<bool>>();
        
        // Check if any job or vehicle has skills
        _hasSkills = _jobs.Any(j => j.Skills.Count > 0) || _vehicles.Any(v => v.Skills.Count > 0);
        
        for (int v = 0; v < _vehicles.Count; v++)
        {
            var vehicleCompatibility = new List<bool>();
            var vehicleSkills = _vehicles[v].Skills;
            
            for (int j = 0; j < _jobs.Count; j++)
            {
                bool isCompatible = true;
                
                if (_hasSkills)
                {
                    // Check if vehicle has all required skills
                    foreach (var skill in _jobs[j].Skills)
                    {
                        if (!vehicleSkills.Contains(skill))
                        {
                            isCompatible = false;
                            break;
                        }
                    }
                }
                
                vehicleCompatibility.Add(isCompatible);
            }
            
            _vehicleToJobCompatibility.Add(vehicleCompatibility);
        }
    }
    
    /// <summary>
    /// Sets up vehicle-to-vehicle compatibility based on common jobs.
    /// </summary>
    public void SetVehiclesCompatibility()
    {
        if (_vehicleToJobCompatibility == null)
            SetSkillsCompatibility();
            
        _vehicleToVehicleCompatibility = new List<List<bool>>();
        
        for (int v1 = 0; v1 < _vehicles.Count; v1++)
        {
            var v1Compatibility = new List<bool>();
            
            for (int v2 = 0; v2 < _vehicles.Count; v2++)
            {
                if (v1 == v2)
                {
                    v1Compatibility.Add(true);
                    continue;
                }
                
                // Check if vehicles have at least one common compatible job
                bool hasCommonJob = false;
                for (int j = 0; j < _jobs.Count; j++)
                {
                    if (_vehicleToJobCompatibility![v1][j] && _vehicleToJobCompatibility[v2][j])
                    {
                        hasCommonJob = true;
                        break;
                    }
                }
                
                v1Compatibility.Add(hasCommonJob);
            }
            
            _vehicleToVehicleCompatibility.Add(v1Compatibility);
        }
    }
    
    /// <summary>
    /// Sets extra compatibility considering capacity and time windows.
    /// </summary>
    public void SetExtraCompatibility()
    {
        if (_vehicleToJobCompatibility == null)
            SetSkillsCompatibility();
            
        CompatibleVehiclesForJob.Clear();
        
        for (int j = 0; j < _jobs.Count; j++)
        {
            var compatibleVehicles = new List<int>();
            
            for (int v = 0; v < _vehicles.Count; v++)
            {
                if (!_vehicleToJobCompatibility![v][j])
                    continue;
                    
                // Check if job fits in vehicle capacity
                var emptyRoute = new RawRoute(this, v);
                bool isCompatible = emptyRoute.IsValidAdditionForCapacity(
                    GetJobPickup(j),
                    GetJobDelivery(j),
                    0);
                    
                if (isCompatible)
                {
                    compatibleVehicles.Add(v);
                }
                else
                {
                    _vehicleToJobCompatibility[v][j] = false;
                }
            }
            
            CompatibleVehiclesForJob.Add(compatibleVehicles);
        }
    }
    
    /// <summary>
    /// Finalize input setup and detect problem type.
    /// </summary>
    public void FinalizeSetup()
    {
        // Set up compatibility matrices
        SetSkillsCompatibility();
        SetVehiclesCompatibility();
        SetExtraCompatibility();
        
        // Detect problem type
        DetectProblemType();
    }
    
    /// <summary>
    /// Detects the problem type based on constraints.
    /// </summary>
    private void DetectProblemType()
    {
        if (_vehicles.Count == 1 && !HasTimeWindows && !HasShipments)
        {
            ProblemType = ProblemType.TSP;
        }
        else if (HasTimeWindows)
        {
            ProblemType = ProblemType.VRPTW;
        }
        else if (HasShipments)
        {
            ProblemType = ProblemType.PDPTW; // Pickup-delivery with time windows
        }
        else
        {
            ProblemType = ProblemType.CVRP;
        }
    }
    
    /// <summary>
    /// Gets the index of a location.
    /// </summary>
    public int GetLocationIndex(Location location)
    {
        if (_locationToIndex.TryGetValue(location, out var index))
            return index;
        return -1;
    }
    
    /// <summary>
    /// Gets zero costs (used for initialization).
    /// </summary>
    public Eval GetZeroCosts() => new Eval(0, 0, 0);
}