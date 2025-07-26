using OptiRoute.Core.Models;
using OptiRoute.Core.Problems.VRP;

namespace OptiRoute.Core.Algorithms;

/// <summary>
/// Represents a route under construction with capacity tracking (inspired by VROOM's RawRoute).
/// </summary>
public class RawRoute : IRoute
{
    protected Input _input = null!;
    private readonly List<Amount> _fwdPickups;
    private readonly List<Amount> _fwdDeliveries;
    private readonly List<Amount> _bwdPickups;
    private readonly List<Amount> _bwdDeliveries;
    private readonly List<Amount> _currentLoads;
    private Amount _zero = null!;
    
    // Additional tracking from VROOM
    private readonly List<Amount> _pdLoads;  // Shipment loads at each rank
    private readonly List<int> _nbPickups;   // Number of pickups up to rank
    private readonly List<int> _nbDeliveries; // Number of deliveries up to rank
    private readonly List<Amount> _fwdPeaks; // Peak loads up to step
    private readonly List<Amount> _bwdPeaks; // Peak loads after step
    private Amount _deliveryMargin;
    private Amount _pickupMargin;

    public int VehicleRank { get; private set; }
    public int VehicleType { get; private set; }
    public List<int> Route { get; private set; }
    public Amount Capacity { get; private set; }
    public bool HasStart { get; private set; }
    public bool HasEnd { get; private set; }

    public RawRoute() 
    {
        // Parameterless constructor for generic constraints
        _input = null!;
        _fwdPickups = new List<Amount>();
        _fwdDeliveries = new List<Amount>();
        _bwdPickups = new List<Amount>();
        _bwdDeliveries = new List<Amount>();
        _currentLoads = new List<Amount>();
        _pdLoads = new List<Amount>();
        _nbPickups = new List<int>();
        _nbDeliveries = new List<int>();
        _fwdPeaks = new List<Amount>();
        _bwdPeaks = new List<Amount>();
        _zero = new Amount(0);
        _deliveryMargin = new Amount(0);
        _pickupMargin = new Amount(0);
        Route = new List<int>();
        Capacity = new Amount(0);
        VehicleType = 0;
    }
    
    public RawRoute(Input input, int vehicleRank)
    {
        _input = input;
        VehicleRank = vehicleRank;
        Route = new List<int>();
        
        var vehicle = input.Vehicles[vehicleRank];
        Capacity = input.GetVehicleCapacity(vehicleRank);
        HasStart = vehicle.StartLocation != null;
        HasEnd = vehicle.EndLocation != null;
        
        _zero = input.ZeroAmount;
        VehicleType = vehicle.Type;
        _fwdPickups = new List<Amount>();
        _fwdDeliveries = new List<Amount>();
        _bwdPickups = new List<Amount>();
        _bwdDeliveries = new List<Amount>();
        _currentLoads = new List<Amount> { new Amount(_zero) };
        _pdLoads = new List<Amount>();
        _nbPickups = new List<int>();
        _nbDeliveries = new List<int>();
        _fwdPeaks = new List<Amount>();
        _bwdPeaks = new List<Amount>();
        
        // Initialize margins
        _deliveryMargin = new Amount(Capacity);
        _pickupMargin = new Amount(Capacity);
        
        // Initialize amounts for empty route
        UpdateAmounts();
    }

    public bool Empty => Route.Count == 0;
    public int Size => Route.Count;

    /// <summary>
    /// Updates the capacity tracking arrays after route changes.
    /// </summary>
    public void UpdateAmounts()
    {
        var stepSize = Route.Count + 2;
        
        // Resize arrays
        _fwdPickups.Clear();
        _fwdDeliveries.Clear();
        _bwdPickups.Clear();
        _bwdDeliveries.Clear();
        _pdLoads.Clear();
        _nbPickups.Clear();
        _nbDeliveries.Clear();
        _currentLoads.Clear();
        _fwdPeaks.Clear();
        _bwdPeaks.Clear();

        if (Route.Count == 0) 
        {
            // Fill with zeros for consistency
            for (int i = 0; i < stepSize; i++)
            {
                _fwdPeaks.Add(new Amount(_zero));
                _bwdPeaks.Add(new Amount(_zero));
                _currentLoads.Add(new Amount(_zero));
            }
            _deliveryMargin = new Amount(Capacity);
            _pickupMargin = new Amount(Capacity);
            return;
        }

        // Forward computation
        var currentPickup = new Amount(_zero);
        var currentDelivery = new Amount(_zero);
        var currentPdLoad = new Amount(_zero);
        int currentNbPickups = 0;
        int currentNbDeliveries = 0;

        for (int i = 0; i < Route.Count; i++)
        {
            var job = _input.Jobs[Route[i]];
            
            var pickup = _input.GetJobPickup(Route[i]);
            var delivery = _input.GetJobDelivery(Route[i]);
            
            switch (job.Type)
            {
                case JobType.Single:
                    currentPickup = currentPickup + pickup;
                    currentDelivery = currentDelivery + delivery;
                    break;
                case JobType.Pickup:
                    currentPdLoad = currentPdLoad + pickup;
                    currentNbPickups++;
                    break;
                case JobType.Delivery:
                    // Assert delivery <= current_pd_load
                    currentPdLoad = currentPdLoad - delivery;
                    currentNbDeliveries++;
                    break;
            }
            
            _fwdPickups.Add(new Amount(currentPickup));
            _fwdDeliveries.Add(new Amount(currentDelivery));
            _pdLoads.Add(new Amount(currentPdLoad));
            _nbPickups.Add(currentNbPickups);
            _nbDeliveries.Add(currentNbDeliveries);
        }
        
        // Initialize current loads array
        for (int i = 0; i < stepSize; i++)
        {
            _currentLoads.Add(new Amount(_zero));
        }
        
        // Set last current load
        _currentLoads[_currentLoads.Count - 1] = new Amount(_fwdPickups[_fwdPickups.Count - 1]);

        // Backward computation
        currentDelivery = new Amount(_zero);
        currentPickup = new Amount(_zero);

        for (int i = 0; i < Route.Count; i++)
        {
            var bwdI = Route.Count - i - 1;
            _bwdDeliveries.Insert(0, new Amount(currentDelivery));
            _bwdPickups.Insert(0, new Amount(currentPickup));
            
            _currentLoads[bwdI + 1] = _fwdPickups[bwdI] + _pdLoads[bwdI] + currentDelivery;
            
            var job = _input.Jobs[Route[bwdI]];
            if (job.Type == JobType.Single)
            {
                currentDelivery = currentDelivery + _input.GetJobDelivery(Route[bwdI]);
                currentPickup = currentPickup + _input.GetJobPickup(Route[bwdI]);
            }
        }
        _currentLoads[0] = new Amount(currentDelivery);
        
        // Compute forward peaks
        var peak = new Amount(_currentLoads[0]);
        _fwdPeaks.Add(new Amount(peak));
        for (int s = 1; s < stepSize; s++)
        {
            // Handle max component-wise
            for (int r = 0; r < _zero.Size; r++)
            {
                peak[r] = Math.Max(peak[r], _currentLoads[s][r]);
            }
            _fwdPeaks.Add(new Amount(peak));
        }
        
        // Compute backward peaks
        peak = new Amount(_currentLoads[_currentLoads.Count - 1]);
        _bwdPeaks.Add(new Amount(peak)); // Add last one first
        for (int s = 1; s < stepSize; s++)
        {
            var bwdS = stepSize - s - 1;
            // Handle max component-wise
            for (int r = 0; r < _zero.Size; r++)
            {
                peak[r] = Math.Max(peak[r], _currentLoads[bwdS][r]);
            }
            _bwdPeaks.Insert(0, new Amount(peak));
        }
        
        // Compute margins
        if (Route.Count == 0)
        {
            _deliveryMargin = new Amount(Capacity);
            _pickupMargin = new Amount(Capacity);
        }
        else
        {
            var pickupsSum = _fwdPickups[_fwdPickups.Count - 1];
            _deliveryMargin = Capacity - _currentLoads[0];
            _pickupMargin = Capacity - pickupsSum;
        }
    }

    /// <summary>
    /// Checks if adding a job at the given rank is valid for capacity.
    /// </summary>
    public bool IsValidAdditionForCapacity(Amount pickup, Amount delivery, int rank)
    {
        if (rank > Route.Count)
            throw new ArgumentOutOfRangeException(nameof(rank));

        // Ensure peaks are available
        if (_fwdPeaks.Count <= rank || _bwdPeaks.Count <= rank)
        {
            // Peaks not properly initialized, return conservative answer
            if (pickup.Size != Capacity.Size || delivery.Size != Capacity.Size)
            {
                throw new ArgumentException($"Amount size mismatch: pickup={pickup.Size}, delivery={delivery.Size}, capacity={Capacity.Size}, vehicleRank={VehicleRank}, input.AmountSize={_input?.AmountSize}");
            }
            return (pickup <= Capacity) && (delivery <= Capacity);
        }

        // Match VROOM's implementation
        return (_fwdPeaks[rank] + delivery <= Capacity) &&
               (_bwdPeaks[rank] + pickup <= Capacity);
    }

    /// <summary>
    /// Adds a job at the specified rank.
    /// </summary>
    public void Add(Input input, int jobRank, int rank)
    {
        if (rank > Route.Count)
            throw new ArgumentOutOfRangeException(nameof(rank));
            
        Route.Insert(rank, jobRank);
        UpdateAmounts();
    }

    /// <summary>
    /// Removes jobs starting at the specified rank.
    /// </summary>
    public void Remove(int rank, int count)
    {
        if (rank + count > Route.Count)
            throw new ArgumentOutOfRangeException();
            
        Route.RemoveRange(rank, count);
        UpdateAmounts();
    }
    
    /// <summary>
    /// Remove jobs from the route with Input parameter (for compatibility).
    /// </summary>
    public void Remove(Input input, int rank, int count)
    {
        Remove(rank, count);
    }
    
    /// <summary>
    /// Check if removal is valid.
    /// </summary>
    public virtual bool IsValidRemoval(Input input, int rank, int count)
    {
        // Basic implementation - derived classes can override for TW checks
        return rank >= 0 && rank + count <= Route.Count;
    }
    
    /// <summary>
    /// Replaces a range of jobs in the route.
    /// </summary>
    public void Replace(Input input, Amount delivery, int[] jobs, int firstJob, int lastJob, int firstRank, int lastRank)
    {
        if (firstRank > lastRank || lastRank > Route.Count)
            throw new ArgumentException("Invalid rank range");
            
        // Remove existing jobs in the range
        if (lastRank > firstRank)
        {
            Route.RemoveRange(firstRank, lastRank - firstRank);
        }
        
        // Insert new jobs
        for (int i = firstJob; i < lastJob; i++)
        {
            Route.Insert(firstRank + i - firstJob, jobs[i]);
        }
        
        UpdateAmounts();
    }
    
    /// <summary>
    /// Checks if adding a job at the given rank is valid for time windows.
    /// </summary>
    public virtual bool IsValidAdditionForTw(Input input, int jobRank, int rank)
    {
        // For RawRoute (no time window tracking), always return true
        // This will be overridden in TWRoute
        return true;
    }
    
    /// <summary>
    /// Checks if adding a job sequence is valid for time windows.
    /// </summary>
    public virtual bool IsValidAdditionForTw(Input input, Amount delivery, int[] jobs, int firstJob, int lastJob, int firstRank, int lastRank)
    {
        // For RawRoute (no time window tracking), always return true
        // This will be overridden in TWRoute
        return true;
    }
    
    /// <summary>
    /// Checks if adding a job is valid for load (without delivery).
    /// </summary>
    public bool IsValidAdditionForLoad(Input input, Amount pickup, int rank)
    {
        if (rank > Route.Count)
            throw new ArgumentOutOfRangeException(nameof(rank));

        var load = Route.Count == 0 ? _zero : _currentLoads[rank];
        return load + pickup <= Capacity;
    }
    
    /// <summary>
    /// Checks if adding a job is valid for time windows without max load check.
    /// </summary>
    public virtual bool IsValidAdditionForTwWithoutMaxLoad(Input input, int jobRank, int rank)
    {
        // For RawRoute (no time window tracking), always return true
        // This will be overridden in TWRoute
        return true;
    }

    public Amount JobPickupsSum() => Route.Count > 0 ? _fwdPickups[^1] : _zero;
    public Amount JobDeliveriesSum() => Route.Count > 0 ? _fwdDeliveries[^1] : _zero;
    
    public Amount MaxLoad()
    {
        if (_currentLoads.Count == 0) return _zero;
        
        var max = new Amount(_currentLoads[0]);
        foreach (var load in _currentLoads.Skip(1))
        {
            for (int i = 0; i < max.Size; i++)
            {
                if (load[i] > max[i])
                    max[i] = load[i];
            }
        }
        return max;
    }
    
    /// <summary>
    /// Gets the capacity margin at a given rank (how much more can be added).
    /// </summary>
    public Amount CapacityMargin(int rank)
    {
        if (rank >= Route.Count)
            return Capacity;
            
        var currentLoad = _currentLoads[rank];
        return Capacity - currentLoad;
    }
    
    /// <summary>
    /// Gets the maximum load at any point in the route.
    /// </summary>
    public Amount MaxLoadAtRank(int rank)
    {
        if (rank >= _currentLoads.Count)
            return _zero;
        return _currentLoads[rank];
    }

    /// <summary>
    /// Checks if adding a job sequence is valid for capacity margins.
    /// </summary>
    public bool IsValidAdditionForCapacityMargins(Amount pickup, Amount delivery, int firstRank, int lastRank)
    {
        if (lastRank < 1)
            throw new ArgumentException("lastRank must be at least 1");
        if (lastRank > Route.Count + 1)
            throw new ArgumentException("lastRank cannot exceed Route.Count + 1");

        // Match VROOM's implementation exactly
        var firstDeliveries = (firstRank == 0) ? _currentLoads[0] : _bwdDeliveries[firstRank - 1];
        var firstPickups = (firstRank == 0) ? _zero : _fwdPickups[firstRank - 1];
        
        var replacedDeliveries = firstDeliveries - _bwdDeliveries[lastRank - 1];
        
        return (_fwdPeaks[firstRank] + delivery <= Capacity + replacedDeliveries) &&
               (_bwdPeaks[lastRank] + pickup <= Capacity + _fwdPickups[lastRank - 1] - firstPickups);
    }

    /// <summary>
    /// Checks if adding a job sequence with inclusion is valid for capacity.
    /// </summary>
    public bool IsValidAdditionForCapacityInclusion(Amount delivery, int[] jobs, int firstJob, int lastJob, int firstRank, int lastRank)
    {
        if (firstRank > lastRank)
            throw new ArgumentException("firstRank must be <= lastRank");
        if (lastRank > Route.Count + 1)
            throw new ArgumentException("lastRank cannot exceed Route.Count + 1");

        // Match VROOM's implementation
        var initLoad = Route.Count == 0 ? _zero : _currentLoads[0];
        
        var firstDeliveries = (firstRank == 0) ? initLoad : _bwdDeliveries[firstRank - 1];
        var lastDeliveries = (lastRank == 0) ? initLoad : _bwdDeliveries[lastRank - 1];
        
        var replacedDeliveries = firstDeliveries - lastDeliveries;
        
        delivery = delivery + (Route.Count == 0 ? _zero : _currentLoads[firstRank]) - replacedDeliveries;
        
        bool valid = delivery <= Capacity;
        
        for (int jobIter = firstJob; jobIter < lastJob && jobIter < jobs.Length; jobIter++)
        {
            if (!valid)
                break;
                
            delivery = delivery + _input.GetJobPickup(jobs[jobIter]);
            delivery = delivery - _input.GetJobDelivery(jobs[jobIter]);
            
            valid = delivery <= Capacity;
        }
        
        return valid;
    }

    /// <summary>
    /// Checks if swapping job sequences is valid for capacity.
    /// </summary>
    public bool IsValidSwapForCapacity(Amount removePickup, Amount removeDelivery, int removeRank, 
                                       Amount addPickup, Amount addDelivery, int addRank)
    {
        // Calculate new total amounts after swap
        var newTotalPickup = JobPickupsSum() - removePickup + addPickup;
        var newTotalDelivery = JobDeliveriesSum() - removeDelivery + addDelivery;
        
        if (!(newTotalPickup <= Capacity) || !(newTotalDelivery <= Capacity))
            return false;
        
        // Check load changes
        // This is simplified - full implementation would track exact load changes
        return true;
    }

    #region IRoute Implementation

    public void Initialize(Input input, int vehicleIndex)
    {
        _input = input;
        VehicleRank = vehicleIndex;
        Route = new List<int>();
        
        var vehicle = input.Vehicles[vehicleIndex];
        Capacity = input.GetVehicleCapacity(vehicleIndex);
        HasStart = vehicle.StartLocation != null;
        HasEnd = vehicle.EndLocation != null;
        
        _zero = input.ZeroAmount;
        VehicleType = vehicle.Type;
        
        // Initialize delivery and pickup margins
        _deliveryMargin = new Amount(Capacity);
        _pickupMargin = new Amount(Capacity);
        
        // Initialize amounts for empty route
        UpdateAmounts();
    }

    public int GetVehicleIndex()
    {
        return VehicleRank;
    }
    
    /// <summary>
    /// Check if the route is empty.
    /// </summary>
    public bool IsEmpty() => Route.Count == 0;
    

    public Amount GetCapacity()
    {
        return Capacity;
    }

    public List<int> GetJobSequence()
    {
        return Route;
    }

    public void InsertJob(int jobRank, int position)
    {
        Add(_input, jobRank, position);
    }

    #endregion
    
    // Additional IRoute interface implementations
    public int VehicleId => VehicleRank;
    
    public void Add(int jobIndex)
    {
        Add(_input, jobIndex, Route.Count);
    }
    
    public void Add(int jobIndex, int position)
    {
        Add(_input, jobIndex, position);
    }
    
    public bool CanAddForCapacity(Amount pickup, Amount delivery)
    {
        // Check if we can add considering current capacity
        var currentPickup = Route.Count > 0 ? _fwdPickups[Route.Count - 1] : _zero;
        var currentDelivery = Route.Count > 0 ? _fwdDeliveries[Route.Count - 1] : _zero;
        
        return (currentPickup + pickup <= Capacity) && 
               (currentDelivery + delivery <= Capacity);
    }
    
    public Eval Eval()
    {
        // Simple evaluation - return zero for now
        // In a full implementation, this would calculate route cost/duration/distance
        return new Eval(0, 0, 0);
    }
    
    /// <summary>
    /// Checks if adding a job is valid considering only the load up to a certain point.
    /// </summary>
    public bool IsValidAdditionForLoad(Amount pickup, int rank)
    {
        if (rank > Route.Count)
            throw new ArgumentOutOfRangeException(nameof(rank));
            
        // Check if we can handle the pickup amount
        var currentPickup = rank > 0 ? _fwdPickups[rank - 1] : _zero;
        return (currentPickup + pickup) <= Capacity;
    }
    
    
    /// <summary>
    /// Gets backward deliveries at a given rank.
    /// </summary>
    public Amount BwdDeliveries(int rank)
    {
        if (rank >= Route.Count)
            return _zero;
        return _bwdDeliveries[rank];
    }
    
    /// <summary>
    /// Gets backward pickups at a given rank.
    /// </summary>
    public Amount BwdPickups(int rank)
    {
        if (rank >= Route.Count)
            return _zero;
        return _bwdPickups[rank];
    }
    
    // VROOM-style method names
    public Amount bwd_deliveries(int rank) => BwdDeliveries(rank);
    public Amount bwd_pickups(int rank) => BwdPickups(rank);
    
    /// <summary>
    /// Gets pickup amount in a range of jobs.
    /// </summary>
    public Amount PickupInRange(int i, int j)
    {
        if (i > j || j > _fwdPickups.Count)
            throw new ArgumentException("Invalid range");
        if (i == j || Route.Count == 0)
            return _zero;
        if (i == 0)
            return _fwdPickups[j - 1];
        return _fwdPickups[j - 1] - _fwdPickups[i - 1];
    }
    
    /// <summary>
    /// Gets delivery amount in a range of jobs.
    /// </summary>
    public Amount DeliveryInRange(int i, int j)
    {
        if (i > j || j > _bwdDeliveries.Count)
            throw new ArgumentException("Invalid range");
        if (i == j || Route.Count == 0)
            return _zero;
        var beforeDeliveries = (i == 0) ? _currentLoads[0] : _bwdDeliveries[i - 1];
        return beforeDeliveries - _bwdDeliveries[j - 1];
    }
    
    /// <summary>
    /// Gets the delivery margin (capacity buffer for deliveries).
    /// </summary>
    public Amount DeliveryMargin() => _deliveryMargin;
    
    /// <summary>
    /// Gets the pickup margin (capacity buffer for pickups).
    /// </summary>
    public Amount PickupMargin() => _pickupMargin;
    
    /// <summary>
    /// Gets forward peak load up to step.
    /// </summary>
    public Amount FwdPeak(int step)
    {
        if (step >= _fwdPeaks.Count)
            return _zero;
        return _fwdPeaks[step];
    }
    
    /// <summary>
    /// Gets backward peak load after step.
    /// </summary>
    public Amount BwdPeak(int step)
    {
        if (step >= _bwdPeaks.Count)
            return _zero;
        return _bwdPeaks[step];
    }
    
    
    /// <summary>
    /// Checks if there are pending deliveries after rank.
    /// </summary>
    public bool HasPendingDeliveryAfterRank(int rank)
    {
        if (rank >= _nbDeliveries.Count || rank >= _nbPickups.Count)
            return false;
        return _nbDeliveries[rank] < _nbPickups[rank];
    }
    
    /// <summary>
    /// Checks if there are any deliveries after rank.
    /// </summary>
    public bool HasDeliveryAfterRank(int rank)
    {
        if (rank >= _nbDeliveries.Count)
            return false;
        return _nbDeliveries.Count > 0 && _nbDeliveries[rank] < _nbDeliveries[_nbDeliveries.Count - 1];
    }
    
    /// <summary>
    /// Checks if there are any pickups up to rank.
    /// </summary>
    public bool HasPickupUpToRank(int rank)
    {
        if (rank >= _nbPickups.Count)
            return false;
        return 0 < _nbPickups[rank];
    }
    
    /// <summary>
    /// Compute max load of sub-route spanning the [0; rank[ range.
    /// </summary>
    public Amount SubRouteMaxLoadBefore(int rank)
    {
        if (rank <= 0 || rank >= Route.Count)
            throw new ArgumentOutOfRangeException(nameof(rank));
        return _fwdPeaks[rank] - _bwdDeliveries[rank - 1];
    }
    
    /// <summary>
    /// Compute max load of sub-route spanning the [rank; size[ range.
    /// </summary>
    public Amount SubRouteMaxLoadAfter(int rank)
    {
        if (rank <= 0 || rank >= Route.Count)
            throw new ArgumentOutOfRangeException(nameof(rank));
        return _bwdPeaks[rank] - _fwdPickups[rank - 1];
    }
    
    /// <summary>
    /// Clears the route.
    /// </summary>
    public void Clear()
    {
        Route.Clear();
        UpdateAmounts();
    }
    
    /// <summary>
    /// Gets forward deliveries at rank i (total deliveries up to rank i).
    /// </summary>
    public Amount FwdDeliveries(int i)
    {
        if (i < 0 || i >= _fwdDeliveries.Count)
            return _zero;
        return _fwdDeliveries[i];
    }
    
    /// <summary>
    /// Gets forward pickups at rank i (total pickups up to rank i).
    /// </summary>
    public Amount FwdPickups(int i)
    {
        if (i < 0 || i >= _fwdPickups.Count)
            return _zero;
        return _fwdPickups[i];
    }
    
    /// <summary>
    /// Check validity for pickup-delivery insertion at different positions.
    /// </summary>
    public bool IsValidAdditionForCapacityMarginsPickupDelivery(
        Amount pickup,
        Amount delivery,
        int pickupRank,
        int deliveryRank)
    {
        if (pickupRank >= deliveryRank)
            throw new ArgumentException("Pickup rank must be less than delivery rank");
            
        // Check capacity after pickup insertion
        for (int i = pickupRank; i < deliveryRank && i < Route.Count; i++)
        {
            var currentMargin = CapacityMargin(i);
            if (!(pickup <= currentMargin))
                return false;
        }
        
        // Check capacity after delivery insertion
        if (deliveryRank < Route.Count)
        {
            var deliveryMargin = CapacityMargin(deliveryRank);
            var netAmount = pickup - delivery;
            if (!(netAmount <= deliveryMargin))
                return false;
        }
        
        return true;
    }
    
    /// <summary>
    /// Gets the load at a given step/rank in the route.
    /// </summary>
    public Amount LoadAtStep(int step)
    {
        if (step < 0 || step >= _currentLoads.Count)
            return _zero;
        return _currentLoads[step];
    }
}