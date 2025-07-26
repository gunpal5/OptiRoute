using OptiRoute.Core.Models;
using OptiRoute.Core.Algorithms.VRP;

namespace OptiRoute.Core.Algorithms;

/// <summary>
/// Time Window Route - extends RawRoute with time window tracking (based on VROOM's TWRoute).
/// </summary>
public class TWRoute : RawRoute
{
    private readonly long _vStart;
    private readonly long _vEnd;

    /// <summary>
    /// Earliest arrival times for each job in the route.
    /// </summary>
    public List<long> Earliest { get; }

    /// <summary>
    /// Latest arrival times for each job in the route.
    /// </summary>
    public List<long> Latest { get; }

    /// <summary>
    /// Total action time (setup + service or just service) for each job.
    /// </summary>
    public List<long> ActionTime { get; }

    /// <summary>
    /// Earliest possible end time for the route.
    /// </summary>
    public long EarliestEnd { get; private set; }

    /// <summary>
    /// Number of breaks to take before each job rank (and before route end).
    /// </summary>
    public List<int> BreaksAtRank { get; }

    /// <summary>
    /// Cumulative break counts up to each rank.
    /// </summary>
    public List<int> BreaksCounts { get; }

    /// <summary>
    /// Earliest time for each break.
    /// </summary>
    public List<long> BreakEarliest { get; }

    /// <summary>
    /// Latest time for each break.
    /// </summary>
    public List<long> BreakLatest { get; }

    /// <summary>
    /// Travel time margin before each break due to time window constraints.
    /// </summary>
    public List<long> BreaksTravelMarginBefore { get; }

    /// <summary>
    /// Travel time margin after each break due to time window constraints.
    /// </summary>
    public List<long> BreaksTravelMarginAfter { get; }

    /// <summary>
    /// Forward smallest break load margins.
    /// </summary>
    public List<Amount> FwdSmallestBreaksLoadMargin { get; }

    /// <summary>
    /// Backward smallest break load margins.
    /// </summary>
    public List<Amount> BwdSmallestBreaksLoadMargin { get; }

    public TWRoute() : base()
    {
        // Parameterless constructor for generic constraints
        Earliest = new List<long>();
        Latest = new List<long>();
        ActionTime = new List<long>();
        BreakEarliest = new List<long>();
        BreakLatest = new List<long>();
        BreaksTravelMarginBefore = new List<long>();
        BreaksTravelMarginAfter = new List<long>();
        FwdSmallestBreaksLoadMargin = new List<Amount>();
        BwdSmallestBreaksLoadMargin = new List<Amount>();
        BreaksAtRank = new List<int>();
        BreaksCounts = new List<int>();
    }
    
    public TWRoute(Input input, int vehicleRank) : base(input, vehicleRank)
    {
        var vehicle = input.Vehicles[vehicleRank];
        _vStart = vehicle.TimeWindow.Start;
        _vEnd = vehicle.TimeWindow.End;

        Earliest = new List<long>();
        Latest = new List<long>();
        ActionTime = new List<long>();

        // Initialize breaks tracking
        var breaksCount = vehicle.Breaks.Count;
        BreaksAtRank = new List<int> { breaksCount };
        BreaksCounts = new List<int> { breaksCount };
        BreakEarliest = new List<long>(breaksCount);
        BreakLatest = new List<long>(breaksCount);
        BreaksTravelMarginBefore = new List<long>(breaksCount);
        BreaksTravelMarginAfter = new List<long>(breaksCount);
        FwdSmallestBreaksLoadMargin = new List<Amount>(breaksCount);
        BwdSmallestBreaksLoadMargin = new List<Amount>(breaksCount);

        // Initialize break earliest/latest times and load margins
        InitializeBreaks(input, vehicle);
    }

    private void InitializeBreaks(Input input, Vehicle vehicle)
    {
        long previousEarliest = _vStart;
        var fwdSmallestMargin = new Amount(new long[input.ZeroAmount.Size]);
        for (int i = 0; i < fwdSmallestMargin.Size; i++)
        {
            fwdSmallestMargin[i] = long.MaxValue;
        }

        // Forward pass for earliest times
        for (int i = 0; i < vehicle.Breaks.Count; i++)
        {
            var b = vehicle.Breaks[i];
            
            // Find first valid time window
            TimeWindow? validTw = null;
            foreach (var tw in b.TimeWindows)
            {
                if (previousEarliest <= tw.End)
                {
                    validTw = tw;
                    break;
                }
            }
            
            if (validTw == null)
                throw new ArgumentException($"Inconsistent breaks for vehicle {vehicle.Id}");

            var earliest = Math.Max(previousEarliest, validTw.Value.Start);
            BreakEarliest.Add(earliest);
            BreaksTravelMarginBefore.Add(earliest - previousEarliest);

            previousEarliest = earliest + b.Duration;

            // Update load margins if break has max load constraint
            if (b.MaxLoad != null)
            {
                for (int a = 0; a < fwdSmallestMargin.Size; a++)
                {
                    if (b.MaxLoad[a] < fwdSmallestMargin[a])
                        fwdSmallestMargin[a] = b.MaxLoad[a];
                }
            }
            FwdSmallestBreaksLoadMargin.Add(new Amount(fwdSmallestMargin));
        }

        // Backward pass for latest times
        long nextLatest = _vEnd;
        var bwdSmallestMargin = new Amount(new long[input.ZeroAmount.Size]);
        for (int i = 0; i < bwdSmallestMargin.Size; i++)
        {
            bwdSmallestMargin[i] = long.MaxValue;
        }

        for (int ri = 0; ri < vehicle.Breaks.Count; ri++)
        {
            int i = vehicle.Breaks.Count - 1 - ri;
            var b = vehicle.Breaks[i];

            if (nextLatest < b.Duration)
                throw new ArgumentException($"Inconsistent breaks for vehicle {vehicle.Id}");

            nextLatest -= b.Duration;

            // Find last valid time window
            TimeWindow? validTw = null;
            for (int twi = b.TimeWindows.Count - 1; twi >= 0; twi--)
            {
                if (b.TimeWindows[twi].Start <= nextLatest)
                {
                    validTw = b.TimeWindows[twi];
                    break;
                }
            }

            if (validTw == null)
                throw new ArgumentException($"Inconsistent breaks for vehicle {vehicle.Id}");

            var latest = Math.Min(nextLatest, validTw.Value.End);
            BreakLatest.Insert(i, latest);
            BreaksTravelMarginAfter.Insert(i, nextLatest - latest);

            nextLatest = latest;

            if (latest < BreakEarliest[i])
                throw new ArgumentException($"Inconsistent breaks for vehicle {vehicle.Id}");

            // Update load margins if break has max load constraint
            if (b.MaxLoad != null)
            {
                for (int a = 0; a < bwdSmallestMargin.Size; a++)
                {
                    if (b.MaxLoad[a] < bwdSmallestMargin[a])
                        bwdSmallestMargin[a] = b.MaxLoad[a];
                }
            }
            BwdSmallestBreaksLoadMargin.Insert(i, new Amount(bwdSmallestMargin));
        }
    }

    /// <summary>
    /// Get information about the previous step before inserting at rank.
    /// </summary>
    private PreviousInfo GetPreviousInfo(Input input, int jobRank, int rank)
    {
        var vehicle = input.Vehicles[VehicleRank];
        var job = input.Jobs[jobRank];

        var previous = new PreviousInfo(_vStart, 0);
        
        if (rank > 0)
        {
            var previousJob = input.Jobs[Route[rank - 1]];
            previous.Earliest = Earliest[rank - 1] + ActionTime[rank - 1];
            previous.Travel = input.GetDuration(VehicleRank, 
                Array.IndexOf(input.Locations.ToArray(), previousJob.Location),
                Array.IndexOf(input.Locations.ToArray(), job.Location));
            previous.LocationIndex = Array.IndexOf(input.Locations.ToArray(), previousJob.Location);
        }
        else if (HasStart)
        {
            previous.LocationIndex = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
            previous.Travel = input.GetDuration(VehicleRank, 
                previous.LocationIndex.Value,
                Array.IndexOf(input.Locations.ToArray(), job.Location));
        }

        return previous;
    }

    /// <summary>
    /// Get information about the next step after inserting at rank.
    /// </summary>
    private NextInfo GetNextInfo(Input input, int jobRank, int rank)
    {
        var vehicle = input.Vehicles[VehicleRank];
        var job = input.Jobs[jobRank];

        var next = new NextInfo(_vEnd, 0);
        
        if (rank == Route.Count)
        {
            if (HasEnd)
            {
                next.Travel = input.GetDuration(VehicleRank,
                    Array.IndexOf(input.Locations.ToArray(), job.Location),
                    Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation));
            }
        }
        else
        {
            next.Latest = Latest[rank];
            next.Travel = input.GetDuration(VehicleRank,
                Array.IndexOf(input.Locations.ToArray(), job.Location),
                Array.IndexOf(input.Locations.ToArray(), input.Jobs[Route[rank]].Location));
        }

        return next;
    }

    /// <summary>
    /// Check if adding a job at the given rank is valid for time windows.
    /// </summary>
    public override bool IsValidAdditionForTw(Input input, int jobRank, int rank)
    {
        if (rank > Route.Count)
            throw new ArgumentOutOfRangeException(nameof(rank));
        if (input.Jobs[jobRank].Type != JobType.Single)
            throw new ArgumentException("Expected single job");

        return IsValidAdditionForTw(input, input.Jobs[jobRank].Delivery,
            new[] { jobRank }, 0, 1, rank, rank);
    }

    /// <summary>
    /// Check if adding a job at the given rank is valid for time windows without max load checks.
    /// </summary>
    public override bool IsValidAdditionForTwWithoutMaxLoad(Input input, int jobRank, int rank)
    {
        if (rank > Route.Count)
            throw new ArgumentOutOfRangeException(nameof(rank));

        return IsValidAdditionForTw(input, input.Jobs[jobRank].Delivery,
            new[] { jobRank }, 0, 1, rank, rank);
    }

    /// <summary>
    /// Check validity for inclusion of a range of jobs in the route.
    /// </summary>
    public override bool IsValidAdditionForTw(
        Input input,
        Amount delivery,
        int[] jobs,
        int firstJob,
        int lastJob,
        int firstRank,
        int lastRank)
    {
        // This is a simplified implementation
        // Full implementation would need to:
        // 1. Check time window feasibility for all jobs
        // 2. Handle break constraints
        // 3. Update earliest/latest times
        // 4. Check load constraints if required

        // For now, just check basic time windows
        var vehicle = input.Vehicles[VehicleRank];
        long currentTime = _vStart;

        // Check if we can reach the first job in time
        if (firstRank > 0 && Earliest.Count > firstRank - 1)
        {
            currentTime = Earliest[firstRank - 1] + ActionTime[firstRank - 1];
        }

        // Check each job to be inserted
        for (int i = firstJob; i < lastJob; i++)
        {
            var job = input.Jobs[jobs[i]];
            
            // Add travel time
            if (i == firstJob && firstRank > 0)
            {
                var prevLoc = Array.IndexOf(input.Locations.ToArray(), 
                    input.Jobs[Route[firstRank - 1]].Location);
                var jobLoc = Array.IndexOf(input.Locations.ToArray(), job.Location);
                currentTime += input.GetDuration(VehicleRank, prevLoc, jobLoc);
            }
            else if (i > firstJob)
            {
                var prevLoc = Array.IndexOf(input.Locations.ToArray(), 
                    input.Jobs[jobs[i - 1]].Location);
                var jobLoc = Array.IndexOf(input.Locations.ToArray(), job.Location);
                currentTime += input.GetDuration(VehicleRank, prevLoc, jobLoc);
            }

            // Check if we can meet any time window
            bool foundValidTw = false;
            foreach (var tw in job.TimeWindows)
            {
                if (currentTime <= tw.End)
                {
                    currentTime = Math.Max(currentTime, tw.Start);
                    foundValidTw = true;
                    break;
                }
            }

            if (!foundValidTw)
                return false;

            // Add service time
            currentTime += job.Service;
        }

        // Check if we can still meet subsequent constraints
        if (lastRank < Route.Count && Latest.Count > lastRank)
        {
            var nextJob = input.Jobs[Route[lastRank]];
            var lastInsertedLoc = Array.IndexOf(input.Locations.ToArray(), 
                input.Jobs[jobs[lastJob - 1]].Location);
            var nextLoc = Array.IndexOf(input.Locations.ToArray(), nextJob.Location);
            var travelTime = input.GetDuration(VehicleRank, lastInsertedLoc, nextLoc);

            if (currentTime + travelTime > Latest[lastRank])
                return false;
        }

        return true;
    }
    
    /// <summary>
    /// Forward update earliest times from a given rank.
    /// </summary>
    public void FwdUpdateEarliestFrom(Input input, int rank)
    {
        var vehicle = input.Vehicles[VehicleRank];
        
        long currentEarliest = Earliest[rank];
        bool handleLastBreaks = true;
        
        for (int i = rank + 1; i < Route.Count; i++)
        {
            var nextJob = input.Jobs[Route[i]];
            var remainingTravelTime = input.GetDuration(VehicleRank,
                Array.IndexOf(input.Locations.ToArray(), input.Jobs[Route[i - 1]].Location),
                Array.IndexOf(input.Locations.ToArray(), nextJob.Location));
            var previousActionTime = ActionTime[i - 1];
            
            // Update earliest dates and margins for breaks
            var breakRank = BreaksCounts[i] - BreaksAtRank[i];
            
            for (int r = 0; r < BreaksAtRank[i]; r++, breakRank++)
            {
                var b = vehicle.Breaks[breakRank];
                currentEarliest += previousActionTime;
                
                // Find valid time window
                TimeWindow? validTw = null;
                foreach (var tw in b.TimeWindows)
                {
                    if (currentEarliest <= tw.End)
                    {
                        validTw = tw;
                        break;
                    }
                }
                
                if (validTw == null)
                    throw new InvalidOperationException("No valid time window for break");
                
                if (currentEarliest < validTw.Value.Start)
                {
                    var margin = validTw.Value.Start - currentEarliest;
                    BreaksTravelMarginBefore[breakRank] = margin;
                    if (margin < remainingTravelTime)
                        remainingTravelTime -= margin;
                    else
                        remainingTravelTime = 0;
                    
                    currentEarliest = validTw.Value.Start;
                }
                else
                {
                    BreaksTravelMarginBefore[breakRank] = 0;
                }
                
                BreakEarliest[breakRank] = currentEarliest;
                previousActionTime = b.Duration;
            }
            
            // Back to the job after breaks
            currentEarliest += previousActionTime + remainingTravelTime;
            
            // Find valid time window for job
            TimeWindow? jobTw = null;
            foreach (var tw in nextJob.TimeWindows)
            {
                if (currentEarliest <= tw.End)
                {
                    jobTw = tw;
                    break;
                }
            }
            
            if (jobTw == null)
                throw new InvalidOperationException("No valid time window for job");
                
            currentEarliest = Math.Max(currentEarliest, jobTw.Value.Start);
            
            // Check consistency
            if (i < Latest.Count && currentEarliest > Latest[i] && !(i == rank + 1 && Latest[i] == 0))
                throw new InvalidOperationException("Time window infeasibility");
                
            if (currentEarliest == Earliest[i])
            {
                // No further updates needed
                handleLastBreaks = false;
                break;
            }
            
            Earliest[i] = currentEarliest;
        }
        
        if (handleLastBreaks && Route.Count > 0)
        {
            // Update earliest dates for breaks before route end
            var i = Route.Count;
            var remainingTravelTime = HasEnd ?
                input.GetDuration(VehicleRank,
                    Array.IndexOf(input.Locations.ToArray(), input.Jobs[Route[i - 1]].Location),
                    Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation)) : 0;
            
            var previousActionTime = ActionTime[i - 1];
            var breakRank = BreaksCounts[i] - BreaksAtRank[i];
            
            for (int r = 0; r < BreaksAtRank[i]; r++, breakRank++)
            {
                var b = vehicle.Breaks[breakRank];
                currentEarliest += previousActionTime;
                
                // Find valid time window
                TimeWindow? validTw = null;
                foreach (var tw in b.TimeWindows)
                {
                    if (currentEarliest <= tw.End)
                    {
                        validTw = tw;
                        break;
                    }
                }
                
                if (validTw == null)
                    throw new InvalidOperationException("No valid time window for break");
                
                if (currentEarliest < validTw.Value.Start)
                {
                    var margin = validTw.Value.Start - currentEarliest;
                    BreaksTravelMarginBefore[breakRank] = margin;
                    if (margin < remainingTravelTime)
                        remainingTravelTime -= margin;
                    else
                        remainingTravelTime = 0;
                    
                    currentEarliest = validTw.Value.Start;
                }
                else
                {
                    BreaksTravelMarginBefore[breakRank] = 0;
                }
                
                BreakEarliest[breakRank] = currentEarliest;
                previousActionTime = b.Duration;
            }
            
            EarliestEnd = currentEarliest + previousActionTime + remainingTravelTime;
        }
    }
    
    /// <summary>
    /// Backward update latest times from a given rank.
    /// </summary>
    public void BwdUpdateLatestFrom(Input input, int rank)
    {
        var vehicle = input.Vehicles[VehicleRank];
        
        long currentLatest = Latest[rank];
        // bool handleFirstBreaks = true; // Not implemented yet
        
        for (int nextI = rank; nextI > 0; nextI--)
        {
            var previousJob = input.Jobs[Route[nextI - 1]];
            var remainingTravelTime = input.GetDuration(VehicleRank,
                Array.IndexOf(input.Locations.ToArray(), previousJob.Location),
                Array.IndexOf(input.Locations.ToArray(), input.Jobs[Route[nextI]].Location));
            
            // Update latest dates and margins for breaks
            var breakRank = BreaksCounts[nextI];
            
            for (int r = 0; r < BreaksAtRank[nextI]; r++)
            {
                breakRank--;
                var b = vehicle.Breaks[breakRank];
                
                if (b.Duration > currentLatest)
                    throw new InvalidOperationException("Break duration exceeds available time");
                    
                currentLatest -= b.Duration;
                
                // Find valid time window (reverse order)
                TimeWindow? validTw = null;
                for (int twi = b.TimeWindows.Count - 1; twi >= 0; twi--)
                {
                    if (b.TimeWindows[twi].Start <= currentLatest)
                    {
                        validTw = b.TimeWindows[twi];
                        break;
                    }
                }
                
                if (validTw == null)
                    throw new InvalidOperationException("No valid time window for break");
                
                if (validTw.Value.End < currentLatest)
                {
                    var margin = currentLatest - validTw.Value.End;
                    BreaksTravelMarginAfter[breakRank] = margin;
                    if (margin < remainingTravelTime)
                        remainingTravelTime -= margin;
                    else
                        remainingTravelTime = 0;
                    
                    currentLatest = validTw.Value.End;
                }
                else
                {
                    BreaksTravelMarginAfter[breakRank] = 0;
                }
                
                BreakLatest[breakRank] = currentLatest;
            }
            
            // Back to the previous job
            currentLatest -= ActionTime[nextI - 1] + remainingTravelTime;
            
            // Find valid time window for previous job
            TimeWindow? jobTw = null;
            for (int twi = previousJob.TimeWindows.Count - 1; twi >= 0; twi--)
            {
                if (previousJob.TimeWindows[twi].Start <= currentLatest)
                {
                    jobTw = previousJob.TimeWindows[twi];
                    break;
                }
            }
            
            if (jobTw == null)
                throw new InvalidOperationException("No valid time window for job");
                
            currentLatest = Math.Min(currentLatest, jobTw.Value.End);
            
            // Check consistency
            if (currentLatest < Earliest[nextI - 1])
                throw new InvalidOperationException("Time window infeasibility");
                
            if (currentLatest == Latest[nextI - 1])
            {
                // No further updates needed
                // handleFirstBreaks = false;
                break;
            }
            
            Latest[nextI - 1] = currentLatest;
        }
        
        // Handle breaks at start if needed
        // This would be similar to the end breaks handling but in reverse
    }

    /// <summary>
    /// Check if removing jobs is valid.
    /// </summary>
    public override bool IsValidRemoval(Input input, int rank, int count)
    {
        if (Route.Count == 0)
            throw new InvalidOperationException("Cannot remove from empty route");
        if (rank + count > Route.Count)
            throw new ArgumentOutOfRangeException();

        return IsValidAdditionForTw(input, input.ZeroAmount, 
            Array.Empty<int>(), 0, 0, rank, rank + count);
    }

    /// <summary>
    /// Replace jobs in the route with a new sequence.
    /// </summary>
    public new void Replace(
        Input input,
        Amount delivery,
        int[] jobs,
        int firstJob,
        int lastJob,
        int firstRank,
        int lastRank)
    {
        // Remove old jobs
        for (int i = lastRank - 1; i >= firstRank; i--)
        {
            Route.RemoveAt(i);
            if (i < Earliest.Count) Earliest.RemoveAt(i);
            if (i < Latest.Count) Latest.RemoveAt(i);
            if (i < ActionTime.Count) ActionTime.RemoveAt(i);
        }

        // Insert new jobs
        for (int i = firstJob; i < lastJob; i++)
        {
            Route.Insert(firstRank + i - firstJob, jobs[i]);
        }

        // Update capacity tracking
        UpdateAmounts();

        // Update time windows
        UpdateTimeWindows(input);
    }

    /// <summary>
    /// Update time window information after route changes.
    /// </summary>
    private void UpdateTimeWindows(Input input)
    {
        Earliest.Clear();
        Latest.Clear();
        ActionTime.Clear();

        if (Route.Count == 0)
        {
            EarliestEnd = _vStart;
            return;
        }

        var vehicle = input.Vehicles[VehicleRank];

        // Forward pass - compute earliest times
        long currentTime = _vStart;
        for (int i = 0; i < Route.Count; i++)
        {
            var job = input.Jobs[Route[i]];
            
            // Add travel time
            if (i == 0 && HasStart)
            {
                var startIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.StartLocation);
                var jobIdx = Array.IndexOf(input.Locations.ToArray(), job.Location);
                currentTime += input.GetDuration(VehicleRank, startIdx, jobIdx);
            }
            else if (i > 0)
            {
                var prevIdx = Array.IndexOf(input.Locations.ToArray(), 
                    input.Jobs[Route[i - 1]].Location);
                var jobIdx = Array.IndexOf(input.Locations.ToArray(), job.Location);
                currentTime += input.GetDuration(VehicleRank, prevIdx, jobIdx);
            }

            // Find feasible time window
            foreach (var tw in job.TimeWindows)
            {
                if (currentTime <= tw.End)
                {
                    currentTime = Math.Max(currentTime, tw.Start);
                    break;
                }
            }

            Earliest.Add(currentTime);
            ActionTime.Add(job.Service);
            currentTime += job.Service;
        }

        // Compute earliest end
        if (HasEnd)
        {
            var lastIdx = Array.IndexOf(input.Locations.ToArray(), 
                input.Jobs[Route[^1]].Location);
            var endIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
            EarliestEnd = currentTime + input.GetDuration(VehicleRank, lastIdx, endIdx);
        }
        else
        {
            EarliestEnd = currentTime;
        }

        // Backward pass - compute latest times
        currentTime = _vEnd;
        for (int i = Route.Count - 1; i >= 0; i--)
        {
            var job = input.Jobs[Route[i]];
            
            // Subtract travel time to next
            if (i == Route.Count - 1 && HasEnd)
            {
                var jobIdx = Array.IndexOf(input.Locations.ToArray(), job.Location);
                var endIdx = Array.IndexOf(input.Locations.ToArray(), vehicle.EndLocation);
                currentTime -= input.GetDuration(VehicleRank, jobIdx, endIdx);
            }
            else if (i < Route.Count - 1)
            {
                var jobIdx = Array.IndexOf(input.Locations.ToArray(), job.Location);
                var nextIdx = Array.IndexOf(input.Locations.ToArray(), 
                    input.Jobs[Route[i + 1]].Location);
                currentTime -= input.GetDuration(VehicleRank, jobIdx, nextIdx);
            }

            // Subtract service time
            currentTime -= job.Service;

            // Find feasible time window
            for (int twi = job.TimeWindows.Count - 1; twi >= 0; twi--)
            {
                if (job.TimeWindows[twi].Start <= currentTime)
                {
                    currentTime = Math.Min(currentTime, job.TimeWindows[twi].End);
                    break;
                }
            }

            if (i < Latest.Count)
                Latest[i] = currentTime;
            else
                Latest.Insert(i, currentTime);
        }
    }

    #region IRoute Implementation

    public new void Initialize(Input input, int vehicleIndex)
    {
        // Already initialized in constructor
    }

    public new void InsertJob(int jobRank, int position)
    {
        if (!IsValidAdditionForTw(_input, jobRank, position))
            throw new InvalidOperationException("Cannot insert job - time window violation");

        base.InsertJob(jobRank, position);
        UpdateTimeWindows(_input);
    }

    #endregion
}

/// <summary>
/// Information about the previous step in a route.
/// </summary>
internal struct PreviousInfo
{
    public long Earliest { get; set; }
    public long Travel { get; set; }
    public int? LocationIndex { get; set; }

    public PreviousInfo(long earliest, long travel)
    {
        Earliest = earliest;
        Travel = travel;
        LocationIndex = null;
    }
}

/// <summary>
/// Information about the next step in a route.
/// </summary>
internal struct NextInfo
{
    public long Latest { get; set; }
    public long Travel { get; set; }

    public NextInfo(long latest, long travel)
    {
        Latest = latest;
        Travel = travel;
    }
}

/// <summary>
/// Job type for single jobs (not pickup/delivery pairs).
/// </summary>
internal class SingleJob : Job
{
    public SingleJob(string id, Location location) : base(id, location)
    {
    }
}