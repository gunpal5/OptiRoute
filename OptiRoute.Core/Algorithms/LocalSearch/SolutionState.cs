using OptiRoute.Core.Models;
using OptiRoute.Core.Problems.VRP;

namespace OptiRoute.Core.Algorithms.LocalSearch;

/// <summary>
/// Maintains the state of a solution during local search (based on VROOM).
/// </summary>
internal class SolutionState
{
    private readonly Input _input;

    /// <summary>
    /// Current evaluation for each route.
    /// </summary>
    public List<Eval> RouteEvals { get; }

    /// <summary>
    /// fwd_costs[v][new_v][i] stores the total cost from job at rank 0
    /// to job at rank i in the route for vehicle v, from the point of
    /// view of a vehicle new_v.
    /// </summary>
    public List<List<List<Eval>>> FwdCosts { get; }

    /// <summary>
    /// bwd_costs[v][new_v][i] stores the total cost from job at rank i 
    /// to job at rank 0 (i.e. when reversing all edges) in the route 
    /// for vehicle v, from the point of view of a vehicle new_v.
    /// </summary>
    public List<List<List<Eval>>> BwdCosts { get; }

    /// <summary>
    /// Node gains for each position in each route.
    /// Gain of removing job at rank r in route v.
    /// </summary>
    public List<List<Eval>> NodeGains { get; }

    /// <summary>
    /// Edge gains for each position in each route.
    /// Gain of removing edge starting at rank r in route v.
    /// </summary>
    public List<List<Eval>> EdgeGains { get; }

    /// <summary>
    /// For each vehicle and pickup rank, stores the matching delivery rank.
    /// </summary>
    public List<Dictionary<int, int>> MatchingDeliveryRank { get; }

    /// <summary>
    /// Cheapest job rank in route v after job at rank r for pickup.
    /// </summary>
    public List<List<int>> CheapestJobRankInRoutesFromPickup { get; }

    /// <summary>
    /// Cheapest job rank in route v after job at rank r for delivery.
    /// </summary>
    public List<List<int>> CheapestJobRankInRoutesFromDelivery { get; }
    
    /// <summary>
    /// Store unassigned jobs.
    /// </summary>
    public HashSet<int> Unassigned { get; }
    
    /// <summary>
    /// Skill compatibility tracking - max rank reachable by each vehicle pair.
    /// </summary>
    public List<List<int>> FwdSkillRank { get; }
    public List<List<int>> BwdSkillRank { get; }
    
    /// <summary>
    /// Priority tracking - cumulative priorities forward and backward.
    /// </summary>
    public List<List<int>> FwdPriority { get; }
    public List<List<int>> BwdPriority { get; }
    
    /// <summary>
    /// PD gains for pickup-delivery pairs.
    /// </summary>
    public List<List<Eval>> PdGains { get; }
    
    /// <summary>
    /// For deliveries, stores matching pickup rank.
    /// </summary>
    public List<Dictionary<int, int>> MatchingPickupRank { get; }
    
    /// <summary>
    /// Edge evaluations around nodes and edges.
    /// </summary>
    public List<List<Eval>> EdgeEvalsAroundNode { get; }
    public List<List<Eval>> EdgeEvalsAroundEdge { get; }
    
    /// <summary>
    /// Best candidates for node/edge removal.
    /// </summary>
    public List<int> NodeCandidates { get; }
    public List<int> EdgeCandidates { get; }
    
    /// <summary>
    /// Insertion rank bounds for time window feasibility.
    /// </summary>
    public List<List<int>> InsertionRanksBegin { get; }
    public List<List<int>> InsertionRanksEnd { get; }
    public List<List<int>> WeakInsertionRanksBegin { get; }
    public List<List<int>> WeakInsertionRanksEnd { get; }
    
    /// <summary>
    /// Cheapest job ranks to/from other routes.
    /// </summary>
    public List<List<List<int>>> CheapestJobRankInRoutesFrom { get; }
    public List<List<List<int>>> CheapestJobRankInRoutesTo { get; }

    public SolutionState(Input input, int vehicleCount)
    {
        _input = input;
        
        RouteEvals = new List<Eval>(vehicleCount);
        FwdCosts = new List<List<List<Eval>>>(vehicleCount);
        BwdCosts = new List<List<List<Eval>>>(vehicleCount);
        NodeGains = new List<List<Eval>>(vehicleCount);
        EdgeGains = new List<List<Eval>>(vehicleCount);
        MatchingDeliveryRank = new List<Dictionary<int, int>>(vehicleCount);
        CheapestJobRankInRoutesFromPickup = new List<List<int>>(vehicleCount);
        CheapestJobRankInRoutesFromDelivery = new List<List<int>>(vehicleCount);
        Unassigned = new HashSet<int>();
        
        FwdSkillRank = new List<List<int>>(vehicleCount);
        BwdSkillRank = new List<List<int>>(vehicleCount);
        FwdPriority = new List<List<int>>(vehicleCount);
        BwdPriority = new List<List<int>>(vehicleCount);
        PdGains = new List<List<Eval>>(vehicleCount);
        MatchingPickupRank = new List<Dictionary<int, int>>(vehicleCount);
        EdgeEvalsAroundNode = new List<List<Eval>>(vehicleCount);
        EdgeEvalsAroundEdge = new List<List<Eval>>(vehicleCount);
        NodeCandidates = new List<int>(vehicleCount);
        EdgeCandidates = new List<int>(vehicleCount);
        InsertionRanksBegin = new List<List<int>>(vehicleCount);
        InsertionRanksEnd = new List<List<int>>(vehicleCount);
        WeakInsertionRanksBegin = new List<List<int>>(vehicleCount);
        WeakInsertionRanksEnd = new List<List<int>>(vehicleCount);
        CheapestJobRankInRoutesFrom = new List<List<List<int>>>(vehicleCount);
        CheapestJobRankInRoutesTo = new List<List<List<int>>>(vehicleCount);

        for (int v = 0; v < vehicleCount; v++)
        {
            RouteEvals.Add(new Eval(0, 0, 0));
            FwdCosts.Add(new List<List<Eval>>(vehicleCount));
            BwdCosts.Add(new List<List<Eval>>(vehicleCount));
            NodeGains.Add(new List<Eval>());
            EdgeGains.Add(new List<Eval>());
            MatchingDeliveryRank.Add(new Dictionary<int, int>());
            CheapestJobRankInRoutesFromPickup.Add(new List<int>());
            CheapestJobRankInRoutesFromDelivery.Add(new List<int>());
            
            FwdSkillRank.Add(new List<int>(vehicleCount));
            BwdSkillRank.Add(new List<int>(vehicleCount));
            FwdPriority.Add(new List<int>());
            BwdPriority.Add(new List<int>());
            PdGains.Add(new List<Eval>());
            MatchingPickupRank.Add(new Dictionary<int, int>());
            EdgeEvalsAroundNode.Add(new List<Eval>());
            EdgeEvalsAroundEdge.Add(new List<Eval>());
            NodeCandidates.Add(-1);
            EdgeCandidates.Add(-1);
            InsertionRanksBegin.Add(new List<int>());
            InsertionRanksEnd.Add(new List<int>());
            WeakInsertionRanksBegin.Add(new List<int>());
            WeakInsertionRanksEnd.Add(new List<int>());
            CheapestJobRankInRoutesFrom.Add(new List<List<int>>(vehicleCount));
            CheapestJobRankInRoutesTo.Add(new List<List<int>>(vehicleCount));
            
            // Initialize skill ranks for all vehicle pairs
            for (int v2 = 0; v2 < vehicleCount; v2++)
            {
                FwdSkillRank[v].Add(0);
                BwdSkillRank[v].Add(0);
                CheapestJobRankInRoutesFrom[v].Add(new List<int>());
                CheapestJobRankInRoutesTo[v].Add(new List<int>());
            }

            // Initialize forward and backward costs for all vehicle pairs
            for (int v2 = 0; v2 < vehicleCount; v2++)
            {
                FwdCosts[v].Add(new List<Eval>());
                BwdCosts[v].Add(new List<Eval>());
            }
        }
    }

    /// <summary>
    /// Setup the solution state for all routes.
    /// </summary>
    public void Setup(List<IRoute> solution)
    {
        for (int v = 0; v < solution.Count; v++)
        {
            UpdateRoute(v, solution[v]);
        }
        
        // Initialize unassigned jobs
        Unassigned.Clear();
        for (int j = 0; j < _input.Jobs.Count; j++)
        {
            Unassigned.Add(j);
        }
        
        // Remove assigned jobs
        foreach (var route in solution)
        {
            foreach (var j in route.Route)
            {
                Unassigned.Remove(j);
            }
        }
    }
    
    /// <summary>
    /// Update state for a single route.
    /// </summary>
    public void UpdateRoute(int vehicleIndex, IRoute route)
    {
        var vehicle = _input.Vehicles[vehicleIndex];
        var routeJobs = route.Route;

        // Update route evaluation
        RouteEvals[vehicleIndex] = ComputeRouteEval(vehicle, routeJobs);

        // Update forward and backward costs
        UpdateCosts(vehicleIndex, routeJobs);

        // Update node gains
        UpdateNodeGains(vehicleIndex, vehicle, route, routeJobs);

        // Update edge gains
        UpdateEdgeGains(vehicleIndex, vehicle, route, routeJobs);

        // Update matching delivery ranks for pickup-delivery pairs
        UpdateMatchingDeliveryRanks(vehicleIndex, routeJobs);

        // Update cheapest job ranks
        UpdateCheapestJobRanks(vehicleIndex, vehicle, routeJobs);
        
        // Update priority tracking
        UpdatePriorities(vehicleIndex, routeJobs);
        
        // Update skill compatibility
        UpdateSkillCompatibility(vehicleIndex, routeJobs);
        
        // Update edge evaluations
        UpdateEdgeEvaluations(vehicleIndex, vehicle, routeJobs);
        
        // Update PD gains for pickup-delivery pairs
        UpdatePdGains(vehicleIndex, vehicle, routeJobs);
    }

    private Eval ComputeRouteEval(Vehicle vehicle, List<int> route)
    {
        var eval = new Eval(0, 0, 0);
        
        if (route.Count == 0)
            return eval;

        eval = eval with { Cost = eval.Cost + vehicle.Costs.Fixed };

        // Add travel from start to first job
        if (vehicle.StartLocation != null)
        {
            var startIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.StartLocation);
            var firstJobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[0]].Location);
            var startEval = _input.GetEval(_input.Vehicles.ToList().IndexOf(vehicle), startIdx, firstJobIdx);
            eval = eval + startEval;
        }

        // Add travel between jobs
        for (int i = 0; i < route.Count - 1; i++)
        {
            var fromIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
            var toIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i + 1]].Location);
            var jobEval = _input.GetEval(_input.Vehicles.ToList().IndexOf(vehicle), fromIdx, toIdx);
            eval = eval + jobEval;
        }

        // Add travel from last job to end
        if (vehicle.EndLocation != null)
        {
            var lastJobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[^1]].Location);
            var endIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.EndLocation);
            var endEval = _input.GetEval(_input.Vehicles.ToList().IndexOf(vehicle), lastJobIdx, endIdx);
            eval = eval + endEval;
        }

        return eval;
    }

    private void UpdateCosts(int vehicleIndex, List<int> route)
    {
        // Update forward costs from perspective of all vehicles
        for (int v2 = 0; v2 < _input.Vehicles.Count; v2++)
        {
            FwdCosts[vehicleIndex][v2].Clear();
            BwdCosts[vehicleIndex][v2].Clear();

            if (route.Count == 0) continue;

            var vehicle2 = _input.Vehicles[v2];
            
            // Forward costs
            var fwdCum = new Eval(0, 0, 0);
            for (int i = 0; i < route.Count; i++)
            {
                if (i == 0)
                {
                    // Cost from start to first job
                    if (vehicle2.StartLocation != null)
                    {
                        var startIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle2.StartLocation);
                        var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
                        fwdCum = _input.GetEval(v2, startIdx, jobIdx);
                    }
                }
                else
                {
                    // Cost from previous job to current job
                    var prevIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i-1]].Location);
                    var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
                    fwdCum = fwdCum + _input.GetEval(v2, prevIdx, jobIdx);
                }
                FwdCosts[vehicleIndex][v2].Add(fwdCum);
            }

            // Backward costs (reversed edges)
            var bwdCum = new Eval(0, 0, 0);
            for (int i = route.Count - 1; i >= 0; i--)
            {
                if (i == route.Count - 1)
                {
                    // Cost from end to last job (reversed)
                    if (vehicle2.EndLocation != null)
                    {
                        var endIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle2.EndLocation);
                        var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
                        bwdCum = _input.GetEval(v2, jobIdx, endIdx);
                    }
                }
                else
                {
                    // Cost from current job to next job (reversed)
                    var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
                    var nextIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i+1]].Location);
                    bwdCum = bwdCum + _input.GetEval(v2, nextIdx, jobIdx);
                }
                BwdCosts[vehicleIndex][v2].Insert(0, bwdCum);
            }
        }
    }

    private void UpdateNodeGains(int vehicleIndex, Vehicle vehicle, IRoute fullRoute, List<int> route)
    {
        NodeGains[vehicleIndex].Clear();

        for (int r = 0; r < route.Count; r++)
        {
            var gain = ComputeNodeGain(vehicleIndex, vehicle, route, r);
            NodeGains[vehicleIndex].Add(gain);
        }
    }

    private Eval ComputeNodeGain(int vehicleIndex, Vehicle vehicle, List<int> route, int rank)
    {
        var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[rank]].Location);

        Eval before = new Eval(0, 0, 0);
        Eval after = new Eval(0, 0, 0);
        Eval removed = new Eval(0, 0, 0);

        if (rank == 0)
        {
            // First job
            if (vehicle.StartLocation != null)
            {
                var startIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.StartLocation);
                before = _input.GetEval(vehicleIndex, startIdx, jobIdx);
            }
            if (route.Count > 1)
            {
                var nextIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[1]].Location);
                after = _input.GetEval(vehicleIndex, jobIdx, nextIdx);
                
                if (vehicle.StartLocation != null)
                {
                    var startIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.StartLocation);
                    removed = _input.GetEval(vehicleIndex, startIdx, nextIdx);
                }
            }
            else if (vehicle.EndLocation != null)
            {
                var endIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.EndLocation);
                after = _input.GetEval(vehicleIndex, jobIdx, endIdx);
                
                if (vehicle.StartLocation != null)
                {
                    var startIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.StartLocation);
                    removed = _input.GetEval(vehicleIndex, startIdx, endIdx);
                }
            }
        }
        else if (rank == route.Count - 1)
        {
            // Last job
            var prevIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[rank - 1]].Location);
            before = _input.GetEval(vehicleIndex, prevIdx, jobIdx);
            
            if (vehicle.EndLocation != null)
            {
                var endIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.EndLocation);
                after = _input.GetEval(vehicleIndex, jobIdx, endIdx);
                removed = _input.GetEval(vehicleIndex, prevIdx, endIdx);
            }
        }
        else
        {
            // Middle job
            var prevIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[rank - 1]].Location);
            var nextIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[rank + 1]].Location);
            before = _input.GetEval(vehicleIndex, prevIdx, jobIdx);
            after = _input.GetEval(vehicleIndex, jobIdx, nextIdx);
            removed = _input.GetEval(vehicleIndex, prevIdx, nextIdx);
        }

        return before + after - removed;
    }

    private void UpdateEdgeGains(int vehicleIndex, Vehicle vehicle, IRoute fullRoute, List<int> route)
    {
        EdgeGains[vehicleIndex].Clear();

        for (int r = 0; r < route.Count - 1; r++)
        {
            var fromIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[r]].Location);
            var toIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[r + 1]].Location);
            var edgeEval = _input.GetEval(vehicleIndex, fromIdx, toIdx);
            EdgeGains[vehicleIndex].Add(edgeEval);
        }
    }

    private void UpdateMatchingDeliveryRanks(int vehicleIndex, List<int> route)
    {
        MatchingDeliveryRank[vehicleIndex].Clear();
        MatchingPickupRank[vehicleIndex].Clear();

        for (int r = 0; r < route.Count; r++)
        {
            var job = _input.Jobs[route[r]];
            if (job.Type == JobType.Pickup)
            {
                // Find matching delivery
                for (int d = r + 1; d < route.Count; d++)
                {
                    if (route[d] == route[r] + 1) // Delivery follows pickup in job ranks
                    {
                        MatchingDeliveryRank[vehicleIndex][r] = d;
                        MatchingPickupRank[vehicleIndex][d] = r;
                        break;
                    }
                }
            }
        }
    }

    private void UpdateCheapestJobRanks(int vehicleIndex, Vehicle vehicle, List<int> route)
    {
        CheapestJobRankInRoutesFromPickup[vehicleIndex].Clear();
        CheapestJobRankInRoutesFromDelivery[vehicleIndex].Clear();

        // This would compute cheapest subsequent jobs for pickups/deliveries
        // For now, initialize with defaults
        for (int r = 0; r < route.Count; r++)
        {
            CheapestJobRankInRoutesFromPickup[vehicleIndex].Add(-1);
            CheapestJobRankInRoutesFromDelivery[vehicleIndex].Add(-1);
        }
    }
    
    private void UpdatePriorities(int vehicleIndex, List<int> route)
    {
        FwdPriority[vehicleIndex].Clear();
        BwdPriority[vehicleIndex].Clear();
        
        if (route.Count == 0) return;
        
        // Forward priorities
        int cumPriority = 0;
        for (int i = 0; i < route.Count; i++)
        {
            cumPriority += _input.Jobs[route[i]].Priority;
            FwdPriority[vehicleIndex].Add(cumPriority);
        }
        
        // Backward priorities
        cumPriority = 0;
        for (int i = route.Count - 1; i >= 0; i--)
        {
            cumPriority += _input.Jobs[route[i]].Priority;
            BwdPriority[vehicleIndex].Insert(0, cumPriority);
        }
    }
    
    private void UpdateSkillCompatibility(int vehicleIndex, List<int> route)
    {
        // Update skill compatibility ranks for all vehicle pairs
        for (int v2 = 0; v2 < _input.Vehicles.Count; v2++)
        {
            var v2Skills = _input.Vehicles[v2].Skills;
            
            // Forward skill rank - how far can v2 handle jobs from start
            int fwdRank = 0;
            for (int i = 0; i < route.Count; i++)
            {
                if (_input.Jobs[route[i]].Skills.All(s => v2Skills.Contains(s)))
                {
                    fwdRank = i + 1;
                }
                else
                {
                    break;
                }
            }
            FwdSkillRank[vehicleIndex][v2] = fwdRank;
            
            // Backward skill rank - from where can v2 handle jobs to end
            int bwdRank = route.Count;
            for (int i = route.Count - 1; i >= 0; i--)
            {
                if (_input.Jobs[route[i]].Skills.All(s => v2Skills.Contains(s)))
                {
                    bwdRank = i;
                }
                else
                {
                    break;
                }
            }
            BwdSkillRank[vehicleIndex][v2] = bwdRank;
        }
    }
    
    private void UpdateEdgeEvaluations(int vehicleIndex, Vehicle vehicle, List<int> route)
    {
        EdgeEvalsAroundNode[vehicleIndex].Clear();
        EdgeEvalsAroundEdge[vehicleIndex].Clear();
        
        // Edge evaluations around each node
        for (int i = 0; i < route.Count; i++)
        {
            var eval = new Eval(0, 0, 0);
            
            // Add edge before node
            if (i == 0)
            {
                if (vehicle.StartLocation != null)
                {
                    var startIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.StartLocation);
                    var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
                    eval = eval + _input.GetEval(vehicleIndex, startIdx, jobIdx);
                }
            }
            else
            {
                var prevIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i - 1]].Location);
                var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
                eval = eval + _input.GetEval(vehicleIndex, prevIdx, jobIdx);
            }
            
            // Add edge after node
            if (i == route.Count - 1)
            {
                if (vehicle.EndLocation != null)
                {
                    var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
                    var endIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.EndLocation);
                    eval = eval + _input.GetEval(vehicleIndex, jobIdx, endIdx);
                }
            }
            else
            {
                var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
                var nextIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i + 1]].Location);
                eval = eval + _input.GetEval(vehicleIndex, jobIdx, nextIdx);
            }
            
            EdgeEvalsAroundNode[vehicleIndex].Add(eval);
        }
        
        // Edge evaluations around each edge (for edge removal)
        for (int i = 0; i < route.Count - 1; i++)
        {
            var eval = new Eval(0, 0, 0);
            
            // Add edge before this edge
            if (i == 0)
            {
                if (vehicle.StartLocation != null)
                {
                    var startIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.StartLocation);
                    var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
                    eval = eval + _input.GetEval(vehicleIndex, startIdx, jobIdx);
                }
            }
            else
            {
                var prevIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i - 1]].Location);
                var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i]].Location);
                eval = eval + _input.GetEval(vehicleIndex, prevIdx, jobIdx);
            }
            
            // Add edge after this edge  
            if (i == route.Count - 2)
            {
                if (vehicle.EndLocation != null)
                {
                    var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i + 1]].Location);
                    var endIdx = Array.IndexOf(_input.Locations.ToArray(), vehicle.EndLocation);
                    eval = eval + _input.GetEval(vehicleIndex, jobIdx, endIdx);
                }
            }
            else
            {
                var jobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i + 1]].Location);
                var nextIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[route[i + 2]].Location);
                eval = eval + _input.GetEval(vehicleIndex, jobIdx, nextIdx);
            }
            
            EdgeEvalsAroundEdge[vehicleIndex].Add(eval);
        }
        
        // Update best candidates
        if (NodeGains[vehicleIndex].Count > 0)
        {
            int bestNode = 0;
            var bestGain = NodeGains[vehicleIndex][0];
            for (int i = 1; i < NodeGains[vehicleIndex].Count; i++)
            {
                if (NodeGains[vehicleIndex][i].Cost > bestGain.Cost)
                {
                    bestNode = i;
                    bestGain = NodeGains[vehicleIndex][i];
                }
            }
            NodeCandidates[vehicleIndex] = bestNode;
        }
        
        if (EdgeGains[vehicleIndex].Count > 0)
        {
            int bestEdge = 0;
            var bestGain = EdgeGains[vehicleIndex][0];
            for (int i = 1; i < EdgeGains[vehicleIndex].Count; i++)
            {
                if (EdgeGains[vehicleIndex][i].Cost > bestGain.Cost)
                {
                    bestEdge = i;
                    bestGain = EdgeGains[vehicleIndex][i];
                }
            }
            EdgeCandidates[vehicleIndex] = bestEdge;
        }
    }
    
    private void UpdatePdGains(int vehicleIndex, Vehicle vehicle, List<int> route)
    {
        PdGains[vehicleIndex].Clear();
        
        // Calculate gains for removing pickup-delivery pairs
        foreach (var kvp in MatchingDeliveryRank[vehicleIndex])
        {
            int pickupRank = kvp.Key;
            int deliveryRank = kvp.Value;
            
            // Calculate cost of removing both pickup and delivery
            var gain = NodeGains[vehicleIndex][pickupRank];
            
            // Add gain from removing delivery (adjusted for pickup removal)
            // This is simplified - VROOM has more complex calculation
            if (deliveryRank < NodeGains[vehicleIndex].Count)
            {
                gain = gain + NodeGains[vehicleIndex][deliveryRank];
            }
            
            PdGains[vehicleIndex].Add(gain);
        }
    }
    
    /// <summary>
    /// Update cheapest job ranks from one route to another.
    /// </summary>
    public void UpdateCheapestJobRankInRoutes(List<int> routeFrom, List<int> routeTo, int v1, int v2)
    {
        // Clear existing data
        CheapestJobRankInRoutesFrom[v1][v2].Clear();
        CheapestJobRankInRoutesTo[v1][v2].Clear();
        
        if (routeFrom.Count == 0 || routeTo.Count == 0)
            return;
        
        // For each job in routeFrom, find cheapest job in routeTo
        for (int i = 0; i < routeFrom.Count; i++)
        {
            int cheapestFromRank = 0;
            int cheapestToRank = 0;
            var minFromCost = long.MaxValue;
            var minToCost = long.MaxValue;
            
            var fromJobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[routeFrom[i]].Location);
            
            for (int j = 0; j < routeTo.Count; j++)
            {
                var toJobIdx = Array.IndexOf(_input.Locations.ToArray(), _input.Jobs[routeTo[j]].Location);
                
                // Check from-to cost
                var fromCost = _input.GetEval(v2, toJobIdx, fromJobIdx).Cost;
                if (fromCost < minFromCost)
                {
                    minFromCost = fromCost;
                    cheapestFromRank = j;
                }
                
                // Check to-from cost
                var toCost = _input.GetEval(v2, fromJobIdx, toJobIdx).Cost;
                if (toCost < minToCost)
                {
                    minToCost = toCost;
                    cheapestToRank = j;
                }
            }
            
            CheapestJobRankInRoutesFrom[v1][v2].Add(cheapestFromRank);
            CheapestJobRankInRoutesTo[v1][v2].Add(cheapestToRank);
        }
    }
}