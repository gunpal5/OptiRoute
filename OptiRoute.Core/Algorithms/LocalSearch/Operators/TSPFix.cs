using OptiRoute.Core.Models;
using OptiRoute.Core.Problems.TSP;
using OptiRoute.Core.Utils;

namespace OptiRoute.Core.Algorithms.LocalSearch.Operators;

/// <summary>
/// TSPFix operator - optimizes job ordering within a route using TSP solver (based on VROOM).
/// </summary>
internal class TSPFix : Operator
{
    protected List<int> TspRoute = new();
    private readonly Amount _sDelivery;

    public TSPFix(
        Input input,
        SolutionState solutionState,
        RawRoute route,
        int vehicle)
        : base(OperatorName.TSPFix, input, solutionState, route, vehicle,
               0, route, vehicle, 0) // Use dummy 0 values for unused ranks
    {
        if (route.Route.Count < 2)
            throw new ArgumentException("Route must have at least 2 jobs");
            
        _sDelivery = route.MaxLoadAtRank(0);
    }

    protected override void ComputeGain()
    {
        var jobs = new List<int>(SourceRoute);
        var tsp = new TspSolver(Input, SourceVehicle, jobs);
        TspRoute = tsp.Solve(threadCount: 1, timeout: TimeSpan.Zero);
        
        SourceGain = SolutionState.RouteEvals[SourceVehicle] -
                     Helpers.RouteEvalForVehicle(Input, SourceVehicle, TspRoute);
        StoredGain = SourceGain;
        GainComputed = true;
    }

    public override bool IsValid()
    {
        if (!GainComputed)
            ComputeGain();
            
        bool valid = IsValidForSourceRangeBounds();
        
        if (valid)
        {
            var route = new RawRoute(Input, SourceVehicle);
            
            valid = route.IsValidAdditionForCapacityInclusion(
                _sDelivery,
                TspRoute.ToArray(),
                0,
                TspRoute.Count,
                0,
                0);
        }
        
        return valid;
    }

    public override void Apply()
    {
        // Replace source route with optimized TSP route
        SourceRoute.Clear();
        SourceRoute.AddRange(TspRoute);
        
        Source.UpdateAmounts();
    }

    public override List<int> AdditionCandidates()
    {
        return new List<int> { SourceVehicle };
    }

    public override List<int> UpdateCandidates()
    {
        return new List<int> { SourceVehicle };
    }
}