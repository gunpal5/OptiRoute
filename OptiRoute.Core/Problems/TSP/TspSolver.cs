using OptiRoute.Core.Models;

namespace OptiRoute.Core.Problems.TSP;

/// <summary>
/// Solves the Traveling Salesman Problem (inspired by VROOM's TSP implementation).
/// </summary>
public class TspSolver
{
    private readonly Input _input;
    private readonly int _vehicleIndex;
    private readonly List<int> _jobRanks;
    private readonly bool _hasStart;
    private readonly bool _hasEnd;
    private int? _startLocation;
    private int? _endLocation;
    private readonly double[,] _matrix;
    private readonly double[,] _symmetrizedMatrix;
    private readonly bool _isSymmetric;
    private readonly bool _roundTrip;
    private int _threadCount = 1;

    public TspSolver(Input input, int vehicleIndex, List<int> jobRanks)
    {
        ArgumentNullException.ThrowIfNull(input);
        ArgumentNullException.ThrowIfNull(jobRanks);
        
        if (jobRanks.Count == 0)
            throw new ArgumentException("Job ranks cannot be empty", nameof(jobRanks));
        
        _input = input;
        _vehicleIndex = vehicleIndex;
        _jobRanks = jobRanks;
        
        var vehicle = input.Vehicles[vehicleIndex];
        _hasStart = vehicle.StartLocation != null;
        _hasEnd = vehicle.EndLocation != null;
        
        // Build TSP matrix
        var matrixRanks = BuildMatrixRanks();
        _matrix = BuildCostMatrix(matrixRanks);
        _symmetrizedMatrix = SymmetrizeMatrix(_matrix);
        _isSymmetric = CheckSymmetry(_matrix);
        _roundTrip = _hasStart && _hasEnd && _startLocation == _endLocation;
    }

    /// <summary>
    /// Solves the TSP and returns job indices in order.
    /// </summary>
    public List<int> Solve(int threadCount = 1, TimeSpan? timeout = null)
    {
        _threadCount = threadCount;
        var deadline = timeout.HasValue ? DateTime.UtcNow + timeout.Value : (DateTime?)null;
        
        // Apply Christofides heuristic
        var initialSolution = Christofides.Solve(_symmetrizedMatrix);
        
        // Apply local search improvements
        var improvedSolution = ApplyLocalSearch(initialSolution, _symmetrizedMatrix, deadline);
        
        // If asymmetric, try both directions and apply asymmetric local search
        if (!_isSymmetric)
        {
            var reverseSolution = new List<int>(improvedSolution);
            reverseSolution.Reverse();
            
            var directCost = ComputeCost(improvedSolution, _matrix);
            var reverseCost = ComputeCost(reverseSolution, _matrix);
            
            var bestSolution = directCost <= reverseCost ? improvedSolution : reverseSolution;
            improvedSolution = ApplyAsymmetricLocalSearch(bestSolution, _matrix, deadline);
        }
        
        // Convert back to job ranks
        return ConvertToJobRanks(improvedSolution);
    }

    private List<int> BuildMatrixRanks()
    {
        var matrixRanks = new List<int>();
        
        // Add job locations
        foreach (var jobRank in _jobRanks)
        {
            var job = _input.Jobs[jobRank];
            var locationIndex = Array.IndexOf(_input.Locations.ToArray(), job.Location);
            matrixRanks.Add(locationIndex);
        }
        
        // Add start location if present
        int? startIndex = null;
        if (_hasStart)
        {
            var vehicle = _input.Vehicles[_vehicleIndex];
            startIndex = Array.IndexOf(_input.Locations.ToArray(), vehicle.StartLocation!);
            _startLocation = matrixRanks.Count;
            matrixRanks.Add(startIndex.Value);
        }
        
        // Add end location if present and different from start
        if (_hasEnd)
        {
            var vehicle = _input.Vehicles[_vehicleIndex];
            var endIndex = Array.IndexOf(_input.Locations.ToArray(), vehicle.EndLocation!);
            
            if (_hasStart && startIndex.HasValue && startIndex.Value == endIndex)
            {
                _endLocation = _startLocation;
            }
            else
            {
                _endLocation = matrixRanks.Count;
                matrixRanks.Add(endIndex);
            }
        }
        
        return matrixRanks;
    }

    private double[,] BuildCostMatrix(List<int> matrixRanks)
    {
        var n = matrixRanks.Count;
        var matrix = new double[n, n];
        
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i == j)
                {
                    matrix[i, j] = double.PositiveInfinity;
                }
                else
                {
                    var eval = _input.GetEval(_vehicleIndex, matrixRanks[i], matrixRanks[j]);
                    matrix[i, j] = eval.Cost;
                }
            }
        }
        
        // Handle open tour cases
        if (!_roundTrip)
        {
            if (_hasStart && !_hasEnd)
            {
                // Force start location
                for (int i = 0; i < n; i++)
                {
                    if (i != _startLocation)
                        matrix[i, _startLocation!.Value] = 0;
                }
            }
            else if (!_hasStart && _hasEnd)
            {
                // Force end location
                for (int j = 0; j < n; j++)
                {
                    if (j != _endLocation)
                        matrix[_endLocation!.Value, j] = 0;
                }
            }
            else if (_hasStart && _hasEnd && _startLocation != _endLocation)
            {
                // Force start and end
                matrix[_endLocation!.Value, _startLocation!.Value] = 0;
                for (int j = 0; j < n; j++)
                {
                    if (j != _startLocation && j != _endLocation)
                        matrix[_endLocation.Value, j] = double.PositiveInfinity;
                }
            }
        }
        
        return matrix;
    }

    private double[,] SymmetrizeMatrix(double[,] matrix)
    {
        var n = matrix.GetLength(0);
        var symmetric = new double[n, n];
        
        for (int i = 0; i < n; i++)
        {
            symmetric[i, i] = matrix[i, i];
            for (int j = i + 1; j < n; j++)
            {
                var value = Math.Min(matrix[i, j], matrix[j, i]);
                symmetric[i, j] = value;
                symmetric[j, i] = value;
            }
        }
        
        return symmetric;
    }

    private bool CheckSymmetry(double[,] matrix)
    {
        var n = matrix.GetLength(0);
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                if (Math.Abs(matrix[i, j] - matrix[j, i]) > 1e-9)
                    return false;
            }
        }
        return true;
    }

    private double ComputeCost(List<int> tour, double[,] matrix)
    {
        if (tour.Count == 0) return 0;
        
        double cost = 0;
        for (int i = 0; i < tour.Count - 1; i++)
        {
            cost += matrix[tour[i], tour[i + 1]];
        }
        
        if (_roundTrip && tour.Count > 0)
        {
            cost += matrix[tour[^1], tour[0]];
        }
        
        return cost;
    }

    private List<int> ApplyLocalSearch(List<int> tour, double[,] matrix, DateTime? deadline)
    {
        // Use VROOM's TSP local search implementation
        var localSearch = new TspLocalSearch(
            matrix,
            (!_roundTrip && _hasStart && _hasEnd, _startLocation ?? 0),
            tour,
            _threadCount);
        
        double totalGain = 0;
        double gain;
        
        do
        {
            // All 2-opt moves
            gain = localSearch.PerformAllTwoOptSteps(deadline);
            totalGain += gain;
            
            // All relocate moves
            gain = localSearch.PerformAllRelocateSteps(deadline);
            totalGain += gain;
            
            // All Or-opt moves
            gain = localSearch.PerformAllOrOptSteps(deadline);
            totalGain += gain;
            
            // Check deadline
            if (deadline.HasValue && DateTime.UtcNow >= deadline.Value)
                break;
                
        } while (gain > 0);
        
        // Get the optimized tour
        int firstIndex = _hasStart ? _startLocation!.Value : 
                        (_hasEnd ? _endLocation!.Value : 0);
        return localSearch.GetTour(firstIndex);
    }

    private List<int> ApplyAsymmetricLocalSearch(List<int> tour, double[,] matrix, DateTime? deadline)
    {
        // For asymmetric TSP, VROOM uses additional avoid-loops operator
        // For now, reuse symmetric local search
        return ApplyLocalSearch(tour, matrix, deadline);
    }


    private List<int> ConvertToJobRanks(List<int> solution)
    {
        var result = new List<int>();
        
        // Find starting position based on start/end constraints
        int startPos = 0;
        if (_hasStart)
        {
            startPos = solution.IndexOf(_startLocation!.Value);
            if (startPos == -1) startPos = 0;
        }
        else if (_hasEnd)
        {
            startPos = solution.IndexOf(_endLocation!.Value);
            if (startPos == -1) startPos = 0;
        }
        
        // Reorder solution starting from the correct position
        var reordered = new List<int>();
        for (int i = 0; i < solution.Count; i++)
        {
            reordered.Add(solution[(startPos + i) % solution.Count]);
        }
        
        // Convert back to job ranks, skipping start/end locations
        foreach (var index in reordered)
        {
            if (index < _jobRanks.Count)
            {
                result.Add(_jobRanks[index]);
            }
        }
        
        return result;
    }
}