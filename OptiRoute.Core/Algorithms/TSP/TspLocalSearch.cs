using System.Collections.Concurrent;

namespace OptiRoute.Core.Algorithms.TSP;

/// <summary>
/// Local search improvements for TSP (based on VROOM's TSP LocalSearch).
/// </summary>
internal class TspLocalSearch
{
    private readonly double[,] _matrix;
    private readonly bool _avoidStartRelocate;
    private readonly int _avoidStartIndex;
    private readonly int[] _edges; // Next node in tour for each node
    private readonly int _nbThreads;
    private readonly int[] _rankLimits;
    private readonly int[] _symTwoOptRankLimits;

    public TspLocalSearch(
        double[,] matrix,
        (bool avoid, int index) avoidStartRelocate,
        List<int> tour,
        int nbThreads = 1)
    {
        _matrix = matrix;
        _avoidStartRelocate = avoidStartRelocate.avoid;
        _avoidStartIndex = avoidStartRelocate.index;
        _nbThreads = Math.Min(nbThreads, tour.Count);
        
        // Build edges vector representation
        _edges = new int[matrix.GetLength(0)];
        BuildEdges(tour);
        
        // Build rank limits for parallel processing
        _rankLimits = BuildRankLimits(_edges.Length, _nbThreads);
        _symTwoOptRankLimits = BuildSymTwoOptRankLimits(_edges.Length, _nbThreads);
    }

    private void BuildEdges(List<int> tour)
    {
        if (tour.Count == 0) return;
        
        for (int i = 0; i < tour.Count - 1; i++)
        {
            _edges[tour[i]] = tour[i + 1];
        }
        _edges[tour[^1]] = tour[0]; // Close the tour
    }

    private int[] BuildRankLimits(int size, int nbThreads)
    {
        var limits = new int[nbThreads + 1];
        int rangeWidth = size / nbThreads;
        
        for (int i = 0; i < nbThreads; i++)
        {
            limits[i] = i * rangeWidth;
        }
        
        // Distribute remainder
        int remainder = size % nbThreads;
        int shift = 0;
        for (int i = 1; i < nbThreads; i++)
        {
            if (shift < remainder) shift++;
            limits[i] += shift;
        }
        limits[nbThreads] = size;
        
        return limits;
    }

    private int[] BuildSymTwoOptRankLimits(int size, int nbThreads)
    {
        var limits = new List<int> { 0 };
        
        if (nbThreads > 1 && size > 3)
        {
            // Calculate workload distribution for symmetric 2-opt
            var numberOfLookups = new int[size - 1];
            numberOfLookups[0] = size - 3;
            
            for (int i = 1; i < numberOfLookups.Length; i++)
            {
                numberOfLookups[i] = numberOfLookups[i - 1] - 1;
            }
            
            var cumulatedLookups = new int[numberOfLookups.Length];
            cumulatedLookups[0] = numberOfLookups[0];
            for (int i = 1; i < numberOfLookups.Length; i++)
            {
                cumulatedLookups[i] = cumulatedLookups[i - 1] + numberOfLookups[i];
            }
            
            int totalLookups = size * (size - 3) / 2;
            int threadLookupShare = totalLookups / nbThreads;
            
            int rank = 0;
            for (int i = 1; i < nbThreads; i++)
            {
                while (rank < cumulatedLookups.Length && 
                       cumulatedLookups[rank] < i * threadLookupShare)
                {
                    rank++;
                }
                limits.Add(Math.Min(rank + 1, size));
            }
        }
        
        limits.Add(size);
        return limits.ToArray();
    }

    /// <summary>
    /// Performs all relocate improvements until no more gains.
    /// </summary>
    public double PerformAllRelocateSteps(DateTime? deadline)
    {
        double totalGain = 0;
        double gain;
        
        do
        {
            gain = RelocateStep();
            totalGain += gain;
            
            if (deadline.HasValue && DateTime.UtcNow >= deadline.Value)
                break;
        } while (gain > 0);
        
        return totalGain;
    }

    /// <summary>
    /// Single relocate step - moves one node to a different position.
    /// </summary>
    private double RelocateStep()
    {
        if (_edges.Length < 3) return 0;
        
        double bestGain = 0;
        int bestEdge1Start = -1;
        int bestEdge2Start = -1;
        
        // Parallel search for best relocate move
        if (_nbThreads == 1)
        {
            LookUpRelocate(0, _edges.Length, ref bestGain, ref bestEdge1Start, ref bestEdge2Start);
        }
        else
        {
            var tasks = new Task[_nbThreads];
            var results = new (double gain, int edge1, int edge2)[_nbThreads];
            
            for (int t = 0; t < _nbThreads; t++)
            {
                int threadId = t;
                tasks[t] = Task.Run(() =>
                {
                    double localBestGain = 0;
                    int localEdge1 = -1, localEdge2 = -1;
                    LookUpRelocate(_rankLimits[threadId], _rankLimits[threadId + 1],
                                  ref localBestGain, ref localEdge1, ref localEdge2);
                    results[threadId] = (localBestGain, localEdge1, localEdge2);
                });
            }
            
            Task.WaitAll(tasks);
            
            // Find best among all threads
            foreach (var result in results)
            {
                if (result.gain > bestGain)
                {
                    bestGain = result.gain;
                    bestEdge1Start = result.edge1;
                    bestEdge2Start = result.edge2;
                }
            }
        }
        
        // Apply best move if found
        if (bestGain > 0)
        {
            int edge1End = _edges[bestEdge1Start];
            int next = _edges[edge1End];
            
            _edges[bestEdge1Start] = next;
            _edges[edge1End] = _edges[bestEdge2Start];
            _edges[bestEdge2Start] = edge1End;
        }
        
        return bestGain;
    }

    private void LookUpRelocate(int start, int end, ref double bestGain, 
                                ref int bestEdge1Start, ref int bestEdge2Start)
    {
        for (int edge1Start = start; edge1Start < end; edge1Start++)
        {
            if (_avoidStartRelocate && edge1Start == _avoidStartIndex)
                continue;
            
            int edge1End = _edges[edge1Start];
            int next = _edges[edge1End];
            
            // Precompute weights
            double firstPotentialAdd = _matrix[edge1Start, next];
            double edge1Weight = _matrix[edge1Start, edge1End];
            double edge1EndNextWeight = _matrix[edge1End, next];
            
            if (edge1Weight + edge1EndNextWeight - firstPotentialAdd < bestGain)
                continue;
            
            int edge2Start = next;
            while (edge2Start != edge1Start)
            {
                int edge2End = _edges[edge2Start];
                double beforeCost = edge1Weight + edge1EndNextWeight + 
                                   _matrix[edge2Start, edge2End];
                
                double afterCost = firstPotentialAdd + 
                                  _matrix[edge2Start, edge1End] + 
                                  _matrix[edge1End, edge2End];
                
                double gain = beforeCost - afterCost;
                if (gain > bestGain)
                {
                    bestGain = gain;
                    bestEdge1Start = edge1Start;
                    bestEdge2Start = edge2Start;
                }
                
                edge2Start = edge2End;
            }
        }
    }

    /// <summary>
    /// Performs all 2-opt improvements until no more gains.
    /// </summary>
    public double PerformAllTwoOptSteps(DateTime? deadline)
    {
        double totalGain = 0;
        double gain;
        
        do
        {
            gain = TwoOptStep();
            totalGain += gain;
            
            if (deadline.HasValue && DateTime.UtcNow >= deadline.Value)
                break;
        } while (gain > 0);
        
        return totalGain;
    }

    /// <summary>
    /// Single 2-opt step - reverses a segment of the tour.
    /// </summary>
    private double TwoOptStep()
    {
        double bestGain = 0;
        int bestI = -1;
        int bestJ = -1;
        
        // For symmetric TSP
        for (int i = 0; i < _edges.Length - 2; i++)
        {
            int iNext = _edges[i];
            
            for (int j = i + 2; j < _edges.Length; j++)
            {
                if (j == iNext) continue; // Adjacent edges
                
                int jNext = _edges[j];
                if (jNext == i) continue; // Would create a subtour
                
                double currentCost = _matrix[i, iNext] + _matrix[j, jNext];
                double newCost = _matrix[i, j] + _matrix[iNext, jNext];
                double gain = currentCost - newCost;
                
                if (gain > bestGain)
                {
                    bestGain = gain;
                    bestI = i;
                    bestJ = j;
                }
            }
        }
        
        // Apply best 2-opt move
        if (bestGain > 0)
        {
            ReverseTourSegment(bestI, bestJ);
        }
        
        return bestGain;
    }

    private void ReverseTourSegment(int i, int j)
    {
        // Reverse edges between i and j
        var toReverse = new List<(int from, int to)>();
        int current = _edges[i];
        
        while (current != j)
        {
            int next = _edges[current];
            toReverse.Add((current, next));
            current = next;
        }
        toReverse.Add((j, _edges[j]));
        
        // Apply reversal
        _edges[i] = j;
        foreach (var (from, to) in toReverse)
        {
            _edges[to] = from;
        }
    }

    /// <summary>
    /// Performs all Or-opt improvements (moving sequences of 1, 2, or 3 nodes).
    /// </summary>
    public double PerformAllOrOptSteps(DateTime? deadline)
    {
        double totalGain = 0;
        
        // Try different sequence lengths (1, 2, 3)
        for (int length = 1; length <= 3; length++)
        {
            double gain;
            do
            {
                gain = OrOptStep(length);
                totalGain += gain;
                
                if (deadline.HasValue && DateTime.UtcNow >= deadline.Value)
                    return totalGain;
            } while (gain > 0);
        }
        
        return totalGain;
    }

    private double OrOptStep(int length)
    {
        if (_edges.Length < length + 2) return 0;
        
        double bestGain = 0;
        int bestStart = -1;
        int bestInsertAfter = -1;
        
        // Try removing sequence starting at each position
        for (int start = 0; start < _edges.Length; start++)
        {
            // Build sequence
            var sequence = new int[length];
            int current = start;
            bool validSequence = true;
            
            for (int i = 0; i < length; i++)
            {
                sequence[i] = current;
                current = _edges[current];
                if (current == start) // Avoid wrapping
                {
                    validSequence = false;
                    break;
                }
            }
            
            if (!validSequence) continue;
            
            int seqEnd = sequence[^1];
            int afterSeq = _edges[seqEnd];
            int beforeSeq = GetPredecessor(start);
            
            // Cost of removing sequence
            double removeCost = _matrix[beforeSeq, start] + _matrix[seqEnd, afterSeq] -
                               _matrix[beforeSeq, afterSeq];
            
            // Try inserting sequence elsewhere
            int insertPoint = afterSeq;
            while (insertPoint != beforeSeq)
            {
                int afterInsert = _edges[insertPoint];
                
                // Cost of inserting sequence
                double insertCost = _matrix[insertPoint, start] + _matrix[seqEnd, afterInsert] -
                                   _matrix[insertPoint, afterInsert];
                
                double gain = removeCost - insertCost;
                if (gain > bestGain)
                {
                    bestGain = gain;
                    bestStart = start;
                    bestInsertAfter = insertPoint;
                }
                
                insertPoint = afterInsert;
            }
        }
        
        // Apply best Or-opt move
        if (bestGain > 0)
        {
            ApplyOrOpt(bestStart, length, bestInsertAfter);
        }
        
        return bestGain;
    }

    private int GetPredecessor(int node)
    {
        for (int i = 0; i < _edges.Length; i++)
        {
            if (_edges[i] == node) return i;
        }
        return -1;
    }

    private void ApplyOrOpt(int start, int length, int insertAfter)
    {
        // Extract sequence
        var sequence = new int[length];
        int current = start;
        for (int i = 0; i < length; i++)
        {
            sequence[i] = current;
            current = _edges[current];
        }
        
        int seqEnd = sequence[^1];
        int afterSeq = _edges[seqEnd];
        int beforeSeq = GetPredecessor(start);
        
        // Remove sequence
        _edges[beforeSeq] = afterSeq;
        
        // Insert sequence after insertAfter
        int afterInsert = _edges[insertAfter];
        _edges[insertAfter] = start;
        _edges[seqEnd] = afterInsert;
    }

    /// <summary>
    /// Gets the tour starting from a specific index.
    /// </summary>
    public List<int> GetTour(int firstIndex)
    {
        var tour = new List<int>();
        int current = firstIndex;
        
        do
        {
            tour.Add(current);
            current = _edges[current];
        } while (current != firstIndex && tour.Count < _edges.Length);
        
        return tour;
    }
}