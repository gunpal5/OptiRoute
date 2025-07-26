using System.Collections.Generic;

namespace OptiRoute.Core.Algorithms;

/// <summary>
/// Implements the Hungarian (Munkres) algorithm for minimum weight perfect matching.
/// Based on VROOM's munkres implementation.
/// </summary>
internal static class Munkres
{
    /// <summary>
    /// Finds minimum weight perfect matching using the Hungarian algorithm.
    /// </summary>
    public static Dictionary<int, int> MinimumWeightPerfectMatching(double[,] matrix)
    {
        var n = matrix.GetLength(0);
        
        // Trivial initial labeling
        var labelingX = new double[n];
        var labelingY = new double[n];
        
        for (int i = 0; i < n; i++)
        {
            double minWeight = double.MaxValue;
            for (int j = 0; j < n; j++)
            {
                if (matrix[i, j] < minWeight)
                {
                    minWeight = matrix[i, j];
                }
            }
            labelingX[i] = minWeight;
        }
        
        // Initial empty matching
        var matchingXY = new Dictionary<int, int>();
        var matchingYX = new Dictionary<int, int>();
        
        // Alternating tree
        var alternatingTree = new Dictionary<int, int>();
        
        while (matchingXY.Count < n)
        {
            // Step 1: Initialize
            alternatingTree.Clear();
            var sList = new List<int>();
            var sSet = new HashSet<int>();
            var tSet = new HashSet<int>();
            
            // Find any unmatched x
            int unmatchedX = 0;
            while (matchingXY.ContainsKey(unmatchedX))
            {
                unmatchedX++;
            }
            sSet.Add(unmatchedX);
            sList.Add(unmatchedX);
            
            // Initialize slacks and alternating tree
            var slack = new double[n];
            for (int y = 0; y < n; y++)
            {
                if (Math.Abs(labelingX[unmatchedX] + labelingY[y] - matrix[unmatchedX, y]) < 1e-9)
                {
                    alternatingTree[y] = unmatchedX;
                }
                slack[y] = matrix[unmatchedX, y] - labelingX[unmatchedX] - labelingY[y];
            }
            
            bool augmentedPath = false;
            
            while (!augmentedPath)
            {
                // Check if neighbors of S in equality graph equals T_set
                if (alternatingTree.Count == tSet.Count)
                {
                    // Step 2: Update labelings
                    double alpha = double.MaxValue;
                    
                    // Find minimum slack for y not in T
                    for (int y = 0; y < n; y++)
                    {
                        if (!tSet.Contains(y))
                        {
                            if (slack[y] < alpha)
                            {
                                alpha = slack[y];
                            }
                        }
                    }
                    
                    // Update labelings
                    foreach (var x in sList)
                    {
                        labelingX[x] += alpha;
                    }
                    foreach (var y in tSet)
                    {
                        labelingY[y] -= alpha;
                    }
                    
                    // Update alternating tree and slacks
                    for (int y = 0; y < n; y++)
                    {
                        if (!tSet.Contains(y))
                        {
                            slack[y] -= alpha;
                            
                            if (!alternatingTree.ContainsKey(y))
                            {
                                foreach (var x in sList)
                                {
                                    if (Math.Abs(labelingX[x] + labelingY[y] - matrix[x, y]) < 1e-9)
                                    {
                                        alternatingTree[y] = x;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                
                // Step 3: Find y in equality neighbors not in T_set
                int chosenY = -1;
                foreach (var kvp in alternatingTree)
                {
                    if (!tSet.Contains(kvp.Key))
                    {
                        chosenY = kvp.Key;
                        break;
                    }
                }
                
                if (matchingYX.ContainsKey(chosenY))
                {
                    // Chosen y is matched, update S and T_set
                    int matchedX = matchingYX[chosenY];
                    
                    if (sSet.Add(matchedX))
                    {
                        sList.Add(matchedX);
                    }
                    tSet.Add(chosenY);
                    
                    // Update slacks
                    for (int y = 0; y < n; y++)
                    {
                        double currentValue = slack[y];
                        double newValue = matrix[matchedX, y] - labelingX[matchedX] - labelingY[y];
                        if (newValue < currentValue)
                        {
                            slack[y] = newValue;
                        }
                    }
                }
                else
                {
                    // Found augmenting path
                    int currentY = chosenY;
                    int currentX = alternatingTree[currentY];
                    
                    while (currentX != unmatchedX)
                    {
                        int nextY = matchingXY[currentX];
                        
                        // Remove alternating edge
                        matchingXY.Remove(currentX);
                        matchingYX.Remove(nextY);
                        
                        // Add edge from alternating tree
                        matchingXY[currentX] = currentY;
                        matchingYX[currentY] = currentX;
                        
                        currentY = nextY;
                        currentX = alternatingTree[currentY];
                    }
                    
                    // Add last edge
                    matchingXY[currentX] = currentY;
                    matchingYX[currentY] = currentX;
                    
                    augmentedPath = true;
                }
            }
        }
        
        return matchingXY;
    }
    
    /// <summary>
    /// Greedy symmetric approximation for minimum weight perfect matching.
    /// </summary>
    public static Dictionary<int, int> GreedySymmetricApproxMWPM(double[,] matrix)
    {
        var n = matrix.GetLength(0);
        if (n % 2 != 0)
            throw new ArgumentException("Matrix size must be even for perfect matching");
        
        var matching = new Dictionary<int, int>();
        var remaining = new HashSet<int>();
        for (int i = 0; i < n; i++)
        {
            remaining.Add(i);
        }
        
        while (remaining.Count > 0)
        {
            double minWeight = double.MaxValue;
            int chosenI = -1, chosenJ = -1;
            
            foreach (var i in remaining)
            {
                foreach (var j in remaining)
                {
                    if (i != j && matrix[i, j] < minWeight)
                    {
                        minWeight = matrix[i, j];
                        chosenI = i;
                        chosenJ = j;
                    }
                }
            }
            
            matching[chosenI] = chosenJ;
            remaining.Remove(chosenI);
            remaining.Remove(chosenJ);
        }
        
        return matching;
    }
}