namespace OptiRoute.Core.Models;

/// <summary>
/// Represents amounts for multiple dimensions (inspired by VROOM's Amount class).
/// Used for capacity constraints in multiple dimensions (weight, volume, etc.).
/// </summary>
public class Amount : IComparable<Amount>
{
    private readonly long[] _values;

    public int Size => _values.Length;
    
    public long this[int index]
    {
        get => _values[index];
        set => _values[index] = value;
    }

    public Amount(int size)
    {
        _values = new long[size];
    }

    public Amount(params long[] values)
    {
        _values = values.ToArray();
    }

    public Amount(Amount other)
    {
        _values = other._values.ToArray();
    }

    public static Amount operator +(Amount a, Amount b)
    {
        if (a.Size != b.Size)
            throw new ArgumentException("Amounts must have the same size");
            
        var result = new Amount(a.Size);
        for (int i = 0; i < a.Size; i++)
        {
            result[i] = a[i] + b[i];
        }
        return result;
    }

    public static Amount operator -(Amount a, Amount b)
    {
        if (a.Size != b.Size)
            throw new ArgumentException("Amounts must have the same size");
            
        var result = new Amount(a.Size);
        for (int i = 0; i < a.Size; i++)
        {
            result[i] = a[i] - b[i];
        }
        return result;
    }

    public static bool operator <=(Amount a, Amount b)
    {
        if (a.Size != b.Size)
            throw new ArgumentException("Amounts must have the same size");
            
        for (int i = 0; i < a.Size; i++)
        {
            if (a[i] > b[i]) return false;
        }
        return true;
    }

    public static bool operator >=(Amount a, Amount b) => b <= a;

    public static bool operator <(Amount a, Amount b)
    {
        // Lexicographical comparison like VROOM
        if (a.Size != b.Size)
            throw new ArgumentException("Amounts must have the same size");
        
        if (a.Size == 0)
            return false;
            
        for (int i = 0; i < a.Size - 1; i++)
        {
            if (a[i] < b[i]) return true;
            if (a[i] > b[i]) return false;
        }
        
        return a[a.Size - 1] < b[a.Size - 1];
    }

    public static bool operator >(Amount a, Amount b) => b < a;

    public static bool operator ==(Amount? a, Amount? b)
    {
        if (ReferenceEquals(a, b)) return true;
        if (a is null || b is null) return false;
        if (a.Size != b.Size) return false;
        
        for (int i = 0; i < a.Size; i++)
        {
            if (a[i] != b[i]) return false;
        }
        return true;
    }

    public static bool operator !=(Amount? a, Amount? b) => !(a == b);

    public int CompareTo(Amount? other)
    {
        if (other is null) return 1;
        if (Size != other.Size)
            throw new ArgumentException("Cannot compare amounts of different sizes");
        
        // Lexicographical comparison    
        for (int i = 0; i < Size; i++)
        {
            var comparison = _values[i].CompareTo(other._values[i]);
            if (comparison != 0) return comparison;
        }
        return 0;
    }

    public override bool Equals(object? obj) => obj is Amount other && this == other;
    
    public override int GetHashCode()
    {
        var hash = new HashCode();
        foreach (var value in _values)
        {
            hash.Add(value);
        }
        return hash.ToHashCode();
    }

    public override string ToString() => $"[{string.Join(", ", _values)}]";
}