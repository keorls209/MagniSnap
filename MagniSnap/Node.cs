using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace intelligent_scissors
{
    public class Node
    {

        public int x, y;
        public Node(int X, int Y)
        {
            x = X;
            y = Y;

        }

        // Required for Dictionary, HashSet, PriorityQueue
        // To avoid duplicates
        //If we don’t define Equals:
        //Node(3,5) != Node(3,5) 
        public override bool Equals(object obj)
        {
            if (obj is Node other)
                return this.x == other.x && this.y == other.y;

            return false;
        }

        // Required for Dictionary keys for same nodes

        // HashCode like an address for each object in dictionary.
        //Since, Equals() says two same nodes are equal,
        //then their hash must be same.
        public override int GetHashCode()
        {
            return (x * 397) ^ y;
        }

    }
}