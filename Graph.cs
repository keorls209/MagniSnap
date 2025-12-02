using System;
using System.Collections.Generic;
using System.Linq;

namespace MagniSnap
{
    /// <summary>
    /// Represents an undirected weighted graph constructed from an image.
    /// Each pixel in the image becomes a node, and edges connect neighboring pixels.
    /// </summary>
    public class Graph
    {
        #region Enums

        /// <summary>
        /// Defines the connectivity pattern for neighboring pixels
        /// </summary>
        public enum ConnectivityType
        {
            /// <summary>
            /// 4-connectivity: Connect to top, bottom, left, right neighbors
            /// </summary>
            FourConnected,

            /// <summary>
            /// 8-connectivity: Connect to all 8 surrounding neighbors (includes diagonals)
            /// </summary>
            EightConnected
        }

        /// <summary>
        /// Defines how edge weights are calculated
        /// </summary>
        public enum WeightCalculationMethod
        {
            /// <summary>
            /// Weight = Absolute difference in grayscale intensity
            /// </summary>
            GrayscaleDifference,

            /// <summary>
            /// Weight = Euclidean distance in RGB color space
            /// </summary>
            RGBEuclidean,

            /// <summary>
            /// Weight = Gaussian similarity based on color difference
            /// </summary>
            GaussianSimilarity,

            /// <summary>
            /// Weight = 1 (uniform weight for all edges)
            /// </summary>
            Uniform
        }

        #endregion

        #region Properties

        /// <summary>
        /// 2D array of nodes corresponding to image pixels
        /// nodes[y, x] represents the node at pixel (x, y)
        /// </summary>
        public Node[,] Nodes { get; private set; }

        /// <summary>
        /// Image width in pixels
        /// </summary>
        public int Width { get; private set; }

        /// <summary>
        /// Image height in pixels
        /// </summary>
        public int Height { get; private set; }

        /// <summary>
        /// Total number of nodes in the graph
        /// </summary>
        public int NodeCount { get; private set; }

        /// <summary>
        /// Total number of edges in the graph
        /// </summary>
        public int EdgeCount { get; private set; }

        /// <summary>
        /// Connectivity type used when building the graph
        /// </summary>
        public ConnectivityType Connectivity { get; private set; }

        /// <summary>
        /// Weight calculation method used
        /// </summary>
        public WeightCalculationMethod WeightMethod { get; private set; }

        /// <summary>
        /// Sigma parameter for Gaussian similarity (if applicable)
        /// </summary>
        public double GaussianSigma { get; private set; }

        #endregion

        #region Constructors

        /// <summary>
        /// Default constructor
        /// </summary>
        public Graph()
        {
            GaussianSigma = 10.0; // Default sigma value
        }

        /// <summary>
        /// Constructor that builds graph from image matrix
        /// </summary>
        /// <param name="imageMatrix">2D array of RGB pixels</param>
        /// <param name="connectivity">Connectivity type (4 or 8)</param>
        /// <param name="weightMethod">Method to calculate edge weights</param>
        /// <param name="sigma">Sigma for Gaussian similarity (default: 10.0)</param>
        public Graph(RGBPixel[,] imageMatrix, 
                     ConnectivityType connectivity = ConnectivityType.FourConnected,
                     WeightCalculationMethod weightMethod = WeightCalculationMethod.GrayscaleDifference,
                     double sigma = 10.0)
        {
            GaussianSigma = sigma;
            BuildGraphFromImage(imageMatrix, connectivity, weightMethod);
        }

        #endregion

        #region Graph Construction Methods

        /// <summary>
        /// Build the graph from an image matrix
        /// </summary>
        /// <param name="imageMatrix">2D array of RGB pixels</param>
        /// <param name="connectivity">Connectivity type (4 or 8)</param>
        /// <param name="weightMethod">Method to calculate edge weights</param>
        public void BuildGraphFromImage(RGBPixel[,] imageMatrix, 
                                        ConnectivityType connectivity = ConnectivityType.FourConnected,
                                        WeightCalculationMethod weightMethod = WeightCalculationMethod.GrayscaleDifference)
        {
            Height = imageMatrix.GetLength(0);
            Width = imageMatrix.GetLength(1);
            NodeCount = Width * Height;
            Connectivity = connectivity;
            WeightMethod = weightMethod;
            EdgeCount = 0;

            // Step 1: Create all nodes
            CreateNodes(imageMatrix);

            // Step 2: Create edges between neighboring nodes
            CreateEdges();
        }

        /// <summary>
        /// Create nodes for all pixels in the image
        /// </summary>
        /// <param name="imageMatrix">2D array of RGB pixels</param>
        private void CreateNodes(RGBPixel[,] imageMatrix)
        {
            Nodes = new Node[Height, Width];

            for (int y = 0; y < Height; y++)
            {
                for (int x = 0; x < Width; x++)
                {
                    int id = y * Width + x;
                    Nodes[y, x] = new Node(id, x, y, imageMatrix[y, x]);
                }
            }
        }

        /// <summary>
        /// Create edges between neighboring nodes based on connectivity type
        /// </summary>
        private void CreateEdges()
        {
            for (int y = 0; y < Height; y++)
            {
                for (int x = 0; x < Width; x++)
                {
                    Node currentNode = Nodes[y, x];
                    List<Node> neighbors = GetNeighbors(x, y);

                    foreach (Node neighbor in neighbors)
                    {
                        double weight = CalculateEdgeWeight(currentNode, neighbor);
                        currentNode.AddEdge(neighbor, weight);
                        EdgeCount++;
                    }
                }
            }

            // Since it's an undirected graph, each edge is counted twice
            EdgeCount /= 2;
        }

        /// <summary>
        /// Get valid neighbors for a pixel based on connectivity type
        /// </summary>
        /// <param name="x">X-coordinate</param>
        /// <param name="y">Y-coordinate</param>
        /// <returns>List of neighbor nodes</returns>
        private List<Node> GetNeighbors(int x, int y)
        {
            List<Node> neighbors = new List<Node>();

            if (Connectivity == ConnectivityType.FourConnected)
            {
                // 4-connectivity: top, right, bottom, left
                int[,] offsets = { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 } };

                for (int i = 0; i < 4; i++)
                {
                    int nx = x + offsets[i, 0];
                    int ny = y + offsets[i, 1];

                    if (IsValidCoordinate(nx, ny))
                    {
                        neighbors.Add(Nodes[ny, nx]);
                    }
                }
            }
            else // EightConnected
            {
                // 8-connectivity: all surrounding neighbors
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        if (dx == 0 && dy == 0) continue; // Skip self

                        int nx = x + dx;
                        int ny = y + dy;

                        if (IsValidCoordinate(nx, ny))
                        {
                            neighbors.Add(Nodes[ny, nx]);
                        }
                    }
                }
            }

            return neighbors;
        }

        /// <summary>
        /// Check if coordinates are within image bounds
        /// </summary>
        /// <param name="x">X-coordinate</param>
        /// <param name="y">Y-coordinate</param>
        /// <returns>True if valid, false otherwise</returns>
        private bool IsValidCoordinate(int x, int y)
        {
            return x >= 0 && x < Width && y >= 0 && y < Height;
        }

        /// <summary>
        /// Calculate edge weight between two nodes based on the selected method
        /// </summary>
        /// <param name="node1">First node</param>
        /// <param name="node2">Second node</param>
        /// <returns>Edge weight</returns>
        private double CalculateEdgeWeight(Node node1, Node node2)
        {
            switch (WeightMethod)
            {
                case WeightCalculationMethod.GrayscaleDifference:
                    return node1.GrayDifference(node2);

                case WeightCalculationMethod.RGBEuclidean:
                    return node1.ColorDifference(node2);

                case WeightCalculationMethod.GaussianSimilarity:
                    double colorDiff = node1.ColorDifference(node2);
                    return Math.Exp(-(colorDiff * colorDiff) / (2 * GaussianSigma * GaussianSigma));

                case WeightCalculationMethod.Uniform:
                    return 1.0;

                default:
                    return node1.GrayDifference(node2);
            }
        }

        #endregion

        #region Graph Access Methods

        /// <summary>
        /// Get node at specific pixel coordinates
        /// </summary>
        /// <param name="x">X-coordinate</param>
        /// <param name="y">Y-coordinate</param>
        /// <returns>Node at the specified position</returns>
        public Node GetNode(int x, int y)
        {
            if (!IsValidCoordinate(x, y))
                throw new ArgumentOutOfRangeException("Coordinates out of bounds");

            return Nodes[y, x];
        }

        /// <summary>
        /// Get node by ID
        /// </summary>
        /// <param name="id">Node ID</param>
        /// <returns>Node with the specified ID</returns>
        public Node GetNodeByID(int id)
        {
            int y = id / Width;
            int x = id % Width;
            return Nodes[y, x];
        }

        /// <summary>
        /// Get all nodes in the graph as a flat list
        /// </summary>
        /// <returns>List of all nodes</returns>
        public List<Node> GetAllNodes()
        {
            List<Node> allNodes = new List<Node>(NodeCount);

            for (int y = 0; y < Height; y++)
            {
                for (int x = 0; x < Width; x++)
                {
                    allNodes.Add(Nodes[y, x]);
                }
            }

            return allNodes;
        }

        #endregion

        #region Graph Utility Methods

        /// <summary>
        /// Reset all visited flags to false
        /// </summary>
        public void ResetVisitedFlags()
        {
            for (int y = 0; y < Height; y++)
            {
                for (int x = 0; x < Width; x++)
                {
                    Nodes[y, x].Visited = false;
                }
            }
        }

        /// <summary>
        /// Get graph statistics
        /// </summary>
        /// <returns>String with graph information</returns>
        public string GetGraphInfo()
        {
            return $"Graph Info:\n" +
                   $"  Dimensions: {Width}x{Height}\n" +
                   $"  Nodes: {NodeCount}\n" +
                   $"  Edges: {EdgeCount}\n" +
                   $"  Connectivity: {Connectivity}\n" +
                   $"  Weight Method: {WeightMethod}\n" +
                   $"  Avg Degree: {(EdgeCount * 2.0 / NodeCount):F2}";
        }

        /// <summary>
        /// Print graph information to console
        /// </summary>
        public void PrintGraphInfo()
        {
            Console.WriteLine(GetGraphInfo());
        }

        /// <summary>
        /// Get the minimum edge weight in the graph
        /// </summary>
        /// <returns>Minimum edge weight</returns>
        public double GetMinEdgeWeight()
        {
            double min = double.MaxValue;

            foreach (Node node in GetAllNodes())
            {
                foreach (var edge in node.Edges)
                {
                    if (edge.Value < min)
                        min = edge.Value;
                }
            }

            return min;
        }

        /// <summary>
        /// Get the maximum edge weight in the graph
        /// </summary>
        /// <returns>Maximum edge weight</returns>
        public double GetMaxEdgeWeight()
        {
            double max = double.MinValue;

            foreach (Node node in GetAllNodes())
            {
                foreach (var edge in node.Edges)
                {
                    if (edge.Value > max)
                        max = edge.Value;
                }
            }

            return max;
        }

        /// <summary>
        /// Get the average edge weight in the graph
        /// </summary>
        /// <returns>Average edge weight</returns>
        public double GetAverageEdgeWeight()
        {
            double sum = 0;
            int count = 0;

            foreach (Node node in GetAllNodes())
            {
                foreach (var edge in node.Edges)
                {
                    sum += edge.Value;
                    count++;
                }
            }

            return count > 0 ? sum / count : 0;
        }

        #endregion

        #region Graph Traversal Methods (Placeholder for future use)

        /// <summary>
        /// Perform Breadth-First Search starting from a specific node
        /// </summary>
        /// <param name="startX">Start X-coordinate</param>
        /// <param name="startY">Start Y-coordinate</param>
        /// <returns>List of nodes in BFS order</returns>
        public List<Node> BFS(int startX, int startY)
        {
            List<Node> result = new List<Node>();
            Queue<Node> queue = new Queue<Node>();
            ResetVisitedFlags();

            Node startNode = GetNode(startX, startY);
            queue.Enqueue(startNode);
            startNode.Visited = true;

            while (queue.Count > 0)
            {
                Node current = queue.Dequeue();
                result.Add(current);

                foreach (Node neighbor in current.GetNeighbors())
                {
                    if (!neighbor.Visited)
                    {
                        neighbor.Visited = true;
                        queue.Enqueue(neighbor);
                    }
                }
            }

            return result;
        }

        /// <summary>
        /// Perform Depth-First Search starting from a specific node
        /// </summary>
        /// <param name="startX">Start X-coordinate</param>
        /// <param name="startY">Start Y-coordinate</param>
        /// <returns>List of nodes in DFS order</returns>
        public List<Node> DFS(int startX, int startY)
        {
            List<Node> result = new List<Node>();
            ResetVisitedFlags();
            Node startNode = GetNode(startX, startY);
            DFSRecursive(startNode, result);
            return result;
        }

        /// <summary>
        /// Recursive helper for DFS
        /// </summary>
        private void DFSRecursive(Node node, List<Node> result)
        {
            node.Visited = true;
            result.Add(node);

            foreach (Node neighbor in node.GetNeighbors())
            {
                if (!neighbor.Visited)
                {
                    DFSRecursive(neighbor, result);
                }
            }
        }

        #endregion
    }
}
