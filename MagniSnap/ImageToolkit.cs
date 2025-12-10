using MagniSnap;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Forms;
using System.Xml.Linq;
using Priority_Queue;
using System.IO;
namespace MagniSnap
{
    /// <summary>
    /// Holds the pixel color in 3 byte values: red, green and blue
    /// </summary>
    public struct RGBPixel
    {
        public byte red, green, blue;
    }

    public struct RGBPixelD
    {
        public double red, green, blue;
    }


    /// <summary>
    /// Holds the edge energy between 
    ///     1. a pixel and its right one (X)
    ///     2. a pixel and its bottom one (Y)
    /// </summary>
    public struct Vector2D
    {
        public double X { get; set; }
        public double Y { get; set; }
    }
    /// <summary>
    /// Library of static functions that deal with images
    /// </summary>
    public struct Node
    {
        public int x, y;
        public Node(int x, int y) { this.x = x; this.y = y; }
    }

    public class ImageToolkit
    {


        // Task 1 : Constructing graph
        // Task 1 : Constructing graph (subgraph version for large images)
        public static List<(Node, double)> GetNeighbors(Node n, RGBPixel[,] image)
        {
            int height = image.GetLength(0);
            int width = image.GetLength(1);
            List<(Node, double)> neighbors = new List<(Node, double)>();

            if (n.y < width - 1)
            {
                Node r = new Node(n.x, n.y + 1);
                neighbors.Add((r, 1.0 / CalculatePixelEnergies(r.y, r.x, image).X));
            }
            if (n.y > 0)
            {
                Node l = new Node(n.x, n.y - 1);
                neighbors.Add((l, 1.0 / CalculatePixelEnergies(l.y, l.x, image).X));
            }
            if (n.x < height - 1)
            {
                Node d = new Node(n.x + 1, n.y);
                neighbors.Add((d, 1.0 / CalculatePixelEnergies(d.y, d.x, image).Y));
            }
            if (n.x > 0)
            {
                Node u = new Node(n.x - 1, n.y);
                neighbors.Add((u, 1.0 / CalculatePixelEnergies(u.y, u.x, image).Y));
            }

            return neighbors;
        }

        //Task 2: find Shortest Path

        public static Dictionary<Node, Node> DijkstraSubgraph(RGBPixel[,] image, Node anchor, int radius)
        {
            SimplePriorityQueue<Node, double> pq = new SimplePriorityQueue<Node, double>();
            Dictionary<Node, Node> parents = new Dictionary<Node, Node>();
            Dictionary<Node, double> distances = new Dictionary<Node, double>();
            HashSet<Node> visited = new HashSet<Node>();

            pq.Enqueue(anchor, 0);
            distances[anchor] = 0;

            while (pq.Count > 0)
            {
                Node current = pq.Dequeue();
                if (visited.Contains(current)) continue;
                visited.Add(current);

                foreach (var (neighbor, weight) in GetNeighbors(current, image))
                {
                    // Only process pixels inside radius
                    if (Math.Abs(neighbor.x - anchor.x) > radius || Math.Abs(neighbor.y - anchor.y) > radius)
                        continue;

                    double newDist = distances[current] + weight;
                    if (!distances.ContainsKey(neighbor) || newDist < distances[neighbor])
                    {
                        distances[neighbor] = newDist;
                        parents[neighbor] = current;
                        pq.Enqueue(neighbor, newDist);
                    }
                }
            }

            return parents;
        }


        //smiley face

        //Task 3 : Backtracking shortest path
        public static List<Node> BacktrackShortestPath(Dictionary<Node, Node> parents, Node targetNode)
        {
            List<Node> reversePath = new List<Node>();

            if (parents == null)
                return reversePath;
            if (!parents.ContainsKey(targetNode))
                return reversePath;

            Node current = targetNode;
            while (parents.ContainsKey(current))
            {
                reversePath.Add(current);
                current = parents[current];
            }
            reversePath.Add(current); // add start node
            reversePath.Reverse();

            return reversePath;
        }


        //Task 4

        public static void DrawPath(RGBPixel[,] image, List<Node> path)
        {
            if (path == null) return;

            int height = image.GetLength(0);
            int width = image.GetLength(1);

            foreach (var node in path)
            {
                int row = node.x; // because you use new Node(i, j)
                int col = node.y;

                if (row >= 0 && row < height && col >= 0 && col < width)
                {
                    image[row, col].red = 255;
                    image[row, col].green = 0;
                    image[row, col].blue = 0;   // red path
                }
            }
        }



        ///
        /// <summary>
        /// Open an image and load it into 2D array of colors (size: Height x Width)
        /// </summary>
        /// <param name="ImagePath">Image file path</param>
        /// <returns>2D array of colors</returns>
        public static RGBPixel[,] OpenImage(string ImagePath)
        {
            Bitmap original_bm = new Bitmap(ImagePath);
            int Height = original_bm.Height;
            int Width = original_bm.Width;

            RGBPixel[,] Buffer = new RGBPixel[Height, Width];

            unsafe
            {
                #region Do Change Remove Template Code
                /// 08e850689d67340abacf2bc76d98212d
                #endregion
                BitmapData bmd = original_bm.LockBits(new Rectangle(0, 0, Width, Height), ImageLockMode.ReadWrite, original_bm.PixelFormat);
                int x, y;
                int nWidth = 0;
                bool Format32 = false;
                bool Format24 = false;
                bool Format8 = false;

                if (original_bm.PixelFormat == PixelFormat.Format24bppRgb)
                {
                    Format24 = true;
                    nWidth = Width * 3;
                }
                else if (original_bm.PixelFormat == PixelFormat.Format32bppArgb || original_bm.PixelFormat == PixelFormat.Format32bppRgb || original_bm.PixelFormat == PixelFormat.Format32bppPArgb)
                {
                    Format32 = true;
                    nWidth = Width * 4;
                }
                else if (original_bm.PixelFormat == PixelFormat.Format8bppIndexed)
                {
                    Format8 = true;
                    nWidth = Width;
                }
                #region Do Change Remove Template Code
                /// 08e850689d67340abacf2bc76d98212d
                #endregion
                int nOffset = bmd.Stride - nWidth;
                byte* p = (byte*)bmd.Scan0;
                for (y = 0; y < Height; y++)
                {
                    for (x = 0; x < Width; x++)
                    {
                        if (Format8)
                        {
                            Buffer[y, x].red = Buffer[y, x].green = Buffer[y, x].blue = p[0];
                            p++;
                        }
                        else
                        {
                            Buffer[y, x].red = p[0];
                            Buffer[y, x].green = p[1];
                            Buffer[y, x].blue = p[2];
                            if (Format24) p += 3;
                            else if (Format32) p += 4;
                        }
                    }
                    p += nOffset;
                }
                original_bm.UnlockBits(bmd);
            }

            return Buffer;
        }

        /// <summary>
        /// Get the height of the image 
        /// </summary>
        /// <param name="ImageMatrix">2D array that contains the image</param>
        /// <returns>Image Height</returns>
        public static int GetHeight(RGBPixel[,] ImageMatrix)
        {
            return ImageMatrix.GetLength(0);
        }

        /// <summary>
        /// Get the width of the image 
        /// </summary>
        /// <param name="ImageMatrix">2D array that contains the image</param>
        /// <returns>Image Width</returns>
        public static int GetWidth(RGBPixel[,] ImageMatrix)
        {
            return ImageMatrix.GetLength(1);
        }

        /// <summary>
        /// Calculate edge energy between
        ///     1. the given pixel and its right one (X)
        ///     2. the given pixel and its bottom one (Y)
        /// </summary>
        /// <param name="x">pixel x-coordinate</param>
        /// <param name="y">pixel y-coordinate</param>
        /// <param name="ImageMatrix">colored image matrix</param>
        /// <returns>edge energy with the right pixel (X) and with the bottom pixel (Y)</returns>
        public static Vector2D CalculatePixelEnergies(int x, int y, RGBPixel[,] ImageMatrix)
        {
            if (ImageMatrix == null) throw new Exception("image is not set!");

            Vector2D gradient = CalculateGradientAtPixel(x, y, ImageMatrix);
            #region Do Change Remove Template Code
            /// 08e850689d67340abacf2bc76d98212d
            #endregion
            double gradientMagnitude = Math.Sqrt(gradient.X * gradient.X + gradient.Y * gradient.Y);
            double edgeAngle = Math.Atan2(gradient.Y, gradient.X);
            double rotatedEdgeAngle = edgeAngle + Math.PI / 2.0;
            #region Do Change Remove Template Code
            /// 08e850689d67340abacf2bc76d98212d
            #endregion
            Vector2D energy = new Vector2D();
            energy.X = Math.Abs(gradientMagnitude * Math.Cos(rotatedEdgeAngle));
            energy.Y = Math.Abs(gradientMagnitude * Math.Sin(rotatedEdgeAngle));

            return energy;
        }
        /// <summary>
        /// Display the given image on the given PictureBox object
        /// </summary>
        /// <param name="ImageMatrix">2D array that contains the image</param>
        /// <param name="PicBox">PictureBox object to display the image on it</param>
        public static void ViewImage(RGBPixel[,] ImageMatrix, PictureBox PicBox)
        {
            // Create Image:
            //==============
            int Height = ImageMatrix.GetLength(0);
            int Width = ImageMatrix.GetLength(1);

            Bitmap ImageBMP = new Bitmap(Width, Height, PixelFormat.Format24bppRgb);

            unsafe
            {
                #region Do Change Remove Template Code
                /// 08e850689d67340abacf2bc76d98212d
                #endregion
                BitmapData bmd = ImageBMP.LockBits(new Rectangle(0, 0, Width, Height), ImageLockMode.ReadWrite, ImageBMP.PixelFormat);
                int nWidth = 0;
                nWidth = Width * 3;
                int nOffset = bmd.Stride - nWidth;
                byte* p = (byte*)bmd.Scan0;
                for (int i = 0; i < Height; i++)
                {
                    #region Do Change Remove Template Code
                    /// 08e850689d67340abacf2bc76d98212d
                    #endregion
                    for (int j = 0; j < Width; j++)
                    {
                        p[0] = ImageMatrix[i, j].red;
                        p[1] = ImageMatrix[i, j].green;
                        p[2] = ImageMatrix[i, j].blue;
                        p += 3;
                    }

                    p += nOffset;
                }
                ImageBMP.UnlockBits(bmd);
            }
            PicBox.Image = ImageBMP;
        }


        /// <summary>
        /// Apply Gaussian smoothing filter to enhance the edge detection 
        /// </summary>
        /// <param name="ImageMatrix">Colored image matrix</param>
        /// <param name="filterSize">Gaussian mask size</param>
        /// <param name="sigma">Gaussian sigma</param>
        /// <returns>smoothed color image</returns>
        public static RGBPixel[,] GaussianFilter1D(RGBPixel[,] ImageMatrix, int filterSize, double sigma)
        {
            int Height = GetHeight(ImageMatrix);
            int Width = GetWidth(ImageMatrix);

            RGBPixelD[,] VerFiltered = new RGBPixelD[Height, Width];
            RGBPixel[,] Filtered = new RGBPixel[Height, Width];


            // Create Filter in Spatial Domain:
            //=================================
            //make the filter ODD size
            if (filterSize % 2 == 0) filterSize++;

            double[] Filter = new double[filterSize];
            #region Do Change Remove Template Code
            /// 08e850689d67340abacf2bc76d98212d
            #endregion
            //Compute Filter in Spatial Domain :
            //==================================
            double Sum1 = 0;
            int HalfSize = filterSize / 2;
            for (int y = -HalfSize; y <= HalfSize; y++)
            {
                //Filter[y+HalfSize] = (1.0 / (Math.Sqrt(2 * 22.0/7.0) * Segma)) * Math.Exp(-(double)(y*y) / (double)(2 * Segma * Segma)) ;
                Filter[y + HalfSize] = Math.Exp(-(double)(y * y) / (double)(2 * sigma * sigma));
                Sum1 += Filter[y + HalfSize];
            }
            for (int y = -HalfSize; y <= HalfSize; y++)
            {
                Filter[y + HalfSize] /= Sum1;
            }
            #region Do Change Remove Template Code
            /// 08e850689d67340abacf2bc76d98212d
            #endregion
            //Filter Original Image Vertically:
            //=================================
            int ii, jj;
            RGBPixelD Sum;
            RGBPixel Item1;
            RGBPixelD Item2;

            for (int j = 0; j < Width; j++)
                for (int i = 0; i < Height; i++)
                {
                    Sum.red = 0;
                    Sum.green = 0;
                    Sum.blue = 0;
                    for (int y = -HalfSize; y <= HalfSize; y++)
                    {
                        ii = i + y;
                        if (ii >= 0 && ii < Height)
                        {
                            Item1 = ImageMatrix[ii, j];
                            Sum.red += Filter[y + HalfSize] * Item1.red;
                            Sum.green += Filter[y + HalfSize] * Item1.green;
                            Sum.blue += Filter[y + HalfSize] * Item1.blue;
                        }
                    }
                    VerFiltered[i, j] = Sum;
                }
            #region Do Change Remove Template Code
            /// 08e850689d67340abacf2bc76d98212d
            #endregion
            //Filter Resulting Image Horizontally:
            //===================================
            for (int i = 0; i < Height; i++)
                for (int j = 0; j < Width; j++)
                {
                    Sum.red = 0;
                    Sum.green = 0;
                    Sum.blue = 0;
                    for (int x = -HalfSize; x <= HalfSize; x++)
                    {
                        jj = j + x;
                        if (jj >= 0 && jj < Width)
                        {
                            Item2 = VerFiltered[i, jj];
                            Sum.red += Filter[x + HalfSize] * Item2.red;
                            Sum.green += Filter[x + HalfSize] * Item2.green;
                            Sum.blue += Filter[x + HalfSize] * Item2.blue;
                        }
                    }
                    Filtered[i, j].red = (byte)Sum.red;
                    Filtered[i, j].green = (byte)Sum.green;
                    Filtered[i, j].blue = (byte)Sum.blue;
                }

            return Filtered;
        }


        #region Private Functions
        /// <summary>
        /// Calculate Gradient vector between the given pixel and its right and bottom ones
        /// </summary>
        /// <param name="x">pixel x-coordinate</param>
        /// <param name="y">pixel y-coordinate</param>
        /// <param name="ImageMatrix">colored image matrix</param>
        /// <returns></returns>
        private static Vector2D CalculateGradientAtPixel(int x, int y, RGBPixel[,] ImageMatrix)
        {
            Vector2D gradient = new Vector2D();

            RGBPixel mainPixel = ImageMatrix[y, x];
            double pixelGrayVal = 0.21 * mainPixel.red + 0.72 * mainPixel.green + 0.07 * mainPixel.blue;

            if (y == GetHeight(ImageMatrix) - 1)
            {
                //boundary pixel.
                for (int i = 0; i < 3; i++)
                {
                    gradient.Y = 0;
                }
            }
            else
            {
                RGBPixel downPixel = ImageMatrix[y + 1, x];
                double downPixelGrayVal = 0.21 * downPixel.red + 0.72 * downPixel.green + 0.07 * downPixel.blue;

                gradient.Y = pixelGrayVal - downPixelGrayVal;
            }
            #region Do Change Remove Template Code
            /// 08e850689d67340abacf2bc76d98212d
            #endregion
            if (x == GetWidth(ImageMatrix) - 1)
            {
                //boundary pixel.
                gradient.X = 0;

            }
            else
            {
                RGBPixel rightPixel = ImageMatrix[y, x + 1];
                double rightPixelGrayVal = 0.21 * rightPixel.red + 0.72 * rightPixel.green + 0.07 * rightPixel.blue;

                gradient.X = pixelGrayVal - rightPixelGrayVal;
            }
            
            return gradient;
        }


        #endregion
    }

}
