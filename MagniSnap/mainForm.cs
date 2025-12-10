using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;
using MagniSnap;

namespace MagniSnap
{
    #region
    /// 4d17639adfad0a300acd78759e07a4f2
    #endregion
    public partial class MainForm : Form
    {
        RGBPixel[,] ImageMatrix;
        bool isLassoEnabled = false;

        //Task 4:
        private int radius = 50;
        Dictionary<Node, List<(Node, double)>> graph;
        Dictionary<Node, Node> parents;
        Node anchorNode;
        bool anchorSelected = false;
        //
        public MainForm()
        {
            InitializeComponent();
            indicator_pnl.Hide();
        }

        private void menuButton_Click(object sender, EventArgs e)
        {
            #region Do Change Remove Template Code
            /// 4d17639adfad0a300acd78759e07a4f2
            #endregion

            indicator_pnl.Top = ((Control)sender).Top;
            indicator_pnl.Height = ((Control)sender).Height;
            indicator_pnl.Left = ((Control)sender).Left;
            ((Control)sender).BackColor = Color.FromArgb(37, 46, 59);
            indicator_pnl.Show();
        }

        private void menuButton_Leave(object sender, EventArgs e)
        {
            ((Control)sender).BackColor = Color.FromArgb(26, 32, 40);
        }

        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        private void openToolStripMenuItem_Click(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.Filter = "Image Files|*.jpg;*.jpeg;*.png;*.bmp;*.gif";

            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                string OpenedFilePath = openFileDialog1.FileName;
                ImageMatrix = ImageToolkit.OpenImage(OpenedFilePath);
                ImageToolkit.ViewImage(ImageMatrix, mainPictureBox);

                txtWidth.Text = ImageToolkit.GetWidth(ImageMatrix).ToString();
                txtHeight.Text = ImageToolkit.GetHeight(ImageMatrix).ToString();

                anchorSelected = false; // reset anchor
                parents = null; // clear previous Dijkstra paths
            }
        }


        private void clearToolStripMenuItem_Click(object sender, EventArgs e)
        {
            mainPictureBox.Refresh();
        }

        private void btnLivewire_Click(object sender, EventArgs e)
        {
            menuButton_Click(sender, e);

            mainPictureBox.Cursor = Cursors.Cross;

            isLassoEnabled = true;
            anchorSelected = false; // reset anchor so new click sets it
        }
        

        private void btnLivewire_Leave(object sender, EventArgs e)
        {
            menuButton_Leave(sender, e);

            mainPictureBox.Cursor = Cursors.Default;
            isLassoEnabled = false;
        }


        // --- MouseClick handler ---
        private void mainPictureBox_MouseClick(object sender, MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left || ImageMatrix == null || !isLassoEnabled) return;

            int row = e.Y;
            int col = e.X;
            anchorNode = new Node(row, col);
            anchorSelected = true;

            int radius = 50; // can be adjusted
            parents = ImageToolkit.DijkstraSubgraph(ImageMatrix, anchorNode, radius);
        }



        // --- MouseMove handler ---
        private void mainPictureBox_MouseMove(object sender, MouseEventArgs e)
        {
            // 1️⃣ Safety checks
            if (ImageMatrix == null || !isLassoEnabled || !anchorSelected)
                return;

            int row = e.Y;
            int col = e.X;

            // 2️⃣ Bounds check
            int height = ImageToolkit.GetHeight(ImageMatrix);
            int width = ImageToolkit.GetWidth(ImageMatrix);
            if (row < 0 || row >= height || col < 0 || col >= width)
                return;

            Node targetNode = new Node(row, col);

            // 3️⃣ Recompute subgraph if mouse leaves current radius (only for large images)
            if (height * width > 512 * 512)
            {
                if (Math.Abs(row - anchorNode.x) > radius || Math.Abs(col - anchorNode.y) > radius)
                {
                    // reset anchor to current mouse position
                    anchorNode = new Node(row, col);
                    parents = ImageToolkit.DijkstraSubgraph(ImageMatrix, anchorNode, radius);
                }
            }

            // 4️⃣ Backtrack shortest path from current mouse to anchor
            var path = ImageToolkit.BacktrackShortestPath(parents, targetNode);

            // 5️⃣ Draw path on a temporary clone (so original image is preserved)
            RGBPixel[,] temp = (RGBPixel[,])ImageMatrix.Clone();
            ImageToolkit.DrawPath(temp, path);

            // 6️⃣ Display temp image with livewire
            ImageToolkit.ViewImage(temp, mainPictureBox);
        }


    }
}
