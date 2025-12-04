using System;
using System.Collections.Generic;
using System.Drawing;
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
            #region Do Change Remove Template Code
            /// 4d17639adfad0a300acd78759e07a4f2
            #endregion

            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            //
            openFileDialog1.Filter = "Image Files|*.jpg;*.jpeg;*.png;*.bmp;*.gif";
            //

            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {


                string OpenedFilePath = openFileDialog1.FileName;
                ImageMatrix = ImageToolkit.OpenImage(OpenedFilePath);
                ImageToolkit.ViewImage(ImageMatrix, mainPictureBox);

                int width = ImageToolkit.GetWidth(ImageMatrix);
                txtWidth.Text = width.ToString();
                int height = ImageToolkit.GetHeight(ImageMatrix);
                txtHeight.Text = height.ToString();

                //Task 4
                graph = ImageToolkit.Construct_Graph(ImageMatrix);
                anchorSelected = false;
                //

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
        }

        private void btnLivewire_Leave(object sender, EventArgs e)
        {
            menuButton_Leave(sender, e);

            mainPictureBox.Cursor = Cursors.Default;
            isLassoEnabled = false;
        }

        private void mainPictureBox_MouseClick(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                if (ImageMatrix != null && isLassoEnabled)
                {
                    int row = e.Y;
                    int col = e.X;

                    // Bounds check (in case PictureBox is larger than image)
                    int height = ImageToolkit.GetHeight(ImageMatrix);
                    int width = ImageToolkit.GetWidth(ImageMatrix);

                    if (row < 0 || row >= height || col < 0 || col >= width)
                        return;

                    // Set anchor
                    anchorNode = new Node(row, col);
                    anchorSelected = true;

                    // Run Dijkstra ONCE from anchor to all pixels
                    parents = ImageToolkit.Dijkstra(graph, row, col, 0, 0); // end coords ignored
                
                // Refresh to redraw points
                //Task 4 -> comment: mainPictureBox.Refresh();
            }
            }
        }

        private void mainPictureBox_MouseMove(object sender, MouseEventArgs e)
        {
            txtMousePosX.Text = e.X.ToString();
            txtMousePosY.Text = e.Y.ToString();

            if (ImageMatrix != null && isLassoEnabled)
            {
                int row = e.Y;
                int col = e.X;

                int height = ImageToolkit.GetHeight(ImageMatrix);
                int width = ImageToolkit.GetWidth(ImageMatrix);

                if (row < 0 || row >= height || col < 0 || col >= width)
                    return;

                Node targetNode = new Node(row, col);

                // Backtrack shortest path from mouse to anchor
                var path = ImageToolkit.BacktrackShortestPath(parents, targetNode);

                // Clone original image so we don't modify it permanently
                RGBPixel[,] temp = (RGBPixel[,])ImageMatrix.Clone();

                // Draw path on temp image
                ImageToolkit.DrawPath(temp, path);

                // Display temp image with the livewire
                ImageToolkit.ViewImage(temp, mainPictureBox);
            
            // Refresh to redraw points
            //Task 4 -> comment:mainPictureBox.Refresh();
        }
        }
    }
}
