using Microsoft.Kinect;
using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace ViewModule
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Used to display 1,000 points on screen.
        private List<Ellipse> _points = new List<Ellipse>();
        private List<CameraSpacePoint> _vertices = new List<CameraSpacePoint>();
        private KinectSensor _sensor = KinectSensor.GetDefault();

        private List<double> _columnsCSV;
        private List<List<double>> _rowsCSV;

        private float _headX;
        private float _headY;
        private float _headZ;

        private float _deltaX;
        private float _deltaY;
        private float _deltaZ;

        private List<double> _rotationW;
        private List<double> _rotationX;
        private List<double> _rotationY;
        private List<double> _rotationZ;

        private Boolean _printDebugs = false;

        private Boolean _isLoad = false;

        public MainWindow()
        {
            this.InitializeComponent();
            Initialize_Sensor();
           

            Thread.Sleep(5000);
            //UpdateFacePoints();
        }

        private void Initialize_Sensor()
        {
            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                Console.WriteLine("Sensor Ready");

                // Start tracking!        
                _sensor.Open();
            }
        }

        private void FaceDataToVetices(int rowIndex)
        {

            if (rowIndex < 0 || rowIndex >= _rowsCSV.Count) rowIndex = 0;
            List<double> row = _rowsCSV[rowIndex];

            _vertices = new List<CameraSpacePoint>();

            _deltaX = (float)row[0];
            _deltaY = (float)row[1];
            _deltaZ = (float)row[2] - 1;

            //System.Diagnostics.Debug.WriteLine("INDEX: " + rowIndex);
            //System.Diagnostics.Debug.WriteLine("length rot W: " + _rotationW.Count);

            float rotW = (float)_rotationW[rowIndex];
            float rotX = (float)_rotationX[rowIndex];
            float rotY = (float)_rotationY[rowIndex];
            float rotZ = (float)_rotationZ[rowIndex];

            if (_printDebugs)
            {
                Console.WriteLine("HEADX: " + _headX);
                Console.WriteLine("HEADY: " + _headY);
                Console.WriteLine("HEADZ: " + _headZ);
            }

            for (int i = 0; i < row.Count; i += 3)
            {
                CameraSpacePoint vert = new CameraSpacePoint();

                Boolean doNormalize = true;

                if (doNormalize)
                {
                    
                    vert.X = (float)row[i] - _deltaX;
                    vert.Y = (float)row[i + 1] - _deltaY;
                    vert.Z = (float)row[i + 2] - _deltaZ;

                    /*
                    float yaw = (float)Math.Atan2(2.0 * (rotY * rotZ + rotW * rotX), rotW * rotW - rotX * rotX - rotY * rotY + rotZ * rotZ);
                    float pitch = (float)Math.Asin(-2.0 * (rotX * rotZ - rotW * rotY));
                    float roll = (float)Math.Atan2(2.0 * (rotX * rotY + rotW * rotZ), rotW * rotW + rotX * rotX - rotY * rotY - rotZ * rotZ);
                    */

                    /*
                    float roll = (float)Math.Atan2(2 * rotY * rotW - 2 * rotX * rotZ, 1 - 2 * rotY * rotY - 2 * rotZ * rotZ);
                    float pitch = (float)Math.Atan2(2 * rotX * rotW - 2 * rotY * rotZ, 1 - 2 * rotX * rotX - 2 * rotZ * rotZ);
                    float yaw = (float)Math.Asin(2 * rotX * rotY + 2 * rotZ * rotW);
                    */

                    
                    float roll = (float)Math.Atan2(2 * rotY * rotW + 2 * rotX * rotZ, 1 - 2 * rotY * rotY - 2 * rotZ * rotZ);
                    float pitch = (float)Math.Atan2(2 * rotX * rotW + 2 * rotY * rotZ, 1 - 2 * rotX * rotX - 2 * rotZ * rotZ);
                    float yaw = (float)Math.Asin(2 * rotX * rotY + 2 * rotZ * rotW);
                    

                    /*
                    float yaw = (float)Math.Atan(rotX / (-rotY)) / (float)0.01745329252;
                    float pitch = (float)Math.Atan(Math.Sqrt(rotX * rotX + rotY * rotY) / rotZ) / (float)0.01745329252;
                    float roll = 0;
                    */

                    /*
                    roll = roll / (float)0.01745329252;
                    pitch = pitch / (float)0.01745329252;
                    yaw = yaw / (float)0.01745329252;
                    */

                    /*
                    System.Diagnostics.Debug.WriteLine("YAW " + yaw);
                    System.Diagnostics.Debug.WriteLine("PITCH " + pitch);
                    System.Diagnostics.Debug.WriteLine("ROLL " + roll);
                    */

                    /*
                    System.Diagnostics.Debug.WriteLine("yaw " + yaw);
                    System.Diagnostics.Debug.WriteLine("pitch " + pitch);
                    System.Diagnostics.Debug.WriteLine("roll " + roll);
                    */

                    /*
                    float yaw;
                    float pitch;
                    float roll;

                    //if (_rotationW > 1) q1.normalise(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
                    double angle = 2 * Math.Acos(rotW);
                    double s = Math.Sqrt(1 - rotW * rotW); // assuming quaternion normalised then w is less than 1, so term always positive.
                    if (s < 0.001)
                    { // test to avoid divide by zero, s is always positive due to sqrt
                      // if s close to zero then direction of axis not important
                        pitch = rotX; // if it is important that axis is normalised then replace with x=1; y=z=0;
                        yaw = rotY;
                        roll = rotZ;
                    }
                    else
                    {
                        pitch = (float)(rotX / s); // normalise axis
                        yaw = (float)(rotY / s);
                        roll = (float)(rotZ / s);
                    }
                    */

                    Point3d normalized = rotate_3D(_headX, _headY, _headZ, vert.X, vert.Y, vert.Z, yaw, pitch, roll);

                    vert.X = normalized.X;
                    vert.Y = normalized.Y;
                    vert.Z = normalized.Z;

                    /*
                    System.Diagnostics.Debug.WriteLine("X " + vert.X);
                    System.Diagnostics.Debug.WriteLine("Y " + vert.Y);
                    System.Diagnostics.Debug.WriteLine("Z " + vert.Z);*/
                    
                }
                else
                {
                    vert.X = (float)row[i];
                    vert.Y = (float)row[i + 1];
                    vert.Z = (float)row[i + 2];
                }

                _vertices.Add(vert);
            }
        }

        private void UpdateFacePoints()
        {

            if (!_isLoad) return;
            //TODO Read Vertices here, pass to _vertices

            //Console.WriteLine("11111111111111111111111111111111111111111");
            FaceDataToVetices((int)(Slider_frame.Value/10*_rowsCSV.Count()));

            if (_points.Count == 0)
            {

                //Console.WriteLine("222222222222222222222222222");
                for (int index = 0; index < _vertices.Count(); index++)
                {
                    Ellipse ellipse = new Ellipse
                    {
                        Width = 2.0,
                        Height = 2.0,
                        Fill = new SolidColorBrush(Colors.Yellow)
                    };

                    _points.Add(ellipse);


                    //Console.WriteLine("333333333333333333333333333333333");

                }

                foreach (Ellipse ellipse in _points)
                {

                    //Console.WriteLine("444444444444444444444444444");
                    canvas.Children.Add(ellipse);
                }
            }
            /*
                            if (_pointTextBlocks.Count == 0)
                            {
                                for (int index = 0; index < vertices.Count(); index++)
                                {
                                    TextBlock textBlock = new TextBlock
                                    {
                                        Text = _keyPoints.ElementAt(index).ToString(),

                                        Foreground = new SolidColorBrush(Colors.White),
                                        TextAlignment = TextAlignment.Left,
                                        FontSize = 5
                                    };


                                    _pointTextBlocks.Add(textBlock);
                                }

                                foreach (TextBlock textBlock in _pointTextBlocks)
                                {
                                    canvas.Children.Add(textBlock);
                                }
                            }

            */

            if (_printDebugs)
            {
                Console.WriteLine("===================================================");
                Console.WriteLine("|                   A  ROW                        |");
                Console.WriteLine("===================================================");
            }

            for (int index = 0; index < _vertices.Count(); index++)
            {


                //Console.WriteLine("5555555555555555555555");
                CameraSpacePoint vertice = _vertices[index];

                //vertice.X = vertice.X - _deltaX;
                //vertice.Y = vertice.Y - _deltaY;
                //vertice.Z = vertice.Z - _deltaZ;

                if (_printDebugs)
                {
                    Console.WriteLine("Vertex X:" + vertice.X);
                    Console.WriteLine("Vertex Y:" + vertice.Y);
                    Console.WriteLine("Vertex Z:" + vertice.Z);
                }

                DepthSpacePoint point = _sensor.CoordinateMapper.MapCameraPointToDepthSpace(vertice);

                

                //Console.WriteLine("YESSSSSSSSS"+point.X +" "+point.Y);
                if (float.IsInfinity(point.X) || float.IsInfinity(point.Y)) return;

                Ellipse ellipse = _points[index];

                ////Console.WriteLine("77777777777777777777");

                //TextBlock textBlock = _pointTextBlocks[index];

                Canvas.SetLeft(ellipse, point.X);
                Canvas.SetTop(ellipse, point.Y);

                //Canvas.SetLeft(textBlock, point.X + 5);
                //Canvas.SetTop(textBlock, point.Y);

            }


            



        }

        private void Dir_Click(object sender, RoutedEventArgs e)
        {

            OpenFileDialog openFileDialog = new OpenFileDialog();
            openFileDialog.Filter = "CSV (*.csv)|*.csv|All files (*.*)|*.*";
            if (openFileDialog.ShowDialog() == true)
               Box_File.Text = (openFileDialog.FileName);
        }

        private void Load_Click(object sender, RoutedEventArgs e)
        {
            _rotationW = new List<double>();
            _rotationX = new List<double>();
            _rotationY = new List<double>();
            _rotationZ = new List<double>();

            using (var reader = new StreamReader(Box_File.Text))
            {
                reader.ReadLine();
                reader.ReadLine();

                _rowsCSV = new List<List<double>>();

                while (!reader.EndOfStream)
                {
                    _columnsCSV = new List<double>();
                    

                    var line = reader.ReadLine();
                    var values = line.Split(',');

                    _rotationW.Add(float.Parse(values[2]));
                    _rotationX.Add(float.Parse(values[3]));
                    _rotationY.Add(float.Parse(values[4]));
                    _rotationZ.Add(float.Parse(values[5]));

                    for (int i = 6; i < values.Length; i++)
                    {
                        if(values[i]!=null && values[i] != "")
                        _columnsCSV.Add(Double.Parse(values[i]));
                    }

                    _rowsCSV.Add(_columnsCSV);
                }
            }

            _isLoad = true;
            Slider_frame.Value = 0;
            //Normalize();
            UpdateFacePoints();
        }

        private void Slider_frame_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            UpdateFacePoints();
        }

        private void Normalize()
        {
            int row = 0;

            _headX = (float)_rowsCSV[row][0];
            _headY = (float)_rowsCSV[row][1];
            _headZ = (float)_rowsCSV[row][2];

            _headX = (float)0;// 0.014;
            _headY = (float)0;// 0.15;
            _headZ = (float)1;// 0.95;

            Console.WriteLine("HEADX: " + _headX);
            Console.WriteLine("HEADY: " + _headY);
            Console.WriteLine("HEADZ: " + _headZ);
            

            /*
            while(Slider_frame.Value < Slider_frame.Maximum)
            {
                Slider_frame.Value++;

            }
            */
        }

        void rotate_2D(float cx, float cy, float angle, Point2d p)
        {
            float s = (float)Math.Sin(angle);
            float c = (float)Math.Cos(angle);

            // translate point back to origin:
            p.X -= cx;
            p.Y -= cy;

            // rotate point
            float xnew = p.X * c - p.Y * s;
            float ynew = p.X * s + p.Y * c;

            // translate point back:
            p.X = xnew + cx;
            p.Y = ynew + cy;
        }

        Point3d rotate_3D(float headX, float headY, float headZ, float x, float y, float z, float yaw, float pitch, float roll)
                {

                Point2d tempPoint;

                    //yaw = z
                tempPoint = new Point2d(x, y);
                rotate_2D(headX, headY, -roll, tempPoint);
                x = tempPoint.X;
                y = tempPoint.Y;

                //pitch = y
                tempPoint = new Point2d(x, z);
                rotate_2D(headX, headZ, -yaw, tempPoint);
                x = tempPoint.X;
                z = tempPoint.Y;

                //roll = x
                tempPoint = new Point2d(y, z);
                rotate_2D(headY, headZ, -pitch, tempPoint);
                y = tempPoint.X;
                z = tempPoint.Y;


                return new Point3d(x, y, z);
        }
    }
}