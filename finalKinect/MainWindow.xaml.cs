// -----------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace FaceTrackingBasics
{
    using System;
    using System.IO;
    using System.Collections.Generic;
    using System.Linq;
    using System.Text;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Data;
    using System.Windows.Documents;
    using System.Windows.Input;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Toolkit;
    using Amazon.S3;
    using Amazon.S3.Transfer;
    using System.Net;

    using System.Threading;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 

    public partial class MainWindow : Window
    {

        static string existingBucketName = "movistorage";
        static string filePath;
        static string filePath2;
        StringBuilder dataParams = new StringBuilder();
        String vertex;


        #region audio
        public static void WriteWavFile(KinectAudioSource source, FileStream fileStream)
        {
            var size = 0;
            WriteWavHeader(fileStream, size);
            using (var audioStream = source.Start())
            {
                while (audioStream.Read(buffer, 0, buffer.Length) > 0 && _isRecording)
                {
                    fileStream.Write(buffer, 0, buffer.Length);
                    size += buffer.Length;
                }
            }

            long prePosition = fileStream.Position;
            fileStream.Seek(0, SeekOrigin.Begin);
            WriteWavHeader(fileStream, size);
            fileStream.Seek(prePosition, SeekOrigin.Begin);
            fileStream.Flush();
        }

        string _recordingFileName;


        /*
        void MainWindow_FinishedRecording(object sender, RoutedEventArgs e)
        {
            //This is only required if recording on a separate thread to ensure that enabling the buttons
            //happens on the UI thread
            Dispatcher.BeginInvoke(new ThreadStart(ReenableButtons));

            //use this if recording on the same thread
            //ReenableButtons(); 
        }
        
        
        private void ReenableButtons()
        {
            RecordButton.IsEnabled = true;
            StopButton.IsEnabled = false;
        }
         */
        static byte[] buffer = new byte[4096];
        static bool _isRecording;

        public static bool IsRecording
        {
            get { return _isRecording; }
            set { _isRecording = value; }
        }



        private object lockObj = new object();
        private void RecordKinectAudio()
        {
            IsRecording = true;


            using (var fileStream =
                new FileStream(_recordingFileName, FileMode.Create))
            {
                WriteWavFile(KinectSensor.KinectSensors[0].AudioSource, fileStream);
            }
            IsRecording = false;

        }



        private void RecordAudio(object kinectSensor)
        {

            KinectSensor _sensor = (KinectSensor)kinectSensor;
            RecordAudio(_sensor);
        }
        /// <summary>
        /// A bare bones WAV file header writer
        /// </summary>        
        static void WriteWavHeader(Stream stream, int dataLength)
        {
            //We need to use a memory stream because the BinaryWriter will close the underlying stream when it is closed
            using (var memStream = new MemoryStream(64))
            {
                int cbFormat = 18; //sizeof(WAVEFORMATEX)
                WAVEFORMATEX format = new WAVEFORMATEX()
                {
                    wFormatTag = 1,
                    nChannels = 1,
                    nSamplesPerSec = 16000,
                    nAvgBytesPerSec = 32000,
                    nBlockAlign = 2,
                    wBitsPerSample = 16,
                    cbSize = 0
                };

                using (var bw = new BinaryWriter(memStream))
                {
                    //RIFF header
                    WriteString(memStream, "RIFF");
                    bw.Write(dataLength + cbFormat + 4); //File size - 8
                    WriteString(memStream, "WAVE");
                    WriteString(memStream, "fmt ");
                    bw.Write(cbFormat);

                    //WAVEFORMATEX
                    bw.Write(format.wFormatTag);
                    bw.Write(format.nChannels);
                    bw.Write(format.nSamplesPerSec);
                    bw.Write(format.nAvgBytesPerSec);
                    bw.Write(format.nBlockAlign);
                    bw.Write(format.wBitsPerSample);
                    bw.Write(format.cbSize);

                    //data header
                    WriteString(memStream, "data");
                    bw.Write(dataLength);
                    memStream.WriteTo(stream);
                }
            }
        }

        static void WriteString(Stream stream, string s)
        {
            byte[] bytes = Encoding.ASCII.GetBytes(s);
            stream.Write(bytes, 0, bytes.Length);
        }

        struct WAVEFORMATEX
        {
            public ushort wFormatTag;
            public ushort nChannels;
            public uint nSamplesPerSec;
            public uint nAvgBytesPerSec;
            public ushort nBlockAlign;
            public ushort wBitsPerSample;
            public ushort cbSize;
        }



        private void StopKinect(KinectSensor sensor)
        {
            if (sensor != null)
            {
                if (sensor.IsRunning)
                {
                    //stop sensor 
                    sensor.Stop();

                    //stop audio if not null
                    if (sensor.AudioSource != null)
                    {
                        sensor.AudioSource.Stop();
                    }


                }
            }
        }


        #endregion

        int Sleep = 0;
        double count = 0;
        double S1 = 0;
        double S2 = 0;
        double S3 = 0;
        double S4 = 0;
        double S5 = 0;
        double S6 = 0;

        double SP1 = 0;
        double SP2 = 0;
        double SP3 = 0;
        double SP4 = 0;
        double SP5 = 0;
        double SP6 = 0;

        int moving1 = 0;
        int moving2 = 0;
        int moving3 = 0;
        string movings = " ";
        string movings1 = " ";

        System.Windows.Point ShoulderCenter1 = new System.Windows.Point();

        System.Windows.Point ShoulderRight1 = new System.Windows.Point();
        System.Windows.Point ElbowRight1 = new System.Windows.Point();
        System.Windows.Point WristRight1 = new System.Windows.Point();


        System.Windows.Point ShoulderLeft1 = new System.Windows.Point();
        System.Windows.Point ElbowLeft1 = new System.Windows.Point();
        System.Windows.Point WristLeft1 = new System.Windows.Point();


        System.Windows.Point HipRight1 = new System.Windows.Point();
        System.Windows.Point KneeRight1 = new System.Windows.Point();
        System.Windows.Point AnkleRight1 = new System.Windows.Point();


        System.Windows.Point HipLeft1 = new System.Windows.Point();
        System.Windows.Point KneeLeft1 = new System.Windows.Point();
        System.Windows.Point AnkleLeft1 = new System.Windows.Point();


        double UR;
        double UL;
        double USR;
        double USL;
        double DR;
        double DL;
        private static readonly int Bgr32BytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        private readonly KinectSensorChooser sensorChooser = new KinectSensorChooser();
        private WriteableBitmap colorImageWritableBitmap;
        private byte[] colorImageData;
        private ColorImageFormat currentColorImageFormat = ColorImageFormat.Undefined;

        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        public MainWindow()
        {
            InitializeComponent();

            var faceTrackingViewerBinding = new Binding("Kinect") { Source = sensorChooser };
            faceTrackingViewer.SetBinding(FaceTrackingViewer.KinectProperty, faceTrackingViewerBinding);
            
            sensorChooser.KinectChanged += SensorChooserOnKinectChanged;
            StopButton.IsEnabled = false;
            sensorChooser.Start();
        }

        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new System.Windows.Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new System.Windows.Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new System.Windows.Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new System.Windows.Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        /// 
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser1_KinectSensorChanged);
            
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();
            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }
            

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

        }
        void kinectSensorChooser1_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            var old = (KinectSensor)e.OldValue;

            if (old != null)
            {
                old.Stop();
                old.AudioSource.Stop();
            }

            KinectSensor sensor = (KinectSensor)e.NewValue;
            sensor.Start();

        }

        int hour = 0;
        int min = 0;
        int sec = 0;
        int hour1 = 0;
        int min1 = 0;
        int sec1 = 0;

       

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }
        public double Getdistance(double x1, double y1, double x2, double y2)
        {
            double distance = 0;

            distance = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);

            return distance;
        }
        public double GetAngle(System.Windows.Point x, System.Windows.Point y, System.Windows.Point z)
        {
            double a = Math.Sqrt((x.Y - y.Y) * (x.Y - y.Y) + (x.X - y.X) * (x.X - y.X));
            double b = Math.Sqrt((z.Y - y.Y) * (z.Y - y.Y) + (z.X - y.X) * (z.X - y.X));
            double c = Math.Sqrt((x.Y - z.Y) * (x.Y - z.Y) + (x.X - z.X) * (x.X - z.X));
            double cosc = (a * a + b * b - c * c) / (2 * a * b);
            double angle = 180 - Math.Acos(cosc) / Math.PI * 180;
            return angle;
        }
        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            if (Sleep == 1) sleepstateBlock.Text = "수면시작 : " + sleepStart;
            else sleepstateBlock.Text = "준비완료 !";
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new System.Windows.Rect(0.0, 0.0, RenderWidth, RenderHeight));

                //이 구간에서 잡아준다.
                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);

                            foreach (Joint joint in skel.Joints)
                            {
                                if (skel.TrackingState == SkeletonTrackingState.Tracked)
                                {

                                    switch (joint.JointType)
                                    {
                                        case JointType.WristLeft:
                                            WristLeft1.X = skel.Joints[JointType.WristLeft].Position.X;
                                            WristLeft1.Y = skel.Joints[JointType.WristLeft].Position.Y;
                                            break;
                                        case JointType.ElbowLeft:
                                            ElbowLeft1.X = skel.Joints[JointType.ElbowLeft].Position.X;
                                            ElbowLeft1.Y = skel.Joints[JointType.ElbowLeft].Position.Y;
                                            break;
                                        case JointType.ShoulderLeft:
                                            ShoulderLeft1.X = skel.Joints[JointType.ShoulderLeft].Position.X;
                                            ShoulderLeft1.Y = skel.Joints[JointType.ShoulderLeft].Position.Y;
                                            break;
                                        case JointType.ShoulderRight:
                                            ShoulderRight1.X = skel.Joints[JointType.ShoulderRight].Position.X;
                                            ShoulderRight1.Y = skel.Joints[JointType.ShoulderRight].Position.Y;
                                            break;
                                        case JointType.ElbowRight:
                                            ElbowRight1.X = skel.Joints[JointType.ElbowRight].Position.X;
                                            ElbowRight1.Y = skel.Joints[JointType.ElbowRight].Position.Y;
                                            break;
                                        case JointType.WristRight:
                                            WristRight1.X = skel.Joints[JointType.WristRight].Position.X;
                                            WristRight1.Y = skel.Joints[JointType.WristRight].Position.Y;
                                            break;

                                        case JointType.ShoulderCenter:
                                            ShoulderCenter1.X = skel.Joints[JointType.ShoulderCenter].Position.X;
                                            ShoulderCenter1.Y = skel.Joints[JointType.ShoulderCenter].Position.Y;
                                            break;

                                        case JointType.HipRight:
                                            HipRight1.X = skel.Joints[JointType.HipRight].Position.X;
                                            HipRight1.Y = skel.Joints[JointType.HipRight].Position.Y;
                                            break;
                                        case JointType.KneeRight:
                                            KneeRight1.X = skel.Joints[JointType.KneeRight].Position.X;
                                            KneeRight1.Y = skel.Joints[JointType.KneeRight].Position.Y;
                                            break;
                                        case JointType.AnkleRight:
                                            AnkleRight1.X = skel.Joints[JointType.AnkleRight].Position.X;
                                            AnkleRight1.Y = skel.Joints[JointType.AnkleRight].Position.Y;
                                            break;


                                        case JointType.HipLeft:
                                            HipLeft1.X = skel.Joints[JointType.HipLeft].Position.X;
                                            HipLeft1.Y = skel.Joints[JointType.HipLeft].Position.Y;
                                            break;
                                        case JointType.KneeLeft:
                                            KneeLeft1.X = skel.Joints[JointType.KneeLeft].Position.X;
                                            KneeLeft1.Y = skel.Joints[JointType.KneeLeft].Position.Y;
                                            break;
                                        case JointType.AnkleLeft:
                                            AnkleLeft1.X = skel.Joints[JointType.AnkleLeft].Position.X;
                                            AnkleLeft1.Y = skel.Joints[JointType.AnkleLeft].Position.Y;
                                            break;


                                    }

                                    USR = GetAngle(ShoulderCenter1, ShoulderRight1, ElbowRight1);
                                    USL = GetAngle(ShoulderCenter1, ShoulderLeft1, ElbowLeft1);

                                    UR = GetAngle(ShoulderRight1, ElbowRight1, WristRight1);
                                    UL = GetAngle(ShoulderLeft1, ElbowLeft1, WristLeft1);

                                    DR = GetAngle(HipRight1, KneeRight1, AnkleRight1);
                                    DL = GetAngle(HipLeft1, KneeLeft1, AnkleLeft1);

     
                                }
                                else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                                {
                                    dc.DrawEllipse(
                                    this.centerPointBrush,
                                    null,
                                    this.SkeletonPointToScreen1(skel.Position),
                                    BodyCenterThickness,
                                    BodyCenterThickness);
                                }
                            }

                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new System.Windows.Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// 
        int dichuck = 0; // 총 뒤척임
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            double handDistance;
            handDistance = Getdistance(WristRight1.X, WristRight1.Y, WristLeft1.X, WristLeft1.Y);

            double shoulderDistance;
            shoulderDistance = Getdistance(ShoulderRight1.X, ShoulderRight1.Y, ShoulderLeft1.X, ShoulderLeft1.Y);



            if (Sleep == 1)
            {
               
                //forntBack.Text = "앞 뒤 판별 : " + faceTrackingViewer.isRight();
                moving1++;
                if (moving1 > 50)
                {
                    moving1 = 0;

                    if (movings == sleepAttitude.Text)
                    {
                        moving2++;
                        if (moving2 > 2) // 같은 자세를 얼마나
                        {
                            moving3++;
                            if (moving3 == 2) // 원하는 시간
                            {
                                if (movings1 != movings)
                                {
                                    if (dichuck != 0)
                                    {
                                        ativityBlock.Text = DateTime.Now.ToLongTimeString() + "에 움직임 발생\r\n" + movings1 + " 에서 " + movings + "으로.. \r\n 총 뒤척인 횟수 : " + dichuck;
                                    }
                                    dichuck++;
                                }

                                movings1 = movings;
                            }
                            // 약 ?초동안 자세가 같음을 얘기. if 문을 처음에 만들어서 moving3이 몇 이상이 되고, 자세가 똑같이 ?초동안 바뀌면 뒤척임 체크;
                        }
                    }
                    else
                    {
                        movings = sleepAttitude.Text;
                        moving2 = 0;
                        moving3 = 0;

                    }
                    
                }




                count++;

                SP1 = Math.Round(S1 / count * 100);
                SP2 = Math.Round(S2 / count * 100);
                SP3 = Math.Round(S3 / count * 100);
                SP4 = Math.Round(S4 / count * 100);
                SP5 = Math.Round(S5 / count * 100);
                SP6 = Math.Round(S6 / count * 100);


                testboxx.Text = "shoulderDistance : " + shoulderDistance
                    + "\nShoulderRight.X : " + ShoulderRight1.X + "\nShoulderRight.Y :" + ShoulderRight1.Y
                    + "\nShoulderLeft.X : " + ShoulderLeft1.X + "\nShoulderLeft.Y : " + ShoulderLeft1.Y
                    + "front or back : " + faceTrackingViewer.isRight() ;

                if (shoulderDistance < 0.08)
                {//둘다 한쪽에 몰려있을때
                    //sleepAttitude.Text = "태아형,통나무형, 갈구형";
                    if (DR > 40 && DR > 40)
                    {
                        sleepAttitude.Text = "태아형";
                        S1++;
                        sleepAttitudeNumber.Text = "⑤";

                    }
                    else
                    {
                        if (UR > 60 || UL > 60)
                        {
                            sleepAttitude.Text = "갈구형";
                            S3++;
                            sleepAttitudeNumber.Text = "④";
                        }
                        else
                        {
                            sleepAttitude.Text = "통나무형";
                            S2++;
                            sleepAttitudeNumber.Text = "③";

                        }
                    }
                }

                else//군인형, 불가사리형, 불가사리형
                {
                    if (UR < 60 && UL < 60 
                        && ElbowLeft1.Y>WristLeft1.Y 
                        && ElbowRight1.Y>WristRight1.Y)
                    {

                        sleepAttitude.Text = "군인형";
                        S4++;
                        sleepAttitudeNumber.Text = "①";
                    }
                    else
                    {
                        if (faceTrackingViewer.isRight())
                        {
                            sleepAttitude.Text = " 불가사리형";
                            S6++;
                            sleepAttitudeNumber.Text = "②";
                        }
                        else
                        {
                            sleepAttitude.Text = "자유낙하형";
                            S5++;
                            sleepAttitudeNumber.Text = "⑥";
                        }
                    }


                }
            }
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen1(joint.Position), JointThickness, JointThickness);
                }
            }

            vertex = skeleton.Joints[JointType.Head].Position.X + "," +
                       skeleton.Joints[JointType.Head].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ShoulderCenter].Position.X + "," +
                       skeleton.Joints[JointType.ShoulderCenter].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ShoulderCenter].Position.X + "," +
                       skeleton.Joints[JointType.ShoulderCenter].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ShoulderLeft].Position.X + "," +
                       skeleton.Joints[JointType.ShoulderLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ShoulderCenter].Position.X + "," +
                       skeleton.Joints[JointType.ShoulderCenter].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ShoulderRight].Position.X + "," +
                       skeleton.Joints[JointType.ShoulderRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ShoulderCenter].Position.X + "," +
                       skeleton.Joints[JointType.ShoulderCenter].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.Spine].Position.X + "," +
                       skeleton.Joints[JointType.Spine].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.Spine].Position.X + "," +
                       skeleton.Joints[JointType.Spine].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.HipCenter].Position.X + "," +
                       skeleton.Joints[JointType.HipCenter].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.HipCenter].Position.X + "," +
                       skeleton.Joints[JointType.HipCenter].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.HipLeft].Position.X + "," +
                       skeleton.Joints[JointType.HipLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.HipCenter].Position.X + "," +
                       skeleton.Joints[JointType.HipCenter].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.HipRight].Position.X + "," +
                       skeleton.Joints[JointType.HipRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ShoulderLeft].Position.X + "," +
                       skeleton.Joints[JointType.ShoulderLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ElbowLeft].Position.X + "," +
                       skeleton.Joints[JointType.ElbowLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ElbowLeft].Position.X + "," +
                       skeleton.Joints[JointType.ElbowLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.WristLeft].Position.X + "," +
                       skeleton.Joints[JointType.WristLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.WristLeft].Position.X + "," +
                       skeleton.Joints[JointType.WristLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.HandLeft].Position.X + "," +
                       skeleton.Joints[JointType.HandLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ShoulderRight].Position.X + "," +
                       skeleton.Joints[JointType.ShoulderRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ElbowRight].Position.X + "," +
                       skeleton.Joints[JointType.ElbowRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.ElbowRight].Position.X + "," +
                       skeleton.Joints[JointType.ElbowRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.WristRight].Position.X + "," +
                       skeleton.Joints[JointType.WristRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.WristRight].Position.X + "," +
                       skeleton.Joints[JointType.WristRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.HandRight].Position.X + "," +
                       skeleton.Joints[JointType.HandRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.HipLeft].Position.X + "," +
                       skeleton.Joints[JointType.HipLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.KneeLeft].Position.X + "," +
                       skeleton.Joints[JointType.KneeLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.KneeLeft].Position.X + "," +
                       skeleton.Joints[JointType.KneeLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.AnkleLeft].Position.X + "," +
                       skeleton.Joints[JointType.AnkleLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.AnkleLeft].Position.X + "," +
                       skeleton.Joints[JointType.AnkleLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.FootLeft].Position.X + "," +
                       skeleton.Joints[JointType.FootLeft].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.HipRight].Position.X + "," +
                       skeleton.Joints[JointType.HipRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.KneeRight].Position.X + "," +
                       skeleton.Joints[JointType.KneeRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.KneeRight].Position.X + "," +
                       skeleton.Joints[JointType.KneeRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.AnkleRight].Position.X + "," +
                       skeleton.Joints[JointType.AnkleRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.AnkleRight].Position.X + "," +
                       skeleton.Joints[JointType.AnkleRight].Position.Y + "," + "\n" +
                       skeleton.Joints[JointType.FootRight].Position.X + "," +
                       skeleton.Joints[JointType.FootRight].Position.Y + ",";

            dataParams.Append(vertex + "\n");
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private System.Windows.Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new System.Windows.Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        /// 
        
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen1(joint0.Position), this.SkeletonPointToScreen1(joint1.Position));
        }
        private void SensorChooserOnKinectChanged(object sender, KinectChangedEventArgs kinectChangedEventArgs)
        {
            KinectSensor oldSensor = kinectChangedEventArgs.OldSensor;
            KinectSensor newSensor = kinectChangedEventArgs.NewSensor;
            // Create the drawing group we'll use for drawing

            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    //this.sensor.Start();
                    // 원래 있는 것 ! 
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }
            if (oldSensor != null)
            {
                oldSensor.AllFramesReady -= KinectSensorOnAllFramesReady;
                oldSensor.ColorStream.Disable();
                oldSensor.DepthStream.Disable();
                oldSensor.DepthStream.Range = DepthRange.Default;
                oldSensor.SkeletonStream.Disable();
                oldSensor.SkeletonStream.EnableTrackingInNearRange = false;
                oldSensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
            }

            if (newSensor != null)
            {
                try
                {
                    newSensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                    newSensor.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30);
                    try
                    {
                        // This will throw on non Kinect For Windows devices.
                        newSensor.DepthStream.Range = DepthRange.Near;
                        newSensor.SkeletonStream.EnableTrackingInNearRange = true;
                    }
                    catch (InvalidOperationException)
                    {
                        newSensor.DepthStream.Range = DepthRange.Default;
                        newSensor.SkeletonStream.EnableTrackingInNearRange = false;
                    }

                    //wSensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                    newSensor.SkeletonStream.Enable();
                    newSensor.AllFramesReady += KinectSensorOnAllFramesReady;
                  ;
                }
                catch (InvalidOperationException)
                {
                    // This exception can be thrown when we are trying to
                    // enable streams on a device that has gone away.  This
                    // can occur, say, in app shutdown scenarios when the sensor
                    // goes away between the time it changed status and the
                    // time we get the sensor changed notification.
                    //
                    // Behavior here is to just eat the exception and assume
                    // another notification will come along if a sensor
                    // comes back.
                }
            }
        }

        private void WindowClosed(object sender, EventArgs e)
        {
            sensorChooser.Stop();
            faceTrackingViewer.Dispose();
        }

        private void KinectSensorOnAllFramesReady(object sender, AllFramesReadyEventArgs allFramesReadyEventArgs)
        {
            using (var colorImageFrame = allFramesReadyEventArgs.OpenColorImageFrame())
            {
                if (colorImageFrame == null)
                {
                    return;
                }

                // Make a copy of the color frame for displaying.
                var haveNewFormat = this.currentColorImageFormat != colorImageFrame.Format;
                if (haveNewFormat)
                {
                    this.currentColorImageFormat = colorImageFrame.Format;
                    this.colorImageData = new byte[colorImageFrame.PixelDataLength];
                    this.colorImageWritableBitmap = new WriteableBitmap(
                        colorImageFrame.Width, colorImageFrame.Height, 96, 96, PixelFormats.Bgr32, null);
                    ColorImage.Source = this.colorImageWritableBitmap;
                }

                colorImageFrame.CopyPixelDataTo(this.colorImageData);
                this.colorImageWritableBitmap.WritePixels(
                    new Int32Rect(0, 0, colorImageFrame.Width, colorImageFrame.Height),
                    this.colorImageData,
                    colorImageFrame.Width * Bgr32BytesPerPixel,
                    0);
            }
        }

        private System.Windows.Point SkeletonPointToScreen1(SkeletonPoint skelpoint)
        {

            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new System.Windows.Point(depthPoint.X, depthPoint.Y);
        }
        public string  nowUser="이름을 입력하시오.";
        private void RecordButton_Click(object sender, RoutedEventArgs e)
        {
            Sleep = 1;
            Recording();
            RecordButton.IsEnabled = false;
            StopButton.IsEnabled = true;
            dataParams.Append("&data=");
            nowUser = userName.Text;
        }

        private void StopButton_Click(object sender, RoutedEventArgs e)
        {
 
            Sleep = 0;
            Stoping();
            RecordButton.IsEnabled = true;
            StopButton.IsEnabled = false;
            
            hour1 = Convert.ToInt32(DateTime.Now.ToString("HH")) - hour;
            min1 = Convert.ToInt32(DateTime.Now.ToString("mm")) - min;
            sec1 = Convert.ToInt32(DateTime.Now.ToString("ss")) - sec;
           
            //var streamBuffer = File.ReadAllBytes(filePath);

            double peak = 100;
 
            //for (var i = 44; i < streambuffer.length; i = i + 2)
            //{
            //    var sample = bitconverter.toint16(streambuffer, i);

            //    if (sample > peak)
            //        peak = sample;
            //    else if (sample < -peak)
            //        peak = -sample;
            //}
            //soundBlock.Text = "최고 크기는 : " + peak;

            String name = "&name=" +nowUser;
            String pose = "&pose=" + dichuck +","+ SP1 + "," + SP2 + "," + SP3 + "," + SP4 + "," + SP5 + "," + SP6;
            String time = "&time=" + hour1 + "," + min1 + "," + sec1;
            String maxsound = "&max=" + peak;

            dataParams.Append(name);
            dataParams.Append(pose);
            dataParams.Append(time);
            dataParams.Append(maxsound);

            String callUrl = "http://localhost:3333";

            HttpWebRequest httpWebRequest = (HttpWebRequest)WebRequest.Create(callUrl);
            // 인코딩 UTF-8

            byte[] sendData = UTF8Encoding.UTF8.GetBytes(dataParams.ToString());
            httpWebRequest.ContentType = "application/x-www-form-urlencoded; charset=UTF-8";
            httpWebRequest.Method = "POST";
            httpWebRequest.ContentLength = sendData.Length;
            Stream requestStream = httpWebRequest.GetRequestStream();
            requestStream.Write(sendData, 0, sendData.Length);
            requestStream.Close();

            dataParams.Clear();
            //S3 FILE UPLOAD
            //{
            //    try
            //    {
            //        TransferUtility fileTransferUtility = new
            //            TransferUtility(new AmazonS3Client(Amazon.RegionEndpoint.APNortheast2));

            //        1.Upload a file, file name is used as the object key name.
            //      fileTransferUtility.Upload(filePath, existingBucketName);

            //    }
            //    catch (AmazonS3Exception s3Exception)
            //    {
            //        Console.WriteLine(s3Exception.Message,
            //                          s3Exception.InnerException);
            //    }
            //}
            dichuck = 0;
            sleepAttitude.Text = "";
            testboxx.Text = "";
            sleepAttitudeNumber.Text = "";
            ativityBlock.Text = "";
        }

       
        string _recordingFileName2;
        string __;
        string sleepStart;
        private void Recording()
        {
            //ID hwd = new ID();
            string ee = "d";
            sleepStart = DateTime.Now.ToString();
            __ = DateTime.Now.ToString("yyyy-MM-dd-HH-mm-ss_") + ee;//hwd.CPU();
            _recordingFileName = userName.Text + ".mp4";
            _recordingFileName2 = __;

            hour = Convert.ToInt32(DateTime.Now.ToString("HH"));
            min = Convert.ToInt32(DateTime.Now.ToString("mm"));
            sec = Convert.ToInt32(DateTime.Now.ToString("ss"));

            Thread thread = new Thread(new ThreadStart(RecordKinectAudio));
            thread.Priority = ThreadPriority.Highest;
            thread.Start();

        }
        private void Stoping()
        {

            IsRecording = false;

            KinectSensor.KinectSensors[0].AudioSource.Stop();
            /*
            filepath = "c:/users/user/documents/카카오톡 받은 파일/끄치여/finalkinect/bin/x86/debug/" + _recordingfilename;
            filepath2 = "c:/users/dev/movi/public/assets/" + _recordingfilename;
            file.move(filepath, filepath2);
            */
        }

        private void userName_TextChanged(object sender, TextChangedEventArgs e)
        {

        }
    }
}
