using System;
using System.IO;

using System.Linq;
using System.Windows;

using System.Drawing;

using System.Windows.Input;
using TelloLib;
using Emgu.CV; // the mom
using Emgu.CV.Aruco; // the hero
using Emgu.CV.CvEnum; // the book
using Emgu.CV.Structure; // the storage
using Emgu.CV.Util; // the side kick

using System.Threading;
using System.Threading.Tasks;
using WebEye.Controls.Wpf.StreamPlayerControl;
using System.Windows.Media;
using System.Windows.Controls;
using System.Windows.Input;

using System.Windows.Media.Imaging;
using ProtoBuf;
using System.IO;
using System.IO.Pipes;
using NetworkCommsDotNet;

using System.Net;
using System.Net.Sockets;
using System.Text;

using System.Diagnostics;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Drawing.Imaging;

namespace DroneHQ
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 


    public partial class MainWindow : System.Windows.Window 
    {
       public static StreamPlayerControl vision_player  = new StreamPlayerControl();
       ImageConverter converter = new ImageConverter();
       static float sensitivity = 0.5f;
        static int Transmission_falg = 0;

        




        /*  static void PrintArucoBoard(GridBoard ArucoBoard, int markersX = 1, int markersY = 1, int markersLength = 80, int markersSeparation = 30)
          {
              // Size of the border of a marker in bits
              int borderBits = 1;

              // Draw the board on a cv::Mat
              System.Drawing.Size imageSize = new System.Drawing.Size();
              Mat boardImage = new Mat();
              imageSize.Width = markersX * (markersLength + markersSeparation) - markersSeparation + 2 * markersSeparation;
              imageSize.Height = markersY * (markersLength + markersSeparation) - markersSeparation + 2 * markersSeparation;
              // ArucoBoard.Draw(imageSize, boardImage, markersSeparation, borderBits);

              // Save the image
              //  boardImage.Bitmap.Save("D:/arucoboard.png");
          }*/
        //VideoCapture capture;
        private static System.DateTime last_time;//for connection timeouts.


        int msec = DateTime.Now.Millisecond;
        bool flag = false;
        bool start_flag = false;
        byte[] my_data;
 
        public PointF center = new PointF(0, 0);
        public static double altitude_value = 0;
        Emgu.CV.Mat rmat = new Emgu.CV.Mat();
        int recv=0;
        double yaw=0, yaw_temp=0;
        double yaw_send = 0;
        double yaw_sum = 0;

        float last_X, last_Y;
        float cutoff_freq = 0.4f;
        float yaw_cutoff_freq = 0.05f;
        float rc_filter, yaw_rc_filter;
        double last_yaw_send =0 ;

        float roll_point = 0.0f;
        float pith_point = 0.0f;
        int detected_id;
        float final_x_point_sum = 0;
        float final_y_point_sum = 0;
        double Altitude_Point = 0;
        double Altitude_Point_sum = 0;



        double[] x = {  0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350,
                        0,25,50,75,100,125,150 ,175,200,225,250,275,300  ,325 ,350 };


        double[] y = {  0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                        25 ,25 ,25 ,25 ,25 ,25 ,25 ,25 ,25 ,25 ,25 ,25 ,25 ,25 ,25 ,
                        50 ,50 ,50 ,50 ,50 ,50 ,50 ,50 ,50 ,50 ,50 ,50 ,50 ,50 ,50 ,
                        75 ,75 ,75 ,75 ,75 ,75 ,75 ,75 ,75 ,75 ,75 ,75 ,75 ,75 ,75 ,
                        100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,
                        125,125,125,125,125,125,125,125,125,125,125,125,125,125,125,
                        150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,
                        175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,
                        200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,
                        225,225,225,225,225,225,225,225,225,225,225,225,225,225,225,
                        250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,
                        275,275,275,275,275,275,275,275,275,275,275,275,275,275,275,
                        300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,
                        325,325,325,325,325,325,325,325,325,325,325,325,325,325,325,
                        350,350,350,350,350,350,350,350,350,350,350,350,350,350,350
        };







        TypeConverter tc = TypeDescriptor.GetConverter(typeof(Bitmap));

        Bitmap bitmap1;
        Bitmap currentFrame;





        private static CancellationTokenSource cancelTokens = new CancellationTokenSource();//used to cancel listeners
      //  private IOutputArray rvecs;

        public static StreamPlayerControl Vision_player { get => vision_player; set => vision_player = value; }
        //public IOutputArray Rvecs { get => rvecs; set => rvecs = value; }
        //public IOutputArray Tvecs { get; set; }
        public IOutputArray RecoveredIdxs { get; set; }

        public MainWindow()
        {
            rc_filter = 1.0f / (2.0f * 3.14f * cutoff_freq);
            yaw_rc_filter = 1.0f / (2.0f * 3.14f * yaw_cutoff_freq);
            //Thread thread = new Thread(new ThreadStart(WorkThreadFunction));
            //thread.Start();

            Thread TCPthread = new Thread(new ThreadStart(TCPThreadFunction));
            //TCPthread.Start();



        BackgroundWorker bw = new BackgroundWorker();
            if (bw.IsBusy != true)
            {
                bw.RunWorkerAsync();

            }



            bw.DoWork += bw_DoWork;
         
            bw.WorkerReportsProgress = true;
            bw.WorkerSupportsCancellation = true; //Allow for the process to be cancelled


            BackgroundWorker tcpbw = new BackgroundWorker();
            if (tcpbw.IsBusy != true)
            {
                tcpbw.RunWorkerAsync();

            }



            tcpbw.DoWork += tcpbw_DoWork;

            tcpbw.WorkerReportsProgress = true;
            tcpbw.WorkerSupportsCancellation = true; //Allow for the process to be cancelled






            InitializeComponent();




            // capture = new VideoCapture(0);
            //vision_player = player;

            EventManager.RegisterClassHandler(typeof(System.Windows.Window), Keyboard.KeyDownEvent, new KeyEventHandler(keyDown), true);

            Tello.startConnecting();


            Tello.onConnection += (Tello.ConnectionState newState) =>
            {
                if (newState != Tello.ConnectionState.Connected)
                {
                }
                if (newState == Tello.ConnectionState.Connected)
                {

                    Console.WriteLine("ding ding ");



                }
                Console.WriteLine("Tello " + newState.ToString());
            };

            var logPath = "logs/";
            System.IO.Directory.CreateDirectory(Path.Combine("../", logPath));
            var logStartTime = DateTime.Now;
            var logFilePath = Path.Combine("../", logPath + logStartTime.ToString("yyyy-dd-M--HH-mm-ss") + ".csv");

           // #region Initialize and save Aruco dictionary and gridboard








            Tello.onUpdate += (cmdId) =>
            {



                if (cmdId == 86)//ac update
                {

                    // Console.WriteLine("loop is working-1");

                    Transmission_falg = Tello.Transmission_falg_tello;



        //write update to log.
        var elapsed = DateTime.Now - logStartTime;
                   // Console.WriteLine("elapsed time: " + elapsed);

                    File.AppendAllText(logFilePath, elapsed.ToString(@"mm\:ss\:ff\,") + Tello.state.getLogLine());
                    this.Dispatcher.Invoke(() =>
                    {
                        batpercent.Content = "Battery: " + Tello.state.batteryPercentage + "%";
                        altitude.Content = "location-X: " + center.X ;
                        speed.Content = "location-Y: " + center.Y;
                        wifi.Content = "WiFi: " + Tello.state.wifiStrength.ToString();
                        disturbance.Content = "Attenuation: " + Tello.state.wifiDisturb + "dB";
                        batteryV.Content = "Battery Voltage: " + Tello.state.droneBatteryLeft + "mV";
                       // sensi.Content = recv.ToString(); 

                    });
                }
                // }
            };
           // var videoClient_imageprocessing = UdpUser.ConnectTo("127.0.0.2", 7039);

            var videoClient = UdpUser.ConnectTo("127.0.0.1", 7070);




            //subscribe to Tello video data
            Tello.onVideoData += (byte[] data) =>
            {
                try
                {
                    my_data = data;


                    this.Dispatcher.Invoke(() =>
                    {

                        videoClient.Send(data.Skip(2).ToArray());//Skip 2 byte header and send to ffplay. 
   
                        flag = true;

                    });
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error in Video Feed: " + ex.ToString());
                }
            };
            

        }


        void keyDown(object sender, KeyEventArgs e)
        {
            float[] axis = new float[] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
            switch (e.Key)
            {
                case Key.W:
                    {
                        axis[3] = sensitivity;
                    }
                    break;
                case Key.A:
                    {
                        axis[2] = -sensitivity;
                    }
                    break;
                case Key.S:
                    {
                        axis[3] = -sensitivity;
                    }
                    break;
                case Key.D:
                    {
                        axis[2] = sensitivity;
                    }
                    break;
                case Key.Q:
                    {
                        axis[0] = sensitivity;
                    }
                    break;
                case Key.E:
                    {
                        axis[0] = -sensitivity;
                    }
                    break;
                case Key.Z:
                    {
                        axis[1] = sensitivity;
                    }
                    break;
                case Key.X:
                    {
                        axis[1] = -sensitivity;
                    }
                    break;
                case Key.D1:
                    {
                        if (sensitivity + 0.25f <= 1f)
                            sensitivity += 0.25f;
                    }
                    break;
                case Key.D2:
                    {
                        if (sensitivity - 0.25f >= 0.0f)
                            sensitivity -= 0.25f;
                    }
                    break;

                case Key.Tab:
                    {
                        if (Tello.connected)
                        {
                            if (Tello.state.flying)
                            {
                                Tello.land();
                            }
                            else
                            {
                                Tello.takeOff();
                            }
                        }
                    }
                    break;
            }
            Tello.controllerState.setAxis(axis[0], axis[1], axis[2], axis[3]);
        }



        private void StreamPlayerControl_Loaded(object sender, RoutedEventArgs e)
        {

            player.StartPlay(new Uri("udp://127.0.0.1:7070"));

            vision_player = player;


        }



        private void TCPThreadFunction()
        {

        }
        private void tcpbw_DoWork(object sender, DoWorkEventArgs e) { 


            
           //  vision_player.StartPlay(new Uri("udp://127.0.0.1:7038"));

            while (true)
            {
                byte[] data = new byte[1024];
                byte[] mydata = new byte[1024];

                IPEndPoint ipep = new IPEndPoint(IPAddress.Any,
                                       9070);

                Socket newsock = new
                Socket(AddressFamily.InterNetwork,SocketType.Stream, ProtocolType.Tcp);

                newsock.Bind(ipep);
                newsock.Listen(10);
                Console.WriteLine("Waiting for a client...");
                Socket client = newsock.Accept();
                IPEndPoint clientep = (IPEndPoint)client.RemoteEndPoint;
                Console.WriteLine("Connected with {0} at port {1}", clientep.Address, clientep.Port);


                string welcome = "Welcome to my test server";
                data = Encoding.ASCII.GetBytes(welcome);
                client.Send(data, data.Length, SocketFlags.None);
                while (true)
                {


                    try
                    {
                        client.Send(data, data.Length,
                      SocketFlags.None);
                        data = new byte[1024];
                        mydata = new byte[1024];
                        recv = client.Receive(data);

                        this.Dispatcher.Invoke(() =>
                        {
                            //sensi.Content = Encoding.ASCII.GetString(data, 0, recv);
                            sensi.Content = ((int)(yaw_send)).ToString();
                            //sensi.Content = ((int)(altitude_value)).ToString();


                            


                        });

                        //     Console.WriteLine("recieved message in server" +  Encoding.ASCII.GetString(data, 0, recv));

                        char[] ms = new char[1];
                        ms[0] = 'm';



                        var LoopDt = System.DateTime.Now - last_time;

                        last_time = System.DateTime.Now;
                      //  Console.WriteLine("Dt in tello opencv " + LoopDt.Milliseconds);

                        center.X = last_X + (((float)(LoopDt.Milliseconds) / ((float)(LoopDt.Milliseconds) + rc_filter)) * (center.X - last_X));
                        last_X = center.X;

                        center.Y = last_Y + (((float)(LoopDt.Milliseconds) / ((float)(LoopDt.Milliseconds) + rc_filter)) * (center.Y - last_Y));
                        last_Y = center.Y;


                        /* if ( (last_yaw_send > 0 && last_yaw_send < 40) && yaw_send >320) 
                          {
                              if (yaw_send - 360 - last_yaw_send < -40 || yaw_send - last_yaw_send > 40) yaw_send = last_yaw_send;
                          }
                         else if ((last_yaw_send > 320) && (yaw_send >=0 && yaw_send < 40)){

                              if (last_yaw_send - yaw_send - 360  < -40 || yaw_send - last_yaw_send > 40) yaw_send = last_yaw_send;
                          }
                          else
                          {
                              if (yaw_send - last_yaw_send < -40 || yaw_send - last_yaw_send > 40) yaw_send = last_yaw_send;
                          }*/


                        yaw_send = last_yaw_send + (((float)(LoopDt.Milliseconds) / ((float)(LoopDt.Milliseconds) + yaw_rc_filter)) * (yaw_send - last_yaw_send));
                        //yaw_send = last_yaw_send + (0.75* (last_yaw_send - yaw_send ));

                        last_yaw_send = yaw_send;

                        // self.x_point = self.x_point_last + (( self.DT/(self.DT + self.RC))*(self.x_point - self.x_point_last))
                        Transmission_falg = Tello.Transmission_falg_tello;


                        int var = (int)(center.Y);
                        int var1 = (int)(center.X);
                        int var2 = (int)(yaw_send);
                        int var3 = (int)(Altitude_Point/10);
                        int var4 = Transmission_falg;




                        //  mydata = Encoding.ASCII.GetBytes(var.ToString()+','+var1.GetTypeCode());
                        mydata = Encoding.ASCII.GetBytes(var.ToString() + "," + var1.ToString() + "," + var2.ToString() + "," + var3.ToString() + "," + var4.ToString());

                        //   mydata = BitConverter.GetBytes(var);
                        client.Send(mydata, mydata.Length, SocketFlags.None);
                        //  client.Send(data, recv, SocketFlags.None);


                        System.Threading.Thread.Sleep(10);


                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("tcp thread error:" + ex.Message);
                    }




                }
                Console.WriteLine("Disconnected from {0}",
                                  clientep.Address);
                // client.Close();
                // newsock.Close();
            }

        }


          private void WorkThreadFunction()
          {
            }

        private void bw_DoWork(object sender, DoWorkEventArgs e)
        {
  
         


      //  private void WorkThreadFunction()
      //  {


     


            Console.WriteLine("StreamPlayerControl_Loaded ");

            cancelTokens = new CancellationTokenSource();
            CancellationToken token = cancelTokens.Token;
            int id_length=0;
            int markersX = 15;
            int markersY = 15;
            float markersLength =  90.0f;
            float markersSeparation = 152.0f;
            Dictionary ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.DictArucoOriginal); // bits x bits (per marker) _ number of markers in dict
            GridBoard ArucoBoard = new GridBoard(markersX, markersY, markersLength, markersSeparation, ArucoDict,1);
            // PrintArucoBoard(ArucoBoard, markersX, markersY, markersLength, markersSeparation);
            //  #endregion



            #region Initialize Aruco parameters for markers detection
            DetectorParameters ArucoParameters = new DetectorParameters();
            ArucoParameters = DetectorParameters.GetDefault();
            ArucoParameters.AdaptiveThreshConstant = 7; 

            #endregion

            #region Initialize Camera calibration matrix with distortion coefficients 
            // Calibration done with https://docs.opencv.org/3.4.3/d7/d21/tutorial_interactive_calibration.html
            String cameraConfigurationFile = "D:/MetaOptima/MAVIC AIR/OpenCV and EmguCV/CSharp-Aruco-Tuto-master/cameraParameters.xml";
            Emgu.CV.FileStorage fs = new Emgu.CV.FileStorage(cameraConfigurationFile, Emgu.CV.FileStorage.Mode.Read);
            if (!fs.IsOpened)
            {
                Console.WriteLine("Could not open configuration file#333333333333333333333333333333333333333333333 " + cameraConfigurationFile);
                return;
            }

            Emgu.CV.Mat cameraMatrix = new Emgu.CV.Mat(new System.Drawing.Size(3, 3), DepthType.Cv32F, 1);

            Emgu.CV.Mat distortionMatrix = new Emgu.CV.Mat(1, 5, DepthType.Cv32F, 1);
            //Mat distortionMatrix =    new Mat(3,3);

            fs["cameraMatrix"].ReadMat(cameraMatrix);
            fs["dist_coeffs"].ReadMat(distortionMatrix);
            #endregion
            Console.WriteLine("3" + cameraConfigurationFile);
            Console.WriteLine("cameraMatrix " + cameraMatrix);
            Console.WriteLine("dist_coeffs " + distortionMatrix);



            // using (var file = File.Create("D:/MetaOptima/MAVIC AIR/Windows-SDK-master/Sample Code/DJIWindowsSDKSample_x64/bin/x64/Debug/pass.bin"))
            //  var myfile = File.Create(@"D:\MetaOptima\MAVIC AIR\Windows-SDK-master\Sample Code\DJIWindowsSDKSample_x64\bin\x64\Debug\humaninfo.bin");


            using (var myfile = File.Create(@"D:\MetaOptima\MAVIC AIR\Windows-SDK-master\Sample Code\DJIWindowsSDKSample_x64\bin\x64\Debug\humaninfo.bin"))

            {
              //  Serializer.Serialize(myfile, position);
            }


            int loopCounter = 1;




            while (true)
                {
                    try
                    {
                        this.Dispatcher.Invoke(() =>
                        {

                                if ( start_flag == true)
                                {
                                    flag = false;
                                var bytes = my_data;
                                currentFrame = vision_player.GetCurrentFrame();
                                //VideoCapture capturee = new VideoCapture(); //create a camera capture
                               // Bitmap currentFrame = capturee.QueryFrame().Bitmap; //take a picture
                                //  Console.WriteLine("TASK THREAD");
                                Emgu.CV.Image<Bgr, byte> capture = new Image<Bgr, byte>(currentFrame); //mizane tasvir ro kharab mikone


                                //  capture.Bytes = my_data;

                                // CvInvoke.NamedWindow(windowName);
                                Mat frame = new Mat();
                                Mat filtered_frame = new Mat();

                                frame = capture.Mat;//.QueryFrame();
                                                             
                                 // Console.WriteLine("query ");
                                 #region Detect markers on last retrieved frame
                                 VectorOfInt ids = new VectorOfInt(); // name/id of the detected markers
                                 VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF(); // corners of the detected marker
                                //   Mat corners = new Mat();
                                VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF(); // rejected contours

                               // Emgu.CV.CvInvoke.Threshold(frame, filtered_frame, 150, 255, Emgu.CV.CvEnum.ThresholdType.ToZero);

                                 ArucoInvoke.DetectMarkers(frame, ArucoDict, corners, ids, ArucoParameters, rejected);
                                ArucoInvoke.RefineDetectedMarkers(frame, ArucoBoard, corners, ids, rejected, cameraMatrix, distortionMatrix,5.0f,0.6f,true,RecoveredIdxs, ArucoParameters);

                                #endregion

                                if (ids.Size > 0)
                                {
                                    #region Draw detected markers
                                    ArucoInvoke.DrawDetectedMarkers(frame, corners, ids, new MCvScalar(255, 0, 255));
                                    #endregion

                                    #region Estimate pose for each marker using camera calibration matrix and distortion coefficents
                                    Mat rvecs = new Mat(); // rotation vector
                                    Mat tvecs = new Mat(); // translation vector
                                    ArucoInvoke.EstimatePoseSingleMarkers(corners, markersLength, cameraMatrix, distortionMatrix, rvecs, tvecs);
                                    #endregion

                                    #region Draw 3D orthogonal axis on markers using estimated pose
                                    if (ids.Size > 20) id_length = 20;
                                    else id_length = ids.Size;
                                    yaw_sum = 0;
                                    final_x_point_sum = 0;
                                    final_y_point_sum = 0;
                                    Altitude_Point_sum = 0;

                                    for (int i = 0; i < id_length; i++)
                                    {



                                        using (Mat rvecMat = rvecs.Row(i))
                                        using (Mat tvecMat = tvecs.Row(i))
                                        using (VectorOfDouble rvec = new VectorOfDouble())
                                        using (VectorOfDouble tvec = new VectorOfDouble())
                                        {
                                            Emgu.CV.CvInvoke.Rodrigues(rvecMat, rmat);
                                            var rmat_inter = rmat.T();


                                            double[] values = new double[3];
                                            double[] xyz = new double[3];

                                            rvecMat.CopyTo(values);
                                            tvecMat.CopyTo(xyz);
                                            rvec.Push(values);
                                            tvecMat.CopyTo(values);
                                            tvec.Push(values);
                                            ArucoInvoke.DrawAxis(frame,
                                                                 cameraMatrix,
                                                                 distortionMatrix,
                                                                 rvec,
                                                                 tvec,
                                                                 markersLength * 0.75f);

                                            Emgu.CV.Matrix<Double> rmat_matrix = new Emgu.CV.Matrix<Double>(rmat_inter.Rows, rmat_inter.Cols);
                                            rmat_inter.CopyTo(rmat_matrix);

                                            Emgu.CV.Matrix<Double> tvecs_matrix = new Emgu.CV.Matrix<Double>(tvecs.Rows, tvecs.Cols);
                                            tvecs.CopyTo(tvecs_matrix);


                                            // var roll = Math.Atan2(-rmat_matrix[2, 1], rmat_matrix[2, 2]);
                                            // var pitch = Math.Asin(rmat_matrix[2, 0]);
                                            if (i == id_length-1)
                                            {
                                                yaw = Math.Atan2(-rmat_matrix[1, 0], rmat_matrix[0, 0]);
                                                yaw = -yaw;
                                            }
                                            xyz[0] = -xyz[0];
                                            xyz[1] = -xyz[1];
                                            detected_id = ids[i];


                                            Console.WriteLine("x: " + xyz[0] + "y: " + xyz[1] + "z: " + xyz[2]);//xyz[0] roll or x        xyz[1] pitch or y      xyz[2] z

                                            //final_x_point_sum = final_x_point_sum + (float)(xyz[0] * Math.Cos(yaw)) + (float)(xyz[1] * Math.Sin(yaw)) + (float)(10*x[detected_id]);
                                            //final_y_point_sum = final_y_point_sum + (float)(xyz[1] * Math.Cos(yaw)) - (float)(xyz[0] * Math.Sin(yaw)) + (float)(10*y[detected_id]);
                                            final_x_point_sum = final_x_point_sum + (float)(xyz[0] * Math.Cos(yaw)) + (float)(xyz[1] * Math.Sin(yaw)) + (float)(10 * x[detected_id]);
                                            final_y_point_sum = final_y_point_sum + (float)(xyz[1] * Math.Cos(yaw)) - (float)(xyz[0] * Math.Sin(yaw)) + (float)(10 * y[detected_id]);


                                            Altitude_Point_sum = Altitude_Point_sum + xyz[2];


                                        }

                                        if (yaw < 0) yaw = yaw + (double)(Math.PI * 2);

                                        yaw_send = (57.2958 * yaw);


                                        center.Y = final_y_point_sum / id_length;
                                        center.X = final_x_point_sum / id_length;
                                        Altitude_Point = Altitude_Point_sum/ id_length;


                                    }
                                    #endregion
                                }
                                /*if (ids.Size > 0)
                                 {
                                     #region Draw detected markers
                                     ArucoInvoke.DrawDetectedMarkers(frame, corners, ids, new MCvScalar(255, 0, 255));
                                    #endregion
                                    
                                  //  Emgu.CV.Mat rvecs =   new Emgu.CV.Mat(); // rotation vector
                                 //   Emgu.CV.Mat tvecs = new Emgu.CV.Mat(); // translation vectorv

                                    Matrix<Double> rvecs = new Matrix<Double>(3, 3); //works for single markers
                                    Matrix<Double> tvecs = new Matrix<Double>(1, 4); //works for single markers


                                    //ArucoInvoke.EstimatePoseBoard( corners, ids, ArucoBoard,  cameraMatrix, distortionMatrix, rvecs,  tvecs );
                                    ArucoInvoke.EstimatePoseSingleMarkers(corners, markersLength, cameraMatrix, distortionMatrix, rvecs, tvecs);
                                    //Console.WriteLine("###################################################################################");

                                    Emgu.CV.CvInvoke.Rodrigues(rvecs, rmat);
                                    // Console.WriteLine("###################################################################################");

                                    var rmat_inter = rmat.T();


                                    Emgu.CV.Matrix<Double> rmat_matrix = new Emgu.CV.Matrix<Double>(rmat_inter.Rows, rmat_inter.Cols);
                                     rmat_inter.CopyTo(rmat_matrix);
                                    Emgu.CV.Matrix<Double> tvecs_matrix = new Emgu.CV.Matrix<Double>(tvecs.Rows, tvecs.Cols);
                                     tvecs.CopyTo(tvecs_matrix);
                                    // Console.WriteLine("###################################################################################");


                                    var roll  = Math.Atan2(-rmat_matrix[2, 1], rmat_matrix[2, 2]);
                                    var pitch = Math.Asin(rmat_matrix[2, 0]);
                                    yaw = Math.Atan2(-rmat_matrix[1, 0], rmat_matrix[0, 0]);
                                    yaw = (57.2958 * yaw);
                                    if (yaw < 0) yaw = yaw + 360;
                                    yaw = yaw -90;
                                    if (yaw < 0) yaw = yaw + 360;
                                    yaw_send = yaw;
                                    rmat_matrix = rmat_matrix.Mul(-1);

                                    Matrix<Double> cameraPosition= rmat_matrix.Mul(tvecs_matrix);

                                      Console.WriteLine("feeding");


                                    center.Y = (float)(cameraPosition[0, 0]);
                                    center.X = - (float)(cameraPosition[1, 0]);// + 3750.0f;
                                    altitude_value = cameraPosition[2, 0];
                                    //   Console.WriteLine(cameraPosition[2, 0]);

                                }*/

                                Emgu.CV.CvInvoke.Imshow("Image", frame);
                             //  Emgu.CV.CvInvoke.Imshow("Imagee", frame);


                                Emgu.CV.CvInvoke.WaitKey(1);



                            }

                        });



                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Video image processing receive thread error:" + ex.Message);
                    Debug.WriteLine("Video image processing receive thread error:" + ex.Message);

                }
            }
           // }, token);
        }


        private void ClientSession(Socket clientSocket)
        {
            clientSocket.Send(ASCIIEncoding.ASCII.GetBytes("Hello from server"));
            byte[] b = new byte[255];
           // Console.WriteLine(" receive");

            clientSocket.Receive(b);
          //  sensi.Content = b.ToString();
            Console.WriteLine(" socket is created well");
        }



        private void connect_Click(object sender, RoutedEventArgs e)
        {
            if (connect.Content.ToString() == "Connect")
            {
                Tello.startConnecting();
                Thread thread = new Thread(new ThreadStart(WorkThreadFunction));

                // WorkThreadFunction();
                thread.Start();

                start_flag = true;

                connect.Content = "Disconnect";
             
            }
            else
            {
                connect.Content = "Connect";


            }
        }

        public static double[,] multiply(double[,] A, double[,] B)
        {
            int mA = A.GetLength(0);
            int nA = A.GetLength(1);
            int mB = B.GetLength(0);
            int nB = B.GetLength(1);
            if (nA != mB)
                throw new SystemException("Illegal matrix dimensions.");
            double[,] C = new double[mA, nB];
            for (int i = 0; i < mA; i++)
                for (int j = 0; j < nB; j++)
                    for (int k = 0; k < nA; k++)
                        C[i, j] += A[i, k] * B[k, j];
            return C;
        }



        PointF GetCentroidFromCorner(VectorOfPointF corner)
        {
            PointF center = new PointF(0, 0);
            center.X = (corner[0].X + corner[1].X + corner[2].X + corner[3].X) / 4; //X is on horizontal axis = cols /!\ opencv Mat are row based (y,x) = (i,j), left to right, top to bottom
            center.Y = (corner[0].Y + corner[1].Y + corner[2].Y + corner[3].Y) / 4; //Y is on vertical axis = rows /!\ opencv Mat are row based (y,x) = (i,j), left to right, top to bottom
            return center;
        }

    }
}
