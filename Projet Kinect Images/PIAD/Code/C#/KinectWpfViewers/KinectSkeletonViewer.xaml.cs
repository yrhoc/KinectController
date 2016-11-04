//------------------------------------------------------------------------------
// <copyright file="KinectSkeletonViewer.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.WpfViewers
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Media;
    using System.Windows.Shapes;
    using Microsoft.Kinect;
    using MySql.Data.MySqlClient;
    
    public enum ImageType
    {
        Color,
        Depth,
    }

    internal enum TrackingMode
    {
        DefaultSystemTracking,
        Closest1Player,
        Closest2Player,
        Sticky1Player,
        Sticky2Player,
        MostActive1Player,
        MostActive2Player
    }

    /// <summary>
    /// Interaction logic for KinectSkeletonViewer.xaml
    /// </summary>
    public partial class KinectSkeletonViewer : ImageViewer, INotifyPropertyChanged
    {
        private const float ActivityFalloff = 0.98f;
        private readonly List<ActivityWatcher> recentActivity = new List<ActivityWatcher>();
        private readonly List<int> activeList = new List<int>();
        private List<KinectSkeleton> skeletonCanvases;
        private List<Dictionary<JointType, JointMapping>> jointMappings = new List<Dictionary<JointType, JointMapping>>();
        private Skeleton[] skeletonData;
        
        // ***PIAD
        private bool enregistrement=false;
        private DateTime decompte;
        private bool firstTracked = false;
        private Label labelDecompte;
        private Dictionary<int, Dictionary<JointType, SkeletonPoint>> enregistrementSquel;
        int nb_iterations;

        public DateTime Decompte
        {
            get
            {
                return decompte;
            }
        }

        private String[] dbconfig = PIADLibrary.PIADLib.LoadDatabase();
        // PIAD***

        public KinectSkeletonViewer()
        {
            InitializeComponent();
            this.ShowJoints = true;
            this.ShowBones = true;
            this.ShowCenter = true;
        }

        public bool ShowBones { get; set; }

        public bool ShowJoints { get; set; }

        public bool ShowCenter { get; set; }

        public ImageType ImageType { get; set; }

        internal TrackingMode TrackingMode { get; set; }

        public void HideAllSkeletons()
        {
            if (this.skeletonCanvases != null)
            {
                foreach (KinectSkeleton skeletonCanvas in this.skeletonCanvases)
                {
                    skeletonCanvas.Reset();
                }
            }
        }

        protected override void OnKinectChanged(KinectSensor oldKinectSensor, KinectSensor newKinectSensor)
        {
            if (oldKinectSensor != null)
            {
                oldKinectSensor.AllFramesReady -= this.KinectAllFramesReady;
                this.HideAllSkeletons();
            }

            if (newKinectSensor != null && newKinectSensor.Status == KinectStatus.Connected)
            {
                newKinectSensor.AllFramesReady += this.KinectAllFramesReady;
            }

        }

        private void KinectAllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            // Have we already been "shut down" by the user of this viewer, 
            // or has the SkeletonStream been disabled since this event was posted?
            if ((this.Kinect == null) || !((KinectSensor)sender).SkeletonStream.IsEnabled)
            {
                return;
            }

            bool haveSkeletonData = false;

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    if (this.skeletonCanvases == null)
                    {
                        this.CreateListOfSkeletonCanvases();
                    }

                    if ((this.skeletonData == null) || (this.skeletonData.Length != skeletonFrame.SkeletonArrayLength))
                    {
                        this.skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    }

                    skeletonFrame.CopySkeletonDataTo(this.skeletonData);

                    haveSkeletonData = true;
                }
            }

            if (haveSkeletonData)
            {
                using (DepthImageFrame depthImageFrame = e.OpenDepthImageFrame())
                {
                    if (depthImageFrame != null)
                    {
                        int trackedSkeletons = 0;

                        foreach (Skeleton skeleton in this.skeletonData)
                        {
                            Dictionary<JointType, JointMapping> jointMapping = this.jointMappings[trackedSkeletons];
                            jointMapping.Clear();

                            KinectSkeleton skeletonCanvas = this.skeletonCanvases[trackedSkeletons++];
                            skeletonCanvas.ShowBones = this.ShowBones;
                            skeletonCanvas.ShowJoints = this.ShowJoints;
                            skeletonCanvas.ShowCenter = this.ShowCenter;

                            // ***PIAD
                            if (this.enregistrement && skeleton.TrackingState.Equals(SkeletonTrackingState.Tracked) && !firstTracked)
                            {
                                firstTracked = true;
                                decompte = DateTime.Now.AddSeconds(5);
                                enregistrementSquel = new Dictionary<int, Dictionary<JointType, SkeletonPoint>>();
                                nb_iterations = 1;
                            }

                            if (firstTracked && decompte > DateTime.Now)
                                this.labelDecompte.Content = (decompte - DateTime.Now).Seconds;

                            //if (this.enregistrement && firstTracked && skeleton.TrackingState.Equals(SkeletonTrackingState.Tracked) && decompte <= DateTime.Now)
                            //{
                            //    this.labelDecompte.Content = "Enregistrement en cours...";
                            //    //System.Console.WriteLine(skeleton.Joints[JointType.Head].Position.X);
                            //    //enregistrementSquel.Add(skeleton);
                            //}
                            // PIAD***

                            // Transform the data into the correct space
                            // For each joint, we determine the exact X/Y coordinates for the target view
                            Dictionary<JointType, SkeletonPoint> temp = new Dictionary<JointType,SkeletonPoint>();
                            foreach (Joint joint in skeleton.Joints)
                            {
                                temp.Add(joint.JointType, joint.Position);

                                Point mappedPoint = this.GetPosition2DLocation(depthImageFrame, joint.Position);
                                jointMapping[joint.JointType] = new JointMapping
                                    {
                                        Joint = joint, 
                                        MappedPoint = mappedPoint
                                    };
                            }

                            if (this.enregistrement && firstTracked && skeleton.TrackingState.Equals(SkeletonTrackingState.Tracked) && decompte <= DateTime.Now)
                            {
                                this.labelDecompte.Content = "Enregistrement en cours...";
                                enregistrementSquel.Add(nb_iterations, temp);
                                nb_iterations++;
                                //System.Console.WriteLine(skeleton.Joints[JointType.Head].Position.X);
                                //enregistrementSquel.Add(skeleton);
                            }

                            // Look up the center point
                            Point centerPoint = this.GetPosition2DLocation(depthImageFrame, skeleton.Position);

                            // Scale the skeleton thickness
                            // 1.0 is the desired size at 640 width
                            double scale = this.RenderSize.Width / 640;

                            skeletonCanvas.RefreshSkeleton(skeleton, jointMapping, centerPoint, scale);
                        }

                        if (ImageType == ImageType.Depth)
                        {
                            this.ChooseTrackedSkeletons(this.skeletonData);
                        }
                    }
                }
            }
        }

        private Point GetPosition2DLocation(DepthImageFrame depthFrame, SkeletonPoint skeletonPoint)
        {
            DepthImagePoint depthPoint = depthFrame.MapFromSkeletonPoint(skeletonPoint);

            switch (ImageType)
            {
                case ImageType.Color:
                    ColorImagePoint colorPoint = depthFrame.MapToColorImagePoint(depthPoint.X, depthPoint.Y, this.Kinect.ColorStream.Format);

                    // map back to skeleton.Width & skeleton.Height
                    return new Point(
                        (int)(this.RenderSize.Width * colorPoint.X / this.Kinect.ColorStream.FrameWidth),
                        (int)(this.RenderSize.Height * colorPoint.Y / this.Kinect.ColorStream.FrameHeight));
                case ImageType.Depth:
                    return new Point(
                        (int)(this.RenderSize.Width * depthPoint.X / depthFrame.Width),
                        (int)(this.RenderSize.Height * depthPoint.Y / depthFrame.Height));
                default:
                    throw new ArgumentOutOfRangeException("ImageType was a not expected value: " + ImageType.ToString());
            }
        }

        private void CreateListOfSkeletonCanvases()
        {
            this.skeletonCanvases = new List<KinectSkeleton>
                {
                    this.skeletonCanvas1,
                    this.skeletonCanvas2,
                    this.skeletonCanvas3,
                    this.skeletonCanvas4,
                    this.skeletonCanvas5,
                    this.skeletonCanvas6
                };

            this.skeletonCanvases.ForEach(s => this.jointMappings.Add(new Dictionary<JointType, JointMapping>()));
        }

        // NOTE: The ChooseTrackedSkeletons part of the KinectSkeletonViewer would be useful
        // separate from the SkeletonViewer.
        private void ChooseTrackedSkeletons(IEnumerable<Skeleton> skeletonDataValue)
        {
            switch (TrackingMode)
            {
                case TrackingMode.Closest1Player:
                    this.ChooseClosestSkeletons(skeletonDataValue, 1);
                    break;
                case TrackingMode.Closest2Player:
                    this.ChooseClosestSkeletons(skeletonDataValue, 2);
                    break;
                case TrackingMode.Sticky1Player:
                    this.ChooseOldestSkeletons(skeletonDataValue, 1);
                    break;
                case TrackingMode.Sticky2Player:
                    this.ChooseOldestSkeletons(skeletonDataValue, 2);
                    break;
                case TrackingMode.MostActive1Player:
                    this.ChooseMostActiveSkeletons(skeletonDataValue, 1);
                    break;
                case TrackingMode.MostActive2Player:
                    this.ChooseMostActiveSkeletons(skeletonDataValue, 2);
                    break;
            }
        }

        private void ChooseClosestSkeletons(IEnumerable<Skeleton> skeletonDataValue, int count)
        {
            SortedList<float, int> depthSorted = new SortedList<float, int>();

            foreach (Skeleton s in skeletonDataValue)
            {
                if (s.TrackingState != SkeletonTrackingState.NotTracked)
                {
                    float valueZ = s.Position.Z;
                    while (depthSorted.ContainsKey(valueZ))
                    {
                        valueZ += 0.0001f;
                    }

                    depthSorted.Add(valueZ, s.TrackingId);
                }
            }

            this.ChooseSkeletonsFromList(depthSorted.Values, count);
        }

        private void ChooseOldestSkeletons(IEnumerable<Skeleton> skeletonDataValue, int count)
        {
            List<int> newList = new List<int>();
            
            foreach (Skeleton s in skeletonDataValue)
            {
                if (s.TrackingState != SkeletonTrackingState.NotTracked)
                {
                    newList.Add(s.TrackingId);
                }
            }

            // Remove all elements from the active list that are not currently present
            this.activeList.RemoveAll(k => !newList.Contains(k));

            // Add all elements that aren't already in the activeList
            this.activeList.AddRange(newList.FindAll(k => !this.activeList.Contains(k)));

            this.ChooseSkeletonsFromList(this.activeList, count);
        }

        private void ChooseMostActiveSkeletons(IEnumerable<Skeleton> skeletonDataValue, int count)
        {
            foreach (ActivityWatcher watcher in this.recentActivity)
            {
                watcher.NewPass();
            }

            foreach (Skeleton s in skeletonDataValue)
            {
                if (s.TrackingState != SkeletonTrackingState.NotTracked)
                {
                    ActivityWatcher watcher = this.recentActivity.Find(w => w.TrackingId == s.TrackingId);
                    if (watcher != null)
                    {
                        watcher.Update(s);
                    }
                    else
                    {
                        this.recentActivity.Add(new ActivityWatcher(s));
                    }
                }
            }

            // Remove any skeletons that are gone
            this.recentActivity.RemoveAll(aw => !aw.Updated);

            this.recentActivity.Sort();
            this.ChooseSkeletonsFromList(this.recentActivity.ConvertAll(f => f.TrackingId), count);
        }

        private void ChooseSkeletonsFromList(IList<int> list, int max)
        {
            if (this.Kinect.SkeletonStream.IsEnabled)
            {
                int argCount = Math.Min(list.Count, max);

                if (argCount == 0)
                {
                    this.Kinect.SkeletonStream.ChooseSkeletons();
                }

                if (argCount == 1)
                {
                    this.Kinect.SkeletonStream.ChooseSkeletons(list[0]);
                }

                if (argCount >= 2)
                {
                    this.Kinect.SkeletonStream.ChooseSkeletons(list[0], list[1]);
                }
            }
        }

        // ***PIAD
        public void activerEnregistrement(Label decompte, string id_user, string id_geste)
        {
            this.enregistrement = !this.enregistrement;
            this.labelDecompte = decompte;
            this.firstTracked = false;
            if (decompte == null)
            {
                //create a MySQL connection with a query string
                MySqlConnection connection = new MySqlConnection("server=" + dbconfig[0] + ";database=" + dbconfig[1] + ";uid=" + dbconfig[2] + ";password=" + dbconfig[3]);

                MySqlCommand commande = connection.CreateCommand();


                int id_sequence = 0;

                commande.CommandText = "select max(id_sequence) as max from gestes_utilisateurs;";

                try
                {
                    //open the connection
                    connection.Open();
                    MySqlDataReader reader = commande.ExecuteReader();
                    reader.Read();
                    id_sequence = reader["max"].ToString() == "" ? 1 : (int)reader["max"] + 1;
                    connection.Close();
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }

                commande.CommandText = "";

                foreach (KeyValuePair<int, Dictionary<JointType, SkeletonPoint>> s in enregistrementSquel)
                {
                    commande.CommandText += "insert into squelettes (id_sequence,nb_iterations,hip_center_x,hip_center_y,hip_center_z,spine_x,spine_y,spine_z,shoulder_center_x,"
                                          + "shoulder_center_y,shoulder_center_z,head_x,head_y,head_z,shoulder_left_x,shoulder_left_y,shoulder_left_z,elbow_left_x,elbow_left_y,"
                                          + "elbow_left_z,wrist_left_x,wrist_left_y,wrist_left_z,hand_left_x,hand_left_y,hand_left_z,shoulder_right_x,shoulder_right_y,shoulder_right_z,"
                                          + "elbow_right_x,elbow_right_y,elbow_right_z,wrist_right_x,wrist_right_y,wrist_right_z,hand_right_x,hand_right_y,hand_right_z,hip_left_x,"
                                          + "hip_left_y,hip_left_z,knee_left_x,knee_left_y,knee_left_z,ankle_left_x,ankle_left_y,ankle_left_z,foot_left_x,foot_left_y,foot_left_z,"
                                          + "hip_right_x,hip_right_y,hip_right_z,knee_right_x,knee_right_y,knee_right_z,ankle_right_x,ankle_right_y,ankle_right_z,foot_right_x,"
                                          + "foot_right_y,foot_right_z)";
                    commande.CommandText += " values (" + id_sequence + "," + s.Key;

                    foreach (KeyValuePair<JointType, SkeletonPoint> joint in s.Value)
                    {
                        commande.CommandText += ",'" + joint.Value.X.ToString() + "'";
                        commande.CommandText += ",'" + joint.Value.Y.ToString() + "'";
                        if(joint.Value.Z.ToString() == "0")
                            commande.CommandText += ",'0,000001'";
                        else
                            commande.CommandText += ",'" + joint.Value.Z.ToString() + "'";
                    }
                    commande.CommandText += ");";
                    //System.IO.File.WriteAllText(@"testtt.txt", commande.CommandText);
                }

                commande.CommandText += "insert into gestes_utilisateurs (id_user,id_geste,id_sequence,type) values(" + id_user + "," + id_geste + "," + id_sequence + ",'brut');";
                //System.Console.WriteLine(commande.CommandText);

                try
                {
                    //open the connection
                    connection.Open();
                    commande.ExecuteNonQuery();
                    connection.Close();
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }
            }
        }
        // PIAD***

        private class ActivityWatcher : IComparable<ActivityWatcher>
        {
            private float activityLevel;
            private SkeletonPoint previousPosition;
            private SkeletonPoint previousDelta;

            internal ActivityWatcher(Skeleton s)
            {
                this.activityLevel = 0.0f;
                this.TrackingId = s.TrackingId;
                this.Updated = true;
                this.previousPosition = s.Position;
                this.previousDelta = new SkeletonPoint();
            }

            internal int TrackingId { get; private set; }

            internal bool Updated { get; private set; }

            public int CompareTo(ActivityWatcher other)
            {
                // Use the existing CompareTo on float, but reverse the arguments,
                // since we wish to have larger activityLevels sort ahead of smaller values.
                return other.activityLevel.CompareTo(this.activityLevel);
            }

            internal void NewPass()
            {
                this.Updated = false;
            }

            internal void Update(Skeleton s)
            {
                SkeletonPoint newPosition = s.Position;
                SkeletonPoint newDelta = new SkeletonPoint
                    {
                        X = newPosition.X - this.previousPosition.X,
                        Y = newPosition.Y - this.previousPosition.Y,
                        Z = newPosition.Z - this.previousPosition.Z
                    };

                SkeletonPoint deltaV = new SkeletonPoint
                    {
                        X = newDelta.X - this.previousDelta.X,
                        Y = newDelta.Y - this.previousDelta.Y,
                        Z = newDelta.Z - this.previousDelta.Z
                    };

                this.previousPosition = newPosition;
                this.previousDelta = newDelta;

                float deltaVLengthSquared = (deltaV.X * deltaV.X) + (deltaV.Y * deltaV.Y) + (deltaV.Z * deltaV.Z);
                float deltaVLength = (float)Math.Sqrt(deltaVLengthSquared);

                this.activityLevel = this.activityLevel * ActivityFalloff;
                this.activityLevel += deltaVLength;

                this.Updated = true;
            }
        }
    }
}
