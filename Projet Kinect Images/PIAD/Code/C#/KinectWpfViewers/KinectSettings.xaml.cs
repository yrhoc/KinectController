//------------------------------------------------------------------------------
// <copyright file="KinectSettings.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.WpfViewers
{
    using System;
    using System.Diagnostics;
    using System.Linq;
    using System.Threading;
    using System.Threading.Tasks;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Threading;
    using Microsoft.Kinect;
    using MySql.Data.MySqlClient;

    /// <summary>
    /// Interaction logic for KinectSettings.xaml
    /// </summary>
    internal partial class KinectSettings : UserControl
    {
        private readonly DispatcherTimer debounce = new DispatcherTimer { IsEnabled = false, Interval = TimeSpan.FromMilliseconds(200) };

        private int lastSetSensorAngle = int.MaxValue;
        private bool userUpdate = true;
        private bool backgroundUpdateInProgress;
        private bool enregistrement = false;
        private String[] dbconfig = PIADLibrary.PIADLib.LoadDatabase();

        public KinectSettings(KinectDiagnosticViewer diagViewer)
        {
            this.DiagViewer = diagViewer;
            InitializeComponent();
            this.debounce.Tick += this.DebounceElapsed;

            // PIAD
            comboBoxBase.Items.Add("Sélectionnez une base");
            comboBoxBase.SelectedIndex = 0;

            MySqlConnection connection = new MySqlConnection("server=" + dbconfig[0] + ";database=" + dbconfig[1] + ";uid=" + dbconfig[2] + ";password=" + dbconfig[3]);

            MySqlCommand commande = connection.CreateCommand();

            commande.CommandText = "Select distinct base from gestes order by base;";

            try
            {
                //open the connection
                connection.Open();
                MySqlDataReader reader = commande.ExecuteReader();
                while (reader.Read())
                {
                    comboBoxBase.Items.Add(reader["base"].ToString());
                }

                connection.Close();

            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
            // PIAD
        }

        public KinectDiagnosticViewer DiagViewer { get; set; }

        public KinectSensor Kinect { get; set; }

        private static bool IsSkeletalViewerAvailable
        {
            get { return KinectSensor.KinectSensors.All(k => (!k.IsRunning || !k.SkeletonStream.IsEnabled)); }
        }

        internal void PopulateComboBoxesWithFormatChoices()
        {
            foreach (ColorImageFormat colorImageFormat in Enum.GetValues(typeof(ColorImageFormat)))
            {
                switch (colorImageFormat)
                {
                    case ColorImageFormat.Undefined:
                        break;
                    case ColorImageFormat.RawYuvResolution640x480Fps15:
                    // don't add RawYuv to combobox.
                    // That colorImageFormat works, but needs YUV->RGB conversion code which this sample doesn't have yet.
                        break;
                    default:
                        colorFormats.Items.Add(colorImageFormat);
                        break;
                }
            }

            foreach (DepthImageFormat depthImageFormat in Enum.GetValues(typeof(DepthImageFormat)))
            {
                switch (depthImageFormat)
                {
                    case DepthImageFormat.Undefined:
                        break;
                    default:
                        depthFormats.Items.Add(depthImageFormat);
                        break;
                }
            }

            foreach (TrackingMode trackingMode in Enum.GetValues(typeof(TrackingMode)))
            {
                trackingModes.Items.Add(trackingMode);
            }

            foreach (DepthRange depthRange in Enum.GetValues(typeof(DepthRange)))
            {
                depthRanges.Items.Add(depthRange);
            }

            depthRanges.SelectedIndex = 0;
        }

        internal void UpdateUiElevationAngleFromSensor()
        {
            if (this.Kinect != null)
            {
                this.userUpdate = false;

                // If it's never been set, retrieve the value.
                if (this.lastSetSensorAngle == int.MaxValue)
                {
                    this.lastSetSensorAngle = this.Kinect.ElevationAngle;
                }

                // Use the cache to prevent race conditions with the background thread which may 
                // be in the process of setting this value.
                this.ElevationAngle.Value = this.lastSetSensorAngle;
                this.userUpdate = true;
            }
        }

        private void ColorFormatsSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBox comboBox = sender as ComboBox;
            if (comboBox == null)
            {
                return;
            }

            if (this.Kinect != null && this.Kinect.Status == KinectStatus.Connected && comboBox.SelectedItem != null)
            {
                if (this.Kinect.ColorStream.IsEnabled)
                {
                    this.Kinect.ColorStream.Enable((ColorImageFormat)this.colorFormats.SelectedItem);
                }
            }
        }

        private void DepthFormatsSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBox comboBox = sender as ComboBox;
            if (comboBox == null)
            {
                return;
            }

            if (this.Kinect != null && this.Kinect.Status == KinectStatus.Connected && comboBox.SelectedItem != null)
            {
                if (this.Kinect.DepthStream.IsEnabled)
                {
                    this.Kinect.DepthStream.Enable((DepthImageFormat)this.depthFormats.SelectedItem);
                }
            }
        }

        private void SkeletonsChecked(object sender, RoutedEventArgs e)
        {
            CheckBox checkBox = sender as CheckBox;
            if (checkBox == null)
            {
                return;
            }

            if (this.Kinect != null && this.Kinect.Status == KinectStatus.Connected && checkBox.IsChecked.HasValue)
            {
                this.SetSkeletalTracking(checkBox.IsChecked.Value);
                this.EnableDepthStreamBasedOnDepthOrSkeletonEnabled(this.Kinect.DepthStream, this.depthFormats);
            }
        }

        private void SetSkeletalTracking(bool enable)
        {
            if (enable)
            {
                if (IsSkeletalViewerAvailable)
                {
                    this.Kinect.SkeletonStream.Enable();
                    trackingModes.IsEnabled = true;
                    this.DiagViewer.KinectSkeletonViewerOnColor.Visibility = System.Windows.Visibility.Visible;
                    this.DiagViewer.KinectSkeletonViewerOnDepth.Visibility = System.Windows.Visibility.Visible;
                    SkeletonStreamEnable.IsChecked = true;
                }
                else
                {
                    SkeletonStreamEnable.IsChecked = false;
                }
            }
            else
            {
                this.Kinect.SkeletonStream.Disable();
                trackingModes.IsEnabled = false;

                // To ensure that old skeletons aren't displayed when SkeletonTracking
                // is reenabled, we ask SkeletonViewer to hide them all now.
                this.DiagViewer.KinectSkeletonViewerOnColor.HideAllSkeletons();
                this.DiagViewer.KinectSkeletonViewerOnDepth.HideAllSkeletons();
                this.DiagViewer.KinectSkeletonViewerOnColor.Visibility = System.Windows.Visibility.Hidden;
                this.DiagViewer.KinectSkeletonViewerOnDepth.Visibility = System.Windows.Visibility.Hidden;
            }
        }

        private void TrackingModesSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBox comboBox = sender as ComboBox;
            if (comboBox == null)
            {
                return;
            }

            if (this.Kinect != null && this.Kinect.Status == KinectStatus.Connected && comboBox.SelectedItem != null)
            {
                TrackingMode newMode = (TrackingMode)comboBox.SelectedItem;
                this.Kinect.SkeletonStream.AppChoosesSkeletons = newMode != TrackingMode.DefaultSystemTracking;
                this.DiagViewer.KinectSkeletonViewerOnColor.TrackingMode = newMode;
                this.DiagViewer.KinectSkeletonViewerOnDepth.TrackingMode = newMode;
            }
        }

        private void DepthRangesSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBox comboBox = sender as ComboBox;
            if (comboBox == null)
            {
                return;
            }

            if (this.Kinect != null && this.Kinect.Status == KinectStatus.Connected && comboBox.SelectedItem != null)
            {
                try
                {
                    this.Kinect.DepthStream.Range = (DepthRange)comboBox.SelectedItem;
                }
                catch (InvalidOperationException)
                {
                    comboBox.SelectedIndex = 0;
                    comboBox.Items.RemoveAt(1);
                    comboBox.Items.Add("-- NearMode not supported on this device. See Readme. --");
                }
                catch (InvalidCastException)
                {
                    // they chose the error string, switch back
                    comboBox.SelectedIndex = 0;
                }
            }
        }

        private void ColorStreamEnabled(object sender, RoutedEventArgs e)
        {
            CheckBox checkBox = (CheckBox)sender;
            this.DisplayColumnBasedOnIsChecked(checkBox, 1, 2);
            this.DisplayPanelBasedOnIsChecked(checkBox, this.DiagViewer.colorPanel);
            if (this.Kinect != null)
            {
                this.EnableColorImageStreamBasedOnIsChecked(checkBox, this.Kinect.ColorStream, this.colorFormats);
            }
        }

        private void EnableDepthStreamBasedOnDepthOrSkeletonEnabled(
            DepthImageStream depthImageStream, ComboBox depthFormatsValue)
        {
            if (depthFormatsValue.SelectedItem != null)
            {
                // SkeletonViewer needs depth. So if DepthViewer or SkeletonViewer is enabled, enabled depthStream.
                if ((DepthStreamEnable.IsChecked.HasValue && DepthStreamEnable.IsChecked.Value)
                    || (SkeletonStreamEnable.IsChecked.HasValue && SkeletonStreamEnable.IsChecked.Value))
                {
                    depthImageStream.Enable((DepthImageFormat)depthFormatsValue.SelectedItem);
                }
                else
                {
                    depthImageStream.Disable();
                }
            }
        }

        private void EnableColorImageStreamBasedOnIsChecked(
            CheckBox checkBox, ColorImageStream imageStream, ComboBox colorFormatsValue)
        {
            if (checkBox.IsChecked.HasValue && checkBox.IsChecked.Value)
            {
                imageStream.Enable((ColorImageFormat)colorFormatsValue.SelectedItem);
            }
            else
            {
                imageStream.Disable();
            }
        }

        private void DepthStreamEnabled(object sender, RoutedEventArgs e)
        {
            CheckBox checkBox = (CheckBox)sender;
            this.DisplayColumnBasedOnIsChecked(checkBox, 2, 1);
            this.DisplayPanelBasedOnIsChecked(checkBox, this.DiagViewer.depthPanel);
            if (this.Kinect != null)
            {
                this.EnableDepthStreamBasedOnDepthOrSkeletonEnabled(this.Kinect.DepthStream, this.depthFormats);
            }
        }

        private void DisplayPanelBasedOnIsChecked(CheckBox checkBox, Grid panel)
        {
            // on load of XAML page, panel will be null.
            if (panel == null)
            {
                return;
            }

            if (checkBox.IsChecked.HasValue && checkBox.IsChecked.Value)
            {
                panel.Visibility = Visibility.Visible;
            }
            else
            {
                panel.Visibility = Visibility.Collapsed;
            }
        }

        private void DisplayColumnBasedOnIsChecked(CheckBox checkBox, int column, int stars)
        {
            if (checkBox.IsChecked.HasValue && checkBox.IsChecked.Value)
            {
                this.DiagViewer.LayoutRoot.ColumnDefinitions[column].Width = new GridLength(stars, GridUnitType.Star);
            }
            else
            {
                this.DiagViewer.LayoutRoot.ColumnDefinitions[column].Width = new GridLength(0);
            }
        }

        private void ElevationAngleChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (this.userUpdate)
            {
                this.debounce.Stop();
                this.debounce.Start();
            }
        }

        private void DebounceElapsed(object sender, EventArgs e)
        {
            // The time has elapsed.  We may start it again later.
            this.debounce.Stop();

            int angleToSet = (int)ElevationAngle.Value;

            // Is there an update in progress?
            if (this.backgroundUpdateInProgress)
            {
                // Try again in a few moments.
                this.debounce.Start();
            }
            else
            {
                this.backgroundUpdateInProgress = true;

                Task.Factory.StartNew(
                    () =>
                        {
                            try
                            {
                                // Check for not null and running
                                if ((this.Kinect != null) && this.Kinect.IsRunning)
                                {
                                    // We must wait at least 1 second, and call no more frequently than 15 times every 20 seconds
                                    // So, we wait at least 1350ms afterwards before we set backgroundUpdateInProgress to false.
                                    this.Kinect.ElevationAngle = angleToSet;
                                    this.lastSetSensorAngle = angleToSet;
                                    Thread.Sleep(1350);
                                }
                            }
                            finally
                            {
                                this.backgroundUpdateInProgress = false;
                            }
                        }).ContinueWith(
                            results =>
                                {
                                    // This can happen if the Kinect transitions from Running to not running
                                    // after the check above but before setting the ElevationAngle.
                                    if (results.IsFaulted)
                                    {
                                        var exception = results.Exception;

                                        Debug.WriteLine(
                                            "Set Elevation Task failed with exception " +
                                            exception);
                                    }
                                });
            }
        }

        // ***PIAD
        private string id_geste = "";
       
        private void Enregistrer_Clicked(object sender, RoutedEventArgs e)
        {
            if (Enregistrer.Content.Equals("Enregistrer"))
                Enregistrer.Content = "Arrêter";
            else
            {
                Enregistrer.Content = "Enregistrer";
                Decompte.Content = null;
                if (id_geste != "")
                    this.DiagViewer.KinectSkeletonViewerOnColor.activerEnregistrement(null, DiagViewer.Id_user, id_geste);
                UpdateInfosGestes();
            }
            
            enregistrement = !enregistrement;
            if (this.Kinect != null && this.Kinect.Status == KinectStatus.Connected && enregistrement)
            {
                if(id_geste != "")
                    this.DiagViewer.KinectSkeletonViewerOnColor.activerEnregistrement(this.Decompte, DiagViewer.Id_user, id_geste);
            }
        }

        private void Base_Selected(object sender, SelectionChangedEventArgs e)
        {
            comboBoxGeste.Items.Clear();
            comboBoxGeste.Items.Add("Sélectionnez un geste");
            comboBoxGeste.SelectedIndex = 0;

            MySqlConnection connection = new MySqlConnection("server=" + dbconfig[0] + ";database=" + dbconfig[1] + ";uid=" + dbconfig[2] + ";password=" + dbconfig[3]);

            MySqlCommand commande = connection.CreateCommand();

            commande.CommandText = "Select nom from gestes where base='" + comboBoxBase.SelectedItem.ToString() + "' order by nom;";
          
            try
            {
                //open the connection
                connection.Open();
                MySqlDataReader reader = commande.ExecuteReader();
                while (reader.Read())
                {
                    comboBoxGeste.Items.Add(reader["nom"].ToString());
                }

                connection.Close();

            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
            if(comboBoxBase.SelectedIndex != 0)
                comboBoxGeste.Visibility = Visibility.Visible;
            if (comboBoxBase.SelectedIndex == 0)
                comboBoxGeste.Visibility = Visibility.Hidden;

        }

        private void Geste_Selected(object sender, SelectionChangedEventArgs e)
        {
            if (comboBoxGeste.SelectedIndex != 0 && comboBoxGeste.SelectedItem != null)
            {
                UpdateInfosGestes();
                Enregistrer.IsEnabled = true;
            }
            else
            {
                Enregistrer.IsEnabled = false;
            }
        }

        private void UpdateInfosGestes()
        {
            MySqlConnection connection = new MySqlConnection("server=" + dbconfig[0] + ";database=" + dbconfig[1] + ";uid=" + dbconfig[2] + ";password=" + dbconfig[3]);

            MySqlCommand commande = connection.CreateCommand();

            // Affichage de la description du geste
            commande.CommandText = "Select description, id_geste from gestes where base='" + comboBoxBase.SelectedItem.ToString() + "' and nom='" + comboBoxGeste.SelectedItem.ToString() + "';";

            try
            {
                //open the connection
                connection.Open();
                MySqlDataReader reader = commande.ExecuteReader();
                while (reader.Read())
                {
                    labelDescription.Text = reader["description"].ToString();
                    id_geste = reader["id_geste"].ToString();
                }

                connection.Close();

            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }

            // Affichage du nombre total d'itération pour le geste sélectionné
            commande.CommandText = "select count(id_geste) as count from gestes_utilisateurs where id_geste='" + id_geste + "';";

            try
            {
                //open the connection
                connection.Open();
                MySqlDataReader reader = commande.ExecuteReader();
                while (reader.Read())
                {
                    labelNbGesteTotal.Content = "Itérations total : " + reader["count"].ToString();
                }

                connection.Close();

            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }

            // Affichage du nombre total d'itération pour le geste sélectionné par utilisateur
            commande.CommandText = "select count(id_geste) as count from gestes_utilisateurs where id_geste='" + id_geste + "' and id_user='" + DiagViewer.Id_user + "';";

            try
            {
                //open the connection
                connection.Open();
                MySqlDataReader reader = commande.ExecuteReader();
                while (reader.Read())
                {
                    labelNbGesteParUser.Content = "Itérations utilisateur : " + reader["count"].ToString();
                }

                connection.Close();

            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
        }
        // PIAD***
    }
}
