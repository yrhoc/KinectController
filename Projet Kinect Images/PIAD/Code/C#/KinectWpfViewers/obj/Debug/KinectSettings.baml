   M S B A M L   `   `   `  ����'  #Microsoft.Samples.Kinect.WpfViewers8   2Microsoft.Samples.Kinect.WpfViewers.KinectSettings   5      \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5      R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5      Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     5      @d2http://schemas.microsoft.com/expression/blend/2008     5      -   5        $
��Root��5      $y�Visible=�5   8   .��5      ��   �� Orientation$  
Horizontal=�6   
���   �$y�Visible=�6=    � 	Resources 5	   
    �� 
TargetType �� Property �� Value��%�   '��       �� 5
      " ��6   .�5      ��  � ��$ 3q�6+   �� 5       �@��Padding$ 5q�6,   �� 5       �@��CornerRadius$ 10��61   �� 5       � ��
 ��İ�6/   5      5      .+�5   
   �� ._�5      �� .+�5      �� $
��Bold"�6   $��Enregistrement��6*   $��28��6C   $	��217��6O   �� 5      -   5        $��comboBoxBase��5   '   $��23��6   $	��170��6;   6G   �� 5      -   5        $��comboBoxGeste��5   '   $��23��6   $	��170��6<   $y�Hidden=�6H   6\   �� 5      -   5        $��Enregistrer��5   ;   $��Enregistrer��6   $��23��6/   $��75��6N   6Y   �� 5      -   5        $��Decompte��5   $   $��28��6   $��Center=�64   5      5      �� 5    
   ._�5!      �� .+�5"      �� $
��Bold"�6   $��Infos geste��6*   $��28��6@   $	��217��6L   �� 5#      -   5        $��labelDescription��5#   9   $	��216��6   $��Italic$�6&   $
��Left=�6Q    �� 
IsReadOnly .  J� Opacity$ 1Q�6~    �� TextWrapping$
 Wrap=�6�   
����İ�$��0q�6�   �� 5$      -   5        $��labelNbGesteTotal��5$   $   $��28��6   $��Itérations total : ��6=   �� 5%      -   5        $��labelNbGesteParUser��5%   $   $��28��6   $ ��Itérations utilisateur : ��6?   5&      5'      �� 5)   
   $y�Hidden=�6   ._�5*      �� $��2q�6   $��1��6$   $y�Hidden=�64   .+�5+      �� $
��Bold"�6   $y�Hidden=�6[    �� Target 6*   �� 	 �� ElementName$	 ColorStreamEnable��.��6o   ._ColorStream6~   �� 5,      -	   5        $��ColorStreamEnable��5,   ,   
 d� 	IsChecked$

 TrueY�6   $y�Hidden=�5.   F   .��6Z   	Enabled6c   �� 5/      -
   5        $��colorFormats��5/      $y�Hidden=�6x   50      51      �� 52   
   $y�Hidden=�6   ._�53      �� $y�Hidden=�6   .+�54      �� $
��Bold"�6   $y�Hidden=�6[    6*   �� $	 DepthStreamEnable��.��6o   ._DepthStream6~   �� 55      -   5        $��DepthStreamEnable��55   ,   $

 TrueY�6   $y�Hidden=�57   F   .��6Z   	Enabled6c   �� 58      -   5        $��depthFormats��58      $y�Hidden=�6z   �� 59      $  
Horizontal=�6   $y�Hidden=�66   .+�5:      �� $��Center=�6G   $y�Hidden=�6b    6   �� $	 depthRanges��.��6v   	_Range:6   �� 5;      -   5        $��depthRanges��5;      $y�Hidden=�6a   5<      5=      5>      �� 5?   
   $y�Hidden=�6   ._�5@      �� $y�Hidden=�6   .+�5A      �� $
��Bold"�6   $y�Hidden=�6^    6*   �� $	 SkeletonStreamEnable��.��6r   ._SkeletonStream6�   �� 5B      -   5        $��SkeletonStreamEnable��5C      $y�Hidden=�6P   .��6d   	Enabled6m   �� 5D      $  
Horizontal=�6   $��Right=�66   $	��Top=�6R   $��3��6j   $��5q�6w   $y�Hidden=�6�   .+�5E      �� $y�Hidden=�6I    6   �� $	 trackingModes��.��6]   _Tracking Mode:6n   �� 5F      -   5        $��trackingModes��5F      $��Center=�64   $y�Hidden=�6�   5G      5H      5I      �� 5J   
   $y�Hidden=�6   ._�5K      �� $y�Hidden=�6   .+�5L      �� $
��Bold"�6   $y�Hidden=�6X    6*   �� $	 ElevationAngle��.��6l   ._ElevationAngle6~   �� 5M      {�.  $y�Hidden=�6c   ��6   �� $	 ElevationAngle�� �� Path$ Value�6c   �� 5N      -   5        $��ElevationAngle��5N       �� Orientation$ Vertical=�6/    � Minimum$	 -27Q�6F    � Maximum$ 27Q�6T   $	��100��6a   $��Center=�6n    �� TickFrequency$ 1Q�5O       �� TickPlacement$ BottomRight=�6+    �� IsSnapToTickEnabled . $y�Hidden=�5P   Y   5Q      5R      5S      5T      