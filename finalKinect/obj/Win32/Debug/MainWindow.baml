   M S B A M L   `   `   `  ����Q  MFaceTrackingBasics-WPF, Version=0.0.0.0, Culture=neutral, PublicKeyToken=null7 clr-namespace:FaceTrackingBasicsFaceTrackingBasics  5   	   ^ ZMicrosoft.Samples.Kinect.WpfViewers, Version=0.0.0.0, Culture=neutral, PublicKeyToken=null�^clr-namespace:Microsoft.Samples.Kinect.WpfViewers;assembly=Microsoft.Samples.Kinect.WpfViewers#Microsoft.Samples.Kinect.WpfViewers 5      #    FaceTrackingBasics.MainWindow   5   	   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   2local clr-namespace:FaceTrackingBasics     5   	   mmy^clr-namespace:Microsoft.Samples.Kinect.WpfViewers;assembly=Microsoft.Samples.Kinect.WpfViewers     5      -   5          � Title$  지암 4조��5   	    � WindowState$ 	Maximized=�5   �    � 	Resources 5	         MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush �� 
TargetType �� Property �� Value��%�   &	        &	 #     &	 F     '�� i     �� 5
   
   $ 	#ff6e6e6e��62   �� 5   
   $ 	#ff52318f��64   �� 5   
   $ 	#ff00BCF2��62   �� 5   
   " ��6   .�5      ��  �@J�SnapsToDevicePixels . 64   5      5      .��5      � -   5        $��
layoutGrid��5      $��
10 0 10 10q�6    � RowDefinitions 5   
   �� 5      $
��Auto �6   �� 5      $��* �6   .+�5   
   ]� $��0��6   $��0 0 0 20q�6!   .+�5      �� $
��Left=�6   $��Images\Logo.png��6*   $
��Fill=�6C   $��32��6R   $��81��6^   $��0 10 0 5q�6i   �� 5      $��Right=�6   $��0 0 -1 0q�6/   #���  6A   $��Viner Hand ITC)�6p   $��36(�6�   $	��205��6�   $��Right=�6�   get some Z`s6�   5      (� 5   
   $��1��6    (� Stretch$ Uniform=�6   $��Center=�61   .��5      � -   5        $��MainGrid��5      $
��1158��6#   $	��519��60   	 � ColumnDefinitions	 5      �� 5      $
��270* �6'   �� 5      $
��173* �6'   �� 5       $��0* �6'   �� 5!      $
��219* �6'   �� 5"      $
��496* �6'   .+�5$      �� -   5        $��
ColorImage��5$      $��10,262,89,10q�6*   $��2��6@   
 J� RenderTransformOrigin$
 0.5,0.5&�6T   z�5%      V� 5&      .~�5'      �� �� 5(      �� 5)       �� Angle$
 0.53Q�6.   T� 5*      5+      5-      �� 5.      -   5        $��Image��5.   .   $��10,10,89,262q�6   $��2��6;   �� 5/      -   5        $��RecordButton��5/   �   $�� ��6   $��4��6$   $��32(�64   $	��155��6C   $
��Left=�6P   $��149,364,0,0q�6k   $	��Top=�6�   $	��171��6�   $��Microsoft YaHei)�6�   ��50      �� 51       �� ImageSource$ Images/sleep_btn.png��6%   ��53      �� 54      $ Images/sleep_btn.png��6%   ��56      �� 57      $ Images/sleep_btn.png��6%    J� OpacityMask 59      �� 5:      $ Images/sleep_btn.png��6%   5<      �� 5=      -   5        $��
StopButton��5=   n   $�� ��6   $	��155��6$   $��32(�61   $��Right=�6?   $��	0,364,0,0q�6[   $	��Top=�6�   $	��171��6�   $��4��6�   {�.  ��5>      �� 5?      $ Images/wakeup_btn.png��6%   ��5A      �� 5B      $ Images/wakeup_btn.png��6%   ��5D      �� 5E      $ Images/wakeup_btn.png��6%    5G      �� 5H      $ Images/wakeup_btn.png��6%   5J      �� 5K      -   5        $��sleepstateBlock��5K   @   $��20(�6   $��0,-1,229,425q�6*   $��준비중..��6W   $��3��6d   $��2��6t   �� 5L      -	   5        $��ativityBlock��5L   L   $��18(�6   $	��138��6*   $��123,226,0,0q�67   $�� ��6`   $	��Top=�6h   $��4��6�   �� 5M      -
   5        $��sleepAttitude��5M   L   $��20(�6   $��35��6*   $��284,168,55,0q�66   $��현재의 수면자세��6a   $	��Top=�6q   $��4��6�   =  7Microsoft.Samples.Kinect.WpfViewers.KinectSensorChooser  5N      -   5        $��kinectSensorChooser1��5N   B   $��152,168,408,105q�6)   $��1��6^   $��4��6n   �� 5O      -   5         ��Name$ userName��5O   (   $��25(�6   $��4��6:   $
��Left=�6K   $��50��6f   $��
284,90,0,0q�6r    �� TextWrapping$
 Wrap=�6�   $��이름��6�   $	��Top=�6�   $	��129��6�   6�   �� 5P      -   5         ��Name$ testboxx��5P   A   $��15(�6   $��0,118,229,262q�6*   $�� ��6S   $��3��6\   $��2��6l   �� 5Q      -   5        $ sleepAttitudeNumber��5Q   N   $	��200(�6   $	��262��6+   $��62,247,444,0q�68   $�� ��6k   $	��Top=�6s   $��3��6�   $��2��6�   5R      5S      +   %FaceTrackingBasics.FaceTrackingViewer  5T   
   -   5          Name$ faceTrackingViewer��5T   #   $��	0,416,0,0q�6?   $
��Left=�6R   $��0��6m   $��1��6w   $
 0.496,0.519&�6�   $��0��6�   $	��Top=�6�   5U      5V      