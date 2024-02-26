#!/usr/bin/env python
from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QShortcut, QLabel, QHBoxLayout
from PyQt5.Qt import Qt
class Ui_MainWindow(object):	
    #disp_value = TwistToMotors(node_name='twist_to_motors')
    
    def __init__(self):
    	super(Ui_MainWindow, self).__init__()
    	self.pub = rospy.Publisher('button_action_CMD', UInt8, queue_size=10)
    	self.sld_pub = rospy.Publisher('slider_CMD', Float32MultiArray, queue_size=10)    	
    	rospy.Subscriber("disp_spd", Float32MultiArray, self.display_speed) 
    	rospy.Subscriber('odom', Odometry, self.arm_pose_vel)   	
    	rospy.init_node('GUI_CMD_MONITOR', anonymous=True)    	
    	self.rate = rospy.Rate(20)
    	self.updated_pose_x_value = 0.0
    	self.updated_pose_y_value = 0.0
    	self.updated_theta_value = 0.0
    	self.updated_lin_vel = 0.0
    	self.updated_ang_vel = 0.0
    	self.lin_speed = 0.0
    	self.ang_speed = 0.0
    	self.lftwhrpm = 0.0
    	self.rgtwhrpm = 0.0      	  	   	
    	# Timer for updating line edits
    	self.update_timer = QtCore.QTimer()
    	self.update_timer.timeout.connect(self.update_lineedits)
    	self.update_timer.timeout.connect(self.publish_wheel_and_bot_speed_accdec)
    	self.update_timer.start(30)  # Update every 1000 milliseconds (1 second)
    	
    def setupUi(self, MainWindow):
        #Diffbot Control
        QShortcut(Qt.Key_W, MainWindow, self.forward)
        QShortcut(Qt.Key_Q, MainWindow, self.forward_left)
        QShortcut(Qt.Key_E, MainWindow, self.forward_right)
        QShortcut(Qt.Key_S, MainWindow, self.backward)
        QShortcut(Qt.Key_Z, MainWindow, self.backward_left)
        QShortcut(Qt.Key_C, MainWindow, self.backward_right)
        QShortcut(Qt.Key_A, MainWindow, self.left)
        QShortcut(Qt.Key_D, MainWindow, self.right)
        
        # Wheel Control
        QShortcut(Qt.Key_I, MainWindow, self.rwforward)
        QShortcut(Qt.Key_N, MainWindow, self.rwbackward)
        QShortcut(Qt.Key_O, MainWindow, self.lwforward)
        QShortcut(Qt.Key_M, MainWindow, self.lwbackward)     
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(720, 635)
        MainWindow.setMinimumSize(QtCore.QSize(720, 635)) #rgb(236,236,237
        MainWindow.setStyleSheet("QWidget{\n"
"background-color:rgb(236,236,237);\n"
"}\n"
"\n"
"QPushButton{\n"
"background-color:rgba(214,107,35,200);\n"
"}\n"
"QPushButton:Hover{\n"
"background-color:rgb(244,166,215);\n"
"}")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(18)
        # Main label layout       
        self.label.setFont(font)
        self.label.setStyleSheet("color:black")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.label.setStyleSheet("color:white;\n"
"background-color:#1035AC;")        #1035AC
        self.gridLayout_2.addWidget(self.label, 1, 0, 1, 1, QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        # Diff bot controller layout
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        # Diff bot controller label        
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setMinimumSize(QtCore.QSize(300, 25))
        self.label_4.setMaximumSize(QtCore.QSize(16777215, 25))
        self.label_4.setStyleSheet("color:white;\n"
"background-color:#A52A2A;")
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.verticalLayout_6.addWidget(self.label_4)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        #diff bot controller buttons       
        self.btn_f = QtWidgets.QPushButton(self.centralwidget)
        self.btn_f.setObjectName("btn_f")
        self.btn_f.clicked.connect(self.forward)
        self.gridLayout.addWidget(self.btn_f, 1, 1, 1, 1)
        self.btn_rf = QtWidgets.QPushButton(self.centralwidget)
        self.btn_rf.setObjectName("btn_rf")
        self.btn_rf.clicked.connect(self.forward_right)
        self.gridLayout.addWidget(self.btn_rf, 1, 2, 1, 1)
        self.btn_r = QtWidgets.QPushButton(self.centralwidget)
        self.btn_r.setObjectName("btn_r")
        self.btn_r.clicked.connect(self.right)
        self.gridLayout.addWidget(self.btn_r, 2, 2, 1, 1)
        self.btn_lb = QtWidgets.QPushButton(self.centralwidget)
        self.btn_lb.setObjectName("btn_lb")
        self.btn_lb.clicked.connect(self.backward_left)
        self.gridLayout.addWidget(self.btn_lb, 3, 0, 1, 1)
        self.btn_rb = QtWidgets.QPushButton(self.centralwidget)
        self.btn_rb.setObjectName("btn_rb")
        self.btn_rb.clicked.connect(self.backward_right)
        self.gridLayout.addWidget(self.btn_rb, 3, 2, 1, 1)
        self.btn_b = QtWidgets.QPushButton(self.centralwidget)
        self.btn_b.setObjectName("btn_b")
        self.btn_b.clicked.connect(self.backward)
        self.gridLayout.addWidget(self.btn_b, 3, 1, 1, 1)
        self.btn_lf = QtWidgets.QPushButton(self.centralwidget)
        self.btn_lf.setObjectName("btn_lf")
        self.btn_lf.clicked.connect(self.forward_left)
        self.gridLayout.addWidget(self.btn_lf, 1, 0, 1, 1)
        self.btn_s = QtWidgets.QPushButton(self.centralwidget)
        self.btn_s.setObjectName("btn_s")
        self.btn_s.clicked.connect(self.AMR_stop)
        self.gridLayout.addWidget(self.btn_s, 2, 1, 1, 1)
        self.btn_l = QtWidgets.QPushButton(self.centralwidget)	
        self.btn_l.setObjectName("btn_l")
        self.btn_l.clicked.connect(self.left)
        self.gridLayout.addWidget(self.btn_l, 2, 0, 1, 1)
        self.verticalLayout_6.addLayout(self.gridLayout)
        self.gridLayout_2.addLayout(self.verticalLayout_6, 2, 2, 1, 1)
        
        # CONT & MON layout
        self.verticalLayout_1 = QtWidgets.QVBoxLayout()
        self.verticalLayout_1.setContentsMargins(1, 30, 1, 1) #10, -1, 10, -1
        self.verticalLayout_1.setObjectName("verticalLayout_1")
        
        #DIFF-BOT WHEEL RPM CONTROL
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy)
        self.label_8.setMinimumSize(QtCore.QSize(320, 25))
        self.label_8.setMaximumSize(QtCore.QSize(16777215, 25))
        self.label_8.setStyleSheet("color:white;\n"
"background-color:#A52A2A;")
        self.label_8.setObjectName("label_8")
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)        
        
        # DIFF-BOT WHEEL CONTROL LABEL        
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setMinimumSize(QtCore.QSize(300, 25))
        self.label_3.setMaximumSize(QtCore.QSize(16777215, 25))
        self.label_3.setStyleSheet("color:white;\n"
"background-color:#A52A2A;")
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")

        # BOT POSITION LABEL MONITOR         
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy)
        self.label_6.setMinimumSize(QtCore.QSize(320, 25))
        self.label_6.setMaximumSize(QtCore.QSize(16777215, 25))
        self.label_6.setStyleSheet("color:white;\n"
"background-color:#A52A2A;")
        self.label_6.setObjectName("label_6")
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
    
        # BOT VELOCITY MONITOR         
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_7.sizePolicy().hasHeightForWidth())
        self.label_7.setSizePolicy(sizePolicy)
        self.label_7.setMinimumSize(QtCore.QSize(320, 25))
        self.label_7.setMaximumSize(QtCore.QSize(16777215, 25))
        self.label_7.setStyleSheet("color:white;\n"
"background-color:#A52A2A;")
        self.label_7.setObjectName("label_7")
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        
        #WHEEL SPEED LIMIT GRID
        self.gridLayout_2.addWidget(self.label_8, 3, 0, 1, 1, QtCore.Qt.AlignTop)  # Adjust the row and column accordingly
        self.gridLayout_8 = QtWidgets.QGridLayout()
        self.gridLayout_8.setContentsMargins(0, 2, 0, 0)
        self.gridLayout_8.setVerticalSpacing(5)
        self.gridLayout_8.setObjectName("gridLayout_8")
        
        #WHEEL SPEED LIMIT SLIDER & LABEL
        self.lwspeed = QtWidgets.QSlider(self.centralwidget)
        self.label_lwspeed = QtWidgets.QLabel(self.centralwidget)
        self.label_lwspeed.setText("Left wheel in RPM")
        self.label_lwspeed.setStyleSheet("background-color: #B87333; color: #FFFFFF;")  
        self.lwspeed.setStyleSheet("background-color: grey;")        
        self.gridLayout_8.addWidget(self.label_lwspeed, 0, 0, 1, 1)
        self.lineedit_lwspeed = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_lwspeed.setObjectName("lineedit_lwspeed")
        self.lineedit_lwspeed.setStyleSheet("background-color: white;")
        self.gridLayout_8.addWidget(self.lineedit_lwspeed, 0, 1, 1, 1)
        self.lwspeed.setMaximum(255)
        self.lwspeed.setOrientation(QtCore.Qt.Horizontal)
        self.lwspeed.setObjectName("lwspeed") 
        #self.lwspeed.sliderReleased.connect(self.publish_wheel_and_bot_speed_accdec)       
        self.gridLayout_8.addWidget(self.lwspeed, 1, 0, 1, 2)  # Use addWidget here instead of insertWidget
        #self.verticalLayout_1.insertWidget(1, self.lwspeed)
        self.rwspeed = QtWidgets.QSlider(self.centralwidget)
        self.label_rwspeed = QtWidgets.QLabel(self.centralwidget)
        self.label_rwspeed.setText("Right Wheel in RPM")
        self.label_rwspeed.setStyleSheet("background-color: #B87333; color: #FFFFFF;")  
        self.rwspeed.setStyleSheet("background-color: grey;")
        self.gridLayout_8.addWidget(self.label_rwspeed, 2, 0, 1, 1)
        self.lineedit_rwspeed = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_rwspeed.setObjectName("lineedit_rwspeed")
        self.lineedit_rwspeed.setStyleSheet("background-color: white;")
        self.gridLayout_8.addWidget(self.lineedit_rwspeed, 2, 1, 1, 1)        
        self.rwspeed.setMaximum(255)  
        self.rwspeed.setOrientation(QtCore.Qt.Horizontal)
        self.rwspeed.setObjectName("rwspeed")
        #self.rwspeed.sliderReleased.connect(self.publish_wheel_and_bot_speed_accdec)
        self.gridLayout_8.addWidget(self.rwspeed, 3, 0, 1, 2)  # Use addWidget here instead of insertWidget
        self.verticalLayout_1.insertLayout(1, self.gridLayout_8)        
        
        # WHEEL CONTROL GRID              
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setContentsMargins(0, 2, 0, 0)
        self.gridLayout_3.setVerticalSpacing(5)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.gridLayout_3.addWidget(self.label_3, 0, 0, 1, 2, QtCore.Qt.AlignTop)
        
        # WHEEL CONTROL BUTTONS
        self.btn_lwf = QtWidgets.QPushButton(self.centralwidget)
        self.btn_lwf.setObjectName("btn_lwf")
        self.btn_lwf.clicked.connect(self.lwforward)  
        self.gridLayout_3.addWidget(self.btn_lwf, 1, 0, 1, 1)
        self.btn_rwf = QtWidgets.QPushButton(self.centralwidget)
        self.btn_rwf.setObjectName("btn_rwf")
        self.btn_rwf.clicked.connect(self.rwforward)  
        self.gridLayout_3.addWidget(self.btn_rwf, 1, 1, 1, 1)
        self.btn_lwb = QtWidgets.QPushButton(self.centralwidget)
        self.btn_lwb.setObjectName("btn_lwb")  
        self.btn_lwb.clicked.connect(self.lwbackward) 
        self.gridLayout_3.addWidget(self.btn_lwb, 2, 0, 1, 1)
        self.btn_rwb = QtWidgets.QPushButton(self.centralwidget)
        self.btn_rwb.setObjectName("btn_rwb")
        self.btn_rwb.clicked.connect(self.rwbackward)  
        self.gridLayout_3.addWidget(self.btn_rwb, 2, 1, 1, 1)
        self.verticalLayout_1.addLayout(self.gridLayout_3) 
        
        # BOT POSITION GRID
        self.gridLayout_5 = QtWidgets.QGridLayout()
        self.gridLayout_5.setContentsMargins(0, 5, 0, 5)
        self.gridLayout_5.setVerticalSpacing(5)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.gridLayout_5.addWidget(self.label_6, 0, 0, 1, 2, QtCore.Qt.AlignTop)
        
        # BOT POSIITON ENTRY LABEL AND EDIT        
        self.lbl_poseX = QtWidgets.QLabel(self.centralwidget)
        self.lbl_poseX.setObjectName("lbl_poseX")
        self.lbl_poseX.setStyleSheet("background-color: #B87333; color: #FFFFFF;")         
        #self.btn_poseX.clicked.connect(self.position_X)  
        self.gridLayout_5.addWidget(self.lbl_poseX, 1, 0, 1, 1)
        self.lineedit_PX = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_PX.setObjectName("lineedit_PX")
        self.lineedit_PX.setStyleSheet("background-color: white;")
        # Connect the appropriate signal (e.g., editingFinished) to the desired slot
        #self.lineedit_PX.editingFinished.connect(self.position_X)
        self.gridLayout_5.addWidget(self.lineedit_PX, 1, 1, 1, 1)   
        self.lbl_poseY = QtWidgets.QLabel(self.centralwidget)      
        self.lbl_poseY.setObjectName("lbl_poseY")
        self.lbl_poseY.setStyleSheet("background-color: #B87333; color: #FFFFFF;")       
        #self.lbl_poseY.clicked.connect(self.position_Y)          
        self.gridLayout_5.addWidget(self.lbl_poseY, 2, 0, 1, 1)
        self.lineedit_PY = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_PY.setObjectName("lineedit_PY")
        self.lineedit_PY.setStyleSheet("background-color: white;")
        # Connect the appropriate signal (e.g., editingFinished) to the desired slot
        #self.lineedit_PY.editingFinished.connect(self.position_Y)
        self.gridLayout_5.addWidget(self.lineedit_PY, 2, 1, 1, 1)         
        self.lbl_theta = QtWidgets.QLabel(self.centralwidget)
        self.lbl_theta.setObjectName("btn_theta")
        self.lbl_theta.setStyleSheet("background-color: #B87333; color: #FFFFFF;")          
        #self.lbl_theta.clicked.connect(self.theta)  
        self.gridLayout_5.addWidget(self.lbl_theta, 3, 0, 1, 1)
        self.lineedit_THETA = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_THETA.setObjectName("lineedit_THETA")
        self.lineedit_THETA.setStyleSheet("background-color: white;")
        # Connect the appropriate signal (e.g., editingFinished) to the desired slot
        #self.lineedit_THETA.editingFinished.connect(self.Ori_THETA)
        self.gridLayout_5.addWidget(self.lineedit_THETA, 3, 1, 1, 1)         
        #self.lbl_poseY.setObjectName("btn_poseY")
        #self.lbl_poseY.clicked.connect(self.position_Y)  
        #self.gridLayout_5.addWidget(self.lbl_poseY, 2, 0, 1, 1)
        #self.btn_theta = QtWidgets.QPushButton(self.centralwidget)                           
        self.verticalLayout_1.addLayout(self.gridLayout_5)
        
        # BOT VELOCITY GRID LAYOUT
        self.gridLayout_7 = QtWidgets.QGridLayout()
        self.gridLayout_7.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_7.setVerticalSpacing(5)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.gridLayout_7.addWidget(self.label_7, 0, 0, 1, 2, QtCore.Qt.AlignTop)        

        # BOT VELOCITY ENTRY LABEL AND EDIT        
        self.lbl_botLV = QtWidgets.QLabel(self.centralwidget)
        self.lbl_botLV.setObjectName("lbl_botLV")
        self.lbl_botLV.setStyleSheet("background-color: #B87333; color: #FFFFFF;")         
        #self.btn_poseX.clicked.connect(self.position_X)  
        self.gridLayout_7.addWidget(self.lbl_botLV, 1, 0, 1, 1)
        self.lineedit_botLV = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_botLV.setObjectName("lineedit_botLV")
        self.lineedit_botLV.setStyleSheet("background-color: white;")
        # Connect the appropriate signal (e.g., editingFinished) to the desired slot
        #self.lineedit_PX.editingFinished.connect(self.position_X)
        self.gridLayout_7.addWidget(self.lineedit_botLV, 1, 1, 1, 1)   
        self.lbl_botAV = QtWidgets.QLabel(self.centralwidget)      
        self.lbl_botAV.setObjectName("lbl_botAV")
        self.lbl_botAV.setStyleSheet("background-color: #B87333; color: #FFFFFF;")       
        #self.lbl_poseY.clicked.connect(self.position_Y)          
        self.gridLayout_7.addWidget(self.lbl_botAV, 2, 0, 1, 1)
        self.lineedit_botAV = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_botAV.setObjectName("lineedit_botAV")
        self.lineedit_botAV.setStyleSheet("background-color: white;")
        # Connect the appropriate signal (e.g., editingFinished) to the desired slot
        #self.lineedit_PY.editingFinished.connect(self.position_Y)
        self.gridLayout_7.addWidget(self.lineedit_botAV, 2, 1, 1, 1)                         
        self.verticalLayout_1.addLayout(self.gridLayout_7)                 
        self.gridLayout_2.addLayout(self.verticalLayout_1, 3, 0, 1, 1, QtCore.Qt.AlignTop) 
        
        # LOGS LAYOUT
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setContentsMargins(1, 1, 1, 1)
        self.verticalLayout_2.setObjectName("verticalLayout_2")       
        
        #ACCELERATION/DECELERATION - INPUT 
        self.horizontalLayout_0 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_0.setObjectName("horizontalLayout_0")
        self.label_0 = QtWidgets.QLabel(self.centralwidget)        
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_0.sizePolicy().hasHeightForWidth())
        self.label_0.setSizePolicy(sizePolicy)
        self.label_0.setMinimumSize(QtCore.QSize(300, 25))
        self.label_0.setMaximumSize(QtCore.QSize(16777215, 25))
        self.label_0.setStyleSheet("color:white;\n"
"background-color:#A52A2A;")  #A52A2A
        self.label_0.setAlignment(QtCore.Qt.AlignCenter)
        self.label_0.setObjectName("label_0")
        
        #ACC/DEC GRID LAYOUT FRAMING        
        self.gridLayout_0 = QtWidgets.QGridLayout()
        self.gridLayout_0.setObjectName("gridLayout_0")
        self.gridLayout_0.addWidget(self.label_0, 0, 0, 1, 2)        
        self.verticalLayout_2.addLayout(self.gridLayout_0)
        
        #ACC/DEC GRID LINE EDIT
        self.label_amraccel = QtWidgets.QLabel(self.centralwidget)
        self.label_amraccel.setText("AMR ACCELERATION")
        self.label_amraccel.setStyleSheet("background-color: #B87333; color: #FFFFFF;")       
        self.gridLayout_0.addWidget(self.label_amraccel, 1, 0, 1, 1)
        self.lineedit_amraccel = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_amraccel.setObjectName("lineedit_amraccel")
        self.lineedit_amraccel.setStyleSheet("background-color: white;")
        self.lineedit_amraccel.setText("0.01")  # Set default text as a string representation of the float
        self.gridLayout_0.addWidget(self.lineedit_amraccel, 1, 1, 1, 1)
        self.label_amrdec = QtWidgets.QLabel(self.centralwidget)
        self.label_amrdec.setText("AMR DECELERATION")
        self.label_amrdec.setStyleSheet("background-color: #B87333; color: #FFFFFF;")        
        self.gridLayout_0.addWidget(self.label_amrdec, 2, 0, 1, 1)
        self.lineedit_amrdec = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_amrdec.setObjectName("lineedit_amrdec")
        self.lineedit_amrdec.setStyleSheet("background-color: white;")
        self.lineedit_amrdec.setText("0.01")  # Set default text as a string representation of the float
        self.gridLayout_0.addWidget(self.lineedit_amrdec, 2, 1, 1, 1)    	
    	                
        #SECOND ROW LAYOUT
        self.horizontalLayout_1 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_1.setObjectName("horizontalLayout_1")
        self.label_9 = QtWidgets.QLabel(self.centralwidget)        
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_9.sizePolicy().hasHeightForWidth())
        self.label_9.setSizePolicy(sizePolicy)
        self.label_9.setMinimumSize(QtCore.QSize(300, 25))
        self.label_9.setMaximumSize(QtCore.QSize(16777215, 25))
        self.label_9.setStyleSheet("color:white;\n"
"background-color:#A52A2A;")  #A52A2A
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        
        #GRID LAYOUT FRAMING        
        self.gridLayout_9 = QtWidgets.QGridLayout()
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.gridLayout_9.addWidget(self.label_9, 0, 0, 1, 2)        
        self.verticalLayout_2.addLayout(self.gridLayout_9)
        
        # Add AUTO/MANUAL push buttons in the same row below label_0        
        self.btn_auto = QtWidgets.QPushButton(self.centralwidget)
        self.btn_auto.setObjectName("btn_auto")
        self.btn_auto.clicked.connect(self.auto_mode)
        self.gridLayout_9.addWidget(self.btn_auto, 2, 0, 1, 1)
        
        self.btn_manl = QtWidgets.QPushButton(self.centralwidget)
        self.btn_manl.setObjectName("btn_manl")
        self.btn_manl.clicked.connect(self.manual_mode)  # You can connect it to a different slot if needed
        self.gridLayout_9.addWidget(self.btn_manl, 2, 1, 1, 1)
        self.verticalLayout_2.addLayout(self.horizontalLayout_1)        
        
               
        # SECOND ROW LAYOUT
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        self.label_5.setMinimumSize(QtCore.QSize(300, 25))
        self.label_5.setMaximumSize(QtCore.QSize(16777215, 25))
        self.label_5.setStyleSheet("color:white;\n"
"background-color:#A52A2A;")  #A52A2A
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")       
        self.horizontalLayout_2.addWidget(self.label_5)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        
        # LOGS GRID
        self.logs = QtWidgets.QTextEdit(self.centralwidget)
        self.logs.setReadOnly(True)
        self.logs.setObjectName("logs")
        self.logs.setStyleSheet("color:green")
        self.logs.setStyleSheet("color:red;\n"
"background-color:white;color:Blue;")           #A52A2A
        self.verticalLayout_2.addWidget(self.logs)
        self.gridLayout_2.addLayout(self.verticalLayout_2, 3, 1, 1, 2)
        
        #DIFF-BOT SPEED CONTROL LAYOUT
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setContentsMargins(1, 1, 1, 1)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")        
        
        #DIFF-BOT SPEED CONTROL LABEL       
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setMinimumSize(QtCore.QSize(300, 25))
        self.label_2.setMaximumSize(QtCore.QSize(16777215, 25))
        self.label_2.setStyleSheet("color:white;\n"
"background-color:#A52A2A;color:white;")
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)        
        self.label_lnspeed = QtWidgets.QLabel(self.centralwidget)
        self.label_lnspeed.setText("Linear Speed in m/s")
        self.verticalLayout.insertWidget(1,self.label_lnspeed)        
        self.lineedit_lnspeed = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_lnspeed.setObjectName("lineedit_lnspeed")
        self.lineedit_lnspeed.setStyleSheet("background-color: white;")        
        alignment = QtCore.Qt.Alignment(QtCore.Qt.AlignCenter)
        self.lineedit_lnspeed.setAlignment(alignment)
        self.verticalLayout.insertWidget(2,self.lineedit_lnspeed)
        self.lnspeed = QtWidgets.QSlider(self.centralwidget)
        self.lnspeed.setStyleSheet("background-color: grey;")
        self.lnspeed.setMaximum(255)
        self.lnspeed.setOrientation(QtCore.Qt.Horizontal)
        self.lnspeed.setObjectName("lnspeed")
        #self.lnspeed.sliderReleased.connect(self.publish_wheel_and_bot_speed_accdec)        
        self.verticalLayout.insertWidget(3, self.lnspeed)
        self.anspeed = QtWidgets.QSlider(self.centralwidget)
        self.label_anspeed = QtWidgets.QLabel(self.centralwidget)
        self.label_anspeed.setText("Angular Speed in rad/s")
        self.anspeed.setStyleSheet("background-color: grey;")
        self.verticalLayout.insertWidget(4,self.label_anspeed)
        self.lineedit_anspeed = QtWidgets.QLineEdit(self.centralwidget)
        self.lineedit_anspeed.setObjectName("lineedit_anspeed")
        self.lineedit_anspeed.setStyleSheet("background-color: white;")        
        alignment = QtCore.Qt.Alignment(QtCore.Qt.AlignCenter)
        self.lineedit_anspeed.setAlignment(alignment)
        self.verticalLayout.insertWidget(5,self.lineedit_anspeed)
        self.anspeed.setMaximum(255)  
        self.anspeed.setOrientation(QtCore.Qt.Horizontal)
        self.anspeed.setObjectName("anspeed")
        #self.anspeed.sliderReleased.connect(self.publish_wheel_and_bot_speed_accdec)
        self.verticalLayout.insertWidget(6, self.anspeed)
        self.gridLayout_2.addLayout(self.verticalLayout, 2, 0, 1, 1)     
        MainWindow.setCentralWidget(self.centralwidget)
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
	
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "BIT-TEXSONICS MOBILE ROBOT APP-2023"))
        self.label.setText(_translate("MainWindow", "AMR CONTROLLER"))
        self.label_2.setText(_translate("MainWindow", "BOT SPEED CONTROL"))
        self.label_4.setText(_translate("MainWindow", "DIFFBOT CONTROL"))
        self.btn_f.setText(_translate("MainWindow", "Forward"))
        self.btn_rf.setText(_translate("MainWindow", "R. Forward"))
        self.btn_r.setText(_translate("MainWindow", "Right"))
        self.btn_lb.setText(_translate("MainWindow", "L. Backward"))
        self.btn_rb.setText(_translate("MainWindow", "R. Backward"))
        self.btn_b.setText(_translate("MainWindow", "Back"))
        self.btn_lf.setText(_translate("MainWindow", "L. Forward"))
        self.btn_s.setText(_translate("MainWindow", "Stop"))
        self.btn_l.setText(_translate("MainWindow", "Left"))     
        self.label_3.setText(_translate("MainWindow", "WHEEL CONTROL"))
        self.btn_rwf.setText(_translate("MainWindow", "RW_Forward"))
        self.btn_rwb.setText(_translate("MainWindow", "RW_Backward"))
        self.btn_lwf.setText(_translate("MainWindow", "LW_Forward"))
        self.btn_lwb.setText(_translate("MainWindow", "LW_Backward"))
        self.label_5.setText(_translate("MainWindow", "LOG MESSAGES"))  
        self.label_6.setText(_translate("MainWindow", "BOT POSITION"))
        self.lbl_poseX.setText(_translate("MainWindow", "Bot_Position_X in m"))
        self.lbl_poseY.setText(_translate("MainWindow", "Bot_Position_Y in m"))
        self.lbl_theta.setText(_translate("MainWindow", "BOT_THETA in deg"))
        self.label_7.setText(_translate("MainWindow", "BOT VELOCITY")) 
        self.label_8.setText(_translate("MainWindow", "WHEEL SPEED LIMIT"))                   
        self.lbl_botLV.setText(_translate("MainWindow", "Bot_Lin Velocity-m/s  "))
        self.lbl_botAV.setText(_translate("MainWindow", "Bot_Ang Velocity-rad/s"))
        self.label_9.setText(_translate("MainWindow", "MODE OF OPERATION"))
        self.btn_auto.setText(_translate("MainWindow", "AUTO MODE"))
        self.btn_manl.setText(_translate("MainWindow", "MANUAL MODE"))
        self.label_0.setText(_translate("MainWindow", "SET AMR ACC/DEC"))      
	# Diffbot Control
	
    def forward(self):
        print("Forward")        
        rospy.loginfo("Forward")
        self.pub.publish(1)        
        self.logs.setText(self.logs.toPlainText()+"Forward\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())
        
    def forward_left(self):
        print("Forward Left")
        rospy.loginfo("Forward Left")
        self.pub.publish(2)
        self.logs.setText(self.logs.toPlainText()+"Left Forward\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())
        
    def forward_right(self):
        print("Forward Right")
        rospy.loginfo("Forward Right")
        self.pub.publish(3)
        self.logs.setText(self.logs.toPlainText()+"Right Forward\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())
        
    def left(self):
        print("Left")
        rospy.loginfo("Left")
        self.pub.publish(4)
        self.logs.setText(self.logs.toPlainText()+"Left\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())
        
    def right(self):
        print("Right")
        rospy.loginfo("Right")
        self.pub.publish(5)
        self.logs.setText(self.logs.toPlainText()+"Right\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())

    def backward(self):
        print("Backward")
        rospy.loginfo("Backward")
        self.pub.publish(6)
        self.logs.setText(self.logs.toPlainText()+"Backward\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())
        
    def backward_right(self):
        print("Backward Right")
        rospy.loginfo("Backward Right")
        self.pub.publish(7)
        self.logs.setText(self.logs.toPlainText()+"Right Backward\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())
    
    def backward_left(self):
        print("Backward Left")
        rospy.loginfo("Backward Left")
        self.pub.publish(8)
        self.logs.setText(self.logs.toPlainText()+"Left Backward\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())

    def AMR_stop(self):
        print("AMR Stopped")
        rospy.loginfo("AMR Stopped")
        self.pub.publish(9)
        self.logs.setText(self.logs.toPlainText()+"AMR Stopped\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())
    
    def rwforward(self):    	       
        print("RW_Forward")
        rospy.loginfo("RW_Forward")
        self.pub.publish(10)        
        self.logs.setText(self.logs.toPlainText()+"RW_Forward\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())

    def rwbackward(self):    	       
        print("RW_Backward")
        rospy.loginfo("RW_Backward")
        self.pub.publish(11)        
        self.logs.setText(self.logs.toPlainText()+"RW_Backward\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())

    def lwforward(self):    	       
        print("LW_Forward")
        rospy.loginfo("LW_Forward")
        self.pub.publish(12)        
        self.logs.setText(self.logs.toPlainText()+"LW_Forward\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())

    def lwbackward(self):    	       
        print("LW_Backward")
        rospy.loginfo("LW_Backward")
        self.pub.publish(13)        
        self.logs.setText(self.logs.toPlainText()+"LW_Backward\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())

    def auto_mode(self):
        print("Auto_MODE")        
        rospy.loginfo("Auto_MODE")
        self.pub.publish(14)        
        self.logs.setText(self.logs.toPlainText()+"Auto_MODE\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum())    	

    def manual_mode(self):
        print("Manual_MODE")        
        rospy.loginfo("Manual_MODE")
        self.pub.publish(15)        
        self.logs.setText(self.logs.toPlainText()+"Manual_MODE\n")
        x = self.logs.verticalScrollBar()
        x.setValue(x.maximum()) 
    
    def publish_wheel_and_bot_speed_accdec(self):
    	# Get the left and right wheel speeds from the sliders
    	try:
	    	lwspeed_value = self.lwspeed.value()
	    	rwspeed_value = self.rwspeed.value()
	    	lnspeed_value = self.lnspeed.value()
	    	anspeed_value = self.anspeed.value()
	    	amraccel_value = float(self.lineedit_amraccel.text())
	    	amrdec_value = float(self.lineedit_amrdec.text())
	    	bot_whl_speed_msg = Float32MultiArray()
	    	bot_whl_speed_msg.layout.dim = []
	    	bot_whl_speed_msg.layout.data_offset = 0
	    	# Create a UInt8 message with the wheel speeds
	    	bot_whl_speed_msg.data = [lwspeed_value, rwspeed_value, lnspeed_value, anspeed_value, amraccel_value, amrdec_value]   # Use .data to set the value
	    	# Publish the message to the 'action_GUI' topic
	    	self.sld_pub.publish(bot_whl_speed_msg)
	    	#print("sliderpublished")
	    	#print(bot_whl_speed_msg.data[0], bot_whl_speed_msg.data [1], bot_whl_speed_msg.data [2], bot_whl_speed_msg.data [3])
    	except ValueError:
    		print("Invalid input in acceleration/deceleration fields")        	
    
    def arm_pose_vel(self, msg):
    	self.updated_pose_x_value = msg.pose.pose.position.x
    	self.updated_pose_y_value = msg.pose.pose.position.y
    	self.updated_theta_value = msg.pose.pose.orientation.z
    	self.updated_lin_vel = msg.twist.twist.linear.x
    	self.updated_ang_vel = msg.twist.twist.angular.z    	
    	
    def display_speed(self, msg_speed):
    	self.lin_speed = msg_speed.data[0]
    	self.ang_speed = msg_speed.data[1]
    	self.lftwhrpm = msg_speed.data[2]
    	self.rgtwhrpm = msg_speed.data[3]    
    	
    def update_lineedits(self):
    	self.lineedit_PX.setText(str(self.updated_pose_x_value))
    	self.lineedit_PY.setText(str(self.updated_pose_y_value))
    	self.lineedit_THETA.setText(str(self.updated_theta_value))
    	self.lineedit_lnspeed.setText(str(self.lin_speed))
    	self.lineedit_anspeed.setText(str(self.ang_speed))   
    	self.lineedit_lwspeed.setText(str(self.lftwhrpm))
    	self.lineedit_rwspeed.setText(str(self.rgtwhrpm))
    	self.lineedit_botLV.setText(str(self.updated_lin_vel))
    	self.lineedit_botAV.setText(str(self.updated_ang_vel))
    	#print("DISPLAY")
    	#print(self.lin_speed, self.ang_speed, self.lftwhrpm, self.rgtwhrpm)      	
	
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
