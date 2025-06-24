from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QFileDialog
from PyQt5.uic import loadUi
import numpy as np
import pyqtgraph as pg

from pyqtgraph.Qt import QtCore
import csv


class RobotUI(QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi("/home/liemtran/liem_ws/src/user_interface/ui/test.ui", self)
        # loadUi("/home/liemtran/liem_ws/src/user_interface/ui/final.ui", self)
        
        self.setWindowTitle("Mobile Robot Control")
        self.setGeometry(100, 100, 1300, 900)  # Set window size and position
        
        self.light_color_stop()

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')  # 'w' viết tắt cho white
        plot_layout = QVBoxLayout()              # Tạo layout
        plot_layout.addWidget(self.plot_widget)            # Thêm plot vào layout
        self.plotGroupBox.setLayout(plot_layout)           # Gán layout vào groupbox

        # Gán tên cho từng trục
        # self.plot_widget.setLabel('left', 'Y Axis', units='m')       # trục Y
        # self.plot_widget.setLabel('bottom', 'X Axis', units='m')     # trục X

        self.plot_widget.setLabel('left', 'Y')       # trục Y
        self.plot_widget.setLabel('bottom', 'X')     # trục X        
        self.plot_widget.setAspectLocked(True) # Giữ tỉ lệ khung hình cố định

        self.setpoint_x= []
        self.setpoint_y= []
        self.current_x = []
        self.current_y = []
        self.plot_widget.addLegend(offset=(0, 10))
        # Tạo plot
        self.curve_setpoint = self.plot_widget.plot(self.setpoint_x, self.setpoint_y,
                                                    pen=pg.mkPen(color=(0,0,255), width=4), name="Waypoints (Setpoint)")
        self.curve_current = self.plot_widget.plot(self.current_x, self.current_y,
                                                pen=pg.mkPen(color=(0,255,0), width=4), name="Current_Position")

        self.clearGraphButton.clicked.connect(self.clearGraph)

    def clearGraph(self):
        self.setpoint_x.clear()
        self.setpoint_y.clear()
        self.current_x.clear()
        self.current_y.clear()
        self.curve_setpoint.setData([], [])
        self.curve_current.setData([], [])

    def plot_setpoint(self, file_path):
        self.setpoint_x = []
        self.setpoint_y = []

        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Bỏ dòng tiêu đề

            for row in reader:
                if len(row) >= 2:
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        self.setpoint_x.append(x)
                        self.setpoint_y.append(y)
                    except ValueError:
                        continue  # Bỏ qua dòng không hợp lệ

        self.num_points = len(self.setpoint_x)
        self.curve_setpoint.setData(self.setpoint_x, self.setpoint_y)


    def update_current_position(self, x, y):

        if not self.current_x:
            self.current_x.append(x)
            self.current_y.append(y)
        else:
            x_prev = self.current_x[-1]
            y_prev = self.current_y[-1]

            dx = abs(x - x_prev)
            dy = abs(y - y_prev)
            threshold = 1.0  

            if dx > threshold or dy > threshold:

                self.current_x.append(np.nan)
                self.current_y.append(np.nan)

            self.current_x.append(x)
            self.current_y.append(y)

        self.curve_current.setData(self.current_x, self.current_y)

        self.xCurrentPos.display(x)
        self.yCurrentPos.display(y)

    def update_Velocity(self, linear, angular):
        self.linearVel.display(linear)
        self.angularVel.display(angular)

    def light_color_stop(self):
        self.stopIndicator.setStyleSheet("background-color: gold;"
                                         "border-radius: 35px;" 
                                         "border: 2px solid black;")
        self.runIndicator.setStyleSheet("background-color: lightgray;"
                                         "border-radius: 35px;" 
                                         "border: 2px solid black;")
        self.errorIndicator.setStyleSheet("background-color: lightgray;"
                                         "border-radius: 35px;" 
                                         "border: 2px solid black;")
    def light_color_run(self):
        self.runIndicator.setStyleSheet("background-color: lime;"
                                         "border-radius: 35px;" 
                                         "border: 2px solid black;")
        
        self.stopIndicator.setStyleSheet("background-color: lightgray;"
                                         "border-radius: 35px;" 
                                         "border: 2px solid black;")
        
        self.errorIndicator.setStyleSheet("background-color: lightgray;"
                                         "border-radius: 35px;" 
                                         "border: 2px solid black;")
        
    def light_color_error(self):
        self.errorIndicator.setStyleSheet("background-color: red;"
                                         "border-radius: 35px;" 
                                         "border: 2px solid black;")
        self.runIndicator.setStyleSheet("background-color: lightgray;"
                                         "border-radius: 35px;" 
                                         "border: 2px solid black;")
        self.stopIndicator.setStyleSheet("background-color: lightgray;"
                                         "border-radius: 35px;" 
                                         "border: 2px solid black;")