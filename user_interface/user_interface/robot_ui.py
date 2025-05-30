from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QFileDialog
from PyQt5.uic import loadUi

import pyqtgraph as pg
import random
from pyqtgraph.Qt import QtCore
import csv
import sys

class RobotUI(QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi("/home/liemtran/liem_ws/src/user_interface/ui/test.ui", self)
        
        self.setWindowTitle("Mobile Robot Control")
        self.setGeometry(100, 100, 1300, 900)  # Set window size and position


        self.stopButton.pressed.connect(self.stop)
        self.forwardButton.pressed.connect(self.forward_button_press)
        self.leftButton.pressed.connect(self.left_button_press)
        self.rightButton.pressed.connect(self.right_button_press)
        self.backwardButton.pressed.connect(self.backward_button_press)


        self.forwardButton.released.connect(self.stop)
        self.leftButton.released.connect(self.stop)
        self.rightButton.released.connect(self.stop)
        self.backwardButton.released.connect(self.stop)


        self.sendGoalButton.clicked.connect(self.sendGoal_clicked)
        self.pauseButton.clicked.connect(self.pauseButton_clicked)
        

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
        # self.plot_widget.setTitle("Robot Waypoint Tracking")
        self.plot_widget.setAspectLocked(True) # Giữ tỉ lệ khung hình cố định

        self.setpoint_x= []
        self.setpoint_y= []
        self.current_x = []
        self.current_y = []
        self.plot_widget.addLegend(offset=(0, 10))
        # Tạo plot
        self.curve_setpoint = self.plot_widget.plot(self.setpoint_x, self.setpoint_y,
                                                    pen=pg.mkPen('b', width=2), name="Setpoint")
        self.curve_current = self.plot_widget.plot(self.current_x, self.current_y,
                                                pen=pg.mkPen('g', width=2), name="Current")

        # Internal time counter
        self.t = 0

        # self.load_csv_data("waypoints.csv")
        self.loadCSVButton.clicked.connect(self.choose_csv_file)
        self.clearGraphButton.clicked.connect(self.clearGraph)

    def clearGraph(self):
        self.curve_setpoint.setData([], [])
        self.curve_current.setData([], [])

    def choose_csv_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open CSV File", "", "CSV Files (*.csv)")
        if file_path:
            self.load_csv_data(file_path)

            # self.pathLabel.setWordWrapMode(QTextOption.WordWrap)
            self.pathLabel.setText(file_path)  # Hiển thị đường dẫn file đã chọn
            self.pathLabel.adjustSize()  # Tự động điều chỉnh kích thước nhãn


    def load_csv_data(self, file_path):
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
        
        self.xCurrentPos.display(x)
        self.yCurrentPos.display(y)
        self.current_x.append(x)
        self.current_y.append(y)
        self.curve_current.setData(self.current_x, self.current_y)



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

    def stop(self):
        self.light_color_stop()
    
        self.update_plot()
    def forward_button_press(self):
        self.light_color_run()

    def left_button_press(self):
        self.light_color_run()

    def right_button_press(self):
        self.light_color_run()

    def backward_button_press(self):
        self.light_color_run()

    def sendGoal_clicked(self):
        self.light_color_run()

    def pauseButton_clicked(self):

        if self.pauseButton.text() == "Pause":
            self.pauseButton.setText("Resume")

            self.light_color_stop()
        else:
            self.pauseButton.setText("Pause")

            self.light_color_run()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotUI()
    window.show()
    sys.exit(app.exec_())