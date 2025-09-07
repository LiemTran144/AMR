from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QFileDialog
from PyQt5.uic import loadUi
import numpy as np
import pyqtgraph as pg
from PyQt5.QtSql import QSqlDatabase, QSqlQuery
from pyqtgraph.Qt import QtCore
import csv

import mysql.connector
from mysql.connector import Error
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from datetime import datetime
from reportlab.lib.pagesizes import A4
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, Paragraph, Spacer
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet
import pandas as pd

def mydb(self):
    try:
        # 1. Tạo tên bảng theo ngày
        today_str = datetime.now().strftime("%Y%m%d")
        self.table = f"data_{today_str}"

        # 2. Kết nối tới MySQL
        self.connection = mysql.connector.connect(
            host='localhost',
            user='liemtran',
            password='Liempk1234@',
            database='test1'
        )
        self.connection.autocommit = True

        if self.connection.is_connected():
            self.dataLabel.setText("✅ Successfully connect to MySQL")
            self.dataLabel.adjustSize()

            self.cursor = self.connection.cursor()

            # 3. Tạo bảng nếu chưa tồn tại
            self.cursor.execute(f"""
                CREATE TABLE IF NOT EXISTS {self.table} (
                    date DATETIME DEFAULT CURRENT_TIMESTAMP,
                    x DOUBLE,
                    y DOUBLE,
                    error DOUBLE,
                    linear_vel DOUBLE,
                    angular_vel DOUBLE
                )
            """)
    except mysql.connector.Error as e:
        print(f"❌ Lỗi MySQL: {e}")

class RobotUI(QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi("/home/liemtran/liem_ws/src/user_interface/ui/projectD.ui", self)
        # loadUi("/home/liemtran/liem_ws/src/user_interface/ui/final.ui", self)

        self.setWindowTitle("Mobile Robot Control")
        self.setGeometry(100, 100, 1300, 1000)  # Set window size and position        
        self.light_color_stop()

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')  # 'w' viết tắt cho white
        plot_layout = QVBoxLayout()              # Tạo layout
        plot_layout.addWidget(self.plot_widget)            # Thêm plot vào layout
        self.trackingPlot.setLayout(plot_layout)           # Gán layout vào groupbox

        # Gán tên cho từng trục
        self.plot_widget.setLabel('left', 'Y Axis', units='m')       # trục Y
        self.plot_widget.setLabel('bottom', 'X Axis', units='m')     # trục X

        # self.plot_widget.setLabel('left', 'Y(m)')       # trục Ys
        # self.plot_widget.setLabel('bottom', 'X(m)')     # trục X        
        self.plot_widget.setAspectLocked(True) # Giữ tỉ lệ khung hình cố định

        self.setpoint_x= []
        self.setpoint_y= []
        self.current_x = []
        self.current_y = []
        self.plot_widget.addLegend(offset=(0, 10))
        # Tạo plot
        self.curve_setpoint = self.plot_widget.plot(self.setpoint_x, self.setpoint_y,
                                                    pen=pg.mkPen(color=(0,0,255), width=4), name="Waypoints")
        self.curve_current = self.plot_widget.plot(self.current_x, self.current_y,
                                                pen=pg.mkPen(color=(0,255,0), width=4), name="Current Position")
        
        self.error_x = []
        self.error_y = []
        self.curve_error = self.plot_widget.plot(self.error_x,self.error_y ,
                                                pen=pg.mkPen(color=(255,0,0), width=1), name="Error")
        

        #plot error
        self.error_plot_widget = pg.PlotWidget()
        self.error_plot_widget.setBackground('w')
        self.error_plot_widget.setLabel('left', 'Error (m)')
        self.error_plot_widget.setLabel('bottom', 'Index')
        self.error_plot_widget.addLegend(offset=(0, 10))
        self.error_plot_curve = self.error_plot_widget.plot([], [],
                                                            pen=pg.mkPen(color=(255,0,0), width=3),
                                                            name="Error")
        # Thêm error_plot_widget vào một layout hoặc groupbox trên UI
        self.errorLayout = QVBoxLayout() 
        self.errorLayout.addWidget(self.error_plot_widget)  # errorPlotLayout là QVBoxLayout trong .ui
        self.errorPlot.setLayout(self.errorLayout)
        self.error_history = []





        #database
        mydb(self)
        self.clearGraphButton.clicked.connect(self.clearGraph)
        self.tableNameTextEdit.setText(self.table)

        self.dateFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.timeFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.errorFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.linearFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.angularFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.filterButton.clicked.connect(self.filter_data)
        self.exportButton.clicked.connect(self.export_to_pdf)
        self.exportExcelButton.clicked.connect(self.export_to_excel)


    def update_error_plot(self, error_value):
        self.error_history.append(error_value)
        self.error_plot_curve.setData(list(range(len(self.error_history))), self.error_history)

    def toggle_filters(self):
        self.dateEdit.setEnabled(self.dateFilterCheckBox.isChecked())
        self.timeEdit.setEnabled(self.timeFilterCheckBox.isChecked())
        self.errorEdit.setEnabled(self.errorFilterCheckBox.isChecked())
        self.linearEdit.setEnabled(self.linearFilterCheckBox.isChecked())
        self.angularEdit.setEnabled(self.angularFilterCheckBox.isChecked())


    def filter_data(self):
        table_name = self.tableNameTextEdit.toPlainText()
        conditions = []

        if self.dateFilterCheckBox.isChecked():
            date_str = self.dateEdit.date().toString('yyyy-MM-dd')
            conditions.append(f"DATE(date) = '{date_str}'")

        if self.timeFilterCheckBox.isChecked():
            time_str = self.timeEdit.time().toString("HH:mm:ss")
            conditions.append(f"TIME(date) >= '{time_str}'")

        if self.errorFilterCheckBox.isChecked():
            error_cond = self.errorEdit.toPlainText().strip()
            if error_cond:
                conditions.append(f"error {error_cond}")

        if self.linearFilterCheckBox.isChecked():
            linear_cond = self.linearEdit.toPlainText().strip()
            if linear_cond:
                conditions.append(f"linear_vel {linear_cond}")

        if self.angularFilterCheckBox.isChecked():
            angular_cond = self.angularEdit.toPlainText().strip()
            if angular_cond:
                conditions.append(f"angular_vel {angular_cond}")
        query = f"SELECT * FROM {table_name} WHERE " + " AND ".join(conditions)
        if not conditions:
            query = f"SELECT * FROM {table_name}"  # Nếu không có điều kiện, lấy tất cả dữ liệu
        try:
            self.cursor.execute(query)
            rows = self.cursor.fetchall()

            # Cập nhật bảng
            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(["Date", "X", "Y", "Error", "Linear Vel", "Angular Vel"])
            for row in rows:
                items = [QStandardItem(str(cell)) for cell in row]
                model.appendRow(items)

            self.dataTableView.setModel(model)
            self.dataTableView.resizeColumnsToContents()
            self.dataTableView.resizeRowsToContents()

        except Exception as e:
            #english
            self.dataLabel.setText(f"❌ Error filtering data: {e}")
            self.dataLabel.adjustSize()

    def insert_data(self, x, y, error, linear_vel, angular_vel):
        table_name = self.tableNameTextEdit.toPlainText()

        # Kiểm tra xem bảng có tồn tại không
        self.cursor.execute(f"SHOW TABLES LIKE '{table_name}'")
        result = self.cursor.fetchone()

        if result:
            # Chèn dữ liệu vào bảng
            self.cursor.execute(f"""
                INSERT INTO {table_name} (x, y, error, linear_vel, angular_vel)
                VALUES (%s, %s, %s, %s, %s)
            """, (x, y, error, linear_vel, angular_vel))

    def show_data(self):
        table_name = self.tableNameTextEdit.toPlainText()

        # Kiểm tra xem bảng có tồn tại không
        self.cursor.execute(f"SHOW TABLES LIKE '{table_name}'")
        result = self.cursor.fetchone()

        if result:
            # Lấy danh sách các cột từ bảng
            self.cursor.execute(f"SHOW COLUMNS FROM {table_name}")
            columns = [column[0] for column in self.cursor.fetchall()]  # Lấy tên các cột

            # Lấy dữ liệu từ bảng
            self.cursor.execute(f"SELECT * FROM {table_name}")
            rows = self.cursor.fetchall()

            # Tạo model và gán vào QTableView
            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(columns)  # Sử dụng danh sách cột làm header

            for row in rows:
                items = [QStandardItem(str(cell)) for cell in row]
                model.appendRow(items)

            self.dataTableView.setModel(model)
            self.dataTableView.resizeColumnsToContents()
            self.dataTableView.resizeRowsToContents()
            self.dataTableView.scrollToBottom()

    def clearGraph(self):
        self.setpoint_x.clear()
        self.setpoint_y.clear()
        self.current_x.clear()
        self.current_y.clear()
        self.curve_setpoint.setData([], [])
        self.curve_current.setData([], [])
        self.pathLabel.clear()

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
    
    def update_error(self, current_x, waypoint_x, current_y, waypoint_y):
        # Thêm đoạn đường mới vào danh sách
        self.error_x.extend([np.nan, current_x, waypoint_x])  # Sử dụng np.nan để ngắt đoạn
        self.error_y.extend([np.nan, current_y, waypoint_y])

        # Cập nhật dữ liệu lên đồ thị
        self.curve_error.setData(self.error_x, self.error_y)

    def update_Velocity(self, linear, angular):
        self.linearVel.display(linear)
        self.angularVel.display(angular)

    def export_to_pdf(self):
        """Export table data to a PDF file."""
        file_path, _ = QFileDialog.getSaveFileName(self, "Save PDF", "", "PDF Files (*.pdf)")
        if not file_path:
            return

        doc = SimpleDocTemplate(file_path, pagesize=A4)
        elements = []
        styles = getSampleStyleSheet()

        elements.extend(
            [
                Paragraph("Project D - Robot Log Report", styles["Title"]),
                Paragraph(f"Reporter: {self.reporterTextBox.toPlainText()} ID: {self.idTextBox.toPlainText()}", styles["Normal"]),
                Paragraph(f"Export Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", styles["Normal"]),
                Paragraph(f"Table Name: {self.tableNameTextEdit.toPlainText()}", styles["Normal"]),
                Spacer(1, 12),
            ]
        )

        model = self.dataTableView.model()
        if model is None:
            self.dataLabel.setText("❌ No data to export")
            self.dataLabel.adjustSize()
            return

        headers = [model.headerData(col, QtCore.Qt.Horizontal) for col in range(model.columnCount())]
        data = [headers]

        for row in range(model.rowCount()):
            row_data = []
            for col in range(model.columnCount()):
                index = model.index(row, col)
                value = str(model.data(index))
                try:
                    value = f"{float(value):.4f}"
                except ValueError:
                    pass
                row_data.append(value)
            data.append(row_data)

        table = Table(data, repeatRows=1)
        table.setStyle(
            TableStyle(
                [
                    ("GRID", (0, 0), (-1, -1), 0.5, colors.black),
                    ("BACKGROUND", (0, 0), (-1, 0), colors.lightblue),
                    ("ALIGN", (0, 0), (-1, -1), "CENTER"),
                    ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                    ("FONTSIZE", (0, 0), (-1, -1), 9),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
                ]
            )
        )

        elements.append(table)
        doc.build(elements)
        self.dataLabel.setText(f"✅ Exported to PDF: {file_path}")
        self.dataLabel.adjustSize()

    def export_to_excel(self):
        """Export table data to a CSV file."""
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save CSV", "", "CSV Files (*.csv)"
        )
        if not file_path:
            return

        # Đảm bảo file có đuôi .csv
        if not file_path.lower().endswith('.csv'):
            file_path += '.csv'

        model = self.dataTableView.model()
        if model is None:
            self.dataLabel.setText("❌ No data to export")
            self.dataLabel.adjustSize()
            return

        # Lấy header và dữ liệu
        headers = [model.headerData(col, QtCore.Qt.Horizontal) for col in range(model.columnCount())]
        data = []
        for row in range(model.rowCount()):
            row_data = []
            for col in range(model.columnCount()):
                index = model.index(row, col)
                value = str(model.data(index))
                row_data.append(value)
            data.append(row_data)

        # Ghi ra file CSV
        try:
            df = pd.DataFrame(data, columns=headers)
            df.to_csv(file_path, index=False)
            self.dataLabel.setText(f"✅ Exported to CSV: {file_path}")
        except Exception as e:
            self.dataLabel.setText(f"❌ Error exporting to CSV: {e}")
        self.dataLabel.adjustSize()

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
    def closeEvent(self, event):
        # Đóng kết nối MySQL nếu đang mở
        if hasattr(self, 'connection') and self.connection.is_connected():
            self.cursor.close()
            self.connection.close()
        event.accept()