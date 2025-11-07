from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QFileDialog, QMessageBox
from PyQt5.uic import loadUi
import numpy as np
import pyqtgraph as pg
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from datetime import datetime
from reportlab.lib.pagesizes import A4
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, Paragraph, Spacer
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet
import pandas as pd
import mysql.connector
from mysql.connector import Error
import csv
import os  # For env vars

def get_mysql_config():
    """Get MySQL config from env vars for security."""
    # Use os.environ.get('MYSQL_HOST', 'localhost') in production
    return {
        'host': 'localhost',
        'user': 'liemtran',
        'password': 'Liempk1234@',
        'database': 'test1'
    }

class RobotUI(QMainWindow):
    def __init__(self):
        super().__init__()
        ui_path = "/home/liemtran/liem_ws/src/user_interface/ui/projectD.ui"  # or final.ui
        if not os.path.exists(ui_path):
            QMessageBox.critical(self, "Error", f"UI file not found: {ui_path}")
            raise FileNotFoundError(f"UI file missing: {ui_path}")
        loadUi(ui_path, self)

        self.setWindowTitle("Mobile Robot Control")
        self.setGeometry(100, 100, 1200, 659)
        self.light_color_stop()

        # Plot setup
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setLabel('left', 'Y Axis', units='m')
        self.plot_widget.setLabel('bottom', 'X Axis', units='m')
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.addLegend(offset=(0, 10))
        plot_layout = QVBoxLayout()
        plot_layout.addWidget(self.plot_widget)
        self.trackingPlot.setLayout(plot_layout)

        self.setpoint_x = []
        self.setpoint_y = []
        self.current_x = []
        self.current_y = []
        # self.error_x = []
        # self.error_y = []
        self.curve_setpoint = self.plot_widget.plot(self.setpoint_x, self.setpoint_y,
                                                     pen=pg.mkPen(color=(0,0,255), width=4), name="Target Path")  #Waypoints
        self.curve_current = self.plot_widget.plot(self.current_x, self.current_y,
                                                   pen=pg.mkPen(color=(0,255,0), width=4), name="Current Position")
        # self.curve_error = self.plot_widget.plot(self.error_x, self.error_y,
        #                                          pen=pg.mkPen(color=(255,0,0), width=1), name="Error")

        # Error plot
        self.error_plot_widget = pg.PlotWidget()
        self.error_plot_widget.setBackground('w')
        self.error_plot_widget.setLabel('left', 'Error (m)')
        self.error_plot_widget.setLabel('bottom', 'Time (0.1s)')
        self.error_plot_widget.addLegend(offset=(0, 10))
        self.error_plot_curve = self.error_plot_widget.plot([], [],
                                                            pen=pg.mkPen(color=(255,0,0), width=3), name="Error")
        error_layout = QVBoxLayout()
        error_layout.addWidget(self.error_plot_widget)
        self.errorPlot.setLayout(error_layout)
        self.error_history = []

        # Database setup
        self.mydb_init()
        self.clearGraphButton.clicked.connect(self.clearGraph)
        self.tableNameTextEdit.setText(self.table)

        # Filters
        self.dateFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.timeFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.errorFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.linearFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.angularFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.filterButton.clicked.connect(self.filter_data)
        self.exportButton.clicked.connect(self.export_to_pdf)
        self.exportExcelButton.clicked.connect(self.export_to_excel)

        # Batch insert buffer
        self.db_buffer = []
        self.batch_size = 10  # Insert every 10 updates

    def mydb_init(self):
        """Initialize MySQL connection and table."""
        try:
            today_str = datetime.now().strftime("%Y%m%d")
            self.table = f"data_{today_str}"
            config = get_mysql_config()
            self.connection = mysql.connector.connect(**config)
            self.connection.autocommit = False  # For batch commits
            if self.connection.is_connected():
                self.dataLabel.setText("✅ Successfully connected to MySQL")
                self.cursor = self.connection.cursor()
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
                self.connection.commit()
        except Error as e:
            self.dataLabel.setText(f"❌ MySQL Error: {e}")
            QMessageBox.critical(self, "MySQL Error", str(e))

    def update_error_plot(self, error_value):
        """Update error history plot."""
        self.error_history.append(error_value)
        self.error_plot_curve.setData(list(range(len(self.error_history))), self.error_history)

    def toggle_filters(self):
        """Enable/disable filter inputs based on checkboxes."""
        self.dateEdit.setEnabled(self.dateFilterCheckBox.isChecked())
        self.timeEdit.setEnabled(self.timeFilterCheckBox.isChecked())
        self.errorEdit.setEnabled(self.errorFilterCheckBox.isChecked())
        self.linearEdit.setEnabled(self.linearFilterCheckBox.isChecked())
        self.angularEdit.setEnabled(self.angularFilterCheckBox.isChecked())

    def filter_data(self):
        """Filter and display data based on user inputs."""
        table_name = self.tableNameTextEdit.toPlainText()
        try:
            self.cursor.execute(f"SHOW TABLES LIKE '{table_name}'")
            if not self.cursor.fetchone():
                raise ValueError(f"Table '{table_name}' does not exist.")

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

            query = f"SELECT * FROM {table_name}"
            if conditions:
                query += " WHERE " + " AND ".join(conditions)

            self.cursor.execute(query)
            rows = self.cursor.fetchall()

            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(["Date", "X", "Y", "Error", "Linear Vel", "Angular Vel"])
            for row in rows:
                items = [QStandardItem(str(cell)) for cell in row]
                model.appendRow(items)

            self.dataTableView.setModel(model)
            self.dataTableView.resizeColumnsToContents()
            self.dataTableView.resizeRowsToContents()
        except Error as e:
            self.dataLabel.setText(f"❌ Error filtering data: {e}")
            QMessageBox.warning(self, "Filter Error", str(e))

    def insert_data(self, x, y, error, linear_vel, angular_vel):
        """Buffer data for batch insert to DB."""
        self.db_buffer.append((x, y, error, linear_vel, angular_vel))
        if len(self.db_buffer) >= self.batch_size:
            self._flush_db_buffer()

    def _flush_db_buffer(self):
        """Commit buffered data to DB."""
        if not self.db_buffer:
            return
        table_name = self.tableNameTextEdit.toPlainText()
        try:
            self.cursor.execute(f"SHOW TABLES LIKE '{table_name}'")
            if self.cursor.fetchone():
                query = f"INSERT INTO {table_name} (x, y, error, linear_vel, angular_vel) VALUES (%s, %s, %s, %s, %s)"
                self.cursor.executemany(query, self.db_buffer)
                self.connection.commit()
                self.db_buffer.clear()
        except Error as e:
            self.dataLabel.setText(f"❌ DB Insert Error: {e}")

    def show_data(self):
        """Display all data from table in UI."""
        table_name = self.tableNameTextEdit.toPlainText()
        try:
            self.cursor.execute(f"SHOW TABLES LIKE '{table_name}'")
            if not self.cursor.fetchone():
                raise ValueError(f"Table '{table_name}' does not exist.")

            self.cursor.execute(f"SHOW COLUMNS FROM {table_name}")
            columns = [column[0] for column in self.cursor.fetchall()]

            self.cursor.execute(f"SELECT * FROM {table_name}")
            rows = self.cursor.fetchall()

            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(columns)
            for row in rows:
                items = [QStandardItem(str(cell)) for cell in row]
                model.appendRow(items)

            self.dataTableView.setModel(model)
            self.dataTableView.resizeColumnsToContents()
            self.dataTableView.resizeRowsToContents()
            self.dataTableView.scrollToBottom()
        except Error as e:
            self.dataLabel.setText(f"❌ Error showing data: {e}")

    def clearGraph(self):
        """Clear all plots and histories."""
        self.setpoint_x.clear()
        self.setpoint_y.clear()
        self.current_x.clear()
        self.current_y.clear()
        self.error_history.clear()
        self.error_plot_curve.setData([], [])
        self.curve_setpoint.setData([], [])
        self.curve_current.setData([], [])

    def plot_setpoint(self, file_path):
        """Load and plot setpoints from CSV."""
        self.setpoint_x = []
        self.setpoint_y = []
        try:
            with open(file_path, 'r') as file:
                reader = csv.reader(file)
                header = next(reader)
                if 'x' not in header or 'y' not in header:
                    raise ValueError("CSV must have 'x' and 'y' columns.")
                for row in reader:
                    if len(row) >= 2:
                        try:
                            x = float(row[0])
                            y = float(row[1])
                            self.setpoint_x.append(x)
                            self.setpoint_y.append(y)
                        except ValueError:
                            continue
            self.num_points = len(self.setpoint_x)
            self.curve_setpoint.setData(self.setpoint_x, self.setpoint_y)
        except Exception as e:
            QMessageBox.warning(self, "CSV Error", f"Failed to load CSV: {e}")



    def update_current_position(self, x, y):
        """Update current position plot, insert NaN on large jumps."""
        if not self.current_x:
            self.current_x.append(x)
            self.current_y.append(y)
        else:
            x_prev = self.current_x[-1]
            y_prev = self.current_y[-1]
            dx = abs(x - x_prev)
            dy = abs(y - y_prev)
            threshold = 1.0  # Adjustable
            if dx > threshold or dy > threshold:
                self.current_x.append(np.nan)
                self.current_y.append(np.nan)
            self.current_x.append(x)
            self.current_y.append(y)
        self.curve_current.setData(self.current_x, self.current_y)
        self.xCurrentPos.display(x)
        self.yCurrentPos.display(y)

    # def update_error(self, current_x, waypoint_x, current_y, waypoint_y):
    #     """Update error line plot."""
    #     self.error_x.extend([np.nan, current_x, waypoint_x])
    #     self.error_y.extend([np.nan, current_y, waypoint_y])
    #     self.curve_error.setData(self.error_x, self.error_y)

    def update_Velocity(self, linear, angular):
        """Update velocity displays."""
        self.linearVel.display(linear)
        self.angularVel.display(angular)

    def export_to_pdf(self):
        """Export table to PDF."""
        file_path, _ = QFileDialog.getSaveFileName(self, "Save PDF", "", "PDF Files (*.pdf)")
        if not file_path:
            return
        try:
            doc = SimpleDocTemplate(file_path, pagesize=A4)
            elements = []
            styles = getSampleStyleSheet()
            elements.extend([
                Paragraph("Project D - Robot Log Report", styles["Title"]),
                Paragraph(f"Reporter: {self.reporterTextBox.toPlainText()} ID: {self.idTextBox.toPlainText()}", styles["Normal"]),
                Paragraph(f"Export Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", styles["Normal"]),
                Paragraph(f"Table Name: {self.tableNameTextEdit.toPlainText()}", styles["Normal"]),
                Spacer(1, 12),
            ])
            model = self.dataTableView.model()
            if model is None:
                raise ValueError("No data to export")
            headers = [model.headerData(col, pg.QtCore.Qt.Horizontal) for col in range(model.columnCount())]
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
            table.setStyle(TableStyle([
                ("GRID", (0, 0), (-1, -1), 0.5, colors.black),
                ("BACKGROUND", (0, 0), (-1, 0), colors.lightblue),
                ("ALIGN", (0, 0), (-1, -1), "CENTER"),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("FONTSIZE", (0, 0), (-1, -1), 9),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
            ]))
            elements.append(table)
            doc.build(elements)
            self.dataLabel.setText(f"✅ Exported to PDF: {file_path}")
        except Exception as e:
            self.dataLabel.setText(f"❌ PDF Export Error: {e}")
            QMessageBox.warning(self, "Export Error", str(e))

    def export_to_excel(self):
        """Export table to CSV (Excel compatible)."""
        file_path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV Files (*.csv)")
        if not file_path:
            return
        if not file_path.lower().endswith('.csv'):
            file_path += '.csv'
        try:
            model = self.dataTableView.model()
            if model is None:
                raise ValueError("No data to export")
            headers = [model.headerData(col, pg.QtCore.Qt.Horizontal) for col in range(model.columnCount())]
            data = []
            for row in range(model.rowCount()):
                row_data = []
                for col in range(model.columnCount()):
                    index = model.index(row, col)
                    value = str(model.data(index))
                    row_data.append(value)
                data.append(row_data)
            df = pd.DataFrame(data, columns=headers)
            df.to_csv(file_path, index=False)
            self.dataLabel.setText(f"✅ Exported to CSV: {file_path}")
        except Exception as e:
            self.dataLabel.setText(f"❌ CSV Export Error: {e}")
            QMessageBox.warning(self, "Export Error", str(e))

    def light_color_stop(self):
        """Set UI indicators to stop state."""
        self.stopIndicator.setStyleSheet("background-color: gold; border-radius: 35px; border: 2px solid black;")
        self.runIndicator.setStyleSheet("background-color: lightgray; border-radius: 35px; border: 2px solid black;")
        self.errorIndicator.setStyleSheet("background-color: lightgray; border-radius: 35px; border: 2px solid black;")

    def light_color_run(self):
        """Set UI indicators to run state."""
        self.runIndicator.setStyleSheet("background-color: lime; border-radius: 35px; border: 2px solid black;")
        self.stopIndicator.setStyleSheet("background-color: lightgray; border-radius: 35px; border: 2px solid black;")
        self.errorIndicator.setStyleSheet("background-color: lightgray; border-radius: 35px; border: 2px solid black;")

    def light_color_error(self):
        """Set UI indicators to error state."""
        self.errorIndicator.setStyleSheet("background-color: red; border-radius: 35px; border: 2px solid black;")
        self.runIndicator.setStyleSheet("background-color: lightgray; border-radius: 35px; border: 2px solid black;")
        self.stopIndicator.setStyleSheet("background-color: lightgray; border-radius: 35px; border: 2px solid black;")

    def closeEvent(self, event):
        """Clean up on close."""
        self._flush_db_buffer()  # Commit any remaining buffer
        if hasattr(self, 'connection') and self.connection.is_connected():
            self.cursor.close()
            self.connection.close()
        event.accept()