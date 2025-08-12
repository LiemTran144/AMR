#! /usr/bin/env python3
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QFileDialog
from PyQt5.uic import loadUi
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import csv
import mysql.connector
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from datetime import datetime
from reportlab.lib.pagesizes import A4
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, Paragraph, Spacer
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet

# Constants
INDICATOR_STYLE = (
    "border-radius: 35px;"
    "border: 2px solid black;"
)
COLORS = {
    "stop": "gold",
    "run": "lime",
    "error": "red",
    "default": "lightgray",
}
PLOT_THRESHOLD = 1.0

class RobotUI(QMainWindow):
    """Main UI class for controlling and visualizing a mobile robot."""

    def __init__(self):
        super().__init__()
        loadUi("/home/liemtran/liem_ws/src/user_interface/ui/projectD.ui", self)
        self.setWindowTitle("Mobile Robot Control")
        self.setGeometry(100, 100, 1300, 1000)

        # Initialize plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("w")
        plot_layout = QVBoxLayout()
        plot_layout.addWidget(self.plot_widget)
        self.plotGroupBox.setLayout(plot_layout)

        # Set plot labels and properties
        self.plot_widget.setLabel("left", "Y")
        self.plot_widget.setLabel("bottom", "X")
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.addLegend(offset=(0, 10))

        # Initialize data lists
        self.setpoint_x = []
        self.setpoint_y = []
        self.current_x = []
        self.current_y = []
        self.error_x = []
        self.error_y = []

        # Initialize plot curves
        self.curve_setpoint = self.plot_widget.plot(
            self.setpoint_x,
            self.setpoint_y,
            pen=pg.mkPen(color=(0, 0, 255), width=4),
            name="Waypoints (Setpoint)",
        )
        self.curve_current = self.plot_widget.plot(
            self.current_x,
            self.current_y,
            pen=pg.mkPen(color=(0, 255, 0), width=4),
            name="Current_Position",
        )
        self.curve_error = self.plot_widget.plot(
            self.error_x,
            self.error_y,
            pen=pg.mkPen(color=(255, 0, 0), width=4),
            name="Error",
        )

        # Initialize database
        self.connect_to_database()

        # Connect UI signals
        self.clearGraphButton.clicked.connect(self.clear_graph)
        self.tableNameTextEdit.setText(self.table)
        self.dateFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.timeFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.errorFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.linearFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.angularFilterCheckBox.stateChanged.connect(self.toggle_filters)
        self.filterButton.clicked.connect(self.filter_data)
        self.exportButton.clicked.connect(self.export_to_pdf)

        # Set initial indicator state
        self.set_indicator_state("stop")

    # Database Methods
    def connect_to_database(self):
        """Connect to MySQL database and create table if it doesn't exist."""
        try:
            today_str = datetime.now().strftime("%Y%m%d")
            self.table = f"data_{today_str}"

            self.connection = mysql.connector.connect(
                host="localhost",
                user="liemtran",
                password="Liempk1234@",
                database="test1",
            )
            self.connection.autocommit = True

            if self.connection.is_connected():
                self.dataLabel.setText("✅ Successfully connected to MySQL")
                self.dataLabel.adjustSize()

                self.cursor = self.connection.cursor()
                self.cursor.execute(
                    f"""
                    CREATE TABLE IF NOT EXISTS {self.table} (
                        date DATETIME DEFAULT CURRENT_TIMESTAMP,
                        x DOUBLE,
                        y DOUBLE,
                        error DOUBLE,
                        linear_vel DOUBLE,
                        angular_vel DOUBLE
                    )
                    """
                )
        except mysql.connector.Error as e:
            self.dataLabel.setText(f"❌ MySQL Error: {e}")
            self.dataLabel.adjustSize()

    def insert_data(self, x, y, error, linear_vel, angular_vel):
        """Insert data into the database."""
        table_name = self.tableNameTextEdit.toPlainText()
        rounded_values = (
            round(x, 4),
            round(y, 4),
            round(error, 4),
            round(linear_vel, 4),
            round(angular_vel, 4),
        )

        try:
            self.cursor.execute(f"SHOW TABLES LIKE '{table_name}'")
            if self.cursor.fetchone():
                self.cursor.execute(
                    f"""
                    INSERT INTO {table_name} (x, y, error, linear_vel, angular_vel)
                    VALUES (%s, %s, %s, %s, %s)
                    """,
                    rounded_values,
                )
        except mysql.connector.Error as e:
            self.dataLabel.setText(f"❌ Error inserting data: {e}")
            self.dataLabel.adjustSize()

    def show_data(self):
        """Display data from the database in the table view."""
        table_name = self.tableNameTextEdit.toPlainText()

        try:
            self.cursor.execute(f"SHOW TABLES LIKE '{table_name}'")
            if self.cursor.fetchone():
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
        except mysql.connector.Error as e:
            self.dataLabel.setText(f"❌ Error fetching data: {e}")
            self.dataLabel.adjustSize()

    # Plotting Methods
    def plot_setpoint(self, file_path):
        """Plot setpoint data from a CSV file."""
        self.setpoint_x = []
        self.setpoint_y = []

        try:
            with open(file_path, "r") as file:
                reader = csv.reader(file)
                next(reader)  # Skip header
                for row in reader:
                    if len(row) >= 2:
                        try:
                            self.setpoint_x.append(float(row[0]))
                            self.setpoint_y.append(float(row[1]))
                        except ValueError:
                            continue
            self.num_points = len(self.setpoint_x)
            self.curve_setpoint.setData(self.setpoint_x, self.setpoint_y)
        except Exception as e:
            self.dataLabel.setText(f"❌ Error plotting setpoint: {e}")
            self.dataLabel.adjustSize()

    def update_current_position(self, x, y):
        """Update the current position on the plot."""
        if not self.current_x:
            self.current_x.append(x)
            self.current_y.append(y)
        else:
            x_prev, y_prev = self.current_x[-1], self.current_y[-1]
            if abs(x - x_prev) > PLOT_THRESHOLD or abs(y - y_prev) > PLOT_THRESHOLD:
                self.current_x.append(np.nan)
                self.current_y.append(np.nan)
            self.current_x.append(x)
            self.current_y.append(y)

        self.curve_current.setData(self.current_x, self.current_y)
        self.xCurrentPos.display(x)
        self.yCurrentPos.display(y)

    def update_error(self, current_x, waypoint_x, current_y, waypoint_y):
        """Update the error line on the plot."""
        self.error_x.extend([np.nan, current_x, waypoint_x])
        self.error_y.extend([np.nan, current_y, waypoint_y])
        self.curve_error.setData(self.error_x, self.error_y)

    def clear_graph(self):
        """Clear all data from the plot."""
        self.setpoint_x.clear()
        self.setpoint_y.clear()
        self.current_x.clear()
        self.current_y.clear()
        self.error_x.clear()
        self.error_y.clear()
        self.curve_setpoint.setData([], [])
        self.curve_current.setData([], [])
        self.curve_error.setData([], [])
        self.pathLabel.clear()

    # UI Methods
    def set_indicator_state(self, state):
        """Set the state of the indicator lights."""
        for indicator, color in [
            (self.stopIndicator, "stop"),
            (self.runIndicator, "run"),
            (self.errorIndicator, "error"),
        ]:
            indicator_color = COLORS[state] if indicator == getattr(self, f"{state}Indicator") else COLORS["default"]
            indicator.setStyleSheet(f"background-color: {indicator_color};{INDICATOR_STYLE}")

    def update_velocity(self, linear, angular):
        """Update the displayed velocity values."""
        self.linearVel.display(linear)
        self.angularVel.display(angular)

    def toggle_filters(self):
        """Enable or disable filter inputs based on checkbox state."""
        self.dateEdit.setEnabled(self.dateFilterCheckBox.isChecked())
        self.timeEdit.setEnabled(self.timeFilterCheckBox.isChecked())
        self.errorEdit.setEnabled(self.errorFilterCheckBox.isChecked())
        self.linearEdit.setEnabled(self.linearFilterCheckBox.isChecked())
        self.angularEdit.setEnabled(self.angularFilterCheckBox.isChecked())

    def filter_data(self):
        """Filter data in the table view based on user inputs."""
        table_name = self.tableNameTextEdit.toPlainText()
        conditions = []

        if self.dateFilterCheckBox.isChecked():
            date_str = self.dateEdit.date().toString("yyyy-MM-dd")
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

        query = (
            f"SELECT * FROM {table_name} WHERE {' AND '.join(conditions)}"
            if conditions
            else f"SELECT * FROM {table_name}"
        )

        try:
            self.cursor.execute(query)
            rows = self.cursor.fetchall()
            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(
                ["Date", "X", "Y", "Error", "Linear Vel", "Angular Vel"]
            )

            for row in rows:
                items = [QStandardItem(str(cell)) for cell in row]
                model.appendRow(items)

            self.dataTableView.setModel(model)
            self.dataTableView.resizeColumnsToContents()
            self.dataTableView.resizeRowsToContents()
        except Exception as e:
            self.dataLabel.setText(f"❌ Error filtering data: {e}")
            self.dataLabel.adjustSize()

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
                Paragraph("Reporter: Tran Trung Liem ID: 2131100018", styles["Normal"]),
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

    def closeEvent(self, event):
        """Close database connection when the window is closed."""
        if hasattr(self, "connection") and self.connection.is_connected():
            self.cursor.close()
            self.connection.close()
        event.accept()
