import re
from itertools import batched

import numpy as np
from loguru import logger
from pyqtgraph import ArrowItem, CircleROI, PlotDataItem, PlotWidget
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QTextCursor
from PySide6.QtSerialPort import QSerialPort
from PySide6.QtWidgets import (
    QDockWidget,
    QHBoxLayout,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)

from ..widget.serial_combo_box import SerialComboBox


class Lab3MainWindow(QMainWindow):
    signal_write = Signal(str)

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.__central_widget = Lab3MainWindowCentralWidget()
        self.setCentralWidget(self.__central_widget)

        self.__text_browser = QTextBrowser()
        self.signal_write.connect(self.__slot_write)

        dock_widget = QDockWidget()
        dock_widget.setWidget(self.__text_browser)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock_widget)

        self.setWindowTitle("MECH423FinalProjectGUI")

    def write(self, text: str) -> None:
        self.signal_write.emit(text)

    def flush(self) -> None:
        pass

    def __slot_write(self, text: str) -> None:
        self.__text_browser.moveCursor(QTextCursor.MoveOperation.End)
        self.__text_browser.insertPlainText(text)
        self.__text_browser.ensureCursorVisible()


class Lab3MainWindowCentralWidget(QWidget):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        # serial port
        self.__serial_port = QSerialPort()
        self.__serial_rx_buffer = bytearray()
        self.__serial_port.readyRead.connect(self.__slot_on_serial_ready)

        serial_port_layout = QHBoxLayout()
        self.__serial_port_combobox = SerialComboBox()

        self.__serial_connect_button = QPushButton("Connect")
        self.__serial_connect_button.setCheckable(True)
        self.__serial_connect_button.clicked.connect(self.__slot_on_serial_connect)

        serial_port_layout.addWidget(self.__serial_port_combobox)
        serial_port_layout.addWidget(self.__serial_connect_button)

        self.__tick_spin_box = QSpinBox()
        self.__tick_spin_box.setPrefix("Detent: ")
        self.__tick_spin_box.setMinimum(0)
        self.__tick_spin_box.setMaximum(35)
        self.__tick_spin_box.valueChanged.connect(self.__slot_on_tick_change)

        self.__velocity_mode_button = QPushButton("Velocity Mode")
        self.__velocity_mode_button.clicked.connect(self.__slot_on_velocity_mode)
        self.__torque_mode_button = QPushButton("Torque Mode")
        self.__torque_mode_button.clicked.connect(self.__slot_on_torque_mode)

        self.__mech_angle_plot_widget = PlotWidget()
        self.__mech_angle_plot_widget.setXRange(-1, 1)
        self.__mech_angle_plot_widget.setYRange(-1, 1)
        self.__mech_angle_plot_widget.setAspectLocked(True)
        self.__mech_angle_plot_widget.setTitle("Mechanical Angle")
        self.__mech_angle_plot_widget.addLegend()
        self.__mech_angle_plot_widget.addItem(CircleROI([-1, -1], [2, 2], pen="k"))
        self.__mech_angle_plot_widget.hideAxis("bottom")
        self.__mech_angle_plot_widget.hideAxis("left")

        self.__mech_plot_mech_arrow = ArrowItem(pen="k", brush="k")
        self.__mech_plot_mech_arrow.setStyle(angle=0)
        self.__mech_angle_plot_widget.addItem(self.__mech_plot_mech_arrow)
        self.__mech_plot_mech_line = PlotDataItem(
            name="Rotor Mechanical Angle", pen="k"
        )
        self.__mech_angle_plot_widget.addItem(self.__mech_plot_mech_line)

        self.__elec_angle_plot_widget = PlotWidget()
        self.__elec_angle_plot_widget.setXRange(-1, 1)
        self.__elec_angle_plot_widget.setYRange(-1, 1)
        self.__elec_angle_plot_widget.setAspectLocked(True)
        self.__elec_angle_plot_widget.setTitle("Electrical Angle")
        self.__elec_angle_plot_widget.addLegend()
        self.__elec_angle_plot_widget.addItem(CircleROI([-1, -1], [2, 2], pen="k"))
        self.__elec_angle_plot_widget.hideAxis("bottom")
        self.__elec_angle_plot_widget.hideAxis("left")

        self.__elec_plot_mech_arrow = ArrowItem(pen="k", brush="k")
        self.__elec_plot_mech_line = PlotDataItem(
            name="Rotor Electrical Angle", pen="k"
        )
        self.__elec_angle_plot_widget.addItem(self.__elec_plot_mech_arrow)
        self.__elec_angle_plot_widget.addItem(self.__elec_plot_mech_line)
        self.__elec_plot_current_arrow = ArrowItem(pen="r", brush="r")
        self.__elec_plot_current_line = PlotDataItem(
            name="Field Electrical Vector", pen="r"
        )
        self.__elec_angle_plot_widget.addItem(self.__elec_plot_current_arrow)
        self.__elec_angle_plot_widget.addItem(self.__elec_plot_current_line)

        control_layout = QHBoxLayout()
        control_layout.addWidget(self.__velocity_mode_button)
        control_layout.addWidget(self.__torque_mode_button)
        control_layout.addWidget(self.__tick_spin_box)

        plot_layout = QHBoxLayout()
        plot_layout.addWidget(self.__mech_angle_plot_widget)
        plot_layout.addWidget(self.__elec_angle_plot_widget)

        self.setLayout(QVBoxLayout())
        self.layout().addLayout(serial_port_layout)  # type: ignore
        self.layout().addLayout(control_layout)  # type: ignore
        self.layout().addLayout(plot_layout)  # type: ignore

    def __slot_on_serial_connect(self):
        if self.__serial_connect_button.isChecked():
            self.__serial_port.setPortName(self.__serial_port_combobox.currentText())
            self.__serial_port.setBaudRate(QSerialPort.BaudRate.Baud115200)
            self.__serial_port.setDataBits(QSerialPort.DataBits.Data8)
            self.__serial_port.setParity(QSerialPort.Parity.NoParity)
            self.__serial_port.setStopBits(QSerialPort.StopBits.OneStop)
            self.__serial_port.setFlowControl(QSerialPort.FlowControl.NoFlowControl)

            if not self.__serial_port.open(QSerialPort.OpenModeFlag.ReadWrite):
                QMessageBox.critical(self, "Error", "Cannot open serial port")
                self.__serial_connect_button.setChecked(False)
                return
            self.__serial_port_combobox.setEnabled(False)
            self.__serial_connect_button.setText("Disconnect")
        else:
            self.__serial_port.close()
            self.__serial_port_combobox.setEnabled(True)
            self.__serial_connect_button.setText("Connect")

    def __slot_on_serial_ready(self):
        data = self.__serial_port.readAll().data()
        # logger.debug(f"{data.hex().upper()}")
        logger.debug(data.decode("ascii", errors="ignore"))

        self.__serial_rx_buffer += data
        while b"\n" in self.__serial_rx_buffer:
            line, self.__serial_rx_buffer = self.__serial_rx_buffer.split(b"\n", 1)
            message = line.decode("ascii", errors="ignore")

            match = re.match(
                r"(?P<pos_angle>[-+]?\d+.\d+), (?P<I_u>[-+]?\d+.\d+), (?P<I_v>[-+]?\d+.\d+), (?P<I_w>[-+]?\d+.\d+)",
                message,
            )
            if match:
                pos_angle = -float(match.group("pos_angle"))
                mech_plot_x = np.cos(np.deg2rad(pos_angle))
                mech_plot_y = np.sin(np.deg2rad(pos_angle))
                self.__mech_plot_mech_arrow.setPos(mech_plot_x, mech_plot_y)
                self.__mech_plot_mech_arrow.setStyle(angle=180 - pos_angle, tailLen=1)
                self.__mech_plot_mech_line.setData(
                    x=[0, mech_plot_x], y=[0, mech_plot_y]
                )

                elec_plot_mech_x = np.cos(np.deg2rad(7 * pos_angle))
                elec_plot_mech_y = np.sin(np.deg2rad(7 * pos_angle))

                self.__elec_plot_mech_arrow.setPos(elec_plot_mech_x, elec_plot_mech_y)
                self.__elec_plot_mech_arrow.setStyle(angle=180 - 7 * pos_angle)
                self.__elec_plot_mech_line.setData(
                    x=[0, elec_plot_mech_x], y=[0, elec_plot_mech_y]
                )

                I_u = -float(match.group("I_u")) / 400
                I_v = -float(match.group("I_v")) / 400
                I_w = -float(match.group("I_w")) / 400

                I_x = I_u - I_v / 2 - I_w / 2
                I_y = np.sqrt(3) / 2 * (I_v - I_w)
                self.__elec_plot_current_arrow.setPos(I_x, I_y)
                self.__elec_plot_current_arrow.setStyle(
                    angle=180 + np.rad2deg(-np.arctan2(I_y, I_x))
                )
                self.__elec_plot_current_line.setData(x=[0, I_x], y=[0, I_y])

    def __slot_on_serial_write(self, message: bytearray):
        self.__serial_port.write(message)
        hex_string = ", ".join(
            [x + y for (x, y) in (batched(message.hex().upper(), 2))]
        )
        logger.debug(f"serial_bytes: {hex_string}")

    def __slot_on_tick_change(self, value):
        base_36_c = np.base_repr(value, 36)
        self.__serial_port.write(f"{base_36_c}".encode("ascii"))

    def __slot_on_velocity_mode(self):
        self.__serial_port.write(b"v")

    def __slot_on_torque_mode(self):
        self.__serial_port.write(b"t")
