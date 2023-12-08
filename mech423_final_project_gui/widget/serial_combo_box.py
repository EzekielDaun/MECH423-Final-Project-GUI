from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtWidgets import QComboBox


class SerialComboBox(QComboBox):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        for port in QSerialPortInfo.availablePorts():
            self.addItem(port.portName())

    def showPopup(self) -> None:
        super().showPopup()
        self.clear()
        for port in QSerialPortInfo.availablePorts():
            self.addItem(port.portName())
