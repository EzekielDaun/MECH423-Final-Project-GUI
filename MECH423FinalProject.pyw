import sys
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import pyqtgraph as pg
from loguru import logger
from PySide6.QtWidgets import QApplication

from mech423_final_project_gui.window.lab3_main_window import Lab3MainWindow

if __name__ == "__main__":
    app = QApplication([])

    pg.setConfigOption("background", "w")
    pg.setConfigOption("foreground", "k")

    main_window = Lab3MainWindow()
    main_window.show()
    with redirect_stdout(main_window):  # type: ignore
        with redirect_stderr(main_window):  # type: ignore
            logger.remove()
            logger.add(sys.stdout)
            logger.add(
                Path(__file__).parent.parent / ".log" / "{time}.log",
                rotation="12:00",
                retention="2 days",
            )
            logger.enable("MECH423FinalProject.window")
            app.exec()
