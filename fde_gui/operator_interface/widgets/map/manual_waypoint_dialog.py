from PySide6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QLabel,
    QVBoxLayout,
)


class ManualWaypointDialog(QDialog):
    def __init__(
        self,
        robot,
        parent=None,
    ):
        super().__init__(parent)

        self.robot = robot.lower()

        self.setWindowTitle(
            f"Add {self.robot.title()} Waypoint"
        )

        self.setMinimumWidth(300)

        layout = QVBoxLayout(self)

        title = QLabel(
            f"Enter {self.robot.title()} waypoint coordinates"
        )

        layout.addWidget(title)

        form = QFormLayout()

        self.x_input = self._coordinate_input()
        self.y_input = self._coordinate_input()

        form.addRow(
            "X (m):",
            self.x_input,
        )

        form.addRow(
            "Y (m):",
            self.y_input,
        )

        # Drone gets altitude.
        if self.robot == "drone":

            self.z_input = self._coordinate_input()

            self.z_input.setMinimum(0.0)
            self.z_input.setValue(10.0)

            form.addRow(
                "Z / Altitude (m):",
                self.z_input,
            )

        else:

            self.z_input = None

        layout.addLayout(form)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Cancel
            | QDialogButtonBox.Ok
        )

        buttons.button(
            QDialogButtonBox.Ok
        ).setText("Add Waypoint")

        buttons.accepted.connect(
            self.accept
        )

        buttons.rejected.connect(
            self.reject
        )

        layout.addWidget(buttons)

    def _coordinate_input(self):

        spin = QDoubleSpinBox()

        spin.setRange(
            -10000.0,
            10000.0,
        )

        spin.setDecimals(2)

        spin.setSingleStep(1.0)

        spin.setSuffix(" m")

        return spin

    def coordinates(self):

        x = self.x_input.value()
        y = self.y_input.value()

        if self.z_input is not None:
            z = self.z_input.value()
        else:
            z = 0.0

        return x, y, z