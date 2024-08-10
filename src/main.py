import dataclasses
import enum
import sys
from functools import partial

from mavsdk import System, info
import asyncio
from loguru import logger
from PySide6 import QtWidgets, QtCore, QtGui
from PySide6.QtGui import QKeyEvent

DEFAULT_PORT = 14550

MAVLINK_CONNECTIONS = {
    "UDP": f"udp://:{DEFAULT_PORT}",
    "TCP": f"tcp://:{DEFAULT_PORT}",
    "SERIAL": "serial:///dev/tty.usbmodem1101"
}


class SignalsType(enum.Enum):
    TELEMETRY_PITCH = "TELEMETRY_PITCH"
    TELEMETRY_YAW = "TELEMETRY_YAW"
    TELEMETRY_ROLL = "TELEMETRY_ROLL"
    WORKER_STATE_STR = "WORKER_STATE_STR"


class ExceptionTypes(enum.Enum):
    CONNECTION_ERROR = "CONNECTION_ERROR"


@dataclasses.dataclass
class SignalValue:
    signal_type: SignalsType
    signal_value: float | str


@dataclasses.dataclass
class SignalException:
    signal_type: ExceptionTypes


class SharedSignal(QtCore.QObject):
    values = QtCore.Signal(object)


class SharedException(QtCore.QObject):
    value = QtCore.Signal(SignalException)


class ToWorkerSignals(QtCore.QObject):
    value = QtCore.Signal(object)


class MavlinkWorker(QtCore.QRunnable):

    def __init__(self, connection_address):
        super().__init__()
        self.connection_address = connection_address
        self._drone = System()
        self.signal = SharedSignal()
        self.exception = SharedException()
        self.to_worker_signal = ToWorkerSignals()
        self.to_worker_signal.value.connect(self._worker_process)
        self._terminate = False
        self._armed = None
        self._drone_is_armed = False
        self._z = 0

    async def _arm(self):
        while True:
            if self._armed is True:
                await self._drone.action.arm()
                self._armed = None
                self._drone_is_armed = True
                self.signal.values.emit([
                    SignalValue(
                        signal_type=SignalsType.WORKER_STATE_STR,
                        signal_value="Аппарат в состоянии ARM ⚠️")])
            elif self._armed is False:
                await self._drone.action.disarm()
                self._drone_is_armed = False
                self._armed = None
                self.signal.values.emit([
                    SignalValue(
                        signal_type=SignalsType.WORKER_STATE_STR,
                        signal_value="Аппарат в состоянии DISARM 🛟")])
            await asyncio.sleep(0.1)

    async def _movements(self):
        while True:
            if self._drone_is_armed:
                logger.debug(f'Update z value to {self._z}')
                try:
                    await self._drone.manual_control.set_manual_control_input(0, 0, self._z, 0)
                except Exception as e:
                    logger.error(e)
            await asyncio.sleep(0.1)

    def _worker_process(self, s):
        logger.info(f"Get signal for worker {s}")
        if s == "arm":
            logger.info(f"Set ROV to arm state")
            self._armed = True
        elif s == "disarm":
            logger.info(f"Set ROV to disarm state")
            self._armed = False
        elif s == "up_pressed":
            self._z = 0.5
        elif s == "down_pressed":
            self._z = -0.5
        elif s == "up_released":
            self._z = 0
        elif s == "down_released":
            self._z = 0

    async def _initialize_connection(self):
        self.signal.values.emit([
            SignalValue(
                signal_type=SignalsType.WORKER_STATE_STR,
                signal_value="🔄 Подключение к ROV")])

        logger.info("Connecting to drone by address: {}", self.connection_address)
        await self._drone.connect(system_address=self.connection_address)

        logger.info("🔄 Waiting for drone to connect...")
        self.signal.values.emit([
            SignalValue(
                signal_type=SignalsType.WORKER_STATE_STR,
                signal_value="🔄 Ожидание подключения")])

        async for state in self._drone.core.connection_state():
            if state.is_connected:
                logger.info("✅ Connected to drone success")
                self.signal.values.emit([
                    SignalValue(
                        signal_type=SignalsType.WORKER_STATE_STR,
                        signal_value="✅ Подключено")])
                break

        drone_info: info.Version = await self._drone.info.get_version()
        drone_software_version = f"{drone_info.flight_sw_major}.{drone_info.flight_sw_minor}.{drone_info.flight_sw_patch}"
        drone_os_version = f"{drone_info.os_sw_major}.{drone_info.os_sw_minor}.{drone_info.os_sw_patch}"

        logger.info("Drone software version: {}, os version: {}", drone_software_version, drone_os_version)

        logger.info("🔄 Get drone all parameters")
        self.signal.values.emit([
            SignalValue(
                signal_type=SignalsType.WORKER_STATE_STR,
                signal_value="🔄 Получение параметры системы")])
        # all_params = await self._drone.param.get_all_params()
        logger.info("✅ All parameters have been received")
        self.signal.values.emit([
            SignalValue(
                signal_type=SignalsType.WORKER_STATE_STR,
                signal_value="✅ Подключено, все параметры получены")])

    async def _telemetry_attitude_euler_publisher(self):
        try:
            async for altitude in self._drone.telemetry.attitude_euler():
                self.signal.values.emit([
                    SignalValue(
                        signal_type=SignalsType.TELEMETRY_PITCH,
                        signal_value=altitude.pitch_deg),
                    SignalValue(
                        signal_type=SignalsType.TELEMETRY_YAW,
                        signal_value=altitude.yaw_deg),
                    SignalValue(
                        signal_type=SignalsType.TELEMETRY_ROLL,
                        signal_value=altitude.roll_deg)
                ])

                if self._terminate:
                    logger.info("Terminate telemetry altitude publisher")
                    self.signal.values.emit([
                        SignalValue(
                            signal_type=SignalsType.TELEMETRY_PITCH,
                            signal_value=0),
                        SignalValue(
                            signal_type=SignalsType.TELEMETRY_YAW,
                            signal_value=0),
                        SignalValue(
                            signal_type=SignalsType.TELEMETRY_ROLL,
                            signal_value=0)
                    ])
                    return
        except Exception as e:
            logger.error(e)
            self.exception.value.emit(SignalException(signal_type=ExceptionTypes.CONNECTION_ERROR))

    async def _run(self):
        self._loop = asyncio.get_running_loop()
        await self._initialize_connection()
        task = asyncio.create_task(self._telemetry_attitude_euler_publisher())
        task2 = asyncio.create_task(self._arm())
        task3 = asyncio.create_task(self._movements())
        await asyncio.wait([task, task2, task3])

    @QtCore.Slot()
    def run(self):
        asyncio.run(self._run())


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        self._mavlink_state = {
            "connected": False,
        }

        self._drone_state = {
            "armed": False
        }

        self.window_title = "MSUR Ground Station v.0.1.0"
        self.setWindowTitle(self.window_title)

        connection_status_label = QtWidgets.QLabel("Состояние подключения: ")
        self.connection_status = QtWidgets.QLabel("Не подключено 🚫")

        connection_selector_label = QtWidgets.QLabel("Интерфейс подключения")
        self.connection_selector = QtWidgets.QComboBox()
        self.connection_button = QtWidgets.QPushButton("✅ Подключить")
        self.connection_button.pressed.connect(self.connect_)

        self.connection_selector.addItems(list(MAVLINK_CONNECTIONS.keys()))

        connection_status_widget = QtWidgets.QWidget()
        connection_status_layout = QtWidgets.QHBoxLayout()
        connection_status_layout.addWidget(connection_status_label)
        connection_status_layout.addWidget(self.connection_status)
        connection_status_widget.setLayout(connection_status_layout)

        connection_widget = QtWidgets.QWidget()
        connection_layout = QtWidgets.QHBoxLayout()
        connection_layout.addWidget(connection_selector_label)
        connection_layout.addWidget(self.connection_selector)
        connection_layout.addWidget(self.connection_button)
        connection_widget.setLayout(connection_layout)

        telemetry_widget = QtWidgets.QWidget()
        telemetry_layout = QtWidgets.QHBoxLayout()
        pitch_label = QtWidgets.QLabel("pitch:")
        self.pitch_value = QtWidgets.QLabel("-")
        yaw_label = QtWidgets.QLabel("yaw:")
        self.yaw_value = QtWidgets.QLabel("-")
        roll_label = QtWidgets.QLabel("roll:")
        self.roll_value = QtWidgets.QLabel("-")
        telemetry_layout.addWidget(pitch_label)
        telemetry_layout.addWidget(self.pitch_value)
        telemetry_layout.addWidget(yaw_label)
        telemetry_layout.addWidget(self.yaw_value)
        telemetry_layout.addWidget(roll_label)
        telemetry_layout.addWidget(self.roll_value)
        telemetry_widget.setLayout(telemetry_layout)

        self.arm_disarm_button = QtWidgets.QPushButton("Arm")
        self.arm_disarm_button.pressed.connect(self._switch_arm)
        self.arm_disarm_button.setDisabled(True)

        self.up_btn = QtWidgets.QPushButton("⬆️Всплыть")
        self.up_btn.setDisabled(True)
        self.up_btn.pressed.connect(partial(self._control_movements, 'up_pressed'))
        self.up_btn.released.connect(partial(self._control_movements, 'up_released'))
        self.down_btn = QtWidgets.QPushButton("⬇️Погрузиться")
        self.down_btn.pressed.connect(partial(self._control_movements, 'down_pressed'))
        self.down_btn.released.connect(partial(self._control_movements, 'down_released'))
        self.down_btn.setDisabled(True)

        control_widget = QtWidgets.QWidget()
        control_layout = QtWidgets.QHBoxLayout()
        control_layout.addWidget(self.down_btn)
        control_layout.addWidget(self.up_btn)
        control_widget.setLayout(control_layout)

        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(connection_status_widget)
        main_layout.addWidget(connection_widget)
        main_layout.addWidget(telemetry_widget)
        main_layout.addWidget(self.arm_disarm_button)
        main_layout.addWidget(control_widget)

        widget = QtWidgets.QWidget()
        widget.setLayout(main_layout)

        self.setCentralWidget(widget)
        self.mavlink_worker = None
        self.threadpool = QtCore.QThreadPool()

    def connect_(self):
        if not self._mavlink_state["connected"]:
            self._mavlink_state["connected"] = True
            self.connection_button.setText("🚫 Отключить")

            connection_address_name = self.connection_selector.currentText()
            connection_address = MAVLINK_CONNECTIONS[connection_address_name]
            self.mavlink_worker = MavlinkWorker(None)
            self.mavlink_worker.connection_address = connection_address

            self.mavlink_worker.signal.values.connect(self._update_shared_value)
            self.mavlink_worker.exception.value.connect(self._shared_exception)
            self.mavlink_worker._terminate = False
            self.threadpool.start(self.mavlink_worker)
            self.arm_disarm_button.setDisabled(False)
        else:
            self._mavlink_state["connected"] = False
            self.connection_button.setText("✅ Подключить")
            self.mavlink_worker._terminate = True
            self.connection_status.setText("Не подключено 🚫")
            self.arm_disarm_button.setDisabled(True)
            self.up_btn.setDisabled(True)
            self.down_btn.setDisabled(True)

    def _control_movements(self, s: str):
        if s == "up_pressed":
            self.mavlink_worker.to_worker_signal.value.emit('up_pressed')
        elif s == "down_pressed":
            self.mavlink_worker.to_worker_signal.value.emit('down_pressed')
        elif s == "up_released":
            self.mavlink_worker.to_worker_signal.value.emit('up_released')
        elif s == "down_released":
            self.mavlink_worker.to_worker_signal.value.emit('down_released')

    def _switch_arm(self):
        if self._drone_state["armed"]:
            self.arm_disarm_button.setText('Arm')
            self._drone_state["armed"] = False
            self.mavlink_worker.to_worker_signal.value.emit('disarm')
            self.up_btn.setDisabled(True)
            self.down_btn.setDisabled(True)
        else:
            self.arm_disarm_button.setText('Disarm')
            self._drone_state["armed"] = True
            self.mavlink_worker.to_worker_signal.value.emit('arm')
            self.up_btn.setDisabled(False)
            self.down_btn.setDisabled(False)

    def _update_shared_value(self, shared_value: list[SignalValue]):
        for value in shared_value:
            if value.signal_type == SignalsType.WORKER_STATE_STR:
                self.connection_status.setText(value.signal_value)

            if value.signal_type == SignalsType.TELEMETRY_PITCH:
                self.pitch_value.setText(str(round(value.signal_value, 2)))

            if value.signal_type == SignalsType.TELEMETRY_YAW:
                self.yaw_value.setText(str(round(value.signal_value, 2)))

            if value.signal_type == SignalsType.TELEMETRY_ROLL:
                self.roll_value.setText(str(round(value.signal_value, 2)))

    def _shared_exception(self, shared_exception: SignalException):
        if shared_exception.signal_type == ExceptionTypes.CONNECTION_ERROR:
            self._mavlink_state["connected"] = False
            self.connection_status.setText("Подключение разорвано 🤷")
            self.connection_button.setText("✅ Подключить")
            self.arm_disarm_button.setDisabled(True)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    window = MainWindow()
    window.resize(800, 600)
    window.show()

    sys.exit(app.exec_())
