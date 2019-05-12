package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.SerialPort
import org.ghrobotics.frc2019.subsystems.intake.Intake
import org.ghrobotics.frc2019.vision.JeVoisManager
import org.ghrobotics.lib.wrappers.FalconRobot
import kotlin.concurrent.thread

object LEDs {

    private var wantedLEDMode = Mode.None

    init {
        thread {
            while (true) {
                try {
                    val port = SerialPort(9600, SerialPort.Port.kMXP)
                    port.setTimeout(0.5190)
                    var currentLEDMode: Mode? = null
                    @Suppress("ConvertTryFinallyToUseCall")
                    try {
                        while (true) {
                            val newLEDMode = wantedLEDMode
                            if (currentLEDMode != newLEDMode) {
                                port.writeString("${newLEDMode.value}\n")
                                currentLEDMode = newLEDMode
                            }
                            Thread.sleep(1000 / 50)
                        }
                    } finally {
                        port.close()
                    }
                } catch (@Suppress("TooGenericExceptionCaught") e: Throwable) {
                    e.printStackTrace()
                    Thread.sleep(5000)
                }
            }
        }
    }

    fun update() {
        wantedLEDMode = when {
            Robot.emergencyActive || (Robot.lastRobotMode == FalconRobot.Mode.DISABLED &&
                (!JeVoisManager.isFrontJeVoisConnected || !JeVoisManager.isBackJeVoisConnected)) -> Mode.Emergency

            Controls.backModifier -> Mode.BackModifier
            Controls.isClimbing -> Mode.Climb
//            VisionDriveCommand.isActive || TrajectoryVisionTrackerCommand.visionActive -> Mode.VISION
            Intake.isHoldingCargo || Intake.isHoldingHatch -> Mode.HasObtained
            Robot.lastRobotMode == FalconRobot.Mode.DISABLED -> Mode.Disabled
            else -> Mode.None
        }
    }

    enum class Mode(val value: Int) {
        BackModifier(6),
        HasObtained(5),
        Climb(4),
        Vision(3),
        Disabled(1),
        Emergency(2),
        None(0)
    }
}