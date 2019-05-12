package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.CANifier
import edu.wpi.first.wpilibj.AnalogInput
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.CircularBuffer
import org.ghrobotics.lib.utils.DoubleSource

object Understructure {

    private val canifier = CANifier(Constants.kCanifierId)
    private val frontOnPlatformSensor = AnalogInput(Constants.kClimberSensorId)

    private val rollingLidarAverage = CircularBuffer(20)
    private val tempPWMData = DoubleArray(2)

    private var lidarAverage = 0.0
    private var frontOnPlatform = false

    private var aboveLIDARThresholdHeight = false

    fun update() {
        canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, tempPWMData)
        rollingLidarAverage.add(tempPWMData[0])
        lidarAverage = rollingLidarAverage.average

        frontOnPlatform = frontOnPlatformSensor.averageVoltage > 3.4
    }

    fun autoClimb(isLevel2: Boolean) = sequential {
        // Reset LIDAR Threshold
        +InstantRunnableCommand { aboveLIDARThresholdHeight = false }

        // Step 1: Extend stilts and begin moving forward toward the HAB platform.
        +parallel {
            // Climb to height
            if (isLevel2) {
                +ClosedLoopStiltsCommand(frontTarget = 8.inch, backTarget = 9.inch)
            } else {
                +ClosedLoopStiltsCommand(frontTarget = 23.5.inch, backTarget = 18.5.inch)
            }

            // Start moving forward
            +sequential {
                if (isLevel2) {
                    +ConditionCommand {
                        Stilts.frontHeight > 6.5.inch &&
                            Stilts.backHeight > 7.5.inch
                    }
                } else {
                    +ConditionCommand {
                        Stilts.frontHeight > 19.inch &&
                            Stilts.backHeight > 15.inch
                    }
                }
                +InstantRunnableCommand { aboveLIDARThresholdHeight = true }
                +OpenLoopHABDriveCommand { 0.75 }
            }
        }.withExit {
            // Exit only when LIDAR is detected, robot is above platform, and the stilts are above a certain
            // threshold height.
            lidarAverage > 25 && lidarAverage < 600 && aboveLIDARThresholdHeight
        }

        // Step 2: Retract back stilts
        +parallel {
            +ResetStiltsCommand(resetFront = false)
            +OpenLoopHABDriveCommand { 0.3 }
            +DriveWithPercentCommand { -0.01 }
        }.withExit { Stilts.backLimitSwitchEngaged }

        // Step 3: Drive until the front is safe to retract
        +parallel {
            +DriveWithPercentCommand { -0.4 }
            +OpenLoopHABDriveCommand { 1.0 }
        }.apply {
            if (isLevel2) {
                withTimeout(1.75.second)
            } else {
                withExit { frontOnPlatform }
            }
        }

        // Step 4: Retract front stilts
        +parallel {
            +ResetStiltsCommand(resetFront = true)
            +DriveWithPercentCommand { -0.05 }
        }.withExit { Stilts.frontLimitSwitchEngaged }

        // Step 5: Drive
        +DriveWithPercentCommand { -0.5 }
    }

    class DriveWithPercentCommand(val percentSource: DoubleSource) : FalconCommand(Drivetrain) {
        override suspend fun execute() {
            Drivetrain.tankDrive(percentSource(), percentSource())
        }

        override suspend fun dispose() {
            Drivetrain.zeroOutputs()
        }
    }

}