package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconSubsystem

object HABDriver : FalconSubsystem() {
    private val masterMotor = TalonSRX(Constants.kClimbWheelId)

    private val periodicIO = PeriodicIO()

    init {
        defaultCommand = TeleopHABDriverCommand()
    }

    override fun periodic() {
        masterMotor.set(ControlMode.PercentOutput, periodicIO.demand)
    }

    fun setOpenLoop(percent: Double) {
        periodicIO.demand = percent
    }

    override fun zeroOutputs() {
        periodicIO.demand = 0.0
    }

    private class PeriodicIO {
        var demand: Double = 0.0
    }
}