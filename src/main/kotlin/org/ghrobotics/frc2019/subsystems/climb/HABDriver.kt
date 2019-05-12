package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconSubsystem

object HABDriver : FalconSubsystem() {
    private val masterMotor = TalonSRX(Constants.kClimbWheelId)

    init {
        defaultCommand = TeleopHABDriverCommand()
    }

    override fun periodic() {
        masterMotor.set(ControlMode.PercentOutput, PeriodicIO.demand)
    }

    fun setOpenLoop(percent: Double) {
        PeriodicIO.demand = percent
    }

    override fun zeroOutputs() {
        PeriodicIO.demand = 0.0
    }

    private object PeriodicIO {
        var demand: Double = 0.0
    }
}