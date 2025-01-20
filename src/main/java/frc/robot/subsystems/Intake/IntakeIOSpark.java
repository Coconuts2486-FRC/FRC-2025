package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.CANandPowerPorts;

public class IntakeIOSpark implements IntakeIO{
  private final SparkMax intakeRoller =
      new SparkMax(22, MotorType.kBrushless);
  private final SparkMax intakePivot = new SparkMax(21, MotorType.kBrushless);
        private final SparkClosedLoopController rollerPid = intakeRoller.getClosedLoopController();
      


        public IntakeIOSpark () {}

        @Override
        public void runIntakeRollers(double speedRPS) {
          rollerPid.setReference(1,ControlType.kMAXMotionVelocityControl);
        }
        @Override
        public double getRollerCurrent() {
          return intakeRoller.getOutputCurrent();
        }
}