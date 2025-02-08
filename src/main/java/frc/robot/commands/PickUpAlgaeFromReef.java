package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algae_mech.AlgaeMech;
import frc.robot.subsystems.elevator.Elevator;

public class PickUpAlgaeFromReef extends SequentialCommandGroup {
  public PickUpAlgaeFromReef(Elevator m_elevator, AlgaeMech m_algaeMech) {}
}
