package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algae_mech.AlgaeMech;
import frc.robot.subsystems.elevator.Elevator;

public class PickUpAlgaeFromReef extends SequentialCommandGroup {
  Elevator m_elevator;
  AlgaeMech m_algaeMech;
  public PickUpAlgaeFromReef(Elevator m_elevator, AlgaeMech m_algaeMech) {
    this.m_elevator = m_elevator;
    this.m_algaeMech = m_algaeMech;
  }
}
