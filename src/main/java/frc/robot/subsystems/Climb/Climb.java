package frc.robot.subsystems.Climb;

public class Climb {
  private ClimbIO io;
  private boolean rachetToggle;
 
  public Climb(ClimbIO io){
    this.io = io;
    io.configPID(1, 0, 0);
    rachetToggle = false;
  }

  public void twistToPosition(double position){
    io.twistMotorToPosition(position);
  }

  public void manualControl(double controlVolts){
    io.twistMotorVolts(controlVolts*3);
  }

  public void rachetToggle (boolean toggleSwitch) {
    if(rachetToggle){
      io.turnClimbServo(0);
    }else{
      io.turnClimbServo(1);
    }
  }

}
