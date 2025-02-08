package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoConstants {
  public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(1, 1, 360, 360);

  static {
    registerCommands();
  }

  private static void registerCommands() {
    NamedCommands.registerCommand("Coral", Commands.print("Coral"));
  }
}
