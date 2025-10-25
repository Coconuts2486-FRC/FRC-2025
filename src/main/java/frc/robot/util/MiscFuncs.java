// Copyright (c) 2025 FRC 2486
// http://github.com/Coconuts2486-FRC
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

/** The home for wayward functions */
public class MiscFuncs {

  public static boolean isclose(double a, double b, double rtol, double atol) {
    return Math.abs(a - b) <= Math.max(atol, rtol * Math.abs(b));
  }

  /** The scoring position at any one REEF face */
  public static enum ScoringPosition {
    LEFT, // Left reef post (coral)
    RIGHT, // Right reef post (coral)
    CENTER, // Center position (algae grab)
    CENTERCLOSE, // Center position but closer to the reef to pick algae
    CENTERFAR,
    RIGHTAUTO,
    LEFTAUTO // It's being weird in auto so this is to fix that its just RIGHT and LEFT but closer
    // to the
    // tag
  }
}
