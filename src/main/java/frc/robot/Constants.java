// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
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

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static String canBusName = "canivore";

  public class algaeIntakeConstants {
    public static int canID = 23;
    public static double statCurLim = 40;
    public static boolean enableStatCurLim = true;
    public static double supCurLim = 35;
    public static boolean enableSupCurLim = true;
    public static double supLowLim = 20;
    public static double supLowTime = 1;
    public static NeutralModeValue neutralMode = NeutralModeValue.Coast;
    public static InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static double kS = .25;
    public static double kV = .12;
    public static double kA = .01;
    public static double kP = 4;
    public static double kI = 0.1;
    public static double kD = 0.2;
    public static double mmCruiseVel = 80;
    public static double mmAccel = 160;
    public static double mmJerk = 1600;

    public static double intakeVelo = 30;
    public static double shootVelo = 100;
    public static double dropVelo = -25;
    public static double stopVelo = 5;
  }

  public class algaeIntakeWristConstants {}

  public class flywheelConstants {
    public static int canID = 24;
    public static double statCurLim = 40;
    public static boolean enableStatCurLim = true;
    public static double supCurLim = 35;
    public static boolean enableSupCurLim = true;
    public static double supLowLim = 30;
    public static double supLowTime = 1;
    public static NeutralModeValue neutralMode = NeutralModeValue.Coast;
    public static InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static double kS = .25;
    public static double kV = .12;
    public static double kA = .01;
    public static double kP = 4;
    public static double kI = 0.005;
    public static double kD = 0.2;
    public static double mmCruiseVel = 80;
    public static double mmAccel = 160;
    public static double mmJerk = 1600;

    public static double intakeVelo = -15;
    public static double shootVelo = 100;
    public static double dropVelo = -8;
    public static double stopVelo = 0;
  }

  public class indexerConstants {
    public static int canID = 25;
    public static double statCurLim = 25;
    public static boolean enableStatCurLim = true;
    public static double supCurLim = 15;
    public static boolean enableSupCurLim = true;
    public static double supLowLim = 8;
    public static double supLowTime = 1;
    public static NeutralModeValue neutralMode = NeutralModeValue.Coast;
    public static InvertedValue inverted = InvertedValue.CounterClockwise_Positive;
    public static double kS = .25;
    public static double kV = .12;
    public static double kA = .01;
    public static double kP = 4.8;
    public static double kI = 0;
    public static double kD = 0.1;
    public static double mmCruiseVel = 80;
    public static double mmAccel = 160;
    public static double mmJerk = 1600;

    public static double intakeVelo = .4;
    public static double shootVelo = -1;
    public static double dropVelo = -.4;
    public static double stopVelo = 0;

    public static double shootTime = .25;
  }

  public class coralIntakeConstants {
    public static int canID = 35;
    public static int LCCanID = 3;
    public static double statCurLim = 25;
    public static boolean enableStatCurLim = true;
    public static double supCurLim = 22;
    public static boolean enableSupCurLim = true;
    public static double supLowLim = 15;
    public static double supLowTime = 1;
    public static NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static InvertedValue inverted = InvertedValue.CounterClockwise_Positive;

    public static double intakeVelo = .25;
    public static double scoreVelo = .45;
    public static double dropVelo = -.25;
    public static double stopVelo = 0;
  }

  public class coralIntakeWristConstants {
    public static int canID = 33;
    public static double statCurLim = 20;
    public static boolean enableStatCurLim = true;
    public static double supCurLim = 15;
    public static boolean enableSupCurLim = true;
    public static double supLowLim = 8;
    public static double supLowTime = 1;
    public static NeutralModeValue neutralMode = NeutralModeValue.Coast;
    public static InvertedValue inverted = InvertedValue.CounterClockwise_Positive;
    public static double kG = 0;
    public static double kS = .03;
    public static double kV = .01;
    public static double kA = .003;
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.02;
    public static double mmCruiseVel = 80;
    public static double mmAccel = 160;
    public static double mmJerk = 1600;

    public static double storePos = .5;
    public static double sourcePos = 4;
    public static double L1Pos = 4.5;
    public static double L23Pos = 18;
    public static double L4Pos = 16.75;
    public static double dPos = 15.5;
  }

  public class elevatorConstants {
    public static int canID = 31;
    public static double statCurLim = 40;
    public static boolean enableStatCurLim = true;
    public static double supCurLim = 35;
    public static boolean enableSupCurLim = true;
    public static double supLowLim = 25;
    public static double supLowTime = 1;
    public static NeutralModeValue neutralMode = NeutralModeValue.Coast;
    public static InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static double kG = 15;
    public static double kS = 2;
    public static double kV = .6;
    public static double kA = .4;
    public static double kP = 35;
    public static double kI = 0.03;
    public static double kD = 4;
    public static double mmCruiseVel = 80;
    public static double mmAccel = 200;
    public static double mmJerk = 1600;

    public static double storePos = .5;
    public static double sourcePos = 37.5;
    public static double L1Pos = 6;
    public static double L2Pos = 4;
    public static double L3Pos = 40;
    public static double L4Pos = 88;
    public static double ldPos = 1;
    public static double hdPos = 25;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
