package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class flywheel extends SubsystemBase {
  TalonFX flywheelMotor = new TalonFX(Constants.flywheelConstants.canID, Constants.canBusName);
  final MotionMagicVelocityVoltage fControl = new MotionMagicVelocityVoltage(0);

  public flywheel() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = Constants.flywheelConstants.statCurLim;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable =
        Constants.flywheelConstants.enableStatCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = Constants.flywheelConstants.supCurLim;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable =
        Constants.flywheelConstants.enableSupCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerLimit = Constants.flywheelConstants.supLowLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = Constants.flywheelConstants.supLowTime;
    talonFXConfigs.MotorOutput.Inverted = Constants.flywheelConstants.inverted;
    talonFXConfigs.MotorOutput.NeutralMode = Constants.flywheelConstants.neutralMode;
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS =
        Constants.flywheelConstants.kS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV =
        Constants.flywheelConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA =
        Constants.flywheelConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP =
        Constants.flywheelConstants.kP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = Constants.flywheelConstants.kI; // no output for integrated error
    slot0Configs.kD =
        Constants.flywheelConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        Constants.flywheelConstants.mmCruiseVel; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        Constants.flywheelConstants.mmAccel; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk =
        Constants.flywheelConstants.mmJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    flywheelMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void fShoot() {
    flywheelMotor.setControl(fControl.withVelocity(Constants.flywheelConstants.shootVelo));
  }

  public Command fShootCommand() {
    return this.runOnce(() -> fShoot());
  }

  public void fStop() {
    flywheelMotor.setControl(fControl.withVelocity(Constants.flywheelConstants.stopVelo));
  }

  public Command fStopCommand() {
    return this.runOnce(() -> fStop());
  }

  public void fDrop() {
    flywheelMotor.setControl(fControl.withVelocity(Constants.flywheelConstants.dropVelo));
  }

  public Command fDropCommand() {
    return this.runOnce(() -> fDrop());
  }

  public void fIntake() {
    flywheelMotor.setControl(fControl.withVelocity(Constants.flywheelConstants.intakeVelo));
  }

  public Command fIntakeCommand() {
    return this.runOnce(() -> fIntake());
  }
}
