package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class algaeIntake extends SubsystemBase {
  TalonFX algaeIntakeMotor =
      new TalonFX(Constants.algaeIntakeConstants.canID, Constants.canBusName);
  final MotionMagicVelocityVoltage aiControl = new MotionMagicVelocityVoltage(0);

  public algaeIntake() {

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = Constants.algaeIntakeConstants.statCurLim;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable =
        Constants.algaeIntakeConstants.enableStatCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = Constants.algaeIntakeConstants.supCurLim;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable =
        Constants.algaeIntakeConstants.enableSupCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerLimit = Constants.algaeIntakeConstants.supLowLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = Constants.algaeIntakeConstants.supLowTime;
    talonFXConfigs.MotorOutput.Inverted = Constants.algaeIntakeConstants.inverted;
    talonFXConfigs.MotorOutput.NeutralMode = Constants.algaeIntakeConstants.neutralMode;
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS =
        Constants.algaeIntakeConstants.kS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV =
        Constants.algaeIntakeConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA =
        Constants.algaeIntakeConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP =
        Constants.algaeIntakeConstants
            .kP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = Constants.algaeIntakeConstants.kI; // no output for integrated error
    slot0Configs.kD =
        Constants.algaeIntakeConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        Constants.algaeIntakeConstants.mmCruiseVel; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        Constants.algaeIntakeConstants.mmAccel; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk =
        Constants.algaeIntakeConstants.mmJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    algaeIntakeMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void aIntake() {
    algaeIntakeMotor.setControl(aiControl.withVelocity(Constants.algaeIntakeConstants.intakeVelo));
  }

  public Command aIntakeCommand() {
    return this.run(() -> aIntake());
  }

  public void aShoot() {
    algaeIntakeMotor.setControl(aiControl.withVelocity(Constants.algaeIntakeConstants.shootVelo));
  }

  public Command aShootCommand() {
    return this.runOnce(() -> aShoot());
  }

  public void aDrop() {
    algaeIntakeMotor.setControl(aiControl.withVelocity(Constants.algaeIntakeConstants.dropVelo));
  }

  public Command aDropCommand() {
    return this.runOnce(() -> aDrop());
  }

  public void aStop() {
    algaeIntakeMotor.setControl(aiControl.withVelocity(Constants.algaeIntakeConstants.stopVelo));
  }

  public Command aStopCommand() {
    return this.runOnce(() -> aStop());
  }
}
