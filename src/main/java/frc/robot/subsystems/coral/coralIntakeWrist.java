package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class coralIntakeWrist extends SubsystemBase {
  TalonFX coralIntakeWristMotor =
      new TalonFX(Constants.coralIntakeWristConstants.canID, Constants.canBusName);
  final MotionMagicDutyCycle ciwControl = new MotionMagicDutyCycle(0);

  public coralIntakeWrist() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit =
        Constants.coralIntakeWristConstants.statCurLim;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable =
        Constants.coralIntakeWristConstants.enableStatCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = Constants.coralIntakeWristConstants.supCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.coralIntakeWristConstants.enableSupCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerLimit =
        Constants.coralIntakeWristConstants.supLowLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime =
        Constants.coralIntakeWristConstants.supLowTime;
    talonFXConfigs.MotorOutput.Inverted = Constants.coralIntakeWristConstants.inverted;
    talonFXConfigs.MotorOutput.NeutralMode = Constants.coralIntakeWristConstants.neutralMode;

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kG = Constants.coralIntakeWristConstants.kG;
    slot0Configs.kS = Constants.coralIntakeWristConstants.kS;
    slot0Configs.kV = Constants.coralIntakeWristConstants.kV;
    slot0Configs.kA = Constants.coralIntakeWristConstants.kA;
    slot0Configs.kP = Constants.coralIntakeWristConstants.kP;
    slot0Configs.kI = Constants.coralIntakeWristConstants.kI;
    slot0Configs.kD = Constants.coralIntakeWristConstants.kD;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.coralIntakeWristConstants.mmCruiseVel;
    motionMagicConfigs.MotionMagicAcceleration = Constants.coralIntakeWristConstants.mmAccel;
    motionMagicConfigs.MotionMagicJerk = Constants.coralIntakeWristConstants.mmJerk;

    coralIntakeWristMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void ciwStore() {
    coralIntakeWristMotor.setControl(
        ciwControl.withPosition(Constants.coralIntakeWristConstants.storePos));
  }

  public Command ciwStoreCommand() {
    return this.runOnce(() -> ciwStore());
  }

  public void ciwSource() {
    coralIntakeWristMotor.setControl(
        ciwControl.withPosition(Constants.coralIntakeWristConstants.sourcePos));
  }

  public Command ciwSourceCommand() {
    return this.runOnce(() -> ciwSource());
  }

  public void ciwL1() {
    coralIntakeWristMotor.setControl(
        ciwControl.withPosition(Constants.coralIntakeWristConstants.L1Pos));
  }

  public Command ciwL1Command() {
    return this.runOnce(() -> ciwL1());
  }

  public void ciwL23() {
    coralIntakeWristMotor.setControl(
        ciwControl.withPosition(Constants.coralIntakeWristConstants.L23Pos));
  }

  public Command ciwL23Command() {
    return this.runOnce(() -> ciwL23());
  }

  public void ciwL4() {
    coralIntakeWristMotor.setControl(
        ciwControl.withPosition(Constants.coralIntakeWristConstants.L4Pos));
  }

  public Command ciwL4Command() {
    return this.runOnce(() -> ciwL4());
  }

  public void ciwd() {
    coralIntakeWristMotor.setControl(
        ciwControl.withPosition(Constants.coralIntakeWristConstants.dPos));
  }

  public Command ciwDCommand() {
    return this.runOnce(() -> ciwd());
  }
}
