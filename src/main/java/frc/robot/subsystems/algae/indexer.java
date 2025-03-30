package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class indexer extends SubsystemBase {
  TalonFX indexerMotor = new TalonFX(Constants.indexerConstants.canID, Constants.canBusName);

  public indexer() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = Constants.indexerConstants.statCurLim;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable =
        Constants.indexerConstants.enableStatCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = Constants.indexerConstants.supCurLim;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable =
        Constants.indexerConstants.enableSupCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerLimit = Constants.indexerConstants.supLowLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = Constants.indexerConstants.supLowTime;
    talonFXConfigs.MotorOutput.Inverted = Constants.indexerConstants.inverted;
    talonFXConfigs.MotorOutput.NeutralMode = Constants.indexerConstants.neutralMode;

    indexerMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void iIntake() {
    indexerMotor.set(Constants.indexerConstants.intakeVelo);
  }

  public Command iIntakeCommand() {
    return this.run(() -> iIntake());
  }

  public void iShoot() {
    indexerMotor.set(Constants.indexerConstants.shootVelo);
  }

  public Command iShootCommand() {
    return this.run(() -> iShoot()).withTimeout(Constants.indexerConstants.shootTime);
  }

  public void iDrop() {
    indexerMotor.set(Constants.indexerConstants.dropVelo);
  }

  public Command iDropCommand() {
    return this.run(() -> iDrop());
  }

  public void iStop() {
    indexerMotor.set(Constants.indexerConstants.stopVelo);
  }

  public Command iStopCommand() {
    return this.run(() -> iStop());
  }
}
