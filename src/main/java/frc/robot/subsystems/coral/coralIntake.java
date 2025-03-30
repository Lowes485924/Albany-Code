package frc.robot.subsystems.coral;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
// import au.grapplerobotics.LaserCan.ConfigurationFailedException;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class coralIntake extends SubsystemBase {
  TalonFX coralIntakeMotor =
      new TalonFX(Constants.coralIntakeConstants.canID, Constants.canBusName);
  LaserCan lc = new LaserCan(3);
  LaserCan.Measurement measurement;
  double i;
  public boolean coralPresent = false;

  public coralIntake() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = Constants.coralIntakeConstants.statCurLim;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable =
        Constants.coralIntakeConstants.enableStatCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = Constants.coralIntakeConstants.supCurLim;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable =
        Constants.coralIntakeConstants.enableSupCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerLimit = Constants.coralIntakeConstants.supLowLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = Constants.coralIntakeConstants.supLowTime;
    talonFXConfigs.MotorOutput.Inverted = Constants.coralIntakeConstants.inverted;
    talonFXConfigs.MotorOutput.NeutralMode = Constants.coralIntakeConstants.neutralMode;

    coralIntakeMotor.getConfigurator().apply(talonFXConfigs);
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 8, 8));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }
  }

  public void coralScore() {
    i = 0;
    coralIntakeMotor.set(Constants.coralIntakeConstants.scoreVelo);
  }

  public Command coralScoreCommand() {
    return this.run(() -> coralScore());
  }

  public void coralDrop() {
    i = 0;
    coralIntakeMotor.set(Constants.coralIntakeConstants.dropVelo);
  }

  public Command coralDropCommand() {
    return this.run(() -> coralDrop());
  }

  public void coralStop() {
    coralIntakeMotor.stopMotor();
  }

  public Command coralStopCommand() {
    return this.run(() -> coralStop());
  }

  public void coralLoad() {
    if (coralPresent && i > 1) {
      coralIntakeMotor.set(Constants.coralIntakeConstants.stopVelo);
    } else {
      coralIntakeMotor.set(Constants.coralIntakeConstants.intakeVelo);
      measurement = lc.getMeasurement();
      if (measurement.distance_mm < 100) {
        coralPresent = true;
        i++;
      }
    }
  }

  public Command coralLoadCommand() {
    return this.run(() -> coralLoad());
  }
}
