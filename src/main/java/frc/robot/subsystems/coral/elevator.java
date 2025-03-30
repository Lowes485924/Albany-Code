package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class elevator extends SubsystemBase {
  TalonFX elevatorMotor = new TalonFX(Constants.elevatorConstants.canID, Constants.canBusName);
  final MotionMagicTorqueCurrentFOC elevatorControl = new MotionMagicTorqueCurrentFOC(0);

  public elevator() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = Constants.elevatorConstants.statCurLim;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable =
        Constants.elevatorConstants.enableStatCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = Constants.elevatorConstants.supCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.elevatorConstants.enableSupCurLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerLimit = Constants.elevatorConstants.supLowLim;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = Constants.elevatorConstants.supLowTime;
    talonFXConfigs.MotorOutput.Inverted = Constants.elevatorConstants.inverted;
    talonFXConfigs.MotorOutput.NeutralMode = Constants.elevatorConstants.neutralMode;

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kG = Constants.elevatorConstants.kG;
    slot0Configs.kS = Constants.elevatorConstants.kS;
    slot0Configs.kV = Constants.elevatorConstants.kV;
    slot0Configs.kA = Constants.elevatorConstants.kA;
    slot0Configs.kP = Constants.elevatorConstants.kP;
    slot0Configs.kI = Constants.elevatorConstants.kI;
    slot0Configs.kD = Constants.elevatorConstants.kD;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.elevatorConstants.mmCruiseVel;
    motionMagicConfigs.MotionMagicAcceleration = Constants.elevatorConstants.mmAccel;
    motionMagicConfigs.MotionMagicJerk = Constants.elevatorConstants.mmJerk;

    elevatorMotor.getConfigurator().apply(talonFXConfigs);
  }

  public StatusSignal<Angle> getElevatorPosition() {
    return elevatorMotor.getPosition();
  }

  public void eStore() {
    elevatorMotor.setControl(elevatorControl.withPosition(Constants.elevatorConstants.storePos));
  }

  public Command eStoreCommand() {
    return this.runOnce(() -> eStore());
  }

  public void eSource() {
    elevatorMotor.setControl(elevatorControl.withPosition(Constants.elevatorConstants.sourcePos));
  }

  public Command eSourceCommand() {
    return this.runOnce(() -> eSource());
  }

  public void eL1() {
    elevatorMotor.setControl(elevatorControl.withPosition(Constants.elevatorConstants.L1Pos));
  }

  public Command eL1Command() {
    return this.runOnce(() -> eL1());
  }

  public void eL2() {
    elevatorMotor.setControl(elevatorControl.withPosition(Constants.elevatorConstants.L2Pos));
  }

  public Command eL2Command() {
    return this.runOnce(() -> eL2());
  }

  public void eL3() {
    elevatorMotor.setControl(elevatorControl.withPosition(Constants.elevatorConstants.L3Pos));
  }

  public Command eL3Command() {
    return this.runOnce(() -> eL3());
  }

  public void eL4() {
    elevatorMotor.setControl(elevatorControl.withPosition(Constants.elevatorConstants.L4Pos));
  }

  public Command eL4Command() {
    return this.runOnce(() -> eL4());
  }

  public void lds() {
    elevatorMotor.setControl(elevatorControl.withPosition(Constants.elevatorConstants.ldPos));
  }

  public Command ldCommand() {
    return this.runOnce(() -> lds());
  }

  public void hds() {
    elevatorMotor.setControl(elevatorControl.withPosition(Constants.elevatorConstants.hdPos));
  }

  public Command hdCommand() {
    return this.runOnce(() -> hds());
  }
}
