package frc.robot.subsystems.algae;

// import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeIntakeWrist extends SubsystemBase {
  SparkMax algaeIntakeWristMotor1 = new SparkMax(2, MotorType.kBrushless);
  SparkClosedLoopController aiw1Pid = algaeIntakeWristMotor1.getClosedLoopController();
  SparkMaxConfig config;

  SparkMax algaeIntakeWristMotor2 = new SparkMax(4, MotorType.kBrushless);
  SparkClosedLoopController aiw2Pid = algaeIntakeWristMotor2.getClosedLoopController();
  SparkMaxConfig config2;

  public algaeIntakeWrist() {
    config = new SparkMaxConfig();

    config.inverted(false).smartCurrentLimit(12).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(1).velocityConversionFactor(6000);
    config
        .closedLoop
        // .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(2.5)
        .i(0.0000)
        .d(0.0)
        .outputRange(-1.0, 1.0);

    algaeIntakeWristMotor1.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config2 = new SparkMaxConfig();

    config2.inverted(false).smartCurrentLimit(12).idleMode(IdleMode.kBrake);
    config2.encoder.positionConversionFactor(1).velocityConversionFactor(6000);
    config2
        .closedLoop
        // .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(2.8)
        .i(0.0000)
        .d(0.001)
        .outputRange(-1.0, 1.0);

    algaeIntakeWristMotor2.configure(
        config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void awIn() {
    aiw1Pid.setReference(.24, ControlType.kPosition);
    aiw2Pid.setReference(.76, ControlType.kPosition);
  }

  public Command awInCommand() {
    return this.run(() -> awIn());
  }

  public void awOut() {
    aiw1Pid.setReference(.10, ControlType.kPosition);
    aiw2Pid.setReference(.92, ControlType.kPosition);
  }

  public Command awOutCommand() {
    return this.run(() -> awOut());
  }

  public void awMid() {
    aiw1Pid.setReference(.18, ControlType.kPosition);
    aiw2Pid.setReference(.84, ControlType.kPosition);
  }

  public Command awMidCommand() {
    return this.run(() -> awMid());
  }
}
