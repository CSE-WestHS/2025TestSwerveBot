package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO flywheel;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final String name;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  public Flywheel(FlywheelIO io, FlywheelGains gains) {
    flywheel = io;

    name = io.getName();

    kP = new LoggedTunableNumber(name + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(name + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(name + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(name + "/Gains/kS", gains.kS());
    kV = new LoggedTunableNumber(name + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(name + "/Gains/kA", gains.kA());
  }

  @Override
  public void periodic() {
    flywheel.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          flywheel.setGains(
              new FlywheelGains(values[0], values[1], values[2], values[3], values[4], values[5]));
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);
  }

  public void setVoltage(double voltage) {
    flywheel.setVoltage(voltage);
  }

  public void setVelocity(double velocity) {
    flywheel.setVelocity(velocity);
  }

  public void setPosition(double position) {
    flywheel.setPosition(position);
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public double getPosition() {
    return inputs.motorPositions[0];
  }

  public double getVelocitySetpoint() {
    return inputs.desiredVelocity;
  }
}
