package frc.robot.subsystems.PositionalPID;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PositionalPID.PositionalPIDConstants.PositionalPIDGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class PositionalPID extends SubsystemBase {
  private final PositionalPIDIO positionalpid;
  private final PositionalPIDIOInputsAutoLogged inputs = new PositionalPIDIOInputsAutoLogged();

  private final String name;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  public PositionalPID(PositionalPIDIO io, PositionalPIDGains gains) {
    positionalpid = io;

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
    positionalpid.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    positionalpid.setPosition(50);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          positionalpid.setGains(
              new PositionalPIDGains(
                  values[0], values[1], values[2], values[3], values[4], values[5]));
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);
  }

  public void setVoltage(double voltage) {
    positionalpid.setVoltage(voltage);
  }

  public void setVelocity(double velocity) {
    positionalpid.setVelocity(velocity);
  }

  public void setPosition(double position) {
    positionalpid.setPosition(position);
  }

  // public void incrementPosition(double delta) {
  //   goal.position += delta;
  // }

  // public void decrementPosition(double delta) {
  //   goal.position -= delta;
  // }

  public double getVelocity() {
    return inputs.velocity;
  }

  public double getPosition() {
    return inputs.motorPositions[0];
  }

  public double getVelocitySetpoint() {
    return inputs.desiredVelocity;
  }

  // public boolean isFInished() {
  //   return Math.abs(inputs.motorPositions[0] - goal.position) < 5;
  // }
}
