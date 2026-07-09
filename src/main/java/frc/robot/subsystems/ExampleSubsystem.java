// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

	import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
	import edu.wpi.first.units.measure.Angle; 
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;

import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ExampleSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(50, 0, 0)
      .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(50, 0, 0)
      // Feedforward Constants
      .withFeedforward(new ArmFeedforward(0, 0, 0))
      .withSimFeedforward(new ArmFeedforward(0, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
      // your motor.
      // You could also use .withGearing(12) which does the same thing.
      // .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withGearing(new MechanismGearing(15))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))

       // Starting position is where your arm starts
  .withStartingPosition(Degrees.of(-5))
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(Degrees.of(-20), Degrees.of(10));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(30, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

   private ArmConfig armCfg = new ArmConfig()
   .withSmartMotorController(sparkSmartMotorController)
  // Hard limit is applied to the simulation.
  .withHardLimits(Degrees.of(-30), Degrees.of(40))
  // Length and mass of your arm for sim.
  .withLength(Feet.of(3))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm arm = new Arm(armCfg);

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  	  /**
   * Run the arm to the given angle, does not stop when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command run(Angle angle) { 
    return arm.run(angle);}
  
  /**
   * Run the arm to the given angle, ends the command when the arm reaches the setpoint within tolerance.
   * @param angle Angle to go to.
   * @param tolerance Angle tolerance for completion.
   * @return A Command
   */
  public Command runTo(Angle angle, Angle tolerance) { 
    return arm.runTo(angle, tolerance);}
  
  /**
   * Set arm closed loop controller to go to the specified mechanism position.
   * @param angle Angle to go to.
   */
  public void setAngleSetpoint(Angle angle) { arm.setMechanismPositionSetpoint(angle); }

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { return arm.set(dutycycle);}


  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      // This method will be called once per scheduler run
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
     // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }
}
