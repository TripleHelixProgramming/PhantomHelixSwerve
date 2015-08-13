package org.usfirst.frc.team2363.robot.subsystems;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule implements PIDSource, PIDOutput {
	
	private final CANTalon turnMotor;
	private final CANTalon driveMotor;
	
	private PIDController angleController;
	
	private final int OFFSET;
	private static final double MAX_SPEED = 1;
	private static final double ENCODER_MAX = 1000;
	
	public SwerveModule(int turnMotorChannel, int driveMotorChannel, int offset) {
		turnMotor = new CANTalon(turnMotorChannel);
		turnMotor.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
		turnMotor.changeControlMode(ControlMode.PercentVbus);
		turnMotor.reverseOutput(true);
		turnMotor.disable();
		turnMotor.enableControl();
		
		driveMotor = new CANTalon(driveMotorChannel);
		driveMotor.setPID(0, 0, 0);
		driveMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		driveMotor.changeControlMode(ControlMode.PercentVbus);
		
		OFFSET = offset;
		
		angleController = new PIDController(0.001, 0, 0, this, this);
		angleController.setInputRange(-180, 180);
		angleController.setContinuous(true);
		angleController.setOutputRange(-1, 1);
		angleController.enable();
	}
	
	/**
	 * Set the desired angle of the module
	 * @param angle the desired angle of the module
	 */
	public void setAngle(double angle) {
		SmartDashboard.putNumber("Setpoint", angle);
		angleController.setSetpoint(angle);
	}
	
	/**
	 * Get the current angle of the module
	 * @return the current angle of the module
	 */
	public double getAngle() {
		double rawAngle = turnMotor.getPosition();
		SmartDashboard.putNumber("Raw Angle", rawAngle);
		double offsetAngle = (rawAngle - OFFSET + ENCODER_MAX) % ENCODER_MAX;
		double convertedAngle = convertToAbsoluteAngle(offsetAngle);
		return convertedAngle;
	}
	
	private double convertToAbsoluteAngle(double rawAngle) {
		return (rawAngle / ENCODER_MAX * 360.0) - 180.0;
	}
	
	/**
	 * Set the speed of the drive wheel
	 * @param speed the desired speed of the drive wheel
	 */
	public void setSpeed(double speed) {
		driveMotor.set(speed * MAX_SPEED);
	}
	
	/**
	 * Get the current speed of the drive wheel
	 * @return the current speed of the drive wheel
	 */
	public double getSpeed() {
		return 0;
	}
	
	@Override
	public void pidWrite(double output) {
		SmartDashboard.putNumber("Output", output);
		turnMotor.set(output);
	}

	@Override
	public double pidGet() {
		SmartDashboard.putNumber("Angle", getAngle());
		return getAngle();
	}
}
