package org.usfirst.frc.team2363.robot.subsystems;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
	
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
		turnMotor.enableBrakeMode(true);
		turnMotor.reverseSensor(true);
		
		driveMotor = new CANTalon(driveMotorChannel);
		driveMotor.setPID(0, 0, 0);
		driveMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		driveMotor.changeControlMode(ControlMode.PercentVbus);
		driveMotor.disableControl();
		
		OFFSET = offset;
		
		angleController = new PIDController(0.01, 0, 0, new TalonInput(), turnMotor);
		angleController.setInputRange(-180, 180);
		angleController.setContinuous(true);
		angleController.enable();
	}
	
	/**
	 * Set the desired angle of the module
	 * @param angle the desired angle of the module
	 */
	public void setAngle(double angle) {
		SmartDashboard.putNumber("Setpoint", angle);
		angleController.setSetpoint(angle);
		SmartDashboard.putNumber("Error", angleController.getError());
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
		SmartDashboard.putNumber("Converted Angle", convertedAngle);
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
	
	private class TalonInput implements PIDSource {

		@Override
		public double pidGet() {
			SmartDashboard.putNumber("Angle", getAngle());
			return getAngle();
		}
	}
}
