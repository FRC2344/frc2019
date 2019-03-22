/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The MotionMagic example demonstrates the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually. This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction. If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Ensure your feedback device is in-phase with the motor,
 * and you have followed the walk-through in the Talon SRX Software Reference Manual.
 * 
 * Controls:
 * Button 1: When held, put Talon in Motion Magic mode and allow Talon to drive [-10, 10] 
 * 	rotations.
 * Button 2: When pushed, the selected feedback sensor gets zero'd
 * Button 5(Left shoulder): When pushed, will decrement the smoothing of the motion magic down to 0
 * Button 6(Right shoulder): When pushed, will increment the smoothing of the motion magic up to 8
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon SRX forward and reverse, use to confirm hardware setup.
 * 	+ Motion Maigic: SErvo Talon SRX forward and reverse, [-10, 10] rotations.
 * 
 * Gains for Motion Magic may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

	/* Hardware */
	CANSparkMax left, left_primary, left_secondary, right, right_primary, right_secondary;
	WPI_TalonSRX elevator, intake, lift1, lift2, lift3, lift4;
	Solenoid shift, _intake, front_hatch, back_hatch, back_push;
	Joystick driver, operator;
	DifferentialDrive drive;

	/* create some followers */
	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	public void robotInit() {
		left = new CANSparkMax(1, MotorType.kBrushless);
		left_primary = new CANSparkMax(2, MotorType.kBrushless);
		left_secondary = new CANSparkMax(3, MotorType.kBrushless);
		right = new CANSparkMax(4, MotorType.kBrushless);
		right_primary = new CANSparkMax(5, MotorType.kBrushless);
		right_secondary = new CANSparkMax(6, MotorType.kBrushless);
		

		System.out.println("left" + left.getFirmwareVersion());
		System.out.println("left_primary" + left_primary.getFirmwareVersion());
		System.out.println("left_secondary" + left_secondary.getFirmwareVersion());
		System.out.println("right" + right.getFirmwareVersion());
		System.out.println("right_primary" + right_primary.getFirmwareVersion());
		System.out.println("right_secondary" + right_secondary.getFirmwareVersion());
		/* Followers */
		left_primary.follow(left);
		left_secondary.follow(left);
		right_primary.follow(right);
		right_secondary.follow(right);

		/* Talons */
		elevator = new WPI_TalonSRX(10);
		intake = new WPI_TalonSRX(7);
		lift1 = new WPI_TalonSRX(8);
		lift2 = new WPI_TalonSRX(9);
		lift3 = new WPI_TalonSRX(11);
		lift4 = new WPI_TalonSRX(12);
		driver = new Joystick(0);
		operator = new Joystick(1);

		drive = new DifferentialDrive(left, right);
		shift = new Solenoid(0);
		front_hatch = new Solenoid(1);
		back_hatch = new Solenoid(2);
		_intake = new Solenoid(3);
		back_push = new Solenoid(4);
		/* Factory default hardware to prevent unexpected behavior */
		elevator.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											Constants.kPIDLoopIdx, 
											Constants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		elevator.setSensorPhase(true);
		elevator.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		elevator.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		elevator.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		elevator.configNominalOutputForward(0, Constants.kTimeoutMs);
		elevator.configNominalOutputReverse(0, Constants.kTimeoutMs);
		elevator.configPeakOutputForward(1, Constants.kTimeoutMs);
		elevator.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		elevator.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		elevator.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		elevator.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		elevator.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		elevator.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		elevator.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		elevator.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* Zero the sensor */
		elevator.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	public void robotPeriodic(){
		_sb.append("\tPosition:" + elevator.getSelectedSensorPosition(0));
	}
	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		/* DRIVE */
		drive.arcadeDrive(-driver.getRawAxis(1), driver.getRawAxis(2));

		if(driver.getRawButtonPressed(6)){
			shift.set(true);
		} else if(driver.getRawButtonReleased(6)){
			shift.set(false);
		}

		if(operator.getRawButtonPressed(5)){
			_intake.set(true);
		} else if(operator.getRawButtonReleased(5)){
			_intake.set(false);
		}
		/* Operator */
		
		
		/* Get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * operator.getY();
		if (Math.abs(leftYstick) < 0.10) { leftYstick = 0;} /* deadband 10% */

		/* Get current Talon SRX motor output */
		double motorOutput = elevator.getMotorOutputPercent();

		/* Prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(elevator.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

		elevator.set(ControlMode.PercentOutput, leftYstick);
		
		
		/**
		 * Peform Motion Magic when Button 1 is held,
		 * else run Percent Output, which can be used to confirm hardware setup.
		 */
		if(operator.getRawButton(1)) {
			elevator.set(ControlMode.MotionMagic, 2000);
		}

		if(operator.getRawButton(2)){
			elevator.set(ControlMode.MotionMagic, 10000);
		}
		if(operator.getRawButton(3)){
			elevator.set(ControlMode.MotionMagic, 15000);
		}
		if(operator.getRawButton(4)){
			elevator.set(ControlMode.MotionMagic, 20000);
		}

		if(operator.getRawButtonPressed(5)){
			intake.set(ControlMode.PercentOutput, .25);
		} else{
			intake.stopMotor();
		}

	

		
		if(operator.getRawButton(6))
		{
			/* Clear sensor positions */
			elevator.getSensorCollection().setQuadraturePosition(0, 0);

			System.out.println("Voltage is: " + elevator.getBusVoltage());
		}

		if(operator.getRawButtonPressed(7))
		{
			/* Decrease smoothing */
			_smoothing--;
			if(_smoothing < 0) _smoothing = 0;
			elevator.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		}
		if(operator.getRawButtonPressed(8))
		{
			/* Increase smoothing */
			_smoothing++;
			if(_smoothing > 8) _smoothing = 8;
			elevator.configMotionSCurveStrength(_smoothing);
			
			System.out.println("Smoothing is set to: " + _smoothing);
		}

		/* Instrumentation */
		Instrum.Process(elevator, _sb);
	}
}
