// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 31;
   // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(96.855);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 34;
   // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(178.42);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 33;
   // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(358.95);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 32;
   // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0);

    public static final int MOTOR_INTAKE_ROLLERS_ONE = 18; // SparkMax, NEO
    //public static final int MOTOR_FEEDER_TOP_ONE = 19; // SparkMax, NEO
    //public static final int MOTOR_FEEDER_BOTTOM_ONE = 20; // SparkMax, NEO
    public static final int MOTOR_CONVEYOR_BOTTOM_ONE = 9; // SparkMax, NEO550
    public static final int MOTOR_CONVEYOR_TOP_ONE = 10; // SparkMax, NEO550
    //public static final int MOTOR_TURRET_AZIMUTH_ONE = 11; // SparkMax, NEO550
    public static final int MOTOR_BOOSTER_ONE = 11; // SparkMax, NEO
    public static final int MOTOR_HOOD_ONE = 14; // SparkMax, NEO550
    public static final int MOTOR_SHOOTER_WHEEL_ONE = 12; // SparkMax, NEO
    public static final int MOTOR_SHOOTER_WHEEL_TWO = 13; // SparkMax, NEO
    //public static final int MOTOR_CLIMBER_ONE = 16; // SparkMax, NEO
    //public static final int MOTOR_CLIMBER_TWO = 17; // SparkMax, NEO

    // PH Ports
    public static final int CYLINDER_FEEDER_GATE_TOP_ONE = 6; 
    public static final int CYLINDER_FEEDER_GATE_BOTTOM_ONE = 5;
    public static final int CYLINDER_INTAKE_PIVOT_ONE = 7;

    public static final double MAXIMUM_VOLTAGE = 12.0;

    public static final double K_MAX_SPEED = 3.0; // 3 meters per second
    public static final double K_MAX_ANGULAR_SPEED = Units.rotationsToRadians(0.5); // 1/2 rotation per second

    public static final double DRIVE_PID0_P = 0.08;//8E-02;
    public static final double DRIVE_PID0_I = 0;
    public static final double DRIVE_PID0_D = 0;
    public static final double DRIVE_PID0_F = 0.00156;

    public static final double ROTATION_PID0_P = 1.5;
    public static final double ROTATION_PID0_I = 0.0;
    public static final double ROTATION_PID0_D = 0;
    public static final double ROTATION_PID0_F = 0.00156;

    public static final double CONVEYOR_BOTTOM_PID1_P = 4.5776E-10;
    public static final double CONVEYOR_BOTTOM_PID1_I = 0;
    public static final double CONVEYOR_BOTTOM_PID1_D = 0;
    public static final double CONVEYOR_BOTTOM_PID1_F = 1.5e-5;

    public static final double CONVEYOR_BOTTOM_KS = 0.1652;
    public static final double CONVEYOR_BOTTOM_KV = 0.48782;
    public static final double CONVEYOR_BOTTOM_KA = 0.019205;

    public static final double CONVEYOR_TOP_PID1_P = 2.2382E-09;
    public static final double CONVEYOR_TOP_PID1_I = 0;
    public static final double CONVEYOR_TOP_PID1_D = 0;
    public static final double CONVEYOR_TOP_PID1_F = 1.5e-5;

    public static final double CONVEYOR_TOP_KS = 0.14207;
    public static final double CONVEYOR_TOP_KV = 0.49344;
    public static final double CONVEYOR_TOP_KA = 0.024371;

    public static final double ROTATION_PID0_MAX_ACCEL = 5700; // RPM
    public static final double ROTATION_PID0_MAX_VELOCITY = 3000;

    public static final int ENCODER_RESET_ITERATIONS = 500;
    public static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

    public static final double PID_IZ_VALUE = 0;

    public static final double OUTPUT_RANGE_MIN = -1;
    public static final double OUTPUT_RANGE_MAX = 1;

    /*public static final double DRIVE_KS = 0.56385;
    public static final double DRIVE_KV = 2.3;//2.379;
    public static final double DRIVE_KA = 0.7754;*/

    public static final double DRIVE_KS = 0.02;
    public static final double DRIVE_KV = 2.30;
    public static final double DRIVE_KA = 0.1;

    public static final double ROTATION_KS = 0.17085;
    public static final double ROTATION_KV = 1.5851;
    public static final double ROTATION_KA = 0.056595;

    public static final boolean DRIVE_MOTOR_INVERSION = true;
    public static final boolean ROTATION_MOTOR_INVERSION = false;
    public static final boolean CONVEYOR_TOP_MOTOR_INVERSION = false;
    public static final boolean CONVEYOR_BOTTOM_MOTOR_INVERSION = true;

    public static final int SECONDS_PER_MINUTE = 60;

    // Gear Ratios
    public static final double MK4_L2_DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double MK4_L2_ROTATION_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
    public static final double MK4_L2_WHEEL_DIAMETER = 0.10033; // Meters

    //public static final double GEAR_RATIO_FEEDER_TOP = 5.0 / 1.0; //FIXME
    //public static final double GEAR_RATIO_FEEDER_BOTTOM = 5.0 / 1.0; //FIX ME
    //public static final double GEAR_RATIO_INTAKE_ROLLERS = 2.0 / 1.0; // Overdriven, 2020 bot
    public static final double GEAR_RATIO_CONVEYOR_BOTTOM = 4.0 / 1.0; // Direct Drive
    public static final double GEAR_RATIO_CONVEYOR_TOP = 4.0 / 1.0; // Direct Drive
    //public static final double GEAR_RATIO_CLIMBER = 320.0 / 1.0; // Overdriven, 2020 bot
    public static final double GEAR_RATIO_SHOOTER = 4.0 / 3.0;
    public static final double GEAR_RATIO_BOOSTER = 2.0 / 1.0;
    //public static final double GEAR_RATIO_TURRET = 96.5;// 12*138/15;
    public static final double GEAR_RATIO_HOOD = 20 * 450 / 24;

    // Current Limits
    public static final int CURRENT_LIMIT_DRIVE_MOTOR = 40;
    public static final int CURRENT_LIMIT_ROTATION_MOTOR = 40;
    public static final int CURRENT_LIMIT_SHOOTER_WHEEL = 60;
    public static final int CURRENT_LIMIT_BOOSTER = 30;
    public static final int CURRENT_LIMIT_TURRET_AZIMUTH = 20;
    public static final int CURRENT_LIMIT_HOOD = 20;
    //public static final int CURRENT_LIMIT_FEEDER_TOP = 40;
    //public static final int CURRENT_LIMIT_FEEDER_BOTTOM = 40;
    //public static final int CURRENT_LIMIT_CLIMBER = 80;
    //public static final int CURRENT_LIMIT_INTAKE_ROLLERS = 60;
    public static final int CURRENT_LIMIT_CONVEYOR_BOTTOM = 20;
    public static final int CURRENT_LIMIT_CONVEYOR_TOP = 20;

    public static final int MIN_PRESSURE = 90;
    public static final int MAX_PRESSURE = 110;

    public static final boolean INTAKE_EXTEND = true;
    public static final boolean INTAKE_RETRACT = false;
    public static final boolean FRONT_GATE_EXTEND = true;
    public static final boolean FRONT_GATE_RETRACT = false;
    public static final boolean BACK_GATE_EXTEND = true;
    public static final boolean BACK_GATE_RETRACT = false;

    // XBox controller ports
    public static final int OPERATOR_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER = 1;
    public static final double TRIGGER_THRESHOLD = 0.1;
    public static final double JOYSTICK_THRESHOLD = 0.1;

    public static final double LL_TARGET_HEIGHT  = 101.625 +1.2;
    public static final double LL_MOUNT_ANGLE = 40.0;
    public static final double LL_MOUNT_HEIGHT = 42.9;
    public static final double LL_POSE_TOLERANCE = 3.5; //meters
    public static final double LL_LATENCY = 0.02; //seconds
    
    public static final int HOOD_MAX_LIMIT = 22;
    public static final int HOOD_MIN_LIMIT = 0;

    public static final int CONVEYOR_INTAKE_SPEED_TOP = 1500;
    public static final int CONVEYOR_INTAKE_SPEED_BOTTOM = 1500;
    public static final int CONVEYOR_OUTTAKE_SPEED_TOP = -500;
    public static final int CONVEYOR_OUTTAKE_SPEED_BOTTOM = -500;
    public static final int SHOOTER_MAX_SPEED = 4000; //RPM
    
    public static final int FLYWHEEL_SPEED_FENDER_HIGH = 2250;
    public static final double HOOD_ANGLE_FENDER_HIGH = 1.5;

    public static final boolean GYRO_REVERSED = true;

    
    public static final double MAX_SPEED_METERS_PER_SECOND = 2;
    public static final double MAX_ACTUAL_SPEED_METERS_PER_SECOND = 4;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.75;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;


    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 0.5;

    public static final double TURN_P = 0.5;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0.05;
    public static final double TURN_TOLERANCE = 2; //degrees 
    public static final double TURN_RATE_TOLERANCE = 5; //degrees per second

    public static final double LL_TURN_P = 0.2;
    public static final double LL_TURN_I = 0;
    public static final double LL_TURN_D = 0;
    public static final double LL_TURN_TOLERANCE = 2; //degrees 
    public static final double LL_TURN_RATE_TOLERANCE = 5; //degrees per second

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double HOOD_ZEROING_SPEED = -0.165;
    public static final int NEO550_CURRENT_SPIKE = 20;

    public static final double HOOD_ANGLE_LOW_LOB = 20;
    public static final double FLYWHEEL_SPEED_LOW_LOB = 1000;

    public static final double SHOOTER_KS = 0.35321;
    public static final double SHOOTER_KV = 0.13604;
    public static final double SHOOTER_KA = 0.0109;

    public static final double BOOSTER_KS = 0.76732;
    public static final double BOOSTER_KV = 0.066627;
    public static final double BOOSTER_KA = 0.16846;

    public static final double SHOOTER_PID0_P = 6.1481E-12;
    public static final double SHOOTER_PID0_I = 0;
    public static final double SHOOTER_PID0_D = 0;
    public static final double SHOOTER_PID0_F = 0.0000156;

    public static final double BOOSTER_PID0_P = 2.0619E-06;
    public static final double BOOSTER_PID0_D = 0;
    public static final double BOOSTER_PID0_I = 0;
    public static final double BOOSTER_PID0_F = 0.00156;

    public static final double HOOD_PID1_P = 1E-00; //position
    public static final double HOOD_PID1_I = 0;
    public static final double HOOD_PID1_D = 0.1;
    public static final double HOOD_PID1_F = 3e-3;

    public static final double BOOSTER_SPEED = 300;
    
}
