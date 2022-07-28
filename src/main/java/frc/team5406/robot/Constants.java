// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

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
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 34;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(96.855);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 32;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(178.42);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 31;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(358.95);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 33;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(285.12);

    public static final int MOTOR_INTAKE_ROLLERS_ONE = 5; //SparkMax, NEO
     public static final int MOTOR_FEEDER_TOP_ONE = 7; //SparkMax, NEO
     public static final int MOTOR_FEEDER_BOTTOM_ONE = 7; //SparkMax, NEO
     public static final int MOTOR_CONVEYOR_BOTTOM_ONE = 9; //SparkMax, NEO550 
     public static final int MOTOR_CONVEYOR_TOP_ONE = 10; //SparkMax, NEO550
     public static final int MOTOR_TURRET_AZIMUTH_ONE = 11; //SparkMax, NEO550
     public static final int MOTOR_BOOSTER_ONE = 12; //SparkMax, NEO
     public static final int MOTOR_HOOD_ONE = 13; //SparkMax, NEO550
     public static final int MOTOR_SHOOTER_WHEEL_ONE = 14; // SparkMax, NEO
     public static final int MOTOR_SHOOTER_WHEEL_TWO = 15; // SparkMax, NEO
     public static final int MOTOR_CLIMBER_ONE = 16; //SparkMax, NEO
     public static final int MOTOR_CLIMBER_TWO = 17; //SparkMax, NEO

    //PH Ports
    public static final int CYLINDER_FEEDER_GATE_TOP_ONE = 9; //FIXME
    public static final int CYLINDER_FEEDER_GATE_BOTTOM_ONE = 9; //FIXME
    public static final int CYLINDER_INTAKE_PIVOT_ONE = 8;

    public static final double MAXIMUM_VOLTAGE = 12.0;


    // Values from LHSPanthers
    public static final double DRIVE_PID0_P = 5e-5;
    public static final double DRIVE_PID0_I = 1e-6;
    public static final double DRIVE_PID0_D = 0;
    public static final double DRIVE_PID0_F = 0.00156;

    public static final double ROTATION_PID0_P = 1.0;
    public static final double ROTATION_PID0_I = 0.0;
    public static final double ROTATION_PID0_D = 0.1;
    public static final double ROTATION_PID0_F = 0.00156;

    public static final double ROTATION_PID0_MAX_ACCEL = 5700; // RPM
    public static final double ROTATION_PID0_MAX_VELOCITY = 3000;

    public static final double PID_IZ_VALUE = 0;

    public static final double OUTPUT_RANGE_MIN = -1;
    public static final double OUTPUT_RANGE_MAX = 1;

    public static final double DRIVE_KS = 0.0978;
    public static final double DRIVE_KV = 3.16;

    public static final double ROTATION_KS = 0.0978;
    public static final double ROTATION_KV = 3.16;

    public static final boolean DIRVE_MOTOR_INVERSION = true;
    public static final boolean ROTATION_MOTOR_INVERSION = true;

    public static final int SECONDS_PER_MINUTE = 60;

    //Gear Ratios
    public static final double MK4_L2_DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double MK4_L2_ROTATION_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
    public static final double MK4_L2_WHEEL_DIAMETER = 0.10033; // Meters

    public static final double GEAR_RATIO_SERIALIZER_LEFT = 5.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_SERIALIZER_RIGHT = 5.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_INTAKE_ROLLERS= 2.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_CONVEYOR_BOTTOM= 4.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_CONVEYOR_TOP= 4.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_CLIMBER = 320.0/1.0; //Overdriven, 2020 bot
    public static final double GEAR_RATIO_SHOOTER = 4.0 / 3.0;
    public static final double GEAR_RATIO_BOOSTER = 2.0/1.0;
    public static final double GEAR_RATIO_TURRET = 96.5;// 12*138/15;
    public static final double GEAR_RATIO_HOOD =  20*450/24;

    // Current Limits
    public static final int CURRENT_LIMIT_DRIVE_MOTOR = 80;
    public static final int CURRENT_LIMIT_ROTATION_MOTOR = 20;
    public static final int CURRENT_LIMIT_SHOOTER_WHEEL = 60;
    public static final int CURRENT_LIMIT_BOOSTER = 20;
    public static final int CURRENT_LIMIT_TURRET_AZIMUTH = 20;
    public static final int CURRENT_LIMIT_HOOD = 20;
    public static final int CURRENT_LIMIT_FEEDER_TOP = 40;
    public static final int CURRENT_LIMIT_FEEDER_BOTTOM = 40;
    public static final int CURRENT_LIMIT_CLIMBER = 80;
    public static final int CURRENT_LIMIT_INTAKE_ROLLERS = 60;
    public static final int CURRENT_LIMIT_CONVEYOR_BOTTOM = 20;
    public static final int CURRENT_LIMIT_CONVEYOR_TOP = 20;

    
    public static final int MIN_PRESSURE = 90;
    public static final int MAX_PRESSURE = 110;

    public static final boolean INTAKE_EXTEND = true;
    public static final boolean INTAKE_RETRACT = false;

}
