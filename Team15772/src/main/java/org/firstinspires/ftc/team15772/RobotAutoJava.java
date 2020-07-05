package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "RobotAutoJava", group = "")
//@Disabled
public class RobotAutoJava extends LinearOpMode {

    // Define class members
    private Servo GripperLft;
    private Servo GripperRgt;
    private Servo Antenna_Servo;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor Antenna;

    private ElapsedTime     runtime = new ElapsedTime();

//    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 0.78 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final int COUNTS_PER_INCH = 70;

    @Override
    public void runOpMode() {

        GripperLft = hardwareMap.servo.get("GripperLft");
        GripperRgt = hardwareMap.servo.get("GripperRgt");
        Antenna_Servo = hardwareMap.servo.get("Antenna");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        FL = hardwareMap.dcMotor.get("FL");
        Antenna = hardwareMap.dcMotor.get("Antenna");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GripperLft.setPosition(0.1);
        GripperRgt.setPosition(0.9);
        Antenna_Servo.setPosition(0.3);

        // Wait for the start button
        waitForStart();

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setTargetPosition(COUNTS_PER_INCH * 10);
        BR.setTargetPosition(COUNTS_PER_INCH * 10);
        FL.setTargetPosition(COUNTS_PER_INCH * 10);
        BL.setTargetPosition(COUNTS_PER_INCH * 10);

        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BR.setPower(-0.5);
        BL.setPower(0.5);
        FR.setPower(-0.5);
        FL.setPower(0.5);

        while (!!(FR.isBusy() && FL.isBusy())) {
            telemetry.update();
        }

        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);


    }
}