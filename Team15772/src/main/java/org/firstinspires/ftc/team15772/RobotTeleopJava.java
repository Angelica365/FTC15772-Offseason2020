package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "RobotTeleopJava", group = "")
//@Disabled
public class RobotTeleopJava extends LinearOpMode {

    // Define class members
    private Servo GripperLft;
    private Servo GripperRgt;
    private Servo Antenna_Servo;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor Antenna;

    @Override
    public void runOpMode() {
        float MovementX;
        float MovementY;
        float MovementTurn;

        GripperLft = hardwareMap.servo.get("GripperLft");
        GripperRgt = hardwareMap.servo.get("GripperRgt");
        Antenna_Servo = hardwareMap.servo.get("Antenna");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        FL = hardwareMap.dcMotor.get("FL");
        Antenna = hardwareMap.dcMotor.get("Antenna");

        GripperLft.setPosition(0.1);
        GripperRgt.setPosition(0.9);
        Antenna_Servo.setPosition(0.3);

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the start button
        waitForStart();

        while(opModeIsActive()){
        // Change angle of antenna
            if (gamepad1.x) {
                sleep(150);
                Antenna_Servo.setPosition(Antenna_Servo.getPosition() - 0.035);
            }
            else if (gamepad1.y) {
                sleep(150);
                Antenna_Servo.setPosition(Antenna_Servo.getPosition() + 0.035);
            }
        // Extend or retract antenna
            if (gamepad1.a) {
                Antenna.setPower(1.0);
            }
            else if (gamepad1.b) {
                Antenna.setPower(-1.0);
            }
            else {
                Antenna.setPower(0.0);
            }
        // Grip or ungrip stone
            if (gamepad1.right_bumper) {
                GripperLft.setPosition(0.49);
                GripperRgt.setPosition(0.51);
            }
            else if (gamepad1.left_bumper) {
                GripperLft.setPosition(0.35);
                GripperRgt.setPosition(0.65);
            }
        // Drive the mecanum chassis
            MovementX = gamepad1.left_stick_x;
            MovementY = gamepad1.left_stick_y;
            MovementTurn = gamepad1.right_stick_x;
            BR.setPower(-MovementY + -MovementTurn + MovementX);
            BL.setPower(-MovementY + (MovementTurn - MovementX));
            FR.setPower(-MovementY + (-MovementTurn - MovementX));
            FL.setPower(-MovementY + MovementTurn + MovementX);
            telemetry.update();
        }
    }
}