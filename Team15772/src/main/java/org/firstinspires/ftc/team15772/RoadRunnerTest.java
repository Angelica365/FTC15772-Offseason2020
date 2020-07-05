package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.NewStoneDetector;
import org.firstinspires.ftc.teamcode.drive.mecanum.VertexDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareDrivetrain;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import java.io.*;
import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.opmodes.OldLiftConstants.k_G;
import static org.firstinspires.ftc.teamcode.opmodes.OldLiftConstants.k_d;
import static org.firstinspires.ftc.teamcode.opmodes.OldLiftConstants.k_i;
import static org.firstinspires.ftc.teamcode.opmodes.OldLiftConstants.k_p;

@Autonomous(name = "Road Runner Test")
public class RoadRunnerTest extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = (8192/5.93687);

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "FR", rbName = "BR", lfName = "FL", lbName = "BL";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = rbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        }

        // reset lift encoders and servos (enter them here)

        ElapsedTime timer = new ElapsedTime();
        HashMap<State, Double> stateTimes = new HashMap<>();  // map of state -> start time

        Trajectory trajectory = null;

        trajectory = drive.trajectoryBuilder()
            .addMarker(() -> {
                stateTimes.put(State.RESET_SERVOS, null);
                return null;
            })
            .lineTo(new Vector2d(-2, -24), new SplineInterpolator(0, Math.toRadians(-45)))
            .lineTo(new Vector2d(12, -32), new ConstantInterpolator(Math.toRadians(-45)))
            .lineTo(new Vector2d(16, -35), new ConstantInterpolator(Math.toRadians(-45)))
            .lineTo(new Vector2d(25, -23), new LinearInterpolator(Math.toRadians(-45), Math.toRadians(-135)))
            .addMarker(() -> {
                stateTimes.put(State.INTAKE_OUT_AND_IN, null);
                stateTimes.put(State.DROP_GRIPPERS_HALFWAY, null);
                return null;
            })
            .lineTo(new Vector2d(70, -23), new ConstantInterpolator(Math.toRadians(-180)))
            .addMarker(() -> {
                stateTimes.put(State.STOP_INTAKE, null);
                return null;
            })
            .lineTo(new Vector2d(87, -32), new ConstantInterpolator(Math.toRadians(90)))
            .addMarker(new Vector2d(87, -34), () -> {
                stateTimes.put(State.DROP_GRIPPERS_FULLY, null);
                return null; // go to stack pos
            })
            .lineTo(new Vector2d(87, -36), new ConstantInterpolator(Math.toRadians(90)))
            .build();

        drive.followTrajectory(trajectory);

        while (opModeIsActive()) {
            drive.update();

            double currentTime = timer.milliseconds();

            // set all start times if they don't exist
            for (State key : stateTimes.keySet()) {
                if (stateTimes.get(key) == null) {
                    stateTimes.put(key, currentTime);
                }
            }
        }
            telemetry.addData("stateTimes", stateTimes);
            telemetry.update();
        }
    }
}