package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.openCV.SkystoneDeterminationExample;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Vector;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous()
public class DuckAuto extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException
    {
        double retractTimer;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-42.5,-64,0));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
        phoneCam.showFpsMeterOnViewport(false);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        robot.encoderservo.setPosition(0.25);

        //start to carousel
        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(new Pose2d(-42.5,-64,0))
                .lineTo(new Vector2d(-42.5, -48))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-61.5, -48))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-61.5, -54))
                .build();

        //carousel to hub
        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(seq1.end())
                .lineTo(new Vector2d(-55, -48))
                .turn(Math.toRadians(-90))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-68, -48))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-68, -36))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-54, -36))
                .build();

        //hub to parking
        TrajectorySequence seq3 = drive.trajectorySequenceBuilder(seq2.end())
                .lineTo(new Vector2d(-72, -36))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-72, -50))
                .build();



        int level = pipeline.getAnalysis().ordinal() + 1;

        while (!isStarted())
        {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            level = pipeline.getAnalysis().ordinal() + 1;
        }

        phoneCam.stopStreaming();

        if (opModeIsActive())
        {

            robot.setLevel(level);

            telemetry.addData("Location", level);

            robot.extendState = Robot.ExtendState.EXTEND;

            //intake down
            robot.setIntakeBucketState(Robot.IntakeBucket.RIGHT);

            //drive to carousel
            drive.followTrajectorySequence(seq1);

            drive.update();
            robot.setCarSpeed(-1);
            sleep(2000);
            robot.setCarSpeed(0);

            //Turn so back is pressed against wall + drive to hub
            drive.followTrajectorySequence(seq2);

            //extend arm + deliver freight
            while (opModeIsActive() && robot.extendState != Robot.ExtendState.RESET)
            {
                robot.updateExtend();
                robot.updateLiftServo();
                robot.updateIntakeBucket();
            }


            //park + retract ar
            drive.followTrajectorySequenceAsync(seq3);

            retractTimer = getRuntime();
            while (opModeIsActive() && (getRuntime()-retractTimer < 5))
            {
                drive.update();
                robot.updateExtend();
                robot.updateLiftServo();
            }
        }
    }
}
