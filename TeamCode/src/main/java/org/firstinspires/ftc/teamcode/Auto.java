package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.openCV.SkystoneDeterminationExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous()
public class Auto extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
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

        //start to hub
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-42.5,-64,0))
                .splineTo(new Vector2d(-12, -64), 0)
                .build();

        //hub to warehouse
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(42, -64), 0)
                .build();

        //warehouse to hub
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(),true)
                .splineTo(new Vector2d(-12, -64), 0)
                .build();



        int level = pipeline.getAnalysis().ordinal() + 1;

        while (!isStarted() && opModeIsActive())
        {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            level = pipeline.getAnalysis().ordinal() + 1;
        }

        phoneCam.stopStreaming();

        if (opModeIsActive()) {

            robot.setLevel(level);

            telemetry.addData("Location", level);

            robot.extendState = Robot.ExtendState.EXTEND;

            //intake down
            robot.setIntakeBucketState(Robot.IntakeBucket.RIGHT);

            //drive to hub
            drive.followTrajectoryAsync(traj1);
            while (opModeIsActive() && robot.extendState != Robot.ExtendState.RESET) {
                drive.update();
                robot.updateExtend();
                robot.updateLiftServo();
                robot.updateIntakeBucket();
            }

            while(opModeIsActive()) {

                //put encoder servo down
                robot.encoderservo.setPosition(0.25);

                robot.setIntake1Speed(1);

                //drive into warehouse
                drive.followTrajectoryAsync(traj2);
                while (robot.getColor(-1, 1) > 1 && opModeIsActive()) {
                    drive.update();
                    robot.updateExtend();
                    robot.updateLiftServo();
                    robot.updateIntakeBucket();robot.setIntake1Speed(1);
                }

                //turn off intake and raise intake bucket
                robot.setIntake1Speed(0);
                robot.setIntakeBucketState(Robot.IntakeBucket.UP);

                //drive to hub
                drive.followTrajectoryAsync(traj3);
                while (drive.isBusy() && opModeIsActive()) {
                    drive.update();
                    robot.updateExtend();
                    robot.updateLiftServo();
                    robot.updateIntakeBucket();
                    robot.setIntake1Speed(0);
                }

                //set target to high goal and extend
                robot.setLevel(3);
                robot.extendState = Robot.ExtendState.EXTEND;

                //intake down
                robot.setIntakeBucketState(Robot.IntakeBucket.RIGHT);

                //wait until we finished dumping
                while (robot.extendState != Robot.ExtendState.RESET && opModeIsActive()) {
                    robot.updateExtend();
                    robot.updateLiftServo();
                    robot.setIntake1Speed(0);
                }
            }
        }
    }
}