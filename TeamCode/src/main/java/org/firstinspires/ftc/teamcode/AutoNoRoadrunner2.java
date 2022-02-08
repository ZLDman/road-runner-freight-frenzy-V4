package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PathFollower.ZachPathFollower;
import org.firstinspires.ftc.teamcode.PathFollower.ZachPathGenerator;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.openCV.SkystoneDeterminationExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Disabled()
public class AutoNoRoadrunner2 extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);
        ZachPathFollower pathFollower = new ZachPathFollower(drive,0.5,10);

        //if we should strafe towards the wall
        SampleMecanumDrive.wall = true;

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        long autoTime = System.currentTimeMillis();

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


        while (!isStarted())
        {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }



        if (opModeIsActive()) {

            autoTime = System.currentTimeMillis();

            int level = pipeline.getAnalysis().ordinal() + 1;

            phoneCam.closeCameraDevice();

            robot.setLevel(level);

            //put encoder servo down
            robot.encoderservo.setPosition(0.25);

            telemetry.addData("Location", level);

            robot.extendState = Robot.ExtendState.EXTEND;

            //intake down
            robot.setIntakeBucketState(Robot.IntakeBucket.RIGHT);

            //drive to hub
            pathFollower.FollowPathAsync(new ZachPathGenerator( -20));
            while (opModeIsActive() && robot.extendState != Robot.ExtendState.RESET && pathFollower.isBusy()) {
                drive.update();
                pathFollower.update();
                robot.updateExtend();
                robot.updateLiftServo();
                robot.updateIntakeBucket();
            }

            while(opModeIsActive()) {

                robot.setIntake1Speed(1);

                //drive into warehouse
                pathFollower.FollowPath(new ZachPathGenerator(38));
                while (drive.isBusy() && opModeIsActive()) {
                    drive.update();
                    pathFollower.update();
                    robot.updateExtend();
                    robot.updateLiftServo();
                    robot.updateIntakeBucket();
                    robot.setIntake1Speed(1);
                    if(robot.getColor(1) > 1){
                        pathFollower.cancleFollowing();
                    }
                }

                //intake
                while (robot.getColor(1) > 1 && opModeIsActive()) {
                    drive.setDrivePower(new Pose2d(0.3,0,0));
                    drive.update();
                    robot.updateExtend();
                    robot.updateLiftServo();
                    robot.updateIntakeBucket();
                    robot.setIntake1Speed(1);
                }

                drive.setDrivePower(new Pose2d(0,0,0));

                //turn off intake and raise intake bucket
                robot.setIntake1Speed(-1);
                sleep(100);
                robot.setIntake1Speed(0);
                robot.setIntakeBucketState(Robot.IntakeBucket.UP);

                //drive to hub
                pathFollower.FollowPath(new ZachPathGenerator(-12));
                while (pathFollower.isBusy() && opModeIsActive()) {
                    drive.update();
                    pathFollower.update();
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

                if(System.currentTimeMillis() - autoTime < 9000){
                    return;
                }
            }
        }
    }
}