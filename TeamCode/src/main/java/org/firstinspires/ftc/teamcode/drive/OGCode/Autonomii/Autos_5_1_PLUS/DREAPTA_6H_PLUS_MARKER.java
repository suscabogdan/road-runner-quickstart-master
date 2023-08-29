package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii.Autos_5_1_PLUS;

import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.PICK_UP_CONE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.OGCode.ClawController;
import org.firstinspires.ftc.teamcode.drive.OGCode.FourBarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.GhidajController;
import org.firstinspires.ftc.teamcode.drive.OGCode.LiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.OGCode.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;


@Config
@Autonomous(group = "b")

public class DREAPTA_6H_PLUS_MARKER extends LinearOpMode {
    enum STROBOT
    {
        START,
        PLACE_PRELOAD,
        PICK_UP_CONE,
        GO_TO_STACK,
        COLLECT,
        GO_PLACE_FROM_STACK,
        STOP_JOC,
        PLACE_STACK_CONE,
        PARK
    }

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166; //m

    // Tag IDs of sleeve
    int left = 5;
    int middle = 8;
    int right = 19;

    public static double x_PLACE_PRELOAD = -30, y_PLACE_PRELOAD = -5, backPreload = 45;
    public static double x_GTS_FIRST_LT2 = -65, y_GTS_FIRST_LT2 = -11;
    public static double x_PLACE_FIRST_STS = -30, y_PLACE_FIRST_STS = -5;

    int junctionHeight = 0;
    ElapsedTime TIMERGLOBAL = new ElapsedTime(), timerRetract = new ElapsedTime(), timerLift =new ElapsedTime() , timeCollect = new ElapsedTime();

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double currentVoltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = batteryVoltageSensor.getVoltage();
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        ClawController clawController = new ClawController();
        FourBarController fourBarController = new FourBarController();
        LiftController liftController = new LiftController();
        GhidajController ghidajController = new GhidajController();
        RobotController robotController = new RobotController();
        fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
        clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
        liftController.CurrentStatus = LiftController.liftStatus.GROUND;
        ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
        robot.servoGheara.setPosition(0.5);
        int nr = 0, NRCON = 6, CAZ = 1;
        int trajCount = 0;
        ElapsedTime timePLACE_PRELOAD = new ElapsedTime();
        Pose2d startPose = new Pose2d(35, -63, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        AtomicReference<DREAPTA_6H_PLUS_MARKER.STROBOT> status = new AtomicReference<>(DREAPTA_6H_PLUS_MARKER.STROBOT.START);

        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .setConstraints(drive.getVelocityConstraint(40, 10, 11.19), drive.getAccelerationConstraint(40))
                .back(10)
                .splineTo(new Vector2d(15, -35), Math.toRadians(140))
                .back(12)
                .addSpatialMarker(new Vector2d(27, -42), () -> {
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    junctionHeight = 0;
                    robotController.CurrentStatus = PICK_UP_CONE;
                })
                .addSpatialMarker(new Vector2d(10, -30), () -> {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.PLACE;
                    clawController.CurrentStatus = ClawController.closeClawStatus.OPEN;
                })
                .resetConstraints()
                .build();

        TrajectorySequence GTS_FIRST = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
//                .setConstraints(drive.getVelocityConstraint(35, 10, 11.19), drive.getAccelerationConstraint(35))
                .lineToLinearHeading(new Pose2d(12, -13, Math.toRadians(0)))
                .forward(1)
                .splineTo(new Vector2d(40, -13.5), Math.toRadians(0))
                .splineTo(new Vector2d(66, -12), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(15, -12), () -> {
                    junctionHeight = 8;
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    clawController.CurrentStatus = ClawController.closeClawStatus.OPEN;
                })
                .addSpatialMarker(new Vector2d(65, -12), () -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                })
                .build();

        TrajectorySequence PLACE_FIRST = drive.trajectorySequenceBuilder(GTS_FIRST.end())
//                .setConstraints(drive.getVelocityConstraint(35, 10, 11.19), drive.getAccelerationConstraint(35))
                .addSpatialMarker(new Vector2d(66, -12), () -> {
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    junctionHeight = 0;
                })
                .back(1)
                .splineTo(new Vector2d(30, -12), Math.toRadians(180))
                .splineTo(new Vector2d(5, -20), Math.toRadians(220))
                .addSpatialMarker(new Vector2d(50, -15), () -> {
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.INTER_POSITION;
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.INTER_PICK_UP_STACK;
                })
                .addSpatialMarker(new Vector2d(10, -12), () -> {
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.PLACE_POSITION;
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.OUTTAKE;
                    clawController.CurrentStatus = ClawController.closeClawStatus.OPEN;
                })
                .addSpatialMarker(new Vector2d(20, -14), () -> {
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.OUTTAKE;
                })
                .build();

        TrajectorySequence GTS_SECOND = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
//                .setConstraints(drive.getVelocityConstraint(35, 10, 11.19), drive.getAccelerationConstraint(35))
                .forward(1)
                .addDisplacementMarker(() -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
                })
                .splineTo(new Vector2d(30, -14), Math.toRadians(350))
                .splineTo(new Vector2d(65, -12), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(15, -12), () -> {
                    junctionHeight = 7;
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    clawController.CurrentStatus = ClawController.closeClawStatus.OPEN;
                })
                .addSpatialMarker(new Vector2d(63, -12), () -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                })
                .build();

        TrajectorySequence GTS_THIRD = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
//                .setConstraints(drive.getVelocityConstraint(35, 10, 11.19), drive.getAccelerationConstraint(35))
                .forward(1)
                .addDisplacementMarker(() -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
                })
                .splineTo(new Vector2d(30, -14), Math.toRadians(350))
                .splineTo(new Vector2d(65, -12), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(15, -12), () -> {
                    junctionHeight = 6;
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    clawController.CurrentStatus = ClawController.closeClawStatus.OPEN;
                })
                .addSpatialMarker(new Vector2d(63, -12), () -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                })
                .build();

        TrajectorySequence GTS_FOURTH = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
//                .setConstraints(drive.getVelocityConstraint(35, 10, 11.19), drive.getAccelerationConstraint(35))
                .forward(1)
                .addDisplacementMarker(() -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
                })
                .splineTo(new Vector2d(30, -14), Math.toRadians(350))
                .splineTo(new Vector2d(65, -12), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(15, -12), () -> {
                    junctionHeight = 5;
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    clawController.CurrentStatus = ClawController.closeClawStatus.OPEN;
                })
                .addSpatialMarker(new Vector2d(63, -12), () -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                })
                .build();

        TrajectorySequence GTS_FIFTH = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
//                .setConstraints(drive.getVelocityConstraint(60, 10, 11.19), drive.getAccelerationConstraint(60))
                .forward(1)
                .addDisplacementMarker(() -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
                })
                .splineTo(new Vector2d(30, -14), Math.toRadians(350))
                .splineTo(new Vector2d(65, -12), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(15, -12), () -> {
                    junctionHeight = 4;
                    liftController.CurrentStatus = LiftController.liftStatus.POLE;
                    clawController.CurrentStatus = ClawController.closeClawStatus.OPEN;
                })
                .addSpatialMarker(new Vector2d(63, -12), () -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                })
                .build();

        TrajectorySequence PARK_3 = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
//                .waitSeconds(1)
                .setConstraints(drive.getVelocityConstraint(60, 10, 11.19), drive.getAccelerationConstraint(60))
                .addDisplacementMarker(() -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
                    junctionHeight = 0;
                    liftController.CurrentStatus = LiftController.liftStatus.GROUND;
                })
                .forward(1)
                .splineTo(new Vector2d(35, -15), Math.toRadians(0))
                .splineTo(new Vector2d(58, -14), Math.toRadians(0))
                .build();
        TrajectorySequence PARK_2 = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
//                .waitSeconds(1)
                .setConstraints(drive.getVelocityConstraint(60, 10, 11.19), drive.getAccelerationConstraint(60))
                .addDisplacementMarker(() -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
                    junctionHeight = 0;
                    liftController.CurrentStatus = LiftController.liftStatus.GROUND;
                })
                .splineTo(new Vector2d(35, -12), Math.toRadians(0))
                .build();
        TrajectorySequence PARK_1 = drive.trajectorySequenceBuilder(PLACE_FIRST.end())
//                .waitSeconds(1)
                .setConstraints(drive.getVelocityConstraint(60, 10, 11.19), drive.getAccelerationConstraint(60))
                .addSpatialMarker(new Vector2d(25, -12), () -> {
                    clawController.CurrentStatus = ClawController.closeClawStatus.CLOSED;
                    fourBarController.CurrentStatus = FourBarController.fourBarStatus.COLLECT_DRIVE;
                    ghidajController.CurrentStatus = GhidajController.ghidajStatus.INTAKE;
                    junctionHeight = 0;
                    liftController.CurrentStatus = LiftController.liftStatus.GROUND;
                })
                .lineToLinearHeading(new Pose2d(12, -13, Math.toRadians(0)))
                .build();
        while (!isStarted()&&!isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    if(tagOfInterest.id == left)
                    {
                        telemetry.addLine("Sleeve detected! Case: 1");
                    }
                    if(tagOfInterest.id == middle)
                    {
                        telemetry.addLine("Sleeve detected! Case: 2");
                    }
                    if(tagOfInterest.id == right)
                    {
                        telemetry.addLine("Sleeve detected! Case: 3");
                    }
                }
                else
                {
                    telemetry.addLine("Sleeve not in sight!");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("Sleeve never detected. Default case is: 2");
                    }
                    else
                    {
                        if(tagOfInterest.id == left)
                        {
                            telemetry.addLine("Last detection case was: 1");
                        }
                        if(tagOfInterest.id == middle)
                        {
                            telemetry.addLine("Last detection case was: 2");
                        }
                        if(tagOfInterest.id == right)
                        {
                            telemetry.addLine("Last detection case was: 3");
                        }
                    }
                }

            }
            else
            {
                telemetry.addLine("Sleeve not in sight!");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("Sleeve never detected. Default case is: 2");
                }
                else
                {
                    if(tagOfInterest.id == left)
                    {
                        telemetry.addLine("Last detection case was: 1");
                    }
                    if(tagOfInterest.id == middle)
                    {
                        telemetry.addLine("Last detection case was: 2");
                    }
                    if(tagOfInterest.id == right)
                    {
                        telemetry.addLine("Last detection case was: 3");
                    }
                }

            }

            telemetry.addLine("Init Complete");
            telemetry.update();
            sleep(50);
        }
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {

            if (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.START && !drive.isBusy() && trajCount == 0)
            {
                trajCount++;
                drive.followTrajectorySequenceAsync(PLACE_PRELOAD);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.PLACE_PRELOAD);
            }

            if(((status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.PLACE_PRELOAD) || (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.GO_PLACE_FROM_STACK)) && !drive.isBusy() && trajCount == 1) {
                trajCount++;
                drive.followTrajectorySequenceAsync(GTS_FIRST);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.COLLECT);
            }

            if (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.COLLECT && !drive.isBusy() && trajCount == 2)
            {
                trajCount++;
                drive.followTrajectorySequenceAsync(PLACE_FIRST);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.GO_PLACE_FROM_STACK);
            }

            if(((status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.PLACE_PRELOAD) || (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.GO_PLACE_FROM_STACK)) && !drive.isBusy() && trajCount == 3) {
                trajCount++;
                drive.followTrajectorySequenceAsync(GTS_SECOND);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.COLLECT);
            }

            if (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.COLLECT && !drive.isBusy() && trajCount == 4)
            {
                trajCount++;
                drive.followTrajectorySequenceAsync(PLACE_FIRST);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.GO_PLACE_FROM_STACK);
            }

            if(((status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.PLACE_PRELOAD) || (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.GO_PLACE_FROM_STACK)) && !drive.isBusy() && trajCount == 5) {
                trajCount++;
                drive.followTrajectorySequenceAsync(GTS_THIRD);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.COLLECT);
            }

            if (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.COLLECT && !drive.isBusy() && trajCount == 6)
            {
                trajCount++;
                drive.followTrajectorySequenceAsync(PLACE_FIRST);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.GO_PLACE_FROM_STACK);
            }

            if(((status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.PLACE_PRELOAD) || (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.GO_PLACE_FROM_STACK)) && !drive.isBusy() && trajCount == 7) {
                trajCount++;
                drive.followTrajectorySequenceAsync(GTS_FOURTH);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.COLLECT);
            }

            if (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.COLLECT && !drive.isBusy() && trajCount == 8)
            {
                trajCount++;
                drive.followTrajectorySequenceAsync(PLACE_FIRST);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.GO_PLACE_FROM_STACK);
            }

            if(((status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.PLACE_PRELOAD) || (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.GO_PLACE_FROM_STACK)) && !drive.isBusy() && trajCount == 9) {
                trajCount++;
                drive.followTrajectorySequenceAsync(GTS_FIFTH);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.COLLECT);
            }

            if (status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.COLLECT && !drive.isBusy() && trajCount == 10)
            {
                trajCount++;
                drive.followTrajectorySequenceAsync(PLACE_FIRST);
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.PARK);
            }

            if(status.get() == DREAPTA_6H_PLUS_MARKER.STROBOT.PARK && !drive.isBusy() && trajCount == 11) {
                trajCount++;
                if(tagOfInterest == null)
                {
                    CAZ = 2;
                } else {
                    if(tagOfInterest.id == left)
                    {
                        CAZ = 1;
                    } else
                    if(tagOfInterest.id == middle)
                    {
                        CAZ = 2;
                    }
                    if(tagOfInterest.id == right)
                    {
                        CAZ = 3;
                    }
                }
                if (CAZ == 1)
                {
                    drive.followTrajectorySequenceAsync(PARK_1);
                }
                else
                if (CAZ == 2)
                {
                    drive.followTrajectorySequenceAsync(PARK_2);
                }
                else
                if (CAZ == 3)
                {
                    drive.followTrajectorySequenceAsync(PARK_3);
                }
                status.set(DREAPTA_6H_PLUS_MARKER.STROBOT.STOP_JOC);
            }

            drive.update();

            int fourBarPosition = robot.motor4Bar.getCurrentPosition();
            int liftPosition = robot.extensieOuttake.getCurrentPosition();
            fourBarController.update(robot , fourBarPosition,14);
            clawController.update(robot);
            liftController.update(robot,junctionHeight, liftPosition, currentVoltage);
            ghidajController.update(robot);
            robotController.update(robot, fourBarController, ghidajController, liftController, clawController, junctionHeight, fourBarPosition);
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
            telemetry.addData("NRCON: ", NRCON);
            telemetry.addData("Stack Height: ", junctionHeight);
            telemetry.addData("TRAJ COUNT: ", trajCount);
            telemetry.addData("Status",status);
            telemetry.update();
        }
    }

}