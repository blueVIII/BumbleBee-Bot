package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Square Auto", group = "BlueViii-auto")
public class DriveSquare extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);

        Pose2d initPos = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPos);

        while (!isStarted() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            TrajectoryActionBuilder trajBuilder = drive.actionBuilder(initPos)
                    //.waitSeconds(5)
                    //.lineToX(12);


                    .strafeTo(new Vector2d(34, -40)) //14.5 in y, 49.5 in x
                    .waitSeconds(5)
                    .strafeTo(new Vector2d(28, -40))
                    .strafeTo(new Vector2d(30, 19))
                    .waitSeconds(5);


            Actions.runBlocking(trajBuilder.build());
        }
    }
}
