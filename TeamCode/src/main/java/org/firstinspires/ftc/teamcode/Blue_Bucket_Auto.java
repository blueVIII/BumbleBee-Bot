package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Blue_Bucket_Auto", group = "BlueViii-auto")
public class Blue_Bucket_Auto extends LinearOpMode {

    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;
    private Servo liftClaw = null;
    private Servo liftClawRotate_Claw = null;
    private Servo liftClawRotate_Arm = null;
    private Servo liftClawExtender = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);

        // Initialize hardware
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
        liftClaw = hardwareMap.servo.get("liftClaw");
        liftClawRotate_Claw = hardwareMap.servo.get("liftClawRotate_Claw");
        liftClawRotate_Arm = hardwareMap.servo.get("liftClawRotate_Arm");
        liftClawExtender = hardwareMap.servo.get("liftClawExtender");

        // Lift motor configuration
        liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Pose2d initPos = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPos);

        while (!isStarted() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        // Start autonomous sequence
        while (opModeIsActive()) {
            // ------------------------------- ALWAYS RESET ----------------
            //default pos is arm fully rotated back and retracted, claw closed. claw rotater - rotated backward (ie dpad down, pos 0)
            // ------------------------------- ALWAYS RESET ----------------

            //preset to be placed
            telemetry.addData("Lift1 Position", liftMotor1.getCurrentPosition());
            telemetry.addData("Lift2 Position", liftMotor2.getCurrentPosition());
            telemetry.update();
            /*
            liftClaw.setPosition(1);
            liftClawRotate_Arm.setPosition(1);
            sleep(5000);*/


            //raise to -3500
            liftMotor1.setTargetPosition(-4000);
            liftMotor2.setTargetPosition(-4000);

            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power to move the lifts to max height
            liftMotor1.setPower(-0.6);
            liftMotor2.setPower(-0.6);

            // Wait until both motors reach their target positions
            while (liftMotor1.isBusy() && liftMotor2.isBusy() && opModeIsActive()) {
                telemetry.addData("Lift1 Position", liftMotor1.getCurrentPosition());
                telemetry.addData("Lift2 Position", liftMotor2.getCurrentPosition());
                telemetry.update();
            }

            // Hold the lift position
            liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor1.setPower(-0.05);
            liftMotor2.setPower(-0.05);
            sleep(5000);
            //end raise action


            //move to like just in front
            telemetry.addData("Lift1 Position", liftMotor1.getCurrentPosition());
            telemetry.addData("Lift2 Position", liftMotor2.getCurrentPosition());
            telemetry.update();
            /*
            TrajectoryActionBuilder trajBuilder = drive.actionBuilder(initPos)
                    .strafeTo(new Vector2d(20, -43)); // Move to first position
            Actions.runBlocking(trajBuilder.build());
            sleep(5000);*/


            //fast down to -1500 - THIS IS SO WEIRD FEELING, try a diff way
            liftMotor1.setTargetPosition(-2750);
            liftMotor2.setTargetPosition(-2750);

            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power to move the lifts to max height
            liftMotor1.setPower(-0.8);
            liftMotor2.setPower(-0.8);

            // Wait until both motors reach their target positions
            while (liftMotor1.isBusy() && liftMotor2.isBusy() && opModeIsActive()) {
                telemetry.addData("Lift1 Position", liftMotor1.getCurrentPosition());
                telemetry.addData("Lift2 Position", liftMotor2.getCurrentPosition());
                telemetry.update();
            }

            // Hold the lift position
            liftClaw.setPosition(0);
            liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor1.setPower(-0.05);
            liftMotor2.setPower(-0.05);
            sleep(5000);
            //end raise action

            //reset...
            //for now just this
            liftClaw.setPosition(0);











            // Lift action: Raise the lift to 4500/*
            /*liftMotor1.setTargetPosition(-4500);
            liftMotor2.setTargetPosition(-4500);

            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power to move the lifts to max height
            liftMotor1.setPower(-0.6);
            liftMotor2.setPower(-0.6);

            // Wait until both motors reach their target positions
            while (liftMotor1.isBusy() && liftMotor2.isBusy() && opModeIsActive()) {
                telemetry.addData("Lift1 Position", liftMotor1.getCurrentPosition());
                telemetry.addData("Lift2 Position", liftMotor2.getCurrentPosition());
                telemetry.update();
            }
            //end raise action

            // Hold the lift position
            liftMotor1.setPower(-0.05);
            liftMotor2.setPower(-0.05);*/

            // Claw action: Close the claw
            /*liftClaw.setPosition(1);
            sleep(500); // Allow time for claw to close

            //trajBuilder.strafeTo(new Vector2d(0, 0)) // Move to second position, 28, -43
            //        .waitSeconds(2);

            // Arm action: Rotate the arm and extend
            liftClawRotate_Arm.setPosition(1); // Rotate arm to a position
            liftClawExtender.setPosition(1); // Extend the arm
            sleep(1000);

            //trajBuilder.strafeTo(new Vector2d(30, 19)) // Move to third position
            //        .waitSeconds(2);

            // Reset actions: Retract and reset positions
            liftClawExtender.setPosition(0);
            liftClawRotate_Arm.setPosition(0.3);
            liftClaw.setPosition(0);

            //Actions.runBlocking(trajBuilder.build());*/
        }
    }
}
