package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TestOp")
public class TestOpMode extends LinearOpMode{

    @Override
    public void runOpMode() {

        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Lift lift = new Lift(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
//        drivetrain.setUseOdo(true);
//        Detector detector = new Detector;

        ElapsedTime elapsedTime = new ElapsedTime();

        telemetry.addLine("Initialized");
        telemetry.update();

        double lastTime = 0;

        drivetrain.position[2] = 180;
        drivetrain.rPosition = 180;

        drivetrain.setPosition[0] = Double.NaN;
        drivetrain.setPosition[1] = Double.NaN;
        drivetrain.setPosition[2] = Double.NaN;

        waitForStart();

        if (!isStopRequested()) {
            while(opModeIsActive()) {

//
//                if(gamepad1.a) {
//                    lift.set(Lift.state.HIGH);
//                }
//
//                if(gamepad1.x) {
//                    intake.start();
//                }
//
//                if(intake.isBlockDetected()) {
//                    lift.blockDetected();
//                }
//
//                if(gamepad1.a) {
//                    drivetrain.setPosition[0] = 0;
//                    drivetrain.setPosition[1] = 0;
//                }
//
//                if(gamepad1.b) {
//                    drivetrain.setPosition[0] = Double.NaN;
//                    drivetrain.setPosition[1] = Double.NaN;
//                    drivetrain.setPosition[2] = Double.NaN;
//                }

                if(gamepad1.x) {
                    lift.drop();
                }

                if(gamepad1.y) {
//                    lift.raise();
                }

                if(gamepad1.right_bumper) {
                    lift.extend();
                }

                if(gamepad1.left_bumper) {
                    lift.retract();
                }

                if(gamepad1.a) {
                    lift.dump();
                }
                else {
                    lift.unDump();
                }

                drivetrain.setVelocity[0] = - gamepad1.left_stick_x;
                drivetrain.setVelocity[1] = - gamepad1.left_stick_y;
                drivetrain.setVelocity[2] = gamepad1.right_stick_x;

                drivetrain.update();

                telemetry.addData("Angle", drivetrain.position[2]);
                telemetry.addData("Y", drivetrain.position[1]);
                telemetry.addData("X", drivetrain.position[0]);
                telemetry.addData("cycle:", elapsedTime.milliseconds() - lastTime);
                telemetry.update();

                lastTime = elapsedTime.milliseconds();
//                lift.update();
//                intake.update();

//                if(drivetrain.checkPosition() && lift.checkState()) {



//                }

            }

        }

    }

}
