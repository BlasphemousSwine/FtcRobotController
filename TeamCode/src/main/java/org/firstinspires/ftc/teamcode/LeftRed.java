package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "LeftRed", preselectTeleOp = "IdahoTeleOp")
public class LeftRed extends LinearOpMode {

    private enum state {
        START, PARK, EXTEND, WAREHOUSE, RETRACT, EXIT
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        drivetrain.useOdo = false;

        drivetrain.position[0] = -0.4;
        drivetrain.position[1] = -2.5;

        state state = LeftRed.state.START;
        Detector detector = new Detector(hardwareMap, 20, 200);
        Detector.ElementPosition elementPosition = detector.getElementPosition();

        ElapsedTime elapsedTime = new ElapsedTime();

        lift.retract();
        lift.raise();

        int i = 0;

        waitForStart();

        double startTime = elapsedTime.milliseconds();

        while(opModeIsActive()) {

            switch(state) {

                case START:

                    drivetrain.setPosition[2] = -40;
                    drivetrain.setPosition[1] = -0.3;
                    lift.drop();
                    if (i >= 100) {
                        i = 0;
                        state = LeftRed.state.EXTEND;
                    } else {
                        i++;
                    }

                    break;

                case EXTEND:

                    lift.extend();
                    if (i >= 50) {
                        i = 0;
                        state = LeftRed.state.RETRACT;
                    } else {
                        i++;
                    }

                    break;

                case RETRACT:

                    lift.retract();
                    drivetrain.setPosition[1] = -0.35;
                    drivetrain.setPosition[2] = -90;
                    drivetrain.setPosition[0] = 2;

                    if (i >= 100) {
                        i = 0;
                        state = LeftRed.state.WAREHOUSE;
                    }
                    else {
                        i++;
                    }

                    break;

                case WAREHOUSE:

                    lift.intake();
                    drivetrain.setPosition[1] = -1;
                    drivetrain.setVelocity[1] = 0.3;

                    if (i >= 100) {
                        i = 0;
                        state = LeftRed.state.EXIT;
                    }
                    else {
                        i++;
                    }

                    break;

                case EXIT:

                    lift.drop();

                    drivetrain.setPosition[1] = -0.35;
                    drivetrain.setPosition[2] = -90;
                    drivetrain.setPosition[0] = 2;

                    drivetrain.setVelocity[1] = 0;

                    if (i >= 20) {
                        i = 0;
                        state = LeftRed.state.EXTEND;
                    }
                    else {
                        i++;
                    }

                    break;

                case PARK:

                    lift.retract();
                    drivetrain.setPosition[1] = -1;
                    drivetrain.setPosition[0] = 2;
                    drivetrain.setPosition[2] = -90;

                    if (i >= 50) {
                        lift.raise();
                    }
                    else {
                        i++;
                    }

                    break;

            }

            if (elapsedTime.milliseconds() - startTime >= 27000) {
                state = LeftRed.state.PARK;
            }

            drivetrain.update();

            telemetry.addData("State:", drivetrain.checkPosition());
            telemetry.addData("Angle", drivetrain.position[2]);
            telemetry.addData("Y", drivetrain.position[1]);
            telemetry.addData("X", drivetrain.position[0]);
            telemetry.update();

        }

    }

}
