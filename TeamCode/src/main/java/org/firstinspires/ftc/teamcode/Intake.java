package org.firstinspires.ftc.teamcode;

import com.google.blocks.ftcrobotcontroller.runtime.Block;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    final DcMotor IntakeMotor;
    final DistanceSensor BlockProximity;

    final Lift lift;

    public boolean isBlockDetected = false;

    private double i = 0;

    public Intake(HardwareMap hardwareMap) {

        IntakeMotor = hardwareMap.dcMotor.get("Intake");
        BlockProximity = hardwareMap.get(DistanceSensor.class, "BlockProximity");

        lift = new Lift(hardwareMap);

    }

    public void update() {

        if(!isBlockDetected) {

            IntakeMotor.setPower(1);

        }
        else if (i <= 100){

            IntakeMotor.setPower(-1);
            lift.drop();
            i++;

        }
        else {

            IntakeMotor.setPower(0);

        }

        checkForBlock();

    }

    public void start() {

        checkForBlock();
        update();

    }

    private void checkForBlock() {

        double distance = BlockProximity.getDistance(DistanceUnit.CM);
        isBlockDetected = (distance <= 2);

    }

}
