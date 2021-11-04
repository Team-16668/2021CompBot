package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonSettings {

    Gamepad gamepad;
    parkTypes parkType = parkTypes.NORMAL;
    Telemetry telemetry;
    double minimumParkDelay;
    double maximumParkDelay;

    double chosenParkDelay;

    public AutonSettings(Gamepad gamepad, Telemetry telemetry, int minumumParkDelay, int maximumParkDelay) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.minimumParkDelay = minumumParkDelay;
        this.maximumParkDelay = maximumParkDelay;
        chosenParkDelay = minimumParkDelay;
    }

    public void chooseSettings() throws InterruptedException {
        boolean currentUp = false,  prevUp = false, currentDown = false, prevDown = false, currentB = false, prevB = false;

        while(!gamepad.a) {
            currentUp = gamepad.dpad_up;
            currentDown = gamepad.dpad_down;
            currentB = gamepad.b;

            if(currentUp && currentUp != prevUp) {
                if(chosenParkDelay++ <= maximumParkDelay) {
                    chosenParkDelay++;
                }
            }else if (currentDown && currentDown != prevDown) {
                if(chosenParkDelay-- >= minimumParkDelay) {
                    chosenParkDelay--;
                }
            }

            if(currentB && currentB != prevB) {
                if(parkType == parkTypes.NORMAL) {
                    parkType = parkTypes.OFFSET;
                } else if(parkType == parkTypes.OFFSET) {
                    parkType = parkTypes.NORMAL;
                }
            }

            telemetry.addData("Current Settings", "");
            telemetry.addData("Park Delay", chosenParkDelay);
            telemetry.addData("Park Type", parkType);
            telemetry.update();

            prevUp = currentUp;
            prevDown = currentDown;
            prevB = currentB;
            Thread.sleep(20);
        }

        telemetry.addData("SETTINGS SELECTED", "");
        telemetry.addData("Park Delay", chosenParkDelay);
        telemetry.addData("Park Type", parkType);
        telemetry.update();
    }

    public enum parkTypes {
        NORMAL, OFFSET
    }

    public parkTypes getParkType() {
        return parkType;
    }

    public void setParkType(parkTypes parkType) {
        this.parkType = parkType;
    }

    public double getChosenParkDelay() {
        return chosenParkDelay*1000;
    }

    public void setChosenParkDelay(double chosenParkDelay) {
        this.chosenParkDelay = chosenParkDelay / 1000.0;
    }
}
