package frc.robot.other;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class AlertExample {
    //Alerts will show up in Shuffleboard, and are triggered by some boolean condition that is true
    Alert myAlert = new Alert("Some alert text.",AlertType.kWarning);

    //myAlert.set(someBooleanExpression)
    //You would typically put the alert.set(boolean) method in a periodic method if you wanted it checked regulary,
    //or in an init method if you wanted it checked at startup or the initializaiton of a command
    //or subsytem.
    
    //Examples - this is not a working examples
    Alert motorOverheatingAlert = new Alert("Motor is overheating",AlertType.kInfo);
    //myAlert.set(motor1.temperature > 200) - would fire the alert if motor temperature > 200, otherwise
    //does nothing.
}
