public class ElevatorSubsystem {
    
      public void holdElevatorAtPosition(){
        
        double currentPos = getPosition();
    
        //Set target to current position
        targetPosition = currentPos;
    
        //POSITION mode
        currentControlMode = ControlMode.POSITION;
    }
}
