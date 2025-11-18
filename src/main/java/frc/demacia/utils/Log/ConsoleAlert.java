package frc.demacia.utils.Log;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;

public class ConsoleAlert extends Alert{
  Timer timer;

  public ConsoleAlert(String text, AlertType type) {
    super("Console", text, type);
    timer = new Timer();
  }

  @Override
  public void set(boolean active) {
    super.set(active);

    if (active) {
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }
  }

  @Override
  public void setText(String text) {
    super.setText(text);
    timer.reset();
  }

  public boolean isTimerOver() {
    return timer.hasElapsed(ConsoleConstants.CONSOLE_MESSEGE_TIME);
  }
}
