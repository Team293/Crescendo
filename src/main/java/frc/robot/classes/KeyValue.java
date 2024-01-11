package frc.robot.classes;

public class KeyValue<T> {
  public String k;
  public T v;
  public int x;
  public int y;

  public KeyValue(String k, T v) {
    this.k = k;
    this.v = v;
    this.x = -1;
    this.y = -1;
  }

  public KeyValue(String k, T v, int x, int y) {
    this.k = k;
    this.v = v;
    this.x = x;
    this.y = y;
  }
}
