package com.lib.drivers;

import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SerialPortJNI;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.Timer;

public class PicoColorSensor implements AutoCloseable {

    public static class RawColor {
      public RawColor(int r, int g, int b, int _ir) {
        red = r;
        green = g;
        blue = b;
        ir = _ir;
      }
  
      public RawColor() {
        }
  
      public int red;
      public int green;
      public int blue;
      public int ir;

    }
  
    private static class SingleCharSequence implements CharSequence {
      public byte[] data;
  
      @Override
      public int length() {
        return data.length;
      }
  
      @Override
      public char charAt(int index) {
        return (char)data[index];
      }
  
      @Override
      public CharSequence subSequence(int start, int end) {
        return new String(data, start, end, StandardCharsets.UTF_8);
      }
  
    }
  
    private static class IntRef {
      int value;
    }
  
    int parseIntFromIndex(SingleCharSequence charSeq, int readLen, IntRef lastComma) {
      int nextComma = 0;
      try {
        nextComma = findNextComma(charSeq.data, readLen, lastComma.value);
        int value = Integer.parseInt(charSeq, lastComma.value + 1, nextComma, 10);
        lastComma.value = nextComma;
        return value;
      } catch (Exception ex) {
        return 0;
      }
    }
  
    private int findNextComma(byte[] data, int readLen, int lastComma) {
      while (true) {
        if (readLen <= lastComma + 1 ) {
          return readLen;
        }
        lastComma++;
        if (data[lastComma] == ',') {
          break;
        }
      }
      return lastComma;
    }
  
    private final AtomicBoolean debugPrints = new AtomicBoolean();
    private boolean hasColor0;
    private boolean hasColor1;
    public int prox0;
    public int prox1;
    public final RawColor color0 = new RawColor();
    public final RawColor color1 = new RawColor();
    private double lastReadTime;
    private final ReentrantLock threadLock = new ReentrantLock();
    private final Thread readThread;
    private final AtomicBoolean threadRunning = new AtomicBoolean(true);
  
    private void threadMain() {
      // Using JNI for a non allocating read
      int port = SerialPortJNI.serialInitializePort((byte)1); //TODO: change port based on what electrical says
      SerialPortJNI.serialSetBaudRate(port, 115200);
      SerialPortJNI.serialSetDataBits(port, (byte)8);
      SerialPortJNI.serialSetParity(port, (byte)0);
      SerialPortJNI.serialSetStopBits(port, (byte)10);
  
      SerialPortJNI.serialSetTimeout(port, 1); 
      SerialPortJNI.serialEnableTermination(port, '\n');
  
      HAL.report(tResourceType.kResourceType_SerialPort, 2);
  
      byte[] buffer = new byte[257];
      SingleCharSequence charSeq = new SingleCharSequence();
      charSeq.data = buffer;
      IntRef lastComma = new IntRef();
  
      RawColor color0 = new RawColor();
      RawColor color1 = new RawColor();
  
      while (threadRunning.get()) {
        int read = SerialPortJNI.serialRead(port, buffer, buffer.length - 1);
        if (read <= 0) {
          try {
            threadLock.lock();
            this.hasColor0 = false;
            this.hasColor1 = false;
          } finally {
            threadLock.unlock();
          }
          continue;
        }
        if (!threadRunning.get()) {
          break;
        }
  
        // Trim trailing newline if exists
        if (buffer[read - 1] == '\n') {
          read--;
        }
  
        if (read == 0) {
          continue;
        }
  
        if (debugPrints.get()) {
          System.out.println(new String(buffer, 0, read, StandardCharsets.UTF_8));
        }
  
        lastComma.value = -1;
  
        boolean hasColor0 = parseIntFromIndex(charSeq, read, lastComma) != 0;
        boolean hasColor1 = parseIntFromIndex(charSeq, read, lastComma) != 0;
        color0.red = parseIntFromIndex(charSeq, read, lastComma);
        color0.green = parseIntFromIndex(charSeq, read, lastComma);
        color0.blue = parseIntFromIndex(charSeq, read, lastComma);
        color0.ir = parseIntFromIndex(charSeq, read, lastComma);
        int prox0 = parseIntFromIndex(charSeq, read, lastComma);
        color1.red = parseIntFromIndex(charSeq, read, lastComma);
        color1.green = parseIntFromIndex(charSeq, read, lastComma);
        color1.blue = parseIntFromIndex(charSeq, read, lastComma);
        color1.ir = parseIntFromIndex(charSeq, read, lastComma);
        int prox1 = parseIntFromIndex(charSeq, read, lastComma);
  
        double ts = Timer.getFPGATimestamp();
  
        try {
          threadLock.lock();
          this.lastReadTime = ts;
          this.hasColor0 = hasColor0;
          this.hasColor1 = hasColor1;
          if (hasColor0) {
            this.color0.red = color0.red;
            this.color0.green = color0.green;
            this.color0.blue = color0.blue;
            this.color0.ir = color0.ir;
            this.prox0 = prox0;
          }
          if (hasColor1) {
            this.color1.red = color1.red;
            this.color1.green = color1.green;
            this.color1.blue = color1.blue;
            this.color1.ir = color1.ir;
            this.prox1 = prox1;
          }
        } finally {
          threadLock.unlock();
        }
      }
  
      SerialPortJNI.serialClose(port);
    }
  
    public PicoColorSensor() {
      readThread = new Thread(this::threadMain);
      readThread.setName("PicoColorSensorThread");
      readThread.start();
    }
  
    public boolean isSensor0Connected() {
      try {
        threadLock.lock();
        return hasColor0;
      } finally {
        threadLock.unlock();
      }
    }
  
    public boolean isSensor1Connected() {
      try {
        threadLock.lock();
        return hasColor1;
      } finally {
        threadLock.unlock();
      }
    }
  
    public RawColor getRawColor0() {
      try {
        threadLock.lock();
        return new RawColor(color0.red, color0.green, color0.blue, color0.ir);
      } finally {
        threadLock.unlock();
      }
    }
  
    public void getRawColor0(RawColor rawColor) {
      try {
        threadLock.lock();
        rawColor.red = color0.red;
        rawColor.green = color0.green;
        rawColor.blue = color0.blue;
        rawColor.ir = color0.ir;
      } finally {
        threadLock.unlock();
      }
    }
  
    public int getProximity0() {
      try {
        threadLock.lock();
        return prox0;
      } finally {
        threadLock.unlock();
      }
    }
  
    public RawColor getRawColor1() {
      try {
        threadLock.lock();
        return new RawColor(color1.red, color1.green, color1.blue, color1.ir);
      } finally {
        threadLock.unlock();
      }
    }
  
    public void getRawColor1(RawColor rawColor) {
      try {
        threadLock.lock();
        rawColor.red = color1.red;
        rawColor.green = color1.green;
        rawColor.blue = color1.blue;
        rawColor.ir = color1.ir;
      } finally {
        threadLock.unlock();
      }
    }
  
    public int getProximity1() {
      try {
        threadLock.lock();
        return prox1;
      } finally {
        threadLock.unlock();
      }
    }
  
    public double getLastReadTimestampSeconds() {
      try {
        threadLock.lock();
        return lastReadTime;
      } finally {
        threadLock.unlock();
      }
    }
  
    void setDebugPrints(boolean debug) {
      debugPrints.set(debug);
    }
  
    @Override
    public void close() throws Exception {
      threadRunning.set(false);
      readThread.join();
    }

    public void start() {
    }
  }



