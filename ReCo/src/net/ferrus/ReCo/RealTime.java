package net.ferrus.ReCo;

import android.os.SystemClock;

public class RealTime {
	static long bootTime = 0;
	
	static double getTime() {
		if (bootTime == 0) {
			bootTime = System.currentTimeMillis() - SystemClock.elapsedRealtime();	
		}
		
		return (bootTime + SystemClock.elapsedRealtime()) / 1000.0;
	}
	
	static String getTimeString() {
		return "" + getTime();
	}
}
