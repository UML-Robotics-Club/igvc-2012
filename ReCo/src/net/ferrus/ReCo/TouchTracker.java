package net.ferrus.ReCo;

import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.Map;

import android.util.Log;
import android.view.MotionEvent;

public class TouchTracker {
	private static final String TAG = "TouchTracker";
	
	ControlActivity view;
	ControlServer server;
	
	final double FORW_MAX = 0.30;
	final double TURN_MAX = 0.15;
	
	// -1 for no active gesture
	int    forw_id    = -1;
	double forw_start = 0.0;

	int    turn_id    = -1;
	double turn_start = 0.0;

	double forw = 0.0;
	double turn = 0.0;
	
	double lastSend = 0.0;
	
	public TouchTracker(ControlActivity aa, ControlServer ss) {
		view = aa;
		server = ss;
	}
	
	public void onTouchEvent(int nn, int aa, float xx, float yy) {
		if (xx < 0.5) {
			// Left half of screen, forward speed
			gotForwardGesture(nn, aa, xx, yy);
		} else {
			// Right half of screen, turn speed
			gotTurnGesture(nn, aa, xx, yy);
		}
		
		DecimalFormat rounded = new DecimalFormat("#.##");
		
		view.putState("z_forw", "(" + forw_id + ", " + rounded.format(forw) + ")");
		view.putState("z_turn", "(" + turn_id + ", " + rounded.format(turn) + ")");
		view.updateDisplay();
	}
	
	private void gotForwardGesture(int nn, int aa, float xx, float yy) {
		// Start a gesture.
		if (aa == MotionEvent.ACTION_DOWN && forw_id == -1) {
			forw_id    = nn;
			forw_start = yy;
			forw       = 0.0;
		}

		// Renumber a gesture
		if (forw_id != nn) {
			forw_id = nn;
			turn_id = -1;
			turn    = 0.0;
		}
		
		// Continue a gesture.
		if (aa == MotionEvent.ACTION_MOVE && forw_id == nn) {
			forw = forw_start - yy;
		}
		
		// End a gesture
		if ((aa == MotionEvent.ACTION_UP || aa == MotionEvent.ACTION_CANCEL) 
				&& forw_id == nn) {
			forw_id = -1;
			forw    = 0.0;
		}

		//Log.v(TAG, "Speed on ptr = " + nn + ", speed = " + forw);
		setSpeeds(forw, turn);
	}
	
	private void gotTurnGesture(int nn, int aa, float xx, float yy) {	
		// Start a gesture.
		if ((aa == MotionEvent.ACTION_DOWN || aa == MotionEvent.ACTION_POINTER_DOWN)
				&& turn_id == -1) {
			turn_id    = nn;
			turn_start = xx;
			turn       = 0.0;
			Log.v(TAG, "Start turn, id = " + nn);
		}

		// Renumber a gesture
		if (turn_id != nn) {
			turn_id = nn;
			forw_id = -1;
			forw    = 0.0;
		}
		
		// Continue a gesture.
		if (aa == MotionEvent.ACTION_MOVE && turn_id == nn) {
			turn = turn_start - xx;
		}
		
		// End a gesture
		if ((aa == MotionEvent.ACTION_UP || aa == MotionEvent.ACTION_POINTER_UP || aa == MotionEvent.ACTION_CANCEL) 
				&& turn_id == nn) {
			turn_id = -1;
			turn    = 0.0;
		}
		
		//Log.v(TAG, "Turn on ptr = " + nn + ", turn = " + turn + ", aa = " + aa);
		setSpeeds(forw, turn);
	}
	
	public void onStopEvent() {
		forw_id = -1;
		turn_id = -1;

		setSpeeds(0.0, 0.0);
	}
	
	private double scaleMax(double xx, double MAX) {
		xx = (xx / MAX);
		xx = xx >  1.0 ?  1.0 : xx;
		xx = xx < -1.0 ? -1.0 : xx; 
		return xx;
	}
	
	public void setSpeeds() {
		setSpeeds(forw, turn);
	}
	
	public void setSpeeds(double forward, double turn) {
		//Log.v(TAG, "speed = " + forward + ", turn = " + turn);
		double now = RealTime.getTime();
		
		if (now - lastSend > 0.05) {
			lastSend = now;
			
			Map<String, String> map = new HashMap<String, String>();
			map.put("type", "set-speeds");
			map.put("ahead", "" + scaleMax(forward, FORW_MAX));
			map.put("turn",  "" + scaleMax(turn, TURN_MAX));
			server.sendMessage(map);
		}
		
		this.forw = forward;
		this.turn = turn;
	}
}
