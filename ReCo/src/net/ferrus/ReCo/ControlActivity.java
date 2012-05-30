package net.ferrus.ReCo;

import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.TextView;

public class ControlActivity extends Activity {
	private static final String TAG = "ControlActivity";
	
	Map<String, String> state = new HashMap<String, String>();
	
	AlertDialog croakDialog;

	ControlServer   server = new ControlServer(this);
	MainHandler msgHandler = new MainHandler(this, server);
	TouchTracker     touch = new TouchTracker(this, server);

	public void updateDisplay() {
		String text = "";
		text += "Connected to " + state.get("host") + "\n";
		
		Set<String> keys = new TreeSet<String>(state.keySet());
		keys.remove("host");
		
		Iterator<String> it = keys.iterator();
		while(it.hasNext()) {
			String key = it.next();
			String val = state.get(key);
			
			try {
				double dval = Double.parseDouble(val);
				DecimalFormat rounded = new DecimalFormat("#.##");
				text += key + ": " + rounded.format(dval) + "\n";
			}
			catch (NumberFormatException ee) {
				text += key + ": " + val + "\n";
			}
		}

		setStatus(text);
	}
	
	@Override
	public boolean onTouchEvent(MotionEvent ev) {
		View screen = (View) findViewById(R.id.topLayout);
		float HH = screen.getHeight();
		float WW = screen.getWidth();
		
		int count = ev.getPointerCount();
		
		if (count > 2) {
			touch.onStopEvent();
			return true;
		}
		
		for(int ii = 0; ii < count; ++ii) {
			int   nn = ev.getPointerId(ii);
			int   aa = ev.getAction() & MotionEvent.ACTION_MASK;
			float xx = ev.getX(ii) / WW;
			float yy = ev.getY(ii) / HH;
			touch.onTouchEvent(nn, aa, xx, yy);
		}

		return true;
	}
	
	@Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);


    	Log.v(TAG, "onCreate");
        
    	initCroakDialog();
	}
	
    @Override
    public void onStart() {
    	super.onStart();
    	
    	Log.v(TAG, "onStart");
    	
    	server.connect();
    	
    	Log.v(TAG, "onStart (2)");
    }

    public void onPause() {
    	super.onPause();
    }
    
    public void onDestroy() {
    	super.onDestroy();
    	
    	if (server != null) {
    		server.shutdown();
    	}
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main, menu);
        return true;
    }
    
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
       switch (item.getItemId()) {
       default:
    	   finish();
    	   int pid = android.os.Process.myPid();
    	   android.os.Process.killProcess(pid); 
    	   return true;
       }
    }
    
    public Dialog onCreateDialog(int id) {
    	return croakDialog;
    }

    public void putState(String key, String value) {
    	state.put(key, value);
    }
    
    public void setStatus(String text) {
    	TextView status = (TextView) findViewById(R.id.statusText);    		
    	
    	if (status != null) {
        	status.setText(text);
    	} else {
    		Log.v(TAG, "Got null statusText");
    	}
    }
    
    private void initCroakDialog() {
		AlertDialog.Builder ab = new AlertDialog.Builder(this);
		ab.setMessage("Fatal error: default error")
		  .setCancelable(false)
		  .setPositiveButton("Close", new DialogInterface.OnClickListener() {
	           public void onClick(DialogInterface dialog, int id) {
	                ControlActivity.this.finish();
	           }
	       });
		croakDialog = ab.create();
    }
    
    public void croak(String msg) {
		Log.v(TAG, "croak: " + msg);
		
		if (msg.length() < 10) {
			msg = "Fatal Error: " + msg;
		}
		
		croakDialog.setMessage(msg);
    	
    	runOnUiThread(new Runnable() {
    		public void run() {
    			showDialog(1);
    		}
    	});
    }
}