package net.ferrus.ReCo;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import android.os.Handler;
import android.os.Message;
import android.util.Log;

public class MainHandler extends Handler {
	private static final String TAG = "MainHandler";
	
	public ControlActivity view;
	public ControlServer   server;
	
	double pingRequest;
	int    pingSerial;
	
	public MainHandler(ControlActivity aa, ControlServer ss) {
		view       = aa;
		server     = ss;
		pingSerial = 1;
	}
	
    @SuppressWarnings("unchecked")
	@Override
    public void handleMessage(Message rawMsg) {
    	Map<String, String> msg = (Map<String, String>) rawMsg.obj;
    	//Log.v(TAG, "Got message: " + msg.toString());

    	String type;
    	type = msg.get("type");

    	if (type.equals("ready")) {
    		serverReady(msg);
    	} else if (type.equals("hello")) {
    		gotHello(msg);
    	} else if (type.equals("goodbye")) {
    		gotGoodbye(msg);
    	} else if (type.equals("get-status")) {
    		requestStatus();
    	} else if (type.equals("status")) {
    		gotStatus(msg);	    		
    	} else {
    		Log.v(TAG, "Unknown message type: " + type);
    	}
    }
    
    public void serverReady(Map<String, String> msg) {
    	view.setStatus("Waiting for connection on\n" + msg.get("address"));
    }
    
	public void gotHello(Map<String, String> msg) {
		String remoteHost = msg.get("host");
		view.putState("host", remoteHost);
		view.updateDisplay();
		requestStatus();
	}
	
	public void gotStatus(Map<String, String> msg) {
		double pingResponse = RealTime.getTime();
		int    pongSerial   = Integer.parseInt(msg.get("pong"));
		
		if (pongSerial == pingSerial) {
			double delay = pingResponse - pingRequest;
			
			view.putState("delay", "" + delay);
			
			Map<String, String> sync = new HashMap<String, String>();
			sync.put("type", "status-sync");
			sync.put("sync", "" + pongSerial);
			sync.put("delay", "" + delay);
			server.sendMessage(sync);
		}
		
		Set<String> keys = msg.keySet();
		keys.remove("type");
		keys.remove("pong");
		
		Iterator<String> it = keys.iterator();
		while(it.hasNext()) {
			String key = it.next();
			view.putState(key, msg.get(key));
		}
		
		view.updateDisplay();
	}
	
	public void gotGoodbye(Map<String, String> msg) {
		view.croak("Remote host disconnected");
	}

	void requestStatus() {
		pingRequest = RealTime.getTime();
		pingSerial += 1;
		
		Map<String, String> map = new HashMap<String, String>();
		map.put("type",  "get-status");
		map.put("ping",	 "" + pingSerial);
		map.put("stamp", "" + pingRequest);
		server.sendMessage(map);
		
		view.touch.setSpeeds();
		
		// Queue the next request.
		Message msg = obtainMessage();
		msg.obj = (Object) map;
		sendMessageDelayed(msg, 100);
	}
}
