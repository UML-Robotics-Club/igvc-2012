package net.ferrus.ReCo;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import org.json.JSONException;
import org.json.JSONObject;
import android.os.Message;
import android.util.Log;

public class ControlServer implements Runnable {
	private static final String TAG = "ControlServer";
	final int PORT = 4377;

	ServerSocket svr;
	Socket sock;
	Thread inputThread;
	
	InetAddress addr;
	ControlActivity view;
	
	public ControlServer(ControlActivity vv) {
		view = vv;
	}
	
	public void connect() {
		Log.v(TAG, "ControlServer startup");
				
		try {
			addr = findBestAddress();
			svr  = new ServerSocket(PORT, 5, addr);
			svr.setReuseAddress(true);
		} catch (IOException ee) {
			Log.v(TAG, "IOException: " + ee.toString());
			view.croak(ee.toString());
			return;
		}
		
		Log.v(TAG, "Listening on: " + getIP());
		
		Message rawMsg = Message.obtain();
		Map<String, String> msg = new HashMap<String, String>();
		msg.put("type", "ready");
		msg.put("address", getIP());
		rawMsg.obj = msg;
		view.msgHandler.sendMessage(rawMsg);
		
		inputThread = new Thread(this);
		inputThread.start();
	}
	
	public void shutdown() {
		Log.v(TAG, "Closing server socket");
		try {
			svr.close();
		} catch (IOException ee) {
			Log.v(TAG, "IOException: " + ee.toString());			
		}
	}
	
	public String getIP() {
		return addr.getHostAddress();
	}

	public void run() {
		while(true) {
			acceptConnections();
		}
	}
	
	private void acceptConnections() {
		InputStream conn = null;
		JSONObject msg = null;
		
		try {
			sock = svr.accept();
			conn = sock.getInputStream();
		} catch (IOException ee) {
			view.croak(ee.toString());
		}
		
		while(sock.isConnected()) {
			try {
				msg = readMessage(conn);
			} catch (EOFException ee) {
				Log.v(TAG, "Remote host closed socket");
				break;
			} catch (IOException ee) {
				Log.v(TAG, "IO Error: " + ee.toString());
				break;
			}
			
			//Log.v(TAG, msg.toString());

			Message rawMsg = Message.obtain();
			rawMsg.obj = jsonToMap(msg);
			view.msgHandler.sendMessage(rawMsg);
		}
	}
	
	private JSONObject readMessage(InputStream conn) throws IOException {
		DataInputStream din = new DataInputStream(conn);
		String text = din.readUTF();
		JSONObject msg = null;
		
		if (text.length() == 0) {
			throw new IOException("Zero length message");
		}
		
		try {
			msg = new JSONObject(text);
		} catch (JSONException ee) {
			throw new IOException("JSON says: " + ee.toString());
		}
		
		return msg;
	}

	public void sendMessage(Map<String, String> msg) {
		if (sock == null) {
			Log.v(TAG, "Null socket, not sending message");
			return;
		}
		
		try {
			OutputStream conn = sock.getOutputStream();
			DataOutputStream dout = new DataOutputStream(conn);
			JSONObject jsonMsg = mapToJson(msg);
			dout.writeUTF(jsonMsg.toString());
			dout.flush();
		} catch (IOException ee) {
			Log.v(TAG, "IOException: " + ee.toString());
			view.croak(ee.toString());
		}
	}

	private InetAddress findBestAddress() {
		InetAddress best = null;
		
		try {
			Enumeration<NetworkInterface> is = NetworkInterface.getNetworkInterfaces();
			
			while(is.hasMoreElements()) {
				Enumeration<InetAddress> as = is.nextElement().getInetAddresses();

				while(as.hasMoreElements()) {
					InetAddress aa = as.nextElement();
					int block = (int) aa.getAddress()[0];
					block = (block < 0) ? (block + 256) : block; 
					
					Log.v(TAG, "Address: " + aa.getHostAddress());
					Log.v(TAG, "Block: " + block);
					
					// Prefer WiFi, either private address
					// space or UMass Lowell block.
					if (block == 10 || block == 192 || 
							block == 172 || block == 129) {
						return aa;
					}
					
					if (!aa.isLoopbackAddress()) {
						best = aa;
					}
				}
			}			
		} catch (SocketException ee) {
			view.croak(ee.toString());
		}
		
		return best;
	}
	
	@SuppressWarnings("unchecked")
	private Map<String, String> jsonToMap(JSONObject aa) {
		Map<String, String> bb = new HashMap<String, String>();

		Iterator<String> ii = (Iterator<String>) aa.keys();
		while (ii.hasNext()) {
			String key = ii.next();
			try {				
				bb.put(key, aa.getString(key));
			} catch (JSONException e) {
				bb.put(key, "JSONError");
			}
		}
		
		return bb;
	}
	
	private JSONObject mapToJson(Map<String, String> aa) {
		JSONObject bb = new JSONObject();

		Iterator<String> ii = (Iterator<String>) aa.keySet().iterator();
		while (ii.hasNext()) {
			String key = ii.next();
			try {
				bb.put(key, aa.get(key));
			} catch (JSONException ee) {
				ee.printStackTrace();
				throw new Error("Unexpected JSONError");
			}
		}
		
		return bb;
	}
}
