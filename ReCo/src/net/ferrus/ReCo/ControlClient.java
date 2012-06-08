package net.ferrus.ReCo;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import java.util.Queue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Callable;
import java.util.concurrent.LinkedBlockingQueue;

import android.util.Log;

public class ControlClient implements Runnable {
	private static final String TAG = "ControlClient";
	
	static ControlClient singleton;
	static Thread thread;
	
	DatagramSocket sock;
	InetAddress    addr;
	int            port;
	boolean        good;
	
	String mode  = "speed";
	double speed = 0.0;
	double theta = 0.0;
	
	private ControlClient() {
		try {
			sock = new DatagramSocket();
			sock.setSoTimeout(250);
		} catch (SocketException ee) {
			Log.v(TAG, ee.toString());
		}
	}

	public static ControlClient getInstance() {
		if (singleton == null) {
			singleton = new ControlClient();
			thread    = new Thread(singleton);
			thread.start();
		}
		
		return singleton;
	}

	public synchronized void connect(String host, String portString) {
		try {
			addr = InetAddress.getByName(host);
			port = Integer.parseInt(portString); 
		} catch (UnknownHostException ee) {
			Log.v(TAG, ee.toString());
		}
	}
	
	public synchronized void setCommand(boolean woah, double vv, double aa) {
		mode  = woah ? "woah" : "speed";
		speed = vv;
		theta = aa;
	}
	
	public void run() {
		while (true) {
			String ticket = recv();
			sendCommands(ticket);
		}
	}
	
	private synchronized void sendCommands(String ticket) {
		String msg = String.format("%s;%s %.02f %.02f", ticket, mode, speed, theta);
		send(msg);
	}
	
	private synchronized void send(String data) {
		byte[] buffer = data.getBytes();
		DatagramPacket packet = new DatagramPacket(buffer, buffer.length, addr, port);
		try {
			sock.send(packet);
		} catch (IOException ee) {
			Log.v(TAG, ee.toString());
			send("0.0;connect");
		}
	}
	
	private String recv() {
		byte[] buffer = new byte[2048];
		DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
		try {
			sock.receive(packet);
		} catch (IOException ee) {
			Log.v(TAG, ee.toString());
			return "0.0";
		}
		return new String(buffer).split("\0")[0];
	}
}
