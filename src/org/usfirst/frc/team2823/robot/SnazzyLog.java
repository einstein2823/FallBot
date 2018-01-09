package org.usfirst.frc.team2823.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class SnazzyLog {
	File m_f;
	FileWriter m_fw;
	BufferedWriter m_bw;
	String m_file;
	String m_header;
	boolean m_open = false;

	public boolean open(String file, String header) {
		if (file == null) {
			return false;
		}
		m_header = header;
		if(m_open) {
			return true;
		}
		m_file = file;
		try {
			if(Files.exists(Paths.get("/home/lvuser"))) {
				m_f = new File("home/lvuser/" + file);
			} else {
				m_f = new File("/tmp/" + file);
			}

			if(!m_f.exists()) {
				m_f.createNewFile();
			}
			m_fw = new FileWriter(m_f);

		} catch(IOException e) {
			e.printStackTrace();
			return false;
		}

		m_bw = new BufferedWriter(m_fw);
		m_open = true;

		if(m_header != null) {
			return write(header);
		}

		return true;
	}

	public void close() {
		if (m_open) {
			try {
				m_bw.close();
				m_fw.close();

			} catch(IOException e) {

			}
			m_open = false;
		}
	}

	public boolean write(String s) {
		if(!m_open) {
			return false;
		}
		try {
			m_bw.write(s);
			m_bw.flush();

		} catch(IOException e) {
			e.printStackTrace();
			return false;
		}
		return true;
	}
	public void reset() {
		close();
		open(m_file, m_header);
	}
}
