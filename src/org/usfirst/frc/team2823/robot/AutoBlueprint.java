package org.usfirst.frc.team2823.robot;

import java.util.function.IntSupplier;

public class AutoBlueprint {
	double m_timeout;
	IntSupplier m_start;
	IntSupplier m_periodic;
	
	public AutoBlueprint(double timeout, IntSupplier start, IntSupplier periodic) {
		m_timeout = timeout;
		m_start = start;
		m_periodic = periodic;
		
	}
	
	
	
}
