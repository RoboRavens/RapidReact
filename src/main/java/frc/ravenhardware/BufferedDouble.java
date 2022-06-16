package frc.ravenhardware;

import java.util.Collections;
import java.util.LinkedList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;


public class BufferedDouble {
	private DoubleSupplier _supplier;
	private double _listSize;
	private LinkedList<Double> _doubleValues;
    

	public BufferedDouble(DoubleSupplier supplier, Double bufferSize, Double defaultSupplierValue) {
		_supplier = supplier;
		_doubleValues = new LinkedList<Double>();
		_listSize = bufferSize;
		for (int i = 0; i < _listSize; i++) {
			_doubleValues.add(defaultSupplierValue);
		}
	}

	// Adds the current sensor value to the list, and
	// removes the first item if the list is larger than the list size.
	public void maintainState() {
		_doubleValues.add(_supplier.getAsDouble());
		if (_doubleValues.size() > _listSize) {
			_doubleValues.remove();
		}
	}

	public void setSize(Double size) {
		this._listSize = size;
	}

	public double getMean() {
		double cumulativeValue = 0;

		for (Double value : _doubleValues) {
			cumulativeValue += value;
		}

		return cumulativeValue / _doubleValues.size();
	}	
}