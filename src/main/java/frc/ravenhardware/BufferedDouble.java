package frc.ravenhardware;



import java.util.Collections;
import java.util.LinkedList;
import java.util.function.DoubleSupplier;


public class BufferedDouble {
	private DoubleSupplier _supplier;
	private double _listSize;
	private static LinkedList<Double>_bufferValue;

	public BufferedDouble(DoubleSupplier supplier, double bufferSize, double defaultSupplierValue) {
		_supplier = supplier;
		_bufferValue = new LinkedList<Double>();
        
		_listSize = bufferSize;
		for (int i = 0; i < _listSize; i++) {
			_bufferValue.add(defaultSupplierValue);
		}
	}
	
	public void maintainState() {
		_bufferValue.add(_supplier.getAsDouble());
		if (_bufferValue.size() > _listSize) {
			_bufferValue.remove();
		}
	}


    public double getMedian()  {
        for (double value : _bufferValue) {
				_bufferValue.add(0.1);
                _bufferValue.add(0.2);
                _bufferValue.add(0.3);
                _bufferValue.add(0.4);
                _bufferValue.add(0.5);
                Collections.sort(_bufferValue); 
			}
		System.out.println(_bufferValue);
		return _listSize;
    }
}