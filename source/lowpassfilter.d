import std.stdio;
import std.math;

class LowPassFilter {
	public float output;
	public float ePow;

	this () {
		output = 0;
		ePow = 0;
	}

	this (float iCutOffFrequency, float iDeltaTime) {
		output = 0;
		ePow = 1-exp(-iDeltaTime * 2 * PI * iCutOffFrequency);

		if (iDeltaTime <= 0){
			writeln("Warning: A LowPassFilter instance has been configured with 0 s as delta time.");
			ePow = 0;
		}

		if(iCutOffFrequency <= 0){
			writeln("Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.");
			ePow = 0;
		}
	}

	float update(float input){
		return output += (input - output) * ePow;
	}

	float update(float input, float deltaTime, float cutoffFrequency){
		reconfigureFilter(deltaTime, cutoffFrequency); //Changes ePow accordingly.
		return output += (input - output) * ePow;
	}

	void reconfigureFilter(float deltaTime, float cutoffFrequency){
		if (deltaTime <= 0){
			writeln("Warning: A LowPassFilter instance has been configured with 0 s as delta time.");
			ePow = 0;
		}
		if(cutoffFrequency <= 0){
			writeln("Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.");
			ePow = 0;
		}
		ePow = 1-exp(-deltaTime * 2 * PI * cutoffFrequency);
	}
}