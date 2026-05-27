from scipy.signal import butter, lfilter
import numpy as np
import matplotlib.pyplot as plt

def butter3_filter_design(cutoff, fs, order=3):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='lowpass', analog=False)
    return b, a

class Butter3:
    def __init__(self, b, a):
        self.B = b
        self.A = a
        self.X = np.zeros(4)
        self.Y = np.zeros(4)

    def process(self, in_value):
        self.X[3] = in_value
        self.Y[3] = (self.B[0] * self.X[3] + self.B[1] * self.X[2] + self.B[2] * self.X[1] + self.B[3] * self.X[0] 
                    - self.A[1] * self.Y[2] - self.A[2] * self.Y[1] - self.A[3] * self.Y[0])
        out_value = self.Y[3]
        self.X[0] = self.X[1]
        self.X[1] = self.X[2]
        self.X[2] = self.X[3]
        self.Y[0] = self.Y[1]
        self.Y[1] = self.Y[2]
        self.Y[2] = self.Y[3]
        return out_value

# Test
if __name__ == "__main__":
    fs = 1000.0
    cutoff = 20.0
    b, a = butter3_filter_design(cutoff, fs, order=3)
    print(f"b: {b}, a: {a}")

    butter_filter = Butter3(b, a)
    noise_signal = np.random.normal(0, 1, 1000)

    fmt_filtered_output = []
    scipy_filtered_output = []
    for i in range(len(noise_signal)):
        _output = butter_filter.process(noise_signal[i])
        fmt_filtered_output.append(_output)
    scipy_filtered_output = lfilter(b, a, noise_signal)

    plt.figure(figsize=(12, 6))
    plt.plot(noise_signal, label='Noise signal')
    plt.plot(fmt_filtered_output, label='Filtered signal')
    plt.plot(scipy_filtered_output, label='Scipy filtered signal')
    plt.xlabel('Time')
    plt.ylabel('Amplitude')
    plt.title('Butterworth 3rd order filter')
    plt.legend()
    plt.show()

    noise_fft = np.fft.fft(noise_signal)
    filtered_fft = np.fft.fft(fmt_filtered_output)
    scipy_filtered_fft = np.fft.fft(scipy_filtered_output)

    plt.figure(figsize=(12, 6))
    plt.plot(np.abs(noise_fft), label='Noise signal')
    plt.plot(np.abs(filtered_fft), label='Filtered signal')
    plt.plot(np.abs(scipy_filtered_fft), label='Scipy filtered signal')
    plt.xlabel('Frequency')
    plt.ylabel('Magnitude')
    plt.title('Butterworth 3rd order filter')
    plt.legend()
    plt.show()
