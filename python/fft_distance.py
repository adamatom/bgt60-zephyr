# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "numpy",
# ]
# ///
import argparse
import numpy as np
import math


c = 299792458.0
B = 63e9 - 58e9


def main() -> None:
    parser = argparse.ArgumentParser(description="Compute distance from chirp data file.")
    parser.add_argument("filename", help="Path to file containing comma-separated integer chirp data.")
    args = parser.parse_args()

    chirp_data = load_chirp_data_from_bytes_file(args.filename)
    #chirp_data = load_chirp_data_from_csv_file(args.filename)
    print(chirp_data)
    # num_samples = len(chirp_data)
    algo = DistanceAlgo(512)
    # distance_m, _ = algo.compute_distance(chirp_data)
    distance_m, _ = algo.compute_distance_from_data(chirp_data, 512)

    print(f"Peak distance: {distance_m:.2f} meters")


def load_chirp_data_from_bytes_file(filename):
    with open(filename, 'r') as f:
        content = f.read()

    # Split by spaces and filter out empty tokens
    tokens = content.strip().split()
    # Convert from hex strings to integers
    return [int(tok, 16) for tok in tokens if tok]

def load_chirp_data_from_csv_file(filename):
    with open(filename, 'r') as f:
        content = f.read()
    # Strip trailing commas/spaces and split by comma
    tokens = content.strip().strip(',').split(',')
    # Convert to list of integers
    return [int(tok.strip()) for tok in tokens if tok.strip()]


def extract_samples(data, num_samples):
    samples = [0] * num_samples  # Preallocate list for better performance
    sample_index = 0
    data_length = len(data)
    i = 0

    # Process data in chunks of 3 bytes to extract two 12-bit samples each
    while i + 2 < data_length and sample_index < num_samples:
        b0, b1, b2 = data[i], data[i+1], data[i+2]

        # First 12-bit sample
        samples[sample_index] = (b0 << 4) | (b1 >> 4)
        sample_index += 1
        if sample_index >= num_samples:
            break

        # Second 12-bit sample
        samples[sample_index] = ((b1 & 0x0F) << 8) | b2
        sample_index += 1
        i += 3  # Move to the next set of 3 bytes

    return samples


def blackmanharris(M):
    """
    Generate a Blackman-Harris window.
    M: number of samples.
    """
    if M < 1:
        return []

    a0 = 0.35875
    a1 = 0.48829
    a2 = 0.14128
    a3 = 0.01168

    window = []
    for n in range(M):
        term1 = a1 * math.cos(2.0 * math.pi * n / (M - 1))
        term2 = a2 * math.cos(4.0 * math.pi * n / (M - 1))
        term3 = a3 * math.cos(6.0 * math.pi * n / (M - 1))
        value = a0 - term1 + term2 - term3
        window.append(value)

    return window


class DistanceAlgo:
    """Algorithm for computation of distance FFT from raw data"""

    def __init__(self, num_samples):
        self.num_samples = num_samples
        self.bandwidth_hz = B

        # Compute Blackman-Harris Window over chirp samples
        self.range_window = np.array(blackmanharris(num_samples))

        # FFT size with zero-padding
        fft_size = num_samples * 2
        self.range_bin_length = c / (2 * self.bandwidth_hz * fft_size / num_samples)

    # Compute distance from passed in data, extract samples from data
    def compute_distance_from_data(self, data, size, skip=12):
        samples = extract_samples(data, size)
        return self.compute_distance(samples, skip)

    def compute_distance(self, chirp_data, skip=12):
        """
        Computes distance using chirp data (single chirp).

        :param chirp_data: Input data from chirp (ulab array)
        :param skip: Number of initial samples to skip in peak search
        :return: Tuple containing peak distance in meters and absolute FFT spectrum
        """

        # Step 1 - Calculate range FFT spectrum of the frame
        range_fft = self.fft_spectrum(chirp_data, self.range_window)

        # Step 2 - Convert to absolute spectrum
        range_fft_abs = abs(range_fft)

        # Step 3 - Vectorized Summation
        self.distance_data = range_fft_abs / self.num_samples

        # Step 4 - Peak search and distance calculation
        distance_peak = np.argmax(self.distance_data[skip:])
        distance_peak_m = self.range_bin_length * (distance_peak + skip)

        return distance_peak_m, range_fft_abs

    def compute_distance_old(self, chirp_data, skip=12):
        # Computes distance using chirp data (single chirp)

        #save_array_to_file(chirp_data, "data/1-chirp_data.txt")

        # Step 1 - calculate range FFT spectrum of the frame
        range_fft = self.fft_spectrum(chirp_data, self.range_window)
        #save_array_to_file(range_fft, "data/2-fft.txt")

        # Step 2 - convert to absolute spectrum
        range_fft_abs = abs(range_fft)
        #save_array_to_file(range_fft_abs, "data/3-fft-abs.txt")

        # Manually sum along axis=0 (since range_fft_abs is likely a 1D array)
        distance_data = np.zeros(len(range_fft_abs))
        for i in range(len(range_fft_abs)):
            distance_data[i] = range_fft_abs[i] / self.num_samples

        #save_array_to_file(distance_data, "data/4-distance.txt")

        # Step 3 - peak search and distance calculation
        distance_peak = np.argmax(distance_data[skip:])

        distance_peak_m = self.range_bin_length * (distance_peak + skip)
        return distance_peak_m, range_fft_abs

    def fft_spectrum(self, chirp_data, range_window):
        # Calculate FFT spectrum
        # chirp_data: single chirp data (1D array)
        # range_window: window applied on input data before FFT

        # Step 1 - remove DC bias from the single chirp
        avgs = sum(chirp_data) / len(chirp_data)  # Manually compute the average

        # Subtract the average from each element in chirp_data
        chirp_data = [sample - avgs for sample in chirp_data]

        # Convert chirp_data to numpy array for further operations
        chirp_data = np.array(chirp_data)

        # Step 2 - Windowing the Data
        chirp_data = chirp_data * range_window

        #print(f"chirp_data: {chirp_data}")

        # Step 3 - Add zero padding manually
        # Manually append zeros for padding
        padded_chirp = np.zeros(len(chirp_data) * 2)
        for i in range(len(chirp_data)):
            padded_chirp[i] = chirp_data[i]

        # Step 4 - Compute FFT for distance information
        range_fft = np.fft.fft(padded_chirp) / self.num_samples

        # Step 5 - Ignore redundant negative spectrum and double the magnitude
        range_fft = 2 * range_fft[:self.num_samples]

        return range_fft

if __name__ == "__main__":
    main()
