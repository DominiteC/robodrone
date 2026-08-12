# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
from scipy import signal
from scipy.fft import fft, fftfreq
import os

print("Loading vibration data...")
file_path = "G:\\drone\\roboFly20260120\\roboFly\\project\\user\\tool\\python\\data\\vibration_1775635117.csv"
data = pd.read_csv(file_path)

print(f"Data shape: {data.shape}")
print(f"Columns: {data.columns.tolist()}")

# Check gyro data
gyro_cols = ['raw_gx', 'raw_gy', 'raw_gz']
print("\nGyro data statistics:")
for col in gyro_cols:
    print(f"{col}: mean={data[col].mean():.6f}, std={data[col].std():.6f}")

# Calculate sampling parameters
time_ms = data['timestamp_ms'].values
sampling_interval = np.mean(np.diff(time_ms))
sampling_freq = 1000 / sampling_interval
print(f"\nSampling parameters:")
print(f"  Interval: {sampling_interval:.2f} ms")
print(f"  Frequency: {sampling_freq:.2f} Hz")
print(f"  Total time: {(time_ms[-1] - time_ms[0])/1000:.2f} s")
print(f"  Data points: {len(time_ms)}")

# Extract gyro data
gx = data['raw_gx'].values
gy = data['raw_gy'].values
gz = data['raw_gz'].values

# Remove DC component
gx_detrended = gx - np.mean(gx)
gy_detrended = gy - np.mean(gy)
gz_detrended = gz - np.mean(gz)

# Apply window
window = np.hanning(len(gx_detrended))
gx_windowed = gx_detrended * window
gy_windowed = gy_detrended * window
gz_windowed = gz_detrended * window

# FFT
n = len(gx_windowed)
fft_gx = fft(gx_windowed)
fft_gy = fft(gy_windowed)
fft_gz = fft(gz_windowed)

# Frequency axis
freqs = fftfreq(n, d=1/sampling_freq)
positive_freq_idx = freqs >= 0
freqs_pos = freqs[positive_freq_idx]

# Magnitude spectrum
magnitude_gx = np.abs(fft_gx[positive_freq_idx]) / (n/2)
magnitude_gy = np.abs(fft_gy[positive_freq_idx]) / (n/2)
magnitude_gz = np.abs(fft_gz[positive_freq_idx]) / (n/2)

print(f"\nFFT parameters:")
print(f"  FFT points: {n}")
print(f"  Frequency resolution: {freqs_pos[1] - freqs_pos[0]:.4f} Hz")
print(f"  Nyquist frequency: {sampling_freq/2:.2f} Hz")

# Find significant peaks
def find_peaks(magnitude, freqs, threshold_mult=3):
    mean_mag = np.mean(magnitude)
    std_mag = np.std(magnitude)
    threshold = mean_mag + threshold_mult * std_mag
    
    peaks, _ = signal.find_peaks(magnitude, height=threshold, distance=sampling_freq/10)
    
    peak_info = []
    for idx in peaks:
        freq = freqs[idx]
        mag = magnitude[idx]
        total_energy = np.sum(magnitude ** 2)
        energy_percent = (magnitude[idx] ** 2) / total_energy * 100
        
        peak_info.append({
            'frequency': freq,
            'magnitude': mag,
            'energy_percent': energy_percent
        })
    
    return peak_info

print("\n" + "="*60)
print("X-axis gyro significant peaks:")
print("="*60)
peaks_gx = find_peaks(magnitude_gx, freqs_pos, 3)
for i, peak in enumerate(peaks_gx):
    print(f"Peak {i+1}: {peak['frequency']:.2f} Hz, "
          f"Magnitude: {peak['magnitude']:.6f}, "
          f"Energy: {peak['energy_percent']:.2f}%")

print("\n" + "="*60)
print("Y-axis gyro significant peaks:")
print("="*60)
peaks_gy = find_peaks(magnitude_gy, freqs_pos, 3)
for i, peak in enumerate(peaks_gy):
    print(f"Peak {i+1}: {peak['frequency']:.2f} Hz, "
          f"Magnitude: {peak['magnitude']:.6f}, "
          f"Energy: {peak['energy_percent']:.2f}%")

print("\n" + "="*60)
print("Z-axis gyro significant peaks:")
print("="*60)
peaks_gz = find_peaks(magnitude_gz, freqs_pos, 3)
for i, peak in enumerate(peaks_gz):
    print(f"Peak {i+1}: {peak['frequency']:.2f} Hz, "
          f"Magnitude: {peak['magnitude']:.6f}, "
          f"Energy: {peak['energy_percent']:.2f}%")

# Energy distribution
print("\n" + "="*60)
print("Energy distribution analysis:")
print("="*60)

def energy_distribution(magnitude, freqs, axis_name):
    freq_bands = [
        (0, 10, "0-10 Hz"),
        (10, 50, "10-50 Hz"),
        (50, 100, "50-100 Hz"),
        (100, 200, "100-200 Hz"),
        (200, sampling_freq/2, f"200-{sampling_freq/2:.0f} Hz")
    ]
    
    total_energy = np.sum(magnitude ** 2)
    
    print(f"\n{axis_name}-axis:")
    for low, high, label in freq_bands:
        band_mask = (freqs >= low) & (freqs < high)
        if np.any(band_mask):
            band_energy = np.sum(magnitude[band_mask] ** 2)
            energy_percent = band_energy / total_energy * 100
            print(f"  {label}: {energy_percent:.1f}%")

energy_distribution(magnitude_gx, freqs_pos, "X")
energy_distribution(magnitude_gy, freqs_pos, "Y")
energy_distribution(magnitude_gz, freqs_pos, "Z")

# Filter design recommendations
print("\n" + "="*60)
print("Notch filter design recommendations:")
print("="*60)

def filter_recommendations(peaks, axis_name):
    if not peaks:
        print(f"\n{axis_name}-axis: No significant peaks")
        return
    
    print(f"\n{axis_name}-axis recommendations:")
    sorted_peaks = sorted(peaks, key=lambda x: x['energy_percent'], reverse=True)
    
    for i, peak in enumerate(sorted_peaks[:3]):
        freq = peak['frequency']
        energy = peak['energy_percent']
        
        print(f"\n  Peak {i+1} ({freq:.1f} Hz, {energy:.1f}% energy):")
        
        # Bandwidth recommendation
        if freq < 20:
            bandwidth = 2
            q_value = freq / bandwidth
            print(f"    Bandwidth: ±{bandwidth} Hz")
            print(f"    Q value: {q_value:.1f}")
        elif freq < 100:
            bandwidth = 5
            q_value = freq / bandwidth
            print(f"    Bandwidth: ±{bandwidth} Hz")
            print(f"    Q value: {q_value:.1f}")
        else:
            bandwidth = 10
            q_value = freq / bandwidth
            print(f"    Bandwidth: ±{bandwidth} Hz")
            print(f"    Q value: {q_value:.1f}")
        
        # Filter order
        if energy > 10:
            print(f"    Filter: 2nd order IIR notch")
            print(f"    Expected attenuation: > -30 dB")
        elif energy > 5:
            print(f"    Filter: 2nd order IIR notch")
            print(f"    Expected attenuation: -20 to -30 dB")
        else:
            print(f"    Filter: 1st or 2nd order IIR notch")
            print(f"    Expected attenuation: -10 to -20 dB")

filter_recommendations(peaks_gx, "X")
filter_recommendations(peaks_gy, "Y")
filter_recommendations(peaks_gz, "Z")

# Save results
output_dir = "G:\\drone\\roboFly20260120\\roboFly\\project\\user\\tool\\python\\analysis"
os.makedirs(output_dir, exist_ok=True)

report_path = os.path.join(output_dir, "vibration_analysis_summary.txt")
with open(report_path, "w") as f:
    f.write("VIBRATION SPECTRUM ANALYSIS REPORT\n")
    f.write("="*60 + "\n\n")
    
    f.write(f"Data file: vibration_1775635117.csv\n")
    f.write(f"Sampling frequency: {sampling_freq:.2f} Hz\n")
    f.write(f"Total time: {(time_ms[-1] - time_ms[0])/1000:.2f} s\n\n")
    
    f.write("SIGNIFICANT FREQUENCY PEAKS:\n")
    f.write("-"*60 + "\n")
    
    for axis_name, peaks in [("X-axis", peaks_gx), ("Y-axis", peaks_gy), ("Z-axis", peaks_gz)]:
        f.write(f"\n{axis_name}:\n")
        for i, peak in enumerate(peaks):
            f.write(f"  Peak {i+1}: {peak['frequency']:.2f} Hz, "
                   f"Energy: {peak['energy_percent']:.2f}%\n")

print(f"\nReport saved to: {report_path}")
print("\nAnalysis complete!")
