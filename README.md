# Hospital-Grade ECG Monitoring Application

A comprehensive Flutter application for real-time ECG monitoring with hospital-grade filtering, signal quality assessment, and advanced analysis capabilities.

## 🏥 Medical-Grade Features

### Advanced Signal Processings
- **Multi-stage Digital Filtering**: Butterworth high-pass, low-pass, and notch filters
- **DC Offset Removal**: Baseline wander correction
- **Power Line Interference Removal**: 50Hz/60Hz notch filtering
- **Real-time Signal Quality Assessment**: Continuous monitoring of signal integrity
- **Adaptive Noise Reduction**: Dynamic filtering based on signal conditions

### Clinical Compliance
- **12-Lead ECG Support**: Full standard 12-lead configuration
- **Medical Device Standards**: Implements IEC 60601-2-25 guidelines
- **Calibration Standards**: 1mV calibration pulse, 25mm/s paper speed
- **Filter Presets**: Diagnostic (0.05-150Hz), Monitoring (0.5-40Hz), Pediatric (0.05-250Hz)

### Real-time Analysis
- **Heart Rate Detection**: Advanced R-wave detection algorithms
- **Arrhythmia Detection**: Basic rhythm analysis capabilities
- **Lead Connection Monitoring**: Real-time electrode status
- **Battery Monitoring**: Device power level tracking

## 🚀 Quick Start

### Prerequisites
- Flutter SDK (>=3.8.1)
- Dart SDK
- Android Studio / VS Code
- ECG device with Bluetooth Classic support

### Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd ecg_bl
   ```

2. **Install dependencies**
   ```bash
   flutter pub get
   ```

3. **Configure Bluetooth permissions**
   
   **Android** (`android/app/src/main/AndroidManifest.xml`):
   ```xml
   <uses-permission android:name="android.permission.BLUETOOTH" />
   <uses-permission android:name="android.permission.BLUETOOTH_ADMIN" />
   <uses-permission android:name="android.permission.ACCESS_COARSE_LOCATION" />
   <uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />
   ```

4. **Run the application**
   ```bash
   flutter run
   ```

## 📱 Application Architecture

### Core Components

#### 1. Enhanced ECG Controller (`lib/services/enhanced_ecg_controller.dart`)
- Central controller managing ECG data flow
- Integrates filtering, analysis, and UI updates
- Handles device connection and data recording

#### 2. Advanced ECG Filter (`lib/services/advanced_ecg_filter.dart`)
- Hospital-grade digital signal processing
- Multiple filter stages with medical compliance
- Real-time signal quality assessment

#### 3. ECG Data Parser (`lib/services/ecg_data_parser.dart`)
- Robust packet parsing with validation
- Error detection and recovery
- Statistical monitoring

#### 4. Bluetooth Service (`lib/services/bluetooth_service.dart`)
- Reliable Bluetooth communication
- Automatic reconnection
- Connection health monitoring

### Data Models (`lib/models/ecg_data.dart`)
- **ECGLead**: Comprehensive lead definitions
- **ECGPacket**: Validated data packets
- **ECGSample**: Processed 12-lead samples
- **ECGSettings**: Configurable parameters

## 🔧 Configuration

### Filter Settings
```dart
// Diagnostic ECG (0.05-150 Hz)
ECGSettings(
  filterType: ECGFilterType.filter150Hz,
  gain: 1,
  enableNotchFilter: true,
  notchFrequency: 50.0, // or 60.0 for US
)

// Patient Monitoring (0.5-40 Hz)
ECGSettings(
  filterType: ECGFilterType.filter40Hz,
  gain: 2,
  enableNotchFilter: true,
)
```

### Device Connection
```dart
final controller = EnhancedECGController();
await controller.initialize();
final devices = await controller.scanForDevices();
await controller.connectToDevice(devices.first);
```

## 📊 Signal Quality Metrics

The application provides comprehensive signal quality assessment:

- **SNR (Signal-to-Noise Ratio)**: Measured in dB
- **Baseline Stability**: Percentage (0-100%)
- **Amplitude Consistency**: R-wave amplitude variation
- **Noise Level**: High-frequency content assessment
- **Overall Quality**: Composite score (0-100%)

### Quality Levels
- **Excellent** (80-100%): Hospital-grade quality
- **Good** (60-79%): Suitable for monitoring
- **Fair** (40-59%): Marginal quality
- **Poor** (20-39%): Significant artifacts
- **Unacceptable** (0-19%): Invalid signal

## 🏥 Medical Compliance Notes

### ⚠️ Important Disclaimers

1. **Not for Clinical Diagnosis**: This application is for educational and research purposes only
2. **Medical Device Regulation**: Not cleared by FDA or other regulatory bodies
3. **Professional Supervision**: Should only be used under medical professional guidance
4. **Emergency Situations**: Not suitable for emergency or critical care monitoring

### Standards Compliance
- **IEC 60601-2-25**: ECG equipment safety requirements
- **AAMI EC11**: Diagnostic ECG equipment standards
- **AHA/ACC Guidelines**: 12-lead ECG interpretation standards

## 🔍 Troubleshooting

### Common Issues

1. **Bluetooth Connection Failed**
   - Ensure device is paired and within range
   - Check Bluetooth permissions
   - Restart Bluetooth service

2. **Poor Signal Quality**
   - Check electrode connections
   - Verify proper skin preparation
   - Adjust filter settings

3. **High Noise Levels**
   - Enable notch filter for power line interference
   - Check for electromagnetic interference sources
   - Verify proper grounding

### Debug Mode
Enable debug logging for detailed diagnostics:
```dart
debugPrint('ECG Debug: ${controller.getStatistics()}');
```

## 📈 Performance Optimization

### Real-time Processing
- Optimized for 250 Hz sampling rate
- Efficient circular buffers
- Minimal memory allocation
- 60 FPS UI updates

### Memory Management
- Automatic buffer size management
- Garbage collection optimization
- Resource cleanup on dispose

## 🧪 Testing

### Unit Tests
```bash
flutter test
```

### Integration Tests
```bash
flutter test integration_test/
```

### Signal Processing Validation
- Filter frequency response verification
- Signal quality algorithm validation
- Performance benchmarking

## 📚 API Reference

### Key Classes

#### `EnhancedECGController`
```dart
// Initialize and connect
await controller.initialize();
await controller.connectToDevice(device);

// Start/stop recording
controller.startRecording();
controller.stopRecording();

// Get real-time data
final samples = controller.getFilteredSamples(ECGLead.leadII);
final quality = controller.getSignalQuality(ECGLead.leadII);
final heartRate = controller.getCurrentHeartRate();
```

#### `AdvancedECGFilter`
```dart
// Create filter with preset
final filter = AdvancedECGFilter(
  sampleRate: 250.0,
  preset: ECGFilterPreset.diagnostic,
);

// Process sample
final result = filter.process(rawSample);
print('Filtered: ${result.filteredValue}');
print('Quality: ${result.signalQuality.overallQuality}%');
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Implement changes with tests
4. Submit a pull request

### Code Standards
- Follow Dart/Flutter style guidelines
- Include comprehensive documentation
- Add unit tests for new features
- Maintain medical compliance standards

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## ⚖️ Legal Notice

This software is provided "as is" without warranty of any kind. The authors and contributors are not liable for any medical decisions or outcomes based on this software. Always consult qualified medical professionals for clinical decisions.

## 📞 Support

For technical support or medical compliance questions:
- Create an issue on GitHub
- Contact the development team
- Consult medical device regulatory experts

---

**Remember**: This is educational software. Always prioritize patient safety and regulatory compliance in medical applications.
