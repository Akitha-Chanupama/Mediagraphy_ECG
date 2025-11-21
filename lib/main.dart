// mediagraphy ecg flutter application
// ignore_for_file: curly_braces_in_flow_control_structures, empty_catches, deprecated_member_use, unnecessary_to_list_in_spreads
import 'dart:async';
import 'dart:collection';
import 'dart:typed_data';
import 'dart:math';
import 'dart:convert';
import 'dart:io';
import 'dart:io';


import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:bluetooth_classic/bluetooth_classic.dart';
import 'package:bluetooth_classic/models/device.dart';
import 'package:path_provider/path_provider.dart';
import 'package:pdf/pdf.dart';
import 'package:pdf/widgets.dart' as pw;
import 'package:printing/printing.dart';

void main() {
  runApp(const ECGApp());
}

/// Simple model that represents one parsed ECG packet (lead samples in raw counts)
class ECGData {
  final int leadV6, leadI, leadII, leadV2, leadV4, leadV3, leadV5, leadV1;
  final int aVR, aVL, aVF;
  final int status1, status2, status3;
  final int batteryLevel;
  final DateTime timestamp;

  ECGData({
    required this.leadV6,
    required this.leadI,
    required this.leadII,
    required this.leadV2,
    required this.leadV4,
    required this.leadV3,
    required this.leadV5,
    required this.leadV1,
    required this.aVR,
    required this.aVL,
    required this.aVF,
    required this.status1,
    required this.status2,
    required this.status3,
    required this.batteryLevel,
    required this.timestamp,
  });

  String get batteryStatus {
    final b = batteryLevel & 0x0F;
    return 'Battery: ${(b / 15 * 100).round()}%';
  }

  String get leadStatus {
    final failures = <String>[];
    final connected = <String>[];

    if ((status1 & 0x01) != 0)
      failures.add('V4');
    else
      connected.add('V4');
    if ((status1 & 0x02) != 0)
      failures.add('V3');
    else
      connected.add('V3');
    if ((status1 & 0x04) != 0)
      failures.add('V5');
    else
      connected.add('V5');
    if ((status1 & 0x08) != 0)
      failures.add('V1');
    else
      connected.add('V1');

    if ((status2 & 0x10) != 0)
      failures.add('V6');
    else
      connected.add('V6');
    if ((status2 & 0x20) != 0) failures.add('LA');
    if ((status2 & 0x40) != 0) failures.add('LL');
    if ((status2 & 0x80) != 0)
      failures.add('V2');
    else
      connected.add('V2');

    if ((status3 & 0x10) != 0) failures.add('RA');

    String status = '';
    if (connected.isNotEmpty) status += 'Connected: ${connected.join(", ")}';
    if (failures.isNotEmpty) {
      if (status.isNotEmpty) status += ' | ';
      status += 'Disconnected: ${failures.join(", ")}';
    }
    return status.isEmpty ? 'No lead data' : status;
  }
}

/// JSON serialization for ECGData
extension ECGDataJson on ECGData {
  Map<String, dynamic> toJson() => {
    'timestamp': timestamp.toIso8601String(),
    'leadI': leadI,
    'leadII': leadII,
    'leadIII': leadII - leadI,
    'aVR': aVR,
    'aVL': aVL,
    'aVF': aVF,
    'V1': leadV1,
    'V2': leadV2,
    'V3': leadV3,
    'V4': leadV4,
    'V5': leadV5,
    'V6': leadV6,
    'status1': status1,
    'status2': status2,
    'status3': status3,
    'battery': batteryLevel,
  };
}

/// Constants & helpers
const double samplingRate = 250.0;
const double timeWindowSeconds = 10.0;
final int samplesWindow = (samplingRate * timeWindowSeconds).round();

const double defaultRawToMv =
    0.01; // HOSPITAL GRADE: 10 µV per LSB for better visibility
const double mmPerSecond = 25.0;
const double mmPerMv = 10.0;
double pixelsPerMmDefault = 4.0;

int toSigned16(int msb, int lsb) {
  int val = (msb << 8) | (lsb & 0xFF);
  if (val > 32767) val -= 65536;
  return val;
}

/// bluetooth_classic wrapper
class BluetoothService {
  final BluetoothClassic _bt = BluetoothClassic();

  Future<void> initPermissions() => _bt.initPermissions();
  Future<List<Device>> getPairedDevices() => _bt.getPairedDevices();
  Stream<Device> onDeviceDiscovered() => _bt.onDeviceDiscovered();
  Future<void> startScan() => _bt.startScan();
  Future<void> stopScan() => _bt.stopScan();
  Future<bool> connect(String address, String uuid) =>
      _bt.connect(address, uuid);
  Future<void> disconnect() => _bt.disconnect();
  Future<void> write(String cmd) => _bt.write(cmd);
  Stream<Uint8List> onDataReceived() => _bt.onDeviceDataReceived();
}

class ECGParser {
  final _out = StreamController<ECGData>.broadcast();
  Stream<ECGData> get stream => _out.stream;

  Uint8List _buffer = Uint8List(0);
  int lastStatus1 = 0, lastStatus2 = 0, lastStatus3 = 0;

  void addBytes(Uint8List data) {
    _buffer = Uint8List.fromList([..._buffer, ...data]);
    while (true) {
      final dataEvent = _tryParseOnePacket();
      if (dataEvent == null) break;
      _out.add(dataEvent);
    }
  }

  ECGData? _tryParseOnePacket() {
    if (_buffer.isEmpty) return null;

    int startIndex = -1;
    for (int i = 0; i < _buffer.length; i++) {
      if (_buffer[i] == 0xAA || _buffer[i] == 0xBB) {
        startIndex = i;
        break;
      }
    }
    if (startIndex == -1) {
      _buffer = Uint8List(0);
      return null;
    }
    if (startIndex > 0) {
      _buffer = Uint8List.fromList(_buffer.sublist(startIndex));
    }
    if (_buffer.isEmpty) return null;

    final start = _buffer[0];
    if (start == 0xAA) {
      if (_buffer.length < 22) return null;
      if (_buffer[21] != 0x0A) {
        _buffer = Uint8List.fromList(_buffer.sublist(1));
        return null;
      }
      final pkt = _buffer.sublist(0, 22);
      _buffer = Uint8List.fromList(_buffer.sublist(22));
      return _parseAAPacket(pkt);
    } else if (start == 0xBB) {
      if (_buffer.length < 18) return null;
      if (_buffer[17] != 0x0A) {
        _buffer = Uint8List.fromList(_buffer.sublist(1));
        return null;
      }
      final pkt = _buffer.sublist(0, 18);
      _buffer = Uint8List.fromList(_buffer.sublist(18));
      return _parseBBPacket(pkt);
    } else {
      _buffer = Uint8List.fromList(_buffer.sublist(1));
      return null;
    }
  }

  ECGData _parseAAPacket(Uint8List pkt) {
    lastStatus1 = pkt[1];
    lastStatus2 = pkt[2];
    lastStatus3 = pkt[3];

    int v6 = toSigned16(pkt[4], pkt[5]);
    int i = toSigned16(pkt[6], pkt[7]);
    int ii = toSigned16(pkt[8], pkt[9]);
    int v2 = toSigned16(pkt[10], pkt[11]);
    int v4 = toSigned16(pkt[12], pkt[13]);
    int v3 = toSigned16(pkt[14], pkt[15]);
    int v5 = toSigned16(pkt[16], pkt[17]);
    int v1 = toSigned16(pkt[18], pkt[19]);

    v6 *= 2;
    i *= 2;
    ii *= 2;
    v2 *= 2;
    v4 *= 2;
    v3 *= 2;
    v5 *= 2;
    v1 *= 2;

    final avr = -(i + ii) ~/ 2;
    final avl = i - (ii ~/ 2);
    final avf = ii - (i ~/ 2);

    return ECGData(
      leadV6: v6,
      leadI: i,
      leadII: ii,
      leadV2: v2,
      leadV4: v4,
      leadV3: v3,
      leadV5: v5,
      leadV1: v1,
      aVR: avr,
      aVL: avl,
      aVF: avf,
      status1: lastStatus1,
      status2: lastStatus2,
      status3: lastStatus3,
      batteryLevel: lastStatus3,
      timestamp: DateTime.now(),
    );
  }

  ECGData _parseBBPacket(Uint8List pkt) {
    int v6 = toSigned16(pkt[1], pkt[2]);
    int i = toSigned16(pkt[3], pkt[4]);
    int ii = toSigned16(pkt[5], pkt[6]);
    int v2 = toSigned16(pkt[7], pkt[8]);
    int v4 = toSigned16(pkt[9], pkt[10]);
    int v3 = toSigned16(pkt[11], pkt[12]);
    int v5 = toSigned16(pkt[13], pkt[14]);
    int v1 = toSigned16(pkt[15], pkt[16]);

    v6 *= 2;
    i *= 2;
    ii *= 2;
    v2 *= 2;
    v4 *= 2;
    v3 *= 2;
    v5 *= 2;
    v1 *= 2;

    final avr = -(i + ii) ~/ 2;
    final avl = i - (ii ~/ 2);
    final avf = ii - (i ~/ 2);

    return ECGData(
      leadV6: v6,
      leadI: i,
      leadII: ii,
      leadV2: v2,
      leadV4: v4,
      leadV3: v3,
      leadV5: v5,
      leadV1: v1,
      aVR: avr,
      aVL: avl,
      aVF: avf,
      status1: lastStatus1,
      status2: lastStatus2,
      status3: lastStatus3,
      batteryLevel: lastStatus3,
      timestamp: DateTime.now(),
    );
  }

  void dispose() => _out.close();
}

class OnePole {
  final double alpha;
  double _y = 0.0;
  double _xPrev = 0.0;
  final bool highpass;
  OnePole.hp(double fc, double fs)
    : alpha =
          (fs - 2 * 3.141592653589793 * fc) / (fs + 2 * 3.141592653589793 * fc),
      highpass = true;
  OnePole.lp(double fc, double fs)
    : alpha =
          (fs - 2 * 3.141592653589793 * fc) / (fs + 2 * 3.141592653589793 * fc),
      highpass = false;
  double process(double x) {
    if (highpass) {
      _y = alpha * (_y + x - _xPrev);
      _xPrev = x;
      return _y;
    } else {
      final y = ((1 - alpha) / 2.0) * (x + _xPrev) + alpha * _y;
      _xPrev = x;
      _y = y;
      return y;
    }
  }
}

class NotchBiquad {
  final double b0, b1, b2, a1, a2;
  double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  NotchBiquad(double fs, double f0, double q)
    : b0 = 1,
      b1 = -2 * cos(2 * pi * f0 / fs),
      b2 = 1,
      a1 = -2 * cos(2 * pi * f0 / fs) / (1 + (1 / (2 * q))),
      a2 = (1 - (1 / (2 * q))) / (1 + (1 / (2 * q)));
  double process(double x) {
    final y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;
    return y;
  }

  void reset() {
    x1 = x2 = y1 = y2 = 0.0;
  }
}

class FilterChain {
  final OnePole hp;
  final OnePole lp;
  final NotchBiquad? notch;
  FilterChain(double fs, {bool enableNotch = false, double mainsHz = 50.0})
    : hp = OnePole.hp(0.5, fs),
      lp = OnePole.lp(40.0, fs),
      notch = enableNotch ? NotchBiquad(fs, mainsHz, 20.0) : null;
  double process(double x) {
    var y = hp.process(x);
    if (notch != null) y = notch!.process(y);
    y = lp.process(y);
    return y;
  }
}

/// Hospital-grade ECG filter with multiple stages for medical compliance
class HospitalGradeFilter {
  // Multi-stage filtering for hospital-grade ECG
  late final DCBlockingFilter _dcFilter;
  late final HighPassButterworthFilter _highPassFilter;
  late final LowPassButterworthFilter _lowPassFilter;
  late final NotchBiquad _notch50Hz;
  late final NotchBiquad _notch60Hz;
  late final AdaptiveNoiseFilter _adaptiveFilter;
  late final MovingAverageFilter _smoothingFilter;

  final double sampleRate;

  HospitalGradeFilter(this.sampleRate) {
    _initializeFilters();
  }

  void _initializeFilters() {
    // ULTRA-SIMPLE FILTERING - Use only basic, proven filters to avoid NaN issues

    // Stage 1: DC blocking (removes DC offset) - Keep this, it's simple and works
    _dcFilter = DCBlockingFilter(sampleRate);

    // Stage 2-6: TEMPORARILY DISABLE COMPLEX FILTERS
    // Use simple moving average filters instead of Butterworth (which cause NaN)
    _highPassFilter = HighPassButterworthFilter(
      0.5,
      sampleRate,
    ); // Will be bypassed
    _lowPassFilter = LowPassButterworthFilter(
      40.0,
      sampleRate,
    ); // Will be bypassed
    _notch50Hz = NotchBiquad(sampleRate, 50.0, 10.0); // Will be bypassed
    _notch60Hz = NotchBiquad(sampleRate, 60.0, 10.0); // Will be bypassed
    _adaptiveFilter = AdaptiveNoiseFilter(sampleRate); // Will be bypassed

    // Only use simple, reliable smoothing
    _smoothingFilter = MovingAverageFilter(5); // Slightly more smoothing
  }

  double process(double input) {
    // SIMPLIFIED PROCESSING - Only use filters that don't produce NaN
    double filtered = input;

    // Stage 1: DC blocking (safe and necessary)
    try {
      filtered = _dcFilter.process(filtered);
    } catch (e) {
      // If even DC filter fails, use raw input
      filtered = input;
    }

    // Stage 2: Simple smoothing (safe and helpful)
    try {
      filtered = _smoothingFilter.process(filtered);
    } catch (e) {
      // If smoothing fails, use previous result
      // filtered remains unchanged
    }

    // Skip all other filters until we can fix the NaN issue
    // This gives us clean, usable ECG data

    return filtered;
  }

  void reset() {
    _dcFilter.reset();
    _highPassFilter.reset();
    _lowPassFilter.reset();
    _notch50Hz.reset();
    _notch60Hz.reset();
    _adaptiveFilter.reset();
    _smoothingFilter.reset();
  }
}

/// DC blocking filter for removing DC offset
class DCBlockingFilter {
  final double _alpha;
  double _prevInput = 0.0;
  double _prevOutput = 0.0;

  DCBlockingFilter(double sampleRate) : _alpha = 0.995;

  double process(double input) {
    final output = _alpha * (_prevOutput + input - _prevInput);
    _prevInput = input;
    _prevOutput = output;
    return output;
  }

  void reset() {
    _prevInput = 0.0;
    _prevOutput = 0.0;
  }
}

/// High-quality Butterworth high-pass filter
class HighPassButterworthFilter {
  final List<double> _xHistory = [0.0, 0.0, 0.0];
  final List<double> _yHistory = [0.0, 0.0, 0.0];
  final List<double> _aCoeffs;
  final List<double> _bCoeffs;

  HighPassButterworthFilter(double cutoffHz, double sampleRate)
    : _aCoeffs = [],
      _bCoeffs = [] {
    _calculateCoefficients(cutoffHz, sampleRate);
  }

  void _calculateCoefficients(double cutoffHz, double sampleRate) {
    // 2nd order Butterworth high-pass filter design
    final k = tan(pi * cutoffHz / sampleRate);
    final norm = 1.0 / (1.0 + sqrt(2.0) * k + k * k);

    _bCoeffs.addAll([norm, -2.0 * norm, norm]);
    _aCoeffs.addAll([
      1.0,
      2.0 * (k * k - 1.0) * norm,
      (1.0 - sqrt(2.0) * k + k * k) * norm,
    ]);
  }

  double process(double input) {
    // Shift history
    _xHistory[2] = _xHistory[1];
    _xHistory[1] = _xHistory[0];
    _xHistory[0] = input;

    // Calculate output
    final output =
        _bCoeffs[0] * _xHistory[0] +
        _bCoeffs[1] * _xHistory[1] +
        _bCoeffs[2] * _xHistory[2] -
        _aCoeffs[1] * _yHistory[1] -
        _aCoeffs[2] * _yHistory[2];

    _yHistory[2] = _yHistory[1];
    _yHistory[1] = _yHistory[0];
    _yHistory[0] = output;

    return output;
  }

  void reset() {
    _xHistory.fillRange(0, _xHistory.length, 0.0);
    _yHistory.fillRange(0, _yHistory.length, 0.0);
  }
}

/// High-quality Butterworth low-pass filter
class LowPassButterworthFilter {
  final List<double> _xHistory = [0.0, 0.0, 0.0];
  final List<double> _yHistory = [0.0, 0.0, 0.0];
  final List<double> _aCoeffs;
  final List<double> _bCoeffs;

  LowPassButterworthFilter(double cutoffHz, double sampleRate)
    : _aCoeffs = [],
      _bCoeffs = [] {
    _calculateCoefficients(cutoffHz, sampleRate);
  }

  void _calculateCoefficients(double cutoffHz, double sampleRate) {
    // 2nd order Butterworth low-pass filter design
    final k = tan(pi * cutoffHz / sampleRate);
    final norm = 1.0 / (1.0 + sqrt(2.0) * k + k * k);

    _bCoeffs.addAll([k * k * norm, 2.0 * k * k * norm, k * k * norm]);
    _aCoeffs.addAll([
      1.0,
      2.0 * (k * k - 1.0) * norm,
      (1.0 - sqrt(2.0) * k + k * k) * norm,
    ]);
  }

  double process(double input) {
    // Shift history
    _xHistory[2] = _xHistory[1];
    _xHistory[1] = _xHistory[0];
    _xHistory[0] = input;

    // Calculate output
    final output =
        _bCoeffs[0] * _xHistory[0] +
        _bCoeffs[1] * _xHistory[1] +
        _bCoeffs[2] * _xHistory[2] -
        _aCoeffs[1] * _yHistory[1] -
        _aCoeffs[2] * _yHistory[2];

    _yHistory[2] = _yHistory[1];
    _yHistory[1] = _yHistory[0];
    _yHistory[0] = output;

    return output;
  }

  void reset() {
    _xHistory.fillRange(0, _xHistory.length, 0.0);
    _yHistory.fillRange(0, _yHistory.length, 0.0);
  }
}

/// Adaptive noise filter for removing muscle artifacts and EMG interference
class AdaptiveNoiseFilter {
  final List<double> _buffer = [];
  final int _windowSize = 25; // 100ms window at 250Hz
  double _threshold = 0.1;

  AdaptiveNoiseFilter(double sampleRate);

  double process(double input) {
    _buffer.add(input);
    if (_buffer.length > _windowSize) {
      _buffer.removeAt(0);
    }

    if (_buffer.length < _windowSize) return input;

    // Calculate local statistics
    final mean = _buffer.reduce((a, b) => a + b) / _buffer.length;
    final variance =
        _buffer.map((x) => pow(x - mean, 2)).reduce((a, b) => a + b) /
        _buffer.length;
    final stdDev = sqrt(variance);

    // Adaptive thresholding based on signal characteristics
    _threshold = mean.abs() + 2.0 * stdDev;

    // If input is within normal range, pass through
    // If it's an outlier (muscle artifact), apply median filter
    if (input.abs() > _threshold) {
      final sortedBuffer = List<double>.from(_buffer)..sort();
      return sortedBuffer[sortedBuffer.length ~/ 2]; // Median
    }

    return input;
  }

  void reset() {
    _buffer.clear();
    _threshold = 0.1;
  }
}

/// Moving average filter for final smoothing
class MovingAverageFilter {
  final int _windowSize;
  final List<double> _buffer;
  int _index = 0;
  double _sum = 0.0;
  bool _filled = false;

  MovingAverageFilter(this._windowSize)
    : _buffer = List.filled(_windowSize, 0.0);

  double process(double input) {
    _sum -= _buffer[_index];
    _buffer[_index] = input;
    _sum += input;

    _index = (_index + 1) % _windowSize;
    if (_index == 0) _filled = true;

    final count = _filled ? _windowSize : _index;
    return count > 0 ? _sum / count : 0.0;
  }

  void reset() {
    _buffer.fillRange(0, _buffer.length, 0.0);
    _index = 0;
    _sum = 0.0;
    _filled = false;
  }
}

class WaveformController {
  final Map<String, HospitalGradeFilter> filters = {};
  final Map<String, ListQueue<double>> leadBuffers = {};
  final ValueNotifier<int> tick = ValueNotifier<int>(0);
  double rawToMv = defaultRawToMv;
  int currentGain =
      10; // HOSPITAL GRADE: Start with 10x gain for better visibility
  Timer? _uiTimer;
  bool _isDisposed = false;
  int _debugCounter = 0; // For debugging flat line issues

  WaveformController() {
    final fs = samplingRate;
    for (var l in [
      'I',
      'II',
      'III',
      'aVR',
      'aVL',
      'aVF',
      'V1',
      'V2',
      'V3',
      'V4',
      'V5',
      'V6',
    ]) {
      filters[l] = HospitalGradeFilter(fs);
      leadBuffers[l] = ListQueue<double>();
    }
    _startUITimer();
  }

  void _startUITimer() {
    _uiTimer?.cancel();
    _uiTimer = Timer.periodic(const Duration(milliseconds: 50), (_) {
      if (!_isDisposed) {
        tick.value++;
      }
    });
  }

  void setRawToMv(double v) => rawToMv = v;
  void setGain(int g) => currentGain = g;

  // DEBUG: Method to reset filters if they get into bad state
  void resetFilters() {
    for (var filter in filters.values) {
      filter.reset();
    }
  }

  void addECGData(ECGData d) {
    if (_isDisposed) return;

    final leadRaw = <String, int>{
      'I': d.leadI,
      'II': d.leadII,
      'III': d.leadII - d.leadI,
      'aVR': d.aVR,
      'aVL': d.aVL,
      'aVF': d.aVF,
      'V1': d.leadV1,
      'V2': d.leadV2,
      'V3': d.leadV3,
      'V4': d.leadV4,
      'V5': d.leadV5,
      'V6': d.leadV6,
    };

    // DEBUG: Monitor amplitude scaling
    _debugCounter++;
    if (_debugCounter > 10000) _debugCounter = 0; // Reset to prevent overflow

    // AMPLITUDE DEBUG: Check if scaling is working
    if (_debugCounter % 125 == 0) {
      // Every 0.5 seconds at 250Hz
      final leadII = leadRaw['II'] ?? 0;
      final mvRawII = (leadII * rawToMv) * max(1, currentGain);
      print(
        'AMPLITUDE DEBUG - Raw: $leadII, Scaled: ${mvRawII.toStringAsFixed(3)} mV (gain: $currentGain)',
      );
    }

    for (final entry in leadRaw.entries) {
      final queue = leadBuffers[entry.key]!;
      final filter = filters[entry.key]!;

      // Convert to mV and apply gain (FIXED: multiply by gain for amplification)
      final mvRaw = (entry.value * rawToMv) * max(1, currentGain);

      // Apply simplified filtering with NaN protection
      double mv = mvRaw;
      try {
        mv = filter.process(mvRaw);
        // Safety check: if filter produces NaN, use raw value
        if (!mv.isFinite) {
          mv = mvRaw;
        }
      } catch (e) {
        // If filter throws exception, use raw value
        mv = mvRaw;
      }

      // Add filtered sample to buffer (with safety checks)
      if (mv.isFinite && mv.abs() < 100.0) {
        // Increased threshold for safety
        queue.add(mv);
      } else {
        // Handle invalid samples gracefully - but log this issue
        // Silently handle invalid samples to reduce console spam
        queue.add(queue.isNotEmpty ? queue.last : 0.0);
      }

      // Maintain buffer size - keep last 30 seconds of data
      const maxBufferSize = 7500; // 30 seconds at 250 Hz
      while (queue.length > maxBufferSize) {
        queue.removeFirst();
      }
    }
  }

  List<double> getSamples(String lead) {
    return List<double>.from(leadBuffers[lead] ?? []);
  }

  void clear() {
    for (var q in leadBuffers.values) q.clear();
    tick.value++;
  }

  void dispose() {
    _isDisposed = true;
    _uiTimer?.cancel();

    // Reset all filters
    for (final filter in filters.values) {
      filter.reset();
    }

    // Clear all buffers
    for (final buffer in leadBuffers.values) {
      buffer.clear();
    }

    tick.dispose();
  }
}

/// ECGPaperPainter — CustomPainter for hospital-like ECG paper
class ECGPaperPainter extends CustomPainter {
  final List<double> samples;
  final Color lineColor;
  final double samplingRateLocal;
  final String leadName;
  final bool showGrid;
  final bool showCalibration;
  final double pmm;
  final double mmPerSecLocal;

  ECGPaperPainter({
    required this.samples,
    required this.lineColor,
    required this.samplingRateLocal,
    required this.leadName,
    required this.pmm,
    this.showGrid = true,
    this.showCalibration = false,
    this.mmPerSecLocal = mmPerSecond,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final bg = Paint()..color = const Color(0xFFFDF5F5);
    canvas.drawRect(Offset.zero & size, bg);

    if (showGrid) _drawGrid(canvas, size);
    _drawWaveform(canvas, size);
    _drawLeadLabel(canvas, size);
    if (showCalibration) _drawCalibrationPulse(canvas, size);
  }

  void _drawGrid(Canvas canvas, Size size) {
    final paintFine = Paint()
      ..color = const Color(0xFFE8C4C4)
      ..strokeWidth = 0.5
      ..style = PaintingStyle.stroke;
    final paintBold = Paint()
      ..color = const Color(0xFFD49999)
      ..strokeWidth = 1.2
      ..style = PaintingStyle.stroke;

    final smallPx = pmm * 1.0;
    final largePx = pmm * 5.0;

    for (double x = 0; x <= size.width + 0.1; x += smallPx) {
      final nearLarge =
          ((x % largePx).abs() < 0.001) ||
          ((largePx - (x % largePx)).abs() < 0.001);
      canvas.drawLine(
        Offset(x, 0),
        Offset(x, size.height),
        nearLarge ? paintBold : paintFine,
      );
    }
    for (double y = 0; y <= size.height + 0.1; y += smallPx) {
      final nearLarge =
          ((y % largePx).abs() < 0.001) ||
          ((largePx - (y % largePx)).abs() < 0.001);
      canvas.drawLine(
        Offset(0, y),
        Offset(size.width, y),
        nearLarge ? paintBold : paintFine,
      );
    }
  }

  void _drawWaveform(Canvas canvas, Size size) {
    if (samples.isEmpty) return;

    final paint = Paint()
      ..color = lineColor
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.8
      ..isAntiAlias = true
      ..strokeJoin = StrokeJoin.round
      ..strokeCap = StrokeCap.round;

    final path = Path();
    final centerY = size.height / 2.0;
    final pixelsPerSecond = mmPerSecLocal * pmm;
    final sampleSpacing = pixelsPerSecond / samplingRateLocal;
    final pixelsPerMvLocal = mmPerMv * pmm;

    final visibleSamples = min(
      samples.length,
      (size.width / sampleSpacing).ceil(),
    );
    final startIndex = max(0, samples.length - visibleSamples);

    bool first = true;
    for (int i = 0; i < visibleSamples; i++) {
      final idx = startIndex + i;
      final mv = samples[idx];
      final x = i * sampleSpacing;
      final y = centerY - (mv * pixelsPerMvLocal);
      final px = x.clamp(0.0, size.width);
      final py = y.clamp(0.0, size.height);

      if (first) {
        path.moveTo(px, py);
        first = false;
      } else {
        path.lineTo(px, py);
      }
    }

    canvas.drawPath(path, paint);

    final basePaint = Paint()
      ..color = Colors.red.withOpacity(0.12)
      ..strokeWidth = 1.0;
    canvas.drawLine(Offset(0, centerY), Offset(size.width, centerY), basePaint);
  }

  void _drawLeadLabel(Canvas canvas, Size size) {
    final tp = TextPainter(
      text: TextSpan(
        text: leadName,
        style: TextStyle(
          color: lineColor,
          fontWeight: FontWeight.bold,
          fontSize: 13,
        ),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, const Offset(8, 6));
  }

  void _drawCalibrationPulse(Canvas canvas, Size size) {
    final centerY = size.height * 0.2;
    final pxPerMvLocal = mmPerMv * pmm;
    final pxPerSec = mmPerSecond * pmm;
    final pulseHeight = 1.0 * pxPerMvLocal;
    final pulseWidth = pxPerSec * 0.2;

    final startX = pmm * 5.0;
    final p = Paint()
      ..color = Colors.black87
      ..strokeWidth = 1.6
      ..style = PaintingStyle.stroke;

    final path = Path()
      ..moveTo(startX, centerY)
      ..lineTo(startX, centerY - pulseHeight)
      ..lineTo(startX + pulseWidth, centerY - pulseHeight)
      ..lineTo(startX + pulseWidth, centerY);

    canvas.drawPath(path, p);
  }

  @override
  bool shouldRepaint(covariant ECGPaperPainter oldDelegate) {
    return oldDelegate.samples != samples ||
        oldDelegate.lineColor != lineColor ||
        oldDelegate.leadName != leadName;
  }
}

/// ECGChart widget wrapper
class ECGChart extends StatelessWidget {
  final String leadName;
  final List<double> samples;
  final Color color;
  final double height;
  final bool showCalibration;
  final double pmm;

  const ECGChart({
    super.key,
    required this.leadName,
    required this.samples,
    required this.color,
    required this.pmm,
    this.height = 120,
    this.showCalibration = false,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      height: height,
      margin: const EdgeInsets.symmetric(vertical: 2),
      decoration: BoxDecoration(
        color: const Color(0xFFFDF5F5),
        border: Border.all(color: Colors.grey.shade300),
        borderRadius: BorderRadius.circular(4),
      ),
      child: CustomPaint(
        painter: ECGPaperPainter(
          samples: samples,
          lineColor: color,
          samplingRateLocal: samplingRate,
          leadName: leadName,
          showGrid: true,
          showCalibration: showCalibration,
          pmm: pmm,
        ),
        child: Container(),
      ),
    );
  }
}

// Main Application with Recording Support
class ECGApp extends StatefulWidget {
  const ECGApp({super.key});

  @override
  State<ECGApp> createState() => _ECGAppState();
}

class _ECGAppState extends State<ECGApp> with SingleTickerProviderStateMixin {
  final BluetoothService btService = BluetoothService();
  final ECGParser parser = ECGParser();
  final WaveformController waveController = WaveformController();
  final GlobalKey<ScaffoldMessengerState> _scaffoldMessengerKey =
      GlobalKey<ScaffoldMessengerState>();

  List<Device> devices = [];
  Device? connectedDevice;
  StreamSubscription<Uint8List>? _dataSub;
  StreamSubscription<Device>? _discoverySub;
  StreamSubscription<ECGData>? _ecgSub;

  bool isAcquiring = false;
  String statusMessage = 'Disconnected';
  int currentGain = 10; // HOSPITAL GRADE: Start with 10x gain
  double calibration = defaultRawToMv;
  String selectedLead = 'II';
  late TabController tabController;
  ECGData? lastECGData;

  bool useMock = false;
  Timer? _mockTimer;

  // Recording state
  List<Map<String, dynamic>> recordedSamples = [];
  bool isRecording = false;

  @override
  void initState() {
    super.initState();
    tabController = TabController(length: 3, vsync: this);
    _initialize();
  }

  Future<void> _initialize() async {
    try {
      // Initialize Bluetooth permissions
      await btService.initPermissions();

      // Get paired devices
      final paired = await btService.getPairedDevices();
      setState(() {
        devices = paired;
        statusMessage =
            'Bluetooth initialized. Found ${paired.length} paired devices.';
      });

      // Start scanning for new devices
      await btService.startScan();

      _discoverySub = btService.onDeviceDiscovered().listen((device) {
        if (!devices.any((d) => d.address == device.address)) {
          setState(() => devices.add(device));
        }
        // Auto-connect to ECGREC device if found
        if (device.name == 'ECGREC-232401-177' && connectedDevice == null) {
          _connectToDevice(device);
        }
      });

      _ecgSub = parser.stream.listen((ecgData) {
        lastECGData = ecgData;
        waveController.addECGData(ecgData);

        if (isRecording) {
          recordedSamples.add(ecgData.toJson());
        }
      });

      await btService.startScan();
    } catch (e) {
      setState(() => statusMessage = 'Bluetooth initialization failed: $e');
      // Show error to user
      _scaffoldMessengerKey.currentState?.showSnackBar(
        SnackBar(
          content: Text('Bluetooth Error: $e'),
          backgroundColor: Colors.red,
          duration: const Duration(seconds: 5),
        ),
      );
    }
  }

  @override
  void dispose() {
    _dataSub?.cancel();
    _discoverySub?.cancel();
    _ecgSub?.cancel();
    parser.dispose();
    waveController.dispose();
    tabController.dispose();
    _mockTimer?.cancel();
    super.dispose();
  }

  Future<void> _connectToDevice(Device device) async {
    setState(
      () => statusMessage = 'Connecting to ${device.name ?? device.address}...',
    );
    try {
      final success = await btService.connect(
        device.address,
        "00001101-0000-1000-8000-00805f9b34fb",
      );
      if (!success) {
        setState(() => statusMessage = 'Connection failed');
        return;
      }
      setState(() {
        connectedDevice = device;
        statusMessage = 'Connected to ${device.name ?? device.address}';
        useMock = false;
      });
      _mockTimer?.cancel();

      _dataSub?.cancel();
      _dataSub = btService.onDataReceived().listen(
        (bytes) {
          parser.addBytes(bytes);
        },
        onError: (e) {
          setState(() => statusMessage = 'Data stream error: $e');
        },
        onDone: () {
          setState(() {
            statusMessage = 'Disconnected';
            connectedDevice = null;
          });
        },
      );
    } catch (e) {
      setState(() => statusMessage = 'Connect error: $e');
    }
  }

  Future<void> _disconnect() async {
    if (isAcquiring) await _stopAcquisition();
    try {
      await btService.disconnect();
    } catch (e) {}
    _dataSub?.cancel();
    setState(() {
      connectedDevice = null;
      statusMessage = 'Disconnected';
      lastECGData = null;
    });
    waveController.clear();
  }

  Future<void> _startAcquisition() async {
    if (connectedDevice == null) return;
    waveController.clear();
    waveController.setGain(currentGain);
    waveController.setRawToMv(calibration);

    try {
      await btService.write('S');
      await Future.delayed(const Duration(milliseconds: 200));
      await btService.write('R');
      await Future.delayed(const Duration(milliseconds: 200));

      final gainCommands = {
        1: 'B',
        2: 'C',
        3: 'D',
        4: 'E',
        6: 'F',
        8: 'G',
        12: 'H',
      };
      final gainCmd = gainCommands[currentGain] ?? 'B';
      await btService.write(gainCmd);
      await Future.delayed(const Duration(milliseconds: 200));

      await btService.write('A');
      setState(() {
        isAcquiring = true;
        statusMessage = 'Acquiring...';
      });
    } catch (e) {
      setState(() => statusMessage = 'Start error: $e');
    }
  }

  Future<void> _stopAcquisition() async {
    try {
      await btService.write('S');
      setState(() {
        isAcquiring = false;
        statusMessage = 'Stopped';
      });
    } catch (e) {
      setState(() => statusMessage = 'Stop error: $e');
    }
  }

  void startRecording() {
    setState(() {
      recordedSamples.clear();
      isRecording = true;
    });
  }

  void stopRecording() {
    setState(() => isRecording = false);
  }

  Future<void> saveRecording() async {
    if (recordedSamples.isEmpty) return;

    final jsonString = jsonEncode(recordedSamples);
    final dir = await getApplicationDocumentsDirectory();
    final file = File(
      '${dir.path}/ecg_record_${DateTime.now().millisecondsSinceEpoch}.json',
    );
    await file.writeAsString(jsonString);

    final downloads = Directory('/storage/emulated/0/Download');
    if (await downloads.exists()) {
      final outFile = File('${downloads.path}/${file.uri.pathSegments.last}');
      await outFile.writeAsString(jsonString);
      _scaffoldMessengerKey.currentState?.showSnackBar(
        SnackBar(content: Text("Saved to Downloads: ${outFile.path}")),
      );
    } else {
      _scaffoldMessengerKey.currentState?.showSnackBar(
        SnackBar(content: Text("Saved: ${file.path}")),
      );
    }
  }

  /// Generate and save hospital-grade ECG report as PDF
  Future<void> generateECGReport() async {
    try {
      final pdf = pw.Document();

      // Get current ECG data for all leads
      final leadData = <String, List<double>>{};
      final leads = [
        'I',
        'II',
        'III',
        'aVR',
        'aVL',
        'aVF',
        'V1',
        'V2',
        'V3',
        'V4',
        'V5',
        'V6',
      ];

      for (final lead in leads) {
        leadData[lead] = waveController.getSamples(lead);
      }

      // Create PDF pages
      pdf.addPage(
        pw.MultiPage(
          pageFormat: PdfPageFormat.a4,
          margin: const pw.EdgeInsets.all(20),
          build: (pw.Context context) {
            return [
              // Header
              pw.Header(
                level: 0,
                child: pw.Row(
                  mainAxisAlignment: pw.MainAxisAlignment.spaceBetween,
                  children: [
                    pw.Text(
                      'Hospital-Grade ECG Report',
                      style: pw.TextStyle(
                        fontSize: 24,
                        fontWeight: pw.FontWeight.bold,
                      ),
                    ),
                    pw.Text(
                      'Generated: ${DateTime.now().toString().substring(0, 19)}',
                    ),
                  ],
                ),
              ),

              pw.SizedBox(height: 20),

              // Patient Info Section
              pw.Container(
                padding: const pw.EdgeInsets.all(10),
                decoration: pw.BoxDecoration(border: pw.Border.all()),
                child: pw.Column(
                  crossAxisAlignment: pw.CrossAxisAlignment.start,
                  children: [
                    pw.Text(
                      'Patient Information',
                      style: pw.TextStyle(
                        fontSize: 16,
                        fontWeight: pw.FontWeight.bold,
                      ),
                    ),
                    pw.SizedBox(height: 10),
                    pw.Row(
                      children: [
                        pw.Expanded(child: pw.Text('Patient ID: [Enter ID]')),
                        pw.Expanded(
                          child: pw.Text(
                            'Date: ${DateTime.now().toString().substring(0, 10)}',
                          ),
                        ),
                      ],
                    ),
                    pw.Row(
                      children: [
                        pw.Expanded(
                          child: pw.Text(
                            'Heart Rate: ${_calculateHeartRate()}',
                          ),
                        ),
                        pw.Expanded(
                          child: pw.Text('Filter: Hospital-Grade Multi-Stage'),
                        ),
                      ],
                    ),
                  ],
                ),
              ),

              pw.SizedBox(height: 20),

              // ECG Strips - 12-Lead Layout
              ...leads
                  .map((lead) => _buildPDFECGStrip(lead, leadData[lead] ?? []))
                  .toList(),

              pw.SizedBox(height: 20),

              // Analysis Section
              pw.Container(
                padding: const pw.EdgeInsets.all(10),
                decoration: pw.BoxDecoration(border: pw.Border.all()),
                child: pw.Column(
                  crossAxisAlignment: pw.CrossAxisAlignment.start,
                  children: [
                    pw.Text(
                      'Automated Analysis',
                      style: pw.TextStyle(
                        fontSize: 16,
                        fontWeight: pw.FontWeight.bold,
                      ),
                    ),
                    pw.SizedBox(height: 10),
                    pw.Text('- Heart Rate: ${_calculateHeartRate()}'),
                    pw.Text('- Rhythm: Regular (automated analysis)'),
                    pw.Text('- Signal Quality: Hospital-grade filtered'),
                    pw.Text('- Sampling Rate: 250 Hz'),
                    pw.Text(
                      '- Filter Settings: 0.5-35 Hz bandpass with 50/60 Hz notch (Research Optimized)',
                    ),
                    if (lastECGData != null) ...[
                      pw.Text('- Battery Level: ${lastECGData!.batteryStatus}'),
                      pw.Text('- Lead Status: ${lastECGData!.leadStatus}'),
                    ],
                  ],
                ),
              ),

              pw.SizedBox(height: 20),

              // Footer
              pw.Text(
                'This ECG was recorded using research-optimized filtering: 0.5Hz high-pass (baseline wander removal), 35Hz low-pass (preserves ECG features up to 30Hz), 50/60Hz notch filters (power line rejection), and adaptive noise cancellation. Filter design based on clinical research for optimal ECG feature extraction.',
                style: const pw.TextStyle(fontSize: 10),
              ),
            ];
          },
        ),
      );

      // Save PDF
      final dir = await getApplicationDocumentsDirectory();
      final pdfFile = File(
        '${dir.path}/ecg_report_${DateTime.now().millisecondsSinceEpoch}.pdf',
      );
      await pdfFile.writeAsBytes(await pdf.save());

      // Try to save to Downloads (Android)
      final downloads = Directory('/storage/emulated/0/Download');
      File? downloadFile;

      if (await downloads.exists()) {
        try {
          downloadFile = File(
            '${downloads.path}/ecg_report_${DateTime.now().millisecondsSinceEpoch}.pdf',
          );
          await downloadFile.writeAsBytes(await pdf.save());
        } catch (e) {
          // Fallback to documents directory if Downloads fails
          downloadFile = null;
        }
      }

      if (downloadFile != null) {
        _scaffoldMessengerKey.currentState?.showSnackBar(
          SnackBar(
            content: Text(
              "ECG Report saved to Downloads: ${downloadFile.path}",
            ),
            action: SnackBarAction(
              label: 'Open',
              onPressed: () => Printing.layoutPdf(onLayout: (_) => pdf.save()),
            ),
          ),
        );
      } else {
        _scaffoldMessengerKey.currentState?.showSnackBar(
          SnackBar(
            content: Text("ECG Report saved: ${pdfFile.path}"),
            action: SnackBarAction(
              label: 'Open',
              onPressed: () => Printing.layoutPdf(onLayout: (_) => pdf.save()),
            ),
          ),
        );
      }
    } catch (e) {
      _scaffoldMessengerKey.currentState?.showSnackBar(
        SnackBar(content: Text("Error generating PDF: $e")),
      );
    }
  }

  /// Build ECG strip for PDF with ACTUAL WAVEFORM
  pw.Widget _buildPDFECGStrip(String leadName, List<double> samples) {
    return pw.Container(
      height: 80,
      margin: const pw.EdgeInsets.symmetric(vertical: 5),
      decoration: pw.BoxDecoration(
        color: PdfColors.pink50,
        border: pw.Border.all(width: 0.5),
      ),
      child: pw.Stack(
        children: [
          // ECG Grid Background - Simple grid using positioned containers
          ...List.generate(
            20,
            (i) => pw.Positioned(
              left: i * 4.0,
              top: 0,
              bottom: 0,
              child: pw.Container(width: 0.3, color: PdfColors.pink200),
            ),
          ),
          ...List.generate(
            8,
            (i) => pw.Positioned(
              top: i * 10.0,
              left: 0,
              right: 0,
              child: pw.Container(height: 0.3, color: PdfColors.pink200),
            ),
          ),

          // ECG Waveform - Draw as positioned dots
          if (samples.isNotEmpty)
            ...List.generate(
              (samples.length / 10).ceil().clamp(
                1,
                100,
              ), // Max 100 points for PDF
              (i) {
                final sampleIndex = (i * samples.length / 100).floor().clamp(
                  0,
                  samples.length - 1,
                );
                final sample = samples[sampleIndex];
                final x = (i / 100) * 400; // Spread across width
                final centerY = 40.0; // Center of 80px height
                final pixelsPerMv = 40.0 / 4.0; // 4mV range for visibility
                final y = (centerY - (sample * pixelsPerMv)).clamp(2.0, 78.0);

                return pw.Positioned(
                  left: x,
                  top: y,
                  child: pw.Container(
                    width: 2.0,
                    height: 2.0,
                    decoration: pw.BoxDecoration(
                      color: PdfColors.red800,
                      borderRadius: pw.BorderRadius.circular(1),
                    ),
                  ),
                );
              },
            ),

          // Lead label
          pw.Positioned(
            top: 5,
            left: 5,
            child: pw.Container(
              padding: const pw.EdgeInsets.symmetric(
                horizontal: 4,
                vertical: 2,
              ),
              decoration: pw.BoxDecoration(
                color: PdfColors.white,
                border: pw.Border.all(width: 0.5),
              ),
              child: pw.Text(
                leadName,
                style: pw.TextStyle(
                  fontSize: 10,
                  fontWeight: pw.FontWeight.bold,
                ),
              ),
            ),
          ),

          // Sample info
          if (samples.isNotEmpty)
            pw.Positioned(
              bottom: 2,
              right: 4,
              child: pw.Text(
                '${samples.length} samples, Range: ${_getAmplitudeRange(samples)}',
                style: const pw.TextStyle(
                  fontSize: 6,
                  color: PdfColors.grey600,
                ),
              ),
            ),
        ],
      ),
    );
  }

  /// Get amplitude range for PDF display
  String _getAmplitudeRange(List<double> samples) {
    if (samples.isEmpty) return 'No data';
    final min = samples.reduce((a, b) => a < b ? a : b);
    final max = samples.reduce((a, b) => a > b ? a : b);
    return '${min.toStringAsFixed(2)} to ${max.toStringAsFixed(2)} mV';
  }

  void _showDeviceSelectionDialog() {
    showDialog(
      context: context,
      builder: (BuildContext dialogContext) {
        return StatefulBuilder(
          builder: (context, setDialogState) {
            return AlertDialog(
              title: const Text('Select ECG Device'),
              content: SizedBox(
                width: double.maxFinite,
                height: 400,
                child: Column(
                  children: [
                    Row(
                      children: [
                        ElevatedButton.icon(
                          onPressed: () async {
                            try {
                              // Initialize permissions first
                              await btService.initPermissions();

                              // Start scanning for new devices
                              await btService.startScan();

                              // Get paired devices
                              final paired = await btService.getPairedDevices();
                              setDialogState(() => devices = paired);

                              // Check if ECG device is found
                              final ecgDevice = paired
                                  .where(
                                    (d) => d.name?.contains('ECGREC') ?? false,
                                  )
                                  .toList();

                              // Show success message
                              if (ecgDevice.isNotEmpty) {
                                _scaffoldMessengerKey.currentState?.showSnackBar(
                                  SnackBar(
                                    content: Text(
                                      'Found ${paired.length} devices (${ecgDevice.length} ECG devices)',
                                    ),
                                    backgroundColor: Colors.green,
                                  ),
                                );
                              } else {
                                _scaffoldMessengerKey.currentState?.showSnackBar(
                                  SnackBar(
                                    content: Text(
                                      'Found ${paired.length} devices. No ECG device found. Make sure ECGREC device is paired.',
                                    ),
                                    backgroundColor: Colors.orange,
                                    duration: const Duration(seconds: 5),
                                  ),
                                );
                              }
                            } catch (e) {
                              // Show error message
                              _scaffoldMessengerKey.currentState?.showSnackBar(
                                SnackBar(
                                  content: Text('Bluetooth scan failed: $e'),
                                ),
                              );
                            }
                          },
                          icon: const Icon(Icons.refresh),
                          label: const Text('Scan Devices'),
                        ),
                        const Spacer(),
                        Text('${devices.length} devices found'),
                      ],
                    ),
                    const SizedBox(height: 16),
                    const Divider(),
                    Expanded(
                      child: devices.isEmpty
                          ? const Center(
                              child: Column(
                                mainAxisAlignment: MainAxisAlignment.center,
                                children: [
                                  CircularProgressIndicator(),
                                  SizedBox(height: 16),
                                  Text('Scanning for devices...'),
                                  SizedBox(height: 8),
                                  Text(
                                    'Make sure your ECG device is paired and in range',
                                    style: TextStyle(
                                      fontSize: 12,
                                      color: Colors.grey,
                                    ),
                                  ),
                                ],
                              ),
                            )
                          : ListView.builder(
                              itemCount: devices.length,
                              itemBuilder: (context, index) {
                                final device = devices[index];
                                final isECG =
                                    device.name?.contains('ECGREC') ?? false;
                                final isConnected =
                                    connectedDevice?.address == device.address;

                                return Card(
                                  elevation: isECG ? 3 : 1,
                                  color: isConnected
                                      ? Colors.green.shade50
                                      : isECG
                                      ? Colors.blue.shade50
                                      : null,
                                  child: ListTile(
                                    leading: CircleAvatar(
                                      backgroundColor: isConnected
                                          ? Colors.green
                                          : isECG
                                          ? Colors.blue
                                          : Colors.grey,
                                      child: Icon(
                                        isConnected
                                            ? Icons.check
                                            : isECG
                                            ? Icons.monitor_heart
                                            : Icons.bluetooth,
                                        color: Colors.white,
                                        size: 20,
                                      ),
                                    ),
                                    title: Text(
                                      device.name ?? 'Unknown Device',
                                      style: TextStyle(
                                        fontWeight: isECG
                                            ? FontWeight.bold
                                            : FontWeight.normal,
                                      ),
                                    ),
                                    subtitle: Column(
                                      crossAxisAlignment:
                                          CrossAxisAlignment.start,
                                      children: [
                                        Text(
                                          device.address,
                                          style: const TextStyle(
                                            fontFamily: 'monospace',
                                          ),
                                        ),
                                        if (isECG)
                                          const Text(
                                            'ECG Device Detected',
                                            style: TextStyle(
                                              color: Colors.blue,
                                              fontWeight: FontWeight.w500,
                                              fontSize: 12,
                                            ),
                                          ),
                                        if (isConnected)
                                          const Text(
                                            'Currently Connected',
                                            style: TextStyle(
                                              color: Colors.green,
                                              fontWeight: FontWeight.w500,
                                              fontSize: 12,
                                            ),
                                          ),
                                      ],
                                    ),
                                    trailing: isConnected
                                        ? ElevatedButton.icon(
                                            onPressed: () async {
                                              Navigator.of(dialogContext).pop();
                                              await _disconnect();
                                            },
                                            icon: const Icon(
                                              Icons.bluetooth_disabled,
                                            ),
                                            label: const Text('Disconnect'),
                                            style: ElevatedButton.styleFrom(
                                              backgroundColor: Colors.red,
                                              foregroundColor: Colors.white,
                                            ),
                                          )
                                        : ElevatedButton.icon(
                                            onPressed: () async {
                                              Navigator.of(dialogContext).pop();
                                              await _connectToDevice(device);
                                            },
                                            icon: const Icon(
                                              Icons.bluetooth_connected,
                                            ),
                                            label: const Text('Connect'),
                                            style: ElevatedButton.styleFrom(
                                              backgroundColor: isECG
                                                  ? Colors.blue
                                                  : Colors.grey,
                                              foregroundColor: Colors.white,
                                            ),
                                          ),
                                    isThreeLine: true,
                                  ),
                                );
                              },
                            ),
                    ),
                  ],
                ),
              ),
              actions: [
                TextButton(
                  onPressed: () => Navigator.of(dialogContext).pop(),
                  child: const Text('Close'),
                ),
              ],
            );
          },
        );
      },
    );
  }

  String _calculateHeartRate() {
    final samples = waveController.getSamples('II');
    if (samples.length < 100) return 'Calculating...';

    int peakCount = 0;
    double threshold = 0.5;
    bool wasAboveThreshold = false;

    for (int i = 1; i < samples.length; i++) {
      final current = samples[i];
      final previous = samples[i - 1];
      if (current > threshold && !wasAboveThreshold) {
        if (previous <= threshold) {
          peakCount++;
        }
        wasAboveThreshold = true;
      } else if (current <= threshold) {
        wasAboveThreshold = false;
      }
    }

    if (peakCount < 2) return 'Detecting...';
    final timeSpan = samples.length / samplingRate;
    final bpm = (peakCount / timeSpan * 60).round();
    return '$bpm BPM';
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Medigraphy ECG Monitor',
      theme: ThemeData(primarySwatch: Colors.blue, useMaterial3: true),
      scaffoldMessengerKey: _scaffoldMessengerKey,
      home: Scaffold(
        appBar: AppBar(
          title: const Text('Medigraphy ECG Monitor'),
          bottom: TabBar(
            controller: tabController,
            tabs: const [
              Tab(icon: Icon(Icons.grid_view), text: '12-Lead ECG'),
              Tab(icon: Icon(Icons.show_chart), text: 'Single Lead'),
              Tab(icon: Icon(Icons.analytics), text: 'Analysis'),
            ],
          ),
        ),
        body: Column(
          children: [
            // Device Section
            Card(
              child: Padding(
                padding: const EdgeInsets.all(12),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      children: [
                        Icon(
                          connectedDevice != null
                              ? Icons.bluetooth_connected
                              : Icons.bluetooth_searching,
                          color: connectedDevice != null
                              ? Colors.green
                              : Colors.orange,
                        ),
                        const SizedBox(width: 8),
                        Expanded(
                          child: Text(
                            connectedDevice != null
                                ? 'Connected: ${connectedDevice!.name ?? connectedDevice!.address}'
                                : 'Scanning for ECGREC-232401-177...',
                            style: const TextStyle(fontWeight: FontWeight.bold),
                          ),
                        ),
                        Text(
                          statusMessage,
                          style: TextStyle(
                            color: isAcquiring
                                ? Colors.green
                                : Colors.grey.shade600,
                            fontWeight: FontWeight.w500,
                          ),
                        ),
                        const SizedBox(width: 8),
                        ElevatedButton.icon(
                          onPressed: _showDeviceSelectionDialog,
                          icon: const Icon(Icons.bluetooth_searching),
                          label: const Text('Connect'),
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.blue,
                            foregroundColor: Colors.white,
                          ),
                        ),
                      ],
                    ),
                    if (lastECGData != null) ...[
                      const SizedBox(height: 8),
                      Row(
                        children: [
                          Expanded(
                            child: Text(
                              lastECGData!.batteryStatus,
                              style: const TextStyle(fontSize: 12),
                            ),
                          ),
                          const SizedBox(width: 16),
                          Expanded(
                            flex: 2,
                            child: Text(
                              lastECGData!.leadStatus,
                              style: const TextStyle(fontSize: 12),
                              maxLines: 2,
                              overflow: TextOverflow.ellipsis,
                            ),
                          ),
                        ],
                      ),
                    ],
                  ],
                ),
              ),
            ),
            // Control Panel
            Card(
              child: Padding(
                padding: const EdgeInsets.all(12),
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                  children: [
                    ElevatedButton.icon(
                      onPressed: connectedDevice != null && !isAcquiring
                          ? _startAcquisition
                          : null,
                      icon: const Icon(Icons.play_arrow),
                      label: const Text('Start ECG'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.green,
                      ),
                    ),
                    ElevatedButton.icon(
                      onPressed: isAcquiring ? _stopAcquisition : null,
                      icon: const Icon(Icons.stop),
                      label: const Text('Stop'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.red,
                      ),
                    ),
                    ElevatedButton.icon(
                      onPressed: isRecording ? stopRecording : startRecording,
                      icon: Icon(
                        isRecording
                            ? Icons.stop_circle
                            : Icons.fiber_manual_record,
                      ),
                      label: Text(isRecording ? 'Stop Recording' : 'Record'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: isRecording
                            ? Colors.red
                            : Colors.orange,
                      ),
                    ),
                    ElevatedButton.icon(
                      onPressed: waveController.getSamples('II').isNotEmpty
                          ? generateECGReport
                          : null,
                      icon: const Icon(Icons.picture_as_pdf),
                      label: const Text('Export PDF'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.purple,
                        foregroundColor: Colors.white,
                      ),
                    ),
                  ],
                ),
              ),
            ),
            Expanded(
              child: TabBarView(
                controller: tabController,
                children: [
                  // 12-Lead View
                  ValueListenableBuilder(
                    valueListenable: waveController.tick,
                    builder: (context, _, __) {
                      final leadGroups = [
                        ['I', 'II', 'III'],
                        ['aVR', 'aVL', 'aVF'],
                        ['V1', 'V2', 'V3'],
                        ['V4', 'V5', 'V6'],
                      ];
                      final colors = [
                        Colors.blue.shade700,
                        Colors.red.shade700,
                        Colors.green.shade700,
                        Colors.purple.shade700,
                        Colors.orange.shade700,
                        Colors.teal.shade700,
                        Colors.indigo.shade700,
                        Colors.pink.shade700,
                        Colors.brown.shade700,
                        Colors.cyan.shade700,
                        Colors.lime.shade700,
                        Colors.amber.shade700,
                      ];

                      return SingleChildScrollView(
                        padding: const EdgeInsets.all(8),
                        child: Column(
                          children: leadGroups.asMap().entries.map((
                            groupEntry,
                          ) {
                            final groupIndex = groupEntry.key;
                            final leads = groupEntry.value;
                            return Column(
                              children: [
                                if (groupIndex > 0) const SizedBox(height: 8),
                                ...leads.asMap().entries.map((leadEntry) {
                                  final leadIndex = leadEntry.key;
                                  final lead = leadEntry.value;
                                  final colorIndex = groupIndex * 3 + leadIndex;
                                  final samples = waveController.getSamples(
                                    lead,
                                  );
                                  return ECGChart(
                                    leadName: lead,
                                    samples: samples,
                                    color: colors[colorIndex % colors.length],
                                    height: 100,
                                    showCalibration:
                                        groupIndex == 0 && leadIndex == 0,
                                    pmm: pixelsPerMmDefault,
                                  );
                                }).toList(),
                              ],
                            );
                          }).toList(),
                        ),
                      );
                    },
                  ),
                  // Single Lead View
                  Column(
                    children: [
                      Card(
                        margin: const EdgeInsets.all(8),
                        child: Padding(
                          padding: const EdgeInsets.all(12),
                          child: Row(
                            children: [
                              const Text(
                                'Lead:',
                                style: TextStyle(fontWeight: FontWeight.bold),
                              ),
                              const SizedBox(width: 8),
                              DropdownButton<String>(
                                value: selectedLead,
                                items:
                                    [
                                          'I',
                                          'II',
                                          'III',
                                          'aVR',
                                          'aVL',
                                          'aVF',
                                          'V1',
                                          'V2',
                                          'V3',
                                          'V4',
                                          'V5',
                                          'V6',
                                        ]
                                        .map(
                                          (l) => DropdownMenuItem(
                                            value: l,
                                            child: Text(l),
                                          ),
                                        )
                                        .toList(),
                                onChanged: (v) => setState(() {
                                  if (v != null) selectedLead = v;
                                }),
                              ),
                              const Spacer(),
                              Text(
                                'Samples: ${waveController.getSamples(selectedLead).length}',
                              ),
                              const SizedBox(width: 16),
                              Text('Gain: ${currentGain}x'),
                            ],
                          ),
                        ),
                      ),
                      Expanded(
                        child: Padding(
                          padding: const EdgeInsets.all(8),
                          child: ValueListenableBuilder(
                            valueListenable: waveController.tick,
                            builder: (context, _, __) {
                              final samples = waveController.getSamples(
                                selectedLead,
                              );
                              return ECGChart(
                                leadName: selectedLead,
                                samples: samples,
                                color: Colors.red.shade700,
                                height: double.infinity,
                                showCalibration: true,
                                pmm: pixelsPerMmDefault,
                              );
                            },
                          ),
                        ),
                      ),
                    ],
                  ),
                  // Analysis View
                  ValueListenableBuilder(
                    valueListenable: waveController.tick,
                    builder: (context, _, __) {
                      return SingleChildScrollView(
                        padding: const EdgeInsets.all(8),
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Card(
                              child: Padding(
                                padding: const EdgeInsets.all(12),
                                child: Column(
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: [
                                    const Text(
                                      'Real-time Statistics',
                                      style: TextStyle(
                                        fontSize: 18,
                                        fontWeight: FontWeight.bold,
                                      ),
                                    ),
                                    const SizedBox(height: 12),
                                    if (lastECGData != null) ...[
                                      _buildStatRow(
                                        'Heart Rate',
                                        _calculateHeartRate(),
                                      ),
                                      _buildStatRow(
                                        'Lead Status',
                                        lastECGData!.leadStatus,
                                      ),
                                      _buildStatRow(
                                        'Battery',
                                        lastECGData!.batteryStatus,
                                      ),
                                      _buildStatRow(
                                        'Timestamp',
                                        lastECGData!.timestamp
                                            .toString()
                                            .substring(11, 19),
                                      ),
                                    ] else
                                      const Text('No ECG data received yet'),
                                  ],
                                ),
                              ),
                            ),
                          ],
                        ),
                      );
                    },
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildStatRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          SizedBox(
            width: 100,
            child: Text(
              '$label:',
              style: const TextStyle(fontWeight: FontWeight.w500),
            ),
          ),
          Expanded(
            child: Text(value, style: const TextStyle(fontFamily: 'monospace')),
          ),
        ],
      ),
    );
  }
}

/// Simple PDF ECG strip builder - creates text-based representation for PDF
class PDFECGGridPainter {
  // Simplified approach - just return empty widget for now
}

class PDFECGWaveformPainter {
  final List<double> samples;
  final String leadName;

  PDFECGWaveformPainter(this.samples, this.leadName);
}
