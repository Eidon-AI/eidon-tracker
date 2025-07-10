# Troubleshooting Guide

## Firmware Issues

### Device Won't Start / LED Flashing Rapidly

**Symptoms:**
- LED flashes rapidly on startup
- Device doesn't appear in Bluetooth scan

**Causes:**
- IMU initialization failure
- I2C communication error

**Solutions:**
1. Check IMU connections (SDA/SCL pins)
2. Verify IMU is powered (3.3V)
3. Re-flash firmware
4. Check for physical damage to IMU

### Device Not Advertising

**Symptoms:**
- Device powered on but not visible in BLE scan
- No "Eidon Tracker-XXXX" in device list

**Solutions:**
1. Reset device by power cycling
2. Check battery level (may be too low)
3. Verify firmware upload was successful
4. Use nRF Connect app to scan for any BLE devices
5. Check if device is already connected to another host

### Firmware Upload Fails

**Error Messages:**
- "No DFU capable USB device available"
- "Upload error: Failed to upload"

**Solutions:**
1. Enter DFU mode manually:
   ```
   - Connect via USB
   - Open serial monitor (115200 baud)
   - Send 'D' character
   - Device should enter DFU mode
   ```
2. Try different USB cable (data capable)
3. Install/update USB drivers
4. Use PlatformIO instead of Arduino IDE

## Bluetooth Connectivity Issues

### iOS: Device Connects but No Data

**Symptoms:**
- Device connects successfully
- No quaternion data received
- Characteristics appear empty

**Solutions:**
1. Ensure using GATT service (not HID)
2. Enable notifications on quaternion characteristic
3. Check for proper UUID formatting (uppercase)
4. Verify iOS app has Bluetooth permissions

### Android: Location Permission Required

**Symptoms:**
- Bluetooth scan finds no devices
- Permission errors in logs

**Solutions:**
1. Enable location permissions for app
2. Ensure location services are on
3. Add all required permissions to manifest:
   ```xml
   <uses-permission android:name="android.permission.ACCESS_FINE_LOCATION"/>
   <uses-permission android:name="android.permission.BLUETOOTH_SCAN"/>
   <uses-permission android:name="android.permission.BLUETOOTH_CONNECT"/>
   ```

### Connection Drops Frequently

**Symptoms:**
- Device disconnects after few seconds/minutes
- Intermittent data loss

**Solutions:**
1. Reduce distance between device and phone
2. Check for interference (2.4GHz WiFi, microwaves)
3. Update connection parameters:
   ```dart
   await device.requestConnectionPriority(ConnectionPriority.high);
   ```
4. Implement reconnection logic in app
5. Check battery level (low battery affects stability)

## Data Quality Issues

### Quaternion Values Seem Wrong

**Symptoms:**
- Rotation doesn't match physical movement
- Values jumping erratically
- Drift over time

**Solutions:**
1. Calibrate IMU:
   - Send calibration command (0x01)
   - Or double-tap device
   - Wait for blue LED sequence
2. Ensure device is mounted rigidly
3. Check for magnetic interference
4. Verify quaternion parsing (little-endian floats)

### Missing Data Frames

**Symptoms:**
- Gaps in recorded data
- Lower than expected frame rate

**Solutions:**
1. Check BLE connection interval settings
2. Process data in separate thread/isolate
3. Implement buffering in mobile app
4. Monitor for BLE congestion (too many devices)

### Switch States Incorrect

**Symptoms:**
- isLeft/isUpper values wrong
- Switch states not updating

**Solutions:**
1. Check physical switch connections
2. Verify switch pin assignments in firmware
3. Test switches with multimeter
4. Check for short circuits

## Battery and Power Issues

### Battery Drains Too Quickly

**Expected Battery Life:**
- Active use: 8-12 hours
- Standby: 24-48 hours

**Solutions:**
1. Check for firmware bugs causing high CPU usage
2. Reduce IMU sampling rate if possible
3. Optimize BLE connection interval
4. Ensure device enters sleep mode when disconnected

### Device Won't Charge

**Symptoms:**
- No charging LED indication
- Battery level doesn't increase

**Solutions:**
1. Try different USB-C cable
2. Check charging pins for debris
3. Verify charger provides adequate current (>100mA)
4. Check CHG pin state (should be LOW when charging)

### Incorrect Battery Level Reading

**Symptoms:**
- Battery percentage jumps around
- Shows 0% or 100% incorrectly

**Solutions:**
1. Calibrate battery monitoring:
   - Fully discharge device
   - Fully charge device
   - Battery readings should stabilize
2. Check voltage divider components
3. Verify ADC configuration in firmware

## Mobile App Issues

### Flutter: Multiple Device Connection Fails

**Symptoms:**
- Can connect to one device but not multiple
- Second device connection times out

**Solutions:**
1. Ensure each device has unique connection handling
2. Don't share characteristics between devices
3. iOS limit: Maximum 6 simultaneous connections
4. Android: Check manufacturer limits (usually 7-10)

### Data Recording Gaps

**Symptoms:**
- Recorded sessions have missing segments
- Video doesn't sync with motion data

**Solutions:**
1. Implement proper timestamping:
   ```dart
   final timestamp = DateTime.now().toUtc();
   ```
2. Buffer data before writing to storage
3. Use high-priority background task
4. Monitor app lifecycle events

### Upload Failures

**Symptoms:**
- Session data fails to upload
- Timeout errors

**Solutions:**
1. Implement retry logic with exponential backoff
2. Chunk large uploads
3. Compress data before upload
4. Check network connectivity before upload
5. Use background upload tasks

## Hardware Issues

### Physical Damage

**Prevention:**
- Use protective enclosure (IP67 rated)
- Strain relief on cables
- Secure mounting to prevent impacts

**Common Failures:**
- Broken I2C traces: Re-solder connections
- Damaged USB port: Replace or use wireless charging
- Water damage: Dry thoroughly, may need replacement

### EMI/RFI Interference

**Symptoms:**
- Data corruption
- Connection instability near certain equipment

**Solutions:**
1. Add shielding to enclosure
2. Use shorter I2C cables
3. Add ferrite beads to power lines
4. Keep away from strong magnetic fields

## Debug Tools and Techniques

### Serial Monitor Debugging
```bash
# PlatformIO
pio device monitor -b 115200

# Arduino CLI
arduino-cli monitor -p /dev/ttyACM0 -b 115200
```

### BLE Debugging Tools
1. **nRF Connect** (Mobile): 
   - Scan for devices
   - Inspect services/characteristics
   - Send test commands

2. **Wireshark** (Desktop):
   - Capture BLE packets
   - Analyze protocol issues

3. **Web Bluetooth** (Chrome):
   - Quick testing from browser
   - JavaScript console for debugging

### Firmware Debug Flags
Add to firmware for verbose output:
```cpp
#define DEBUG_QUATERNION 1
#define DEBUG_BLE 1
#define DEBUG_BATTERY 1
```

## Getting Help

### Information to Provide
When reporting issues, include:
1. Firmware version
2. Device hardware revision
3. Mobile OS version
4. Error messages/logs
5. Steps to reproduce

### Support Channels
- GitHub Issues: Bug reports and feature requests
- Discord: Community support
- Email: support@eidon.ai for critical issues 