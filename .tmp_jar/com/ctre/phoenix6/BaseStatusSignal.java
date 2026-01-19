/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.ctre.phoenix6;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.Timestamp.TimestampSource;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.ParentDevice.MapGenerator;
import com.ctre.phoenix6.jni.ErrorReportingJNI;
import com.ctre.phoenix6.jni.StatusSignalJNI;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.*;

/**
 * Class that provides operations to retrieve
 * information about a status signal.
 */
public abstract class BaseStatusSignal {
    protected StatusSignalJNI jni = new StatusSignalJNI();

    private final DeviceIdentifier deviceIdentifier;
    private int spn;
    private final String name;
    private final Runnable _reportIfOldFunc;

    private final Map<Integer, String> _unitStrings;
    private int _unitsKey;

    protected String units;
    protected double baseValue = 0;
    protected AllTimestamps timestamps = new AllTimestamps();
    protected StatusCode error = StatusCode.StatusCodeNotInitialized;

    private double _lastTimestamp = 0.0;

    BaseStatusSignal(DeviceIdentifier deviceIdentifier, int spn, String signalName, Runnable reportIfOldFunc) {
        this.deviceIdentifier = deviceIdentifier;
        this.spn = spn;
        this.name = signalName;
        this._reportIfOldFunc = reportIfOldFunc;

        jni.network = deviceIdentifier.getNetwork().getName();
        jni.deviceHash = deviceIdentifier.getDeviceHash();
        jni.spn = spn;

        this.units = jni.JNI_GetUnits();
        this._unitStrings = null;
        this._unitsKey = spn;
    }

    BaseStatusSignal(DeviceIdentifier deviceIdentifier, int spn, String signalName, Runnable reportIfOldFunc, MapGenerator unitsGenerator) {
        this.deviceIdentifier = deviceIdentifier;
        this.spn = spn;
        this.name = signalName;
        this._reportIfOldFunc = reportIfOldFunc;

        jni.network = deviceIdentifier.getNetwork().getName();
        jni.deviceHash = deviceIdentifier.getDeviceHash();
        jni.spn = spn;

        this.units = jni.JNI_GetUnits();
        this._unitStrings = unitsGenerator.run();
        this._unitsKey = spn;

        var tmpJni = jni.clone();
        for (var unitString : _unitStrings.entrySet()) {
            tmpJni.spn = unitString.getKey();
            unitString.setValue(tmpJni.JNI_GetUnits());
        }
    }

    /* Constructor for an invalid BaseStatusSignal */
    BaseStatusSignal(StatusCode error) {
        this(new DeviceIdentifier(), 0, "Invalid", () -> {});
        this.error = error;
    }

    /**
     * Gets the name of this signal.
     *
     * @return Name of this signal
     */
    public final String getName() {
        return name;
    }

    /**
     * Gets the units for this signal.
     *
     * @return String representation of units for this signal
     */
    public final String getUnits() {
        return units;
    }

    /**
     * Gets the value of this signal as a double.
     *
     * @return Value of this signal as a double instead of the generic type
     */
    public final double getValueAsDouble() {
        return baseValue;
    }

    /**
     * Gets all timestamps relevant for this signal.
     *
     * @return All timestamps available for this signal
     */
    public final AllTimestamps getAllTimestamps() {
        return timestamps;
    }

    /**
     * Gets the most accurate timestamp available for this signal.
     * <p>
     * The timestamp sources from most to least accurate are:
     * <ul>
     *   <li> {@link Timestamp.TimestampSource#Device}
     *   <li> {@link Timestamp.TimestampSource#CANivore}
     *   <li> {@link Timestamp.TimestampSource#System}
     * </ul>
     * Note that some of these sources may not be available.
     *
     * @return The most accurate timestamp available
     */
    public final Timestamp getTimestamp() {
        return timestamps.getBestTimestamp();
    }

    /**
     * Gets the error code from when we last received this signal.
     *
     * @return Last cached Error Code
     */
    public final StatusCode getStatus() {
        return error;
    }

    /**
     * Checks whether the signal has been updated since the last check.
     * <p>
     * Note that the signal must be refreshed before calling this routine.
     *
     * @return true if the signal has updated since the previous call of this routine
     */
    public final boolean hasUpdated()
    {
        boolean retval = false;
        /* did we receive an update */
        final var timestamp = getAllTimestamps().getSystemTimestamp();
        if (timestamp.isValid()) {
            /* if the update timestamp is new, then a new frame was sent */
            if (_lastTimestamp != timestamp.getTime()) {
                _lastTimestamp = timestamp.getTime();
                retval = true;
            }
        }
        return retval;
    }

    /**
     * Sets the rate at which the device will publish this signal.
     * <p>
     * A frequency of 0 Hz will turn off the signal. Otherwise, the minimum supported signal
     * frequency is 4 Hz, and the maximum is 1000 Hz. Additionally, some update frequencies are not
     * supported and will be promoted up to the next highest supported frequency.
     * <p>
     * If other StatusSignals in the same status frame have been set to an update frequency,
     * the fastest requested update frequency will be applied to the frame.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     *
     * @param frequencyHz Rate to publish the signal in Hz
     * @return Status code of setting the update frequency
     */
    public final StatusCode setUpdateFrequency(double frequencyHz) {
        return setUpdateFrequency(frequencyHz, 0.100);
    }

    /**
     * Sets the rate at which the device will publish this signal.
     * <p>
     * A frequency of 0 Hz will turn off the signal. Otherwise, the minimum supported signal
     * frequency is 4 Hz, and the maximum is 1000 Hz. Additionally, some update frequencies are not
     * supported and will be promoted up to the next highest supported frequency.
     * <p>
     * If other StatusSignals in the same status frame have been set to an update frequency,
     * the fastest requested update frequency will be applied to the frame.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     *
     * @param frequency Rate to publish the signal
     * @return Status code of setting the update frequency
     */
    public final StatusCode setUpdateFrequency(Frequency frequency) {
        return setUpdateFrequency(frequency.in(Hertz));
    }

    /**
     * Sets the rate at which the device will publish this signal.
     * <p>
     * A frequency of 0 Hz will turn off the signal. Otherwise, the minimum supported signal
     * frequency is 4 Hz, and the maximum is 1000 Hz. Additionally, some update frequencies are not
     * supported and will be promoted up to the next highest supported frequency.
     * <p>
     * If other StatusSignals in the same status frame have been set to an update frequency,
     * the fastest requested update frequency will be applied to the frame.
     *
     * @param frequencyHz Rate to publish the signal in Hz
     * @param timeoutSeconds Maximum amount of time to wait when performing the action
     * @return Status code of setting the update frequency
     */
    public final StatusCode setUpdateFrequency(double frequencyHz, double timeoutSeconds) {
        return StatusCode.valueOf(jni.JNI_SetUpdateFrequency(frequencyHz, timeoutSeconds));
    }

    /**
     * Sets the rate at which the device will publish this signal.
     * <p>
     * A frequency of 0 Hz will turn off the signal. Otherwise, the minimum supported signal
     * frequency is 4 Hz, and the maximum is 1000 Hz. Additionally, some update frequencies are not
     * supported and will be promoted up to the next highest supported frequency.
     * <p>
     * If other StatusSignals in the same status frame have been set to an update frequency,
     * the fastest requested update frequency will be applied to the frame.
     *
     * @param frequency Rate to publish the signal
     * @param timeoutSeconds Maximum amount of time to wait when performing the action
     * @return Status code of setting the update frequency
     */
    public final StatusCode setUpdateFrequency(Frequency frequency, double timeoutSeconds) {
        return setUpdateFrequency(frequency.in(Hertz), timeoutSeconds);
    }

    /**
     * Gets the rate at which the device will publish this signal.
     * <p>
     * This is typically the last value passed into {@link #setUpdateFrequency}. The returned value
     * may be higher if another StatusSignal in the same status frame has been set to a higher
     * update frequency.
     *
     * @return Applied update frequency of the signal in Hz
     */
    public final double getAppliedUpdateFrequency() {
        return jni.JNI_GetAppliedUpdateFrequency();
    }

    /**
     * Gets the rate at which the device will publish this signal as a unit type.
     * <p>
     * This is typically the last value passed into {@link #setUpdateFrequency}. The returned value
     * may be higher if another StatusSignal in the same status frame has been set to a higher
     * update frequency.
     *
     * @return Applied update frequency of the signal
     */
    public final Frequency getAppliedUpdateFrequencyMeasure() {
        return Hertz.of(getAppliedUpdateFrequency());
    }

    /**
     * Performs latency compensation on signal using the signalSlope and signal's latency to determine
     * the magnitude of compensation. The caller must refresh these StatusSignals beforehand;
     * this function only does the math required for latency compensation.
     * <p>
     * This compensates for up to 300ms of latency by default.
     * <p>
     * <b>Example</b>:
     * var compensatedTurns = BaseStatusSignal.getLatencyCompensatedValue(fx.getPosition(), fx.getVelocity());
     *
     * @param signal Signal to be latency compensated. Caller must make sure this signal is up to date
     *               either by calling {@link StatusSignal#refresh()} or {@link StatusSignal#waitForUpdate(double)}.
     * @param signalSlope Derivative of signal that informs compensation magnitude. Caller must make sure this
     *                    signal is up to date either by calling {@link StatusSignal#refresh()} or
     *                    {@link StatusSignal#waitForUpdate(double)}.
     * @return Latency compensated value from the signal StatusSignal.
     */
    public static
    <U extends Unit, U_PER_SEC extends PerUnit<U, TimeUnit>, MEAS extends Measure<U>, MEAS_PER_SEC extends Measure<U_PER_SEC>>
    MEAS getLatencyCompensatedValue(StatusSignal<MEAS> signal, StatusSignal<MEAS_PER_SEC> signalSlope) {
        return getLatencyCompensatedValue(signal, signalSlope, 0.300);
    }

    /**
     * Performs latency compensation on signal using the signalSlope and signal's latency to determine
     * the magnitude of compensation. The caller must refresh these StatusSignals beforehand;
     * this function only does the math required for latency compensation.
     * <p>
     * <b>Example</b>:
     * var compensatedTurns = BaseStatusSignal.getLatencyCompensatedValue(fx.getPosition(), fx.getVelocity());
     *
     * @param signal Signal to be latency compensated. Caller must make sure this signal is up to date
     *               either by calling {@link StatusSignal#refresh()} or {@link StatusSignal#waitForUpdate(double)}.
     * @param signalSlope Derivative of signal that informs compensation magnitude. Caller must make sure this
     *                    signal is up to date either by calling {@link StatusSignal#refresh()} or
     *                    {@link StatusSignal#waitForUpdate(double)}.
     * @param maxLatencySeconds The maximum amount of latency to compensate for in seconds. A negative or zero
     *                          value disables the max latency cap. This is used to cap the contribution of
     *                          latency compensation for stale signals, such as after the device has been
     *                          disconnected from the CAN bus.
     * @return Latency compensated value from the signal StatusSignal.
     */
    public static
    <U extends Unit, U_PER_SEC extends PerUnit<U, TimeUnit>, MEAS extends Measure<U>, MEAS_PER_SEC extends Measure<U_PER_SEC>>
    MEAS getLatencyCompensatedValue(StatusSignal<MEAS> signal, StatusSignal<MEAS_PER_SEC> signalSlope, double maxLatencySeconds) {
        final var nonCompensatedSignal = signal.getValue();
        final var changeInSignal = signalSlope.getValue();
        double latency = signal.getTimestamp().getLatency();
        if (maxLatencySeconds > 0.0 && latency > maxLatencySeconds) {
            latency = maxLatencySeconds;
        }
        @SuppressWarnings("unchecked")
        var retval = (MEAS)nonCompensatedSignal.plus((MEAS)changeInSignal.times(Seconds.of(latency)));
        return retval;
    }

    /**
     * Performs latency compensation on signal using the signalSlope and signal's latency to determine
     * the magnitude of compensation. The caller must refresh these StatusSignals beforehand;
     * this function only does the math required for latency compensation.
     * <p>
     * <b>Important</b>: The signalSlope must be the rate of change of the signal. If it is not the latency
     * compensation may not perform as expected.
     * <p>
     * This compensates for up to 300ms of latency by default.
     * <p>
     * <b>Example</b>:
     * double compensatedTurns = BaseStatusSignal.getLatencyCompensatedValueAsDouble(fx.getPosition(), fx.getVelocity());
     *
     * @param signal Signal to be latency compensated. Caller must make sure this signal is up to date
     *               either by calling {@link StatusSignal#refresh()} or {@link StatusSignal#waitForUpdate(double)}.
     * @param signalSlope Derivative of signal that informs compensation magnitude. Caller must make sure this
     *                    signal is up to date either by calling {@link StatusSignal#refresh()} or
     *                    {@link StatusSignal#waitForUpdate(double)}.
     * @return Latency compensated value from the signal StatusSignal.
     */
    public static double getLatencyCompensatedValueAsDouble(BaseStatusSignal signal, BaseStatusSignal signalSlope) {
        return getLatencyCompensatedValueAsDouble(signal, signalSlope, 0.300);
    }

    /**
     * Performs latency compensation on signal using the signalSlope and signal's latency to determine
     * the magnitude of compensation. The caller must refresh these StatusSignals beforehand;
     * this function only does the math required for latency compensation.
     * <p>
     * <b>Important</b>: The signalSlope must be the rate of change of the signal. If it is not the latency
     * compensation may not perform as expected.
     * <p>
     * <b>Example</b>:
     * double compensatedTurns = BaseStatusSignal.getLatencyCompensatedValueAsDouble(fx.getPosition(), fx.getVelocity());
     *
     * @param signal Signal to be latency compensated. Caller must make sure this signal is up to date
     *               either by calling {@link StatusSignal#refresh()} or {@link StatusSignal#waitForUpdate(double)}.
     * @param signalSlope Derivative of signal that informs compensation magnitude. Caller must make sure this
     *                    signal is up to date either by calling {@link StatusSignal#refresh()} or
     *                    {@link StatusSignal#waitForUpdate(double)}.
     * @param maxLatencySeconds The maximum amount of latency to compensate for in seconds. A negative or zero
     *                          value disables the max latency cap. This is used to cap the contribution of
     *                          latency compensation for stale signals, such as after the device has been
     *                          disconnected from the CAN bus.
     * @return Latency compensated value from the signal StatusSignal.
     */
    public static double getLatencyCompensatedValueAsDouble(BaseStatusSignal signal, BaseStatusSignal signalSlope, double maxLatencySeconds) {
        final double nonCompensatedSignal = signal.getValueAsDouble();
        final double changeInSignal = signalSlope.getValueAsDouble();
        double latency = signal.getTimestamp().getLatency();
        if (maxLatencySeconds > 0.0 && latency > maxLatencySeconds) {
            latency = maxLatencySeconds;
        }
        return nonCompensatedSignal + (changeInSignal * latency);
    }

    /**
     * Waits for new data on all provided signals up to timeout.
     * This API is typically used with CANivore Bus signals as they will be synced using the
     * CANivore Timesync feature and arrive simultaneously. Signals on a roboRIO bus cannot
     * be synced and may require a significantly longer blocking call to receive all signals.
     * <p>
     * Note that CANivore Timesync requires Phoenix Pro.
     * <p>
     * This can also be used with a timeout of zero to refresh many signals at once, which
     * is faster than calling refresh() on every signal. This is equivalent to calling {@link #refreshAll}.
     * <p>
     * If a signal arrives multiple times while waiting, such as when *not* using CANivore
     * Timesync, the newest signal data is fetched. Additionally, if this function times out,
     * the newest signal data is fetched for all signals (when possible). We recommend checking
     * the individual status codes using {@link #getStatus()} when this happens.
     *
     * @param timeoutSeconds Maximum time to wait for new data in seconds.
     *                       Pass zero to refresh all signals without blocking.
     * @param signals        Signals to wait for new data against
     * @return An InvalidParamValue if signals array is empty,
     *         InvalidNetwork if signals are on different CAN bus networks,
     *         RxTimeout if it took longer than timeoutSeconds to receive all the signals,
     *         MultiSignalNotSupported if using the roboRIO bus with more than one signal and a non-zero timeout.
     *         An OK status code means that all signals arrived within timeoutSeconds and they are all OK.
     *
     *         Any other value represents the StatusCode of the first failed signal.
     *         Call getStatus() on each signal to determine which ones failed.
     */
    public static StatusCode waitForAll(double timeoutSeconds, BaseStatusSignal... signals) {
        return waitForAll(timeoutSeconds, Arrays.asList(signals));
    }

    /**
     * Waits for new data on all provided signals up to timeout.
     * This API is typically used with CANivore Bus signals as they will be synced using the
     * CANivore Timesync feature and arrive simultaneously. Signals on a roboRIO bus cannot
     * be synced and may require a significantly longer blocking call to receive all signals.
     * <p>
     * Note that CANivore Timesync requires Phoenix Pro.
     * <p>
     * This can also be used with a timeout of zero to refresh many signals at once, which
     * is faster than calling refresh() on every signal. This is equivalent to calling {@link #refreshAll}.
     * <p>
     * If a signal arrives multiple times while waiting, such as when *not* using CANivore
     * Timesync, the newest signal data is fetched. Additionally, if this function times out,
     * the newest signal data is fetched for all signals (when possible). We recommend checking
     * the individual status codes using {@link #getStatus()} when this happens.
     *
     * @param timeoutSeconds Maximum time to wait for new data in seconds.
     *                       Pass zero to refresh all signals without blocking.
     * @param signals        List of signals to wait for new data against
     * @return An InvalidParamValue if signals array is empty,
     *         InvalidNetwork if signals are on different CAN bus networks,
     *         RxTimeout if it took longer than timeoutSeconds to receive all the signals,
     *         MultiSignalNotSupported if using the roboRIO bus with more than one signal and a non-zero timeout.
     *         An OK status code means that all signals arrived within timeoutSeconds and they are all OK.
     *
     *         Any other value represents the StatusCode of the first failed signal.
     *         Call getStatus() on each signal to determine which ones failed.
     */
    public static StatusCode waitForAll(double timeoutSeconds, List<BaseStatusSignal> signals) {
        return waitForAllImpl("ctre.phoenix6.BaseStatusSignal.waitForAll", timeoutSeconds, signals);
    }

    /**
     * Performs a non-blocking refresh on all provided signals.
     * <p>
     * This provides a performance improvement over separately calling refresh() on each signal.
     *
     * @param signals Signals to refresh
     * @return An InvalidParamValue if signals array is empty,
     *         InvalidNetwork if signals are on different CAN bus networks.
     *         An OK status code means that all signals are OK.
     *
     *         Any other value represents the StatusCode of the first failed signal.
     *         Call getStatus() on each signal to determine which ones failed.
     */
    public static StatusCode refreshAll(BaseStatusSignal... signals) {
        return refreshAll(Arrays.asList(signals));
    }

    /**
     * Performs a non-blocking refresh on all provided signals.
     * <p>
     * This provides a performance improvement over separately calling refresh() on each signal.
     *
     * @param signals List of signals to refresh
     * @return An InvalidParamValue if signals array is empty,
     *         InvalidNetwork if signals are on different CAN bus networks.
     *         An OK status code means that all signals are OK.
     *
     *         Any other value represents the StatusCode of the first failed signal.
     *         Call getStatus() on each signal to determine which ones failed.
     */
    public static StatusCode refreshAll(List<BaseStatusSignal> signals) {
        return waitForAllImpl("ctre.phoenix6.BaseStatusSignal.refreshAll", 0, signals);
    }

    /**
     * Checks if all signals have an OK error code.
     *
     * @param signals Signals to check error code of
     * @return True if all are good, false otherwise
     */
    public static boolean isAllGood(BaseStatusSignal... signals) {
        for (BaseStatusSignal sig : signals) {
            if (!sig.getStatus().isOK()) {
                return false;
            }
        }
        return true;
    }

    /**
     * Checks if all signals have an OK error code.
     *
     * @param signals List of signals to check error code of
     * @return True if all are good, false otherwise
     */
    public static boolean isAllGood(List<BaseStatusSignal> signals) {
        for (BaseStatusSignal sig : signals) {
            if (!sig.getStatus().isOK()) {
                return false;
            }
        }
        return true;
    }

    /**
     * Sets the update frequency of all specified status signals to the provided common frequency.
     * <p>
     * A frequency of 0 Hz will turn off the signal. Otherwise, the minimum supported signal
     * frequency is 4 Hz, and the maximum is 1000 Hz. Additionally, some update frequencies are not
     * supported and will be promoted up to the next highest supported frequency.
     * <p>
     * If other StatusSignals in the same status frame have been set to an update frequency,
     * the fastest requested update frequency will be applied to the frame.
     * <p>
     * This will wait up to 0.100 seconds (100ms) for each signal.
     *
     * @param frequencyHz Rate to publish the signal in Hz
     * @param signals Signals to apply the update frequency to
     * @return Status code of the first failed update frequency set call, or OK if all succeeded
     */
    public static StatusCode setUpdateFrequencyForAll(double frequencyHz, BaseStatusSignal... signals) {
        return setUpdateFrequencyForAll(frequencyHz, Arrays.asList(signals));
    }

    /**
     * Sets the update frequency of all specified status signals to the provided common frequency.
     * <p>
     * A frequency of 0 Hz will turn off the signal. Otherwise, the minimum supported signal
     * frequency is 4 Hz, and the maximum is 1000 Hz. Additionally, some update frequencies are not
     * supported and will be promoted up to the next highest supported frequency.
     * <p>
     * If other StatusSignals in the same status frame have been set to an update frequency,
     * the fastest requested update frequency will be applied to the frame.
     * <p>
     * This will wait up to 0.100 seconds (100ms) for each signal.
     *
     * @param frequencyHz Rate to publish the signal in Hz
     * @param signals List of signals to apply the update frequency to
     * @return Status code of the first failed update frequency set call, or OK if all succeeded
     */
    public static StatusCode setUpdateFrequencyForAll(double frequencyHz, List<BaseStatusSignal> signals) {
        StatusSignalJNI[] toSet = new StatusSignalJNI[signals.size()];
        for (int i = 0; i < signals.size(); ++i) {
            toSet[i] = signals.get(i).jni;
        }
        return StatusCode.valueOf(StatusSignalJNI.JNI_SetUpdateFrequencyForAll(frequencyHz, toSet, 0.100));
    }

    /**
     * Sets the update frequency of all specified status signals to the provided common frequency.
     * <p>
     * A frequency of 0 Hz will turn off the signal. Otherwise, the minimum supported signal
     * frequency is 4 Hz, and the maximum is 1000 Hz. Additionally, some update frequencies are not
     * supported and will be promoted up to the next highest supported frequency.
     * <p>
     * If other StatusSignals in the same status frame have been set to an update frequency,
     * the fastest requested update frequency will be applied to the frame.
     * <p>
     * This will wait up to 0.100 seconds (100ms) for each signal.
     *
     * @param frequency Rate to publish the signal
     * @param signals Signals to apply the update frequency to
     * @return Status code of the first failed update frequency set call, or OK if all succeeded
     */
    public static StatusCode setUpdateFrequencyForAll(Frequency frequency, BaseStatusSignal... signals) {
        return setUpdateFrequencyForAll(frequency.in(Hertz), signals);
    }

    /**
     * Sets the update frequency of all specified status signals to the provided common frequency.
     * <p>
     * A frequency of 0 Hz will turn off the signal. Otherwise, the minimum supported signal
     * frequency is 4 Hz, and the maximum is 1000 Hz. Additionally, some update frequencies are not
     * supported and will be promoted up to the next highest supported frequency.
     * <p>
     * If other StatusSignals in the same status frame have been set to an update frequency,
     * the fastest requested update frequency will be applied to the frame.
     * <p>
     * This will wait up to 0.100 seconds (100ms) for each signal.
     *
     * @param frequency Rate to publish the signal
     * @param signals List of signals to apply the update frequency to
     * @return Status code of the first failed update frequency set call, or OK if all succeeded
     */
    public static StatusCode setUpdateFrequencyForAll(Frequency frequency, List<BaseStatusSignal> signals) {
        return setUpdateFrequencyForAll(frequency.in(Hertz), signals);
    }

    protected final void updateUnits(int unitsKey) {
        if (_unitsKey != unitsKey && _unitStrings != null && _unitStrings.containsKey(unitsKey)) {
            units = _unitStrings.get(unitsKey);
            _unitsKey = unitsKey;
        }
    }

    protected final void refreshValue(boolean waitForSignal, double timeout, boolean reportError) {
        _reportIfOldFunc.run();

        if (waitForSignal) {
            error = StatusCode.valueOf(jni.JNI_WaitForSignal(timeout));
        } else {
            error = StatusCode.valueOf(jni.JNI_RefreshSignal(timeout));
        }
        baseValue = jni.value;
        timestamps.update(
            jni.swtimeStampSeconds, TimestampSource.System, true,
            jni.hwtimeStampSeconds, TimestampSource.CANivore, true,
            jni.ecutimeStampSeconds, TimestampSource.Device, jni.ecutimeStampSeconds != 0.0
        );
        updateUnits(jni.unitsKey);

        if (reportError && !this.error.isOK()) {
            String device = this.deviceIdentifier.toString() + " Status Signal " + this.name;
            ErrorReportingJNI.reportStatusCode(this.error.value, device);
        }
    }

    private static StatusCode waitForAllImpl(String location, double timeoutSeconds, List<BaseStatusSignal> signals) {
        if (signals.size() < 1) {
            /* We don't have any signals to wait for, so return early */
            ErrorReportingJNI.reportStatusCode(StatusCode.InvalidParamValue.value, location);
            return StatusCode.InvalidParamValue;
        }

        String network = signals.get(0).deviceIdentifier.getNetwork().getName();
        StatusSignalJNI[] toGet = new StatusSignalJNI[signals.size()];
        for (int i = 0; i < signals.size(); ++i) {
            var sig = signals.get(i);
            if (i != 0 && !sig.deviceIdentifier.getNetwork().getName().equals(network)) {
                /* Networks don't match, return early */
                for (var s : signals) {
                    s.error = StatusCode.InvalidNetwork;
                }
                ErrorReportingJNI.reportStatusCode(StatusCode.InvalidNetwork.value, location);
                return StatusCode.InvalidNetwork;
            }
            toGet[i] = sig.jni;
        }

        /* Report if any device firmware versions are too old */
        for (var signal : signals) {
            signal._reportIfOldFunc.run();
        }

        /* Now wait for all the signals */
        int err = StatusSignalJNI.JNI_WaitForAll(network, timeoutSeconds, toGet);
        for (int i = 0; i < signals.size(); ++i) {
            var sig = signals.get(i);
            sig.error = StatusCode.valueOf(toGet[i].statusCode);
            sig.baseValue = toGet[i].value;
            sig.timestamps.update(
                toGet[i].swtimeStampSeconds, TimestampSource.System, true,
                toGet[i].hwtimeStampSeconds, TimestampSource.CANivore, true,
                toGet[i].ecutimeStampSeconds, TimestampSource.Device, toGet[i].ecutimeStampSeconds != 0.0
            );
            sig.updateUnits(toGet[i].unitsKey);
        }

        /* error reporting */
        StatusCode retval = StatusCode.valueOf(err);
        if (!retval.isOK()) {
            for (var signal : signals) {
                if (!signal.error.isOK()) {
                    String sigLocation = signal.deviceIdentifier.toString() + " Status Signal " + signal.name;
                    ErrorReportingJNI.reportStatusCode(signal.error.value, sigLocation);
                }
            }
            ErrorReportingJNI.reportStatusCode(retval.value, location);
        }
        return StatusCode.valueOf(err);
    }
};
