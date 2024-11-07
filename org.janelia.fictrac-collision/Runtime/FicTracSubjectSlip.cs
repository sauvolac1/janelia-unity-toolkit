using System;
using System.Collections.Generic;
using UnityEngine;
using static Janelia.FicTracSubjectIntegrated;

// An application using a `Janelia.KinematicSubjectIntegrated` can play back the motion
// captured in the log of a previous session.  See `PlaybackHandler.cs` in org.janelia.collison-handling.

namespace Janelia
{
    // A drop-in replacement for `Janelia.FicTracSubject`, with a few behavioral differences:
    // * it uses the "integrated animal heading (lab)" sent by FicTrac, as mentioned in the FicTrac data_header.txt:
    //   https://github.com/rjdmoore/fictrac/blob/master/doc/data_header.txt;
    // * it does not add collision handling;
    // * it does not support data smoothing or the `smoothingCount` field.

    // An application using a `Janelia.KinematicSubjectIntegrated` can play back the motion
    // captured in the log of a previous session.  See `PlaybackHandler.cs` in org.janelia.collison-handling.

    // For detecting periods of free spinning of the FicTrac trackball (when the fly has lifted its legs
    // off the trackball), indicated by heading changes with an angular speed above a threshold.
    [RequireComponent(typeof(FicTracSpinThresholder))]
    // For recording and storing the moving average (in a window of frames) for the heading angle.
    [RequireComponent(typeof(FicTracAverager))]

    public class FicTracSubjectSlip : MonoBehaviour
    {

        // For detecting periods of free spinning from FicTrac, when the heading changes
        // with an angular speed above a threshold.
        public FicTracSpinThresholder thresholder;
        public string ficTracServerAddress = "127.0.0.1";
        public int ficTracServerPort = 2000;
        public float ficTracBallRadius = 0.5f;
        public float translationalGain = 4;
        public int smoothingCount = 3;
        // The size in bytes of one item in the buffer of FicTrac messages.
        public int ficTracBufferSize = 1024;
        // The number of items in the buffer of FicTrac messages.
        public int ficTracBufferCount = 240;

        // The number of frames between writes to the log file.
        public int logWriteIntervalFrames = 100;
        public bool logFicTracMessages = false;

        private float slipHeading = 0;
        private float elapsedTime = 0f;  // Timer for the primary block
        private float secondaryElapsedTime = 0f;  // Timer for the secondary block
        public float primaryDuration = 2f;  // Duration for the primary block (28 seconds)
        public float secondaryDuration = 15f;  // Duration for the secondary block (2 seconds)
        private bool inSecondaryBlock = false;  // Flag to track if we are in the secondary block
        private float degpersec = 50;  // Degrees per second open loop rotation
        private float direction = 1; //direction of rotation 
        private float headingUnityDeg;
        private float memoryOfSlip; //keeps track of all the slips


        public void Start()
        {
            _currentFicTracParametersLog.ficTracServerAddress = ficTracServerAddress;
            _currentFicTracParametersLog.ficTracServerPort = ficTracServerPort;
            _currentFicTracParametersLog.ficTracBallRadius = ficTracBallRadius;
            _currentFicTracParametersLog.translationalGain = translationalGain;
            Logger.Log(_currentFicTracParametersLog);

            _socketMessageReader = new SocketMessageReader(HEADER, ficTracServerAddress, ficTracServerPort,
                                                           ficTracBufferSize, ficTracBufferCount);
            _socketMessageReader.Start();
			// For detecting periods of free spinning from FicTrac, when the heading changes
            // with an angular speed above a threshold.
            _thresholder = gameObject.GetComponentInChildren<FicTracSpinThresholder>();
            _dCorrection = _dCorrectionLatest = _dCorrectionBase = 0;

            _averager = gameObject.GetComponentInChildren<FicTracAverager>();

            _playbackHandler.ConfigurePlayback();
        }

        public void Update()
        {
            _deltaRotationVectorLabUpdated = Vector3.zero;

            if (_playbackHandler.Update(ref _currentTransformation, transform))
            {
                return;
            }

            if (!inSecondaryBlock)
            {
                // Increment the primary block timer
                elapsedTime += Time.deltaTime;

                if (elapsedTime > primaryDuration)
                {
                    // Enter the secondary block
                    inSecondaryBlock = true;
                }
                else
                {
                    // Original Update method logic
                    LogUtilities.LogDeltaTime();

                    Byte[] dataFromSocket = null;
                    long timestampReadMs = 0;
                    int i0 = -1;
                    while (_socketMessageReader.GetNextMessage(ref dataFromSocket, ref timestampReadMs, ref i0))
                    {
                        bool valid = true;

                        int i6 = 0, len6 = 0;
                        IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 6, ref i6, ref len6);
                        float a = (float)IoUtilities.ParseDouble(dataFromSocket, i6, len6, ref valid);
                        if (!valid)
                            break;

                        int i7 = 0, len7 = 0;
                        IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 7, ref i7, ref len7);
                        float b = (float)IoUtilities.ParseDouble(dataFromSocket, i7, len7, ref valid);
                        if (!valid)
                            break;

                        int i8 = 0, len8 = 0;
                        IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 8, ref i8, ref len8);
                        float c = (float)IoUtilities.ParseDouble(dataFromSocket, i8, len8, ref valid);
                        if (!valid)
                            break;

                        float s = Mathf.Rad2Deg;
                        float heading = c * s;
                        thresholder.UpdateRelative(heading, Time.deltaTime);
                        if (thresholder.angularSpeed < thresholder.threshold)
                        {
                            _deltaRotationVectorLabToSmooth.Set(a, b, c);
                            Smooth();
                            _deltaRotationVectorLabUpdated += _deltaRotationVectorLabToSmooth;
                        }
                        else
                        {
                            thresholder.Log();
                        }

                        if (logFicTracMessages)
                        {
                            int i22 = 0, len22 = 0;
                            IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 22, ref i22, ref len22);
                            long timestampWrite = IoUtilities.ParseLong(dataFromSocket, i22, len22, ref valid);
                            if (!valid)
                                break;

                            _currentFicTracMessageLog.ficTracTimestampWriteMs = timestampWrite;
                            _currentFicTracMessageLog.ficTracTimestampReadMs = timestampReadMs;
                            _currentFicTracMessageLog.ficTracDeltaRotationVectorLab.Set(a, b, c);
                            Logger.Log(_currentFicTracMessageLog);

                            _deltaRotationVectorLabUpdated[0] = a;
                            _deltaRotationVectorLabUpdated[1] = b;
                            _deltaRotationVectorLabUpdated[2] = c;
                        }
                    }

                    _currentTransformation.worldPosition = transform.position;
                    _currentTransformation.worldRotationDegs = transform.eulerAngles;
                    Logger.Log(_currentTransformation);

                    _averager.RecordHeading(transform.eulerAngles.y);

                    _framesSinceLogWrite++;
                    if (_framesSinceLogWrite > logWriteIntervalFrames)
                    {
                        Logger.Write();
                        _framesSinceLogWrite = 0;
                    }
                }
            }
            else
            {
                // Increment the secondary timer
                secondaryElapsedTime += Time.deltaTime;

                // Check if the secondary block has finished
                if (secondaryElapsedTime > secondaryDuration)
                {
                    // Reset the secondary block flag and timers
                    direction = direction*(-1);
                    inSecondaryBlock = false;
                    secondaryElapsedTime = 0f;
                    elapsedTime = 0f;
                }
                else
                {   
                   PerformSecondaryFunctions();
                }

            }

        }

        public Vector3? Translation()
        {
            float s = ficTracBallRadius;
            float g = translationalGain;
            float forward = _deltaRotationVectorLabUpdated[1] * s * g;
            float sideways = _deltaRotationVectorLabUpdated[0] * s * g;
            return new Vector3(forward, 0, sideways);
        }

        public Vector3? RotationDegrees()
        {
            float s = Mathf.Rad2Deg;
            float heading = _deltaRotationVectorLabUpdated[2] * s;
            return new Vector3(0, -heading, 0);
        }
        private void Smooth()
        {
            _dataForSmoothing[_dataForSmoothingOldestIndex] = _deltaRotationVectorLabToSmooth;
            _dataForSmoothingOldestIndex = (_dataForSmoothingOldestIndex + 1) % smoothingCount;

            _deltaRotationVectorLabToSmooth = Vector3.zero;
            foreach (Vector3 value in _dataForSmoothing)
            {
                _deltaRotationVectorLabToSmooth += value;
            }
            _deltaRotationVectorLabToSmooth /= smoothingCount;
        }

        private Vector3 _deltaRotationVectorLabUpdated = new Vector3();

        public void PerformSecondaryFunctions()
        {   
            // Execute the code for the secondary block here
            // This could be logging, resetting variables, or other operations
            // Original Update method logic
            LogUtilities.LogDeltaTime();

            Byte[] dataFromSocket = null;
            long timestampReadMs = 0;
            int i0 = -1;
            while (_socketMessageReader.GetNextMessage(ref dataFromSocket, ref timestampReadMs, ref i0))
            {
                bool valid = true;

                int i6 = 0, len6 = 0;
                IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 6, ref i6, ref len6);
                float a = (float)IoUtilities.ParseDouble(dataFromSocket, i6, len6, ref valid);
                if (!valid)
                    break;

                int i7 = 0, len7 = 0;
                IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 7, ref i7, ref len7);
                float b = (float)IoUtilities.ParseDouble(dataFromSocket, i7, len7, ref valid);
                if (!valid)
                    break;

                int i17 = 0, len17 = 0;
                IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 17, ref i17, ref len17);
                float d = (float)IoUtilities.ParseDouble(dataFromSocket, i17, len17, ref valid);
                if (!valid)
                    break;

                float headingRaw = d * Mathf.Rad2Deg;
                _thresholder.UpdateAbsolute(headingRaw, Time.deltaTime);

                float forward = 0;
                float sideways = 0;
                Vector3 translation = new Vector3(forward, 0, sideways);

                slipHeading = headingUnityDeg + direction*(degpersec*secondaryElapsedTime);
                Vector3 eulerAngles = transform.eulerAngles;
                eulerAngles.y = slipHeading;

                memoryOfSlip = slipHeading - d*Mathf.Rad2Deg;

                transform.Translate(translation);
                transform.eulerAngles = eulerAngles;

                _currentAttempt.fictracAttempt = new Vector3(a, b, d);

                if (logFicTracMessages)
                {
                    int i8 = 0, len8 = 0;
                    IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 8, ref i8, ref len8);
                    float c = (float)IoUtilities.ParseDouble(dataFromSocket, i8, len8, ref valid);
                    if (!valid)
                        break;

                    int i22 = 0, len22 = 0;
                    IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 22, ref i22, ref len22);
                    long timestampWriteMs = IoUtilities.ParseLong(dataFromSocket, i22, len22, ref valid);
                    if (!valid)
                        break;

                    _currentFicTracMessageLog.ficTracTimestampWriteMs = timestampWriteMs;
                    _currentFicTracMessageLog.ficTracTimestampReadMs = timestampReadMs;
                    _currentFicTracMessageLog.ficTracDeltaRotationVectorLab = new Vector3(0, 0, c);
                    _currentFicTracMessageLog.ficTracIntegratedAnimalHeadingLab = d;
                    Logger.Log(_currentFicTracMessageLog);
                }
            }

            _currentTransformation.worldPosition = transform.position;
            _currentTransformation.worldRotationDegs = transform.eulerAngles;
            Logger.Log(_currentTransformation);
            
            Logger.Log(_currentAttempt);

            _averager.RecordHeading(transform.eulerAngles.y);

            _framesSinceLogWrite++;
            if (_framesSinceLogWrite > logWriteIntervalFrames)
            {
                Logger.Write();
                _framesSinceLogWrite = 0;
            }
        }

        public void OnDisable()
        {
            _socketMessageReader.OnDisable();
        }

        public static float Mod360(float value)
        {
            float result = value % 360;
            if ((value < 0 && result > 0) || (value > 0 && result < 0))
            {
                result -= 360;
            }
            return result;
        }

        private void RecordHeading(float heading)
        {
            FicTracAverager averager = GetComponent<FicTracAverager>();
            if (averager != null)
            {
                averager.RecordHeading(heading);
            }
        }

        private SocketMessageReader.Delimiter HEADER = SocketMessageReader.Header((Byte)'F');
        private const Byte SEPARATOR = (Byte)',';    
        SocketMessageReader _socketMessageReader;

        // To make `Janelia.Logger.Log<T>()`'s call to JsonUtility.ToJson() work correctly,
        // the `T` must be marked `[Serlializable]`, but its individual fields need not be
        // marked `[SerializeField]`.  The individual fields must be `public`, though.

        [Serializable]
        private class FicTracParametersLog : Logger.Entry
        {
            public string ficTracServerAddress;
            public int ficTracServerPort;
            public float ficTracBallRadius;
            public float translationalGain;
        };
        private FicTracParametersLog _currentFicTracParametersLog = new FicTracParametersLog();

        [Serializable]
        private class FicTracMessageLog : Logger.Entry
        {
            public long ficTracTimestampWriteMs;
            public long ficTracTimestampReadMs;
            public Vector3 ficTracDeltaRotationVectorLab;
            public float ficTracIntegratedAnimalHeadingLab;
        };
        private FicTracMessageLog _currentFicTracMessageLog = new FicTracMessageLog();

        [Serializable]
        internal class Transformation : PlayableLogEntry
        {
        };
        private Transformation _currentTransformation = new Transformation();

        private FicTracSpinThresholder _thresholder;

        private float _dCorrectionBase;
        private float _dCorrection;
        private float _dCorrectionLatest;

        [Serializable]
        internal class Correction : Logger.Entry
        {
            public float headingCorrectionDegs;
        };
        private Correction _currentCorrection = new Correction();

        [Serializable]
        internal class Attempt : Logger.Entry
        {
            public Vector3 fictracAttempt;
        };
        private Attempt _currentAttempt = new Attempt();

        private Vector3[] _dataForSmoothing;
        private int _dataForSmoothingOldestIndex = 0;
        private Vector3 _deltaRotationVectorLabToSmooth = new Vector3();

        private FicTracAverager _averager;

        private int _framesSinceLogWrite = 0;

        private PlaybackHandler<Transformation> _playbackHandler = new PlaybackHandler<Transformation>();
    }
}
