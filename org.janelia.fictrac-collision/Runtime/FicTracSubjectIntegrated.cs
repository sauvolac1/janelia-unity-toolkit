#define LOG_ALL_MESHES

//using GluonGui.WorkspaceWindow.Views.WorkspaceExplorer;
using System;
using System.Collections.Generic;
using System.Diagnostics.Eventing.Reader;
using UnityEngine;

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

    public class FicTracSubjectIntegrated : MonoBehaviour
    {
        public string ficTracServerAddress = "127.0.0.1";
        public int ficTracServerPort = 2000;
        public float ficTracBallRadius = 0.5f;
        public float translationalGain = 1;
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
        public float primaryDuration = 28f;  // Duration for the primary block (28 seconds)
        public float secondaryDuration = 2f;  // Duration for the secondary block (2 seconds)
        private bool inSecondaryBlock = false;  // Flag to track if we are in the secondary block
        public bool openLoopOnly = false;
        private float direction = 1; //direction of rotation 
        private float headingUnityDeg;
        private float memoryOfSlip; //keeps track of all the slips
        private bool firstFrame = true;
        private float headingRawlast = 0;
        private float dx = 0;
        private float dz = 0;
        public float translationLimit = 20; //translation distance that triggers teleportation back to start
        public bool closedRotDuringSlip = true;
        public bool closedTransDuringSlip = false;
        private float rotMultiplier = 0;
        private float transMultiplier = 0;
        public float degpersec = 50;  // Degrees per second open loop rotation
        public float cmpersec = 15; //Cm per second of translational slip
        private float dmpersec => cmpersec / 10;
       /* private List<float> translationSlips = new List<float> {(float)(Math.PI * 0.75), (float)(Math.PI * 1.0), (float)(Math.PI * 1.75),
                (float)(Math.PI * 0.5), (float)(Math.PI * 2.0), (float)(Math.PI * 0.25),
                (float)(Math.PI * 1.5), (float)(Math.PI * 1.25)};*/
        private List<float> translationSlips = new List<float> {(float)(Math.PI * 1.0), (float)(Math.PI * 1.0)};
        private float theta = 0;
        private float instantaneousTheta = 0;
        private int directionIndex = 0;
        public float flyHeight = 0.01f;
        public bool egocentricTranslationalSlip = true;
        private float switcher = 0;


        public void Start()
        {
            _currentFicTracParametersLog.ficTracServerAddress = ficTracServerAddress;
            _currentFicTracParametersLog.ficTracServerPort = ficTracServerPort;
            _currentFicTracParametersLog.ficTracBallRadius = ficTracBallRadius;
            _currentFicTracParametersLog.translationalGain = translationalGain;

            _socketMessageReader = new SocketMessageReader(HEADER, ficTracServerAddress, ficTracServerPort,
                                                           ficTracBufferSize, ficTracBufferCount);
            _socketMessageReader.Start();

            _playbackHandler.ConfigurePlayback();

#if LOG_ALL_MESHES
            LogUtilities.LogAllMeshes();

            if (closedRotDuringSlip)
            {
                rotMultiplier = 1;
            }

            if (closedTransDuringSlip)
            {
                transMultiplier = 1;
            }

        }
#endif

        public void Update()
        {
            bool isPlayback = _playbackHandler.Update(ref _currentTransformation, transform);
            if (!_parametersLogged)
            {
                _parametersLogged = true;
                _currentFicTracParametersLog.isReplaySession = isPlayback;
                Logger.Log(_currentFicTracParametersLog);
            }

            if (isPlayback)
            {
                // During playback, still consume FicTrac socket messages so the buffer
                // does not back up.  Compute what the world would look like if the fly
                // had closed-loop control, and log both the replayed positions and the
                // attempted positions.
                if (!_attemptInitialized)
                {
                    _attemptPosition = transform.position;
                    _attemptRotationDegs = transform.eulerAngles;
                    _attemptInitialized = true;
                }

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

                    // Apply the same closed-loop transforms as the primary block to compute
                    // what the world position would be if the fly was in control.
                    float attemptHeadingDeg = d * Mathf.Rad2Deg;

                    float forward = b * ficTracBallRadius * translationalGain;
                    float sideways = a * ficTracBallRadius * translationalGain;
                    Vector3 localTranslation = new Vector3(forward, 0, sideways);

                    Quaternion rotation = Quaternion.Euler(0, attemptHeadingDeg, 0);
                    _attemptPosition += rotation * localTranslation;
                    _attemptRotationDegs = new Vector3(0, attemptHeadingDeg, 0);

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

                        // Field 22 in the FicTrac message may not be as documented and so may not
                        // be parsable. Log the rest of the message anyway.
                        if (!valid)
                            timestampWriteMs = 0;

                        _currentFicTracMessageLog.ficTracTimestampWriteMs = timestampWriteMs;
                        _currentFicTracMessageLog.ficTracTimestampReadMs = timestampReadMs;
                        _currentFicTracMessageLog.ficTracDeltaRotationVectorLab = new Vector3(a, b, c);
                        _currentFicTracMessageLog.ficTracIntegratedAnimalHeadingLab = d;
                        Logger.Log(_currentFicTracMessageLog);
                    }
                }

                // Log the replayed world position (what the player sees) alongside the
                // computed attempt position (what the world would look like in closed loop).
                LogUtilities.LogDeltaTime();
                _currentReplayTransformation.worldPositionReplay = _currentTransformation.worldPosition;
                _currentReplayTransformation.worldRotationDegsReplay = _currentTransformation.worldRotationDegs;
                _currentReplayTransformation.worldPositionAttempt = _attemptPosition;
                _currentReplayTransformation.worldRotationDegsAttempt = _attemptRotationDegs;
                Logger.Log(_currentReplayTransformation);

                _framesSinceLogWrite++;
                if (_framesSinceLogWrite > logWriteIntervalFrames)
                {
                    Logger.Write();
                    _framesSinceLogWrite = 0;
                }
                return;
            }

            if (!inSecondaryBlock && !openLoopOnly)
            {
                // Increment the primary block timer
                elapsedTime += Time.deltaTime;

                if (elapsedTime > primaryDuration)
                {
                    // Enter the secondary block
                    inSecondaryBlock = true;
                    LogUtilities.LogDeltaTime();
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

                        int i17 = 0, len17 = 0;
                        IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 17, ref i17, ref len17);
                        float d = (float)IoUtilities.ParseDouble(dataFromSocket, i17, len17, ref valid);
                        float headingFictracRad = d;
                        float headingFictracDeg = headingFictracRad * Mathf.Rad2Deg;
                        headingUnityDeg = headingFictracDeg + memoryOfSlip;
                        if (!valid)
                            break;

                        float forward = b * ficTracBallRadius * translationalGain;
                        float sideways = a * ficTracBallRadius * translationalGain;
                        Vector3 translation = new Vector3(forward, 0, sideways);

                        Vector3 eulerAngles = transform.eulerAngles;
                        eulerAngles.y = headingUnityDeg;

                        if (IsAnyComponentGreaterOrEqual(transform.position, translationLimit))
                        {
                            transform.position = new Vector3(0, flyHeight, 0);
                        }
                        else
                        {
                            transform.Translate(translation);
                        }

                        transform.eulerAngles = eulerAngles;

                        firstFrame = true;



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

                            // Field 22 in the FicTrac message may not be as documented and so may not
                            // be parsable. Log the rest of the message anyway.
                            if (!valid)
                                timestampWriteMs = 0;

                            _currentFicTracMessageLog.ficTracTimestampWriteMs = timestampWriteMs;
                            _currentFicTracMessageLog.ficTracTimestampReadMs = timestampReadMs;
                            _currentFicTracMessageLog.ficTracDeltaRotationVectorLab = new Vector3(a, b, c);
                            _currentFicTracMessageLog.ficTracIntegratedAnimalHeadingLab = d;
                            Logger.Log(_currentFicTracMessageLog);
                        }
                    }


                    _currentTransformation.worldPosition = transform.position;
                    _currentTransformation.worldRotationDegs = transform.eulerAngles;
                    Logger.Log(_currentTransformation);

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
                    direction = direction * (-1);
                    inSecondaryBlock = false;
                    secondaryElapsedTime = 0f;
                    elapsedTime = 0f;
                    LogUtilities.LogDeltaTime();
                    directionIndex = (directionIndex + 1) % translationSlips.Count;
                }
                else
                {
                    PerformSecondaryFunctions();
                }

            }

        }

        public void PerformSecondaryFunctions()
        {
            // Execute the code for the secondary block here
            // This could be logging, resetting variables, or other operations
            // Original Update method logic
            LogUtilities.LogDeltaTime();

            Byte[] dataFromSocket = null;
            long timestampReadMs = 0;
            int i0 = -1;

            if (firstFrame == true)
            {
                headingRawlast = headingUnityDeg - memoryOfSlip;
                firstFrame = false;
            }

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

                float headingRaw = (d * Mathf.Rad2Deg);

                slipHeading = headingUnityDeg + direction * (degpersec * secondaryElapsedTime) + (rotMultiplier * (headingRaw - headingRawlast));
                Vector3 eulerAngles = transform.eulerAngles;
                eulerAngles.y = slipHeading;

                memoryOfSlip = slipHeading - headingRaw;

                switcher = secondaryDuration / 3;
                if (secondaryElapsedTime <= switcher || secondaryElapsedTime >= 2*switcher)
                {
                    theta = 0;
                }
                else
                {
                    theta = translationSlips[directionIndex];
                }

                if (egocentricTranslationalSlip)
                {
                    instantaneousTheta = theta;
                }
                else
                {
                    instantaneousTheta = (Mathf.Deg2Rad * slipHeading) + theta;
                }


                dx = dmpersec * (float)Math.Cos(instantaneousTheta);
                dz = dmpersec * (float)Math.Sin(instantaneousTheta);


                float forward = (b * ficTracBallRadius * translationalGain * transMultiplier) + (dx * Time.deltaTime * translationalGain);
                float sideways = (a * ficTracBallRadius * translationalGain * transMultiplier) - (dz * Time.deltaTime * translationalGain);
                Vector3 translation = new Vector3(forward, 0, sideways);


                if (IsAnyComponentGreaterOrEqual(transform.position, translationLimit))
                {
                    transform.position = new Vector3(0, flyHeight, 0);
                }
                else
                {
                    transform.Translate(translation);
                }
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

                    // Field 22 in the FicTrac message may not be as documented and so may not
                    // be parsable. Log the rest of the message anyway.
                    if (!valid)
                        timestampWriteMs = 0;

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

        bool IsAnyComponentGreaterOrEqual(Vector3 vector, float threshold)
        {
            return (Mathf.Abs(vector.x) >= threshold) || (Mathf.Abs(vector.y) >= threshold) || (Mathf.Abs(vector.z) >= threshold);
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
            public bool isReplaySession;
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


        [Serializable]
        internal class Attempt : Logger.Entry
        {
            public Vector3 fictracAttempt;
        };
        private Attempt _currentAttempt = new Attempt();

        [Serializable]
        private class ReplayTransformation : Logger.Entry
        {
            public Vector3 worldPositionReplay;
            public Vector3 worldRotationDegsReplay;
            public Vector3 worldPositionAttempt;
            public Vector3 worldRotationDegsAttempt;
        };
        private ReplayTransformation _currentReplayTransformation = new ReplayTransformation();

        private Vector3 _attemptPosition;
        private Vector3 _attemptRotationDegs;
        private bool _attemptInitialized = false;
        private bool _parametersLogged = false;
        private int _framesSinceLogWrite = 0;

        private PlaybackHandler<Transformation> _playbackHandler = new PlaybackHandler<Transformation>();
    }
}