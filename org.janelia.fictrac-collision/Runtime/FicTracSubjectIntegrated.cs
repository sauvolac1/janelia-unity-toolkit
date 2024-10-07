using System;
using System.Collections.Generic;
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
        private float degpersec = 50;  // Degrees per second open loop rotation
        private float direction = 1; //direction of rotation 
        private float headingUnityDeg;
        private float memoryOfSlip; //keeps track of all the slips


        public void Start()
        {
            _currentFicTracParametersLog.ficTracServerAddress = ficTracServerAddress;
            _currentFicTracParametersLog.ficTracServerPort = ficTracServerPort;
            _currentFicTracParametersLog.ficTracBallRadius = ficTracBallRadius;
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

                        int i17 = 0, len17 = 0;
                        IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 17, ref i17, ref len17);
                        float d = (float)IoUtilities.ParseDouble(dataFromSocket, i17, len17, ref valid);
                        float headingFictracRad = d;
                        float headingFictracDeg = headingFictracRad * Mathf.Rad2Deg;
                        headingUnityDeg = headingFictracDeg + memoryOfSlip;
                        //Debug.Log(headingUnityDeg);
                        float headingUnityRad = headingUnityDeg * Mathf.Deg2Rad;
                        //Debug.Log("heading secondary", headingSecondary);
                        if (!valid)
                            break;

                        float headingRaw = headingUnityDeg;
                        _thresholder.UpdateAbsolute(headingRaw, Time.deltaTime);
                        if (_thresholder.angularSpeed < _thresholder.threshold)
                        {
                            _dCorrection = _dCorrection + _dCorrectionLatest;
                            float dCorrected = headingUnityRad - _dCorrection;

                            _dCorrectionBase = dCorrected;
                            _dCorrectionLatest = 0;

                            float forward = b * ficTracBallRadius;
                            float sideways = a * ficTracBallRadius;
                            Vector3 translation = new Vector3(forward, 0, sideways);

                            float heading = dCorrected * Mathf.Rad2Deg;
                            Vector3 eulerAngles = transform.eulerAngles;
                            eulerAngles.y = heading;

                            transform.Translate(translation);
                            transform.eulerAngles = eulerAngles;
                            //Debug.Log(headingUnityDeg);
                        }
                        else
                        {
                            _thresholder.Log();
                            _dCorrectionLatest = d - _dCorrection - _dCorrectionBase;

                            _currentCorrection.headingCorrectionDegs = (_dCorrection + _dCorrectionLatest) * Mathf.Rad2Deg;
                            Logger.Log(_currentCorrection);
                        }

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
                            _currentFicTracMessageLog.ficTracDeltaRotationVectorLab = new Vector3(a, b, c);
                            _currentFicTracMessageLog.ficTracIntegratedAnimalHeadingLab = d;
                            Logger.Log(_currentFicTracMessageLog);
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
                    //Debug.Log("leaving secondary block");
                    Debug.Log("memory of slip");
                    Debug.Log(memoryOfSlip);
                }
                else
                {   
                    //Debug.Log("unity heading");
                   PerformSecondaryFunctions();
                }

            }

        }

        public void PerformSecondaryFunctions()
        {   
            // Execute the code for the secondary block here
                                            // Code to execute during the secondary block
            // This could be logging, resetting variables, or other operations
            Debug.Log("Secondary block is executing.");
            // Original Update method logic
            LogUtilities.LogDeltaTime();

            Byte[] dataFromSocket = null;
            long timestampReadMs = 0;
            int i0 = -1;
            while (_socketMessageReader.GetNextMessage(ref dataFromSocket, ref timestampReadMs, ref i0))
            {
                bool valid = true;

				// https://github.com/rjdmoore/fictrac/blob/master/doc/data_header.txt
                // COL     PARAMETER                       DESCRIPTION
                // 1       frame counter                   Corresponding video frame(starts at #1).
                // 6-8     delta rotation vector (lab)     Change in orientation since last frame,
                //                                         represented as rotation angle / axis(radians)
                //                                         in laboratory coordinates(see
                //                                         * configImg.jpg).
                // 17      integrated animal heading (lab) Integrated heading orientation (radians) of
                //                                         the animal in laboratory coordinates. This
                //                                         is the direction the animal is facing.
                
                // https://www.researchgate.net/figure/Visual-output-from-the-FicTrac-software-see-supplementary-video-a-A-segment-of-the_fig2_260044337
                // Rotation about `a_x` is sideways translation
                // Rotation about `a_y` is forward/backward translation
                // (Rotation about `a_z` is heading change; not used here)												  
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
                if (_thresholder.angularSpeed < _thresholder.threshold)
                {
                    _dCorrection = _dCorrection + _dCorrectionLatest;
                    float dCorrected = d - _dCorrection;

                    _dCorrectionBase = dCorrected;
                    _dCorrectionLatest = 0;

                    float forward = 0;
                    float sideways = 0;
                    Vector3 translation = new Vector3(forward, 0, sideways);

                    //headingUnityDeg = headingUnityDeg + direction*(degpersec / framerate);
                    //Debug.Log(headingUnityDeg);
                    slipHeading = headingUnityDeg + direction*(degpersec*secondaryElapsedTime);
                    Vector3 eulerAngles = transform.eulerAngles;
                    eulerAngles.y = slipHeading;

                    memoryOfSlip = slipHeading - d*Mathf.Rad2Deg;

                    transform.Translate(translation);
                    transform.eulerAngles = eulerAngles;
                    
                }
                else
                {
                    _thresholder.Log();
                    _dCorrectionLatest = d - _dCorrection - _dCorrectionBase;

                    _currentCorrection.headingCorrectionDegs = (_dCorrection + _dCorrectionLatest) * Mathf.Rad2Deg;
                    Logger.Log(_currentCorrection);
                }

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
                    _currentFicTracMessageLog.ficTracDeltaRotationVectorLab = new Vector3(a, b, c);
                    _currentFicTracMessageLog.ficTracIntegratedAnimalHeadingLab = d;
                    Logger.Log(_currentFicTracMessageLog);
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

        private FicTracAverager _averager;

        private int _framesSinceLogWrite = 0;

        private PlaybackHandler<Transformation> _playbackHandler = new PlaybackHandler<Transformation>();
    }
}
