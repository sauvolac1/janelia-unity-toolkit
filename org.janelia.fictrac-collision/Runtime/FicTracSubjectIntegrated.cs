using System;
using System.Collections.Generic;
using UnityEngine;

namespace Janelia
{
    // A drop-in replacement for `Janelia.FicTracSubject`, with a few behavioral differences:
    // * it uses the "integrated animal heading (lab)" sent by FicTrac, as mentioned in the FicTrac data_header.txt:
    //   https://github.com/rjdmoore/fictrac/blob/master/doc/data_header.txt;
    // * it does not add collision handling;
    // * it does not support data smoothing or the `smoothingCount` field.

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
        public void Start()
        {
            _currentFicTracParametersLog.ficTracServerAddress = ficTracServerAddress;
            _currentFicTracParametersLog.ficTracServerPort = ficTracServerPort;
            _currentFicTracParametersLog.ficTracBallRadius = ficTracBallRadius;
            Logger.Log(_currentFicTracParametersLog);

            _socketMessageReader = new SocketMessageReader(HEADER, ficTracServerAddress, ficTracServerPort,
                                                           ficTracBufferSize, ficTracBufferCount);
            _socketMessageReader.Start();
        }

        public void Update()
        {
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

                int i17 = 0, len17= 0;
                IoUtilities.NthSplit(dataFromSocket, SEPARATOR, i0, 17, ref i17, ref len17);
                float d = (float)IoUtilities.ParseDouble(dataFromSocket, i17, len17, ref valid);
                if (!valid)
                    break;

                float forward = b * ficTracBallRadius;
                float sideways = a * ficTracBallRadius;
                Vector3 translation = new Vector3(forward, 0, sideways);

                transform.Translate(translation);

                float heading = d * Mathf.Rad2Deg;
                Vector3 eulerAngles = transform.eulerAngles;
                eulerAngles.y = -heading;

                transform.eulerAngles = eulerAngles;

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
        internal class Transformation : Logger.Entry
        {
            public Vector3 worldPosition;
            public Vector3 worldRotationDegs;
        };

        private Transformation _currentTransformation = new Transformation();

        private int _framesSinceLogWrite = 0;
    }
}
