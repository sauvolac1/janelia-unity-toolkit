﻿// A subject (GameObject representing a moving agent) that supports
// kinematic motion, kinematic collision detection, and logging.
// The kinematic motion is provided by the `updater` field, an object
// that must conform to the `Janelia.KinematicSubject.IKinematicUpdater`
// interface.

// An application using a `Janelia.KinematicSubject` can play back the motion
// captured in the log of a previous session.  This playback is controlled by the
// following command-line arguments
// `-playback` : plays back the most recently saved log file
// `-playback logFile` : plays back the specified log file from the standard log directory
// `-playback logDir/logFile` : plays back the specified log file

// This class has a post-build step that plugs user interface into the launcher script
// created by the `Janelia.Logger` class from the  `org.janelia.logging` package.
// This user interface shows a list of the log files generated by the application and
// lets the user choose one for playback.

// In addition to logging the motion (rotation and translation) at each frame, this class
// also logs the time spent processing each frame, and also logs some details of all
// the meshes as of the start of the application (if `LOG_ALL_MESHES` is defined).

#define SUPPORT_COMMANDLINE_ARGUMENTS
// #define SUPPORT_KEYBOARD_SHORTCUTS
#define LOG_ALL_MESHES

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
#if UNITY_EDITOR
using UnityEditor.Callbacks;
#endif
using UnityEngine;

namespace Janelia
{
    // The `DefaultExecutionOrder` attribute needs to specify a value lower than the default of 0,
    // so `Update()` will execute before the cameras rendered by `Janelia.AdjoningDisplaysCamera`.
    // Otherwise, some of those cameras may be a frame behind, producing vertical "tearing".
    [DefaultExecutionOrder(-100)]
    public class KinematicSubject : MonoBehaviour
    {
        // A class conforming to this interface provides the kinematic motion (translation,
        // rotation) for this subject object.
        public interface IKinematicUpdater
        {
            // With C# 8.0, interfaces will be able to have default implementations.  But
            // as of 2020, Unity is not using C# 8.0.  So classes implementing this interface
            // must have `Start()` and `Update()` methods even if they do nothing.
            void Start();
            void Update();

            // These values are relative (displacements).
            Vector3? Translation();
            Vector3? RotationDegrees();
        };

        // The object that provides the kinematic motion, to be set by the constructor of
        // the subclass of this class.
        public IKinematicUpdater updater;

        // Parameters of the collision detection supported by this class.
        public float collisionRadius = 1.0f;
        public Vector3? collisionPlaneNormal = null;

        // Parameters to control when the log is written.  The goal is to avoid writing
        // either too frequently or not frequently enough.  A heuristic is to write when
        // the subject is "still", that is, when the `IKinematicUpdater` has not given any
        // motion for a small number of frames.
        public bool writeLogWhenStill = true;
        public int stillFrames = 5;

        // That heuristic is further refined so that writing when "still" will not happen
        // before a minimum number of frames have passed since the last writing, and writing
        // will not wait more than a maximum number of frames since the last writing, even
        // if not "still".
        public int minWriteInterval = 100;
        public int maxWriteInterval = 200;

        public bool detectCollisions = true;

        public bool debug = false;

        public void Start()
        {
            if (updater == null)
            {
                Debug.LogError("Janelia.KinematicSubject.updater must be set.");
                Application.Quit();
            }

            updater.Start();

            // Set up the collision handler to act on this `GameObject`'s transform.
            _collisionHandler = new KinematicCollisionHandler(transform, collisionPlaneNormal, collisionRadius);

            if (ConfigurePlayback())
            {
                StartPlayback();
            }

#if LOG_ALL_MESHES
            LogUtilities.LogAllMeshes();
#endif
        }

        public void Update()
        {
#if SUPPORT_KEYBOARD_SHORTCUTS
            if (Input.GetKey("l"))
            {
                StartPlayback();
            }
#endif

            _framesBeingStill++;

            _currentTransformation.Clear();
            bool addToLog = false;

            if (!_playbackActive)
            {
                updater.Update();

                Vector3? translation = updater.Translation();
                if (translation != null)
                {
                    Vector3 actualTranslation;
                    if (detectCollisions)
                    {
                        // Let the collision handler correct the translation, with approximated sliding contact,
                        // and apply it to this `GameObject`'s transform.  The corrected translation is returned.
                        actualTranslation = _collisionHandler.Translate((Vector3)translation);
                    }
                    else
                    {
                        actualTranslation = (Vector3)translation;
                        transform.Translate(actualTranslation);
                    }
                    _currentTransformation.attemptedTranslation = (Vector3)translation;
                    _currentTransformation.actualTranslation = actualTranslation;

                    if (debug)
                    {
                        Debug.Log("frame " + Time.frameCount + ": translation " + translation + " becomes " + actualTranslation);
                    }

                    addToLog = true;
                    _framesBeingStill = 0;
                }

                Vector3? rotation = updater.RotationDegrees();
                if (rotation != null)
                {
                    transform.Rotate((Vector3)rotation);
                    _currentTransformation.rotationDegs = (Vector3)rotation;

                    addToLog = true;
                    _framesBeingStill = 0;
                }
            }
            else
            {
                Transformation transformation = CurrentPlaybackTransformation();
                if (transformation != null)
                {
                    _currentTransformation.Set(transformation);

                    transform.position = _currentTransformation.worldPosition;
                    transform.eulerAngles = _currentTransformation.worldRotationDegs;

                    addToLog = true;
                    _framesBeingStill = 0;
                }
            }

            if (addToLog)
            {
                _currentTransformation.worldPosition = transform.position;
                _currentTransformation.worldRotationDegs = transform.eulerAngles;
                Logger.Log(_currentTransformation);
            }

            _framesSinceLogWrite++;
            bool writeLog = false;
            if (writeLogWhenStill)
            {
                if ((_framesBeingStill >= stillFrames) && (_framesSinceLogWrite > minWriteInterval))
                {
                    writeLog = true;
                    if (debug)
                    {
                        Debug.Log("Frame " + Time.frameCount + ", writing log: still for " + _framesBeingStill + " frames, and " +
                            _framesSinceLogWrite + " frames since last write");
                    }
                }
            }
            if (_framesSinceLogWrite >= maxWriteInterval)
            {
                writeLog = true;
                if (debug)
                {
                    Debug.Log("Frame " + Time.frameCount + ", writing log: " + _framesSinceLogWrite + " frames since last write");
                }
            }

            LogUtilities.LogDeltaTime();

            if (writeLog)
            {
                Logger.Write();
                _framesSinceLogWrite = 0;
                _framesBeingStill = 0;
            }
        }

#if UNITY_EDITOR
        // The attribute value orders this function after `Logger.OnPostprocessBuildStart` and
        // before `Logger.OnPostprocessBuildFinish`.
        [PostProcessBuildAttribute(2)]
        public static void OnPostprocessBuild(BuildTarget target, string pathToBuiltProject)
        {
            Debug.Log("Janelia.KinematicSubject.OnPostprocessBuild: " + pathToBuiltProject);

            string n = System.Environment.NewLine;
            string radioButtonLabel = "Replay a log of a previous session";
            string radioButtonOtherHTML =
                "          <div>" + n +
                "            <label for='logs'>Choose a log:</label>" + n +
                "            <select name='logs' id='id_selectLogs'></select>" + n +
                "          </div>";
            string scriptBlockWithRadioButtonFunc =
                "    <script language='javascript'>" + n +
                "      var select = document.getElementById('id_selectLogs');" + n +
                "      var fso = new ActiveXObject('Scripting.FileSystemObject');" + n +
                "      var folderPath = LogDir();" + n +
                "      if (fso.FolderExists(folderPath)) {" + n +
                "        var folder = fso.GetFolder(folderPath);" + n +
                "        var files = [];" + n +
                "        for (var en = new Enumerator(folder.Files); !en.atEnd(); en.moveNext()) {" + n +
                "          var s = en.item();" + n +
                "          if (/Log.*json/.test(s))" + n +
                "            files.push(s);" + n +
                "        }" + n +
                "        files.sort();" + n +
                "        for (var i = files.length - 1; i >= 0; i--)" + n +
                "          select.add(new Option(files[i], files[i]));" + n +
                "      }" + n +
                "      if (select.options.length === 0) {" + n +
                "        select.add(new Option('No logs', 0));" + n +
                "        select.disabled = true;" + n +
                "      }" + n +
                "      function actionKinematicSubject()" + n +
                "      {" + n +
                "        var extra = '-playback ' + document.getElementById('id_selectLogs').value;" + n +
                "        runApp(extra);" + n +
                "      }" + n +
                "    </script>";
            Logger.AddLauncherRadioButtonPlugin(radioButtonLabel, radioButtonOtherHTML, "actionKinematicSubject", scriptBlockWithRadioButtonFunc);
        }
#endif

        private bool ConfigurePlayback()
        {
#if SUPPORT_COMMANDLINE_ARGUMENTS
            _playbackLogFile = Logger.previousLogFile;
            string[] args = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length; i++)
            {
                if (args[i] == "-playback")
                {
                    if (i + 1 < args.Length)
                    {
                        _playbackLogFile = args[i + 1];
                        if (!_playbackLogFile.Contains('/') && !_playbackLogFile.Contains('\\'))
                        {
                            _playbackLogFile = Logger.logDirectory + "/" + _playbackLogFile;
                        }
                        if (!File.Exists(_playbackLogFile))
                        {
                            Debug.Log("Cannot find playback log file '" + _playbackLogFile + "'");
                            return false;
                        }
                    }
                    return true;
                }
            }
#endif
            return false;
        }

        private void StartPlayback()
        {
            if (!_playbackActive)
            {
                Debug.Log("Playing back log file '" + _playbackLogFile + "'");

                _playbackLogEntries = Logger.Read<Transformation>(_playbackLogFile);
                _playbackLogEntries = Filter(_playbackLogEntries);

                _playbackLogIndex = 0;
                _playbackStartFrame = Time.frameCount;
                _playbackActive = true;
            }
        }

        private List<Transformation> Filter(List<Transformation> l)
        {
            return l.Where(x => x.attemptedTranslation != Vector3.zero || x.rotationDegs != Vector3.zero).ToList();
        }

        private Transformation CurrentPlaybackTransformation()
        {
            if (_playbackActive && (_playbackLogEntries != null))
            {
                int adjustedFrame = Time.frameCount - _playbackStartFrame;
                while (_playbackLogIndex < _playbackLogEntries.Count)
                {
                    if (_playbackLogEntries[_playbackLogIndex].frame < adjustedFrame)
                    {
                        _playbackLogIndex++;
                    }
                    else
                    {
                        if (_playbackLogEntries[_playbackLogIndex].frame == adjustedFrame)
                        {
                            return _playbackLogEntries[_playbackLogIndex];
                        }
                        return null;
                    }
                }
            }
            _playbackActive = false;
            return null;
        }

        private KinematicCollisionHandler _collisionHandler;

        // To make `Janelia.Logger.Log(entry)`'s call to JsonUtility.ToJson() work correctly,
        // the type of `entry` must be marked `[Serlializable]`, but its individual fields need not
        // be marked `[SerializeField]`.  The individual fields must be `public`, though.
        [Serializable]
        internal class Transformation : Logger.Entry
        {
            public Vector3 attemptedTranslation;
            public Vector3 actualTranslation;
            public Vector3 worldPosition;
            public Vector3 rotationDegs;
            public Vector3 worldRotationDegs;

            public void Clear()
            {
                attemptedTranslation.Set(0, 0, 0);
                actualTranslation.Set(0, 0, 0);
                worldPosition.Set(0, 0, 0);
                rotationDegs.Set(0, 0, 0);
                worldRotationDegs.Set(0, 0, 0);
            }

            // Needed only for `KinematicSubject`'s playback of the log.
            public void Set(Transformation other)
            {
                attemptedTranslation = other.attemptedTranslation;
                actualTranslation = other.actualTranslation;
                worldPosition = other.worldPosition;
                rotationDegs = other.rotationDegs;
                worldRotationDegs = other.worldRotationDegs;
            }
        };

        private Transformation _currentTransformation = new Transformation();

        private int _framesSinceLogWrite = 0;
        private int _framesBeingStill = 0;

        private bool _playbackActive = false;
        private string _playbackLogFile = "";
        private List<Transformation> _playbackLogEntries;
        private int _playbackLogIndex;
        private int _playbackStartFrame;
    }
}
