using System;
using System.Collections.Generic;
using UnityEngine;

namespace Janelia
{
    using System;

    public class CircularStatistics
    {
        // Function to calculate the circular mean of an array of angles in degrees
        public static float CircularMean(float[] angles)
        {
            float sinSum = 0.0f;
            float cosSum = 0.0f;

            // Summing up the sine and cosine of all angles, converting from degrees to radians
            foreach (float angle in angles)
            {
                // Convert angle from degrees to radians
                float radian = angle * (float)Math.PI / 180.0f;
                sinSum += (float)Math.Sin(radian);
                cosSum += (float)Math.Cos(radian);
            }

            // Averaging the sine and cosine values
            float avgSin = sinSum / angles.Length;
            float avgCos = cosSum / angles.Length;

            // Calculating the circular mean in radians
            float circularMeanRadians = (float)Math.Atan2(avgSin, avgCos);

            // Convert the circular mean from radians back to degrees
            float circularMeanDegrees = circularMeanRadians * (180 / (float)Math.PI);

            // Ensure the result is within [0, 360] degrees
            if (circularMeanDegrees < 0)
            {
                circularMeanDegrees += 360;
            }

            return circularMeanDegrees;
        }
    }

    // Example of usage:
    class Program
    {
        static void Main()
        {
            // Example input in degrees
            float[] angles = { 180.0f, 270.0f };

            // Calculate the circular mean
            float meanAngle = CircularStatistics.CircularMean(angles);

            // Output the result
            Console.WriteLine("Circular Mean: " + meanAngle + " degrees");
        }
    }

    public class FicTracAverager : MonoBehaviour
    {
        public static void StartAveragingHeading(GameObject obj, int windowInFrames)
        {
            FicTracAverager averager = obj.GetComponent<FicTracAverager>();
            if (obj != null)
            {
                averager.StartAveragingHeading(windowInFrames);
            }
        }

        public static float GetAverageHeading(GameObject obj)
        {
            float result = 0;
            FicTracAverager averager = obj.GetComponent<FicTracAverager>();
            if (obj != null)
            {
                result = averager.GetAverageHeading();
            }
            return result;
        }

        // Seems to be called before the event handlers added to `Application.quitting`.
        // The `Logger` class has one of those event handlers to write the log a final time.
        public void OnApplicationQuit()
        {
            // When the session ends, force computation and storage.
            float mean = GetAverageHeading();

            if (_started)
            {
                Debug.Log("Janelia.FicTracAverager stored mean heading " + mean);
            }
        }

        private void StartAveragingHeading(int windowInFrames)
        {
            _window = windowInFrames;
            _headings = new float[_window];
            Array.Clear(_headings, 0, _window);
            _index = 0;
            _started = true;
            _dirty = true;
        }

        internal void RecordHeading(float heading)
        {
            if (_started)
            {
                _headings[_index] = heading;
                _index = (_index + 1) % _window;
                _dirty = true;
            }
        }

        private float GetAverageHeading()
        {
            if (!_dirty)
            {
                return _mean;
            }
            if (!_started)
            {
                _mean = RestoreMeanHeading();
                _dirty = false;
                return _mean;
            }

            _mean = CircularStatistics.CircularMean(_headings);
            Console.WriteLine(_headings);
            StoreMeanHeading(_mean);
            _dirty = false;
            return _mean;
        }

        private void StoreMeanHeading(float mean)
        {
            string playerPrefsKey = "janelia-unity-toolkit.FicTracAverager." + PathName() + ".meanHeadingDegs";
            PlayerPrefs.SetFloat(playerPrefsKey, mean);
            _currentStored.storedMeanHeadingDegs = mean;
            Logger.Log(_currentStored);
        }

        private float RestoreMeanHeading()
        {
            string playerPrefsKey = "janelia-unity-toolkit.FicTracAverager." + PathName() + ".meanHeadingDegs";
            float mean = PlayerPrefs.GetFloat(playerPrefsKey, 0);
            _currentRestored.restoredMeanHeadingDegs = mean;
            Logger.Log(_currentRestored);
            return mean;
        }

        private string PathName()
        {
            GameObject o = gameObject;
            string path = o.name;
            while (o.transform.parent != null)
            {
                o = o.transform.parent.gameObject;
                path = o.name + "-" + path;
            }
            return path;
        }

        [Serializable]
        internal class Stored : Logger.Entry
        {
            public float storedMeanHeadingDegs;
        };
        private Stored _currentStored = new Stored();

        [Serializable]
        internal class Restored : Logger.Entry
        {
            public float restoredMeanHeadingDegs;
        };
        private Restored _currentRestored = new Restored();

        private int _window;
        private float[] _headings;
        private int _index = 0;
        private bool _started = false;
        private float _mean = 0;
        private bool _dirty = true;
    }
}
