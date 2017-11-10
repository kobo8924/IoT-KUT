using System;
using Microsoft.SPOT;

namespace PinKitIoTApp.Models
{
    public class SensorReading
    {
        /*各センサの値をSensorReadingクラスから作成したオブジェクトに
         保存するために、各センサに対応したメンバを作成する*/
        // {get; set;}はC#のプロパティ

        /// <summary>
        /// Temperature
        /// </summary>
        public double temp { get; set; }
        /// <summary>
        /// Acceleration X
        /// </summary>
        public double accelx { get; set; }
        /// <summary>
        /// Acceleration Y
        /// </summary>
        public double accely { get; set; }
        /// <summary>
        /// Acceleration Z
        /// </summary>
        public double accelz { get; set; }
        /// <summary>
        /// Measured Time
        /// </summary>
        public DateTime time { get; set; }
        public double Latitude { get; set; }
        public double Longitude { get; set; }

        /// <summary>
        /// Pressure
        /// </summary>
        public float pressure { get; set; }
        /// <summary>
        /// Proximity, Ambient_light
        /// </summary>

        //近接・照度センサ
        public ushort proximity { get; set; }
        public float ambient_light { get; set; }

        //紫外線センサ
        public float uv { get; set; }

        //3軸地磁気センサ　変数が3つ必要？
        //もしくはgeomagnetismを配列にする
        public float geomagnetism { get; set; }

#if (USE_LIGHTSENSE)
        public double Brightness { get; set; }
#endif
    }
}
