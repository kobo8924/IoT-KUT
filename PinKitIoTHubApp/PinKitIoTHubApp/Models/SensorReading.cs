using System;
using Microsoft.SPOT;

namespace PinKitIoTApp.Models
{
    public class SensorReading
    {
        /*�e�Z���T�̒l��SensorReading�N���X����쐬�����I�u�W�F�N�g��
         �ۑ����邽�߂ɁA�e�Z���T�ɑΉ����������o���쐬����*/
        // {get; set;}��C#�̃v���p�e�B

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

        //�ߐځE�Ɠx�Z���T
        public ushort proximity { get; set; }
        public float ambient_light { get; set; }

        //���O���Z���T
        public float uv { get; set; }

        //3���n���C�Z���T�@�ϐ���3�K�v�H
        //��������geomagnetism��z��ɂ���
        public float geomagnetism { get; set; }

#if (USE_LIGHTSENSE)
        public double Brightness { get; set; }
#endif
    }
}
