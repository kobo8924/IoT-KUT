
/*2017/11/10
* UV Sensor
* Written by KOBO
*
*/

using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace PinKit
{
    public class Uv
    {
        //�A�i���O���͂̃`�����l��
        static Cpu.AnalogChannel aiChannel = (Cpu.AnalogChannel)7;   // P1_15, A5
        AnalogInput aiThermistor;

        //private int _analog_pin;


        /**
         * �R���X�g���N�^
         */
        public Uv()
        {
            aiThermistor = new AnalogInput(aiChannel);
        }

        /*UV�l�@�v��*/
        public float TakeMeasurement()
        {
            float uv;
            var data = aiThermistor.ReadRaw();

            uv = (float)data * 5 / 1024;
            uv = 25 * (uv - 1) / 3;

            if (uv < 0) {
               uv = 0;
            }

            return (uv);

        }
    }
}
