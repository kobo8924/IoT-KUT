using System;
using Microsoft.SPOT;
using System.Threading;
using Microsoft.SPOT.Time;
using System.Net;
using System.IO;

namespace PinKit
{
    public class PinKit
    {
        /**
         * メンバ
         * 各センサクラスのオブジェクトを定義
         */
        private Accelerometer accelerometer;
        private Temperature temperature;
        private BoardFullColorLED led;
        

        //追加　
        private Pressure pressure;//気圧センサ
        private Proximity proximity;//近接・照明センサ
        private Geomagnetism geomagnetism;//3軸地磁気センサ
        private Uv uv;// 紫外線センサ


        //コンストラクタ
        public PinKit()
        {
            /*各センサクラスのオブジェクトを作成*/
            accelerometer = new Accelerometer();
            temperature = new Temperature();
            led = new BoardFullColorLED();

            //追加
            //気圧センサ
            pressure = new Pressure();
            //近接・照明センサ
            proximity = new Proximity();

            //3軸地磁気センサ
            //引数デバイスアドレス(0x0E or 0x0F)
            //引数なし：初期値0x0Eが設定される
            geomagnetism = new Geomagnetism();

            //紫外線センサ
            uv = new Uv();



#if (USE_LIGHTSENSE)
            lightSensor = new LightSensor();
#endif
        }

        private string ipAddress = "";
        public string SetupNetwork()
        {
            foreach (var ni in Microsoft.SPOT.Net.NetworkInformation.NetworkInterface.GetAllNetworkInterfaces())
            {
                if (ni.NetworkInterfaceType == Microsoft.SPOT.Net.NetworkInformation.NetworkInterfaceType.Ethernet)
                {
                    if (!ni.IsDhcpEnabled)
                    {
                        ni.EnableDhcp();
                        Thread.Sleep(1000);
                    }
                    ipAddress = ni.IPAddress;
                    int count = 0;
                    while (ipAddress == "0.0.0.0" & count++ < 10)
                    {
                        ni.RenewDhcpLease();
                        Thread.Sleep(1000);
                        ipAddress = ni.IPAddress;
                    }
                    if (ipAddress != "0.0.0.0")
                    {
                        if (!ni.IsDynamicDnsEnabled)
                        {
                            ni.EnableDynamicDns();
                        }
                    }
                    Debug.Print("Network Connected - " + ipAddress);
                    foreach (var pi in ni.PhysicalAddress)
                    {
                        deviceName += "." + Byte2Hex(pi);
                    }
                    break;
                }
            }
            return ipAddress;
        }

        private string Byte2Hex(byte p)
        {
            string value = "";
            string[] v = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F" };
            value = v[p >> 4];
            value += v[p & 0x0f];
            return value;
        }

        //      private static byte[] TimeServerIPAddress = new byte[] { 133, 243, 238, 243 };
        private static byte[] TimeServerIPAddress = new byte[] { 59, 157, 6, 14 };
        public bool SyncTimeService()
        {
#if false
            var completed = false;
            if (this.IsNetworkConnected)
            {
                TimeService.SystemTimeChanged += TimeService_SystemTimeChanged;
                TimeServiceStatus status = TimeService.UpdateNow(TimeServerIPAddress, 10);
                TimeService.SetTimeZoneOffset(540); // time origin
            }
            int count = 0;
            while (count++ < 5)
            {
                var flag = false;
                lock (this)
                {
                    flag = hasTimeFixed;
                }
                if (flag)
                {
                    completed = true;
                    break;
                }
                Thread.Sleep(1000);
            }
            if (!completed)
            {
                completed = GetTimeFromWeb();
            }
#else
            var completed = GrFamily.Utility.SystemTimeInitializer.InitSystemTime();

#endif
            return completed;
        }

        private bool GetTimeFromWeb()
        {
            var ticks = DateTime.Now.Ticks;
            var utcTicks = DateTime.UtcNow.Ticks;
            Thread.Sleep(1000);
            var aticks = DateTime.Now.Ticks;
            var delta = aticks - ticks;
            bool result = false;
            try
            {
                using (var request = HttpWebRequest.Create("http://egdecode.azurewebsites.net/api/Time?baseDate=1601-1-1T00:00:00.000Z") as HttpWebRequest)
                {
                    using (var response = request.GetResponse() as HttpWebResponse)
                    {
                        if (response.StatusCode == HttpStatusCode.OK)
                        {
                            using (var reader = new StreamReader(response.GetResponseStream()))
                            {
                                var content = reader.ReadToEnd();
                                content = content.Substring(1, content.Length - 2);
                                try
                                {
                                    TimeService.SetUtcTime(long.Parse(content));
                                }
                                catch (Exception ex)
                                {
                                    Debug.Print("SetUtcTime Failed. - " + ex.Message);
                                }
                            }
                        }
                    }
                    result = true;
                }
                result = true;
            }
            catch (Exception ex)
            {
                Debug.Print("GetTimeFromWeb - " + ex.Message);
            }
            return result;
        }

        private bool hasTimeFixed = false;
        private void TimeService_SystemTimeChanged(object sender, SystemTimeChangedEventArgs e)
        {
            lock (this)
            {
                hasTimeFixed = true;
            }
            Debug.Print("Time Fixed");
        }

        /**
         * 9つのセンサに対応したクラスを作成し、ココにgetメソッドを追加する
         *
         **/
        private string deviceName = "PinKit";
        public string DeviceName { get { return deviceName; } }
        public bool IsNetworkConnected { get { return ipAddress != "0.0.0.0"; } }
        public string IPAddress { get { return ipAddress; } }
        public Accelerometer Accelerometer { get { return accelerometer; } }
        public Temperature Temperature { get { return temperature; } }
        public BoardFullColorLED LED { get { return led; } }

        //追加
        public Pressure Pressure { get { return pressure; } } //気圧センサ
        public Proximity Proximity { get { return proximity; } }//近接・照明センサ
        public Geomagnetism Geomagnetism { get { return geomagnetism; } }//3軸地磁気センサ
        public Uv Uv { get { return uv; } }                              //紫外線センサ

#if (USE_LIGHTSENSE)
        private LightSensor lightSensor;
        public LightSensor LightSensor { get { return lightSensor; } }        
#endif
    }
}
