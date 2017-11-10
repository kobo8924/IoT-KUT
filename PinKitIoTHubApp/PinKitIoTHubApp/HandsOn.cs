//#define ACCESS_MOBILE_SERVICE
#define ACCESS_IOT_HUB

using System;
using Microsoft.SPOT;
using System.IO;
using System.Net;
using System.Collections;
using System.Threading;
using System.Text;

#if(ACCESS_IOT_HUB)
using Microsoft.Azure.Devices.Client;
#endif

namespace PinKitIoTHubApp
{
    public partial class Program //�����N���X
    {
#if (ACCESS_MOBILE_SERVICE)
        // Device Entry Configuration
        // string DeviceEntryEndPoint = "http://[MobileAppName].azurewebsites.net";

#endif

#if (ACCESS_IOT_HUB)
        // IoT Hub Configuration
        // string ioTHubEndpoint = "[IoTHubName].azure-devices.net"; -> IoTHubConfig.cs
        string deviceKey = "HPZ1MvJepyW1beWt0vSTaLuAWr6RpCkgPSYxCaw6474=";
#endif

        // Identifier of this board. this value will be set by this app.
        String deviceId = "";

        bool IoTServiceAvailabled = false;

        private void TryConnect()
        {
            using (var request = HttpWebRequest.Create("http://egholservice.azurewebsites.net/api/DeviceConnect") as HttpWebRequest)
            {
                if (proxyHost != "")
                {
                    request.Proxy = new WebProxy(proxyHost, proxyPort);
                }

                if (IoTDeviceId == "" && pinkit.DeviceName != "")
                {
                    IoTDeviceId = pinkit.DeviceName;
                    if (IoTDeviceId == "PinKit.1C.00.11.33.44.55")
                    {
                        Debug.Print("Please set valid MAC Address by MFDeploy tool!");
                        //      throw new ArgumentOutOfRangeException("MAC address is default value.");
                        IoTDeviceId = IoTDeviceId.Substring(0, IoTDeviceId.Length - 5) + (DateTime.Now.Ticks % 1000).ToString();
                    }
                    Debug.Print(" OK! Your DeviceId is '" + IoTDeviceId + "'");
                }
                if (IoTDeviceId != "")
                {
                    request.Headers.Add("device-id", IoTDeviceId.ToString());
                    request.Headers.Add("device-message", "hello from pinkit");
                    using (var response = request.GetResponse() as HttpWebResponse)
                    {
                        if (response.StatusCode == HttpStatusCode.OK)
                        {
                            var reader = new StreamReader(response.GetResponseStream());
                            string message = reader.ReadToEnd();
                            Debug.Print(message);
                        }
                    }
                    deviceId = IoTDeviceId;
                    // At Step 1, Please stop this application at the above line.
                }
            }
        }

        /**
         * IoTHub�ւ̃A�b�v���[�h�̏���
         **/
        void InitializeUpload()
        {
            //�}�N��(ACCESS_MOBILE_SERVICE,ACCESS_IOT_HUB)�ɉ�����IoTServiceAvailabled��TRUE�ɂ���B
            //sensorReadings�I�u�W�F�N�g�𐶐�
            EntryDevice();
            if (IoTServiceAvailabled)
            {
                SetupIoTHub();//IotHub�̃Z�b�g�A�b�v
            }
        }

        void Upload()
        {
            Debug.Print("Sending message of " + srCount + " to IoT Hub..." + DateTime.Now.Ticks);
#if (ACCESS_IOT_HUB)

            string content = "";
            try
            {
                lock (this)
                {
                    for (int i = 0; i < srCount; i++)
                    {
                        if (sensorReadings[i] != null)
                        {
                            var measuredTime = sensorReadings[i].time.ToString("yyyy-MM-ddTHH:mm:ss.fffZ");
                            var srjson = "{\"accelx\":" + sensorReadings[i].accelx
                                + ",\"accely\":" + sensorReadings[i].accely
                                + ",\"accelz\":" + sensorReadings[i].accelz
                                + ",\"temp\":" + sensorReadings[i].temp
#if (USE_LIGHTSENSE)
                                + ",\"brightness\":" + sensorReadings[i].Brightness
#endif
                                + ",\"time\":\"" + measuredTime + "\""
                                + ",\"Latitude\":" + sensorReadings[i].Latitude
                                + ",\"Longitude\":" + sensorReadings[i].Longitude + "}";
                            if (content != "")
                            {
                                content += ",";
                            }
                            content += srjson;
                        }
                    }
                    srCount = 0;
                }
                content = "[" + content + "]";
                using (var message = new Message(System.Text.UTF8Encoding.UTF8.GetBytes(content)))
                {
                    deviceClient.SendEvent(message);
                }
                GC.WaitForPendingFinalizers();
                Debug.Print("Send[" + sendRound++ + "] - " + DateTime.Now.Ticks);
                BlinkPinKitLED(PinKit.BoardFullColorLED.Colors.Blue, 1000, 500, 5);
            }
            catch (Exception ex)
            {
                Debug.Print("Send Error - " + ex.Message);
                BlinkPinKitLED(PinKit.BoardFullColorLED.Colors.Red, 1000, 500, 10);
            }
#endif
        }

        DispatcherTimer measureTimer;
        DispatcherTimer uploadTimer;
        long measureIntervalMSec = 2000; // measure interval 1000 msec
        long uploadIntervalMSec = 60000;  // upload interval 1000 msec

        /**
         * ������
         **/
        private void Initialize()
        {
            InitializeUpload(); //IotHub�ւ̃A�b�v���[�h�̏�����

            measureTimer = new DispatcherTimer();
            //�C���^�[�o�����w��
            measureTimer.Interval = TimeSpan.FromTicks(measureIntervalMSec * TimeSpan.TicksPerMillisecond);
            measureTimer.Tick += MeasureTimer_Tick;
            measureTimer.Start();

            uploadTimer = new DispatcherTimer();
            uploadTimer.Interval = TimeSpan.FromTicks(uploadIntervalMSec * TimeSpan.TicksPerMillisecond);
            uploadTimer.Tick += UploadTimer_Tick;
            uploadTimer.Start();
        }

        private void UploadTimer_Tick(object sender, EventArgs e)
        {
            uploadTimer.Stop();
            lock (this)
            {
                Upload();
            }
            uploadTimer.Start();
        }

        /**
         * MeasureTimer_Tick
         * ���x�A�����x���v������sensorReadings[]�Ɋi�[
         **/
        int sendRound = 0;
        private void MeasureTimer_Tick(object sender, EventArgs e)
        {
            measureTimer.Stop();

            lock (this)
            {
                if (srCount < srMax) //srCoutn=0, srMax=30
                {
#if (false)
                    var now = DateTime.Now; //���݂̎���
                    sensorReadings[srCount].temp = pinkit.Temperature.TakeMeasurement(); 
                    sensorReadings[srCount].time = now;
                    sensorReadings[srCount].Latitude = IoTHoLConfig.Latitude;
                    sensorReadings[srCount].Longitude = IoTHoLConfig.Longitude;

                    Debug.Print("+++ Measured[" + sendRound + "][" + srCount + "].temp=" + sensorReadings[srCount].temp);
                    srCount++;
#endif

                    //var:implicitly type �R���p�C�����^�����肷��B
                    var now = DateTime.Now; //���݂̎���
                    // Accelerometer�N���X�ATakeMeasurements()��������x���擾
                    //var accel = pinkit.Accelerometer.TakeMeasurements();
                    var proximity = pinkit.Proximity.TakeMeasurement();
                    // Temparature�N���X�ATakeMeasurement()���牷�x���擾
                    //sensorReadings[srCount].temp = pinkit.Temperature.TakeMeasurement(); 
                    //Pressure�N���X�ATakeMeasurment()����C�����擾
                    sensorReadings[srCount].pressure = pinkit.Pressure.TakeMeasurement();
                    sensorReadings[srCount].uv = pinkit.Uv.TakeMeasurement();

                    /**
                     * ���̑��̃Z���T�Ɋւ��Ă����l�ɃN���X���쐬���Ēl���擾����B
                     */
                    //sensorReadings[srCount].accelx = accel.X;
                    //sensorReadings[srCount].accely = accel.Y;
                    //sensorReadings[srCount].accelz = accel.Z;
                    sensorReadings[srCount].proximity = proximity.proximity;
                    sensorReadings[srCount].ambient_light = proximity.ambient_light;
                    sensorReadings[srCount].time = now;
                    sensorReadings[srCount].Latitude = IoTHoLConfig.Latitude;
                    sensorReadings[srCount].Longitude = IoTHoLConfig.Longitude;
#if (USE_LIGHTSENSE)
                    sensorReadings[srCount].Brightness = pinkit.LightSensor.TakeMeasurement();
#endif
                    /*�\��*/
                    Debug.Print("---------------------------------------------------------------------");
                    // Debug.Print("!!! Measured[" + sendRound + "][" + srCount + "].temp=" + sensorReadings[srCount].temp);
                    Debug.Print("!!! Measured[" + sendRound + "][" + srCount + "].press=" + sensorReadings[srCount].pressure +"[hPa]");
                    Debug.Print("!!! Measured[" + sendRound + "][" + srCount + "].uv=" + sensorReadings[srCount].uv + "[mW/cm2]");
                    Debug.Print("!!! Measured[" + sendRound + "][" + srCount + "].proximity=" + sensorReadings[srCount].proximity +"[count]");
                    Debug.Print("!!! Measured[" + sendRound + "][" + srCount + "].ambient_light=" + sensorReadings[srCount].ambient_light +"[lx]");
                    // Debug.Print("!!! Measured[" + sendRound + "][" + srCount + "].X=" + sensorReadings[srCount].accelx);
                    // Debug.Print("!!! Measured[" + sendRound + "][" + srCount + "].Y=" + sensorReadings[srCount].accely);
                    // Debug.Print("!!! Measured[" + sendRound + "][" + srCount + "].Z=" + sensorReadings[srCount].accelz);
                    Debug.Print("---------------------------------------------------------------------");

                    srCount++;


                }
            }
            measureTimer.Start();
        }

#if (ACCESS_MOBILE_SERVICE)
        EGIoTKit.Utility.SimpleMobileAppsClient mobileService;
        string DeviceEntryTableName = "DeviceEntry";
#endif

        /**
         * 
         */
        private void EntryDevice()
        {
#if (ACCESS_MOBILE_SERVICE)
            if (mobileService == null)
            {
                mobileService = new EGIoTKit.Utility.SimpleMobileAppsClient(IoTHoLConfig.DeviceEntryEndPoint);
                if (proxyHost != null && proxyHost != "")
                {
                    mobileService.ProxyHost = proxyHost;
                    mobileService.ProxyPort = proxyPort;
                }
            }
            var registered = mobileService.Query(DeviceEntryTableName);
            bool registed = false;
            if (registered != null && registered.Count > 0)
            {
                foreach (var re in registered)
                {
                    var registedEntry = re as Hashtable;
                    if ((string)(registedEntry["deviceId"]) == deviceId)
                    {
                        if (registedEntry["serviceAvailable"] != null)
                        {
                            IoTServiceAvailabled = (bool)registedEntry["serviceAvailable"];
                            if (IoTServiceAvailabled)
                            {
                                IoTHoLConfig.IoTHubEndpoint = (string)registedEntry["iotHubEndpoint"];
                                IoTHoLConfig.DeviceKey = (string)registedEntry["deviceKey"];
                                Debug.Print("IoT Hub Service Availabled - IoTHubEndpoint=" + IoTHoLConfig.IoTHubEndpoint + ",deviceKey=" + IoTHoLConfig.DeviceKey);
                            }
                        }
                        registed = true;
                        break;
                    }
                }
            }
            if (!registed)
            {
                var entry = new Models.DeviceEntry()
                {
                    DeviceId = deviceId.ToString(),
                    ServiceAvailable = false,
                    DeviceKey = "",
                    IoTHubEndpoint = ""
                };
                var insertedItem = mobileService.Insert(DeviceEntryTableName, entry);
            }
#else
#if (ACCESS_IOT_HUB)
            IoTServiceAvailabled = true;
#endif
#endif
            /**
             * 9�Z���T���ғ������邽�߂�sensorReadings�N���X���g������K�v������
             **/
            if (IoTServiceAvailabled)
            {
                // srMax = 60000 / 2000
                srMax = (int)(uploadIntervalMSec / measureIntervalMSec);
                // sensorReadings�I�u�W�F�N�g��30(srMax)����
                sensorReadings = new PinKitIoTApp.Models.SensorReading[srMax];
                for (int i = 0; i < srMax; i++)
                {
                    sensorReadings[i] = new PinKitIoTApp.Models.SensorReading();
                }
                srCount = 0;
            }
        }

#if (ACCESS_IOT_HUB)
        DeviceClient deviceClient;
#endif
        string iotHubConnectionString = "";
        PinKitIoTApp.Models.SensorReading[] sensorReadings;
        int srMax;
        int srCount;

        private void SetupIoTHub()
        {
#if (ACCESS_IOT_HUB)
            iotHubConnectionString = "HostName=" + IoTHoLConfig.IoTHubEndpoint + ";DeviceId=" + deviceId + ";SharedAccessKey=" + IoTHoLConfig.DeviceKey;
            try
            {
                deviceClient = DeviceClient.CreateFromConnectionString(iotHubConnectionString, TransportType.Http1);
                BlinkPinKitLED(PinKit.BoardFullColorLED.Colors.Green, 1000, 500, 3);
                ReceiveCommands();
            }
            catch (Exception ex)
            {
                Debug.Print("IoT Hub Connection Failed. - " + ex.Message);
                uploadTimer.Stop();
            }
#endif
        }
#if (ACCESS_IOT_HUB)
        long receiveTimeoutSec = 60;

        private void ReceiveCommands()
        {
            new Thread(() =>
            {
                Hashtable commandColors = new Hashtable();
                commandColors.Add("black", PinKit.BoardFullColorLED.Colors.Black);
                commandColors.Add("red", PinKit.BoardFullColorLED.Colors.Red);
                commandColors.Add("green", PinKit.BoardFullColorLED.Colors.Green);
                commandColors.Add("yellow", PinKit.BoardFullColorLED.Colors.Yellow);
                commandColors.Add("blue", PinKit.BoardFullColorLED.Colors.Blue);
                commandColors.Add("magenta", PinKit.BoardFullColorLED.Colors.Magenta);
                commandColors.Add("cyan", PinKit.BoardFullColorLED.Colors.Cyan);
                commandColors.Add("white", PinKit.BoardFullColorLED.Colors.White);

                Debug.Print("Start to receive command...");
                while (true)
                {
                    try
                    {
                        var receivedMessage = deviceClient.Receive();
                        if (receivedMessage != null)
                        {
                            var buf = receivedMessage.GetBytes();
                            if (buf != null && buf.Length > 0)
                            {
                                var content = new string(System.Text.UTF8Encoding.UTF8.GetChars(buf));
                                Debug.Print(DateTime.Now.ToLocalTime() + "> Received Message:" + content);

                                deviceClient.Complete(receivedMessage);

                                string lContent = content.ToLower();
                                foreach (var ccKey in commandColors.Keys)
                                {
                                    if (lContent.IndexOf((string)ccKey) > 0)
                                    {
                                        pinkit.LED.SetColor((PinKit.BoardFullColorLED.Colors)commandColors[ccKey]);
                                    }
                                }
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        Debug.Print("Receive Error - " + ex.Message);
                        BlinkPinKitLED(PinKit.BoardFullColorLED.Colors.Yellow, 1000, 500, 5);
                        break;
                    }
                }
            }).Start();
        }
#endif

        private void BlinkPinKitLED(PinKit.BoardFullColorLED.Colors color, int onmsec, int offmsec, int blinkCount)
        {
            new Thread(() =>
            {
                lock (this.pinkit)
                {
                    while (blinkCount-- > 0)
                    {
                        pinkit.LED.SetColor(color);
                        Thread.Sleep(onmsec);
                        pinkit.LED.SetColor(PinKit.BoardFullColorLED.Colors.Black);
                        Thread.Sleep(offmsec);
                    }
                }
            }).Start();
        }

    }
}
