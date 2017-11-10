using System;

using Microsoft.SPOT;
using Microsoft.SPOT.Input;
using Microsoft.SPOT.Presentation;
using Microsoft.SPOT.Presentation.Controls;
using System.Threading;

namespace PinKitIoTHubApp
{
    public partial class Program : Microsoft.SPOT.Application
    {
        public static void Main()
        {
            Program myApplication = new Program();

            Window mainWindow = myApplication.CreateWindow();

            try
            {
                myApplication.ProgramInitialize(); //プログラム初期化
            }
            catch (Exception ex)
            {
                Debug.Print(ex.Message);
            }

            // Start the application
            myApplication.Run(mainWindow);
        }

        private Window mainWindow;

        public Window CreateWindow()
        {
            // Create a window object and set its size to the
            // size of the display.
            mainWindow = new Window();
            mainWindow.Height = SystemMetrics.ScreenHeight;
            mainWindow.Width = SystemMetrics.ScreenWidth;

            return mainWindow;
        }

        PinKit.BoardFullColorLED.Colors blinkColor;
        bool blinking = true;


        /**
         * コンストラクタ
         **/
        public Program()
        {
            //pinkitクラスのオブジェクトを作成
            pinkit = new PinKit.PinKit();

            //LED(点滅)の色を指定
            blinkColor = PinKit.BoardFullColorLED.Colors.Red; 

            //LEDを点滅
            BlinkLED();
        }

        /**
         * LED点滅を制御　
         **/
        private void BlinkLED()
        {
            pinkitStatusLEDThread = new Thread(() =>
            {
                bool on = true;
                bool blinkingStatus;
                lock (this)
                {
                    blinkingStatus = blinking;
                }
                while (blinkingStatus)
                {
                    if (on)
                    {
                        pinkit.LED.SetColor(blinkColor);
                        on = false;
                    }
                    else
                    {
                        pinkit.LED.SetColor(PinKit.BoardFullColorLED.Colors.Black);
                        on = true;
                    }
                    Thread.Sleep(500);
                    lock (this)
                    {
                        blinkingStatus = blinking;
                    }
                }
            });
            pinkitStatusLEDThread.Start();
        }

        PinKit.PinKit pinkit;
        Thread pinkitStatusLEDThread;
        string proxyHost = "";
        int proxyPort = 80;
        string IoTDeviceId = "";


        /**
         * プログラム初期化
         **/
        private bool ProgramInitialize()
        {
            bool result = true;

            var ipAddr = pinkit.SetupNetwork();
            if (ipAddr == "0.0.0.0")
            {
                result = false;
            }
            else
            {
                pinkit.SyncTimeService();

                // LED(点滅）を停止
                pinkitStatusLEDThread.Suspend();

                // LED(点滅）の色を緑にセット
                blinkColor = PinKit.BoardFullColorLED.Colors.Green;

                // LED(点滅）を再開
                pinkitStatusLEDThread.Resume();

                //IoTデバイスに接続
                TryConnect();

                //LED(点滅)を停止
                blinking = false;

                // LED(点滅）の色を青にセット
                pinkit.LED.SetColor(PinKit.BoardFullColorLED.Colors.Blue);

                Initialize();
            }
            return result;
        }
    }
}
