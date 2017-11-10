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
                myApplication.ProgramInitialize(); //�v���O����������
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
         * �R���X�g���N�^
         **/
        public Program()
        {
            //pinkit�N���X�̃I�u�W�F�N�g���쐬
            pinkit = new PinKit.PinKit();

            //LED(�_��)�̐F���w��
            blinkColor = PinKit.BoardFullColorLED.Colors.Red; 

            //LED��_��
            BlinkLED();
        }

        /**
         * LED�_�ł𐧌�@
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
         * �v���O����������
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

                // LED(�_�Łj���~
                pinkitStatusLEDThread.Suspend();

                // LED(�_�Łj�̐F��΂ɃZ�b�g
                blinkColor = PinKit.BoardFullColorLED.Colors.Green;

                // LED(�_�Łj���ĊJ
                pinkitStatusLEDThread.Resume();

                //IoT�f�o�C�X�ɐڑ�
                TryConnect();

                //LED(�_��)���~
                blinking = false;

                // LED(�_�Łj�̐F��ɃZ�b�g
                pinkit.LED.SetColor(PinKit.BoardFullColorLED.Colors.Blue);

                Initialize();
            }
            return result;
        }
    }
}
