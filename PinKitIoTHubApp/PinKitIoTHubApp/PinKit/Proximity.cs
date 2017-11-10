
/**
 * 2017/09/11
 * Written by kobo
 * 
 */

//RegRead()デバッグ用マクロ
#define DEBUG_REGREAD


using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;



namespace PinKit
{
    /**
     * 近接・照度センサ
     * ROHM RPR0521RS
     * 
     */
    public class Proximity
    {
        private const byte RPR0521RS_DEVICE_ADDRESS = (0x38);    // 7bit Addrss
        private const byte RPR0521RS_PART_ID_VAL = (0x0A);
        private const byte RPR0521RS_MANUFACT_ID_VAL = (0xE0);

        private const byte RPR0521RS_SYSTEM_CONTROL = (0x40);
        private const byte RPR0521RS_MODE_CONTROL = (0x41);
        private const byte RPR0521RS_ALS_PS_CONTROL = (0x42);
        private const byte RPR0521RS_PS_CONTROL = (0x43);
        private const byte RPR0521RS_PS_DATA_LSB = (0x44);
        private const byte RPR0521RS_ALS_DATA0_LSB = (0x46);
        private const byte RPR0521RS_MANUFACT_ID = (0x92);

        private const byte RPR0521RS_MODE_CONTROL_MEASTIME_100_100MS = (6 << 0);
        private const byte RPR0521RS_MODE_CONTROL_PS_EN = (1 << 6);
        private const byte RPR0521RS_MODE_CONTROL_ALS_EN = (1 << 7);

        private const byte RPR0521RS_ALS_PS_CONTROL_LED_CURRENT_100MA = (2 << 0);
        private const byte RPR0521RS_ALS_PS_CONTROL_DATA1_GAIN_X1 = (0 << 2);
        private const byte RPR0521RS_ALS_PS_CONTROL_DATA0_GAIN_X1 = (0 << 4);

        private const byte RPR0521RS_PS_CONTROL_PS_GAINX1 = (0 << 4);

        private const byte RPR0521RS_MODE_CONTROL_VAL = (RPR0521RS_MODE_CONTROL_MEASTIME_100_100MS | RPR0521RS_MODE_CONTROL_PS_EN | RPR0521RS_MODE_CONTROL_ALS_EN);
        private const byte RPR0521RS_ALS_PS_CONTROL_VAL = (RPR0521RS_ALS_PS_CONTROL_DATA0_GAIN_X1 | RPR0521RS_ALS_PS_CONTROL_DATA1_GAIN_X1 | RPR0521RS_ALS_PS_CONTROL_LED_CURRENT_100MA);
        private const byte RPR0521RS_PS_CONTROL_VAL = (RPR0521RS_PS_CONTROL_PS_GAINX1);

        private const int RPR0521RS_NEAR_THRESH = (1000); // example value
        private const byte RPR0521RS_FAR_VAL = (0);
        private const byte RPR0521RS_NEAR_VAL = (1);

        // byte型だとなぜかERRORになる
        const int RPR0521RS_ERROR = (-1);

        private ushort _als_data0_gain;
        private ushort _als_data1_gain;
        private ushort _als_measure_time;

        private int timeout = 1000;
        private I2CDevice i2c;               //デバイスアドレスはコンストラクタにてコンフィギュレーション
        private byte[] adata = new byte[1];  //読み出しアドレス
        private byte[] wdata = new byte[2];  //書き込みアドレス(1byte)、書き込みデータ(1byte)

        private I2CDevice.I2CTransaction[] trRegRead;
        private I2CDevice.I2CTransaction[] trRegWrite;


        /**
        * コンストラクタ
        * Caller : クラスPinKitからインスタンス化
        */
        public Proximity(int clockRate = 100, int timeout = 10000)
        {
            this.timeout = timeout;
            //デバイスコンフィギュレーション
            i2c = new I2CDevice(new I2CDevice.Configuration((ushort)RPR0521RS_DEVICE_ADDRESS, clockRate));

            this.setup();

            //テスト用メソッド
            //TakeMeasurement();

        }

        /**
         * デバイスのセットアップ
         * init()
         * 1,Part IDレジスタの読み出し及び確認
         * 2,MANUFACT IDレジスタの読み出しと確認
         * 3,ALS_PS_aCONTROLレジスタへ書き込み(ALS DATA0:gain x1, ALS DATA1: Gain x1, 100mA)
         * 4,PS_CONTROLレジスタへ書き込み(PS Gain x1)
         * 5,MODE_CONTROLレジスタへ書き込み(ALS:ON,PS:ON,PS LED pulse = 200us, ALS:100ms,PS:100ms)
         * 戻り値: rc (0以上なら成功、セットアップ0なら失敗)
         */
        public byte setup()
        {
            byte rc;
            byte val;
            byte[] reg = new byte[1];
            byte index;
            byte[] als_gain_table = new byte[] { 1, 2, 64, 128 };
            ushort[] als_meas_time_table = new ushort[] { 0, 0, 0, 0, 0, 100, 100, 100, 100, 100, 400, 400, 50, 0, 0, 0 };

            Debug.Print("\n########## SetUp Proximity Sensor ##########");

            //IDレジスタを読み出しと確認。戻り値：転送したバイト数
            Debug.Print("=== Read ID_Rgister ===");
            rc = RegRead(RPR0521RS_SYSTEM_CONTROL, ref reg);
            if (rc == 0)
            {
                Debug.Print("Can't access RPR0521RS");
                return rc;
            }
            val = (byte)(reg[0] & 0x3F);
            Debug.Print("RPR0521RS Part ID Register Value = 0x" + val.ToString("X2") + "\n");
            if (val != RPR0521RS_PART_ID_VAL)
            {
                Debug.Print("Can't find RPR0521RS");
                return rc;
            }

            // MANUFACT IDレジスタの読み出しと確認
            Debug.Print("\n=== Read MANUFACT_ID_Rgister ===");
            rc = RegRead(RPR0521RS_MANUFACT_ID, ref reg);
            if (rc == 0)
            {
                Debug.Print("Can't access RPR0521RS");
                return rc;
            }
            val = reg[0];
            Debug.Print("RPR0521RS MANUFACT_ID Register Value = 0x" + val.ToString("X2") + "\n");
            if (val != RPR0521RS_MANUFACT_ID_VAL)
            {
                Debug.Print("Can't find RPR0521RS");
                return rc;
            }

            // ALS_PS_CONTROLレジスタへ書き込み(ALS DATA0:gain x1, ALS DATA1: Gain x1, 100mA)
            Debug.Print("\n=== Write ALS_PS_CONTROL_Rgister ===");
            val = RPR0521RS_ALS_PS_CONTROL_VAL;
            rc = RegWrite(RPR0521RS_ALS_PS_CONTROL, val);
            if (rc == 0)
            {
                Debug.Print("Can't write RPR0521RS ALS_PS_CONTROL register");
                return (rc);
            }

            reg[0] = 0;
            rc = RegRead(RPR0521RS_PS_CONTROL, ref reg); //値が正しく書き込めているか確認
            if (rc == 0)
            {
                Debug.Print("Can't read RPR0521RS PS_CONTROL register");
                return rc;
            }

            // PS_CONTROLレジスタへ書き込み(PS Gain x1)
            Debug.Print("\n=== Write PS_CONTORL_Register ===");
            val = reg[0];
            val |= RPR0521RS_PS_CONTROL_VAL;
            rc = RegWrite(RPR0521RS_PS_CONTROL, val);
            if (rc == 0)
            {
                Debug.Print("Can't write RPR0521RS PS_CONTROL register");
            }

            // MODE_CONTROLレジスタへ書き込み(ALS:ON,PS:ON,PS LED pulse = 200us, ALS:100ms,PS:100ms)
            Debug.Print("\n=== Write MODE_CONTORL_Register ===");
            val = RPR0521RS_MODE_CONTROL_VAL;
            rc = RegWrite(RPR0521RS_MODE_CONTROL, val);
            if (rc == 0)
            {
                Debug.Print("Can't write RPR0521RS MODE CONTROL register");
                return rc;

            }

            val = RPR0521RS_ALS_PS_CONTROL_VAL;
            index = (byte)((val >> 4) & 0x03);
            _als_data0_gain = als_gain_table[index];
            index = (byte)((val >> 2) & 0x03);
            _als_data1_gain = als_gain_table[index];

            index = RPR0521RS_MODE_CONTROL_VAL & 0x0F;
            _als_measure_time = als_meas_time_table[index];

            Debug.Print("########## SetUp Complete!! ##########\n");
            Debug.Print("########## END SetUp Proximity Sensor ##########\n");
            return rc;
        }

        /**
        * get_rawpsalsval()
        * 近接、照度の測定データ6バイト取得(アドレス0x44から6バイト)
        */
        byte get_rawpsalsval(ref byte[] data)
        {
            byte rc = 0;
            rc = RegRead(RPR0521RS_PS_DATA_LSB, ref data);
            if (rc == 0)
                Debug.Print("Can't get RPR0521RS PS/ALS_DATA value");

            return rc;
        }

        /**
        * get_psalsval()
        * get_rawpsalsval関数の実行
        * convert_lx関数の実行
        * 近接値{LSB}はそのまま、照度値は[|x]変換
        * 戻り値：レジスタへ転送したバイト数
        */
        byte get_psalsval(ref ushort ps, ref float als)
        {
            byte rc;
            byte[] val = new byte[6];
            ushort rawps;
            ushort[] rawals = new ushort[2];

            rc = get_rawpsalsval(ref val);
            if (rc == 0)
            {
                return (rc);
            }
            /*
              Original code
                        rawps     = ((ushort)val[1] << 8) | val[0];
                        rawals[0] = ((ushort)val[3] << 8) | val[2];
                        rawals[1] = ((ushort)val[5] << 8) | val[4];
            */

            rawps = (ushort)(((ushort)val[1] << 8) | val[0]);
            rawals[0] = (ushort)(((ushort)val[3] << 8) | val[2]);
            rawals[1] = (ushort)(((ushort)val[5] << 8) | val[4]);

            //近接値
            ps = rawps;

            //レジスタ値から照度値を計算
            als = convert_lx(ref rawals);

            return rc;

        }

        /**
        * check_near_far()
        * 近接値がある値（サンプルでは3000）以上の場合に「Near」、ある値未満の場合は「Far」を返す
        */
        byte check_near_far(ushort data)
        {
            if (data >= RPR0521RS_NEAR_THRESH)
            {
                return (RPR0521RS_NEAR_VAL);
            }
            else
            {
                return (RPR0521RS_FAR_VAL);
            }

        }

        /*
        * convert_lx()
        * レジスタ値から照度値[lx]変換
        * 戻り値:照度値(lx)
        */
        float convert_lx(ref ushort[] data)
        {
            float lx;
            float d0, d1, d1_d0;

            if (_als_data0_gain == 0)
            {
                return (RPR0521RS_ERROR);
            }

            if (_als_data1_gain == 0)
            {
                return (RPR0521RS_ERROR);
            }

            if (_als_measure_time == 0)
            {
                return (RPR0521RS_ERROR);
            }
            else if (_als_measure_time == 50)
            {
                if ((data[0] & 0x8000) == 0x8000)
                {
                    data[0] = 0x7FFF;
                }
                if ((data[1] & 0x8000) == 0x8000)
                {
                    data[1] = 0x7FFF;
                }
            }

            d0 = (float)data[0] * (100 / _als_measure_time) / _als_data0_gain;
            d1 = (float)data[1] * (100 / _als_measure_time) / _als_data1_gain;

            if (d0 == 0)
            {
                lx = 0;
                return (lx);
            }

            d1_d0 = d1 / d0;

            if (d1_d0 < 0.595)
            {
                lx = (float)(1.682 * d0 - 1.877 * d1);
            }
            else if (d1_d0 < 1.015)
            {
                lx = (float)(0.644 * d0 - 0.132 * d1);
            }
            else if (d1_d0 < 1.352)
            {
                lx = (float)(0.756 * d0 - 0.243 * d1);
            }
            else if (d1_d0 < 3.053)
            {
                lx = (float)(0.766 * d0 - 0.25 * d1);
            }
            else
            {
                lx = 0;
            }

            return (lx);

        }


        /**
         * RegRead(byte, ref byte[])
         * レジスタからデータを読み取る
         * addr : デバイスに送信されるバイトの配列(アドレス)
         * rdata : デバイスから読み取られたデータを格納
         * 戻り値:レジスタへ転送したバイト数
         */
        public byte RegRead(byte addr, ref byte[] rdata)
        {
            adata[0] = addr;
            trRegRead = new I2CDevice.I2CTransaction[] {
                   I2CDevice.CreateWriteTransaction(adata),   //デバイスへアドレスを転送
                   I2CDevice.CreateReadTransaction(rdata) };  //デバイスよりデータを読み込み
            int rc = i2c.Execute(trRegRead, timeout);      //戻り値:転送されたバイト数
#if DEBUG_REGREAD
            Debug.Print("Reg Read : result = " + rc);
            Debug.Print("R[0x" + adata[0].ToString("X2") + "]=0x" + rdata[0].ToString("X2"));
#endif
            return (byte)rc;
        }


        /**
         * RegWrite(byte reg, byte val)
         * レジスタへデータを書き込み
         * reg : 書き込み先アドレス
         * val : 書き込みデータ
         * 戻り値:レジスタへ転送したバイト数
         */
        public byte RegWrite(byte reg, byte val)
        {
            wdata[0] = reg; //書き込み先アドレス
            wdata[1] = val; //書き込みデータ
            trRegWrite = new I2CDevice.I2CTransaction[] { I2CDevice.CreateWriteTransaction(wdata) };
            int rc = i2c.Execute(trRegWrite, timeout); //戻り値:転送されたバイト数
            Debug.Print("Reg Write : Number of Transfer Data = " + rc); //値が書き込めているか確認
            return (byte)rc;
        }

        /**
         * 近接値と照度を計測
         * 戻り値:近接値、照度値を格納したSensorReadingオブジェクト
         */
        public SensorReading TakeMeasurement()
        {
            byte rc;
            byte near_far;

            //近接値　
            ushort ps_val = 0;
            //照度
            float als_val = 0;

            Debug.Print("=== TakeMeasurement Proximity & Ambient Light ===");

            rc = get_psalsval(ref ps_val, ref als_val);
            if (rc != 0)
            {
                //近接
                Debug.Print("RPR-0521RS (Proximity) = " + ps_val + "[count]");

                near_far = check_near_far(ps_val);

                if (near_far == RPR0521RS_NEAR_VAL)
                {
                    Debug.Print(" Near");
                }
                else
                {
                    Debug.Print(" Far");
                }

                if (als_val != RPR0521RS_ERROR)
                {
                    //照度　
                    Debug.Print("RPR-0521RS (Ambient Light) = " + als_val + "[lx]\n");
                }
            }
            Debug.Print("=== END ===");

            //遅延 0.5秒
            System.Threading.Thread.Sleep(500);

            //SensorReadingオブジェクトを作成し、近接値、照度値を格納して返す
            return new SensorReading()
            {
                proximity = ps_val,
                ambient_light = als_val
            };

        }
        //近接値、照度値、2つの値を返すためにサブクラスを作成
        public class SensorReading
        {
            public ushort proximity { get; set; }
            public float ambient_light { get; set; }
        }


    }
}
