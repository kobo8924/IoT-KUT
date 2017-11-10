
/**
 * 2017/09/11
 * Written by kobo
 * 課題:外部割込みが生じた際に実行するメソッドの実装
 *     :volatile就職子のコンパイルエラー
 */

//RegRead()デバッグ用マクロ
#define DEBUG_REGREAD


using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

//volatile修飾子を使うときに必要かも（？）
//using System.Runtime.CompilerServices.IsVolatile;


namespace PinKit
{
    /**
     * 3軸地磁気センサ
     * ROHM BM1422GMV-ZE2
     * 
     */
    public class Geomagnetism
    {
        private const byte BM1422_DEVICE_ADDRESS_0E = (0x0E);    // 7bit Addrss
        private const byte BM1422_DEVICE_ADDRESS_0F = (0x0F);    // 7bit Address
        private const byte BM1422_WIA_VAL = (0x41);

        private const byte BM1422_WIA = (0x0F);
        private const byte BM1422_DATAX = (0x10);
        private const byte BM1422_STA1 = (0x18);
        private const byte BM1422_CNTL1 = (0x1B);
        private const byte BM1422_CNTL2 = (0x1C);
        private const byte BM1422_CNTL3 = (0x1D);
        private const byte BM1422_AVE_A = (0x40);
        private const byte BM1422_CNTL4 = (0x5C);

        private const byte BM1422_STA1_RD_DRDY = (1 << 6);

        private const byte BM1422_CNTL1_FS1 = (1 << 1);
        private const byte BM1422_CNTL1_ODR_10Hz = (0 << 3);
        private const byte BM1422_CNTL1_RST_LV = (1 << 5);
        private const byte BM1422_CNTL1_OUT_BIT = (1 << 6);
        private const byte BM1422_CNTL1_PC1 = (1 << 7);

        private const byte BM1422_CNTL2_DRP = (1 << 2);
        private const byte BM1422_CNTL2_DREN = (1 << 3);

        private const byte BM1422_CNTL3_FORCE = (1 << 6);

        private const byte BM1422_AVE_A_AVE4 = (0 << 2);

        private const byte BM1422_CNTL1_VAL = (BM1422_CNTL1_FS1 | BM1422_CNTL1_OUT_BIT | BM1422_CNTL1_PC1);
        private const byte BM1422_CNTL2_VAL = (BM1422_CNTL2_DREN);
        private const byte BM1422_CNTL3_VAL = (BM1422_CNTL3_FORCE);
        private const short BM1422_CNTL4_VAL = (0x0000);
        private const byte BM1422_AVE_A_VAL = (BM1422_AVE_A_AVE4);

        private const byte BM1422_14BIT_SENS = (24);
        private const byte BM1422_12BIT_SENS = (6);


        private int _device_address;
        private byte _sens;

        //volatile修飾子を使うとコンパイルエラー（未解決）
        //mscorlib.dllが関係あるかも
        //private volatile int _drdy_flg;
        private int _drdy_flg;




        private int timeout = 1000;
        private I2CDevice i2c;               //デバイスアドレスはコンストラクタにてコンフィギュレーション
        private byte[] adata = new byte[1];  //読み出しアドレス
        private byte[] wdata = new byte[2];  //書き込みアドレス(1byte)、書き込みデータ(1byte)

        private I2CDevice.I2CTransaction[] trRegRead;
        private I2CDevice.I2CTransaction[] trRegWrite;


        /**
        * コンストラクタ
        * Caller : クラスPinKitからインスタンス化
        * 引数に指定されたデバイスアドレスを内部変数に保持(0x0E or 0x0F)
        */
        public Geomagnetism(int slave_address = 0x0E, int clockRate = 100, int timeout = 10000)
        {
            byte rc = 0;
            _device_address = slave_address;
            this.timeout = timeout;
            //デバイスコンフィギュレーション
            i2c = new I2CDevice(new I2CDevice.Configuration((ushort)_device_address, clockRate));

            rc = this.setup();
            if (rc == 0)
                Debug.Print("BM1422 initialization failed");

            //割り込み関数コース（未実装）
            //あとは割り込みの実装のみ
            //isr_func(0, bm1422_isr);

            //テスト用メソッド
            //TakeMeasurement();

        }

        /*
        * init():デバイスのセットアップ
        * 1,DRDYフラグの初期化
        * 2,WIAレジスタの読み出し
        * 3,CNTL1レジスタへ書き込み（PC1=1,14bitモード,シングル測定モード）
        * 4,CNTL4レジスタへ書き込み（リセット解除）
        * 5,CNTL2レジスタへ書き込み（DREN=1,DRDY端子のEnable）
        * 6,AVE_Aレジスタへ書き込み（平均回数4回）
        */
        public byte setup()
        {
            byte rc = 0;
            byte val = 0;
            byte[] reg = new byte[1];
            byte[] buf = new byte[2];

            Debug.Print("\n########## SetUp Geomagnetism Sensor ##########");

            // 1,DRDYフラグの初期化
            _drdy_flg = 0;

            //2,WIAレジスタの読み出し
            Debug.Print("=== Read WIA_Rgister ===");
            rc = RegRead(BM1422_WIA, ref reg);
            val = reg[0];
            if (rc == 0)
            {
                Debug.Print("Can't access BM1422");
                return (rc);
            }
            Debug.Print("BM1422_WIA Register Value = 0x" + val.ToString("X2"));

            if (val != BM1422_WIA_VAL)
            {
                Debug.Print("Can't find BM1422");
                return (rc);
            }

            //3,CNTL1レジスタへ書き込み（PC1=1,14bitモード,シングル測定モード）
            Debug.Print("\n=== Write CNTL1 Register ===");
            val = BM1422_CNTL1_VAL;
            rc = RegWrite(BM1422_CNTL1, val);
            if (rc == 0)
            {
                Debug.Print("Can't write BM1422_CNTL1 Register");
                return (rc);
            }
            //正しく書き込めているか確認
            //rc = RegRead(BM1422_CNTL1, ref reg);

            // Check 12bit or 14bit
            buf[0] = (BM1422_CNTL1_VAL & BM1422_CNTL1_OUT_BIT);
            if (buf[0] == BM1422_CNTL1_OUT_BIT)
            {
                _sens = BM1422_14BIT_SENS;
            }
            else
            {
                _sens = BM1422_12BIT_SENS;
            }
            //遅延 1ms
            System.Threading.Thread.Sleep(1);


            //4,CNTL4レジスタへ書き込み（リセット解除）
            // CNTL4レジスタはbit幅16bit(0x5C / 0x5D)
            //RegWriteを2回コールしている。nバイト書き込めるメソッドで処理すべきか？
            Debug.Print("\n=== Write CNTL4 Register ===");
            buf[0] = (BM1422_CNTL4_VAL >> 8) & 0xFF;
            buf[1] = (BM1422_CNTL4_VAL & 0xFF);
            rc = RegWrite(BM1422_CNTL4, buf[0]);
            if (rc == 0)
            {
                Debug.Print("Can't write BM1422_CNTL4 Register");
                return (rc);
            }
            rc = RegWrite((BM1422_CNTL4 + 1), buf[1]);
            if (rc == 0)
            {
                Debug.Print("Can't write BM1422_CNTL4 Register");
                return (rc);
            }


            //5,CNTL2レジスタへ書き込み（DREN=1,DRDY端子のEnable）
            Debug.Print("\n=== Write CNTL2 Register ===");
            val = BM1422_CNTL2_VAL;
            rc = RegWrite(BM1422_CNTL2, val);
            if (rc == 0)
            {
                Debug.Print("Can't write BM1422_CNTL2 Register");
                return (rc);
            }

            // Option
            // 6,AVE_Aレジスタへ書き込み（平均回数4回）
            Debug.Print("\n=== Write AVE_A Register ===");
            val = BM1422_AVE_A_VAL;
            rc = RegWrite(BM1422_AVE_A, val);
            if (rc == 0)
            {
                Debug.Print("Can't write BM1422_AVE_A Register");
                return (rc);
            }

            Debug.Print("########## SetUp Complete!! ##########\n");
            return rc;

        }

        /**
        * get_rawval関数
        * CNTL3レジスタへ書き込み（FORCE=1, 測定開始）
        * DRDYフラグが１になるまで待機。DRDYフラグが１になれば次の処理へ
        * X軸、Y軸、Z軸の6バイトのレジスタデータ取得
        */
        public byte get_rawval(ref byte[] data)
        {
            byte rc;
            byte reg;

            // Step4
            reg = BM1422_CNTL3_VAL;
            rc = RegWrite(BM1422_CNTL3, reg);
            if (rc == 0)
            {
                Debug.Print("Can't write BM1422_CNTL3 Register");
                return (rc);
            }

            while (_drdy_flg == 0)
            {
                System.Threading.Thread.Sleep(100);

            }
            _drdy_flg = 0;

            rc = RegRead(BM1422_DATAX, ref data);
            if (rc == 0)
            {
                Debug.Print("Can't get BM1422 magnet values");
            }

            return (rc);
        }

        /*
        * get_val関数
        * レジスタのデータを2バイト化とuT変換
        * オフセット自動調整未実装のため、測定可能範囲は±300[uT]まで
        */
        public byte get_val(ref float[] data)
        {
            byte rc;
            byte[] val = new byte[6];
            short[] mag = new short[3];

            rc = get_rawval(ref val);
            if (rc == 0)
            {
                return (rc);
            }

            mag[0] = (short)(((short)val[1] << 8) | (val[0]));
            mag[1] = (short)(((short)val[3] << 8) | (val[2]));
            mag[2] = (short)(((short)val[5] << 8) | (val[4]));

            convert_uT(ref mag, ref data);

            return (rc);
        }

        /*
        * convert_uT関数
        * レジスタのデータをuTに変換
        */
        public void convert_uT(ref short[] rawdata, ref float[] data)
        {
            // LSB to uT
            data[0] = (float)rawdata[0] / _sens;
            data[1] = (float)rawdata[1] / _sens;
            data[2] = (float)rawdata[2] / _sens;
        }


        /**
         * bm1244_isr
         * set_drdy_flg()をコール
         */
        public void bm1422_isr()
        {
            set_drdy_flg();
        }

        /*
        * set_drdy_flg関数
        * DRDYフラグを1にする
        */
        public void set_drdy_flg()
        {
            _drdy_flg = 1;
        }


        /**
         * 地磁気を計測
         * 戻り値:を格納したSensorReadingオブジェクト
         */
        public SensorReading TakeMeasurement()
        {

            byte rc;
            float[] mag = new float[3];

            rc = get_val(ref mag);

            if (rc != 0)
            {
                Debug.Print("BM1422 XDATA=" + mag[0] + "[uT]");
                Debug.Print("BM1422 YDATA=" + mag[1] + "[uT]");
                Debug.Print("BM1422 ZDATA=" + mag[2] + "[uT]");
            }

            //遅延 0.5秒
            System.Threading.Thread.Sleep(500);

            //SensorReadingオブジェクトを作成し、地磁気(X,Y,Z)を格納して返す
            return new SensorReading()
            {
                geomagnetism_x = mag[0],
                geomagnetism_y = mag[1],
                geomagnetism_z = mag[2]
            };

        }
        //3つの地磁気の値を返すために内部クラスを作成
        public class SensorReading
        {
            public float geomagnetism_x { get; set; }
            public float geomagnetism_y { get; set; }
            public float geomagnetism_z { get; set; }
        }




        /*
        * isr_func関数
        * Caller : コンストラクタ
        * BMI422_CNTL2_VALのDRPbitの設定で、DRDY端子の立ち上がりもしくは立下りで割り込み関数を設定する
        * C#版をどのように実装する？？？
        *
        * arduino用の外部割込みが発生した場合に実行する関数：
        * attachInterrupt(割り込み番号, 関数名, 割り込みモード)
        *
        * 割り込みモード:
        * RISING ピンの状態がLOWからHIGHに変わったときに発生 
        * FALLING ピンの状態がHIGHからLOWに変わったときに発生 
        */
        /*
        public void isr_func(int int_num, void func(void))
        {
          if (BM1422_CNTL2_VAL & BM1422_CNTL2_DRP) {
            attachInterrupt(int_num, func, RISING);
          } else {
            attachInterrupt(int_num, func, FALLING);
          }
        }

        */
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
            Debug.Print("Reg Read, Numbar of transfer data = " + rc);
            Debug.Print("REG[0x" + adata[0].ToString("X2") + "]=0x" + rdata[0].ToString("X2"));
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
        public byte RegWrite(byte addr, byte val)
        {
            wdata[0] = addr; //書き込み先アドレス
            wdata[1] = val; //書き込みデータ
            trRegWrite = new I2CDevice.I2CTransaction[] { I2CDevice.CreateWriteTransaction(wdata) };
            int rc = i2c.Execute(trRegWrite, timeout); //戻り値:転送されたバイト数
            Debug.Print("Reg Write : Number of Transfer Data = " + rc); //値が書き込めているか確認
            return (byte)rc;
        }




        /**
        * RegWrite(byte addr, byte data, int size)
        * 2017/09/13 1バイト以上のデータを書き込めるようにメソッドを拡張。
        * レジスタへデータを書き込み
        * addr : 書き込み先アドレス
        * data[] : 書き込みデータ
        * size   :dataの配列サイズ
        * 戻り値:レジスタへ転送したバイト数
        */
        /*
       public byte RegWrite_n(byte addr, byte[] data, int size)
       {
           byte[] wdata = new byte[size + 1]; //レジスタへ転送する配列
           wdata[0] = addr; //書き込みアドレスをwdata配列の先頭に格納

           //配列の1番目から書き込みデータを格納
           for (int i = 0; i < data.Length; i++)
               wdata[i + 1] = data[i];

           trRegWrite = new I2CDevice.I2CTransaction[] { I2CDevice.CreateWriteTransaction(wdata) };
           int rc = i2c.Execute(trRegWrite, timeout); //戻り値:転送されたバイト数
           Debug.Print("Reg Write : Number of Transfer Data = " + rc); //値が書き込めているか確認
           return (byte)rc;
       }
       */




    }
}
