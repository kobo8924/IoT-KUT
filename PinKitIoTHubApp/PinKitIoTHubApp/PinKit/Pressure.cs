
/**
 * 2017/09/07
 * Written by kobo
 * 
 */

//RegRead()デバッグ用マクロ
#define DEBUG_BM1383GLV


using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;



namespace PinKit
{
    /**
     * 気圧センサ
     * ROHM BM1383GLV-ZE2
     * 気圧範囲 300～1100hPa (16bit)
     */
    public class Pressure
    {


        //7bit Addres
        //デバッグの際に0x50と0x5Dを間違えないように！！
        const byte BM1383GLV_DEVICE_ADDRESS = 0x5D;

        const byte BM1383GLV_ID_VAL = 0x31;

        const byte BM1383GLV_ID = 0x10;
        const byte BM1383GLV_POWER_DOWN = 0x12;
        const byte BM1383GLV_RESET = 0x13;
        const byte BM1383GLV_MODE_CONTROL = 0x14;
        const byte BM1383GLV_PRESSURE_MSB = 0x1C;

        const byte BM1383GLV_POWER_DOWN_PWR_DOWN = (1 << 0);
        const byte BM1383GLV_RESET_RSTB = (1 << 0);
        const byte BM1383GLV_MODE_CONTROL_AVE_NUM64 = (6 << 5);
        const byte BM1383GLV_MODE_CONTROL_T_AVE = (1 << 3);
        const byte BM1383GLV_MODE_CONTORL_MODE_200MS = (4 << 0);


        const byte BM1383GLV_POWER_DOWN_VAL = (BM1383GLV_POWER_DOWN_PWR_DOWN);
        const byte BM1383GLV_RESET_VAL = (BM1383GLV_RESET_RSTB);
        const byte BM1383GLV_MODE_CONTROL_VAL = (BM1383GLV_MODE_CONTROL_AVE_NUM64 | BM1383GLV_MODE_CONTROL_T_AVE | BM1383GLV_MODE_CONTORL_MODE_200MS);

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
        public Pressure(int clockRate = 100, int timeout = 10000)
        {
            this.timeout = timeout;
            //デバイスコンフィギュレーション
            i2c = new I2CDevice(new I2CDevice.Configuration((ushort)BM1383GLV_DEVICE_ADDRESS, clockRate));

            setup();

            //テスト用 メソッド　
            /*
             int i = 50;
            for (i = 0; i < 50; i++)
            {
                this.TakeMeasurement();
                System.Threading.Thread.Sleep(2000);
            }
            */
            this.TakeMeasurement();

            //一秒間（1000ミリ秒）停止する
            // ココをブレークポイントとしてテスト中

        }




        /**
        * setup()
        * 気圧センサのセットアップ
        * Caller : このクラスのコンストラクタ
        * 1,IDレジスタの読み出し
        * 2,パワーダウン解除(POWER_DON)
        * 3,1msのdelay
        * 4,スタンバイ解除(SLEEP)
        * 5,MODE_CONTROLレジスタへ書き込み（64回平均、T_AVE=1、200ms連続測定）
        **/
        public void setup()
        {
            byte[] data = new byte[1];  //ID_REGから読み出したデータを格納
            byte val;
            byte rc;

            Debug.Print("\n########## Setup Pressure Sensor ##########");

            //IDレジスタ読み出し
            //レジスタ(BM1383GLV_ID)にアクセスするとなぜか0x32が読み出される。(0x31が読み出される仕様なのに。。。)
            rc = RegRead(BM1383GLV_ID, ref data);
            Debug.Print("ID Register Reading : Number of Transfer Data = " + rc + "'");
            Debug.Print("!!!!! BM1383GL ID Register Value = 0x" + data[0].ToString("X2") + "'");

            /*
            if (reg!= BM1383GLV_ID_VAL) 
                Debug.Print("===== Can't find BM1383GLV =====");
            else
                Debug.Print("OK! Can fine BM1383GLV !!!");
                */

            //パワーダウン解除(POWER_DON)
            val = BM1383GLV_POWER_DOWN_VAL;
            RegWrite(BM1383GLV_POWER_DOWN, val);


            //スタンバイ解除(SLEEP)
            val = BM1383GLV_RESET_VAL;
            RegWrite(BM1383GLV_RESET, val);

            //MODE_CONTROLレジスタへ書き込み（64回平均、T_AVE=1、200ms連続測定）
            val = BM1383GLV_MODE_CONTROL_VAL;
            RegWrite(BM1383GLV_MODE_CONTROL, val);


            Debug.Print("########## END Setup Pressure Sensor ##########\n");

            //書き込みチェック  デバッグ
            //rc = RegRead(BM1383GLV_MODE_CONTROL, ref data);
        }


        /**
         * get_rawval(ref byte[] data)
         * 気圧値を計算するために気圧レジスタから3バイトデータ読み出し(レジスタ0x1cから3バイト)
         * Caller : get_val(ref float press)
        */
        public byte get_rawval(ref byte[] data)
        {
            byte rc;
            rc = RegRead(BM1383GLV_PRESSURE_MSB, ref data);

            if (rc != 0)
                Debug.Print("BM1383GLV PRESSU value Reading : Number of Transfer Data = " + rc + "'");
            else
                Debug.Print("Can't get BM1383GLV PRESS value");

            return rc;
        }


        /**
        * get_val(ref float press)
        * Caller : TakeMeasurement()
        * 気圧のレジスタ値から気圧値へ変換して変数press(参照呼出し)に格納
        * uint型 : 符号なし32bit整数
        */
        public byte get_val(ref float press)
        {
            byte rc;
            byte[] val = new byte[3];
            uint rawpress;

            //気圧のレジスタから3バイトデータ読み出し
            rc = get_rawval(ref val);
            if (rc == 0)
                return rc;


            Debug.Print("\n=== Debug : Pressure.get_val() ===");
            Debug.Print("VAL[0]=0x" + val[0].ToString("X2"));
            Debug.Print("VAL[1]=0x" + val[1].ToString("X2"));
            Debug.Print("VAL[2]=0x" + val[2].ToString("X2"));
            Debug.Print("=== END Debug Pressure.get_val() ===\n         ");

            //rawpress = ((ulong)val[0] << 16) || ((ulong)val[1] << 8) || (val[2] & 0xFC) >> 2));

            //気圧レジスタ値から気圧値に変換
            //rawpress =  ((uint)val[0] << 16) | ((uint)val[1] << 8) | (((uint)val[2] & 0xFC) >> 2);
            rawpress = (((uint)val[0]) << 16) | (((uint)val[1]) << 8) | (((uint)(val[2] & 0xFC)) >> 2);

            //rawpress = (byte) ((ulong)val[0] << 16) | ((ulong)val[1] << 8) ;
            //rawpress = rawpress | ((val[2] & 0xFC) >> 2);


            press = ((float)rawpress) / 2048;
            return rc;
        }

        /**
         * RegRead(byte, ref byte[])
         * レジスタからデータを読み取る
         * addr : デバイスに送信されるバイトの配列(アドレス)
         * rdata : デバイスから読み取られたデータを格納
         */
        public byte RegRead(byte addr, ref byte[] rdata)
        {
            adata[0] = addr;
            trRegRead = new I2CDevice.I2CTransaction[] {
                   I2CDevice.CreateWriteTransaction(adata),   //デバイスへアドレスを転送
                   I2CDevice.CreateReadTransaction(rdata) };  //デバイスよりデータを読み込み
            int rc = i2c.Execute(trRegRead, timeout);      //戻り値:転送されたバイト数
#if DEBUG_BM1383GLV
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
         */
        public void RegWrite(byte reg, byte val)
        {
            wdata[0] = reg; //書き込み先アドレス
            wdata[1] = val; //書き込みデータ
            trRegWrite = new I2CDevice.I2CTransaction[] { I2CDevice.CreateWriteTransaction(wdata) };
            int result = i2c.Execute(trRegWrite, timeout); //戻り値:転送されたバイト数
            Debug.Print("Reg Write : Number of Transfer Data = " + result); //値が書き込めているか確認
        }

        /**
         * TakeMeasurment()
         * 気圧を計測
         */
        public float TakeMeasurement()
        {
            byte rc;
            float press = 0;

            //気圧値の取得
            rc = get_val(ref press);

            if (rc != 0)
            {
                Debug.Print("BM1383GLV (PRESS) = " + press + "[hPa]");
            }


            //遅延 0.5秒
            System.Threading.Thread.Sleep(500);


            return press;

        }
    }
}
