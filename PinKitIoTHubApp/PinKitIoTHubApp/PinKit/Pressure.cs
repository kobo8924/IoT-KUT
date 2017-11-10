
/**
 * 2017/09/07
 * Written by kobo
 * 
 */

//RegRead()�f�o�b�O�p�}�N��
#define DEBUG_BM1383GLV


using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;



namespace PinKit
{
    /**
     * �C���Z���T
     * ROHM BM1383GLV-ZE2
     * �C���͈� 300�`1100hPa (16bit)
     */
    public class Pressure
    {


        //7bit Addres
        //�f�o�b�O�̍ۂ�0x50��0x5D���ԈႦ�Ȃ��悤�ɁI�I
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
        private I2CDevice i2c;               //�f�o�C�X�A�h���X�̓R���X�g���N�^�ɂăR���t�B�M�����[�V����
        private byte[] adata = new byte[1];  //�ǂݏo���A�h���X
        private byte[] wdata = new byte[2];  //�������݃A�h���X(1byte)�A�������݃f�[�^(1byte)

        private I2CDevice.I2CTransaction[] trRegRead;
        private I2CDevice.I2CTransaction[] trRegWrite;


        /**
        * �R���X�g���N�^
        * Caller : �N���XPinKit����C���X�^���X��
        */
        public Pressure(int clockRate = 100, int timeout = 10000)
        {
            this.timeout = timeout;
            //�f�o�C�X�R���t�B�M�����[�V����
            i2c = new I2CDevice(new I2CDevice.Configuration((ushort)BM1383GLV_DEVICE_ADDRESS, clockRate));

            setup();

            //�e�X�g�p ���\�b�h�@
            /*
             int i = 50;
            for (i = 0; i < 50; i++)
            {
                this.TakeMeasurement();
                System.Threading.Thread.Sleep(2000);
            }
            */
            this.TakeMeasurement();

            //��b�ԁi1000�~���b�j��~����
            // �R�R���u���[�N�|�C���g�Ƃ��ăe�X�g��

        }




        /**
        * setup()
        * �C���Z���T�̃Z�b�g�A�b�v
        * Caller : ���̃N���X�̃R���X�g���N�^
        * 1,ID���W�X�^�̓ǂݏo��
        * 2,�p���[�_�E������(POWER_DON)
        * 3,1ms��delay
        * 4,�X�^���o�C����(SLEEP)
        * 5,MODE_CONTROL���W�X�^�֏������݁i64�񕽋ρAT_AVE=1�A200ms�A������j
        **/
        public void setup()
        {
            byte[] data = new byte[1];  //ID_REG����ǂݏo�����f�[�^���i�[
            byte val;
            byte rc;

            Debug.Print("\n########## Setup Pressure Sensor ##########");

            //ID���W�X�^�ǂݏo��
            //���W�X�^(BM1383GLV_ID)�ɃA�N�Z�X����ƂȂ���0x32���ǂݏo�����B(0x31���ǂݏo�����d�l�Ȃ̂ɁB�B�B)
            rc = RegRead(BM1383GLV_ID, ref data);
            Debug.Print("ID Register Reading : Number of Transfer Data = " + rc + "'");
            Debug.Print("!!!!! BM1383GL ID Register Value = 0x" + data[0].ToString("X2") + "'");

            /*
            if (reg!= BM1383GLV_ID_VAL) 
                Debug.Print("===== Can't find BM1383GLV =====");
            else
                Debug.Print("OK! Can fine BM1383GLV !!!");
                */

            //�p���[�_�E������(POWER_DON)
            val = BM1383GLV_POWER_DOWN_VAL;
            RegWrite(BM1383GLV_POWER_DOWN, val);


            //�X�^���o�C����(SLEEP)
            val = BM1383GLV_RESET_VAL;
            RegWrite(BM1383GLV_RESET, val);

            //MODE_CONTROL���W�X�^�֏������݁i64�񕽋ρAT_AVE=1�A200ms�A������j
            val = BM1383GLV_MODE_CONTROL_VAL;
            RegWrite(BM1383GLV_MODE_CONTROL, val);


            Debug.Print("########## END Setup Pressure Sensor ##########\n");

            //�������݃`�F�b�N  �f�o�b�O
            //rc = RegRead(BM1383GLV_MODE_CONTROL, ref data);
        }


        /**
         * get_rawval(ref byte[] data)
         * �C���l���v�Z���邽�߂ɋC�����W�X�^����3�o�C�g�f�[�^�ǂݏo��(���W�X�^0x1c����3�o�C�g)
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
        * �C���̃��W�X�^�l����C���l�֕ϊ����ĕϐ�press(�Q�ƌďo��)�Ɋi�[
        * uint�^ : �����Ȃ�32bit����
        */
        public byte get_val(ref float press)
        {
            byte rc;
            byte[] val = new byte[3];
            uint rawpress;

            //�C���̃��W�X�^����3�o�C�g�f�[�^�ǂݏo��
            rc = get_rawval(ref val);
            if (rc == 0)
                return rc;


            Debug.Print("\n=== Debug : Pressure.get_val() ===");
            Debug.Print("VAL[0]=0x" + val[0].ToString("X2"));
            Debug.Print("VAL[1]=0x" + val[1].ToString("X2"));
            Debug.Print("VAL[2]=0x" + val[2].ToString("X2"));
            Debug.Print("=== END Debug Pressure.get_val() ===\n         ");

            //rawpress = ((ulong)val[0] << 16) || ((ulong)val[1] << 8) || (val[2] & 0xFC) >> 2));

            //�C�����W�X�^�l����C���l�ɕϊ�
            //rawpress =  ((uint)val[0] << 16) | ((uint)val[1] << 8) | (((uint)val[2] & 0xFC) >> 2);
            rawpress = (((uint)val[0]) << 16) | (((uint)val[1]) << 8) | (((uint)(val[2] & 0xFC)) >> 2);

            //rawpress = (byte) ((ulong)val[0] << 16) | ((ulong)val[1] << 8) ;
            //rawpress = rawpress | ((val[2] & 0xFC) >> 2);


            press = ((float)rawpress) / 2048;
            return rc;
        }

        /**
         * RegRead(byte, ref byte[])
         * ���W�X�^����f�[�^��ǂݎ��
         * addr : �f�o�C�X�ɑ��M�����o�C�g�̔z��(�A�h���X)
         * rdata : �f�o�C�X����ǂݎ��ꂽ�f�[�^���i�[
         */
        public byte RegRead(byte addr, ref byte[] rdata)
        {
            adata[0] = addr;
            trRegRead = new I2CDevice.I2CTransaction[] {
                   I2CDevice.CreateWriteTransaction(adata),   //�f�o�C�X�փA�h���X��]��
                   I2CDevice.CreateReadTransaction(rdata) };  //�f�o�C�X���f�[�^��ǂݍ���
            int rc = i2c.Execute(trRegRead, timeout);      //�߂�l:�]�����ꂽ�o�C�g��
#if DEBUG_BM1383GLV
            Debug.Print("Reg Read : result = " + rc);
            Debug.Print("R[0x" + adata[0].ToString("X2") + "]=0x" + rdata[0].ToString("X2"));
#endif
            return (byte)rc;
        }


        /**
         * RegWrite(byte reg, byte val)
         * ���W�X�^�փf�[�^����������
         * reg : �������ݐ�A�h���X
         * val : �������݃f�[�^
         */
        public void RegWrite(byte reg, byte val)
        {
            wdata[0] = reg; //�������ݐ�A�h���X
            wdata[1] = val; //�������݃f�[�^
            trRegWrite = new I2CDevice.I2CTransaction[] { I2CDevice.CreateWriteTransaction(wdata) };
            int result = i2c.Execute(trRegWrite, timeout); //�߂�l:�]�����ꂽ�o�C�g��
            Debug.Print("Reg Write : Number of Transfer Data = " + result); //�l���������߂Ă��邩�m�F
        }

        /**
         * TakeMeasurment()
         * �C�����v��
         */
        public float TakeMeasurement()
        {
            byte rc;
            float press = 0;

            //�C���l�̎擾
            rc = get_val(ref press);

            if (rc != 0)
            {
                Debug.Print("BM1383GLV (PRESS) = " + press + "[hPa]");
            }


            //�x�� 0.5�b
            System.Threading.Thread.Sleep(500);


            return press;

        }
    }
}
