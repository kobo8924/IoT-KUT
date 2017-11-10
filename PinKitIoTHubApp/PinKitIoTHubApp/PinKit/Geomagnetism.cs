
/**
 * 2017/09/11
 * Written by kobo
 * �ۑ�:�O�������݂��������ۂɎ��s���郁�\�b�h�̎���
 *     :volatile�A�E�q�̃R���p�C���G���[
 */

//RegRead()�f�o�b�O�p�}�N��
#define DEBUG_REGREAD


using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

//volatile�C���q���g���Ƃ��ɕK�v�����i�H�j
//using System.Runtime.CompilerServices.IsVolatile;


namespace PinKit
{
    /**
     * 3���n���C�Z���T
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

        //volatile�C���q���g���ƃR���p�C���G���[�i�������j
        //mscorlib.dll���֌W���邩��
        //private volatile int _drdy_flg;
        private int _drdy_flg;




        private int timeout = 1000;
        private I2CDevice i2c;               //�f�o�C�X�A�h���X�̓R���X�g���N�^�ɂăR���t�B�M�����[�V����
        private byte[] adata = new byte[1];  //�ǂݏo���A�h���X
        private byte[] wdata = new byte[2];  //�������݃A�h���X(1byte)�A�������݃f�[�^(1byte)

        private I2CDevice.I2CTransaction[] trRegRead;
        private I2CDevice.I2CTransaction[] trRegWrite;


        /**
        * �R���X�g���N�^
        * Caller : �N���XPinKit����C���X�^���X��
        * �����Ɏw�肳�ꂽ�f�o�C�X�A�h���X������ϐ��ɕێ�(0x0E or 0x0F)
        */
        public Geomagnetism(int slave_address = 0x0E, int clockRate = 100, int timeout = 10000)
        {
            byte rc = 0;
            _device_address = slave_address;
            this.timeout = timeout;
            //�f�o�C�X�R���t�B�M�����[�V����
            i2c = new I2CDevice(new I2CDevice.Configuration((ushort)_device_address, clockRate));

            rc = this.setup();
            if (rc == 0)
                Debug.Print("BM1422 initialization failed");

            //���荞�݊֐��R�[�X�i�������j
            //���Ƃ͊��荞�݂̎����̂�
            //isr_func(0, bm1422_isr);

            //�e�X�g�p���\�b�h
            //TakeMeasurement();

        }

        /*
        * init():�f�o�C�X�̃Z�b�g�A�b�v
        * 1,DRDY�t���O�̏�����
        * 2,WIA���W�X�^�̓ǂݏo��
        * 3,CNTL1���W�X�^�֏������݁iPC1=1,14bit���[�h,�V���O�����胂�[�h�j
        * 4,CNTL4���W�X�^�֏������݁i���Z�b�g�����j
        * 5,CNTL2���W�X�^�֏������݁iDREN=1,DRDY�[�q��Enable�j
        * 6,AVE_A���W�X�^�֏������݁i���ω�4��j
        */
        public byte setup()
        {
            byte rc = 0;
            byte val = 0;
            byte[] reg = new byte[1];
            byte[] buf = new byte[2];

            Debug.Print("\n########## SetUp Geomagnetism Sensor ##########");

            // 1,DRDY�t���O�̏�����
            _drdy_flg = 0;

            //2,WIA���W�X�^�̓ǂݏo��
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

            //3,CNTL1���W�X�^�֏������݁iPC1=1,14bit���[�h,�V���O�����胂�[�h�j
            Debug.Print("\n=== Write CNTL1 Register ===");
            val = BM1422_CNTL1_VAL;
            rc = RegWrite(BM1422_CNTL1, val);
            if (rc == 0)
            {
                Debug.Print("Can't write BM1422_CNTL1 Register");
                return (rc);
            }
            //�������������߂Ă��邩�m�F
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
            //�x�� 1ms
            System.Threading.Thread.Sleep(1);


            //4,CNTL4���W�X�^�֏������݁i���Z�b�g�����j
            // CNTL4���W�X�^��bit��16bit(0x5C / 0x5D)
            //RegWrite��2��R�[�����Ă���Bn�o�C�g�������߂郁�\�b�h�ŏ������ׂ����H
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


            //5,CNTL2���W�X�^�֏������݁iDREN=1,DRDY�[�q��Enable�j
            Debug.Print("\n=== Write CNTL2 Register ===");
            val = BM1422_CNTL2_VAL;
            rc = RegWrite(BM1422_CNTL2, val);
            if (rc == 0)
            {
                Debug.Print("Can't write BM1422_CNTL2 Register");
                return (rc);
            }

            // Option
            // 6,AVE_A���W�X�^�֏������݁i���ω�4��j
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
        * get_rawval�֐�
        * CNTL3���W�X�^�֏������݁iFORCE=1, ����J�n�j
        * DRDY�t���O���P�ɂȂ�܂őҋ@�BDRDY�t���O���P�ɂȂ�Ύ��̏�����
        * X���AY���AZ����6�o�C�g�̃��W�X�^�f�[�^�擾
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
        * get_val�֐�
        * ���W�X�^�̃f�[�^��2�o�C�g����uT�ϊ�
        * �I�t�Z�b�g���������������̂��߁A����\�͈͂́}300[uT]�܂�
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
        * convert_uT�֐�
        * ���W�X�^�̃f�[�^��uT�ɕϊ�
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
         * set_drdy_flg()���R�[��
         */
        public void bm1422_isr()
        {
            set_drdy_flg();
        }

        /*
        * set_drdy_flg�֐�
        * DRDY�t���O��1�ɂ���
        */
        public void set_drdy_flg()
        {
            _drdy_flg = 1;
        }


        /**
         * �n���C���v��
         * �߂�l:���i�[����SensorReading�I�u�W�F�N�g
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

            //�x�� 0.5�b
            System.Threading.Thread.Sleep(500);

            //SensorReading�I�u�W�F�N�g���쐬���A�n���C(X,Y,Z)���i�[���ĕԂ�
            return new SensorReading()
            {
                geomagnetism_x = mag[0],
                geomagnetism_y = mag[1],
                geomagnetism_z = mag[2]
            };

        }
        //3�̒n���C�̒l��Ԃ����߂ɓ����N���X���쐬
        public class SensorReading
        {
            public float geomagnetism_x { get; set; }
            public float geomagnetism_y { get; set; }
            public float geomagnetism_z { get; set; }
        }




        /*
        * isr_func�֐�
        * Caller : �R���X�g���N�^
        * BMI422_CNTL2_VAL��DRPbit�̐ݒ�ŁADRDY�[�q�̗����オ��������͗�����Ŋ��荞�݊֐���ݒ肷��
        * C#�ł��ǂ̂悤�Ɏ�������H�H�H
        *
        * arduino�p�̊O�������݂����������ꍇ�Ɏ��s����֐��F
        * attachInterrupt(���荞�ݔԍ�, �֐���, ���荞�݃��[�h)
        *
        * ���荞�݃��[�h:
        * RISING �s���̏�Ԃ�LOW����HIGH�ɕς�����Ƃ��ɔ��� 
        * FALLING �s���̏�Ԃ�HIGH����LOW�ɕς�����Ƃ��ɔ��� 
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
         * ���W�X�^����f�[�^��ǂݎ��
         * addr : �f�o�C�X�ɑ��M�����o�C�g�̔z��(�A�h���X)
         * rdata : �f�o�C�X����ǂݎ��ꂽ�f�[�^���i�[
         * �߂�l:���W�X�^�֓]�������o�C�g��
         */
        public byte RegRead(byte addr, ref byte[] rdata)
        {
            adata[0] = addr;
            trRegRead = new I2CDevice.I2CTransaction[] {
                   I2CDevice.CreateWriteTransaction(adata),   //�f�o�C�X�փA�h���X��]��
                   I2CDevice.CreateReadTransaction(rdata) };  //�f�o�C�X���f�[�^��ǂݍ���
            int rc = i2c.Execute(trRegRead, timeout);      //�߂�l:�]�����ꂽ�o�C�g��
#if DEBUG_REGREAD
            Debug.Print("Reg Read, Numbar of transfer data = " + rc);
            Debug.Print("REG[0x" + adata[0].ToString("X2") + "]=0x" + rdata[0].ToString("X2"));
#endif
            return (byte)rc;
        }

        /**
        * RegWrite(byte reg, byte val)
        * ���W�X�^�փf�[�^����������
        * reg : �������ݐ�A�h���X
        * val : �������݃f�[�^
        * �߂�l:���W�X�^�֓]�������o�C�g��
        */
        public byte RegWrite(byte addr, byte val)
        {
            wdata[0] = addr; //�������ݐ�A�h���X
            wdata[1] = val; //�������݃f�[�^
            trRegWrite = new I2CDevice.I2CTransaction[] { I2CDevice.CreateWriteTransaction(wdata) };
            int rc = i2c.Execute(trRegWrite, timeout); //�߂�l:�]�����ꂽ�o�C�g��
            Debug.Print("Reg Write : Number of Transfer Data = " + rc); //�l���������߂Ă��邩�m�F
            return (byte)rc;
        }




        /**
        * RegWrite(byte addr, byte data, int size)
        * 2017/09/13 1�o�C�g�ȏ�̃f�[�^���������߂�悤�Ƀ��\�b�h���g���B
        * ���W�X�^�փf�[�^����������
        * addr : �������ݐ�A�h���X
        * data[] : �������݃f�[�^
        * size   :data�̔z��T�C�Y
        * �߂�l:���W�X�^�֓]�������o�C�g��
        */
        /*
       public byte RegWrite_n(byte addr, byte[] data, int size)
       {
           byte[] wdata = new byte[size + 1]; //���W�X�^�֓]������z��
           wdata[0] = addr; //�������݃A�h���X��wdata�z��̐擪�Ɋi�[

           //�z���1�Ԗڂ��珑�����݃f�[�^���i�[
           for (int i = 0; i < data.Length; i++)
               wdata[i + 1] = data[i];

           trRegWrite = new I2CDevice.I2CTransaction[] { I2CDevice.CreateWriteTransaction(wdata) };
           int rc = i2c.Execute(trRegWrite, timeout); //�߂�l:�]�����ꂽ�o�C�g��
           Debug.Print("Reg Write : Number of Transfer Data = " + rc); //�l���������߂Ă��邩�m�F
           return (byte)rc;
       }
       */




    }
}
