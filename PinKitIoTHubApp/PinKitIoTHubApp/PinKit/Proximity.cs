
/**
 * 2017/09/11
 * Written by kobo
 * 
 */

//RegRead()�f�o�b�O�p�}�N��
#define DEBUG_REGREAD


using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;



namespace PinKit
{
    /**
     * �ߐځE�Ɠx�Z���T
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

        // byte�^���ƂȂ���ERROR�ɂȂ�
        const int RPR0521RS_ERROR = (-1);

        private ushort _als_data0_gain;
        private ushort _als_data1_gain;
        private ushort _als_measure_time;

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
        public Proximity(int clockRate = 100, int timeout = 10000)
        {
            this.timeout = timeout;
            //�f�o�C�X�R���t�B�M�����[�V����
            i2c = new I2CDevice(new I2CDevice.Configuration((ushort)RPR0521RS_DEVICE_ADDRESS, clockRate));

            this.setup();

            //�e�X�g�p���\�b�h
            //TakeMeasurement();

        }

        /**
         * �f�o�C�X�̃Z�b�g�A�b�v
         * init()
         * 1,Part ID���W�X�^�̓ǂݏo���y�ъm�F
         * 2,MANUFACT ID���W�X�^�̓ǂݏo���Ɗm�F
         * 3,ALS_PS_aCONTROL���W�X�^�֏�������(ALS DATA0:gain x1, ALS DATA1: Gain x1, 100mA)
         * 4,PS_CONTROL���W�X�^�֏�������(PS Gain x1)
         * 5,MODE_CONTROL���W�X�^�֏�������(ALS:ON,PS:ON,PS LED pulse = 200us, ALS:100ms,PS:100ms)
         * �߂�l: rc (0�ȏ�Ȃ琬���A�Z�b�g�A�b�v0�Ȃ玸�s)
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

            //ID���W�X�^��ǂݏo���Ɗm�F�B�߂�l�F�]�������o�C�g��
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

            // MANUFACT ID���W�X�^�̓ǂݏo���Ɗm�F
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

            // ALS_PS_CONTROL���W�X�^�֏�������(ALS DATA0:gain x1, ALS DATA1: Gain x1, 100mA)
            Debug.Print("\n=== Write ALS_PS_CONTROL_Rgister ===");
            val = RPR0521RS_ALS_PS_CONTROL_VAL;
            rc = RegWrite(RPR0521RS_ALS_PS_CONTROL, val);
            if (rc == 0)
            {
                Debug.Print("Can't write RPR0521RS ALS_PS_CONTROL register");
                return (rc);
            }

            reg[0] = 0;
            rc = RegRead(RPR0521RS_PS_CONTROL, ref reg); //�l���������������߂Ă��邩�m�F
            if (rc == 0)
            {
                Debug.Print("Can't read RPR0521RS PS_CONTROL register");
                return rc;
            }

            // PS_CONTROL���W�X�^�֏�������(PS Gain x1)
            Debug.Print("\n=== Write PS_CONTORL_Register ===");
            val = reg[0];
            val |= RPR0521RS_PS_CONTROL_VAL;
            rc = RegWrite(RPR0521RS_PS_CONTROL, val);
            if (rc == 0)
            {
                Debug.Print("Can't write RPR0521RS PS_CONTROL register");
            }

            // MODE_CONTROL���W�X�^�֏�������(ALS:ON,PS:ON,PS LED pulse = 200us, ALS:100ms,PS:100ms)
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
        * �ߐځA�Ɠx�̑���f�[�^6�o�C�g�擾(�A�h���X0x44����6�o�C�g)
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
        * get_rawpsalsval�֐��̎��s
        * convert_lx�֐��̎��s
        * �ߐڒl{LSB}�͂��̂܂܁A�Ɠx�l��[|x]�ϊ�
        * �߂�l�F���W�X�^�֓]�������o�C�g��
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

            //�ߐڒl
            ps = rawps;

            //���W�X�^�l����Ɠx�l���v�Z
            als = convert_lx(ref rawals);

            return rc;

        }

        /**
        * check_near_far()
        * �ߐڒl������l�i�T���v���ł�3000�j�ȏ�̏ꍇ�ɁuNear�v�A����l�����̏ꍇ�́uFar�v��Ԃ�
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
        * ���W�X�^�l����Ɠx�l[lx]�ϊ�
        * �߂�l:�Ɠx�l(lx)
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
         * �߂�l:���W�X�^�֓]�������o�C�g��
         */
        public byte RegWrite(byte reg, byte val)
        {
            wdata[0] = reg; //�������ݐ�A�h���X
            wdata[1] = val; //�������݃f�[�^
            trRegWrite = new I2CDevice.I2CTransaction[] { I2CDevice.CreateWriteTransaction(wdata) };
            int rc = i2c.Execute(trRegWrite, timeout); //�߂�l:�]�����ꂽ�o�C�g��
            Debug.Print("Reg Write : Number of Transfer Data = " + rc); //�l���������߂Ă��邩�m�F
            return (byte)rc;
        }

        /**
         * �ߐڒl�ƏƓx���v��
         * �߂�l:�ߐڒl�A�Ɠx�l���i�[����SensorReading�I�u�W�F�N�g
         */
        public SensorReading TakeMeasurement()
        {
            byte rc;
            byte near_far;

            //�ߐڒl�@
            ushort ps_val = 0;
            //�Ɠx
            float als_val = 0;

            Debug.Print("=== TakeMeasurement Proximity & Ambient Light ===");

            rc = get_psalsval(ref ps_val, ref als_val);
            if (rc != 0)
            {
                //�ߐ�
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
                    //�Ɠx�@
                    Debug.Print("RPR-0521RS (Ambient Light) = " + als_val + "[lx]\n");
                }
            }
            Debug.Print("=== END ===");

            //�x�� 0.5�b
            System.Threading.Thread.Sleep(500);

            //SensorReading�I�u�W�F�N�g���쐬���A�ߐڒl�A�Ɠx�l���i�[���ĕԂ�
            return new SensorReading()
            {
                proximity = ps_val,
                ambient_light = als_val
            };

        }
        //�ߐڒl�A�Ɠx�l�A2�̒l��Ԃ����߂ɃT�u�N���X���쐬
        public class SensorReading
        {
            public ushort proximity { get; set; }
            public float ambient_light { get; set; }
        }


    }
}
