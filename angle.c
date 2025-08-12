#include "zf_common_headfile.h"
#include "math.h"

struct Angle angle={
  .q0=1,
  .q1=0,
  .q2=0,
  .q3=0,
  .init_flag=0,
    .pitch=0,
    .roll=0,
    .yaw=0,
    
    
};
float Ki=0.004f;
float Kp=1.0f;

float yaw;
float yaw_last;
float actual_angle;
float Yaw,Roll,Pitch;
float        start_Pitch;
float        start_Yaw;
float        start_Roll;

uint8 page=0;
uint8 line=0;
int8 fold=1;
int16 Lpwm_A26=700;
int16 Rpwm_A27=700;
int isr_1ms = 1;
uint8 stop0_go1=0;

float angle_p =0.050;
float angle_d =0;
float gyro_p =18;
float gyro_d =2;
float Angle_want=90;
uint16 base_speed=250;

float out_angle=0;
int16 out_gyro=0;
float angle_i =0;
int number_times=0;    
    
#define FILTER_NUM 5
#define halfT 0.020     //ms����һ��
#define M_PI 3.1415f   
    // ����ת�Ƕȹ�ʽ: �Ƕ� = ���� �� (180/��)
    const float rad_to_deg = 180.0f / M_PI;
    
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float hx, hy, hz;   //�شżƷ���
    float bx, bz;      //nϵ�еĵش���������ʵֵ[bx,bz,by] by=0


    float vx, vy, vz;  //nϵ���������ٶ�[0,0,1]ת����bϵ�еõ���������[vx,vy,vz]
    float wx, wy, wz;  //nϵ�еĵش�����[bx��by,bz]ת����bϵ�У��õ�[wx,wy,wz]
    float ex, ey, ez;  //����[wx,wy,wz] X [mx,my,mz],[ax,at,az] X [vx,vy,vz]���õ������������

            // ����һЩ������������ת������
    float q0q0 = angle.q0 * angle.q0;
    float q0q1 = angle.q0 * angle.q1;
    float q0q2 = angle.q0 * angle.q2;
    float q0q3 = angle.q0 * angle.q3;
    float q1q1 = angle.q1 * angle.q1;
    float q1q2 = angle.q1 * angle.q2;
    float q1q3 = angle.q1 * angle.q3;
    float q2q2 = angle.q2 * angle.q2;
    float q2q3 = angle.q2 * angle.q3;
    float q3q3 = angle.q3 * angle.q3;

    float qa;
    float qb;
    float qc;
    float qd;

            // ��һ�����ٶȼƺ͵شżƵĶ���
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //��bϵ�еĵشżƷ���[mx,my,mz]ת����nϵ,�õ�[hx,hy,hz]
    hx = 2.0f * (mx*(0.5f - q2q2 - q3q3) + my*(q1q2 - q0q3) + mz*(q1q3 + q0q2));
    hy = 2.0f * (mx*(q1q2 + q0q3) + my*(0.5f - q1q1 - q3q3) + mz*(q2q3 - q0q1));
    hz = 2.0f * (mx*(q1q3 - q0q2) + my*(q2q3 + q0q1) + mz*(0.5f - q1q1 - q2q2));

            //�õ�nϵ�еĵش���������ʵֵ[bx,bz,by],����by=0
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;

            //nϵ�еĵش�����[bx��by,bz]ת����bϵ�У��õ�[wx,wy,wz]
    wx = bx*(0.5f - q2q2 - q3q3) + bz*(q1q3 - q0q2);
    wy = bx*(q1q2 - q0q3) + bz*(q0q1 + q2q3);
    wz = bx*(q0q2 + q1q3) + bz*(0.5f - q1q1 - q2q2);

    //nϵ���������ٶ�[0,0,1]ת����bϵ�еõ���������[vx,vy,vz]
    vx = (q1q3 - q0q2);
    vy = (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //����[wx,wy,wz] X [mx,my,mz]  ��  [ax,at,az] X [vx,vy,vz]���õ������������
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    //  ex = (ay*vz - az*vy);
    //  ey = (az*vx - ax*vz);
    //  ez = (ax*vy - ay*vx);

    //PI�������еĻ��ֲ���
    angle.exInt +=  ex*Ki*halfT;
    angle.eyInt +=  ey*Ki*halfT;
    angle.ezInt +=  ez*Ki*halfT;

    //����PI�����������,Ȼ�󲹳������ٶȵ�����������Kp��Ki����Ҫ���ڵĲ���
    gx = gx + angle.exInt;
    gy = gy + angle.eyInt;
    gz = gz + angle.ezInt;

    gx=gx + Kp*ex;
    gy=gy + Kp*ey;
    gz=gz + Kp*ez;


    gx *=0.5f*halfT;
    gy *=0.5f*halfT;
    gz *=0.5f*halfT;


    qa=angle.q0;
    qb=angle.q1;
    qc=angle.q2;
    qd=angle.q3;

    //һ�����������������Ԫ��
    angle.q0 = qa + (-qb*gx - qc*gy - qd*gz);
    angle.q1 = qb + ( qa*gx + qc*gz - qd*gy);
    angle.q2 = qc + ( qa*gy - qb*gz + qd*gx);
    angle.q3 = qd + ( qa*gz + qb*gy - qc*gx);

    angle.pitch = asinf (2 * angle.q0 * angle.q2 - 2 * angle.q1 * angle.q3) * 180 / M_PI;
    angle.roll  = atan2f(2 * angle.q2 * angle.q3 + 2 * angle.q0 * angle.q1, -2 * angle.q1 * angle.q1 - 2 * angle.q2 * angle.q2 + 1) * 180 /M_PI ;
    angle.yaw   = atan2f(2 * angle.q1 * angle.q2 + 2 * angle.q0 * angle.q3, -2 * angle.q2 * angle.q2 - 2 * angle.q3 * angle.q3 + 1) * 180 /M_PI;
    // ��һ����Ԫ��
    recipNorm = invSqrt(angle.q0 * angle.q0 + angle.q1 * angle.q1 + angle.q2 * angle.q2 + angle.q3 * angle.q3);
    angle.q0 *= recipNorm;
    angle.q1 *= recipNorm;
    angle.q2 *= recipNorm;
    angle.q3 *= recipNorm;
    Pitch = angle.pitch;
    Roll  = angle.roll ;
    Yaw   = angle.yaw;
}
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void SMO_Filter(void)
{
    float k=0.35;

    angle.acc_xout=k*angle.my_acc_x+(1-k)*angle.AVG_Filter[0][0];
    angle.acc_yout=k*angle.my_acc_y+(1-k)*angle.AVG_Filter[1][0];
    angle.acc_zout=k*angle.my_acc_z+(1-k)*angle.AVG_Filter[2][0];

    angle.AVG_Filter[0][0]=angle.my_acc_x;
    angle.AVG_Filter[1][0]=angle.my_acc_y;
    angle.AVG_Filter[2][0]=angle.my_acc_z;
}
void data_get(void)
{
    imu660ra_get_gyro();
    angle.my_gyro_x=((float)imu660ra_gyro_x-angle.data_imu660ra_gyro_x) / 16.4f * M_PI / 180.f;
    angle.my_gyro_y=((float)imu660ra_gyro_y-angle.data_imu660ra_gyro_y) / 16.4f * M_PI / 180.f;
    angle.my_gyro_z=((float)imu660ra_gyro_z-angle.data_imu660ra_gyro_z) / 16.4f * M_PI / 180.f;
 
    if((angle.my_gyro_x)<0.005&&(angle.my_gyro_x)>-0.005)
        angle.my_gyro_x=0;
    if((angle.my_gyro_y)<0.005&&(angle.my_gyro_y)>-0.005)
        angle.my_gyro_y=0;
    if((angle.my_gyro_z)<0.005&&(angle.my_gyro_z)>-0.005)
        angle.my_gyro_z=0;


    imu660ra_get_acc();
    angle.my_acc_x=(float)imu660ra_acc_x / 4096.f ;
    angle.my_acc_y=(float)imu660ra_acc_y / 4096.f ;
    angle.my_acc_z=(float)imu660ra_acc_z / 4096.f ;
}

void angle_init(void)
{
    if(imu660ra_init())
    {
        tft180_show_string(0,80,"gyro_init_1");
        system_delay_ms(1000);
        printf("\r\n IMU660RA init error.");                                 // IMU660RA ��ʼ��ʧ��
//           break;
    }                         //�����ǳ�ʼ��
    angle.data_imu660ra_gyro_x=0;
    angle.data_imu660ra_gyro_y=0;
    angle.data_imu660ra_gyro_z=0;
    for(int i=0;i<600;i++)
    {
        imu660ra_get_gyro();
        angle.data_imu660ra_gyro_x+=(float)imu660ra_gyro_x;
        angle.data_imu660ra_gyro_y+=(float)imu660ra_gyro_y;
        angle.data_imu660ra_gyro_z+=(float)imu660ra_gyro_z;
        system_delay_ms(2);  //�ϵ���ʱ
    }
    angle.data_imu660ra_gyro_x=angle.data_imu660ra_gyro_x/600;
    angle.data_imu660ra_gyro_y=angle.data_imu660ra_gyro_y/600;
    angle.data_imu660ra_gyro_z=angle.data_imu660ra_gyro_z/600;
    if(  ((angle.data_imu660ra_gyro_x)>5) || ((angle.data_imu660ra_gyro_x)< -5)
            || ((angle.data_imu660ra_gyro_y)>5)  ||   ((angle.data_imu660ra_gyro_y)< -5) 
            ||  ((angle.data_imu660ra_gyro_z)>5)  ||  ((angle.data_imu660ra_gyro_z)<-5)  ) //�ɼ��Ѿ���Ч��������ʾ����
    {
        tft180_show_string(0,80,"gyro_bug_0");
        system_delay_ms(500);
    }
    else
    {
        tft180_show_string(0,80,  "gyro_init_success");
        system_delay_ms(500);
//           break;
    }
    angle.init_flag=1;
    angle.st_flag=1;
}

void angle_convert(void)
{

   if(angle.st_flag==0) //����ʼ�Ƕ�����
    {
      angle.st_flag=1;
      yaw_last=yaw;
    }
    if(angle.st_flag==1)
    {
        yaw = angle.yaw;
        if(yaw_last>0&&yaw<-90&&yaw>-180)       actual_angle+=(180-yaw_last)+(180+yaw);   //���ڽ�-180��180�Ľ�ת��
        else if(yaw_last>0&&yaw>-90&&yaw<0)     actual_angle+=-yaw_last+yaw;
        else if(yaw_last<0&&yaw>0&&yaw<90)      actual_angle+=yaw-yaw_last;
        else if(yaw_last<0&&yaw>90&&yaw<180)    actual_angle+=(-180-yaw_last)+(yaw-180);
        else actual_angle+=yaw-yaw_last;
        yaw_last=yaw;
        Yaw = actual_angle;
        if(angle.roll < 0)
        {
            Roll = (angle.roll + 360.f);//�����ǵķ�Χͨ����-180�ȵ�+180��
        }
        else
        {
            Roll = angle.roll;//
        }
        Pitch = angle.pitch;//�����ǵķ�Χͨ����-180�ȵ�+180��
    }
}//Pitch�������ǣ���Roll�������ǣ�,yaw��ƫ���ǣ�

void angle_get(void)
{
//    AHRSupdate(angle.my_gyro_x,angle.my_gyro_y,angle.my_gyro_z,
//                   angle.acc_xout ,angle.acc_yout ,angle.acc_zout,
//                   0 ,0 ,0);//��Ԫ������
//    angle_convert();
    
    static const float delta_t = 0.02f; //
    static const float alpha = 0.98; // ������Ȩ��
    static const float beta = 0.02;  // ���ٶȼ�Ȩ��
    
     angle.yaw+= angle.my_gyro_z * delta_t;
    number_times++;
    // ʹ�ü��ٶȼ���������/���
float acc_pitch = atan2f(angle.my_acc_y, angle.my_acc_z);  // Y-Z �� Pitch
float acc_roll  = atan2f(angle.my_acc_x, angle.my_acc_z);  // X-Z �� Roll  
angle.pitch = alpha * (angle.pitch + angle.my_gyro_y * delta_t) + beta * acc_pitch;
angle.roll  = alpha * (angle.roll + angle.my_gyro_x * delta_t) + beta * acc_roll;
//    angle.pitch+= angle.my_gyro_y * delta_t;
//    angle.roll+= angle.my_gyro_x * delta_t;
        // ======== ������ת��Ϊ�Ƕ� ========

    
    // ת��������ȫ�ֱ���
    Pitch = angle.pitch *180 / M_PI;
    Roll  = angle.roll *180 / M_PI;
    Yaw   = angle.yaw * 180 / M_PI;
    // ƫ���ǹ�һ����-180��~180��
    if(Yaw > 180) {
        Yaw -= 360;
    } else if(Yaw < -180) {
        Yaw += 360;
    }
}




void PID_angle(int16 err1) {   //�ǶȻ�

    static float err11,err111;

 //       if(err1>90)//run start, pwm max
 //           out_angle=PWM_DUTY_MAX;

            out_angle= angle_p*err1+angle_d*(err1-err11)+angle_i*(err1-2*err11+err111);
						//if(out_angle>5) out_angle=5;
						err111 = err11;
						err11 = err1;
}

void PID_gyro()//�ڻ�
{
    float error;
    static float last_error;
    error=10*(out_angle-angle.my_gyro_z);//�ڻ���ƫ�����⻷�����������z��Ľ��ٶ�֮��
	
    out_gyro=(int)(error*gyro_p+(error-last_error)*gyro_d);
	  // if(out_gyro>300)
		//	 out_gyro=300;
    last_error=error;
}


float l_dist_m  = 0.0f;   // [m] �����ۼ�·��
float r_dist_m = 0.0f;   // [m] �����ۼ�·��

void go_type()
{
const float k  = 0.053f;   // [rad/s / PWM] ʵ�⣺PWM ÿƫ�� 1 �� ���ٶȱ仯 k rad/s
const float r  = 0.027f;    // [m] ���Ӱ뾶
const float b  = 0.090f;    // [m] �־�
const float dt = 0.020f;    // [s] 20 ms ����
    
	uint16 Lstop=(L_FORWARD_THRE+L_REVERSE_THRE)/2;
	uint16 Rstop=(R_FORWARD_THRE+R_REVERSE_THRE)/2;
    if(stop0_go1==0)//
    {
    pwm_set_duty(TIM_G8_CH0_A26,Lstop);                // ���¶�Ӧͨ��ռ�ձ�
    pwm_set_duty(TIM_G8_CH1_A27, Rstop);                // ���¶�Ӧͨ��ռ�ձ�
    }
    else if(stop0_go1==1)//����ѭ����ת��
    {
        float omega_L = 0.0f, omega_R = 0.0f;   // [rad/s]
          if (out_gyro + base_speed >= 0)                 // ��ǰ
            {
                Lpwm_A26 = out_gyro + L_FORWARD_THRE + base_speed;
                omega_L  =  k * (Lpwm_A26 - L_FORWARD_THRE);   // ��ת
            }
            else                                            // ���
            {
                Lpwm_A26 = out_gyro + L_REVERSE_THRE + base_speed;
                omega_L  = -k * (L_REVERSE_THRE - Lpwm_A26);   // ��ת�����ţ�
            }

            /* ���֣����� out_gyro - base_speed �ķ��ž������� */
            if (out_gyro - base_speed >= 0)                 // ���ַ�ת��ע��������
            {
                Rpwm_A27 = out_gyro + R_REVERSE_THRE - base_speed;
                omega_R  = -k * (Rpwm_A27 - R_REVERSE_THRE);   // ��ת�����ţ�
            }
            else                                            // ������ת
            {
                Rpwm_A27 = out_gyro + R_FORWARD_THRE - base_speed;
                omega_R  =  k * (R_FORWARD_THRE - Rpwm_A27);   // ��ת
            }
			
        if(Lpwm_A26<=(Lstop-500 ) )
            Lpwm_A26=(Lstop-500 ) ;
        if(Lpwm_A26>(Lstop+500 ) )
            Lpwm_A26=(Lstop+500 ) ;
        if(Rpwm_A27<= (Rstop-500 ) )
            Rpwm_A27=(Rstop-500 );
        if(Rpwm_A27>(Rstop+500 ))
            Rpwm_A27=(Rstop+500 );
        
        pwm_set_duty(TIM_G8_CH0_A26,Lpwm_A26);                // ���¶�Ӧͨ��ռ�ձ�
        pwm_set_duty(TIM_G8_CH1_A27, Rpwm_A27);                // ���¶�Ӧͨ��ռ�ձ�
        
        /* 3-6 �ѽ��ٶ� �� ���ٶ� �� ����·�� ----------------------------- */
        float v_L = omega_L * r;    // [m/s]
        float v_R = omega_R * r;    // [m/s]
        l_dist_m  += v_L * dt;   // [m]
        r_dist_m += v_R * dt;   // [m]
        //if(l_dist_m>3||l_dist_m<-3)
           // stop0_go1=0;
    
    }
    else if(stop0_go1==2)//�ǶȻ�
    {
        
    Lpwm_A26=base_speed+ out_angle;
    Rpwm_A27=base_speed+ out_angle;  
        
    pwm_set_duty(TIM_G8_CH0_A26,Lpwm_A26);                // ���¶�Ӧͨ��ռ�ձ�
    pwm_set_duty(TIM_G8_CH1_A27, Rpwm_A27);                // ���¶�Ӧͨ��ռ�ձ�
    
    }
}
