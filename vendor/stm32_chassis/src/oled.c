#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "delay.h"
#include "key.h"

extern u8 can_ser, abnormal;
extern u8 estop_pressed;
extern volatile u8 can_online;
extern volatile u8 Start_Flag, Start_Flag_one;
extern u8 Start_Flag_STOP;
extern volatile u8 flag_move, MOVE_mode;
extern float Encoder_A, Encoder_B, Encoder_C, Encoder_D;
extern volatile float Angle_current_A, Angle_current_B, Angle_current_C, Angle_current_D;
extern volatile int Voltage;
extern u16 FLAG_USART_ON, FLAG_CAN_ON;
extern volatile int Motor_A, Motor_B, Motor_C, Motor_D;
extern volatile float Yaw;
extern u8 OLED_GRAM[128][8];
u8 oled_boot_done = 0;
u8 debug_mode = 0;

static void cn12(u8 x, u8 y, u8 idx) {
    const u8 *p = cn12_font[idx];
    for(u8 i=0;i<12;i++) {
        u8 b1=p[i*2], b2=p[i*2+1];
        for(u8 j=0;j<8;j++) if(b1&(0x80>>j)) OLED_DrawPoint(x+j, y+i, 1);
        for(u8 j=0;j<4;j++) if(b2&(0x80>>j)) OLED_DrawPoint(x+8+j, y+i, 1);
    }
}
static void c2(u8 x,u8 y,u8 a,u8 b){cn12(x,y,a);cn12(x+12,y,b);}
static void hline(u8 y){for(u8 x=0;x<128;x++)OLED_DrawPoint(x,y,1);}

#define CN_DIAN  0
#define CN_YA    1
#define CN_MO    2
#define CN_SHI   3
#define CN_SU    4
#define CN_DU    5
#define CN_QIAN  6
#define CN_JIN   7
#define CN_HOU   8
#define CN_TUI   9
#define CN_ZUO   10
#define CN_YOU   11
#define CN_TING  12
#define CN_ZHI   13
#define CN_YUN   14
#define CN_XING  15
#define CN_ZHENG 16
#define CN_CHANG 17
#define CN_YI    18
#define CN_CHANG2 19
#define CN_QI    20
#define CN_DONG  21
#define CN_DIAO  22
#define CN_SHI2  23
#define CN_BIAN  24
#define CN_MA    25
#define CN_QI2   26
#define CN_DUO   27
#define CN_JIAO  28
#define CN_MU    29
#define CN_BIAO  30
#define CN_DANG  31
#define CN_QIAN2 32
#define CN_ZHUAN 33
#define CN_DAI   34
#define CN_JI    35
#define CN_XIE   36
#define CN_ZHI2  37
#define CN_XING2 38
#define CN_WAN   39
#define CN_YI2   40
#define CN_TAI   41
#define CN_FAN   42
#define CN_KUI   43
#define CN_DU2   44
#define CN_JIN2  45
#define CN_JI2   46
#define CN_DUAN  47
#define CN_KAI   48
#define CN_WEI   49
#define CN_JIE   50
#define CN_CHI   51
#define CN_SUO   52
#define CN_JEI   53

static void show_angle(u8 x, u8 y, char label, float val) {
    OLED_ShowChar(x, y, label, 12, 1);
    OLED_ShowChar(x+8, y, ':', 12, 1);
    if(val < 0) { OLED_ShowChar(x+14, y, '-', 12, 1); val = -val; }
    else        { OLED_ShowChar(x+14, y, '+', 12, 1); }
    int v = (int)val;
    if(v >= 100)      OLED_ShowChar(x+20, y, '0'+v/100, 12, 1);
    if(v >= 10)       OLED_ShowChar(x+26, y, '0'+(v/10)%10, 12, 1);
    OLED_ShowChar(x+32, y, '0'+v%10, 12, 1);
    OLED_ShowChar(x+38, y, 'o', 12, 1);
}

//启动画面
static void oled_boot_show(void) {
    c2(40, 24, CN_QI, CN_DONG);
    OLED_ShowString(64, 24, "...");
    OLED_Refresh_Gram();
}

//调试页面(双击进入)
static void oled_debug_show(void) {
    float ea=Encoder_A, eb=Encoder_B, ec=Encoder_C, ed=Encoder_D;
    int ma=Motor_A, mb=Motor_B, mc=Motor_C, md=Motor_D;
    float v = (float)Voltage / 100.0f;
    float yw = Yaw;
    for(u8 i=0;i<8;i++) for(u8 n=0;n<128;n++) OLED_GRAM[n][i]=0x00;

    //编码器限幅滤波
    {   static float dea,deb,dec,ded;
        float d;
        d=ea-dea;if(d<0)d=-d;if(d>15)dea=ea;
        d=eb-deb;if(d<0)d=-d;if(d>15)deb=eb;
        d=ec-dec;if(d<0)d=-d;if(d>15)dec=ec;
        d=ed-ded;if(d<0)d=-d;if(d>15)ded=ed;
        ea=dea; eb=deb; ec=dec; ed=ded;
    }

    //第1行: 电压+偏航角
    OLED_ShowChar(0,0,'V',12,1);
    OLED_ShowChar(6,0,':',12,1);
    OLED_ShowChar(12,0,'0'+(int)v/10,12,1);
    OLED_ShowChar(18,0,'0'+(int)v%10,12,1);
    OLED_ShowChar(24,0,'.',12,1);
    OLED_ShowChar(30,0,'0'+((int)(v*10))%10,12,1);
    OLED_ShowChar(36,0,'V',12,1);
    {   float ay=(yw>0?yw:-yw); int iy=(int)ay;
        u8 nd=(iy>=100)?3:(iy>=10)?2:1;
        u8 tw=18+nd*6+12+6;
        u8 sx=128-tw;
        OLED_ShowChar(sx,0,'Y',12,1); sx+=6;
        OLED_ShowChar(sx,0,':',12,1); sx+=6;
        OLED_ShowChar(sx,0,yw<0?'-':'+',12,1); sx+=6;
        OLED_ShowNumber(sx,0,iy,nd,12); sx+=nd*6;
        OLED_ShowChar(sx,0,'.',12,1); sx+=6;
        OLED_ShowChar(sx,0,'0'+((int)(ay*10))%10,12,1); sx+=6;
        OLED_ShowChar(sx,0,'o',12,1);
    }

    hline(11);

    //第2行: 编码器A+B
    {   int va=(int)(ea>0?ea:-ea); u8 sx=0;
        OLED_ShowChar(sx,12,'A',12,1); sx+=6;
        OLED_ShowChar(sx,12,':',12,1); sx+=6;
        OLED_ShowChar(sx,12,ea<0?'-':'+',12,1); sx+=6;
        OLED_ShowNumber(sx,12,va,5,12); sx+=40;
        int vb=(int)(eb>0?eb:-eb);
        OLED_ShowChar(sx,12,'B',12,1); sx+=6;
        OLED_ShowChar(sx,12,':',12,1); sx+=6;
        OLED_ShowChar(sx,12,eb<0?'-':'+',12,1); sx+=6;
        OLED_ShowNumber(sx,12,vb,5,12);
    }

    //第3行: 编码器C+D
    {   int vc=(int)(ec>0?ec:-ec); u8 sx=0;
        OLED_ShowChar(sx,24,'C',12,1); sx+=6;
        OLED_ShowChar(sx,24,':',12,1); sx+=6;
        OLED_ShowChar(sx,24,ec<0?'-':'+',12,1); sx+=6;
        OLED_ShowNumber(sx,24,vc,5,12); sx+=40;
        int vd=(int)(ed>0?ed:-ed);
        OLED_ShowChar(sx,24,'D',12,1); sx+=6;
        OLED_ShowChar(sx,24,':',12,1); sx+=6;
        OLED_ShowChar(sx,24,ed<0?'-':'+',12,1); sx+=6;
        OLED_ShowNumber(sx,24,vd,5,12);
    }

    hline(35);

    //第4行: PWM
    OLED_ShowString(0,36,"PW");
    OLED_ShowNumber(18,36,ma,4,12);
    OLED_ShowNumber(46,36,mb,4,12);
    OLED_ShowNumber(74,36,mc,4,12);
    OLED_ShowNumber(102,36,md,4,12);

    //第5行: CAN在线状态
    OLED_ShowString(0,48,"CAN");
    {   u8 sx=24;
        for(u8 i=0;i<4;i++){
            OLED_ShowChar(sx,48,(can_online&(1<<i))?'1'+i:'-',12,1); sx+=26;
        }
    }

    OLED_Refresh_Gram();
}

void oled_show(void)
{
    if(!oled_boot_done) { oled_boot_show(); return; }
    if(debug_mode) { oled_debug_show(); return; }
    for(u8 i=0;i<8;i++) for(u8 n=0;n<128;n++) OLED_GRAM[n][i]=0x00;
    float ca=Angle_current_A, cb=Angle_current_B, cc=Angle_current_C, cd=Angle_current_D;
    int vbat = Voltage;
    float v = (float)vbat / 100.0f;
    float to_ms = WHEEL_CIRCUMFERENCE_M/(ENCODER_CPR*MOTOR_GEAR_RATIO)*CONTROL_FREQ_HZ;
    //编码器限幅滤波: 跳变超过阈值时丢弃
    float ea,eb,ec,ed;
    {   static float dea,deb,dec,ded; float d;
        d=Encoder_A-dea;if(d<0)d=-d;if(d<800)dea=Encoder_A;
        d=Encoder_B-deb;if(d<0)d=-d;if(d<800)deb=Encoder_B;
        d=Encoder_C-dec;if(d<0)d=-d;if(d<800)dec=Encoder_C;
        d=Encoder_D-ded;if(d<0)d=-d;if(d<800)ded=Encoder_D;
        ea=dea; eb=deb; ec=dec; ed=ded; }
    float sa=ea*to_ms,sb=eb*to_ms,sc=ec*to_ms,sd=ed*to_ms;
    if(sa<0)sa=-sa; if(sb<0)sb=-sb; if(sc<0)sc=-sc; if(sd<0)sd=-sd;
    //速度低通滤波
    static float spd_f = 0.0f;
    float spd_raw = (sa+sb+sc+sd)/4.0f;
    spd_f = spd_f * 0.7f + spd_raw * 0.3f;
    float spd = spd_f;

    //第1行: 电压+状态
    OLED_ShowChar(0,0, 'V', 12, 1);
    OLED_ShowChar(8,0, ':', 12, 1);
    OLED_ShowChar(14,0, '0'+(int)v/10, 12, 1);
    OLED_ShowChar(20,0, '0'+(int)v%10, 12, 1);
    OLED_ShowChar(26,0, '.', 12, 1);
    OLED_ShowChar(32,0, '0'+((int)(v*10))%10, 12, 1);

    //异常标签
    static const u8 err_label[][5] = {
        {0,     CN_FAN,CN_KUI,CN_YI,CN_CHANG},  //反馈异常
        {0,     CN_DU2,CN_ZHUAN,0xFF,0xFF},       //堵转
        {0,     CN_WEI,CN_JIE,CN_DIAN,CN_CHI},    //未接电池
        {0,     CN_BIAN,CN_MA,CN_YI,CN_CHANG},    //编码异常
        {0,     CN_JI2,CN_TING,0xFF,0xFF},         //急停
        {0,     CN_DUAN,CN_KAI,0xFF,0xFF},         //CAN断开
        {'1',CN_BIAN,CN_MA,CN_YI,CN_CHANG},   //A编码异常
        {'2',CN_BIAN,CN_MA,CN_YI,CN_CHANG},   //B编码异常
        {'3',CN_BIAN,CN_MA,CN_YI,CN_CHANG},   //C编码异常
        {'4',CN_BIAN,CN_MA,CN_YI,CN_CHANG},   //D编码异常
    };
    static u8 err_idx = 0;
    static u16 err_timer = 0;
    extern u8 enc_fault;

    if(abnormal || Voltage < 700 || estop_pressed) {
        u8 active[12]; u8 n = 0;
        if(abnormal & 1)  active[n++] = 0;  //反馈异常
        if(abnormal & 2)  active[n++] = 1;  //堵转
        if(Voltage < 700) active[n++] = 2;   //未接电池
        if(abnormal & 8) {
            if(enc_fault & 1) active[n++] = 6; //A编码异常
            if(enc_fault & 2) active[n++] = 7; //B编码异常
            if(enc_fault & 4) active[n++] = 8; //C编码异常
            if(enc_fault & 8) active[n++] = 9; //D编码异常
            if(!enc_fault)    active[n++] = 3; //编码异常(通用)
        }
        if(abnormal & 0x10) active[n++] = 5; //CAN断开
        if(estop_pressed) active[n++] = 4;    //急停

        if(n > 0) {
            if(++err_timer > 75) { err_timer = 0; err_idx = (err_idx + 1) % n; }
            if(err_idx >= n) err_idx = 0;

            const u8 *e = err_label[active[err_idx]];
            u8 pf = e[0];
            u8 cnt = 0; for(u8 i=1;i<5;i++) if(e[i]!=0xFF) cnt++;
            u8 tw = (active[err_idx]==5 ? 24 : (pf ? 6 : 0)) + cnt * 12;
            u8 x = 128 - tw;

            for(u8 yy=0; yy<12; yy++)
                for(u8 xx=40; xx<128; xx++) OLED_DrawPoint(xx, yy, 0);

            if(active[err_idx]==5){OLED_ShowString(x,0,"CAN");x+=24;}
            else if(pf) { OLED_ShowChar(x, 0, pf, 12, 1); x += 6; }
            for(u8 i=1; i<5; i++) {
                if(e[i]!=0xFF) { cn12(x,0,e[i]); x += 12; }
            }

            for(u8 i=0; i<n; i++)
                OLED_DrawPoint(46+i*6, 10, i==err_idx ? 1 : 0);
        }
    } else {
        err_timer = 0; err_idx = 0;
        //运行状态
        if(!Start_Flag)      {c2(104,0,CN_DAI,CN_JI);}
        else if(flag_move==0){c2(104,0,CN_JEI,CN_SUO);}
        else if(flag_move==1){c2(104,0,CN_QIAN,CN_JIN);}
        else if(flag_move==2){c2(104,0,CN_HOU,CN_TUI);}
        else if(flag_move==3){c2(104,0,CN_ZUO,CN_YI2);}
        else if(flag_move==4){c2(104,0,CN_YOU,CN_YI2);}
        else if(flag_move<=8){c2(104,0,CN_XIE,CN_XING2);}
        else                 {c2(104,0,CN_ZHUAN,CN_WAN);}
    }

    hline(11);

    //第2-3行: 舵角
    show_angle(0,  14, 'A', ca);
    OLED_ShowChar(60,14,'|',12,1);
    show_angle(70, 14, 'B', cb);
    show_angle(0,  28, 'C', cc);
    OLED_ShowChar(60,28,'|',12,1);
    show_angle(70, 28, 'D', cd);

    hline(40);

    //第4行: 速度
    {
        int n = (int)spd;
        int d = (int)(spd * 10.0f) % 10;
        u8 nd = (n >= 100) ? 3 : (n >= 10) ? 2 : 1;
        u8 tw = 24 + (nd + 2) * 6 + 18;
        u8 sx = (128 - tw) / 2;
        c2(sx, 42, CN_SU, CN_DU);
        sx += 24;
        if(n >= 100) { OLED_ShowChar(sx, 42, '0'+n/100, 12, 1); sx += 6; }
        if(n >= 10)  { OLED_ShowChar(sx, 42, '0'+(n/10)%10, 12, 1); sx += 6; }
        OLED_ShowChar(sx, 42, '0'+n%10, 12, 1); sx += 6;
        OLED_ShowChar(sx, 42, '.', 12, 1); sx += 6;
        OLED_ShowChar(sx, 42, '0'+d, 12, 1); sx += 6;
        OLED_ShowString(sx, 42, "m/s");
    }

    //第5行: 模式+舵机在线
    OLED_ShowChar(0, 54, '[', 12, 1);
    OLED_ShowChar(6, 54, MOVE_mode ? 'T' : 'A', 12, 1);
    OLED_ShowChar(12, 54, ']', 12, 1);
    {
        u8 sx = 86;
        for(u8 i = 0; i < 4; i++) {
            OLED_ShowChar(sx, 54, (can_online & (1<<i)) ? '1'+i : '-', 12, 1);
            sx += 6;
            if(i < 3) { OLED_ShowChar(sx, 54, ' ', 12, 1); sx += 6; }
        }
    }

    OLED_Refresh_Gram();
}

void oled_show_error(void)
{
    OLED_ShowString(10,10,"Abnormal");
    OLED_ShowString(10,30,"PowerOff");
    OLED_ShowString(10,46,"Restart");
    OLED_Refresh_Gram();
}

u8 OLED_GRAM[128][8];
void OLED_Refresh_Gram(void){
    u8 i,n;
    for(i=0;i<8;i++){OLED_WR_Byte(0xb0+i,OLED_CMD);OLED_WR_Byte(0x00,OLED_CMD);OLED_WR_Byte(0x10,OLED_CMD);
    for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);}
}
void OLED_WR_Byte(u8 dat,u8 cmd){
    u8 i;
    if(cmd)OLED_RS_Set();else OLED_RS_Clr();
    for(i=0;i<8;i++){OLED_SCLK_Clr();if(dat&0x80)OLED_SDIN_Set();else OLED_SDIN_Clr();OLED_SCLK_Set();dat<<=1;}
    OLED_RS_Set();
}
void OLED_Display_On(void){OLED_WR_Byte(0X8D,OLED_CMD);OLED_WR_Byte(0X14,OLED_CMD);OLED_WR_Byte(0XAF,OLED_CMD);}
void OLED_Display_Off(void){OLED_WR_Byte(0X8D,OLED_CMD);OLED_WR_Byte(0X10,OLED_CMD);OLED_WR_Byte(0XAE,OLED_CMD);}
void OLED_Clear(void){u8 i,n;for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;OLED_Refresh_Gram();}
void OLED_DrawPoint(u8 x,u8 y,u8 t){
    u8 pos,bx,temp=0;
    if(x>127||y>63)return;
    pos=7-y/8;bx=y%8;temp=1<<(7-bx);
    if(t)OLED_GRAM[x][pos]|=temp;else OLED_GRAM[x][pos]&=~temp;
}
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode){
    u8 temp,t,t1;u8 y0=y;
    chr-=' ';
    for(t=0;t<size;t++){
        if(size==12)temp=oled_asc2_1206[chr][t];else temp=oled_asc2_1608[chr][t];
        for(t1=0;t1<8;t1++){
            if(temp&0x80)OLED_DrawPoint(x,y,mode);else OLED_DrawPoint(x,y,!mode);
            temp<<=1;y++;
            if((y-y0)==size){y=y0;x++;break;}
        }
    }
}
u32 oled_pow(u8 m,u8 n){u32 r=1;while(n--)r*=m;return r;}
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size){
    u8 t,temp,enshow=0;
    for(t=0;t<len;t++){
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1)){if(temp==0){OLED_ShowChar(x+(size/2)*t,y,' ',size,1);continue;}else enshow=1;}
        OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1);
    }
}
void OLED_ShowString(u8 x,u8 y,const u8 *p){
    while(*p!='\0'){if(x>122){x=0;y+=16;}if(y>58){y=x=0;OLED_Clear();}OLED_ShowChar(x,y,*p,12,1);x+=8;p++;}
}
void OLED_Init(void){
    RCC->APB2ENR|=1<<4;RCC->APB2ENR|=1<<0;
    GPIOC->CRH&=0X0000FFFF;GPIOC->CRH|=0X33330000;
    RCC->APB1ENR|=1<<28;PWR->CR|=1<<8;
    RCC->BDCR&=0xFFFFFFFE;BKP->CR&=0xFFFFFFFE;PWR->CR&=0xFFFFFEFF;
    OLED_RST_Clr();delay_ms(100);OLED_RST_Set();
    OLED_WR_Byte(0xAE,OLED_CMD);OLED_WR_Byte(0xD5,OLED_CMD);OLED_WR_Byte(80,OLED_CMD);
    OLED_WR_Byte(0xA8,OLED_CMD);OLED_WR_Byte(0X3F,OLED_CMD);
    OLED_WR_Byte(0xD3,OLED_CMD);OLED_WR_Byte(0X00,OLED_CMD);
    OLED_WR_Byte(0x40,OLED_CMD);OLED_WR_Byte(0x8D,OLED_CMD);OLED_WR_Byte(0x14,OLED_CMD);
    OLED_WR_Byte(0x20,OLED_CMD);OLED_WR_Byte(0x02,OLED_CMD);
    OLED_WR_Byte(0xA1,OLED_CMD);OLED_WR_Byte(0xC0,OLED_CMD);
    OLED_WR_Byte(0xDA,OLED_CMD);OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0x81,OLED_CMD);OLED_WR_Byte(0xEF,OLED_CMD);
    OLED_WR_Byte(0xD9,OLED_CMD);OLED_WR_Byte(0xf1,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);OLED_WR_Byte(0x30,OLED_CMD);
    OLED_WR_Byte(0xA4,OLED_CMD);OLED_WR_Byte(0xA6,OLED_CMD);OLED_WR_Byte(0xAF,OLED_CMD);
    OLED_Clear();
}
