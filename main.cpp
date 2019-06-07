#include <iostream>
#include <sys/fcntl.h>
#include <unistd.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <IOKit/serial/ioss.h>
#include <time.h>

const char* logo = " _    _ _ _                             _         _____             __  __      _            \n| |  | | | |                           (_)       / ____|           |  \\/  |    | |           \n| |  | | | |_ _ __ __ _ ___  ___  _ __  _  ___  | |  __  __ _ ___  | \\  / | ___| |_ ___ _ __ \n| |  | | | __| '__/ _` / __|/ _ \\| '_ \\| |/ __| | | |_ |/ _` / __| | |\\/| |/ _ \\ __/ _ \\ '__|\n| |__| | | |_| | | (_| \\__ \\ (_) | | | | | (__  | |__| | (_| \\__ \\ | |  | |  __/ ||  __/ |   \n \\____/|_|\\__|_|  \\__,_|___/\\___/|_| |_|_|\\___|  \\_____|\\__,_|___/ |_|  |_|\\___|\\__\\___|_|   \n ";


const float scale = 24.67;// 1.48 -- 60mL
#pragma pack (1)
struct {
    uint8_t head1;
    uint8_t head2;
    int32_t up1;
    int32_t dn1;
    int32_t up2;
    int32_t dn2;
    int32_t up3;
    int32_t dn3;
    int32_t up4;
    int32_t dn4;
    int32_t up5;
    int32_t dn5;
    int32_t up6;
    int32_t dn6;
    int32_t diff;

    int32_t zero_calib;

    int32_t calibration;

    uint8_t up_t1_t2;
    uint8_t up_t2_tidea;
    uint8_t dn_t1_t2;
    uint8_t dn_t2_tidea;
    uint8_t up_offset;
    uint8_t dn_offset;

    int64_t total;

    uint8_t end1;
    uint8_t end2;
}results = {0};

using namespace std;

void syncFrame(int fd) {
    unsigned char a = 0;
    while (a != 0xbb) {
        long l = read(fd, &a, 1);
        if (l != -1)
            printf("sync:read %ld char: %x \n", l, a);
    }
}




int main (int argc,char* argv[]) {
    const char *sp_path = (argc > 1) ? argv[1] : "/dev/tty.usbserial";

    int fd = open(sp_path, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        printf("open port failed");
        return fd;
    }

    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if ( tcgetattr ( fd, &tty ) != 0 )
    {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
    }
    cfsetospeed (&tty, B115200);
    cfsetispeed (&tty, B115200);
    tty.c_cflag     &=  ~PARENB;        // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~CRTSCTS;       // no flow control
    tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
    tty.c_oflag     =   0;                  // no remapping, no delays
    tty.c_cc[VMIN]      =   0;                  // read doesn't block
    tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
    tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    tty.c_oflag     &=  ~OPOST;              // make raw

    /* Flush Port, then applies attributes */
    tcflush( fd, TCIFLUSH );
    if ( tcsetattr ( fd, TCSANOW, &tty ) != 0)
    {
        cout << "Error " << errno << " from tcsetattr" << endl;
    }



    for (int i = 0;; i++) {

        int readCount = 0;
        while (1) {
            usleep(1000);
            long r = read(fd, ((unsigned char *) &results) + readCount, sizeof(results) - readCount);
            if (r > 0)
                readCount += r;
            if (readCount > sizeof(results)) {
                printf("overflow.\n");
                exit(666);
            }
            if (readCount == sizeof(results)) {
                if (results.head1 == 0x55 && results.head2 == 0xaa && results.end1 == 0x66 && results.end2 == 0xbb)
                    break;
                else {
                    printf("\ninvalid frame: read %ld Bytes:\n", sizeof(results));
                    for(int i=0;i< sizeof(results);i++)
                        printf("%02x ",((unsigned char *)&results)[i]);
                    printf("\n");
                    syncFrame(fd);
                    readCount = 0;
                }
            }
        }

        float up[] = {results.up1/262144.f,results.up2/262144.f,results.up3/262144.f,results.up4/262144.f,results.up5/262144.f,results.up6/262144.f};
        float dn[] = {results.dn1/262144.f,results.dn2/262144.f,results.dn3/262144.f,results.dn4/262144.f,results.dn5/262144.f,results.dn6/262144.f};
        float diff[] = {up[0]-dn[0],up[1]-dn[1],up[2]-dn[2],up[3]-dn[3],up[4]-dn[4],up[5]-dn[5]};
        system("clear");
        puts(logo);
        printf("up  : %+10.4f %+10.4f %+10.4f %+10.4f %+10.4f %+10.4f us\n",up[0],up[1],up[2],up[3],up[4],up[5]);
        printf("down: %+10.4f %+10.4f %+10.4f %+10.4f %+10.4f %+10.4f us\n",dn[0],dn[1],dn[2],dn[3],dn[4],dn[5]);
        printf("diff: %+10.4f %+10.4f %+10.4f %+10.4f %+10.4f %+10.4f us(uncalibrated, calculated on PC)\n",diff[0],diff[1],diff[2],diff[3],diff[4],diff[5]);
        printf("AVGDiff(zero calibrated):%10.4f  zeroCali:%10.4f  ",results.diff/262144.f,results.zero_calib/262144.f);
        printf("t1/t2:%5.3f %5.3f  t2/tidea:%5.3f %5.3f offset:%2X %2X  4Mcali:%f\n",results.up_t1_t2/128.f,results.dn_t1_t2/128.f,results.up_t2_tidea/128.f,results.dn_t2_tidea/128.f,results.up_offset,results.dn_offset,results.calibration/65536.f);
        printf("total: %f   => %f L\n",results.total/262144.f,results.total/262144.f/scale);

        time_t timer;
        char buffer[26];
        struct tm* tm_info;
        time(&timer);
        tm_info = localtime(&timer);
        strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
        puts(buffer);

        printf("\n%ld Bytes:", sizeof(results));
        for(int i=0;i< sizeof(results);i++)
            printf("%02x ",((unsigned char *)&results)[i]);
        printf("\n");






        fflush(stdout);
    }
}



//            system("clear");
//            printf("\033[0m[Heat Meter]\n");
//            printf("\033[1;35;40mID:0x%x 累计热量:%7.4f kWh   累计暖水量:%7.4f m^3   进水温度:%3.1f°C   回水温度:%3.1f°C\n",
//                       addr1, data1.accu_Heat, data1.accu_Flow,  data1.T_Display, data1.T_return_Display);

//
//            printf("\033[0m[Water Meter]\n");
//            printf("\033[1;32;40mID:0x%x 累计流量:%7.4f m^3   瞬时流量:%7.4f m^3/h\n",addr3,data3.accu_Flow,data3.inst_Flow);

//
////    转义序列以ESC(\033)开头.  \033[显示方式;前景色;背景色m
////    显示方式:0（默认值）、1（高亮）、22（非粗体）、4（下划线）、24（非下划线）、5（闪烁）、25（非闪烁）、7（反显）、27（非反显）
////    前景色:30（黑色）、31（红色）、32（绿色）、 33（黄色）、34（蓝色）、35（洋红）、36（青色）、37（白色）
////    背景色:40（黑色）、41（红色）、42（绿色）、 43（黄色）、44（蓝色）、45（洋红）、46（青色）、47（白色）
//
//}




