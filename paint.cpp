//
// Created by 马浩程 on 2019/1/12.
//

#include <opencv2/opencv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <cmath>

using namespace cv;

const int yblank=10, redundant = 15,textheight=72,textwidth=60,xblank = 60,thickness=8;

Mat getImg(double num){
    Mat meter = Mat::zeros(textheight + yblank * 2 + redundant * 2, 8*(textwidth+xblank), CV_8UC3);
    for (int i = 5; i < 8; i++) {
        rectangle(meter,Point((textwidth + xblank)* i+xblank/4,0),Point((textwidth + xblank)* i+xblank/4*3+textwidth,textheight + yblank * 2 + redundant * 2),Scalar(0, 0, 255),FILLED);
    }
    if (num < 0)
        num = 0;
    int inte[8];
    float frac[8];
    for (int i = 7; i >= 0; i--) {
        inte[i] = ((int) num) % 10;
        frac[i] = num - (int) num;
        num /= 10;
    }
    for (int i = 6; i >= 0; i--) {
        if (inte[i + 1] == 9)
            frac[i] = frac[i + 1];
        else
            frac[i] = 0;
    }
    for (int i = 0; i < 8; i++) {
        char ch[] = "0";

        int move = frac[i] * (textheight+yblank) + 4;
        ch[0] = (inte[i] + 9) % 10 + '0';
        putText(meter, ch, Point((textwidth + xblank)* i+xblank/2, redundant - move), FONT_HERSHEY_SIMPLEX,3, Scalar(255, 255, 255),
                thickness, 16);
        ch[0] = (inte[i]) % 10 + '0';
        putText(meter, ch, Point((textwidth + xblank)* i+xblank/2, redundant + textheight + yblank - move ), FONT_HERSHEY_SIMPLEX, 3,
                Scalar(255, 255, 255), thickness, 16);
        ch[0] = (inte[i] + 1) % 10 + '0';
        putText(meter, ch, Point((textwidth + xblank)* i+xblank/2, redundant + textheight *2 + yblank * 2 - move ),
                FONT_HERSHEY_SIMPLEX, 3, Scalar(255, 255, 255), thickness, 16);
        ch[0] = (inte[i] + 2) % 10 + '0';
        putText(meter, ch, Point((textwidth + xblank)* i+xblank/2, redundant + textheight *3 + yblank * 3 - move ),
                FONT_HERSHEY_SIMPLEX, 3, Scalar(255, 255, 255), thickness, 16);
    }


    resize(meter, meter, cv::Size(), 0.3, 0.3, INTER_LINEAR );

    return meter;
}

int main(void) {
    double num = 0000123.56;

    while(1) {
        num += 0.03;
        imshow("meter", getImg(num));

        if(waitKey(1)>0)
            break;
    }
    return (0);

}
